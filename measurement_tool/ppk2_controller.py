from __future__ import annotations

import csv
import logging
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger("ppk2_controller")

# Documented dependency:
# Package: ppk2-api
# Version pinned: 0.9.2 (PyPI: https://pypi.org/project/ppk2-api/ / IRNAS: https://github.com/IRNAS/ppk2-api-python)
try:
    from ppk2_api.ppk2_api import PPK2_API, PPK2_MP
    PPK2_API_AVAILABLE = True
except ImportError:
    PPK2_API_AVAILABLE = False
    PPK2_API = None  # type: ignore
    PPK2_MP = None  # type: ignore

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None  # type: ignore

from .energy import energy_mj_from_mw_s


@dataclass
class PPK2PhaseWindow:
    name: str
    start_ns: int
    end_ns: int = 0
    start_sample_idx: int = 0
    end_sample_idx: int = 0
    samples: List[float] = field(default_factory=list)  # in microamps (uA)
    min_current_ma: float = 0.0
    avg_current_ma: float = 0.0
    max_current_ma: float = 0.0
    avg_power_mw: float = 0.0
    total_energy_mj: float = 0.0
    duration_s: float = 0.0
    timing_verified: bool = True
    timing_notes: str = ""


class MockPPK2Device:
    """Deterministic in-memory mock of Nordic PPK2 for offline unit and regression testing."""

    def __init__(self, port: str = "MOCK_PPK2") -> None:
        self.port = port
        self.is_connected = True
        self.mode = "NONE"
        self.voltage_mv = 5000
        self.power_state = "OFF"
        self.is_measuring = False
        self._samples_queue: List[float] = []
        self._lock = threading.Lock()
        self._default_current_ua = 15000.0  # 15 mA default

    def get_modifiers(self) -> Dict[str, Any]:
        return {"calibration": "VALID", "model": "PPK2", "fw_version": "1.0.0"}

    def use_source_meter(self) -> None:
        self.mode = "SOURCE_METER"

    def set_source_voltage(self, mv: int) -> None:
        self.voltage_mv = mv

    def toggle_DUT_power(self, state: str) -> None:
        self.power_state = state.upper()

    def start_measuring(self) -> None:
        self.is_measuring = True

    def stop_measuring(self) -> None:
        self.is_measuring = False

    def set_phase_current_ua(self, current_ua: float) -> None:
        with self._lock:
            self._default_current_ua = current_ua

    def get_data(self) -> bytes:
        if not self.is_measuring or self.power_state != "ON":
            return b""
        # Generate dummy 4-byte sample packet
        time.sleep(0.01)
        return b"\x00\x00\x00\x00" * 10

    def get_samples(self, read_data: bytes) -> Tuple[List[float], List[int]]:
        if not read_data:
            return [], []
        count = len(read_data) // 4
        with self._lock:
            val = self._default_current_ua
        return [val] * count, [0] * count


class PPK2Controller:
    """Dedicated programmable controller and telemetry harvester for Nordic PPK2.

    Hardware Requirements (NUCLEO-L476RG via ST UM1724):
    - PPK2 VOUT connected to NUCLEO E5V (CN7 pin 6 or CN6 pin 3).
    - PPK2 GND connected to NUCLEO GND (CN7 pin 20 or CN6 pin 4).
    - Jumper JP5 set to pins 2-3 (E5V power source selection).
    - Jumper JP1 REMOVED (prevents back-powering ST-Link from E5V).
    - Supply voltage: 5000 mV (5.0 V) regulated by on-board LD1117S33 LDO to 3.3 V VDD.
    - NEVER connect 5 V E5V directly to the MCU's 3.3 V rail.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        voltage_mv: int = 5000,
        mock: bool = False,
        progress_callback: Optional[Any] = None,
    ) -> None:
        self.port = port
        self.voltage_mv = voltage_mv
        self.is_mock = mock or (port is not None and port.upper() in {"MOCK", "SIM", "MOCK_PPK2"})
        self.progress = progress_callback or (lambda msg: None)

        self._device: Any = None
        self._sampling_active = False
        self._sampling_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        # Telemetry storage
        self.sample_rate_hz = 100000  # 100 kS/s fixed PPK2 hardware rate
        self.sample_period_s = 1.0 / self.sample_rate_hz
        self.sampling_start_ns: int = 0
        self.power_on_host_ns: int = 0
        self.power_off_host_ns: int = 0
        self.power_state = "OFF"
        self.is_storage_busy = False

        self.all_currents_ua: List[float] = []
        self.all_timestamps_s: List[float] = []
        self.phases: Dict[str, PPK2PhaseWindow] = {}
        self.active_phase: Optional[str] = None

    @staticmethod
    def list_ppk2_ports() -> List[str]:
        """Discovers serial ports associated with Nordic PPK2."""
        found_ports: List[str] = []
        if PPK2_API_AVAILABLE and PPK2_API is not None:
            try:
                api_ports = PPK2_API.list_devices()
                if api_ports:
                    return api_ports
            except Exception:
                pass

        if list_ports is not None:
            for p in list_ports.comports():
                desc = (p.description or "").lower()
                hwid = (p.hwid or "").lower()
                # Nordic Semiconductor VID: 0x1915, PPK2 PID: 0xC00A
                if "1915:c00a" in hwid or "ppk2" in desc or "power profiler" in desc:
                    found_ports.append(p.device)
        return found_ports

    def connect(self, port: Optional[str] = None, exclude_uart_port: Optional[str] = None) -> None:
        """Connects to PPK2 over USB serial, verifying device communication."""
        selected_port = port or self.port

        if self.is_mock:
            self.progress(f"[PPK2] Connecting to simulated MockPPK2Device on '{selected_port or 'MOCK'}'...")
            self._device = MockPPK2Device(port=selected_port or "MOCK_PPK2")
            self.port = selected_port or "MOCK_PPK2"
            self.progress("[PPK2] Mock connection verified (Source Meter Ready).")
            return

        if not PPK2_API_AVAILABLE:
            raise RuntimeError(
                "ppk2-api is not installed. Install via 'pip install ppk2-api==0.9.2' to enable hardware PPK2 control."
            )

        if not selected_port or selected_port.upper() == "AUTO":
            ports = self.list_ppk2_ports()
            if exclude_uart_port:
                ports = [p for p in ports if p.upper() != exclude_uart_port.upper()]
            if not ports:
                raise RuntimeError(
                    f"No Nordic PPK2 devices detected over USB serial (excluded UART: {exclude_uart_port}). "
                    "Ensure PPK2 USB is connected and Nordic Power Profiler desktop app is closed."
                )
            selected_port = ports[0]

        if exclude_uart_port and selected_port.upper() == exclude_uart_port.upper():
            raise ValueError(
                f"PPK2 port '{selected_port}' collides with STM32 UART port '{exclude_uart_port}'! "
                "PPK2 and STM32 must use two independent serial ports."
            )

        self.port = selected_port
        self.progress(f"[PPK2] Connecting to Nordic PPK2 on '{self.port}'...")

        try:
            # This controller owns the sampling thread, so the direct API is the
            # appropriate transport. PPK2_MP adds a second fetch thread and has
            # an unsafe destructor when its serial port is closed explicitly.
            self._device = PPK2_API(self.port)
            # A previous process may have left the PPK2 streaming binary samples.
            # The 0.9.2 API's metadata reader decodes every byte as UTF-8, so stop
            # any stale stream and discard buffered samples before requesting text.
            serial_port = getattr(self._device, "ser", None)
            if serial_port is not None:
                try:
                    serial_port.reset_input_buffer()
                except Exception:
                    pass
            self._device.stop_measuring()
            time.sleep(0.1)
            if serial_port is not None:
                try:
                    serial_port.reset_input_buffer()
                except Exception:
                    pass

            def read_metadata_safely() -> Optional[str]:
                from ppk2_api.ppk2_api import PPK2_Command

                self._device._write_serial((PPK2_Command.GET_META_DATA,))
                received = bytearray()
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    available = serial_port.in_waiting if serial_port is not None else 0
                    if available:
                        received.extend(serial_port.read(available))
                        if b"END\n" in received or received.endswith(b"END"):
                            break
                    else:
                        time.sleep(0.01)
                lines = []
                for raw_line in bytes(received).splitlines():
                    line = raw_line.decode("ascii", errors="ignore").strip()
                    if line == "END" or re.fullmatch(r"[A-Za-z0-9]+:\s*[-+]?[0-9]+(?:\.[0-9]+)?", line):
                        lines.append(line)
                return "\n".join(lines) if "END" in lines else None

            if serial_port is None:
                raise RuntimeError("PPK2 serial transport is unavailable")
            self._device._read_metadata = read_metadata_safely
            modifiers = self._device.get_modifiers()
            if not modifiers or any(self._device.modifiers.get(name, {}).get("0") is None
                                    for name in ("R", "GS", "GI", "O", "S", "I", "UG")):
                raise RuntimeError("Failed to retrieve PPK2 calibration modifiers.")
            self.progress(f"[PPK2] Hardware connected successfully on '{self.port}'. Modifiers verified.")
        except Exception as ex:
            # PPK2_API has no public close method; release pySerial explicitly so
            # retries and subsequent runs do not inherit a stale COM handle.
            device = self._device
            serial_port = getattr(device, "ser", None)
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
            self._device = None
            raise RuntimeError(
                f"Failed to connect to PPK2 on port '{self.port}': {ex}. "
                "Ensure no other software (e.g. nRF Connect Power Profiler) has opened this port."
            ) from ex

    def configure_source_meter(self, voltage_mv: Optional[int] = None) -> None:
        """Configures PPK2 in Source Meter mode with specified output voltage."""
        if voltage_mv is not None:
            self.voltage_mv = voltage_mv

        if not (800 <= self.voltage_mv <= 5000):
            raise ValueError(
                f"Invalid PPK2 voltage: {self.voltage_mv} mV. Valid Source Meter range is 800 mV to 5000 mV."
            )

        self.progress(f"[PPK2] Configuring Source Meter mode @ {self.voltage_mv} mV (E5V supply)...")
        if self._device is None:
            raise RuntimeError("PPK2 not connected.")

        self._device.use_source_meter()
        self._device.set_source_voltage(self.voltage_mv)
        self.progress(f"[PPK2] Source Meter mode active: {self.voltage_mv} mV configured.")

    def power_off(self, force: bool = False) -> None:
        """Disables DUT power programmatically."""
        if self.is_storage_busy and not force:
            raise RuntimeError(
                "Refusing power removal: Storage or active recording is ongoing! "
                "Data loss or filesystem corruption prevented."
            )

        if self._device:
            self._device.toggle_DUT_power("OFF")
        self.power_state = "OFF"
        self.power_off_host_ns = time.monotonic_ns()
        self.progress("[PPK2] DUT power output set to OFF.")

    def power_on(self) -> int:
        """Enables DUT power programmatically and returns the host monotonic nanosecond timestamp."""
        if self._device is None:
            raise RuntimeError("PPK2 not connected.")

        self.power_state = "ON"
        if self._device:
            self._device.toggle_DUT_power("ON")
        self.power_on_host_ns = time.monotonic_ns()
        self.progress(f"[PPK2] DUT power output set to ON ({self.voltage_mv} mV). Host t_on recorded.")
        return self.power_on_host_ns

    def start_sampling(self) -> int:
        """Starts continuous power sampling in background thread."""
        if self._sampling_active:
            return self.sampling_start_ns

        self.all_currents_ua.clear()
        self.all_timestamps_s.clear()
        self.phases.clear()
        self.active_phase = None

        if self._device is None:
            raise RuntimeError("PPK2 not connected.")

        self._stop_event.clear()
        self._sampling_active = True
        self.sampling_start_ns = time.monotonic_ns()

        self._device.start_measuring()
        self._sampling_thread = threading.Thread(target=self._sampling_worker, daemon=True, name="PPK2SamplingWorker")
        self._sampling_thread.start()

        self.progress("[PPK2] Continuous power sampling started @ 100 kS/s.")
        return self.sampling_start_ns

    def _sampling_worker(self) -> None:
        while not self._stop_event.is_set():
            try:
                raw_bytes = self._device.get_data()
                if raw_bytes:
                    samples, _ = self._device.get_samples(raw_bytes)
                    if samples:
                        with self._lock:
                            base_idx = len(self.all_currents_ua)
                            for i, samp in enumerate(samples):
                                t_s = (base_idx + i) * self.sample_period_s
                                self.all_currents_ua.append(float(samp))
                                self.all_timestamps_s.append(t_s)
                else:
                    time.sleep(0.005)
            except Exception as ex:
                logger.debug("PPK2 worker error: %s", ex)
                time.sleep(0.01)

    def mark_phase_start(self, phase_name: str) -> int:
        """Marks the start of a logical test phase on the host timeline."""
        t_now_ns = time.monotonic_ns()
        with self._lock:
            cur_idx = len(self.all_currents_ua)
            self.phases[phase_name] = PPK2PhaseWindow(
                name=phase_name,
                start_ns=t_now_ns,
                start_sample_idx=cur_idx,
            )
            self.active_phase = phase_name
        self.progress(f"[PPK2] Phase '{phase_name}' started at sample index {cur_idx}.")
        return t_now_ns

    def mark_phase_end(self, phase_name: str, expected_duration_s: Optional[float] = None) -> int:
        """Marks the end of a logical test phase and computes its slice statistics."""
        t_now_ns = time.monotonic_ns()
        with self._lock:
            phase = self.phases.get(phase_name)
            if not phase:
                cur_idx = len(self.all_currents_ua)
                phase = PPK2PhaseWindow(name=phase_name, start_ns=t_now_ns, start_sample_idx=cur_idx)
                self.phases[phase_name] = phase

            cur_idx = len(self.all_currents_ua)
            phase.end_ns = t_now_ns
            phase.end_sample_idx = cur_idx
            phase.duration_s = max(0.0, (t_now_ns - phase.start_ns) / 1e9)

            # Extract sample slice
            start_i = max(0, phase.start_sample_idx)
            end_i = min(len(self.all_currents_ua), max(start_i, phase.end_sample_idx))
            sample_slice = self.all_currents_ua[start_i:end_i]
            phase.samples = sample_slice

            if sample_slice:
                u_arr = sample_slice
                phase.min_current_ma = min(u_arr) / 1000.0
                phase.max_current_ma = max(u_arr) / 1000.0
                phase.avg_current_ma = (sum(u_arr) / len(u_arr)) / 1000.0
                voltage_v = self.voltage_mv / 1000.0
                phase.avg_power_mw = phase.avg_current_ma * voltage_v
                phase.total_energy_mj = energy_mj_from_mw_s(phase.avg_power_mw, phase.duration_s)
            else:
                phase.min_current_ma = 0.0
                phase.max_current_ma = 0.0
                phase.avg_current_ma = 0.0
                phase.avg_power_mw = 0.0
                phase.total_energy_mj = 0.0

            # Verify phase timing against expected duration and sample count
            timing_issues = []
            exp_sample_count = int(phase.duration_s * 100000)
            if phase.duration_s > 0.1:
                sample_diff = abs(len(sample_slice) - exp_sample_count)
                if sample_diff > max(2000, int(0.15 * exp_sample_count)):
                    timing_issues.append(f"sample count {len(sample_slice)} vs expected {exp_sample_count}")

            if expected_duration_s is not None and expected_duration_s > 0:
                if abs(phase.duration_s - expected_duration_s) > 0.5:
                    timing_issues.append(f"duration {phase.duration_s:.2f}s deviates from expected {expected_duration_s:.1f}s")

            if timing_issues:
                phase.timing_verified = False
                phase.timing_notes = f"Phase timing discrepancy: {'; '.join(timing_issues)}"
            else:
                phase.timing_verified = True
                phase.timing_notes = "Verified"

            if self.active_phase == phase_name:
                self.active_phase = None

        self.progress(
            f"[PPK2] Phase '{phase_name}' completed: {phase.avg_current_ma:.2f} mA avg, "
            f"{phase.avg_power_mw:.1f} mW, energy {phase.total_energy_mj:.1f} mJ ({phase.duration_s:.2f}s)."
        )
        return t_now_ns

    def stop_sampling(self) -> None:
        """Stops continuous power sampling."""
        if not self._sampling_active:
            return

        self._stop_event.set()
        if self._sampling_thread and self._sampling_thread.is_alive():
            self._sampling_thread.join(timeout=2.0)
        self._sampling_active = False

        if self._device:
            try:
                self._device.stop_measuring()
            except Exception:
                pass

        # Close any lingering active phase
        if self.active_phase:
            self.mark_phase_end(self.active_phase)

        self.progress(f"[PPK2] Sampling stopped. Captured {len(self.all_currents_ua)} total raw samples.")

    def export_csv(self, filepath: Path | str) -> Path:
        """Saves the complete captured power stream as a timestamped CSV file."""
        out_path = Path(filepath)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        voltage_v = self.voltage_mv / 1000.0

        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "Current_uA", "Current_mA", "Voltage_V", "Power_mW"])
            with self._lock:
                for t_s, i_ua in zip(self.all_timestamps_s, self.all_currents_ua):
                    i_ma = i_ua / 1000.0
                    p_mw = i_ma * voltage_v
                    writer.writerow([f"{t_s:.6f}", f"{i_ua:.2f}", f"{i_ma:.4f}", f"{voltage_v:.3f}", f"{p_mw:.4f}"])

        self.progress(f"[PPK2] Exported complete power CSV: {out_path} ({len(self.all_currents_ua)} rows)")
        return out_path

    def export_phase_csv(self, phase_name: str, filepath: Path | str) -> Optional[Path]:
        """Saves a phase-specific CSV containing only samples within that phase window."""
        phase = self.phases.get(phase_name)
        if not phase or not phase.samples:
            return None

        out_path = Path(filepath)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        voltage_v = self.voltage_mv / 1000.0

        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["Relative_Time_s", "Current_uA", "Current_mA", "Voltage_V", "Power_mW"])
            for idx, i_ua in enumerate(phase.samples):
                t_rel = idx * self.sample_period_s
                i_ma = i_ua / 1000.0
                p_mw = i_ma * voltage_v
                writer.writerow([f"{t_rel:.6f}", f"{i_ua:.2f}", f"{i_ma:.4f}", f"{voltage_v:.3f}", f"{p_mw:.4f}"])

        return out_path

    def get_phase_summary_dict(self) -> Dict[str, Dict[str, Any]]:
        """Returns dictionary of phase metrics for reporting and workbook updates."""
        res: Dict[str, Dict[str, Any]] = {}
        for name, p in self.phases.items():
            res[name] = {
                "avg_current_ma": p.avg_current_ma,
                "min_current_ma": p.min_current_ma,
                "max_current_ma": p.max_current_ma,
                "avg_power_mw": p.avg_power_mw,
                "total_energy_mj": p.total_energy_mj,
                "duration_s": p.duration_s,
                "sample_count": len(p.samples),
                "timing_verified": p.timing_verified,
                "timing_notes": p.timing_notes,
            }
        return res

    def close(self, keep_power_on: bool = False) -> None:
        """Safely shuts down sampling and connection."""
        self.stop_sampling()
        if not keep_power_on:
            try:
                self.power_off(force=True)
            except Exception:
                pass
        device = self._device
        serial_port = getattr(device, "ser", None)
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        self._device = None
        self.progress("[PPK2] Closed connection.")

