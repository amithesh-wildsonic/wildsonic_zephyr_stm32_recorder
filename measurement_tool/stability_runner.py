from __future__ import annotations

import collections
import csv
import json
import logging
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from .controller import SerialController
from .energy import energy_mj_from_mw_s, power_mw_from_v_ma
from .models import TestSession
from .parsers import parse_firmware_log
from .parsers.boot import parse_boot_metrics
from .parsers.recording import parse_recording_metrics
from .ppk2_controller import MockPPK2Device, PPK2Controller
from .stability import (
    AssessmentEngine,
    ErrorDetector,
    ErrorSeverity,
    ErrorSignatureTracker,
    ErrorSubsystem,
    PowerTrendInterval,
    RecoveryEvent,
    SessionTelemetry,
    StructuredError,
)
from .workbook import ComparisonWorkbookManager

logger = logging.getLogger("measurement_tool.stability_runner")


class StabilityRunner:
    """Orchestrates long-duration stability and automatic recovery tests using PPK2 and STM32 UART."""

    def __init__(
        self,
        controller: SerialController,
        ppk2: PPK2Controller,
        sample_rate: int = 96000,
        bit_depth: int = 16,
        voltage_mv: int = 5000,
        duration_hours: float = 8.0,
        duration_seconds: Optional[float] = None,
        session_rect_s: int = 60,
        session_sleep_s: int = 5,
        boot_stabilize_s: float = 1.5,
        discharge_duration_s: float = 2.0,
        recovery_timeout_s: float = 15.0,
        max_signature_recoveries: int = 5,
        max_total_power_cycles: int = 10,
        trend_interval_s: float = 1.0,
        progress_callback: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.controller = controller
        self.ppk2 = ppk2
        self.sample_rate = sample_rate
        self.bit_depth = bit_depth
        self.voltage_mv = voltage_mv
        self.total_target_seconds = duration_seconds if duration_seconds is not None else (duration_hours * 3600.0)
        self.session_rect_s = session_rect_s
        self.session_sleep_s = session_sleep_s
        self.boot_stabilize_s = boot_stabilize_s
        self.discharge_duration_s = discharge_duration_s
        self.recovery_timeout_s = recovery_timeout_s
        self.max_signature_recoveries = max_signature_recoveries
        self.max_total_power_cycles = max_total_power_cycles
        self.trend_interval_s = trend_interval_s
        self.progress = progress_callback or (lambda msg: print(f"  {msg}"))

        # Execution tracking state
        self.start_host_ns: int = 0
        self.power_cycles_count: int = 0
        self.errors: List[StructuredError] = []
        self.error_trackers: Dict[str, ErrorSignatureTracker] = {}
        self.recoveries: List[RecoveryEvent] = []
        self.sessions: List[SessionTelemetry] = []
        self.power_trends: List[PowerTrendInterval] = []
        self.accumulated_energy_mj: float = 0.0
        self.is_running: bool = False
        self.terminal_reason: str = ""
        self.final_status: str = "PENDING"
        self.incomplete_wav_count: int = 0
        self.current_session_id: int = 1

        # High-resolution circular buffer: holds last 15 seconds of raw uA samples
        # 15s * 100 kS/s = 1,500,000 samples
        self.raw_rolling_buffer: Deque[Tuple[float, float]] = collections.deque(maxlen=1500000)

    def run(
        self,
        test_id: str = "STM32-STABILITY-001",
        evidence_type: str = "HARDWARE",
    ) -> TestSession:
        """Executes the full stability test lifecycle."""
        self.progress(f"\n================================================================================")
        self.progress(f"   STM32L476RG LONG-DURATION STABILITY & AUTOMATIC RECOVERY TEST: {test_id}")
        self.progress(f"   Target Duration: {self.total_target_seconds / 3600.0:.2f} hours ({self.total_target_seconds:.0f}s)")
        self.progress(f"   Configuration  : {self.sample_rate} Hz, {self.bit_depth}-bit, Mono | E5V ({self.voltage_mv} mV)")
        self.progress(f"================================================================================\n")

        # 1. Setup output paths
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        raw_log_path = Path("data/stm32/raw_logs") / f"{test_id}_{stamp}.raw.log"
        ts_log_path = Path("data/stm32/raw_logs") / f"{test_id}_{stamp}.timestamped.log"
        errors_csv_path = Path("data/stm32/reports") / f"{test_id}_{stamp}_errors.csv"
        recoveries_csv_path = Path("data/stm32/reports") / f"{test_id}_{stamp}_recoveries.csv"
        sessions_csv_path = Path("data/stm32/reports") / f"{test_id}_{stamp}_sessions.csv"
        power_trend_csv_path = Path("data/ppk2") / f"{test_id}_{stamp}_power_trend.csv"
        parsed_json_path = Path("data/stm32/parsed") / f"{test_id}_{stamp}.json"

        for p in (raw_log_path, errors_csv_path, power_trend_csv_path, parsed_json_path):
            p.parent.mkdir(parents=True, exist_ok=True)

        # 2. Establish connections
        self.progress("[1/6] Connecting to STM32 UART console...")
        self.controller.connect(test_name=test_id)

        self.progress("[2/6] Connecting to Nordic PPK2 Power Controller...")
        self.ppk2.connect(exclude_uart_port=self.controller.port)
        self.ppk2.configure_source_meter(self.voltage_mv)

        # Ensure safe initial power state
        self.ppk2.power_off(force=True)
        time.sleep(self.discharge_duration_s)

        self.progress("[3/6] Starting PPK2 continuous power sampling and energizing target...")
        self.ppk2.start_sampling()
        t_pwr_on = self.ppk2.power_on()
        self.start_host_ns = t_pwr_on
        time.sleep(self.boot_stabilize_s)

        # 3. Verify communication via DEVINFO
        self.progress("[4/6] Verifying device readiness via bounded DEVINFO query...")
        devinfo_ok = self._bounded_devinfo_poll(timeout_s=self.recovery_timeout_s)
        if not devinfo_ok:
            raise RuntimeError(f"Device did not respond to DEVINFO within {self.recovery_timeout_s}s after power-on!")

        # 4. Apply initial stability configuration
        self.progress(f"[5/6] Applying stability settings: SAMF {self.sample_rate}, BTPS {self.bit_depth}, RECT {self.session_rect_s}, SLEEP {self.session_sleep_s}...")
        self._configure_target()

        # 5. Enter long-duration monitoring loop
        self.progress(f"[6/6] Autonomous recording online. Commencing long-duration monitoring loop...")
        self.is_running = True
        last_trend_time = time.monotonic()
        loop_start_time = time.monotonic()

        try:
            while self.is_running:
                now = time.monotonic()
                elapsed = now - loop_start_time

                if elapsed >= self.total_target_seconds:
                    self.progress(f"\n[STABILITY] Target duration reached ({elapsed:.1f}s / {self.total_target_seconds:.1f}s). Completing test gracefully.")
                    self.terminal_reason = "Target test duration completed successfully."
                    self.final_status = "PASS" if not any(t.is_persistent for t in self.error_trackers.values()) else "FAIL"
                    break

                # A. Read incoming UART lines
                lines = self.controller.read_lines()
                for line in lines:
                    self._process_uart_line(line, elapsed, test_id, stamp)

                    # Check if a fatal fault or limit reached stopped execution
                    if not self.is_running:
                        break

                if not self.is_running:
                    break

                # B. Aggregate 1-second power trend statistics
                if now - last_trend_time >= self.trend_interval_s:
                    self._collect_trend_slice(now, power_trend_csv_path)
                    last_trend_time = now

                time.sleep(0.05)

        except KeyboardInterrupt:
            self.progress("\n[STABILITY] Interrupted by user. Finalizing diagnostics...")
            self.terminal_reason = "Manual user abort."
            self.final_status = "INCOMPLETE"
        except Exception as ex:
            logger.error("Stability test runner exception: %s", ex, exc_info=True)
            self.terminal_reason = f"Unhandled exception: {ex}"
            self.final_status = "FAIL"
        finally:
            self._shutdown_and_export(
                test_id=test_id,
                stamp=stamp,
                evidence_type=evidence_type,
                raw_log_path=raw_log_path,
                ts_log_path=ts_log_path,
                errors_csv_path=errors_csv_path,
                recoveries_csv_path=recoveries_csv_path,
                sessions_csv_path=sessions_csv_path,
                power_trend_csv_path=power_trend_csv_path,
                parsed_json_path=parsed_json_path,
            )

        return self._generate_session_object(test_id, evidence_type, raw_log_path, power_trend_csv_path, parsed_json_path)

    def _bounded_devinfo_poll(self, timeout_s: float = 15.0) -> bool:
        """Polls DEVINFO with bounded retries until target responds or times out."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            ok, match = self.controller.send_command("DEVINFO", wait_for=r"(?:SAMF|VER|DT|OK)", timeout=1.5)
            if ok and match:
                return True
            time.sleep(0.5)
        return False

    def _configure_target(self) -> None:
        """Applies sample rate, bit depth, recording duration, and verifies config."""
        self.controller.send_command(f"SAMF {self.sample_rate}", wait_for="Value is set to", timeout=3.0)
        self.controller.send_command(f"BTPS {self.bit_depth}", wait_for="Value is set to", timeout=3.0)
        self.controller.send_command(f"RECT {self.session_rect_s}", wait_for="Value is set to", timeout=3.0)
        self.controller.send_command(f"SLEEP {self.session_sleep_s}", wait_for="Value is set to", timeout=3.0)
        self.controller.send_command("APPLY", wait_for=r"(?:Device configuration is applied|OK)", timeout=3.0)
        self.controller.send_command("STATUS", wait_for=r"(?:=== SYSTEM & RECORDING STATUS ===|OK)", timeout=3.0)

    def _process_uart_line(self, line: str, elapsed_s: float, test_id: str, stamp: str) -> None:
        """Processes a single UART line, detecting errors, session markers, and triggering recovery."""
        # Detect session transitions
        sess_start_m = re.search(r"AUTONOMOUS RECORDING SESSION (\d+) START|NEXT_RECORDING_STARTED:\s*session=(\d+)", line, re.I)
        if sess_start_m:
            self.current_session_id = int(sess_start_m.group(1) or sess_start_m.group(2))

        # Detect session completion and recording accounting summary
        if "RECORDING ACCOUNTING SUMMARY:" in line or "Session" in line and "finished:" in line:
            # Parse completed session metrics from recent text
            recent_text = self.controller.all_text[-2500:]
            metrics = parse_recording_metrics(recent_text)
            if metrics.get("acquired_samples") is not None:
                telemetry = SessionTelemetry(
                    session_id=self.current_session_id,
                    start_uptime_s=max(0.0, elapsed_s - metrics.get("recording_duration", self.session_rect_s)),
                    end_uptime_s=elapsed_s,
                    duration_s=metrics.get("recording_duration", self.session_rect_s),
                    sample_rate=self.sample_rate,
                    bit_depth=self.bit_depth,
                    expected_samples=int(metrics.get("expected_samples", self.sample_rate * self.session_rect_s)),
                    acquired_samples=int(metrics.get("acquired_samples", 0)),
                    processed_samples=int(metrics.get("processed_samples", 0)),
                    samples_written=int(metrics.get("samples_written", 0)),
                    boundary_discarded=int(metrics.get("boundary_samples_discarded", 0)),
                    lost_samples=int(metrics.get("lost_samples", 0)),
                    wav_verified=str(metrics.get("wav_verification", "UNKNOWN")),
                    active_write_throughput_kb_s=float(metrics.get("active_write_throughput", 0.0)),
                    worst_batch_latency_us=float(metrics.get("worst_batch_latency", 0.0)),
                    avg_dsp_us=float(metrics.get("avg_dsp_execution_time", 0.0)),
                    worst_dsp_us=float(metrics.get("worst_dsp_execution_time", 0.0)),
                    adc_overruns=int(metrics.get("adc_overruns", 0)),
                    dma_errors=int(metrics.get("dma_errors", 0)),
                    psram_full_events=int(metrics.get("psram_full_events", 0)),
                    is_complete=True,
                )
                self.sessions.append(telemetry)
                self.progress(
                    f"  [SESSION {telemetry.session_id} OK] Wrote {telemetry.samples_written} samples "
                    f"({telemetry.duration_s:.1f}s, {telemetry.active_write_throughput_kb_s:.0f} KB/s, WAV: {telemetry.wav_verified})"
                )

        # Detect error patterns
        error = ErrorDetector.scan_line(
            line=line,
            uptime_s=elapsed_s,
            session_id=self.current_session_id,
            sample_rate=self.sample_rate,
            context_before=self.controller.all_text[-500:],
        )

        if not error:
            return

        self.errors.append(error)
        sig = error.normalized_signature

        # Update signature tracker
        if sig not in self.error_trackers:
            self.error_trackers[sig] = ErrorSignatureTracker(
                signature=sig,
                subsystem=error.subsystem,
                severity=error.severity,
            )
        tracker = self.error_trackers[sig]
        tracker.record_occurrence(elapsed_s, was_recovered=False)

        self.progress(
            f"\n[ERROR DETECTED] [{error.severity.value}] Subsystem: {error.subsystem.value} | Code: {error.error_code}\n"
            f"  Signature: {sig}\n"
            f"  Message  : {error.raw_message}\n"
            f"  Occurrences: {tracker.total_occurrences} (Uptime: {elapsed_s:.1f}s)"
        )

        # Snapshot high-resolution PPK2 capture around error event
        self._export_event_ppk2_capture(test_id, stamp, f"err_{len(self.errors)}")

        # Handle FATAL severity
        if error.severity == ErrorSeverity.FATAL:
            self.progress(f"[FATAL ERROR] Fatal condition '{sig}' cannot be safely recovered. Entering safe shutdown.")
            tracker.is_persistent = True
            self.terminal_reason = f"Fatal system error: {sig}"
            self.final_status = "FAIL"
            self.is_running = False
            return

        # Check recovery limits
        if tracker.failed_recovery_attempts >= self.max_signature_recoveries:
            self.progress(f"[PERSISTENT ERROR] Signature '{sig}' exceeded maximum recovery limit ({self.max_signature_recoveries}).")
            tracker.is_persistent = True
            self.terminal_reason = f"Persistent error exceeded limit: {sig}"
            self.final_status = "FAIL"
            self.is_running = False
            return

        if self.power_cycles_count >= self.max_total_power_cycles:
            self.progress(f"[LIMIT REACHED] Total power-cycle recovery limit ({self.max_total_power_cycles}) reached.")
            self.terminal_reason = f"Total power cycles exhausted ({self.max_total_power_cycles})"
            self.final_status = "FAIL"
            self.is_running = False
            return

        # Execute automatic PPK2 recovery for recoverable / critical errors
        if error.severity in {ErrorSeverity.RECOVERABLE, ErrorSeverity.CRITICAL}:
            self._execute_ppk2_recovery(error, tracker, elapsed_s, test_id, stamp)

    def _execute_ppk2_recovery(
        self,
        error: StructuredError,
        tracker: ErrorSignatureTracker,
        elapsed_s: float,
        test_id: str,
        stamp: str,
    ) -> None:
        """Executes the safe multi-step PPK2 power-cycle recovery sequence."""
        rec_id = f"REC_{len(self.recoveries) + 1:03d}"
        self.progress(f"\n>>> [AUTOMATIC RECOVERY {rec_id}] Initiating PPK2 power-cycle recovery for '{error.normalized_signature}'...")
        t_rec_start = time.monotonic()
        error.recovery_attempted = True

        # 1. Attempt graceful recording stop
        self.progress("    [1/5] Attempting graceful recording finalization...")
        self.controller.send_command("STOP", wait_for=None, timeout=2.0)
        time.sleep(1.0)

        # 2. Safety Interlock: Check device responsiveness & storage activity
        # If active SD write is still ongoing and responding, wait up to recovery timeout
        wait_deadline = time.monotonic() + 5.0
        is_safe_to_cut = True
        while time.monotonic() < wait_deadline:
            if re.search(r"WAV_FILE_CLOSED|SDMMC_IDLE|Session \d+ finished", self.controller.all_text[-500:], re.I):
                is_safe_to_cut = True
                break
            time.sleep(0.5)

        if not is_safe_to_cut:
            self.incomplete_wav_count += 1
            self.progress("    [WARNING] Recording did not finalize cleanly; file marked incomplete before power cut.")

        # 3. Power cut via PPK2 Source Meter
        self.progress(f"    [2/5] Disabling PPK2 DUT power for discharge ({self.discharge_duration_s:.1f}s)...")
        self.ppk2.power_off(force=True)
        time.sleep(self.discharge_duration_s)
        self.power_cycles_count += 1

        # 4. Power re-enable and boot stabilization
        self.progress(f"    [3/5] Re-enabling PPK2 DUT power @ {self.voltage_mv} mV...")
        t_on_ns = self.ppk2.power_on()
        time.sleep(self.boot_stabilize_s)

        # 5. Poll DEVINFO
        self.progress(f"    [4/5] Polling DEVINFO (timeout {self.recovery_timeout_s:.1f}s)...")
        t_devinfo_start = time.monotonic()
        devinfo_ok = self._bounded_devinfo_poll(timeout_s=self.recovery_timeout_s)
        devinfo_lat_ms = (time.monotonic() - t_devinfo_start) * 1000.0

        if not devinfo_ok:
            self.progress("    [RECOVERY FAILED] Target failed to respond to DEVINFO after power cycle!")
            tracker.failed_recovery_attempts += 1
            rec_event = RecoveryEvent(
                event_id=rec_id,
                timestamp_iso=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
                uptime_s=elapsed_s,
                trigger_error_signature=error.normalized_signature,
                discharge_duration_s=self.discharge_duration_s,
                boot_stabilize_s=self.boot_stabilize_s,
                devinfo_latency_ms=devinfo_lat_ms,
                status_verified=False,
                reconfigured=False,
                result="FAILED",
                total_recovery_duration_s=time.monotonic() - t_rec_start,
                notes="DEVINFO communication timeout after power cycle.",
            )
            self.recoveries.append(rec_event)
            return

        # 6. Reconfigure target recording parameters
        self.progress("    [5/5] Target operational. Re-applying recording configuration...")
        self._configure_target()

        total_rec_dur = time.monotonic() - t_rec_start
        tracker.successful_power_cycles += 1
        error.recovery_successful = True
        error.recovered = True

        rec_event = RecoveryEvent(
            event_id=rec_id,
            timestamp_iso=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            uptime_s=elapsed_s,
            trigger_error_signature=error.normalized_signature,
            discharge_duration_s=self.discharge_duration_s,
            boot_stabilize_s=self.boot_stabilize_s,
            devinfo_latency_ms=devinfo_lat_ms,
            status_verified=True,
            reconfigured=True,
            result="SUCCESS",
            total_recovery_duration_s=total_rec_dur,
            notes="Recovered cleanly; autonomous recording resumed.",
        )
        self.recoveries.append(rec_event)
        self.progress(f"[RECOVERY {rec_id} SUCCESS] System resumed in {total_rec_dur:.2f}s (DEVINFO latency: {devinfo_lat_ms:.1f}ms).")

        # Snapshot post-recovery PPK2 slice
        self._export_event_ppk2_capture(test_id, stamp, f"rec_{rec_id}")

    def _collect_trend_slice(self, now: float, out_csv: Path) -> None:
        """Computes 1-second PPK2 trend statistics and appends to power trend CSV."""
        cur_samples = list(self.ppk2.all_currents_ua)
        if not cur_samples:
            return

        # Grab recent window (last ~1.0 second, i.e., 100,000 samples)
        window_size = 100000
        slice_samples = cur_samples[-window_size:] if len(cur_samples) > window_size else cur_samples
        if not slice_samples:
            return

        # Update circular buffer with timestamped values
        for samp in slice_samples[-1000:]:
            self.raw_rolling_buffer.append((now, samp))

        min_ma = min(slice_samples) / 1000.0
        max_ma = max(slice_samples) / 1000.0
        avg_ma = (sum(slice_samples) / len(slice_samples)) / 1000.0
        volt_v = self.voltage_mv / 1000.0
        power_mw = power_mw_from_v_ma(volt_v, avg_ma)
        dur_s = self.trend_interval_s
        energy_mj = energy_mj_from_mw_s(power_mw, dur_s)
        self.accumulated_energy_mj += energy_mj

        interval = PowerTrendInterval(
            timestamp_s=now,
            sample_count=len(slice_samples),
            min_current_ma=min_ma,
            avg_current_ma=avg_ma,
            max_current_ma=max_ma,
            peak_current_ma=max_ma,
            avg_voltage_v=volt_v,
            avg_power_mw=power_mw,
            interval_energy_mj=energy_mj,
            accumulated_energy_mj=self.accumulated_energy_mj,
        )
        self.power_trends.append(interval)

        # Append to CSV
        file_exists = out_csv.exists()
        with out_csv.open("a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    "Timestamp_s", "Sample_Count", "Min_Current_mA", "Avg_Current_mA",
                    "Max_Current_mA", "Peak_Current_mA", "Voltage_V", "Power_mW",
                    "Interval_Energy_mJ", "Accumulated_Energy_mJ"
                ])
            writer.writerow([
                f"{interval.timestamp_s:.3f}", interval.sample_count,
                f"{interval.min_current_ma:.4f}", f"{interval.avg_current_ma:.4f}",
                f"{interval.max_current_ma:.4f}", f"{interval.peak_current_ma:.4f}",
                f"{interval.avg_voltage_v:.3f}", f"{interval.avg_power_mw:.3f}",
                f"{interval.interval_energy_mj:.2f}", f"{interval.accumulated_energy_mj:.2f}",
            ])

    def _export_event_ppk2_capture(self, test_id: str, stamp: str, event_tag: str) -> None:
        """Dumps high-resolution raw samples from rolling buffer to event-specific CSV."""
        if not self.raw_rolling_buffer:
            return
        event_path = Path("data/ppk2") / f"{test_id}_{stamp}_event_{event_tag}.csv"
        event_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with event_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["Host_Timestamp_s", "Current_uA", "Current_mA", "Voltage_V", "Power_mW"])
                volt_v = self.voltage_mv / 1000.0
                for t_s, i_ua in list(self.raw_rolling_buffer):
                    i_ma = i_ua / 1000.0
                    p_mw = i_ma * volt_v
                    writer.writerow([f"{t_s:.6f}", f"{i_ua:.2f}", f"{i_ma:.4f}", f"{volt_v:.3f}", f"{p_mw:.4f}"])
        except Exception as ex:
            logger.debug("Failed to write event PPK2 CSV: %s", ex)

    def _shutdown_and_export(
        self,
        test_id: str,
        stamp: str,
        evidence_type: str,
        raw_log_path: Path,
        ts_log_path: Path,
        errors_csv_path: Path,
        recoveries_csv_path: Path,
        sessions_csv_path: Path,
        power_trend_csv_path: Path,
        parsed_json_path: Path,
    ) -> None:
        """Safely terminates sampling, outputs CSVs, evaluates metrics, and updates workbook."""
        self.progress("\n================================================================================")
        self.progress("                   FINALIZING STABILITY TEST ARTIFACTS")
        self.progress("================================================================================")

        self.ppk2.stop_sampling()
        self.controller.stop()

        # 1. Export structured errors CSV
        with errors_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Timestamp_ISO", "Uptime_s", "Session_ID", "Sample_Rate", "Subsystem",
                "Severity", "Error_Code", "Normalized_Signature", "Raw_Message",
                "Recovered", "Recovery_Attempted", "Recovery_Successful"
            ])
            for err in self.errors:
                writer.writerow([
                    err.timestamp_iso, f"{err.uptime_s:.3f}", err.session_id, err.sample_rate,
                    err.subsystem.value, err.severity.value, err.error_code, err.normalized_signature,
                    err.raw_message, err.recovered, err.recovery_attempted, err.recovery_successful
                ])

        # 2. Export recovery history CSV
        with recoveries_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Event_ID", "Timestamp_ISO", "Uptime_s", "Trigger_Signature", "Discharge_s",
                "Boot_Stabilize_s", "DEVINFO_Latency_ms", "Status_Verified", "Reconfigured",
                "Result", "Duration_s", "Notes"
            ])
            for rec in self.recoveries:
                writer.writerow([
                    rec.event_id, rec.timestamp_iso, f"{rec.uptime_s:.3f}", rec.trigger_error_signature,
                    f"{rec.discharge_duration_s:.2f}", f"{rec.boot_stabilize_s:.2f}",
                    f"{rec.devinfo_latency_ms:.1f}", rec.status_verified, rec.reconfigured,
                    rec.result, f"{rec.total_recovery_duration_s:.2f}", rec.notes
                ])

        # 3. Export session telemetry CSV
        with sessions_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Session_ID", "Start_Uptime_s", "End_Uptime_s", "Duration_s", "Sample_Rate",
                "Bit_Depth", "Expected_Samples", "Acquired_Samples", "Processed_Samples",
                "Samples_Written", "Boundary_Discarded", "Lost_Samples", "WAV_Verified",
                "Write_Throughput_KB_s", "Worst_Latency_us", "Avg_DSP_us", "Worst_DSP_us",
                "ADC_OVR", "DMA_ERR", "PSRAM_FULL", "Complete"
            ])
            for s in self.sessions:
                writer.writerow([
                    s.session_id, f"{s.start_uptime_s:.2f}", f"{s.end_uptime_s:.2f}", f"{s.duration_s:.2f}",
                    s.sample_rate, s.bit_depth, s.expected_samples, s.acquired_samples, s.processed_samples,
                    s.samples_written, s.boundary_discarded, s.lost_samples, s.wav_verified,
                    f"{s.active_write_throughput_kb_s:.1f}", f"{s.worst_batch_latency_us:.1f}",
                    f"{s.avg_dsp_us:.1f}", f"{s.worst_dsp_us:.1f}", s.adc_overruns, s.dma_errors,
                    s.psram_full_events, s.is_complete
                ])

        # 4. Perform Hardware vs Firmware Assessment
        classification, rationale, evidence_points = AssessmentEngine.assess(
            trackers=self.error_trackers,
            recoveries=self.recoveries,
            power_cycles_count=self.power_cycles_count,
        )

        # 5. Compute min/max/avg metrics across sessions and power trends
        actual_uptime_s = (time.monotonic_ns() - self.start_host_ns) / 1e9 if self.start_host_ns > 0 else 0.0
        tot_rec_time = sum(s.duration_s for s in self.sessions)

        thru_vals = [s.active_write_throughput_kb_s for s in self.sessions if s.active_write_throughput_kb_s > 0]
        thru_min = min(thru_vals) if thru_vals else 0.0
        thru_max = max(thru_vals) if thru_vals else 0.0
        thru_avg = (sum(thru_vals) / len(thru_vals)) if thru_vals else 0.0

        lat_vals = [s.worst_batch_latency_us for s in self.sessions if s.worst_batch_latency_us > 0]
        lat_min = min(lat_vals) if lat_vals else 0.0
        lat_max = max(lat_vals) if lat_vals else 0.0
        lat_avg = (sum(lat_vals) / len(lat_vals)) if lat_vals else 0.0

        dsp_vals = [s.avg_dsp_us for s in self.sessions if s.avg_dsp_us > 0]
        dsp_min = min(dsp_vals) if dsp_vals else 0.0
        dsp_max = max([s.worst_dsp_us for s in self.sessions if s.worst_dsp_us > 0]) if dsp_vals else 0.0
        dsp_avg = (sum(dsp_vals) / len(dsp_vals)) if dsp_vals else 0.0

        pwr_vals = [p.avg_power_mw for p in self.power_trends if p.avg_power_mw > 0]
        pwr_avg = (sum(pwr_vals) / len(pwr_vals)) if pwr_vals else 0.0
        cur_vals = [p.avg_current_ma for p in self.power_trends if p.avg_current_ma > 0]
        cur_min = min([p.min_current_ma for p in self.power_trends]) if cur_vals else 0.0
        cur_max = max([p.max_current_ma for p in self.power_trends]) if cur_vals else 0.0
        cur_avg = (sum(cur_vals) / len(cur_vals)) if cur_vals else 0.0

        rec_lats = [r.devinfo_latency_ms for r in self.recoveries if r.result == "SUCCESS"]
        rec_lat_min = min(rec_lats) if rec_lats else 0.0
        rec_lat_max = max(rec_lats) if rec_lats else 0.0
        rec_lat_avg = (sum(rec_lats) / len(rec_lats)) if rec_lats else 0.0

        # Compile final parsed dictionary
        parsed_dict = {
            "test_id": test_id,
            "category": "stability-test",
            "evidence_type": evidence_type,
            "status": self.final_status,
            "terminal_reason": self.terminal_reason,
            "target_duration_s": self.total_target_seconds,
            "actual_uptime_s": round(actual_uptime_s, 2),
            "sample_rate": self.sample_rate,
            "bit_depth": self.bit_depth,
            "power_rail_voltage_mv": self.voltage_mv,
            "recording_sessions_count": len(self.sessions),
            "total_recording_time_s": round(tot_rec_time, 2),
            "total_errors_count": len(self.errors),
            "unique_error_signatures_count": len(self.error_trackers),
            "power_cycles_count": self.power_cycles_count,
            "successful_recoveries_count": sum(1 for r in self.recoveries if r.result == "SUCCESS"),
            "failed_recoveries_count": sum(1 for r in self.recoveries if r.result != "SUCCESS"),
            "incomplete_wav_count": self.incomplete_wav_count,
            "recovery_latency_ms": {
                "min": round(rec_lat_min, 1),
                "avg": round(rec_lat_avg, 1),
                "max": round(rec_lat_max, 1),
            },
            "sd_write_throughput_kb_s": {
                "min": round(thru_min, 1),
                "avg": round(thru_avg, 1),
                "max": round(thru_max, 1),
            },
            "sd_batch_latency_us": {
                "min": round(lat_min, 1),
                "avg": round(lat_avg, 1),
                "max": round(lat_max, 1),
            },
            "dsp_execution_us": {
                "min": round(dsp_min, 1),
                "avg": round(dsp_avg, 1),
                "max": round(dsp_max, 1),
            },
            "power_metrics": {
                "min_current_ma": round(cur_min, 4),
                "avg_current_ma": round(cur_avg, 4),
                "max_current_ma": round(cur_max, 4),
                "avg_power_mw": round(pwr_avg, 3),
                "accumulated_energy_mj": round(self.accumulated_energy_mj, 2),
            },
            "hardware_vs_firmware_assessment": {
                "classification": classification,
                "rationale": rationale,
                "evidence": evidence_points,
            },
            "error_signatures": {k: tracker.__dict__ for k, tracker in self.error_trackers.items()},
            "failure_reasons": [t.signature for t in self.error_trackers.values() if t.is_persistent] or ([self.terminal_reason] if self.final_status == "FAIL" else []),
            "parsed_json_path": str(parsed_json_path.as_posix()),
            "raw_log_path": str(raw_log_path.as_posix()),
            "errors_csv_path": str(errors_csv_path.as_posix()),
            "recoveries_csv_path": str(recoveries_csv_path.as_posix()),
            "sessions_csv_path": str(sessions_csv_path.as_posix()),
            "power_trend_csv_path": str(power_trend_csv_path.as_posix()),
        }

        # Write parsed JSON
        json_content = json.dumps(parsed_dict, indent=2)
        parsed_json_path.write_text(json_content, encoding="utf-8")
        Path(f"data/parsed/{test_id}_{stamp}.json").write_text(json_content, encoding="utf-8")

        # 6. Update Excel Comparison Workbook (Stability Test sheet)
        wb_mgr = ComparisonWorkbookManager("data/reports/stm32_vs_max78000.xlsx")
        wb_mgr.update_stability_test_results(parsed_dict)

        # 7. Print Terminal Summary
        self._print_terminal_summary(parsed_dict)

    def _generate_session_object(
        self,
        test_id: str,
        evidence_type: str,
        raw_log: Path,
        trend_csv: Path,
        json_path: Path,
    ) -> TestSession:
        return TestSession(
            test_id=test_id,
            board="STM32L476RG",
            firmware_commit=getattr(self.controller, "firmware_commit", "b1c299f43d0f"),
            build_type="test",
            category="stability-test",
            configuration=f"{self.sample_rate // 1000} kHz / 16-bit / {self.total_target_seconds / 3600.0:.1f}h",
            date_time=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            raw_log_path=str(raw_log),
            ppk2_csv_path=str(trend_csv),
            parsed_result=json.loads(json_path.read_text(encoding="utf-8")),
            sample_data={},
            evidence_type=evidence_type.upper(),
            status=self.final_status.lower(),
        )

    def _print_terminal_summary(self, d: Dict[str, Any]) -> None:
        lines = [
            "=" * 80,
            f"          STM32L476RG LONG-DURATION STABILITY TEST SUMMARY ({d['test_id']})",
            "=" * 80,
            f"Configuration: {d['sample_rate']} Hz, {d['bit_depth']}-bit Mono | Supply: E5V ({d['power_rail_voltage_mv']} mV)",
            f"Status       : {d['status']} | Target: {d['target_duration_s'] / 3600.0:.2f}h | Actual Uptime: {d['actual_uptime_s']:.1f}s",
            f"Evidence Type: {d['evidence_type']} | Terminal Reason: {d['terminal_reason']}",
            "-" * 80,
            f"{'Metric Category / Parameter':<38} | {'Value / Range':<22} | {'Units':<14}",
            "-" * 80,
            f"{'Autonomous Recording Sessions':<38} | {d['recording_sessions_count']:<22} | {'sessions':<14}",
            f"{'Total Recorded Audio Duration':<38} | {d['total_recording_time_s']:<22} | {'seconds':<14}",
            f"{'Total Detected Firmware/HW Errors':<38} | {d['total_errors_count']:<22} | {'events':<14}",
            f"{'Unique Error Signatures':<38} | {d['unique_error_signatures_count']:<22} | {'signatures':<14}",
            f"{'PPK2 Power Cycles Performed':<38} | {d['power_cycles_count']:<22} | {'cycles':<14}",
            f"{'Successful Automatic Recoveries':<38} | {d['successful_recoveries_count']:<22} | {'recoveries':<14}",
            f"{'Failed Recovery Attempts':<38} | {d['failed_recoveries_count']:<22} | {'recoveries':<14}",
            f"{'Interrupted / Incomplete WAVs':<38} | {d['incomplete_wav_count']:<22} | {'files':<14}",
            f"{'Recovery Latency (DEVINFO)':<38} | {d['recovery_latency_ms']['avg']:.1f} (max {d['recovery_latency_ms']['max']:.1f}) | {'ms':<14}",
            f"{'SD Active Write Throughput':<38} | {d['sd_write_throughput_kb_s']['avg']:.1f} (min {d['sd_write_throughput_kb_s']['min']:.1f}) | {'KB/s':<14}",
            f"{'SD Worst Batch Latency':<38} | {d['sd_batch_latency_us']['max']:.0f} | {'us':<14}",
            f"{'DSP Execution Time (Avg / Worst)':<38} | {d['dsp_execution_us']['avg']:.1f} / {d['dsp_execution_us']['max']:.1f} | {'us':<14}",
            f"{'Average Power Consumption':<38} | {d['power_metrics']['avg_power_mw']:.2f} | {'mW':<14}",
            f"{'Accumulated Energy Consumed':<38} | {d['power_metrics']['accumulated_energy_mj']:.1f} | {'mJ':<14}",
            "-" * 80,
            f"FAULT ASSESSMENT: {d['hardware_vs_firmware_assessment']['classification'].upper()}",
            f"Rationale: {d['hardware_vs_firmware_assessment']['rationale']}",
        ]

        if d.get("failure_reasons"):
            lines.append("-" * 80)
            lines.append("DECISIVE FAILURE REASONS:")
            for r in d["failure_reasons"]:
                lines.append(f"  - {r}")

        lines.append("-" * 80)
        lines.append(f"Parsed JSON Artifact    : {d['parsed_json_path']}")
        lines.append(f"Errors CSV Log          : {d['errors_csv_path']}")
        lines.append(f"Recoveries CSV Log      : {d['recoveries_csv_path']}")
        lines.append(f"Sessions CSV Log        : {d['sessions_csv_path']}")
        lines.append(f"PPK2 Power Trend CSV    : {d['power_trend_csv_path']}")
        lines.append("=" * 80)
        print("\n".join(lines))

