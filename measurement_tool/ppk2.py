from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .aggregator import convert_unit
from .ppk2_import import parse_ppk2_csv


STANDARD_PHASES = [
    "Idle",
    "Recording",
    "Active SD write",
    "Active SD read",
    "Stop 2 / sleep",
    "RTC wake-up",
    "Peak workload",
]


@dataclass
class PhaseMetrics:
    phase_name: str
    duration_s: float
    min_current_ma: float
    avg_current_ma: float
    max_current_ma: float
    peak_current_ma: float
    avg_voltage_v: float
    avg_power_mw: float
    max_power_mw: float
    total_energy_mj: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "phase": self.phase_name,
            "duration_s": round(self.duration_s, 3),
            "min_current_ma": round(self.min_current_ma, 4),
            "avg_current_ma": round(self.avg_current_ma, 4),
            "max_current_ma": round(self.max_current_ma, 4),
            "peak_current_ma": round(self.peak_current_ma, 4),
            "avg_voltage_v": round(self.avg_voltage_v, 3),
            "avg_power_mw": round(self.avg_power_mw, 3),
            "max_power_mw": round(self.max_power_mw, 3),
            "total_energy_mj": round(self.total_energy_mj, 3),
        }


@dataclass
class PPK2Sample:
    timestamp: float  # seconds relative or unix epoch
    current_ma: float
    voltage_v: float = 3.3


class PPK2Processor:
    """Processes Nordic PPK2 CSV exports, performs phase segmentation, and calculates power/energy metrics."""

    def __init__(self, voltage_default: float = 3.3) -> None:
        self.voltage_default = voltage_default
        self.samples: List[PPK2Sample] = []
        self.summary_metrics: Dict[str, Any] = {}

    def load_csv(self, csv_path: str | Path) -> None:
        path = Path(csv_path)
        if not path.exists():
            raise FileNotFoundError(f"PPK2 CSV not found: {path}")

        # First check if this is a summary CSV (e.g. ppk2_sample.csv)
        summary = parse_ppk2_csv(path)
        self.summary_metrics = summary

        # Read rows
        with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames:
                return

            # Detect current & voltage columns
            cur_col = None
            volt_col = None
            time_col = None

            for col in reader.fieldnames:
                clow = col.strip().lower()
                if "current" in clow:
                    cur_col = col
                elif "volt" in clow or "vdd" in clow:
                    volt_col = col
                elif "time" in clow:
                    time_col = col

            rows = list(reader)
            if not rows or cur_col is None:
                return

            # Check if this has time-series data (> 2 rows)
            if len(rows) > 1:
                t0 = None
                for idx, r in enumerate(rows):
                    raw_cur = r.get(cur_col)
                    if not raw_cur:
                        continue
                    try:
                        c_val = float(raw_cur)
                    except ValueError:
                        continue

                    # Normalize current to mA
                    if "ua" in cur_col.lower() or "µa" in cur_col.lower() or "μa" in cur_col.lower():
                        c_ma = c_val * 1e-3
                    elif " a" in cur_col.lower() or cur_col.lower().endswith("(a)"):
                        c_ma = c_val * 1e3
                    else:
                        c_ma = c_val

                    # Voltage
                    v_val = self.voltage_default
                    if volt_col and r.get(volt_col):
                        try:
                            raw_v = float(r[volt_col])
                            v_val = raw_v * 1e-3 if "mv" in volt_col.lower() else raw_v
                        except ValueError:
                            pass

                    # Timestamp (seconds)
                    t_val = float(idx) * 0.001
                    if time_col and r.get(time_col):
                        raw_t = r[time_col].strip()
                        try:
                            t_val = float(raw_t)
                        except ValueError:
                            try:
                                dt = datetime.fromisoformat(raw_t.replace("Z", "+00:00"))
                                if t0 is None:
                                    t0 = dt.timestamp()
                                t_val = dt.timestamp() - t0
                            except ValueError:
                                pass

                    self.samples.append(PPK2Sample(timestamp=t_val, current_ma=c_ma, voltage_v=v_val))

    def calculate_phase(self, phase_name: str, samples: List[PPK2Sample]) -> PhaseMetrics:
        if not samples:
            return PhaseMetrics(
                phase_name=phase_name,
                duration_s=0.0,
                min_current_ma=0.0,
                avg_current_ma=0.0,
                max_current_ma=0.0,
                peak_current_ma=0.0,
                avg_voltage_v=self.voltage_default,
                avg_power_mw=0.0,
                max_power_mw=0.0,
                total_energy_mj=0.0,
            )

        currents = [s.current_ma for s in samples]
        voltages = [s.voltage_v for s in samples]

        duration = max(0.001, samples[-1].timestamp - samples[0].timestamp) if len(samples) > 1 else 1.0
        min_cur = min(currents)
        max_cur = max(currents)
        avg_cur = sum(currents) / len(currents)
        peak_cur = max_cur

        avg_v = sum(voltages) / len(voltages) if voltages else self.voltage_default
        avg_p_mw = avg_cur * avg_v
        max_p_mw = max_cur * avg_v
        total_e_mj = avg_p_mw * duration

        return PhaseMetrics(
            phase_name=phase_name,
            duration_s=duration,
            min_current_ma=min_cur,
            avg_current_ma=avg_cur,
            max_current_ma=max_cur,
            peak_current_ma=peak_cur,
            avg_voltage_v=avg_v,
            avg_power_mw=avg_p_mw,
            max_power_mw=max_p_mw,
            total_energy_mj=total_e_mj,
        )

    def segment_by_windows(self, windows: Dict[str, Tuple[float, float]]) -> Dict[str, PhaseMetrics]:
        """Segments samples using start/end time windows in seconds: {'Idle': (0.0, 5.0), ...}."""
        result: Dict[str, PhaseMetrics] = {}
        for phase_name, (start_t, end_t) in windows.items():
            matching = [s for s in self.samples if start_t <= s.timestamp <= end_t]
            result[phase_name] = self.calculate_phase(phase_name, matching)
        return result

    def get_summary_phase_metrics(self) -> Dict[str, PhaseMetrics]:
        """Fallback when raw time-series is a single row summary (like ppk2_sample.csv)."""
        metrics = self.summary_metrics
        required = ("average_current", "minimum_current", "maximum_current", "measurement_duration", "voltage")
        missing = [key for key in required if metrics.get(key) is None]
        if missing:
            # An absent/summary-only capture is not a zero-current measurement.
            return {}
        avg_a = metrics["average_current"]
        min_a = metrics["minimum_current"]
        max_a = metrics["maximum_current"]
        peak_a = metrics.get("peak_current", max_a)
        dur_s = metrics["measurement_duration"]
        volt_v = metrics["voltage"]

        # Convert A to mA
        avg_ma = avg_a * 1e3
        min_ma = min_a * 1e3
        max_ma = max_a * 1e3
        peak_ma = peak_a * 1e3

        primary = PhaseMetrics(
            phase_name="Recording",
            duration_s=dur_s,
            min_current_ma=min_ma,
            avg_current_ma=avg_ma,
            max_current_ma=max_ma,
            peak_current_ma=peak_ma,
            avg_voltage_v=volt_v,
            avg_power_mw=avg_ma * volt_v,
            max_power_mw=max_ma * volt_v,
            total_energy_mj=(avg_ma * volt_v) * dur_s,
        )
        return {"Recording": primary}

    def export_phase_csv(self, phases: Dict[str, PhaseMetrics], output_path: str | Path) -> Path:
        out = Path(output_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "phase",
            "duration_s",
            "min_current_ma",
            "avg_current_ma",
            "max_current_ma",
            "peak_current_ma",
            "avg_voltage_v",
            "avg_power_mw",
            "max_power_mw",
            "total_energy_mj",
        ]
        with out.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for phase_obj in phases.values():
                writer.writerow(phase_obj.to_dict())
        return out


def is_automated_ppk2_available() -> Tuple[bool, str]:
    """Checks whether an automated Nordic PPK2 Python API or CLI is present on the host system."""
    try:
        import pynrfjprog  # type: ignore # noqa: F401
        return True, "pynrfjprog API is available"
    except ImportError:
        pass

    try:
        import ppk2_api  # type: ignore # noqa: F401
        return True, "ppk2_api is available"
    except ImportError:
        pass

    return False, "Automated PPK2 API not detected. Use Nordic nRF Connect Power Profiler app to export CSV."

