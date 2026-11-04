from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence


UNIT_SCALES = {
    # Time
    "us": 1e-6,
    "ms": 1e-3,
    "s": 1.0,
    "sec": 1.0,
    # Frequency
    "hz": 1.0,
    "khz": 1e3,
    "mhz": 1e6,
    # Memory / Data size
    "b": 1.0,
    "bytes": 1.0,
    "kb": 1024.0,
    "kib": 1024.0,
    "mb": 1024.0 * 1024.0,
    "mib": 1024.0 * 1024.0,
    # Throughput
    "b/s": 1.0,
    "kb/s": 1024.0,
    "kib/s": 1024.0,
    "mb/s": 1024.0 * 1024.0,
    "mib/s": 1024.0 * 1024.0,
    # Electrical Current
    "ua": 1e-6,
    "ma": 1e-3,
    "a": 1.0,
    # Voltage
    "uv": 1e-6,
    "mv": 1e-3,
    "v": 1.0,
    # Power
    "uw": 1e-6,
    "mw": 1e-3,
    "w": 1.0,
    # Energy
    "uj": 1e-6,
    "mj": 1e-3,
    "j": 1.0,
    "wh": 3600.0,
    "mwh": 3.6,
}


def convert_unit(value: float, from_unit: str, to_unit: str) -> float:
    fu = from_unit.strip().lower().replace("µ", "u").replace("μ", "u")
    tu = to_unit.strip().lower().replace("µ", "u").replace("μ", "u")
    if fu == tu:
        return value
    if fu not in UNIT_SCALES or tu not in UNIT_SCALES:
        raise ValueError(f"Unsupported unit conversion: {from_unit} -> {to_unit}")
    base_val = value * UNIT_SCALES[fu]
    return base_val / UNIT_SCALES[tu]


@dataclass
class AggregateMetric:
    min: Optional[float] = None
    max: Optional[float] = None
    avg: Optional[float] = None
    count: int = 0
    unit: str = ""
    observations: List[float] = field(default_factory=list)

    @classmethod
    def from_values(cls, values: Sequence[float], unit: str = "") -> "AggregateMetric":
        valid = [float(v) for v in values if v is not None and not math.isnan(v)]
        if not valid:
            return cls(min=None, max=None, avg=None, count=0, unit=unit, observations=[])
        return cls(
            min=min(valid),
            max=max(valid),
            avg=sum(valid) / len(valid),
            count=len(valid),
            unit=unit,
            observations=valid,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "min": self.min,
            "max": self.max,
            "avg": round(self.avg, 4) if self.avg is not None else None,
            "count": self.count,
            "unit": self.unit,
            "observations": self.observations,
        }

    def format_triplet(self, precision: int = 2) -> str:
        """Returns 'Min / Avg / Max' or 'Avg' or 'N/A' string."""
        if self.count == 0 or self.avg is None:
            return "N/A"
        if self.min is not None and self.max is not None and (self.min != self.max):
            return f"{self.min:.{precision}f} / {self.avg:.{precision}f} / {self.max:.{precision}f}"
        return f"{self.avg:.{precision}f}"


class MetricAggregator:
    """Collects individual metric observations and computes min, max, average statistics."""

    def __init__(self) -> None:
        self._series: Dict[str, List[float]] = {}
        self._units: Dict[str, str] = {}
        self._scalars: Dict[str, Any] = {}

    def add_observation(self, name: str, value: Any, unit: str = "") -> None:
        if value is None:
            return
        if isinstance(value, (int, float)):
            fval = float(value)
            if math.isnan(fval):
                return
            if name not in self._series:
                self._series[name] = []
                self._units[name] = unit
            self._series[name].append(fval)
        else:
            self._scalars[name] = value

    def add_scalar(self, name: str, value: Any) -> None:
        if value is not None:
            self._scalars[name] = value

    def get_aggregate(self, name: str) -> Optional[AggregateMetric]:
        if name in self._series:
            return AggregateMetric.from_values(self._series[name], self._units.get(name, ""))
        return None

    def get_summary(self) -> Dict[str, Any]:
        summary: Dict[str, Any] = {}
        for name in self._series:
            agg = self.get_aggregate(name)
            if agg:
                summary[name] = agg.to_dict()
        for name, val in self._scalars.items():
            summary[name] = val
        return summary

