from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Dict, Iterable


UNIT_MULTIPLIERS = {
    "ua": 1e-6,
    "ma": 1e-3,
    "a": 1.0,
    "uv": 1e-6,
    "mv": 1e-3,
    "v": 1.0,
    "us": 1e-6,
    "ms": 1e-3,
    "s": 1.0,
    "uj": 1e-6,
    "mj": 1e-3,
    "j": 1.0,
    "wh": 3600.0,
}


def _normalize_key(value: str) -> str:
    cleaned = value.strip().lower().replace("µ", "u").replace("μ", "u")
    cleaned = "".join(ch if ch.isalnum() or ch in "_" else " " for ch in cleaned)
    cleaned = "_".join(cleaned.split())
    return cleaned


def _extract_numeric(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text or text.lower() in {"n/a", "nan", "none", "null"}:
        return None
    numeric = "".join(ch for ch in text if ch.isdigit() or ch in "-.eE+")
    if not numeric:
        return None
    try:
        return float(numeric)
    except ValueError:
        return None


def _convert_value(raw_value: Any, unit_hint: str | None = None) -> float | None:
    numeric = _extract_numeric(raw_value)
    if numeric is None:
        return None
    if unit_hint is None:
        return numeric
    unit = unit_hint.strip().lower()
    if unit in UNIT_MULTIPLIERS:
        return numeric * UNIT_MULTIPLIERS[unit]
    return numeric


def _lookup(row: Dict[str, Any], *keys: str) -> Any:
    normalized = { _normalize_key(k): v for k, v in row.items() }
    for key in keys:
        target = _normalize_key(key)
        for row_key, value in normalized.items():
            if row_key == target or row_key.startswith(target + "_") or row_key.startswith(target) or target in row_key:
                return value
    return None


def parse_ppk2_csv(csv_path: str | Path) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {}
    path = Path(csv_path)
    if not path.exists():
        raise FileNotFoundError(f"PPK2 CSV not found: {path}")

    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            return metrics
        rows = list(reader)

    if not rows:
        return metrics

    row = rows[-1]
    alias_map = {
        "average_current": ["avg current", "average current", "avg current (ua)", "mean current", "current avg"],
        "minimum_current": ["min current", "minimum current", "current min"],
        "maximum_current": ["max current", "maximum current", "current max"],
        "peak_current": ["peak current", "max peak current", "current peak"],
        "measurement_duration": ["duration", "measurement duration", "duration (s)"],
        "energy_consumption": ["energy", "energy consumption", "total energy"],
        "voltage": ["voltage", "vdd", "voltage (v)", "supply voltage"],
    }

    for metric_name, candidates in alias_map.items():
        match_value = None
        match_key = None
        for row_key, row_value in row.items():
            if row_key is None:
                continue
            normalized_row_key = _normalize_key(row_key)
            for candidate in candidates:
                normalized_candidate = _normalize_key(candidate)
                if normalized_row_key == normalized_candidate or normalized_row_key.startswith(normalized_candidate + "_") or normalized_candidate in normalized_row_key:
                    match_value = row_value
                    match_key = row_key
                    break
            if match_value is not None:
                break
        if match_value is None:
            continue

        numeric = _extract_numeric(match_value)
        if numeric is None:
            continue
        unit = ""
        normalized_key = _normalize_key(match_key or "")
        compact_key = normalized_key.replace("_", "")
        for unit_name in ("ua", "ma", "mv", "v", "us", "ms", "s", "uj", "mj", "j", "wh", "a"):
            if unit_name in compact_key:
                unit = unit_name
                break
        if unit:
            numeric *= UNIT_MULTIPLIERS.get(unit, 1.0)
        metrics[metric_name] = numeric

    return {k: v for k, v in metrics.items() if v is not None}
