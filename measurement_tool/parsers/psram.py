from __future__ import annotations

import re
from typing import Any, Dict


def _num(raw: str):
    try:
        return float(raw)
    except ValueError:
        return None


def parse_psram_metrics(log_text: str) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {}

    patterns = {
        "capacity": re.compile(r"PSRAM\s+(?:Physical\s+)?Capacity\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:mb|mib|kb|kib|b)?", re.I),
        "allocation": re.compile(r"PSRAM\s+(?:Audio\s+)?Allocation\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:mb|mib|kb|kib|b)?", re.I),
        "reserved": re.compile(r"PSRAM\s+Reserved\s+Capacity\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:mb|mib|kb|kib|b)?", re.I),
        "slot_count": re.compile(r"PSRAM\s+Slot\s+Count\s*[:=]\s*(\d+)", re.I),
        "slot_size_kb": re.compile(r"PSRAM\s+Slot\s+Size\s*[:=]\s*(\d+)\s*KB", re.I),
        "occupancy": re.compile(r"PSRAM\s+(?:Current\s+)?Occupancy\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:%|kb|kib|b)?", re.I),
        "high_water_mark": re.compile(r"(?:PSRAM\s+)?High-Water\s+Mark\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:mb|mib|kb|kib|b|%)?", re.I),
        "free_slots": re.compile(r"PSRAM\s+Free\s+Slots\s*[:=]\s*(\d+)", re.I),
        "rejected_audio_blocks": re.compile(r"(?:PSRAM\s+)?Rejected\s+(?:Audio\s+)?Blocks\s*[:=]\s*(\d+)", re.I),
        "full_events": re.compile(r"(?:PSRAM\s+)?Full\s+Events\s*[:=]\s*(\d+)", re.I),
        "qspi_txn_count": re.compile(r"PSRAM\s+QSPI\s+Transaction\s+Count\s*[:=]\s*(\d+)", re.I),
        "psram_read_throughput_kb_s": re.compile(r"read\s+throughput\s*[:=]?\s*(\d+)\s*KB/s", re.I),
    }

    for key, pattern in patterns.items():
        match = pattern.search(log_text)
        if not match:
            continue
        value = match.group(1)
        if key in {"rejected_audio_blocks", "full_events", "slot_count", "free_slots", "qspi_txn_count"}:
            metrics[key] = int(value)
        else:
            val = _num(value)
            if val is not None:
                metrics[key] = val

    return metrics
