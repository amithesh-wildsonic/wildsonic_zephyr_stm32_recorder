from __future__ import annotations

import re
from typing import Any, Dict


def parse_power_metrics(log_text: str) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {}
    patterns = {
        "average_current": re.compile(r"average\s+current\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:ua|ma|a)", re.I),
        "minimum_current": re.compile(r"minimum\s+current\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:ua|ma|a)", re.I),
        "maximum_current": re.compile(r"maximum\s+current\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:ua|ma|a)", re.I),
        "peak_current": re.compile(r"peak\s+current\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:ua|ma|a)", re.I),
        "measurement_duration": re.compile(r"measurement\s+duration\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:s|ms)", re.I),
        "energy_consumption": re.compile(r"energy\s+consumption\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:uj|mj|j|wh)", re.I),
        "voltage": re.compile(r"voltage\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:mv|v)", re.I),
    }
    for key, pattern in patterns.items():
        match = pattern.search(log_text)
        if not match:
            continue
        value = match.group(1)
        try:
            metrics[key] = float(value)
        except ValueError:
            metrics[key] = value
    return metrics
