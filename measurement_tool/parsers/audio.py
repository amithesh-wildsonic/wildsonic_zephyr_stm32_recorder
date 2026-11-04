from __future__ import annotations

import re
from typing import Any, Dict


def _num(value: str):
    try:
        return float(value)
    except ValueError:
        return None


def parse_audio_metrics(log_text: str) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {}

    # Check for compound DSP line: "DSP average / worst time : 120 / 240 us"
    compound_dsp = re.search(r"DSP\s+average\s*/\s*worst\s+time\s*[:=]\s*(\d+)\s*/\s*(\d+)\s*us", log_text, re.I)
    if compound_dsp:
        metrics["avg_dsp_execution_time"] = float(compound_dsp.group(1))
        metrics["worst_dsp_execution_time"] = float(compound_dsp.group(2))

    patterns = {
        "sampling_rate": re.compile(r"sampling\s+(?:rate|frequency)(?:\s+set\s+to)?\s*[:=]?\s*([0-9]*\.?[0-9]+)\s*(?:hz|khz|mhz)?", re.I),
        "adc_overruns": re.compile(r"adc\s+overruns\s*[:=]\s*(\d+)", re.I),
        "dma_errors": re.compile(r"dma\s+errors\s*[:=]\s*(\d+)", re.I),
        "dsp_overruns": re.compile(r"dsp\s+overruns\s*[:=]\s*(\d+)", re.I),
        "avg_dsp_execution_time": re.compile(r"average\s+dsp\s+execution\s+time\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:us|ms|s)?", re.I),
        "worst_dsp_execution_time": re.compile(r"worst\s+dsp\s+execution\s+time\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:us|ms|s)?", re.I),
        "block_deadline": re.compile(r"(?:block\s+deadline|DSP\s+block\s+deadline)\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:us|ms|s)?", re.I),
        "dma_max_latency_cycles": re.compile(r"DMA\s+worker\s+max\s+latency\s*[:=]\s*(\d+)\s*cyc", re.I),
        "filled_queue_high_water": re.compile(r"Filled\s+queue\s+high-water\s*[:=]\s*(\d+)", re.I),
    }

    for key, pattern in patterns.items():
        if key in metrics:
            continue
        match = pattern.search(log_text)
        if not match:
            continue
        value = match.group(1)
        val = _num(value)
        if val is not None:
            metrics[key] = val

    return {k: v for k, v in metrics.items() if v is not None}
