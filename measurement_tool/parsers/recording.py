from __future__ import annotations

import re
from typing import Any, Dict


def _coerce_numeric(raw: str):
    text = raw.strip().rstrip("%")
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def parse_recording_metrics(log_text: str) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {}

    # Check for WAV verification in various forms
    if re.search(r"\[TEST\]\s+WAV_VERIFIED", log_text, re.I):
        metrics["wav_verification"] = "PASS"
    elif re.search(r"WAV\s+verification\s*[:=]\s*(pass|fail|ok|error|success)", log_text, re.I):
        m = re.search(r"WAV\s+verification\s*[:=]\s*(pass|fail|ok|error|success)", log_text, re.I)
        metrics["wav_verification"] = m.group(1).upper()
    elif re.search(r"Post-recording file verification failed", log_text, re.I):
        metrics["wav_verification"] = "FAIL"

    # 1. First, check if structured [TEST] RECORDING ACCOUNTING SUMMARY is present
    # We find all accounting summaries and use the latest one
    acct_blocks = list(re.finditer(r"\[TEST\]\s+RECORDING ACCOUNTING SUMMARY:(.*?)(?=\n\s*\n|The total number|\[SCHEDULER\]|\Z)", log_text, re.DOTALL | re.IGNORECASE))
    if acct_blocks:
        last_block = acct_blocks[-1].group(1)

        m_exp = re.search(r"Expected\s+(?:source\s+)?samples\s*:\s*(\d+)", last_block, re.I)
        if m_exp:
            metrics["expected_samples"] = float(m_exp.group(1))

        m_acq = re.search(r"Acquired\s+(?:source\s+)?samples\s*:\s*(\d+)", last_block, re.I)
        if m_acq:
            metrics["acquired_samples"] = float(m_acq.group(1))

        m_proc = re.search(r"(?:Successfully\s+)?processed\s+samples\s*:\s*(\d+)", last_block, re.I)
        if m_proc:
            metrics["processed_samples"] = float(m_proc.group(1))

        m_wrt = re.search(r"Samples\s+written\s*:\s*(\d+)", last_block, re.I)
        if m_wrt:
            metrics["samples_written"] = float(m_wrt.group(1))

        m_lost = re.search(r"(?:Samples\s+lost(?:\s+during\s+active\s+capture)?|Lost\s+samples)\s*:\s*(\d+)", last_block, re.I)
        if m_lost:
            metrics["lost_samples"] = float(m_lost.group(1))

        m_bnd = re.search(r"Boundary\s+samples\s+discarded\s*:\s*(\d+)", last_block, re.I)
        if m_bnd:
            val = float(m_bnd.group(1))
            metrics["boundary_samples_discarded"] = val
            metrics["boundary_discarded_samples"] = val

    # 2. General fallback regex patterns for logs without structured accounting block
    # or fields not covered by the block
    patterns = {
        "expected_samples": re.compile(r"expected\s+(?:source\s+)?samples\s*[:=]\s*(\d+)", re.I),
        "acquired_samples": re.compile(r"acquired\s+(?:source\s+)?samples\s*[:=]\s*(\d+)", re.I),
        "processed_samples": re.compile(r"(?:successfully\s+)?processed\s+samples\s*[:=]\s*(\d+)", re.I),
        # Do not match 'Samples Written : 0 / 1920000' from active progress lines
        "samples_written": re.compile(r"samples\s+written\s*[:=]\s*(\d+)(?!\s*/)", re.I),
        "lost_samples": re.compile(r"(?:samples\s+lost(?:\s+during\s+active\s+capture)?|lost\s+samples)\s*[:=]\s*(\d+)", re.I),
        "uncommitted_samples": re.compile(r"uncommitted\s+samples\s*[:=]\s*(\d+)", re.I),
        "boundary_samples_discarded": re.compile(r"boundary\s+samples\s+discarded\s*[:=]\s*(\d+)", re.I),
        "recording_duration": re.compile(r"recording\s+duration\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:s|sec|seconds)?", re.I),
        "session_duration_s": re.compile(r"duration\s*[:=]\s*(\d+)\s*s", re.I),
    }

    for key, pattern in patterns.items():
        if key in metrics:
            continue
        matches = list(pattern.finditer(log_text))
        if not matches:
            continue
        value = matches[-1].group(1)
        val = _coerce_numeric(value)
        if val is not None:
            metrics[key] = val

    # Ensure aliases are symmetric
    if "boundary_samples_discarded" in metrics and "boundary_discarded_samples" not in metrics:
        metrics["boundary_discarded_samples"] = metrics["boundary_samples_discarded"]
    elif "boundary_discarded_samples" in metrics and "boundary_samples_discarded" not in metrics:
        metrics["boundary_samples_discarded"] = metrics["boundary_discarded_samples"]

    # If recording_duration is missing but session_duration_s exists, use it
    if "recording_duration" not in metrics and "session_duration_s" in metrics:
        metrics["recording_duration"] = metrics["session_duration_s"]

    # Check for session error output
    err_match = re.search(r"\[ERROR\]\s+Recording session failed:\s*(-?\d+)", log_text)
    if err_match:
        metrics["session_error"] = int(err_match.group(1))

    # Pipeline error diagnostics
    diag_match = re.search(r"Pipeline:\s*dma_err=(\d+),\s*adc_ovr=(\d+),\s*dsp_ovr=(\d+),\s*psram_full=(\d+),\s*sd_err=(-?\d+)", log_text)
    if diag_match:
        metrics["dma_errors"] = int(diag_match.group(1))
        metrics["adc_overruns"] = int(diag_match.group(2))
        metrics["dsp_overruns"] = int(diag_match.group(3))
        metrics["psram_full_events"] = int(diag_match.group(4))
        metrics["sd_write_error"] = int(diag_match.group(5))

    # Validate accounting reconciliation if sample counts are present
    acq = metrics.get("acquired_samples")
    proc = metrics.get("processed_samples")
    wrt = metrics.get("samples_written")
    bnd = metrics.get("boundary_samples_discarded", 0.0)
    lost = metrics.get("lost_samples", 0.0)

    if acq is not None and proc is not None and wrt is not None:
        # Accounting relationship:
        # acquired == processed + lost (or acquired == processed when lost == 0)
        # processed == written + boundary_discarded
        proc_reconciled = (proc + lost == acq) or (proc == acq)
        storage_reconciled = (wrt + bnd == proc)
        metrics["accounting_reconciled"] = bool(proc_reconciled and storage_reconciled)

    return {k: v for k, v in metrics.items() if v is not None}
