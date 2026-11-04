from __future__ import annotations

import re
from typing import Any, Dict, List


def _num(raw: str):
    try:
        return float(raw)
    except ValueError:
        return None


def verify_stop2_evidence(log_text: str) -> Dict[str, Any]:
    """Evaluates strict multi-stage evidence for Stop 2 deep sleep entry and wake.

    Requires explicit evidence of:
    1. Sleep preparation.
    2. Actual Stop2 entry.
    3. RTC wake event.
    4. MCU execution resumed after wake.
    """
    evidence: Dict[str, Any] = {
        "sleep_prep_detected": False,
        "stop2_entry_detected": False,
        "rtc_wake_detected": False,
        "execution_resumed_detected": False,
        "missing_evidence": [],
    }

    # 1. Sleep preparation markers
    if re.search(
        r"(?:\[TEST\]\s+SLEEP_INTERVAL_STARTED|\[TEST\]\s+RTC_WUT_ARMED|Entering STM32L476 Stop 2|entering Stop2 standby|Standby wait:|sleep prep(?:aration)? duration)",
        log_text,
        re.I,
    ):
        evidence["sleep_prep_detected"] = True
    else:
        evidence["missing_evidence"].append("Sleep preparation milestone not detected")

    # Check for explicit sleep rejection
    rejected_match = re.search(r"recorder_sleep\s+rejected(?:[:\s]+([^\n]+))?", log_text, re.I)
    if rejected_match:
        evidence["rejected"] = True
        evidence["rejection_reason"] = rejected_match.group(1).strip() if rejected_match.group(1) else "Controller not idle"
        evidence["missing_evidence"].append(f"Sleep rejected: {evidence['rejection_reason']}")

    # 2. Actual Stop2 entry markers
    if re.search(r"(?:\[TEST\]\s+STOP2_ENTERED|stop2\s+entry\s*[:=]\s*pass|time\s+to\s+stop\s*2)", log_text, re.I):
        evidence["stop2_entry_detected"] = True
    else:
        evidence["missing_evidence"].append("Actual Stop 2 entry marker (STOP2_ENTERED) not detected")

    # 3. RTC wake event markers
    if re.search(r"(?:\[TEST\]\s+RTC_WAKE_ISR|\[TEST\]\s+STOP2_EXITED|RTC wake received|wake[- ]up source\s*[:=]\s*RTC)", log_text, re.I):
        evidence["rtc_wake_detected"] = True
    else:
        evidence["missing_evidence"].append("RTC wake event marker (RTC_WAKE_ISR / RTC wake received) not detected")

    # 4. MCU execution resumed markers
    if re.search(
        r"(?:\[TEST\]\s+CLOCKS_RESTORED|\[TEST\]\s+UART_RESTORED|Wake initialization complete|scheduler continuing|\[TEST\]\s+SCHEDULER_RESUMED|\[TEST\]\s+RTC_CALENDAR_ADVANCED)",
        log_text,
        re.I,
    ):
        evidence["execution_resumed_detected"] = True
    else:
        evidence["missing_evidence"].append("Post-wake clock/UART restoration milestone not detected")

    is_all_verified = (
        evidence["sleep_prep_detected"]
        and evidence["stop2_entry_detected"]
        and evidence["rtc_wake_detected"]
        and evidence["execution_resumed_detected"]
        and not evidence.get("rejected", False)
    )

    evidence["is_verified"] = is_all_verified
    return evidence


def parse_sleep_metrics(log_text: str) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {}

    evidence = verify_stop2_evidence(log_text)
    metrics["stop2_verified"] = evidence["is_verified"]
    metrics["stop2_missing_evidence"] = evidence["missing_evidence"]

    if evidence["is_verified"]:
        metrics["stop2_verification"] = "PASS"
        metrics["stop2_entry"] = "PASS"
        metrics["rtc_wake_result"] = "PASS"
    elif evidence.get("rejected"):
        metrics["stop2_verification"] = "FAIL"
        metrics["stop2_entry"] = "FAIL"
        metrics["rtc_wake_result"] = "FAIL"
    else:
        metrics["stop2_verification"] = "UNVERIFIED"
        metrics["stop2_entry"] = "UNVERIFIED"
        metrics["rtc_wake_result"] = "UNVERIFIED"

    # Wake-up source if present
    if re.search(r"RTC\s+wake\s+received", log_text, re.I) or evidence["rtc_wake_detected"]:
        metrics["wake_up_source"] = "RTC"
    elif re.search(r"wake[- ]up\s+source\s*[:=]\s*([A-Za-z0-9_ -]+)", log_text, re.I):
        m = re.search(r"wake[- ]up\s+source\s*[:=]\s*([A-Za-z0-9_ -]+)", log_text, re.I)
        metrics["wake_up_source"] = m.group(1).strip()

    # RTC Wakeup Timer armed duration
    wut_match = re.search(r"\[TEST\]\s+RTC_WUT_ARMED:\s*duration=(\d+)\s*s", log_text, re.I)
    if wut_match:
        metrics["sleep_duration"] = float(wut_match.group(1))

    patterns = {
        "sleep_duration": re.compile(r"sleep\s+duration\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:s|sec|seconds|ms)?", re.I),
        "wake_latency": re.compile(r"wake\s+latency\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:ms|us|s)?", re.I),
        "boot_duration": re.compile(r"boot\s+duration\s*[:=]\s*([0-9]*\.?[0-9]+)\s*(?:ms|s)?", re.I),
        "sleep_prep_duration_ms": re.compile(r"sleep\s+prep(?:aration)?\s+duration\s*[:=]\s*([0-9]*\.?[0-9]+)\s*ms", re.I),
        "time_to_stop2_ms": re.compile(r"time\s+to\s+stop\s*2\s*[:=]\s*([0-9]*\.?[0-9]+)\s*ms", re.I),
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

    return metrics
