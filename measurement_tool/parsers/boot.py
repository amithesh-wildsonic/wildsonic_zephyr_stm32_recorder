from __future__ import annotations

import re
from typing import Any, Dict


def parse_boot_metrics(log_text: str) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {}

    # Check for banner milestones
    if "System initialization complete" in log_text:
        metrics["system_init"] = "PASS"
    if "SD card initialized successfully" in log_text or "SD mounted:" in log_text:
        metrics["sd_init"] = "PASS"
    if "Audio peripheral initialized" in log_text or "ADC/DMA configured" in log_text:
        metrics["audio_init"] = "PASS"
    if "RTC Wakeup Timer initialized" in log_text or "RTC calendar online" in log_text:
        metrics["rtc_init"] = "PASS"
    if "PSRAM initialized" in log_text or "IS66WVS4M8 Detected" in log_text:
        metrics["psram_init"] = "PASS"

    # Reset acknowledgement
    if "Rebooting device" in log_text:
        metrics["reset_ack"] = "PASS"

    # Firmware reboot errors / rejection
    err_match = re.search(r"(RBT:\s*Recorder shutdown timeout[^\r\n]*|aborting reboot[^\r\n]*)", log_text, re.I)
    if err_match:
        metrics["reboot_error"] = err_match.group(0).strip()

    # Firmware version and commit
    ver_match = re.search(r"(?:Firmware\s+Version|Version)\s*:\s*([^\r\n]+)", log_text, re.I)
    if ver_match:
        metrics["firmware_version"] = ver_match.group(1).strip()

    commit_match = re.search(r"(?:Git commit|commit):\s*([0-9a-fA-F]{7,40})", log_text, re.I)
    if commit_match:
        metrics["firmware_commit"] = commit_match.group(1).strip()

    build_date = re.search(r"Build Date:\s*([^\r\n]+)", log_text, re.I)
    if build_date:
        metrics["build_date"] = build_date.group(1).strip()

    # Boot duration if reported
    boot_time = re.search(r"(?:boot\s+duration|Zephyr\s+startup\s+time)\s*[:=]\s*([0-9]*\.?[0-9]+)\s*ms", log_text, re.I)
    if boot_time:
        metrics["boot_duration_ms"] = float(boot_time.group(1))

    # DEVINFO fields
    devinfo_fields = {
        "hardware_platform": r"Hardware\s+Platform\s*:\s*([^\r\n]+)",
        "core_clock": r"Core\s+Clock\s+Frequency\s*:\s*([^\r\n]+)",
        "audio_acquisition": r"Audio\s+Acquisition\s*:\s*([^\r\n]+)",
        "external_memory": r"External\s+Memory\s*:\s*([^\r\n]+)",
        "storage_media": r"Storage\s+Media\s*:\s*([^\r\n]+)",
        "sram_architecture": r"SRAM\s+Architecture\s*:\s*([^\r\n]+)",
        "flash_memory": r"Flash\s+Memory\s*:\s*([^\r\n]+)",
        "power_management": r"Power\s+Management\s*:\s*([^\r\n]+)",
    }
    for key, pattern in devinfo_fields.items():
        m = re.search(pattern, log_text, re.I)
        if m:
            metrics[key] = m.group(1).strip()

    return metrics
