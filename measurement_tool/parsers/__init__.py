from __future__ import annotations

from typing import Any, Dict

from .audio import parse_audio_metrics
from .boot import parse_boot_metrics
from .power import parse_power_metrics
from .psram import parse_psram_metrics
from .recording import parse_recording_metrics
from .sdmmc import parse_sdmmc_metrics
from .sleep import parse_sleep_metrics


def parse_firmware_log(log_text: str) -> Dict[str, Any]:
    extracted: Dict[str, Any] = {}
    for parser in (
        parse_recording_metrics,
        parse_sdmmc_metrics,
        parse_audio_metrics,
        parse_psram_metrics,
        parse_power_metrics,
        parse_sleep_metrics,
        parse_boot_metrics,
    ):
        extracted.update(parser(log_text))
    return extracted
