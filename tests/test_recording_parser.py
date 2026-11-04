from pathlib import Path

from measurement_tool.parsers import parse_firmware_log


def test_recording_parser_extracts_firmware_metrics():
    log_path = Path("data/stm32/raw_logs/stm32_sample_capture.log")
    text = log_path.read_text(encoding="utf-8")

    metrics = parse_firmware_log(text)

    assert metrics["expected_samples"] == 120000.0
    assert metrics["lost_samples"] == 0.0
    assert metrics["wav_verification"] == "PASS"
    assert metrics["recording_duration"] == 6.25
    assert metrics["adc_overruns"] == 0.0
    assert metrics["storage_recovery"] == "PASS"
    assert metrics["wake_up_source"] == "RTC"
    assert metrics["boot_duration"] == 1200.0
