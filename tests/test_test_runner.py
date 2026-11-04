from __future__ import annotations

import json
from pathlib import Path
import pytest

from measurement_tool.config import AcceptanceCriteria
from measurement_tool.controller import MockSerialPort, SerialController
from measurement_tool.test_runner import BenchmarkRunner, _clean_terminal_summary, create_mock_stm32_serial


def test_runner_boot_test(tmp_path):
    storage = tmp_path / "tests.json"
    mock = create_mock_stm32_serial()
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_boot_test(test_id="UNIT-BOOT-01", evidence_type="SAMPLE")
    assert session.test_id == "UNIT-BOOT-01"
    assert session.category == "boot"
    assert session.evidence_type == "SAMPLE"
    assert session.status == "pending"
    assert session.parsed_result.get("boot_duration_ms") == 1200.0 or session.parsed_result.get("system_init") == "PASS"
    assert session.parsed_result.get("boot_verified") is True
    assert session.parsed_result.get("reset_ack") == "PASS"


def test_runner_boot_test_rbt_shutdown_timeout_fails(tmp_path):
    storage = tmp_path / "tests.json"
    responses = {
        r"STATUS": "\r\n  Recording State : IDLE\r\nOK\r\n> ",
        r"RBT": "\r\nRBT: Recorder shutdown timeout (-116); aborting reboot to preserve data\r\n> ",
        r"DEVINFO": "OK\r\n> ",
    }
    mock = MockSerialPort(responses)
    mock.feed_input("Interactive console ready.\r\n> ")
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_boot_test(test_id="STM32-HW-BOOT-001", evidence_type="HARDWARE")
    assert session.status == "fail"
    assert session.parsed_result["boot_verified"] is False
    assert "Recorder shutdown timeout (-116)" in session.parsed_result["reboot_error"]


def test_runner_boot_test_missing_banner_timeout_fails(tmp_path):
    storage = tmp_path / "tests.json"
    responses = {
        r"STATUS": "\r\n  Recording State : IDLE\r\nOK\r\n> ",
        r"RBT": "\r\nRebooting device.\r\n",
    }
    mock = MockSerialPort(responses)
    mock.feed_input("Interactive console ready.\r\n> ")
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_boot_test(test_id="STM32-HW-BOOT-TIMEOUT", evidence_type="HARDWARE", timeout_s=0.5)
    assert session.status == "fail"
    assert session.parsed_result["boot_verified"] is False
    assert "Boot banner timed out" in session.parsed_result["boot_error"]


def test_runner_boot_test_refuses_reboot_during_active_recording(tmp_path):
    storage = tmp_path / "tests.json"
    responses = {
        r"STATUS": (
            "\r\n=== SYSTEM & RECORDING STATUS ===\r\n"
            "  Recording State       : ACTIVE\r\n"
            "  Scheduler Status      : RUNNING\r\n"
            "OK\r\n> "
        ),
        r"RBT": "\r\nRebooting device.\r\n",
    }
    mock = MockSerialPort(responses)
    mock.feed_input("Interactive console ready.\r\n> ")
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_boot_test(test_id="STM32-HW-BOOT-ACTIVE", evidence_type="HARDWARE")
    assert session.status == "blocked"
    assert session.parsed_result["blocked"] is True
    assert "ACTIVE" in session.parsed_result["blocked_reason"]
    assert session.parsed_result["boot_verified"] is False
    # Verify RBT command was never sent to device
    assert "RBT" not in mock.sent_commands


def test_runner_cold_boot_mode(tmp_path):
    storage = tmp_path / "tests.json"
    responses = {
        r"DEVINFO": (
            "\r\n=== DEVICE INFORMATION ===\r\n"
            "  Hardware Platform     : STM32L476RG Nucleo-64 (ARM Cortex-M4F)\r\n"
            "  Core Clock Frequency  : 72 MHz (PLL HSI)\r\n"
            "  Firmware Version      : 2.1.0-prod (Zephyr RTOS)\r\n"
            "OK\r\n> "
        ),
    }
    mock = MockSerialPort(responses)
    # Simulate hardware reset output directly on UART
    mock.feed_input(
        "\r\nSystem initialization complete\r\n"
        "RTC Wakeup Timer initialized\r\n"
        "RTC calendar online\r\n"
        "Audio peripheral initialized\r\n"
        "SD card initialized successfully\r\n"
        "Version: v1.2.0\r\n"
        "Git commit: 6b7a624\r\n"
        "Build Date: Nov  3 2026 12:00:00\r\n"
        "boot duration : 1150 ms\r\n"
        "Interactive console ready.\r\n> "
    )
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_cold_boot_test(test_id="STM32-COLD-01", evidence_type="HARDWARE", timeout_s=2.0)
    assert session.category == "cold-boot"
    assert session.status == "pass"
    assert session.parsed_result["boot_verified"] is True
    assert session.parsed_result["reset_type"] == "COLD_BOOT"
    assert "RBT" not in mock.sent_commands
    assert session.configuration == "Cold Boot (NRST / Power Cycle) & Peripheral Init"


def test_boot_summary_does_not_leak_recording_metrics(tmp_path):
    storage = tmp_path / "tests.json"
    mock = create_mock_stm32_serial()
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_boot_test(test_id="STM32-BOOT-LEAK-CHECK", evidence_type="SAMPLE")
    metrics = session.parsed_result

    # None of the recording / SD throughput metrics should exist in the boot session result
    forbidden_keys = [
        "acquired_samples",
        "samples_written",
        "lost_samples",
        "wav_verification",
        "active_write_throughput",
        "aggregate_writer_throughput",
    ]
    for key in forbidden_keys:
        assert key not in metrics, f"Recording metric {key} leaked into boot session parsed_result"

    # Terminal summary output must not contain recording metric labels
    summary_text = _clean_terminal_summary(session, metrics)
    assert "SOFTWARE REBOOT SUMMARY" in summary_text
    assert "Acquired Samples" not in summary_text
    assert "Written Samples" not in summary_text
    assert "SD Active Throughput" not in summary_text


def test_accounting_validation_catches_unacquired_sample_deficit():
    criteria = AcceptanceCriteria()
    # Actual hardware failure case:
    # Expected 2,880,000; Acquired 2,129,920; Processed 2,129,920; Written 2,129,920; Lost 0
    metrics = {
        "expected_samples": 2880000,
        "acquired_samples": 2129920,
        "processed_samples": 2129920,
        "samples_written": 2129920,
        "lost_samples": 0,
        "wav_verification": "PASS",
        "storage_recovery": "PASS",
        "stop2_entry": "PASS",
        "boot_verified": True,
    }
    warnings = criteria.evaluate(metrics)
    assert any("Sample shortfall" in w for w in warnings)
    assert any("750080 expected samples not acquired" in w for w in warnings)

    runner = BenchmarkRunner(controller=None)  # type: ignore
    status, _ = runner._determine_status("HARDWARE", metrics)
    assert status == "fail"


def test_runner_recording_test(tmp_path):
    storage = tmp_path / "tests.json"
    mock = create_mock_stm32_serial()
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_recording_test(
        test_id="UNIT-REC-01",
        sample_rate=192000,
        duration_s=10,
        evidence_type="SAMPLE",
        ppk2_csv="data/ppk2/ppk2_sample.csv",
    )
    assert session.test_id == "UNIT-REC-01"
    assert session.category == "recording"
    assert session.evidence_type == "SAMPLE"
    assert session.status == "pending"
    metrics = session.parsed_result
    assert metrics["acquired_samples"] == 1920000.0
    assert metrics["samples_written"] == 1920000.0
    assert metrics["lost_samples"] == 0.0
    assert metrics["wav_verification"] == "PASS"
    assert metrics["active_write_throughput"] == 2150.0
    assert metrics["aggregate_writer_throughput"] == 1950.0
    assert session.sample_data != {}


def test_runner_sd_performance_test(tmp_path):
    storage = tmp_path / "tests.json"
    mock = create_mock_stm32_serial()
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_sd_performance_test(test_id="UNIT-SD-01", evidence_type="SAMPLE")
    assert session.test_id == "UNIT-SD-01"
    assert session.category == "sd-performance"
    assert "sdbench_results" in session.parsed_result
    assert len(session.parsed_result["sdbench_results"]) >= 4


def test_runner_sleep_wake_test(tmp_path):
    storage = tmp_path / "tests.json"
    mock = create_mock_stm32_serial()
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    session = runner.run_sleep_wake_test(test_id="UNIT-SLEEP-01", sleep_seconds=5, evidence_type="SAMPLE")
    assert session.test_id == "UNIT-SLEEP-01"
    assert session.category == "sleep-wake"
    assert session.parsed_result.get("stop2_entry") == "PASS"
    assert session.parsed_result.get("wake_up_source") == "RTC"


def test_runner_full_benchmark_sequence(tmp_path):
    storage = tmp_path / "tests.json"
    mock = create_mock_stm32_serial()
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock)
    runner = BenchmarkRunner(controller=controller, storage_path=storage)

    sessions = runner.run_full_benchmark(
        base_id="UNIT-BENCH",
        evidence_type="SAMPLE",
        sample_rate=192000,
        record_duration_s=10,
        ppk2_csv="data/ppk2/ppk2_sample.csv",
    )
    assert len(sessions) == 5
    categories = [s.category for s in sessions]
    assert categories == ["boot", "idle-power", "recording", "sd-performance", "sleep-wake"]
    for s in sessions:
        assert s.status == "pending"  # SAMPLE evidence is always pending
