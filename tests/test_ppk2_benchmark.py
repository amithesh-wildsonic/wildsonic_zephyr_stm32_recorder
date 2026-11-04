from __future__ import annotations

import time
from pathlib import Path
import openpyxl
import pytest

from measurement_tool.controller import MockSerialPort, SerialController
from measurement_tool.ppk2_controller import MockPPK2Device, PPK2Controller
from measurement_tool.test_runner import BenchmarkRunner, create_mock_stm32_serial
from measurement_tool.workbook import ComparisonWorkbookManager


def test_dual_connection_uart_and_ppk2(tmp_path):
    storage = tmp_path / "tests.json"
    mock_uart = create_mock_stm32_serial()
    controller = SerialController(port="COM30", output_dir=tmp_path / "logs", mock_port=mock_uart)
    ppk2 = PPK2Controller(port="COM31", mock=True)

    runner = BenchmarkRunner(controller=controller, ppk2=ppk2, storage_path=storage)
    assert runner.controller.port == "COM30"
    assert runner.ppk2.port == "COM31"

    # Collision test: PPK2 and UART on same port must raise ValueError
    ppk2_colliding = PPK2Controller(port="COM30", mock=False)
    with pytest.raises(ValueError, match="collides with STM32 UART port"):
        ppk2_colliding.connect(port="COM30", exclude_uart_port="COM30")


def test_power_cycle_boot_sequence_without_banner_detection(tmp_path):
    """Verifies that boot readiness is established by valid DEVINFO communication,

    without requiring 'System initialization complete' or any specific Zephyr banner.
    """
    storage = tmp_path / "tests.json"

    # Mock serial with NO boot banner at all
    responses = {
        r"(?:DEVINFO|GET_DEV_INFO)": (
            "\r\n=== DEVICE INFORMATION ===\r\n"
            "  Hardware Platform     : STM32L476RG Nucleo-64 (ARM Cortex-M4F)\r\n"
            "  Core Clock Frequency  : 72 MHz (PLL HSI)\r\n"
            "  Firmware Version      : 2.1.0-prod (Zephyr RTOS)\r\n"
            "OK\r\n> "
        ),
        r"(?:STATUS|GET_STATUS)": (
            "\r\n=== SYSTEM & RECORDING STATUS ===\r\n"
            "  Recording State       : IDLE\r\n"
            "  Scheduler Status      : RUNNING\r\n"
            "OK\r\n> "
        ),
    }
    mock_uart = MockSerialPort(responses)
    # Output does NOT contain "System initialization complete"
    mock_uart.feed_input("Random UART noise after power-up...\r\n> ")

    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock_uart)
    ppk2 = PPK2Controller(port="MOCK_PPK2", mock=True)
    runner = BenchmarkRunner(controller=controller, ppk2=ppk2, storage_path=storage)

    session = runner.run_ppk2_power_cycle_boot(
        test_id="STM32-PPK2-BOOT-TEST",
        voltage_mv=5000,
        boot_stabilize_s=0.1,
        devinfo_timeout_s=3.0,
        evidence_type="SAMPLE",
    )

    assert session.category == "boot"
    assert session.parsed_result["boot_verified"] is True
    assert session.parsed_result["reset_type"] == "PPK2_POWER_CYCLE"
    assert session.parsed_result.get("host_observed_readiness_ms", 0.0) > 0.0
    # Banner was not required
    assert "System initialization complete" not in controller.all_text


def test_devinfo_readiness_retry_timeout(tmp_path):
    """Verifies bounded timeout failure when MCU does not respond to DEVINFO."""
    storage = tmp_path / "tests.json"
    responses = {
        # Empty responses - MCU unresponsive
    }
    mock_uart = MockSerialPort(responses)
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock_uart)
    ppk2 = PPK2Controller(port="MOCK_PPK2", mock=True)
    runner = BenchmarkRunner(controller=controller, ppk2=ppk2, storage_path=storage)

    session = runner.run_ppk2_power_cycle_boot(
        test_id="STM32-PPK2-TIMEOUT",
        voltage_mv=5000,
        boot_stabilize_s=0.1,
        devinfo_timeout_s=0.5,
        evidence_type="HARDWARE",
    )

    assert session.status == "fail"
    assert session.parsed_result["boot_verified"] is False
    assert "timeout" in session.parsed_result["boot_error"]


def test_stop2_with_ppk2_output_continuously_enabled(tmp_path):
    """Verifies that PPK2 power output remains ON throughout Stop 2 sleep measurement."""
    storage = tmp_path / "tests.json"
    mock_uart = create_mock_stm32_serial()
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock_uart)
    ppk2 = PPK2Controller(port="MOCK_PPK2", mock=True)
    runner = BenchmarkRunner(controller=controller, ppk2=ppk2, storage_path=storage)

    session = runner.run_ppk2_full_benchmark(
        base_id="STM32-TEST-STOP2",
        record_duration_s=1,
        sleep_seconds=1,
        idle_duration_s=0.1,
        boot_stabilize_s=0.1,
        devinfo_timeout_s=3.0,
        evidence_type="SAMPLE",
    )

    # During Stop 2 (Phase F), power must be continuously enabled
    stop2_phase = ppk2.phases.get("Phase F: Stop2 Sleep")
    assert stop2_phase is not None
    assert stop2_phase.duration_s >= 0.9
    # Power state must have remained ON throughout
    assert ppk2.power_state == "ON" or ppk2._device.power_state == "ON"


def test_ppk2_full_9_phase_benchmark_execution(tmp_path):
    storage = tmp_path / "tests.json"
    mock_uart = create_mock_stm32_serial()
    controller = SerialController(port="MOCK", output_dir=tmp_path / "logs", mock_port=mock_uart)
    ppk2 = PPK2Controller(port="MOCK_PPK2", mock=True)
    runner = BenchmarkRunner(controller=controller, ppk2=ppk2, storage_path=storage)

    session = runner.run_ppk2_full_benchmark(
        base_id="STM32-PPK2-9PHASE",
        voltage_mv=5000,
        sample_rate=192000,
        record_duration_s=1,
        sleep_seconds=1,
        idle_duration_s=0.2,
        boot_stabilize_s=0.1,
        devinfo_timeout_s=3.0,
        evidence_type="SAMPLE",
    )

    # Verify all 9 phases exist
    expected_phases = [
        "Phase A: Cold Boot",
        "Phase B: Active Idle",
        "Phase C: Recording",
        "Phase D: SD Write",
        "Phase E: SD Read",
        "Phase F: Stop2 Sleep",
        "Phase G: RTC Wakeup",
        "Phase H: Peripheral Recovery",
        "Phase I: Storage Verification",
    ]
    for p in expected_phases:
        assert p in ppk2.phases, f"Phase {p} missing from PPK2 benchmark telemetry"
        assert ppk2.phases[p].duration_s > 0.0

    # Verify master CSV was exported
    assert session.ppk2_csv_path
    assert Path(session.ppk2_csv_path).exists()

    # Verify individual phase CSVs were exported
    stamp = Path(session.ppk2_csv_path).name.split("_ppk2_full")[0].split("STM32-PPK2-9PHASE_")[1]
    boot_phase_csv = Path(f"data/ppk2/STM32-PPK2-9PHASE_{stamp}_phase_a__cold_boot.csv")
    assert boot_phase_csv.exists()

    # Verify comparison workbook updated
    wb = openpyxl.load_workbook("data/reports/stm32_vs_max78000.xlsx")
    ws_pwr = wb["Power & Sleep"]
    found_recording_pwr = False
    for row in ws_pwr.iter_rows(values_only=True):
        if row and "Active Recording" in str(row[0]):
            found_recording_pwr = True
            assert row[4] is not None  # STM32 Avg (mA) was updated
    assert found_recording_pwr

