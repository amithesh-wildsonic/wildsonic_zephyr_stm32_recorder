from __future__ import annotations

import json
from pathlib import Path
import pytest

from measurement_tool.controller import MockSerialPort, SerialController
from measurement_tool.ppk2_controller import PPK2Controller
from measurement_tool.stability import (
    AssessmentEngine,
    ErrorDetector,
    ErrorSeverity,
    ErrorSignatureTracker,
    ErrorSubsystem,
    StructuredError,
)
from measurement_tool.stability_runner import StabilityRunner
from measurement_tool.workbook import ComparisonWorkbookManager


class TestErrorDetectionAndNormalization:
    """Tests error detection, classification, and signature normalization across all 5 subsystems."""

    def test_sdmmc_error_patterns(self):
        crc_err = ErrorDetector.scan_line("SDMMC transfer failed: DATA_CRC_FAIL on block 0x2400", 12.5, 1, 96000)
        assert crc_err is not None
        assert crc_err.subsystem == ErrorSubsystem.SDMMC
        assert crc_err.severity == ErrorSeverity.RECOVERABLE
        assert crc_err.normalized_signature == "SDMMC_DATA_CRC_FAIL"

        timeout_err = ErrorDetector.scan_line("[ERROR] SDMMC command timeout waiting for card response", 15.0, 1, 96000)
        assert timeout_err is not None
        assert timeout_err.normalized_signature == "SDMMC_CMD_TIMEOUT"

        idle_err = ErrorDetector.scan_line("SDMMC controller not idle (-16); Stop 2 entry rejected", 20.0, 1, 96000)
        assert idle_err is not None
        assert idle_err.normalized_signature == "SDMMC_IDLE_TIMEOUT"

    def test_filesystem_error_patterns(self):
        mount_err = ErrorDetector.scan_line("f_mount failed with result FR_DISK_ERR (-1)", 5.0, 1, 96000)
        assert mount_err is not None
        assert mount_err.subsystem == ErrorSubsystem.FILESYSTEM
        assert mount_err.severity == ErrorSeverity.CRITICAL
        assert mount_err.normalized_signature == "FS_MOUNT_ERROR"

        open_err = ErrorDetector.scan_line("f_open failed for /SD:/20260411_test.wav", 6.0, 1, 96000)
        assert open_err is not None
        assert open_err.normalized_signature == "FS_FILE_CREATION_FAILED"

    def test_audio_and_acquisition_patterns(self):
        loss_err = ErrorDetector.scan_line("Samples lost during active capture : 256", 30.0, 1, 192000)
        assert loss_err is not None
        assert loss_err.subsystem == ErrorSubsystem.AUDIO
        assert loss_err.severity == ErrorSeverity.CRITICAL
        assert loss_err.normalized_signature == "AUDIO_CAPTURE_SAMPLE_LOSS"

        adc_err = ErrorDetector.scan_line("<err> audio_adc: ADC overrun detected", 10.0, 1, 192000)
        assert adc_err is not None
        assert adc_err.normalized_signature == "AUDIO_ADC_OVERRUN"

    def test_memory_and_psram_patterns(self):
        txn_err = ErrorDetector.scan_line("PSRAM transaction error during QSPI drain", 45.0, 1, 96000)
        assert txn_err is not None
        assert txn_err.subsystem == ErrorSubsystem.MEMORY
        assert txn_err.severity == ErrorSeverity.CRITICAL
        assert txn_err.normalized_signature == "MEMORY_PSRAM_TRANSACTION_ERROR"

        full_err = ErrorDetector.scan_line("PSRAM buffer full: PSRAM Full Events : 3", 50.0, 1, 96000)
        assert full_err is not None
        assert full_err.normalized_signature == "MEMORY_PSRAM_FULL_EVENT"

    def test_system_and_kernel_fault_patterns(self):
        hardfault = ErrorDetector.scan_line("***** HardFault *****", 100.0, 2, 96000)
        assert hardfault is not None
        assert hardfault.subsystem == ErrorSubsystem.SYSTEM
        assert hardfault.severity == ErrorSeverity.FATAL
        assert hardfault.normalized_signature == "SYSTEM_HARDFAULT"

        assertion = ErrorDetector.scan_line("ASSERTION FAIL [ret == 0] at recorder.c:420", 110.0, 2, 96000)
        assert assertion is not None
        assert assertion.severity == ErrorSeverity.FATAL
        assert assertion.normalized_signature == "SYSTEM_ASSERTION_FAIL"


class TestAssessmentEngine:
    """Tests evidence-based classification of failures."""

    def test_hardware_classification_for_persistent_crc(self):
        trackers = {
            "SDMMC_DATA_CRC_FAIL": ErrorSignatureTracker(
                signature="SDMMC_DATA_CRC_FAIL",
                subsystem=ErrorSubsystem.SDMMC,
                severity=ErrorSeverity.RECOVERABLE,
                total_occurrences=4,
                successful_power_cycles=2,
                occurrences_after_recovery=2,
            )
        }
        classification, rationale, evidence = AssessmentEngine.assess(
            trackers=trackers, recoveries=[], power_cycles_count=2
        )
        assert classification == "Likely hardware-related"
        assert "CRC" in rationale

    def test_firmware_classification_for_assertion(self):
        trackers = {
            "SYSTEM_ASSERTION_FAIL": ErrorSignatureTracker(
                signature="SYSTEM_ASSERTION_FAIL",
                subsystem=ErrorSubsystem.SYSTEM,
                severity=ErrorSeverity.FATAL,
                total_occurrences=1,
            )
        }
        classification, rationale, evidence = AssessmentEngine.assess(
            trackers=trackers, recoveries=[], power_cycles_count=0
        )
        assert classification == "Likely firmware-related"
        assert "assertion" in rationale.lower() or "exception" in rationale.lower()


class TestStabilityRunnerExecution:
    """Tests mocked long-duration stability runner execution, automatic recovery, and reporting."""

    def test_stability_runner_executes_recovery_and_generates_artifacts(self, tmp_path):
        # Create simulated STM32 mock serial responses
        mock_port = MockSerialPort()
        mock_port.queue_response("DEVINFO", (
            "SAMF 96000\r\n"
            "BTPS 16\r\n"
            "RECT 60\r\n"
            "SLEEP 5\r\n"
            "VER 1.2.0, SHA b1c299f43d0f, DT Nov  3 2026\r\n"
            "OK\r\n"
        ))
        mock_port.queue_response("STATUS", (
            "=== SYSTEM & RECORDING STATUS ===\r\n"
            "  Recording State       : IDLE\r\n"
            "OK\r\n"
        ))
        mock_port.queue_response("SAMF 96000", "Value is set to 96000\r\nOK\r\n")
        mock_port.queue_response("BTPS 16", "Value is set to 16\r\nOK\r\n")
        mock_port.queue_response("RECT 2", "Value is set to 2\r\nOK\r\n")
        mock_port.queue_response("SLEEP 1", "Value is set to 1\r\nOK\r\n")
        mock_port.queue_response("APPLY", "Device configuration is applied after reboot.\r\nOK\r\n")
        mock_port.queue_response("STOP", "OK: Stop recording requested\r\n")

        # Inject simulated UART session lines with an injected recoverable CRC error
        session_lines = [
            "=========================================================================\r\n",
            "           [SCHEDULER] AUTONOMOUS RECORDING SESSION 1 START              \r\n",
            "=========================================================================\r\n",
            "SDMMC transfer failed: DATA_CRC_FAIL on block 0x1200\r\n",
            "WAV_FILE_CLOSED\r\n",
            "[TEST] RECORDING ACCOUNTING SUMMARY:\r\n",
            "  Expected source samples            : 192000\r\n",
            "  Acquired source samples            : 192000\r\n",
            "  Successfully processed samples     : 192000\r\n",
            "  Samples written                    : 192000\r\n",
            "  Samples lost during active capture : 0\r\n",
            "  Boundary samples discarded         : 0\r\n",
            "  SD Writer Average Throughput       : 840 KB/s\r\n",
            "[TEST] WAV_VERIFIED\r\n",
            "[SCHEDULER] Session 1 finished: status=0, err=0\r\n",
        ]
        for line in session_lines:
            mock_port.rx_buffer.extend(line.encode("utf-8"))

        controller = SerialController(
            port="MOCK",
            mock_port=mock_port,
            output_dir=str(tmp_path / "raw_logs"),
        )
        ppk2 = PPK2Controller(port="MOCK", mock=True)

        runner = StabilityRunner(
            controller=controller,
            ppk2=ppk2,
            sample_rate=96000,
            bit_depth=16,
            duration_seconds=0.6,  # Fast duration for test
            session_rect_s=2,
            session_sleep_s=1,
            discharge_duration_s=0.01,
            boot_stabilize_s=0.01,
            recovery_timeout_s=2.0,
            trend_interval_s=0.1,
        )

        test_id = "STM32-TEST-STABILITY-001"
        session = runner.run(test_id=test_id, evidence_type="SAMPLE")

        # Verify execution and recovery occurred
        assert len(runner.errors) >= 1
        assert runner.errors[0].normalized_signature == "SDMMC_DATA_CRC_FAIL"
        assert runner.power_cycles_count >= 1
        assert len(runner.recoveries) >= 1
        assert runner.recoveries[0].result == "SUCCESS"

        # Verify generated artifacts
        parsed_path = Path(f"data/stm32/parsed/{test_id}_{runner.recoveries[0].timestamp_iso[:10].replace('-', '')}")
        json_matches = sorted(list(Path("data/stm32/parsed").glob(f"{test_id}_*.json")), key=lambda p: p.stat().st_mtime)
        assert len(json_matches) >= 1
        saved_data = json.loads(json_matches[-1].read_text(encoding="utf-8"))
        assert saved_data["test_id"] == test_id
        assert saved_data["sample_rate"] == 96000
        assert saved_data["power_cycles_count"] >= 1
        assert "hardware_vs_firmware_assessment" in saved_data

        # Verify Stability Test sheet exists in workbook
        wb_mgr = ComparisonWorkbookManager("data/reports/stm32_vs_max78000.xlsx")
        assert "Stability Test" in wb_mgr.workbook.sheetnames
        ws_stab = wb_mgr.workbook["Stability Test"]
        assert ws_stab.max_row >= 2

    def test_max_recovery_limit_stops_execution(self, tmp_path):
        mock_port = MockSerialPort()
        mock_port.queue_response("DEVINFO", "SAMF 96000\r\nBTPS 16\r\nOK\r\n")
        mock_port.queue_response("STATUS", "Recording State : IDLE\r\nOK\r\n")
        mock_port.queue_response("STOP", "OK\r\n")

        controller = SerialController(port="MOCK", mock_port=mock_port, output_dir=str(tmp_path))
        ppk2 = PPK2Controller(port="MOCK", mock=True)

        runner = StabilityRunner(
            controller=controller,
            ppk2=ppk2,
            sample_rate=96000,
            duration_seconds=5.0,
            max_signature_recoveries=2,  # Limit to 2 recoveries
            discharge_duration_s=0.01,
            boot_stabilize_s=0.01,
            recovery_timeout_s=1.0,
        )

        # Inject repeated errors exceeding limit
        for _ in range(5):
            mock_port.rx_buffer.extend(b"DATA_CRC_FAIL on block 0x1000\r\n")

        session = runner.run(test_id="STM32-TEST-LIMIT-001", evidence_type="SAMPLE")

        # Must enter safe stop and mark FAIL
        assert runner.error_trackers["SDMMC_DATA_CRC_FAIL"].is_persistent is True
        assert runner.final_status == "FAIL"
        assert "Persistent error exceeded limit" in runner.terminal_reason

