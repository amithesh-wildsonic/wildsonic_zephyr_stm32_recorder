from __future__ import annotations

import json
from pathlib import Path
import pytest

from measurement_tool.config import AcceptanceCriteria
from measurement_tool.energy import (
    convert_energy,
    energy_j_from_w_s,
    energy_mj_from_mw_s,
    energy_uj_from_uw_s,
    power_mw_from_v_ma,
)
from measurement_tool.models import TestSession
from measurement_tool.parsers.recording import parse_recording_metrics
from measurement_tool.parsers.sleep import parse_sleep_metrics, verify_stop2_evidence
from measurement_tool.ppk2_controller import MockPPK2Device, PPK2Controller, PPK2PhaseWindow
from measurement_tool.report_generator import _effective_status, _session_warnings, update_test_result
from measurement_tool.workbook import ComparisonWorkbookManager


class TestEnergyCalculations:
    """Regression tests for energy calculations and unit conversions."""

    def test_mw_times_seconds_equals_mj(self):
        # 307.9 mW * 5.094 s = ~1568.42 mJ
        power_mw = 307.895507
        duration_s = 5.094
        energy_mj = energy_mj_from_mw_s(power_mw, duration_s)
        assert pytest.approx(energy_mj, rel=1e-4) == 1568.42

    def test_w_times_seconds_equals_j(self):
        power_w = 0.3078955
        duration_s = 5.094
        energy_j = energy_j_from_w_s(power_w, duration_s)
        assert pytest.approx(energy_j, rel=1e-4) == 1.56842

    def test_uw_times_seconds_equals_uj(self):
        power_uw = 307895.5
        duration_s = 5.094
        energy_uj = energy_uj_from_uw_s(power_uw, duration_s)
        assert pytest.approx(energy_uj, rel=1e-4) == 1568420.0

    def test_energy_conversions_across_units(self):
        # 1.56842 J = 1568.42 mJ = 1568420.0 uJ
        assert pytest.approx(convert_energy(1.56842, "J", "mJ"), rel=1e-4) == 1568.42
        assert pytest.approx(convert_energy(1568.42, "mJ", "J"), rel=1e-4) == 1.56842
        assert pytest.approx(convert_energy(1568.42, "mJ", "uJ"), rel=1e-4) == 1568420.0
        assert pytest.approx(convert_energy(1568420.0, "uJ", "mJ"), rel=1e-4) == 1568.42
        # mWh conversion: 1 J = 1/3.6 mWh
        assert pytest.approx(convert_energy(3.6, "J", "mWh"), rel=1e-4) == 1.0


class TestRecordingAccounting:
    """Regression tests for sample accounting, boundary discard, and reconciliation."""

    LOG_SNIPPET = """
    STATUS
    === SYSTEM & RECORDING STATUS ===
      Recording State       : ACTIVE
      Samples Written       : 0 / 1920000
    OK

    [TEST] RECORDING ACCOUNTING SUMMARY:
      Expected source samples            : 1920000
      Acquired source samples            : 1921024
      Successfully processed samples     : 1921024
      Samples written                    : 1920000
      Samples lost during active capture : 0
      Boundary samples discarded         : 1024
      DSP average / worst time           : 8513 / 11181 us

    [TEST] WAV_VERIFIED
    """

    def test_parse_accounting_summary_distinguishes_boundary_discard(self):
        metrics = parse_recording_metrics(self.LOG_SNIPPET)
        assert metrics["expected_samples"] == 1920000.0
        assert metrics["acquired_samples"] == 1921024.0
        assert metrics["processed_samples"] == 1921024.0
        assert metrics["samples_written"] == 1920000.0
        assert metrics["boundary_samples_discarded"] == 1024.0
        assert metrics["lost_samples"] == 0.0
        assert metrics["accounting_reconciled"] is True
        assert metrics["wav_verification"] == "PASS"

    def test_acceptance_criteria_does_not_flag_boundary_discard_as_mismatch(self):
        criteria = AcceptanceCriteria()
        metrics = {
            "expected_samples": 1920000.0,
            "acquired_samples": 1921024.0,
            "processed_samples": 1921024.0,
            "samples_written": 1920000.0,
            "boundary_samples_discarded": 1024.0,
            "lost_samples": 0.0,
            "wav_verification": "PASS",
            "stop2_verified": True,
            "stop2_entry": "PASS",
        }
        warnings = criteria.evaluate(metrics)
        # Should NOT contain false warnings about deficit or mismatch
        assert not any("Sample storage deficit" in w for w in warnings)
        assert not any("Sample counter mismatch" in w for w in warnings)
        assert len(warnings) == 0

    def test_acceptance_criteria_flags_true_lost_samples(self):
        criteria = AcceptanceCriteria(lost_samples_max=0)
        metrics = {
            "expected_samples": 1920000.0,
            "acquired_samples": 1921024.0,
            "processed_samples": 1920000.0,
            "samples_written": 1920000.0,
            "boundary_samples_discarded": 0.0,
            "lost_samples": 1024.0,
            "wav_verification": "PASS",
        }
        warnings = criteria.evaluate(metrics)
        assert any("Active capture samples lost" in w for w in warnings)


class TestStop2Verification:
    """Regression tests for multi-stage Stop 2 lifecycle evidence."""

    def test_verified_when_all_four_milestones_present(self):
        log = """
        [TEST] SLEEP_INTERVAL_STARTED: duration=5 s
        [TEST] RTC_WUT_ARMED: duration=5 s
        [TEST] STOP2_ENTERED
        [TEST] STOP2_EXITED
        [TEST] RTC_WAKE_ISR
        [TEST] CLOCKS_RESTORED
        [TEST] UART_RESTORED
        """
        evidence = verify_stop2_evidence(log)
        assert evidence["is_verified"] is True
        assert len(evidence["missing_evidence"]) == 0

        metrics = parse_sleep_metrics(log)
        assert metrics["stop2_verification"] == "PASS"
        assert metrics["stop2_entry"] == "PASS"
        assert metrics["stop2_verified"] is True

    def test_unverified_when_sleep_command_acknowledged_but_mcu_does_not_sleep(self):
        log = """
        SLEEP 5
        Value is set to 5
        [ACK] Command SLEEP received and validated. Saved to persistent flash (seq 174, pending gen 67).
        [PENDING] Configuration pending application after reboot.
        [SCHEDULER] AUTONOMOUS RECORDING SESSION 3 START
        """
        evidence = verify_stop2_evidence(log)
        assert evidence["is_verified"] is False
        assert "Actual Stop 2 entry marker (STOP2_ENTERED) not detected" in evidence["missing_evidence"]
        assert "RTC wake event marker (RTC_WAKE_ISR / RTC wake received) not detected" in evidence["missing_evidence"]
        assert "Post-wake clock/UART restoration milestone not detected" in evidence["missing_evidence"]

        metrics = parse_sleep_metrics(log)
        assert metrics["stop2_verification"] == "UNVERIFIED"
        assert metrics["stop2_entry"] == "UNVERIFIED"
        assert metrics["stop2_verified"] is False

    def test_unverified_when_wake_event_is_missing(self):
        log = """
        [TEST] SLEEP_INTERVAL_STARTED: duration=5 s
        [TEST] STOP2_ENTERED
        # Device hung, no wake occurred
        """
        evidence = verify_stop2_evidence(log)
        assert evidence["is_verified"] is False
        assert "RTC wake event marker (RTC_WAKE_ISR / RTC wake received) not detected" in evidence["missing_evidence"]


class TestPhaseDurationAndTimingConsistency:
    """Regression tests for PPK2 phase durations and sample count checks."""

    def test_phase_timing_verification_detects_misalignment(self):
        ctrl = PPK2Controller(port="MOCK", mock=True)
        ctrl.connect()
        ctrl.start_sampling()

        # Mark phase start
        ctrl.mark_phase_start("Phase C: Recording")
        # Add simulated samples corresponding to 6.3s instead of 10s
        for _ in range(63000):
            ctrl.all_currents_ua.append(62000.0)

        # End phase with expected 10.0s duration
        ctrl.mark_phase_end("Phase C: Recording", expected_duration_s=10.0)

        phase = ctrl.phases["Phase C: Recording"]
        assert phase.timing_verified is False
        assert "deviates from expected 10.0s" in phase.timing_notes
        ctrl.stop_sampling()

    def test_energy_calculated_accurately_in_controller(self):
        ctrl = PPK2Controller(port="MOCK", mock=True, voltage_mv=5000)
        ctrl.connect()
        ctrl.start_sampling()

        ctrl.mark_phase_start("Phase B: Active Idle")
        # 10,000 samples @ 60 mA = 60,000 uA
        for _ in range(10000):
            ctrl.all_currents_ua.append(60000.0)

        ctrl.mark_phase_end("Phase B: Active Idle", expected_duration_s=0.1)
        phase = ctrl.phases["Phase B: Active Idle"]
        # Power = 60 mA * 5 V = 300 mW
        assert pytest.approx(phase.avg_power_mw, rel=1e-3) == 300.0
        # Energy = 300 mW * duration (not 300 * duration * 1000)
        expected_mj = phase.avg_power_mw * phase.duration_s
        assert pytest.approx(phase.total_energy_mj, rel=1e-3) == expected_mj
        ctrl.stop_sampling()


class TestArtifactHandlingAndWorkbookUpdate:
    """Regression tests for JSON artifact consistency and workbook updates."""

    def test_reprocessed_hw_001_values_in_workbook(self):
        wb_mgr = ComparisonWorkbookManager("data/reports/stm32_vs_max78000.xlsx")
        ws_pwr = wb_mgr.workbook["Power & Sleep"]

        # Check Idle energy is ~1568.42 mJ (not 1.5M mJ!)
        idle_row = None
        for row in ws_pwr.iter_rows(min_row=2, values_only=True):
            if "Idle / Baseline" in str(row[0]):
                idle_row = row
                break
        assert idle_row is not None
        assert idle_row[8] == 1568.42  # Energy mJ
        assert idle_row[7] == 307.896  # Power mW

        # Check Stop 2 row has UNVERIFIED note
        stop2_row = None
        for row in ws_pwr.iter_rows(min_row=2, values_only=True):
            if "Stop 2 Deep Sleep" in str(row[0]):
                stop2_row = row
                break
        assert stop2_row is not None
        assert "UNVERIFIED" in str(stop2_row[10])

        # Check Test Log has failure reasons
        ws_log = wb_mgr.workbook["Test Log"]
        hw_log_row = None
        for row in ws_log.iter_rows(min_row=2, values_only=True):
            if row[0] == "STM32-PPK2-HW-001":
                hw_log_row = row
                break
        assert hw_log_row is not None
        assert hw_log_row[4] == "FAIL"
        assert "Failure Reasons" in str(hw_log_row[10])
        assert "Stop2 deep sleep unverified" in str(hw_log_row[10])
        assert "Phase timing discrepancy" in str(hw_log_row[10])

