from __future__ import annotations

from pathlib import Path
import openpyxl
import pytest

from measurement_tool.models import TestSession
from measurement_tool.workbook import ComparisonWorkbookManager, backup_workbook


REQUIRED_SHEET_NAMES = [
    "Start Here",
    "Test Setup",
    "Executive Summary",
    "Hardware Config",
    "Comparison Summary",
    "Power Consumption",
    "Power & Sleep",
    "Sleep Current & RTC",
    "Wake & RTC",
    "Recording Perf & Integrity",
    "Recording Integrity",
    "SD Card Performance",
    "SD Card I-O",
    "Audio ADC Performance",
    "Audio & ADC",
    "PSRAM Performance",
    "RTOS & Memory",
    "RTOS & Resources",
    "Interfaces & Reliability",
    "Config Verification",
    "Stability Test",
    "Execution Log",
    "Test Log",
    "Evidence & References",
]


def test_workbook_initializes_all_11_required_sheets(tmp_path):
    wb_path = tmp_path / "test_stm32_vs_max78000.xlsx"
    mgr = ComparisonWorkbookManager(wb_path)

    assert wb_path.exists()
    sheet_names = mgr.workbook.sheetnames
    for req in REQUIRED_SHEET_NAMES:
        assert req in sheet_names, f"Missing required comparison sheet: {req}"


def test_workbook_preserves_max78000_reference_data(tmp_path):
    wb_path = tmp_path / "comparison.xlsx"
    mgr = ComparisonWorkbookManager(wb_path)
    wb = mgr.workbook

    # Check Comparison Summary
    ws_sum = wb["Comparison Summary"]
    found_rec_power = False

    for row in ws_sum.iter_rows(values_only=True):
        if row and "Recording average / peak current" in str(row[1]):
            found_rec_power = True
            assert row[2] is None  # No unverified MAX78000 measurement

    assert found_rec_power, "Recording power comparison row missing"

    # Check Power & Sleep
    ws_pwr = wb["Power & Sleep"]
    headers = [cell.value for cell in ws_pwr[1]]

    assert "MAX78000 Current (mA)" in headers
    assert "STM32 Avg (mA)" in headers

    # Verify sleep row exists, but baseline remains unpopulated
    found_sleep = False

    for row in ws_pwr.iter_rows(values_only=True):
        if row and "Stop2 sleep" in str(row[0]):
            found_sleep = True
            assert row[1] is None

    assert found_sleep, "Stop2 sleep row missing"


def test_workbook_keeps_unverified_stop2_current_blank(tmp_path):
    wb_path = tmp_path / "comparison.xlsx"
    mgr = ComparisonWorkbookManager(wb_path)
    ws_sleep = mgr.workbook["Sleep Current & RTC"]
    sleep_row = next((row for row in ws_sleep.iter_rows(values_only=True) if row and "Verified STOP2 current" in str(row[0])), None)
    assert sleep_row is not None
    assert sleep_row[3] is None
    assert sleep_row[4] == "UNVERIFIED"


def test_workbook_backup_mechanism(tmp_path):
    wb_path = tmp_path / "original.xlsx"
    mgr = ComparisonWorkbookManager(wb_path)

    backup_path = backup_workbook(wb_path)
    assert backup_path is not None
    assert backup_path.exists()
    assert backup_path.name.startswith("original_")
    assert backup_path.name.endswith(".bak.xlsx")


def test_workbook_updates_test_log_and_measurements(tmp_path):
    wb_path = tmp_path / "report.xlsx"
    mgr = ComparisonWorkbookManager(wb_path)

    session = TestSession(
        test_id="LOG-001",
        board="STM32L476RG",
        firmware_commit="6b7a624",
        build_type="production",
        category="recording",
        configuration="192 kHz / 10s / 1ch",
        raw_log_path="data/stm32/raw_logs/sample.log",
        ppk2_csv_path="data/ppk2/power.csv",
        parsed_result={"acquired_samples": 1920000.0, "samples_written": 1920000.0, "wav_verification": "PASS"},
        evidence_type="HARDWARE",
        status="pass",
    )

    res = mgr.add_or_update_session(session)
    assert res["status"] == "pass"

    wb = openpyxl.load_workbook(wb_path)
    ws_log = wb["Test Log"]
    found_log = False
    for row in ws_log.iter_rows(values_only=True):
        if row and row[0] == "LOG-001":
            found_log = True
            assert row[2] == "recording"
            assert row[4] == "PASS"
            assert row[5] == "HARDWARE"
    assert found_log, "Session entry not found in Test Log"

