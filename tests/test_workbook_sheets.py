from __future__ import annotations

from pathlib import Path
import openpyxl
import pytest

from measurement_tool.models import TestSession
from measurement_tool.workbook import ComparisonWorkbookManager, backup_workbook


REQUIRED_SHEET_NAMES = [
    "Start Here",
    "Test Setup",
    "Comparison Summary",
    "Power & Sleep",
    "Wake & RTC",
    "Recording Integrity",
    "SD Card I-O",
    "Audio & ADC",
    "RTOS & Resources",
    "Interfaces & Reliability",
    "Test Log",
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
        if row and "Active Recording Current" in str(row[1]):
            found_rec_power = True
            assert "28.50 mA" in str(row[2])  # MAX78000 baseline
    assert found_rec_power, "Active Recording Current comparison row missing"

    # Check Power & Sleep
    ws_pwr = wb["Power & Sleep"]
    headers = [cell.value for cell in ws_pwr[1]]
    assert "MAX78000 Current (mA)" in headers
    assert "STM32 Avg (mA)" in headers

    # Verify that sleep current is populated
    found_sleep = False
    for row in ws_pwr.iter_rows(values_only=True):
        if row and "Stop 2 Deep Sleep" in str(row[0]):
            found_sleep = True
            assert row[1] == pytest.approx(0.0021)  # MAX78000 deep sleep ~2.1 uA
    assert found_sleep, "Stop 2 Deep Sleep row missing"


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

