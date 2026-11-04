from openpyxl import load_workbook

from measurement_tool.models import TestSession
from measurement_tool.report_generator import _effective_status, _session_warnings, update_test_result


def test_sample_evidence_is_flagged_and_kept_pending():
    session = TestSession(
        test_id="SAMPLE-01",
        board="STM32L476RG",
        firmware_commit="a1b2c3d",
        build_type="production",
        category="recording",
        configuration="20 s",
        raw_log_path="data/stm32/raw_logs/stm32_sample_capture.log",
        ppk2_csv_path="data/ppk2/ppk2_sample.csv",
        parsed_result={"expected_samples": 100, "acquired_samples": 100, "processed_samples": 118000, "samples_written": 119000, "recording_duration": 6.25},
        evidence_type="SAMPLE",
        status="pass",
    )

    warnings = _session_warnings(session)
    assert "Evidence type is SAMPLE; hardware validation was not performed." in warnings
    assert _effective_status(session) == "pending"


def test_sample_counter_inconsistency_is_detected():
    session = TestSession(
        test_id="COUNTER-01",
        firmware_commit="abcdef1",
        configuration="20 s",
        parsed_result={"expected_samples": 100, "acquired_samples": 100, "processed_samples": 120, "samples_written": 121, "lost_samples": 0},
    )

    warnings = _session_warnings(session)
    assert any("processed samples exceed acquired samples" in warning for warning in warnings)
    assert any("written samples exceed processed samples" in warning for warning in warnings)


def test_duration_mismatch_is_detected():
    session = TestSession(
        test_id="DURATION-01",
        firmware_commit="abcdef1",
        configuration="20 s",
        parsed_result={"recording_duration": 6.25},
    )

    warnings = _session_warnings(session)
    assert any("Recording duration mismatch" in warning for warning in warnings)


def test_placeholder_commit_is_rejected():
    session = TestSession(
        test_id="COMMIT-01",
        firmware_commit="a1b2c3d",
        parsed_result={"expected_samples": 100, "acquired_samples": 100},
    )

    warnings = _session_warnings(session)
    assert "Firmware commit identifier is missing or invalid." in warnings


def test_report_populates_warning_and_evidence_type_in_workbook(tmp_path):
    workbook_path = tmp_path / "stm32_vs_max78000.xlsx"
    session = TestSession(
        test_id="REPORT-01",
        board="STM32L476RG",
        firmware_commit="a1b2c3d",
        build_type="production",
        category="recording",
        configuration="20 s",
        raw_log_path="data/stm32/raw_logs/stm32_sample_capture.log",
        ppk2_csv_path="data/ppk2/ppk2_sample.csv",
        parsed_result={"expected_samples": 120000, "acquired_samples": 120000, "processed_samples": 121000, "samples_written": 122000},
        sample_data={"average_current": 0.032},
        evidence_type="SAMPLE",
        status="pass",
    )

    result = update_test_result(workbook_path, session)
    workbook = load_workbook(workbook_path)
    ws = workbook["STM32 Measurements"]

    assert result["status"] == "pending"
    assert "Evidence Type: SAMPLE" in ws[f"L{result['row']}"].value
    assert ws[f"N{result['row']}"].value
    warning_text = ws[f"N{result['row']}"].value.lower()
    assert "processed samples exceed acquired samples" in warning_text
    assert "written samples exceed processed samples" in warning_text
