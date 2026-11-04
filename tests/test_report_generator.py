from openpyxl import load_workbook

from measurement_tool.models import TestSession
from measurement_tool.report_generator import update_test_result


def test_report_generator_updates_workbook_and_status(tmp_path):
    workbook_path = tmp_path / "stm32_vs_max78000.xlsx"
    session = TestSession(
        test_id="STM32-TEST-01",
        board="STM32L476RG",
        firmware_commit="abc123",
        build_type="production",
        category="recording",
        configuration="48 kHz / 20 s",
        raw_log_path="data/stm32/raw_logs/stm32_sample_capture.log",
        ppk2_csv_path="data/ppk2/ppk2_sample.csv",
        parsed_result={"adc_overruns": 0, "dma_errors": 0, "lost_samples": 0, "wav_verification": "PASS"},
        status="pending",
    )

    result = update_test_result(workbook_path, session)
    workbook = load_workbook(workbook_path)

    assert result["status"] == "pending"
    assert workbook_path.exists()
    assert result["sheet"] == "STM32 Measurements"
    assert "STM32 Measurements" in workbook.sheetnames
    assert "Comparison Summary" in workbook.sheetnames
