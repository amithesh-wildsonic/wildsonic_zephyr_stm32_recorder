import pytest

from measurement_tool.ppk2_import import parse_ppk2_csv


def test_ppk2_import_extracts_available_current_and_energy_metrics():
    metrics = parse_ppk2_csv("data/ppk2/ppk2_sample.csv")

    assert metrics["average_current"] == pytest.approx(0.032)
    assert metrics["minimum_current"] == pytest.approx(0.028)
    assert metrics["maximum_current"] == pytest.approx(0.042)
    assert metrics["peak_current"] == pytest.approx(0.051)
    assert metrics["measurement_duration"] == pytest.approx(12.5)
    assert metrics["energy_consumption"] == pytest.approx(1.0)
    assert metrics["voltage"] == pytest.approx(3.3)
