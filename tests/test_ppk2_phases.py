from __future__ import annotations

from pathlib import Path
import pytest

from measurement_tool.ppk2 import (
    PPK2Processor,
    PPK2Sample,
    is_automated_ppk2_available,
)


def test_ppk2_summary_phase_metrics():
    proc = PPK2Processor(voltage_default=3.3)
    proc.load_csv("data/ppk2/ppk2_sample.csv")

    phases = proc.get_summary_phase_metrics()
    assert "Recording" in phases
    rec = phases["Recording"]

    assert rec.duration_s == 12.5
    assert rec.avg_current_ma == pytest.approx(32.0)
    assert rec.min_current_ma == pytest.approx(28.0)
    assert rec.max_current_ma == pytest.approx(42.0)
    assert rec.peak_current_ma == pytest.approx(51.0)
    assert rec.avg_voltage_v == pytest.approx(3.3)
    assert rec.avg_power_mw == pytest.approx(32.0 * 3.3)
    assert rec.max_power_mw == pytest.approx(42.0 * 3.3)
    assert rec.total_energy_mj == pytest.approx(32.0 * 3.3 * 12.5)


def test_ppk2_manual_window_segmentation(tmp_path):
    proc = PPK2Processor(voltage_default=3.3)

    # Fabricate 100 samples spanning 0 to 10 seconds
    for sec in range(100):
        t = sec * 0.1
        # 0.0 to 3.0s: Idle (~9.0 mA)
        if t < 3.0:
            c = 9.0 + (sec % 3) * 0.1
        # 3.0 to 7.0s: Active Recording (~26.5 mA)
        elif t < 7.0:
            c = 26.5 + (sec % 5) * 0.5
        # 7.0 to 10.0s: Stop 2 Sleep (~0.0028 mA)
        else:
            c = 0.0028
        proc.samples.append(PPK2Sample(timestamp=t, current_ma=c, voltage_v=3.3))

    windows = {
        "Idle": (0.0, 2.9),
        "Recording": (3.0, 6.9),
        "Stop 2 / sleep": (7.0, 9.9),
    }

    segmented = proc.segment_by_windows(windows)
    assert "Idle" in segmented
    assert "Recording" in segmented
    assert "Stop 2 / sleep" in segmented

    assert segmented["Idle"].avg_current_ma == pytest.approx(9.1, abs=0.2)
    assert segmented["Recording"].avg_current_ma == pytest.approx(27.5, abs=1.0)
    assert segmented["Stop 2 / sleep"].avg_current_ma == pytest.approx(0.0028, abs=0.0005)

    # Export to CSV and verify format
    csv_out = tmp_path / "segmented_power.csv"
    proc.export_phase_csv(segmented, csv_out)
    assert csv_out.exists()
    content = csv_out.read_text(encoding="utf-8")
    assert "Idle" in content
    assert "Recording" in content
    assert "Stop 2 / sleep" in content


def test_automated_ppk2_availability_check():
    avail, msg = is_automated_ppk2_available()
    assert isinstance(avail, bool)
    assert isinstance(msg, str)
    assert len(msg) > 0

