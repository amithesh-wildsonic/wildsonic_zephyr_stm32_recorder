from __future__ import annotations

from pathlib import Path
import pytest

from measurement_tool.ppk2_controller import MockPPK2Device, PPK2Controller, PPK2PhaseWindow


def test_ppk2_connection_and_device_identification():
    ctrl = PPK2Controller(port="MOCK_PPK2", mock=True)
    assert ctrl.port == "MOCK_PPK2"
    ctrl.connect()
    assert ctrl._device is not None
    assert ctrl._device.is_connected is True
    modifiers = ctrl._device.get_modifiers()
    assert modifiers["model"] == "PPK2"
    assert modifiers["calibration"] == "VALID"


def test_ppk2_source_meter_configuration():
    ctrl = PPK2Controller(port="MOCK_PPK2", mock=True)
    ctrl.connect()

    # Valid E5V 5.0 V configuration
    ctrl.configure_source_meter(5000)
    assert ctrl.voltage_mv == 5000
    assert ctrl._device.mode == "SOURCE_METER"
    assert ctrl._device.voltage_mv == 5000

    # Valid VDD 3.3 V configuration
    ctrl.configure_source_meter(3300)
    assert ctrl.voltage_mv == 3300
    assert ctrl._device.voltage_mv == 3300

    # Out of range voltage validation
    with pytest.raises(ValueError, match="Invalid PPK2 voltage"):
        ctrl.configure_source_meter(7000)

    with pytest.raises(ValueError, match="Invalid PPK2 voltage"):
        ctrl.configure_source_meter(500)


def test_ppk2_output_enable_and_disable():
    ctrl = PPK2Controller(port="MOCK_PPK2", mock=True)
    ctrl.connect()
    ctrl.configure_source_meter(5000)

    # Initially OFF
    ctrl.power_off(force=True)
    assert ctrl.power_state == "OFF"
    assert ctrl._device.power_state == "OFF"

    # Turn ON
    t_on_ns = ctrl.power_on()
    assert t_on_ns > 0
    assert ctrl.power_state == "ON"
    assert ctrl._device.power_state == "ON"

    # Turn OFF
    ctrl.power_off()
    assert ctrl.power_state == "OFF"
    assert ctrl._device.power_state == "OFF"


def test_ppk2_power_sampling_start_and_stop():
    ctrl = PPK2Controller(port="MOCK_PPK2", mock=True)
    ctrl.connect()
    ctrl.configure_source_meter(5000)
    ctrl.power_on()

    t_start = ctrl.start_sampling()
    assert t_start > 0
    assert ctrl._sampling_active is True

    # Allow mock worker to collect samples
    import time
    time.sleep(0.05)

    ctrl.stop_sampling()
    assert ctrl._sampling_active is False
    assert len(ctrl.all_currents_ua) > 0
    assert len(ctrl.all_timestamps_s) == len(ctrl.all_currents_ua)


def test_ppk2_phase_marking_and_alignment():
    ctrl = PPK2Controller(port="MOCK_PPK2", mock=True)
    ctrl.connect()
    ctrl.configure_source_meter(5000)
    ctrl.power_on()
    ctrl.start_sampling()

    ctrl.mark_phase_start("Phase B: Active Idle")
    import time
    time.sleep(0.05)
    ctrl.mark_phase_end("Phase B: Active Idle")
    ctrl.stop_sampling()

    assert "Phase B: Active Idle" in ctrl.phases
    phase = ctrl.phases["Phase B: Active Idle"]
    assert phase.duration_s >= 0.04
    assert phase.avg_current_ma == pytest.approx(15.0, rel=0.1)
    assert phase.avg_power_mw == pytest.approx(75.0, rel=0.1)
    assert phase.total_energy_mj > 0.0


def test_ppk2_power_capture_persistence(tmp_path):
    ctrl = PPK2Controller(port="MOCK_PPK2", mock=True)
    ctrl.connect()
    ctrl.configure_source_meter(5000)
    ctrl.power_on()
    ctrl.start_sampling()

    ctrl.mark_phase_start("Phase A: Cold Boot")
    import time
    time.sleep(0.03)
    ctrl.mark_phase_end("Phase A: Cold Boot")
    ctrl.stop_sampling()

    full_csv = tmp_path / "full.csv"
    phase_csv = tmp_path / "phase_a.csv"

    ctrl.export_csv(full_csv)
    ctrl.export_phase_csv("Phase A: Cold Boot", phase_csv)

    assert full_csv.exists()
    assert phase_csv.exists()

    content_full = full_csv.read_text(encoding="utf-8").splitlines()
    assert "Timestamp_s,Current_uA,Current_mA,Voltage_V,Power_mW" in content_full[0]
    assert len(content_full) > 1

    content_phase = phase_csv.read_text(encoding="utf-8").splitlines()
    assert "Relative_Time_s,Current_uA,Current_mA,Voltage_V,Power_mW" in content_phase[0]
    assert len(content_phase) > 1


def test_ppk2_safe_power_off_rejection_during_recording():
    ctrl = PPK2Controller(port="MOCK_PPK2", mock=True)
    ctrl.connect()
    ctrl.power_on()

    # Mark storage busy
    ctrl.is_storage_busy = True

    with pytest.raises(RuntimeError, match="Refusing power removal"):
        ctrl.power_off(force=False)

    # Force power off is permitted during emergency cleanup
    ctrl.power_off(force=True)
    assert ctrl.power_state == "OFF"

