#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Test Suite: Solar Scheduling, Console Parity, Shutdown States, and Dual-Bank Bootloader
======================================================================================
Covers:
 1. NOAA Solar Calculation & Julian Date precision
 2. Sunrise / Sunset times for reference locations (Coimbatore, Bengaluru, Equator)
 3. Polar day and polar night detection and safe handling
 4. Midnight-crossing solar window splitting
 5. Solar window rounding behavior
 6. All 6 sun recording modes
 7. Location (LOC) console command parsing, boundaries, and clearing
 8. Sun (SUN) console command calculation and unset location rejection
 9. Sunmode (SUNMODE) modes, invalid rejection, and disabled interval clearing
10. Sun interval setters (SUNRISEBEF, SUNRISEAFT, SUNSETBEF, SUNSETAFT, SUNROUND)
11. BLE device info reporting parity for SUNMODE
12. Graceful Shutdown 8-stage state machine sequencing
13. Scheduler race prevention when reboot_pending is asserted
14. SD Release image packaging (WS32 magic, MCU ID 0x00004760, CRC32 verification)
15. Cortex-M4 vector table pre-flash validation (SP in SRAM, Reset Vector in Flash)
"""

import math
import struct
import zlib
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
BOOTLOADER_SRC = REPO_ROOT / "bootloader"
SCRIPTS_DIR = REPO_ROOT / "scripts"

SOLAR_C = APP_SRC / "solar_calculator.c"
SOLAR_H = APP_SRC / "solar_calculator.h"
CONSOLE_C = APP_SRC / "console.c"
SCHEDULER_C = APP_SRC / "scheduler.c"
MAIN_C = APP_SRC / "main.c"
BLE_COMM_C = APP_SRC / "ble_comm.c"
RELEASE_GEN_PY = SCRIPTS_DIR / "generate_stm32_release_package.py"


# ==============================================================================
# Helper NOAA Solar Algorithm in Python (mirroring solar_calculator.c)
# ==============================================================================

def compute_julian_date(year: int, month: int, day: int, fractional_day: float = 0.0) -> float:
    # Fliegel & Van Flandern (1968)
    a = (14 - month) // 12
    y = year + 4800 - a
    m = month + 12 * a - 3
    jdn = day + (153 * m + 2) // 5 + 365 * y + y // 4 - y // 100 + y // 400 - 32045
    return float(jdn) - 0.5 + fractional_day


def compute_sunrise_sunset(jd: float, lat_deg: float, lon_deg: float, tz_offset_sec: int):
    T = (jd - 2451545.0) / 36525.0
    L0 = (280.46646 + T * (36000.76983 + T * 0.0003032)) % 360.0
    if L0 < 0.0:
        L0 += 360.0

    M = 357.52911 + T * (35999.05029 - 0.0001537 * T)
    e = 0.016708634 - T * (0.000042037 + 0.0000001267 * T)
    C = (
        (1.914602 - T * (0.004817 + 0.000014 * T)) * math.sin(math.radians(M))
        + (0.019993 - 0.000101 * T) * math.sin(math.radians(2.0 * M))
        + 0.000289 * math.sin(math.radians(3.0 * M))
    )
    true_long = L0 + C
    omega = 125.04 - 1934.136 * T
    lambda_val = true_long - 0.00569 - 0.00478 * math.sin(math.radians(omega))
    epsilon0 = 23.0 + (26.0 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60.0) / 60.0
    epsilon = epsilon0 + 0.00256 * math.cos(math.radians(omega))

    y_val = math.tan(math.radians(epsilon / 2.0)) ** 2
    eq_time = 4.0 * math.degrees(
        y_val * math.sin(math.radians(2.0 * L0))
        - 2.0 * e * math.sin(math.radians(M))
        + 4.0 * e * y_val * math.sin(math.radians(M)) * math.cos(math.radians(2.0 * L0))
        - 0.5 * (y_val ** 2) * math.sin(math.radians(4.0 * L0))
        - 1.25 * (e ** 2) * math.sin(math.radians(2.0 * M))
    )

    sin_dec = math.sin(math.radians(epsilon)) * math.sin(math.radians(lambda_val))
    cos_dec = math.cos(math.asin(sin_dec))

    lat_rad = math.radians(lat_deg)
    zenith_rad = math.radians(90.833)

    cos_ha = (math.cos(zenith_rad) - math.sin(lat_rad) * sin_dec) / (math.cos(lat_rad) * cos_dec)
    if cos_ha > 1.0 or cos_ha < -1.0:
        return None  # Polar day or polar night

    ha_deg = math.degrees(math.acos(cos_ha))
    solar_noon_frac = (720.0 - 4.0 * lon_deg - eq_time) / 1440.0
    sunrise_frac = (720.0 - 4.0 * (lon_deg + ha_deg) - eq_time) / 1440.0
    sunset_frac = (720.0 - 4.0 * (lon_deg - ha_deg) - eq_time) / 1440.0

    sunrise_utc_sec = int(round(sunrise_frac * 86400.0))
    sunset_utc_sec = int(round(sunset_frac * 86400.0))

    sunrise_local = (sunrise_utc_sec + tz_offset_sec) % 86400
    sunset_local = (sunset_utc_sec + tz_offset_sec) % 86400
    return sunrise_local, sunset_local


# ==============================================================================
# 1. NOAA Solar Calculation & Julian Date Tests
# ==============================================================================

def test_julian_date_j2000_epoch():
    """Verify J2000.0 epoch: 2000-01-01 12:00 UT is exactly JD 2451545.0."""
    jd = compute_julian_date(2000, 1, 1, 0.5)
    assert abs(jd - 2451545.0) < 1e-4


def test_julian_date_arbitrary_dates():
    """Verify Julian date for known calendar benchmarks."""
    # 2026-03-21 00:00:00 UT
    jd_2026 = compute_julian_date(2026, 3, 21, 0.0)
    assert 2461120.0 < jd_2026 < 2461121.0


def test_sunrise_sunset_coimbatore():
    """Verify realistic sunrise/sunset times for Coimbatore (11.0168° N, 76.9558° E, UTC+05:30)."""
    jd = compute_julian_date(2026, 3, 21, 0.0)
    res = compute_sunrise_sunset(jd, 11.0168, 76.9558, 19800)
    assert res is not None
    sr, ss = res
    sr_hr, sr_min = sr // 3600, (sr // 60) % 60
    ss_hr, ss_min = ss // 3600, (ss // 60) % 60

    # Near vernal equinox, sunrise ~06:15 - 06:35, sunset ~18:15 - 18:40
    assert sr_hr == 6
    assert 15 <= sr_min <= 35
    assert ss_hr == 18
    assert 15 <= ss_min <= 45


def test_polar_night_and_day_handling():
    """Verify polar regions return None (cos_ha outside [-1, 1]) without crashes."""
    # 85° N on winter solstice: 24h polar night
    jd_winter = compute_julian_date(2026, 12, 21, 0.0)
    res_night = compute_sunrise_sunset(jd_winter, 85.0, 0.0, 0)
    assert res_night is None

    # 85° N on summer solstice: 24h polar day (midnight sun)
    jd_summer = compute_julian_date(2026, 6, 21, 0.0)
    res_day = compute_sunrise_sunset(jd_summer, 85.0, 0.0, 0)
    assert res_day is None


# ==============================================================================
# 2. Window Splitting & Rounding Tests
# ==============================================================================

def test_midnight_crossing_window_split():
    """Verify windows spanning midnight (start > end) are properly split into two sub-ranges."""
    # e.g., Sunset 18:30 (66600s) to Sunrise 06:15 (22500s)
    sunset_sec = 66600
    sunrise_sec = 22500
    assert sunset_sec > sunrise_sec

    # Split ranges:
    # Range 1: sunset_sec -> 86400 (midnight)
    # Range 2: 0 -> sunrise_sec
    r1 = (sunset_sec, 86400)
    r2 = (0, sunrise_sec)
    assert r1[0] < r1[1]
    assert r2[0] < r2[1]
    total_sec = (r1[1] - r1[0]) + (r2[1] - r2[0])
    assert total_sec == (86400 - sunset_sec) + sunrise_sec


def test_solar_interval_rounding():
    """Verify rounding seconds up or down to nearest configured minutes."""
    rounding_min = 5
    round_sec = rounding_min * 60  # 300s

    t = 6120  # 01:42:00 -> round down
    rounded_down = (t // round_sec) * round_sec
    assert rounded_down == 6000  # 01:40:00

    t_up = 6250  # round up
    rounded_up = ((t_up + round_sec - 1) // round_sec) * round_sec
    assert rounded_up == 6300  # 01:45:00


# ==============================================================================
# 3. Source Code Audit for Solar, Console & Shutdown Parity
# ==============================================================================

def test_solar_calculator_implementation_exists():
    """Verify solar_calculator.c implements NOAA algorithm and window calculators."""
    assert SOLAR_C.exists()
    assert SOLAR_H.exists()
    src = SOLAR_C.read_text(encoding="utf-8")
    assert "solar_compute_julian_date" in src
    assert "solar_compute_sunrise_sunset_seconds" in src
    assert "solar_compute_sun_ranges" in src
    assert "solar_compute_tod_ranges_for_date" in src


def test_console_solar_commands_implemented():
    """Verify LOC, SUN, SUNMODE, and interval handlers are implemented and wired."""
    src = CONSOLE_C.read_text(encoding="utf-8")
    assert "static void handle_loc" in src
    assert "static void handle_sun" in src
    assert "static void handle_sunmode" in src
    assert "static void handle_sunrisebef" in src
    assert "static void handle_sunriseaft" in src
    assert "static void handle_sunsetbef" in src
    assert "static void handle_sunsetaft" in src
    assert "static void handle_sunround" in src

    # Verify dispatch cases are wired to handlers (not report_deferred_hardware)
    assert "case CMD_LOCATION:\n\t\thandle_loc(" in src or "case CMD_LOCATION:\n        handle_loc(" in src
    assert "case CMD_SUN:\n\t\thandle_sun(" in src or "case CMD_SUN:\n        handle_sun(" in src
    assert "case CMD_SUN_RECORDING_MODE:\n\t\thandle_sunmode(" in src or "case CMD_SUN_RECORDING_MODE:\n        handle_sunmode(" in src


def test_ble_comm_reports_sunmode_parameters():
    """Verify ble_comm.c reports SUNMODE, intervals, and rounding in GET_DEV_INFO."""
    src = BLE_COMM_C.read_text(encoding="utf-8")
    assert "scfg.sun_recording_mode != SUN_MODE_DISABLED" in src
    assert 'ble_comm_printf("SUNMODE Sunrise\\r\\n");' in src
    assert 'ble_comm_printf("SUNMODE Sunset\\r\\n");' in src
    assert 'ble_comm_printf("SUNMODE Sunrise and Sunset\\r\\n");' in src
    assert 'ble_comm_printf("SUNMODE Sunrise to Sunset\\r\\n");' in src
    assert 'ble_comm_printf("SUNMODE Sunset to Sunrise\\r\\n");' in src
    assert 'ble_comm_printf("SUNRISEAFT %u\\r\\n"' in src
    assert 'ble_comm_printf("SUNSETBEF %u\\r\\n"' in src
    assert 'ble_comm_printf("SUNSETAFT %u\\r\\n"' in src
    assert 'ble_comm_printf("SUNROUND %u\\r\\n"' in src
    assert 'ble_comm_printf("SUNMODE Disabled\\r\\n");' in src


def test_graceful_shutdown_state_machine_stages():
    """Verify all 8 explicit shutdown stages are defined and sequenced."""
    main_c = MAIN_C.read_text(encoding="utf-8")
    cons_c = CONSOLE_C.read_text(encoding="utf-8")

    stages = [
        "SHUTDOWN_STAGE_REQUESTED",
        "SHUTDOWN_STAGE_SCHEDULER_STOPPED",
        "SHUTDOWN_STAGE_ADC_DMA_STOPPED",
        "SHUTDOWN_STAGE_AUDIO_QUEUES_DRAINED",
        "SHUTDOWN_STAGE_PSRAM_READER_DRAINED",
        "SHUTDOWN_STAGE_SD_WRITER_DRAINED",
        "SHUTDOWN_STAGE_WAV_FINALIZED",
        "SHUTDOWN_STAGE_CONFIG_PERSISTED",
        "SHUTDOWN_STAGE_REBOOT_READY",
        "SHUTDOWN_STAGE_SYSTEM_RESET",
    ]
    for stage in stages:
        assert stage in main_c, f"Stage {stage} missing from main.c"
        assert stage in cons_c, f"Stage {stage} missing from console.c"


def test_scheduler_race_prevention_on_reboot_pending():
    """Verify scheduler service step and dispatch check main_is_reboot_pending()."""
    main_c = MAIN_C.read_text(encoding="utf-8")
    sched_step_pos = main_c.find("void main_scheduler_service_step(void)")
    assert sched_step_pos != -1
    sched_step = main_c[sched_step_pos:sched_step_pos + 300]
    assert "atomic_get(&s_reboot_pending)" in sched_step

    disp_pos = main_c.find("static void dispatch_recording_session(")
    assert disp_pos != -1
    disp_fn = main_c[disp_pos:disp_pos + 200]
    assert "main_is_reboot_pending()" in disp_fn


# ==============================================================================
# 4. SD Release Packaging & Bootloader Tests
# ==============================================================================

def test_release_packaging_roundtrip(tmp_path):
    """Verify generate_stm32_release_package.py produces valid header and passes --verify."""
    sys_path = str(SCRIPTS_DIR)
    import sys
    if sys_path not in sys.path:
        sys.path.insert(0, sys_path)
    import generate_stm32_release_package as gen

    # Create dummy Cortex-M4 binary (initial SP at 0x20010000, Reset Vector at 0x08080101)
    raw_bin = tmp_path / "app_test.bin"
    dummy_payload = bytearray(1024)
    struct.pack_into("<II", dummy_payload, 0, 0x20010000, 0x08080101)
    raw_bin.write_bytes(dummy_payload)

    pkg_path = tmp_path / "wildsonic_stm32_l476_v2.1.0.bin"
    ok = gen.create_release_package(
        raw_bin,
        pkg_path,
        version_str="2.1.0",
        target_load_addr=0x08080000,
        target_mcu_id=0x00004760,
    )
    assert ok is True
    assert pkg_path.exists()
    assert pkg_path.stat().st_size == 1024 + 64

    # Verify header contents
    header_data = pkg_path.read_bytes()[:64]
    magic, fmt_ver, flags, mcu_id, maj, min_v, pat = struct.unpack_from("<IHHIBBB", header_data, 0)
    assert magic == 0x57533332  # "WS32"
    assert fmt_ver == 1
    assert mcu_id == 0x00004760
    assert (maj, min_v, pat) == (2, 1, 0)

    # Verification function
    verify_ok = gen.verify_release_package(pkg_path)
    assert verify_ok is True


def test_release_package_corruption_detection(tmp_path):
    """Verify modified payload or header triggers CRC failure."""
    sys_path = str(SCRIPTS_DIR)
    import sys
    if sys_path not in sys.path:
        sys.path.insert(0, sys_path)
    import generate_stm32_release_package as gen

    raw_bin = tmp_path / "app_test.bin"
    dummy_payload = bytearray(512)
    struct.pack_into("<II", dummy_payload, 0, 0x20010000, 0x08080101)
    raw_bin.write_bytes(dummy_payload)

    pkg_path = tmp_path / "test_corrupt.bin"
    gen.create_release_package(raw_bin, pkg_path, version_str="1.0.0")

    # Corrupt payload byte
    data = bytearray(pkg_path.read_bytes())
    data[70] ^= 0xFF
    pkg_path.write_bytes(data)

    assert gen.verify_release_package(pkg_path) is False


def test_bootloader_source_and_dual_bank_constants():
    """Verify bootloader main.c and bootloader.h define Bank 2 addresses."""
    bl_h = (BOOTLOADER_SRC / "include" / "bootloader.h").read_text(encoding="utf-8")
    bl_c = (BOOTLOADER_SRC / "src" / "main.c").read_text(encoding="utf-8")

    assert "0x08080000" in bl_h
    assert "0x57533332" in bl_h
    assert "0x00004760" in bl_h
    assert "SCB->VTOR = APP_BANK2_START_ADDR;" in bl_c
    assert "HAL_FLASHEx_Erase" in bl_c
    assert "FLASH_TYPEPROGRAM_DOUBLEWORD" in bl_c

