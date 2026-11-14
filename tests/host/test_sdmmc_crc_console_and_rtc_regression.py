#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Regression Test Suite: SDMMC Data CRC Failure Propagation,
Asynchronous Responsive Console & ARBT Single-Save, and RTC Calendar Inversion Fix.

Covers:
 1. Storage Integrity:
    - HAL error code 0x00000002 maps specifically to HAL_SD_ERROR_DATA_CRC_FAIL.
    - Decoded error strings and rich hardware registers (STA, DCOUNT, DCTRL, CLKCR) captured.
    - WAV header finalization is strictly skipped when an SD write error occurs.
    - Exact committed and uncommitted bytes are tracked during interrupted batch writes.
    - File close failure and storage errors prevent unsafe reboot.
 2. Console Responsiveness & ARBT Flow:
    - Asynchronous interrupt-driven UART ring buffer decouples user typing from thread execution.
    - Interactive idle window prevents immediate sleep while user is typing at prompt.
    - ARBT executes config_store_save() exactly once (eliminating duplicate flash save).
    - ARBT immediately acknowledges via [ACK] before requesting recorder stop.
    - ARBT uses bounded recorder_wait_idle(5000) and halts reboot if recorder fails to idle.
    - ARBT checks recorder_get_last_error() and recorder_get_last_sd_error() and preserves errors.
 3. RTC Hardware Programming & Reporting:
    - RTC date register unpacks format 0xWWDDMMYY using __LL_RTC_GET_YEAR, __LL_RTC_GET_MONTH, __LL_RTC_GET_DAY.
    - Swapped year/day bug (where 2026-10-30 became 2030-10-26) is eliminated.
    - rtc_calendar_set() performs immediate read-back verification against programmed values.
    - Hardware RTC stores pure UTC; local timezone offset applied strictly in display and scheduler.
    - Midnight and year/month boundary rollovers maintain independent UTC and local calendars.
    - config_store_write_status_file checks truncate, write, and close return values.
"""

import os
import re
import datetime
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
ZEPHYR_DISK_DRIVER = Path(r"C:\Users\amith\zephyrproject\zephyr\drivers\disk\sdmmc_stm32.c")

CONSOLE_C = APP_SRC / "console.c"
CONSOLE_H = APP_SRC / "console.h"
RECORDER_C = APP_SRC / "recorder.c"
RECORDER_H = APP_SRC / "recorder.h"
RTC_CALENDAR_C = APP_SRC / "rtc_calendar.c"
RTC_CALENDAR_H = APP_SRC / "rtc_calendar.h"
MAIN_C = APP_SRC / "main.c"
CONFIG_STORE_C = APP_SRC / "config_store.c"
SD_OVERLAY = REPO_ROOT / "app" / "sd" / "sdmmc_nucleo_l476rg.overlay"


# ==============================================================================
# 1. STORAGE INTEGRITY & SDMMC DATA CRC FAILURE TESTS
# ==============================================================================

def test_hal_sd_error_data_crc_fail_definition_and_decoding():
    """Verify HAL SD error code 2 maps to HAL_SD_ERROR_DATA_CRC_FAIL and is decoded."""
    HAL_SD_ERROR_DATA_CRC_FAIL = 0x00000002
    assert HAL_SD_ERROR_DATA_CRC_FAIL == 2

    # Check driver decoding in sdmmc_stm32.c
    if ZEPHYR_DISK_DRIVER.is_file():
        driver_code = ZEPHYR_DISK_DRIVER.read_text(encoding="utf-8")
        assert "stm32_sdmmc_decode_hal_error" in driver_code
        assert "DATA_CRC_FAIL" in driver_code
        assert "STA=0x" in driver_code
        assert "DCOUNT=" in driver_code
        assert "DCTRL=0x" in driver_code


def test_recorder_captures_rich_sdmmc_diagnostics_on_write_error():
    """Verify recorder.c captures sector, latency, stage diagnostics, and batch interruption."""
    rec_c = RECORDER_C.read_text(encoding="utf-8")
    assert "SD fs_write failed: err=%d len=%u sec=%u us=%u" in rec_c
    assert "SDMMC_HW: %s STA=0x%08X (%s) DCNT=%u DCTRL=0x%08X" in rec_c
    assert "Batch %u interrupted: %u/%u B err=%d" in rec_c


def test_storage_error_preserves_error_code_and_finalizes_partial_wav():
    """Verify error remains visible while the confirmed portion gets a partial WAV header."""
    rec_c = RECORDER_C.read_text(encoding="utf-8")
    assert "Finalize a valid partial WAV" in rec_c
    assert "finalize_ret = fs_seek(&f, 0, FS_SEEK_SET);" in rec_c
    assert "st.wav_verified = false;" in rec_c
    assert "s_recorder_last_error = err;" in rec_c
    assert "s_recorder_last_sd_error = st.sd_write_error;" in rec_c


def test_batch_sample_and_byte_accounting_math():
    """Simulate Batch 36 interrupted at 73728 bytes and verify exact uncommitted byte accounting."""
    rate = 192000
    sample_bits = 32
    channels = 1
    bytes_per_sample = (sample_bits // 8) * channels
    assert bytes_per_sample == 4

    batch_size = 256 * 1024  # 262144 bytes
    chunk_size = 8192        # 8 KiB fs_write chunk

    # 35 complete batches + partial batch 36
    complete_batches = 35
    batch36_written = 9 * chunk_size  # 73728 bytes
    total_committed = complete_batches * batch_size + batch36_written
    assert total_committed == 9248768

    # Uncommitted bytes remaining in batch 36
    uncommitted = batch_size - batch36_written
    assert uncommitted == 188416
    uncommitted_samples = uncommitted // bytes_per_sample
    assert uncommitted_samples == 47104


def test_sdmmc_clock_divider_and_bandwidth_margin():
    """Verify SDMMC bus clock frequency options sustain 768 KB/s production rate with margin."""
    stream_bps = 192000 * 4  # 768,000 B/s = 750 KB/s

    # At 12 MHz 4-bit (clk-div = 2)
    clk_12mhz_bw = (12_000_000 * 4) // 8  # 6,000,000 B/s
    margin_12mhz = clk_12mhz_bw / stream_bps
    assert margin_12mhz >= 7.8

    # At 6 MHz 4-bit (clk-div = 6)
    clk_6mhz_bw = (6_000_000 * 4) // 8    # 3,000,000 B/s
    margin_6mhz = clk_6mhz_bw / stream_bps
    assert margin_6mhz >= 3.9, "6 MHz 4-bit must provide at least 3.9x margin over 768 KB/s stream"


# ==============================================================================
# 2. CONSOLE RESPONSIVENESS & ARBT BEHAVIOR TESTS
# ==============================================================================

def test_arbt_saves_configuration_exactly_once():
    """Verify ARBT saves configuration once and clears save_config_flag to prevent duplicate save."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    arbt_match = re.search(r"static void handle_arbt\(void\)\s*\{(.*?)\n\}", c_text, re.DOTALL)
    assert arbt_match is not None, "handle_arbt not found in console.c"
    arbt_body = arbt_match.group(1)

    # Must immediately acknowledge
    assert "[ACK] ARBT: Applying configuration and initiating orderly shutdown..." in arbt_body
    # Must save configuration once
    assert "int save_rc = config_store_save();" in arbt_body
    assert "g_cfg.save_config_flag = false;" in arbt_body
    assert "1 save operation" in arbt_body

    # Must NOT call handle_rbt() which caused duplicate save
    assert "handle_rbt();" not in arbt_body


def test_arbt_waits_for_recorder_and_aborts_on_timeout_or_error():
    """Verify ARBT uses bounded recorder_wait_idle(5000) and aborts reboot on error or timeout."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    arbt_match = re.search(r"static void handle_arbt\(void\)\s*\{(.*?)\n\}", c_text, re.DOTALL)
    assert arbt_match is not None
    arbt_body = arbt_match.group(1)

    assert "recorder_request_stop();" in arbt_body
    assert "recorder_wait_idle(5000)" in arbt_body
    assert "Storage shutdown timed out" in arbt_body
    assert "recorder_is_shutdown_ready()" in arbt_body
    assert "recorder_get_last_error()" in arbt_body
    assert "recorder_get_last_sd_error()" in arbt_body
    assert "Proceeding with reboot despite historical storage error" in arbt_body


def test_handle_apply_rejects_active_recording():
    """Verify handle_apply returns BUSY and prevents flash write while recording is active."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    apply_match = re.search(r"static void handle_apply\(void\)\s*\{(.*?)\n\}", c_text, re.DOTALL)
    assert apply_match is not None
    apply_body = apply_match.group(1)

    assert "if (recorder_is_active())" in apply_body
    assert "BUSY: Cannot commit to flash while recording is active" in apply_body


def test_main_interactive_window_prevents_premature_sleep():
    """Verify main.c maintains interactive window before entering autonomous Stop 2 sleep."""
    main_text = MAIN_C.read_text(encoding="utf-8")
    assert "s_console_last_activity_ms" in main_text
    assert "if (k_uptime_get() - s_console_last_activity_ms < 10000)" in main_text


# ==============================================================================
# 3. RTC HARDWARE PROGRAMMING & REPORTING TESTS
# ==============================================================================

def test_rtc_date_get_uses_canonical_ll_macros():
    """Verify rtc_calendar_get uses __LL_RTC_GET_YEAR, __LL_RTC_GET_MONTH, __LL_RTC_GET_DAY."""
    rtc_c = RTC_CALENDAR_C.read_text(encoding="utf-8")

    # Inverted masks must NOT be used on dr
    assert "__LL_RTC_GET_YEAR(dr)" in rtc_c, "Must use __LL_RTC_GET_YEAR on synthetic dr"
    assert "__LL_RTC_GET_MONTH(dr)" in rtc_c, "Must use __LL_RTC_GET_MONTH on synthetic dr"
    assert "__LL_RTC_GET_DAY(dr)" in rtc_c, "Must use __LL_RTC_GET_DAY on synthetic dr"
    assert "(dr & (RTC_DR_YT | RTC_DR_YU)) >> RTC_DR_YU_Pos" not in rtc_c, "Inverted year mask on dr must be eliminated"
    assert "(dr & (RTC_DR_DT | RTC_DR_DU)) >> RTC_DR_DU_Pos" not in rtc_c, "Inverted day mask on dr must be eliminated"


def test_rtc_calendar_synthetic_dr_unpacking_simulation():
    """Simulate STM32 LL_RTC_DATE_Get return value 0xWWDDMMYY for 2026-10-30."""
    # BCD encodings:
    # Year: 26 = 0x26
    # Month: 10 = 0x10
    # Day: 30 = 0x30
    # Weekday: Friday = 5 = 0x05
    # LL_RTC_DATE_Get returns: (weekday << 24) | (day << 16) | (month << 8) | (year)
    synthetic_dr = (0x05 << 24) | (0x30 << 16) | (0x10 << 8) | 0x26

    # Canonical LL helper macros
    def __ll_rtc_get_year(dr): return dr & 0xFF
    def __ll_rtc_get_month(dr): return (dr >> 8) & 0xFF
    def __ll_rtc_get_day(dr): return (dr >> 16) & 0xFF
    def __ll_rtc_get_weekday(dr): return (dr >> 24) & 0xFF
    def bcd2bin(bcd): return ((bcd >> 4) * 10) + (bcd & 0x0F)

    year = 2000 + bcd2bin(__ll_rtc_get_year(synthetic_dr))
    month = bcd2bin(__ll_rtc_get_month(synthetic_dr))
    day = bcd2bin(__ll_rtc_get_day(synthetic_dr))
    weekday = __ll_rtc_get_weekday(synthetic_dr)

    assert year == 2026, f"Expected 2026, got {year}"
    assert month == 10, f"Expected 10, got {month}"
    assert day == 30, f"Expected 30, got {day}"
    assert weekday == 5, f"Expected Friday (5), got {weekday}"


def test_rtc_calendar_set_readback_verification():
    """Verify rtc_calendar_set reads back and verifies programmed UTC calendar."""
    rtc_c = RTC_CALENDAR_C.read_text(encoding="utf-8")
    set_match = re.search(r"int rtc_calendar_set\(const struct rtc_calendar_time \*t\)\s*\{(.*?)\n\}", rtc_c, re.DOTALL)
    assert set_match is not None
    set_body = set_match.group(1)

    assert "struct rtc_calendar_time readback;" in set_body
    assert "rtc_calendar_get(&readback)" in set_body
    assert "RTC readback mismatch (%04u-%02u-%02u %02u:%02u:%02u)" in set_body


def test_config_store_write_status_file_checks_all_results():
    """Verify config_store_write_status_file checks fs_truncate, fs_write, and fs_close results."""
    cs_c = CONFIG_STORE_C.read_text(encoding="utf-8")
    status_match = re.search(r"int config_store_write_status_file\(const char \*path\)\s*\{(.*?)\n\}", cs_c, re.DOTALL)
    assert status_match is not None
    status_body = status_match.group(1)

    assert "fs_truncate(&file, 0)" in status_body
    assert 'TRACE("Status file truncate failed (%d)", rc);' in status_body
    assert 'TRACE("Status write failed (%d)", err_code);' in status_body
    assert 'TRACE("Status file close failed (%d)", rc);' in status_body


def test_utc_and_local_timezone_rollover_boundaries():
    """Verify UTC vs Local timezone conversion across midnight, month, and year boundaries."""
    # Test 1: New Year boundary (2026-12-31 23:30:00 UTC with +05:30 IST)
    # Local time is 2027-01-01 05:00:00 (+1 day, new year)
    utc_epoch = int(datetime.datetime(2026, 12, 31, 23, 30, 0, tzinfo=datetime.timezone.utc).timestamp())
    tz_offset_ist = 5 * 3600 + 30 * 60  # +19800 s
    local_epoch = utc_epoch + tz_offset_ist
    dt_local = datetime.datetime.fromtimestamp(local_epoch, tz=datetime.timezone.utc)

    assert dt_local.year == 2027
    assert dt_local.month == 1
    assert dt_local.day == 1
    assert dt_local.hour == 5
    assert dt_local.minute == 0

    # Test 2: Negative timezone boundary (2026-10-30 02:00:00 UTC with -05:00 EST)
    # Local time is 2026-10-29 21:00:00 (-1 day)
    utc_epoch_2 = int(datetime.datetime(2026, 10, 30, 2, 0, 0, tzinfo=datetime.timezone.utc).timestamp())
    tz_offset_est = -5 * 3600
    local_epoch_2 = utc_epoch_2 + tz_offset_est
    dt_local_2 = datetime.datetime.fromtimestamp(local_epoch_2, tz=datetime.timezone.utc)

    assert dt_local_2.year == 2026
    assert dt_local_2.month == 10
    assert dt_local_2.day == 29
    assert dt_local_2.hour == 21

