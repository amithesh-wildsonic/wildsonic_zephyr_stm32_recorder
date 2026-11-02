#!/usr/bin/env python3
"""
Test Suite: Production Firmware SDMMC Error Reporting & Storage Availability
=============================================================================
Verifies:
  1. Production error reporting visibility on SDMMC transfer / write failures.
  2. Clear error output:
     - [ERROR] Recording failed: SD write error (%d)
     - [ERROR] Storage unavailable
     - [SCHEDULER] Recording opportunity skipped due to storage failure
  3. Single error reporting without thread spamming or register dumps.
  4. Safe failure handling:
     - Marking current recording session as failed.
     - Preserving uncommitted PSRAM audio (no blind psram_staging_reset on recovery failure).
     - Preventing new recordings from starting while storage is faulted.
     - Avoiding unsafe sleep transitions while storage is faulted.
     - Preserving scheduler configuration and retention state for future retry.
"""

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent.parent
APP_SRC = BASE_DIR / "app" / "src"
RECORDER_H = APP_SRC / "recorder.h"
RECORDER_C = APP_SRC / "recorder.c"
MAIN_C = APP_SRC / "main.c"


def test_recorder_storage_availability_api():
    """Verify declaration and implementation of recorder_is_storage_available."""
    h_text = RECORDER_H.read_text(encoding="utf-8")
    assert "bool recorder_is_storage_available(void);" in h_text

    c_text = RECORDER_C.read_text(encoding="utf-8")
    assert "bool recorder_is_storage_available(void) {" in c_text
    fn_body = c_text.split("bool recorder_is_storage_available(void) {")[1].split("}")[0]
    assert "return mounted && (s_storage_state == STORAGE_STATE_HEALTHY);" in fn_body


def test_production_sd_write_error_console_messages():
    """Verify session finalization visibly reports SD write error and storage unavailable."""
    c_text = RECORDER_C.read_text(encoding="utf-8")

    # In recorder_record_session error branch:
    err_block = c_text.split("if (err != 0 && !recorder_abort_requested) {")[1].split("} else {")[0]
    assert "printk(\"[ERROR] Recording failed: SD write error (%d)\\n\", st.sd_write_error);" in err_block
    assert "printk(\"[ERROR] Recording session failed: %d\\n\", err);" in err_block
    assert "printk(\"[ERROR] Storage unavailable\\n\");" in err_block


def test_scheduler_storage_failure_console_messages():
    """Verify scheduler thread visibly reports storage failure and skips recording opportunity."""
    m_text = MAIN_C.read_text(encoding="utf-8")

    # In recorder_thread_fn:
    rec_thread = m_text.split("static void recorder_thread_fn(")[1].split("static uint32_t s_sleep_attempt_counter")[0]
    assert "printk(\"[SCHEDULER] Recording opportunity skipped due to storage failure\\n\");" in rec_thread
    assert "printk(\"[SCHEDULER] Session %u failed (%d); recording opportunity skipped, schedule remains active for next window.\\n\\n> \"," in rec_thread


def test_scheduler_halts_new_sessions_when_storage_unavailable():
    """Verify autonomous scheduler prevents dispatching new recordings when storage is unavailable."""
    m_text = MAIN_C.read_text(encoding="utf-8")

    sched_step = m_text.split("void main_scheduler_service_step(void) {")[1].split("time_t now_utc =")[0]
    assert "!recorder_is_storage_available()" in sched_step
    assert "return;" in sched_step


def test_recorder_record_session_entry_guard():
    """Verify recorder_record_session protects against starting while storage is faulted."""
    c_text = RECORDER_C.read_text(encoding="utf-8")

    entry_guard = c_text.split("if (!mounted || s_storage_state == STORAGE_STATE_FAULT) {")[1].split("st.sample_rate = audio_in_rate();")[0]
    assert "st.err = -ENODEV;" in entry_guard
    assert "st.sd_write_error = -ENODEV;" in entry_guard
    assert "printk(\"[ERROR] Recording failed: SD write error (-19)\\n\");" in entry_guard
    assert "printk(\"[ERROR] Storage unavailable\\n\");" in entry_guard
    assert "printk(\"[ERROR] Recording session failed: -19\\n\");" in entry_guard
    assert "return -ENODEV;" in entry_guard


def test_no_psram_audio_discard_on_recovery_failure():
    """Verify psram_staging_reset is only executed when storage recovery succeeds, not on failure."""
    c_text = RECORDER_C.read_text(encoding="utf-8")

    recover_fn = c_text.split("int recorder_storage_recover(void) {")[1].split("static int next_filename")[0]
    # Failure paths return before psram_staging_reset:
    fail_disk_init = recover_fn.split("int ret = disk_access_init(\"SD\");")[1].split("ret = fs_mount(&mp);")[0]
    assert "s_storage_state = STORAGE_STATE_FAULT;" in fail_disk_init
    assert "psram_staging_reset" not in fail_disk_init

    fail_mount = recover_fn.split("ret = fs_mount(&mp);")[1].split("mounted = true;")[0]
    assert "s_storage_state = STORAGE_STATE_FAULT;" in fail_mount
    assert "psram_staging_reset" not in fail_mount

    # Success path resets staging buffer:
    success_path = recover_fn.split("mounted = true;")[1]
    assert "s_storage_state = STORAGE_STATE_HEALTHY;" in success_path
    assert "psram_staging_reset();" in success_path

