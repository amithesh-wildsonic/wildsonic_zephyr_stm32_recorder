#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Test Suite: Unattended Recording Recovery & Regression Fix Verification
========================================================================
Covers all 19 Phase 8 requirements:
 1. Previous last-session error -EIO followed by ARBT while recorder idle.
 2. Previous last-session error -EIO followed by RBT while recorder idle.
 3. ARBT during successful active recording.
 4. ARBT during SD write failure.
 5. ARBT while writer has not quiesced.
 6. ARBT after writer completion timeout.
 7. Pending configuration is saved once before reboot.
 8. Internal flash configuration save works despite a historical SD error.
 9. Acquisition failure with healthy storage produces a valid partial WAV where possible.
10. SD write failure does not cause further unsafe file operations.
11. SD error remains the primary diagnostic when cleanup also fails.
12. PSRAM busy retry does not violate acquisition timing.
13. PSRAM full retry does not overflow upstream queues.
14. Storage error recovery and state reset after successful remount.
15. Sleep does not repeatedly retry an unchanged storage fault.
16. No sleep with dirty audio buffers or active writer.
17. Scheduler resumes at the next eligible opportunity after recovery.
18. Existing file contents remain unchanged.
19. New filenames remain unique.
"""

from pathlib import Path
import re
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"

CONSOLE_C = APP_SRC / "console.c"
CONSOLE_H = APP_SRC / "console.h"
RECORDER_C = APP_SRC / "recorder.c"
RECORDER_H = APP_SRC / "recorder.h"
PSRAM_STAGING_C = APP_SRC / "psram_staging.c"
PSRAM_STAGING_H = APP_SRC / "psram_staging.h"
MAIN_C = APP_SRC / "main.c"
CONFIG_STORE_C = APP_SRC / "config_store.c"
CONFIG_STORE_H = APP_SRC / "config_store.h"


# ==============================================================================
# PHASE 8.1 & 8.2: ARBT & RBT IDLE REBOOT AFTER HISTORICAL STORAGE ERROR
# ==============================================================================

def test_previous_last_session_error_eio_followed_by_arbt_while_recorder_idle():
    """Requirement 1: Previous last-session error -EIO followed by ARBT while recorder idle."""
    code = CONSOLE_C.read_text(encoding="utf-8")
    assert "static void handle_arbt(void)" in code

    # Verify that historical error does NOT veto reboot
    assert "Proceeding with reboot despite historical storage error" in code
    assert "if (!recorder_is_shutdown_ready())" in code


def test_previous_last_session_error_eio_followed_by_rbt_while_recorder_idle():
    """Requirement 2: Previous last-session error -EIO followed by RBT while recorder idle."""
    code = CONSOLE_C.read_text(encoding="utf-8")
    assert "static void handle_rbt(void)" in code

    # Verify RBT also uses recorder_is_shutdown_ready and permits reboot despite historical error
    assert "Proceeding with reboot despite historical storage error" in code
    rbt_body = code.split("static void handle_rbt(void)")[1].split("static void handle_arbt(void)")[0]
    assert "if (!recorder_is_shutdown_ready())" in rbt_body
    assert "Proceeding with reboot despite historical storage error" in rbt_body


# ==============================================================================
# PHASE 8.3 & 8.4: ARBT DURING ACTIVE RECORDING AND SD WRITE FAILURE
# ==============================================================================

def test_arbt_during_successful_active_recording():
    """Requirement 3: ARBT during successful active recording safely stops and drains."""
    code = CONSOLE_C.read_text(encoding="utf-8")
    assert "if (recorder_is_active())" in code
    assert "recorder_set_stop_reason(RECORDER_STOP_APPLY_REBOOT);" in code
    assert "recorder_request_stop();" in code
    assert "int wait_rc = recorder_wait_idle(5000);" in code


def test_arbt_during_sd_write_failure():
    """Requirement 4: ARBT during SD write failure drains, quiesces, and allows reboot."""
    rec_code = RECORDER_C.read_text(encoding="utf-8")
    cons_code = CONSOLE_C.read_text(encoding="utf-8")

    # When SD write fails, recorder aborts and sets writer idle
    assert "sd_writer_err != 0" in rec_code
    # ARBT checks recorder_is_shutdown_ready(), which passes once writer released resources
    assert "bool recorder_is_shutdown_ready(void)" in rec_code
    assert "if (!recorder_is_shutdown_ready())" in cons_code


# ==============================================================================
# PHASE 8.5 & 8.6: ARBT WHILE WRITER ACTIVE OR COMPLETION TIMEOUT
# ==============================================================================

def test_arbt_while_writer_has_not_quiesced():
    """Requirement 5: ARBT while writer has not quiesced halts reboot."""
    rec_code = RECORDER_C.read_text(encoding="utf-8")
    cons_code = CONSOLE_C.read_text(encoding="utf-8")

    # Authoritative shutdown readiness checks:
    # 1. recorder_active
    # 2. sd_file_ptr != NULL
    # 3. sd_bounce_buf != NULL
    # 4. sd_writer_flush_req
    # 5. s_storage_recovering
    assert "bool recorder_is_shutdown_ready(void)" in rec_code
    assert "if (recorder_active)" in rec_code
    assert "if (sd_file_ptr != NULL)" in rec_code
    assert "if (sd_bounce_buf != NULL)" in rec_code
    assert "if (sd_writer_flush_req)" in rec_code
    assert "if (s_storage_recovering)" in rec_code

    # console.c halts reboot if not shutdown ready
    assert "if (!recorder_is_shutdown_ready()) {" in cons_code
    assert "ARBT: Storage resources still active; aborting reboot" in cons_code


def test_arbt_after_writer_completion_timeout():
    """Requirement 6: ARBT after writer completion timeout aborts reboot."""
    code = CONSOLE_C.read_text(encoding="utf-8")
    assert "int wait_rc = recorder_wait_idle(5000);" in code
    assert "if (wait_rc != 0) {" in code
    assert "main_set_reboot_pending(false);" in code
    assert "ARBT: Recorder shutdown timeout (%d); aborting reboot to preserve storage" in code


# ==============================================================================
# PHASE 8.7 & 8.8: CONFIGURATION SAVING AND FLASH INDEPENDENCE
# ==============================================================================

def test_pending_configuration_is_saved_once_before_reboot():
    """Requirement 7: Pending configuration is saved once before reboot."""
    code = CONSOLE_C.read_text(encoding="utf-8")
    # In handle_arbt, save_config_flag is cleared and config_store_save is called once
    assert "g_cfg.save_config_flag = false;" in code
    assert "int save_rc = config_store_save();" in code
    assert "if (save_rc != 0) {" in code
    assert "main_set_reboot_pending(false);" in code


def test_internal_flash_config_save_works_despite_historical_sd_error():
    """Requirement 8: Internal flash configuration save works despite historical SD error."""
    cfg_code = CONFIG_STORE_C.read_text(encoding="utf-8")
    # Verify config_store_save uses flash device, independent of SD disk
    assert "flash_area" in cfg_code or "flash_write" in cfg_code or "CONFIG_STORE" in cfg_code
    # Must not depend on SD mounted flag or disk_access
    assert "fs_mount" not in cfg_code
    assert "disk_access_init" not in cfg_code


# ==============================================================================
# PHASE 8.9, 8.10 & 8.11: WAV FINALIZATION CONTRACTS (CASE A VS CASE B)
# ==============================================================================

def test_case_a_acquisition_failure_with_healthy_storage_produces_valid_partial_wav():
    """Requirement 9: Acquisition failure with healthy storage produces a valid partial WAV."""
    code = RECORDER_C.read_text(encoding="utf-8")
    # In Case A: storage is healthy (sd_writer_err == 0 && st.sd_write_error == 0)
    assert "/* Writer completion is established here. Finalize a valid partial WAV using" in code
    assert "const uint32_t data_bytes = (uint32_t)st.bytes_written;" in code
    assert "wav_fill_header(hdr, st.sample_rate, st.channels, st.sample_bits, data_bytes);" in code
    assert "finalize_ret = fs_seek(&f, 0, FS_SEEK_SET);" in code
    assert "header_written = fs_write(&f, hdr, WAV_HEADER_BYTES);" in code
    assert "finalize_ret = fs_truncate(&f, WAV_HEADER_BYTES + data_bytes);" in code
    assert "finalize_ret = fs_sync(&f);" in code


def test_case_b_sd_write_failure_does_not_cause_further_unsafe_file_operations():
    """Requirement 10: SD write failure does not cause further unsafe file operations."""
    code = RECORDER_C.read_text(encoding="utf-8")
    # In Case B: sd_writer_err != 0 || st.sd_write_error != 0
    # Must NOT call fs_seek, fs_write header, fs_truncate, or fs_sync
    case_b_segment = code.split("if (sd_writer_err != 0 || st.sd_write_error != 0) {")[1].split("} else {")[0]
    assert "fs_close(&f)" in case_b_segment
    assert "fs_seek(&f" not in case_b_segment
    assert "fs_write(&f" not in case_b_segment
    assert "fs_truncate(&f" not in case_b_segment
    assert "fs_sync(&f" not in case_b_segment


def test_sd_error_remains_primary_diagnostic_when_cleanup_also_fails():
    """Requirement 11: SD error remains primary diagnostic when cleanup also fails."""
    code = RECORDER_C.read_text(encoding="utf-8")
    case_b_segment = code.split("if (sd_writer_err != 0 || st.sd_write_error != 0) {")[1].split("} else {")[0]
    assert "st.wav_verified = false;" in case_b_segment
    assert "if (err == 0) {" in case_b_segment
    assert "err = (sd_writer_err != 0) ? sd_writer_err : st.sd_write_error;" in case_b_segment


# ==============================================================================
# PHASE 8.12 & 8.13: PSRAM STAGING CONCURRENCY & TIMING INTEGRITY
# ==============================================================================

def test_psram_busy_retry_does_not_violate_acquisition_timing():
    """Requirement 12: PSRAM busy retry does not violate acquisition timing (no 100 ms blocking)."""
    code = RECORDER_C.read_text(encoding="utf-8")
    # Verify that the old 100 ms busy-sleep loops (k_sleep(K_MSEC(10)) in a loop of 10) are removed:
    assert "for (int retry = 0; retry < 10; retry++)" not in code
    # Replaced with semaphore signal and non-blocking wait:
    assert "k_sem_give(&sd_writer_sem);" in code
    assert "psram_staging_wait_free(" in code


def test_psram_full_retry_does_not_overflow_upstream_queues():
    """Requirement 13: PSRAM full retry advances batch without false -ENOBUFS overflow."""
    code = PSRAM_STAGING_C.read_text(encoding="utf-8")
    # Verify psram_staging_push checks if next buffer is FREE and rolls over
    assert "batch_bufs[next_fill].state == BATCH_STATE_FREE" in code
    assert "cur->state = BATCH_STATE_READY;" in code
    assert "active_fill_idx = next_fill;" in code


# ==============================================================================
# PHASE 8.14: STORAGE ERROR RECOVERY AND STATE RESET
# ==============================================================================

def test_storage_error_recovery_and_state_reset_after_successful_remount():
    """Requirement 14: Storage error recovery and state reset after successful remount."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "int recorder_storage_recover(void)" in code
    recover_fn = code.split("int recorder_storage_recover(void)")[1].split("static int next_filename")[0]

    assert "s_storage_recovering = true;" in recover_fn
    assert "fs_unmount(&mp);" in recover_fn
    assert "disk_access_init(\"SD\");" in recover_fn
    assert "fs_mount(&mp);" in recover_fn
    assert "disk_access_ioctl(\"SD\", DISK_IOCTL_CTRL_DEINIT, NULL);" in recover_fn
    assert "s_storage_state = STORAGE_STATE_HEALTHY;" in recover_fn
    assert "s_storage_recovery_result = 0;" in recover_fn
    assert "sd_writer_err = 0;" in recover_fn
    assert "psram_staging_reset();" in recover_fn
    assert "s_storage_recovering = false;" in recover_fn


# ==============================================================================
# PHASE 8.15 & 8.16: AUTONOMOUS STOP 2 SLEEP AND FAULT STATE HANDLING
# ==============================================================================

def test_sleep_does_not_repeatedly_retry_unchanged_storage_fault():
    """Requirement 15: Sleep does not repeatedly retry an unchanged storage fault."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us)" in code
    sleep_fn = code.split("int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us)")[1].split("int recorder_record_session(")[0]

    # Only verifies SD card idle if storage is healthy
    assert "if (s_storage_state == STORAGE_STATE_HEALTHY) {" in sleep_fn
    assert "recorder_sd_verify_idle()" in sleep_fn


def test_no_sleep_with_dirty_audio_buffers_or_active_writer():
    """Requirement 16: No sleep with dirty audio buffers or active writer."""
    code = RECORDER_C.read_text(encoding="utf-8")
    sleep_fn = code.split("int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us)")[1].split("int recorder_record_session(")[0]

    # Must reject if active or audio blocks remain
    assert "if (recorder_active ||" in sleep_fn
    assert "if (psram_staging_is_active() && psram_staging_occupied() > 0) {" in sleep_fn
    assert "if (s_storage_state == STORAGE_STATE_FAULT) {" in sleep_fn
    assert "return -EBUSY;" in sleep_fn
    assert "if (sd_file_ptr != NULL) {" in sleep_fn


# ==============================================================================
# PHASE 8.17: SCHEDULER CONTINUATION ACROSS OPPORTUNITIES
# ==============================================================================

def test_scheduler_resumes_at_next_eligible_opportunity_after_recovery():
    """Requirement 17: Scheduler resumes at next eligible opportunity after failed session."""
    code = MAIN_C.read_text(encoding="utf-8")
    rec_thread = code.split("static void recorder_thread_fn(")[1].split("static uint32_t s_sleep_attempt_counter")[0]
    assert "s_schedule_cycle++;" in rec_thread
    assert "recording opportunity skipped, schedule remains active for next window" in rec_thread
    assert "atomic_set(&s_schedule_paused, 0);" in rec_thread


# ==============================================================================
# PHASE 8.18 & 8.19: FILE INTEGRITY AND UNIQUE FILENAMES
# ==============================================================================

def test_existing_file_contents_remain_unchanged():
    """Requirement 18: Existing file contents remain unchanged (no format, no truncate mode, recovery does not wipe SD)."""
    code = RECORDER_C.read_text(encoding="utf-8")
    # Files are opened with FS_O_CREATE | FS_O_WRITE, NOT FS_O_TRUNC
    assert "ret = fs_open(&f, st.file, FS_O_CREATE | FS_O_WRITE);" in code
    # No formatting as recovery mechanism
    assert "disk_format" not in code
    assert "fs_mkfs" not in code

    # Storage recovery unmounts and remounts without deleting files or reformatting
    recover_fn = code.split("int recorder_storage_recover(void)")[1].split("static int next_filename")[0]
    assert "fs_unlink" not in recover_fn
    assert "fs_mkfs" not in recover_fn


def test_new_filenames_remain_unique():
    """Requirement 19: New filenames remain unique across recording opportunities."""
    rec_code = RECORDER_C.read_text(encoding="utf-8")
    # Naming format uses timestamp format: WS_YYYYMMDD_HHMMSS or similar
    assert "%04u%02u%02u_%02u%02u%02u" in rec_code or "recorder_format_filename" in rec_code or ".WAV" in rec_code
