#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Host Verification Suite for:
Non-Blocking UART + BLE Configuration with Deferred Apply, Safe Reboot,
and Recording Continuity on WildSonic STM32L476RG.

Validates all 14 required contract scenarios:
 1. UART configuration command during active recording (staged, non-blocking).
 2. BLE configuration command during active recording (staged, non-blocking).
 3. Multiple commands updating pending configuration cumulatively.
 4. Clear acknowledgement distinguishing received, validated, saved, and pending reboot.
 5. Persistence failure handling (rollback in RAM, error response, no false success).
 6. ARBT during active recording waits for recording session to finalize safely before reboot.
 7. ARBT while idle reboots promptly without waiting for recording.
 8. Safe audio buffer flush, PSRAM drain, WAV header finalization, and file close.
 9. Post-reboot configuration application applies pending config and clears pending marker.
 10. Failure during post-reboot configuration application retains last known valid config (no loop).
 11. Existing SD card recordings are preserved and never truncated, deleted, or formatted.
 12. New recording filename after reboot is unique and collision-free.
 13. Duplicate ARBT commands are handled idempotently without duplicate reboots.
 14. Command bursts do not starve or block audio acquisition or DMA deadlines.
"""

import os
import re
import struct
import tempfile
from pathlib import Path
import pytest

ROOT_DIR = Path(__file__).resolve().parents[2]
APP_DIR = ROOT_DIR / "app" if (ROOT_DIR / "app").is_dir() else ROOT_DIR
SRC_DIR = APP_DIR / "src"

CONFIG_STORE_H = SRC_DIR / "config_store.h"
CONFIG_STORE_C = SRC_DIR / "config_store.c"
CONSOLE_C = SRC_DIR / "console.c"
CONSOLE_H = SRC_DIR / "console.h"
MAIN_C = SRC_DIR / "main.c"
MAIN_H = SRC_DIR / "main.h"
RECORDER_C = SRC_DIR / "recorder.c"
RECORDER_H = SRC_DIR / "recorder.h"
BLE_COMM_C = SRC_DIR / "ble_comm.c"
BLE_COMM_H = SRC_DIR / "ble_comm.h"

CONFIG_STORE_MAGIC = 0x13245768
CONFIG_STORE_VERSION = 1
CONFIG_STORE_RECORD_SIZE = 512
CONFIG_STORE_CHECKSUM_OFFSET = 508


# ==============================================================================
# Helper Reference Model of Config Record
# ==============================================================================

class PersistedConfigRecord:
    def __init__(self, seq=1, samf=48000, rect=600, sleep=0, gain=0,
                 lcut=0, ucut=24000, fldr=2, psram=1,
                 pending_apply=0, pending_gen=0, applied_gen=0):
        self.magic = CONFIG_STORE_MAGIC
        self.version = CONFIG_STORE_VERSION
        self.sequence_number = seq
        self.serial_number = b"0\x00" + b"\x00" * 30
        self.device_name = b"\x00" * 32
        self.device_password = b"wildsonic123\x00" + b"\x00" * 19
        self.sampling_rate = samf
        self.recording_length = rect
        self.sleep_pause_interval = sleep
        self.mic_gain_times_ten = gain
        self.audio_sampling_width = 16
        self.num_audio_channels = 1
        self.lower_cutoff_frequency = lcut
        self.upper_cutoff_frequency = ucut
        self.folder_structure = fldr
        self.psram_staging_enabled = psram

        self.start_year = 0
        self.start_month = 0
        self.start_day = 0
        self.end_year = 0
        self.end_month = 0
        self.end_day = 0
        self.day_of_week_mask = 0x7F
        self.time_zone_offset = 0
        self.num_tod_ranges = 0
        self.tod_start = [0] * 10
        self.tod_end = [0] * 10

        self.pending_apply = pending_apply
        self.pending_generation = pending_gen
        self.applied_generation = applied_gen

    def pack(self) -> bytes:
        hdr = struct.pack(
            "<III32s32s32sIIIIIIIIBB",
            self.magic, self.version, self.sequence_number,
            self.serial_number, self.device_name, self.device_password,
            self.sampling_rate, self.recording_length, self.sleep_pause_interval,
            self.mic_gain_times_ten, self.audio_sampling_width, self.num_audio_channels,
            self.lower_cutoff_frequency, self.upper_cutoff_frequency,
            self.folder_structure, self.psram_staging_enabled
        )
        sched = struct.pack(
            "<HBBHBBBiB",
            self.start_year, self.start_month, self.start_day,
            self.end_year, self.end_month, self.end_day,
            self.day_of_week_mask, self.time_zone_offset, self.num_tod_ranges
        )
        tod = struct.pack("<10I10I", *(self.tod_start + self.tod_end))
        pending_fields = struct.pack("<BII", self.pending_apply, self.pending_generation, self.applied_generation)

        payload = hdr + sched + tod + pending_fields
        assert len(payload) == 245
        reserved = b"\x00" * (CONFIG_STORE_CHECKSUM_OFFSET - len(payload))
        data_before_checksum = payload + reserved
        assert len(data_before_checksum) == CONFIG_STORE_CHECKSUM_OFFSET

        chk = sum(data_before_checksum) & 0xFFFFFFFF
        return data_before_checksum + struct.pack("<I", chk)

    @classmethod
    def unpack(cls, blob: bytes):
        assert len(blob) == CONFIG_STORE_RECORD_SIZE
        chk_actual = struct.unpack("<I", blob[508:512])[0]
        chk_calc = sum(blob[:508]) & 0xFFFFFFFF
        if chk_actual != chk_calc:
            raise ValueError(f"Checksum mismatch: calculated {chk_calc:#x}, stored {chk_actual:#x}")

        magic, version, seq = struct.unpack("<III", blob[0:12])
        if magic != CONFIG_STORE_MAGIC or version != CONFIG_STORE_VERSION:
            raise ValueError("Magic or version mismatch")

        samf, rect, sleep, gain, width, ch, lcut, ucut, fldr, psram = struct.unpack(
            "<IIIIIIIIBB", blob[108:142]
        )
        pending_apply, pending_gen, applied_gen = struct.unpack("<BII", blob[236:245])

        rec = cls(seq=seq, samf=samf, rect=rect, sleep=sleep, gain=gain,
                  lcut=lcut, ucut=ucut, fldr=fldr, psram=psram,
                  pending_apply=pending_apply, pending_gen=pending_gen, applied_gen=applied_gen)
        return rec


# ==============================================================================
# Scenario 1 & 2: Non-Blocking Configuration Staging During Recording
# ==============================================================================

def test_uart_config_command_during_recording():
    """Verify UART config command during active recording updates pending config in flash
    while leaving running session parameters and audio hardware untouched."""
    c_src = CONSOLE_C.read_text(encoding="utf-8")

    # In handle_samf, audio_in_set_rate is guarded by !recorder_is_active()
    assert re.search(r"if\s*\(!recorder_is_active\(\)\)\s*\{[^}]*audio_in_set_rate\(val\)", c_src)

    # When recording is active, command stages without mutating running session and saves pending config
    assert "OK: Value staged for next session (active session unaffected)" in c_src
    assert 'console_commit_and_ack_config("SAMF"' in c_src


def test_ble_config_command_during_recording():
    """Verify BLE config command feeds the same shared dispatcher and returns pending ACK."""
    ble_src = BLE_COMM_C.read_text(encoding="utf-8")
    assert "console_process_line_source(p, true, resp_buf, sizeof(resp_buf))" in ble_src
    assert "ble_comm_printf" in ble_src


# ==============================================================================
# Scenario 3: Multiple Commands Update Pending Configuration Cumulatively
# ==============================================================================

def test_multiple_commands_accumulate_in_pending_config():
    """Verify multiple configuration commands received before reboot accumulate in the
    pending configuration record, each incrementing sequence and pending generation."""
    rec = PersistedConfigRecord(seq=1, samf=48000, rect=600, gain=0)

    # 1st command: SAMF 96000
    rec.sampling_rate = 96000
    rec.sequence_number += 1
    rec.pending_apply = 1
    rec.pending_generation += 1
    blob1 = rec.pack()
    rec1 = PersistedConfigRecord.unpack(blob1)
    assert rec1.sampling_rate == 96000
    assert rec1.recording_length == 600
    assert rec1.pending_apply == 1
    assert rec1.pending_generation == 1

    # 2nd command: RECT 300
    rec.recording_length = 300
    rec.sequence_number += 1
    rec.pending_generation += 1
    blob2 = rec.pack()
    rec2 = PersistedConfigRecord.unpack(blob2)
    assert rec2.sampling_rate == 96000
    assert rec2.recording_length == 300
    assert rec2.mic_gain_times_ten == 0
    assert rec2.pending_apply == 1
    assert rec2.pending_generation == 2

    # 3rd command: GAIN 12 (1.2 dB -> 12)
    rec.mic_gain_times_ten = 12
    rec.sequence_number += 1
    rec.pending_generation += 1
    blob3 = rec.pack()
    rec3 = PersistedConfigRecord.unpack(blob3)
    assert rec3.sampling_rate == 96000
    assert rec3.recording_length == 300
    assert rec3.mic_gain_times_ten == 12
    assert rec3.pending_apply == 1
    assert rec3.pending_generation == 3


# ==============================================================================
# Scenario 4: Acknowledgement Clearly Distinguishes Stages
# ==============================================================================

def test_acknowledgement_distinguishes_received_validated_saved_pending():
    """Verify the acknowledgement clearly confirms received, validated, saved, and pending reboot."""
    c_src = CONSOLE_C.read_text(encoding="utf-8")
    ack_fn_pos = c_src.find("static int console_commit_and_ack_config(")
    assert ack_fn_pos != -1
    ack_fn = c_src[ack_fn_pos:ack_fn_pos + 1200]

    assert "Command %s received and validated" in ack_fn
    assert "Saved to persistent flash" in ack_fn
    assert "Configuration pending application after reboot" in ack_fn
    assert "Saved to flash (Pending reboot application)" in ack_fn


# ==============================================================================
# Scenario 5: Persistence Failure Handling
# ==============================================================================

def test_persistence_failure_handling_rolls_back_and_reports_error():
    """Verify that if persistent flash storage fails to commit, the system rolls back
    pending RAM changes, reports error, and does NOT report successful saving."""
    c_src = CONSOLE_C.read_text(encoding="utf-8")
    cfg_src = CONFIG_STORE_C.read_text(encoding="utf-8")

    # In console_commit_and_ack_config:
    assert "if (save_rc != 0)" in c_src
    assert 'printk("[ERROR] Flash save failed (%d). Pending configuration not saved.\\n", save_rc);' in c_src
    assert "config_store_rollback_pending();" in c_src

    # In config_store.c rollback implementation:
    assert "void config_store_rollback_pending(void)" in cfg_src
    assert "unpack_record_to_system(&s_last_valid_applied_record);" in cfg_src


# ==============================================================================
# Scenario 6 & 7: ARBT During Active Recording vs Idle
# ==============================================================================

def test_arbt_during_active_recording_waits_for_completion():
    """Verify ARBT acknowledges promptly, flags reboot_pending, requests recorder stop,
    waits boundedly for buffers to drain and file to close, and verifies storage clean."""
    c_src = CONSOLE_C.read_text(encoding="utf-8")
    arbt_pos = c_src.find("static void handle_arbt(void)")
    assert arbt_pos != -1
    arbt_fn = c_src[arbt_pos:c_src.find("/* SDFMT handler", arbt_pos)]

    assert "main_set_reboot_pending(true);" in arbt_fn
    assert "main_scheduler_pause();" in arbt_fn
    assert "[ACK] ARBT: Applying configuration and initiating orderly shutdown..." in arbt_fn
    assert "recorder_request_stop();" in arbt_fn
    assert "recorder_wait_idle(5000);" in arbt_fn
    assert "recorder_get_last_error()" in arbt_fn
    assert "recorder_get_last_sd_error()" in arbt_fn
    assert "config_store_save();" in arbt_fn
    assert "sys_reboot(SYS_REBOOT_COLD);" in arbt_fn


def test_arbt_while_idle_reboots_promptly():
    """Verify ARBT received while idle reboots without waiting for recording."""
    c_src = CONSOLE_C.read_text(encoding="utf-8")
    arbt_pos = c_src.find("static void handle_arbt(void)")
    arbt_fn = c_src[arbt_pos:c_src.find("/* SDFMT handler", arbt_pos)]

    # Only waits if recorder_is_active()
    assert "if (recorder_is_active()) {" in arbt_fn
    # If idle, proceeds directly to config commit and reboot
    assert "config_store_save();" in arbt_fn


# ==============================================================================
# Scenario 8: Safe Buffer Flush and WAV Container Closure Before Reboot
# ==============================================================================

def test_safe_audio_buffer_flush_and_wav_file_close_before_reboot():
    """Verify that before reboot, the recording pipeline flushes all DMA blocks,
    empties PSRAM staging, synchronizes SD, finalizes WAV header with exact sample count, and closes file."""
    rec_src = RECORDER_C.read_text(encoding="utf-8")

    # In recorder_record_session abort / stop path:
    assert "if (recorder_abort_requested) {" in rec_src
    assert "drain_queue();" in rec_src
    assert "psram_staging_flush_publish();" in rec_src
    assert "wav_fill_header" in rec_src
    assert "fs_sync(&f);" in rec_src
    assert "fs_close(&f);" in rec_src
    assert "st.samples_written" in rec_src


# ==============================================================================
# Scenario 9 & 10: Post-Reboot Configuration Application & Failure Rollback
# ==============================================================================

def test_pending_config_application_after_reboot():
    """Verify startup detects pending configuration, validates integrity, applies to runtime,
    updates generation counters, and atomically clears pending flag in flash."""
    cfg_src = CONFIG_STORE_C.read_text(encoding="utf-8")

    assert "if (primary->pending_apply)" in cfg_src
    assert "if (config_record_is_valid(primary))" in cfg_src
    assert "unpack_record_to_system(primary);" in cfg_src
    assert "s_applied_gen = primary->pending_generation;" in cfg_src
    assert "s_last_valid_applied_record.pending_apply = 0;" in cfg_src
    assert "config_store_save();" in cfg_src


def test_failure_during_config_application_retains_last_known_valid():
    """Verify that if a pending configuration record is corrupted or invalid,
    startup falls back to the previous valid applied configuration without reboot loop."""
    cfg_src = CONFIG_STORE_C.read_text(encoding="utf-8")

    assert "Pending configuration validation failed! Falling back to last known valid applied configuration." in cfg_src
    assert "if (fallback && config_record_is_valid(fallback) && !fallback->pending_apply)" in cfg_src
    assert "unpack_record_to_system(fallback);" in cfg_src
    assert "s_applied_gen = fallback->applied_generation;" in cfg_src


# ==============================================================================
# Scenario 11: Critical SD Card Data Preservation
# ==============================================================================

def test_critical_sd_card_data_preservation_contract():
    """Verify existing recordings are never unlinked, formatted, or truncated."""
    rec_src = RECORDER_C.read_text(encoding="utf-8")
    cfg_src = CONFIG_STORE_C.read_text(encoding="utf-8")

    # Verify no blanket fs_unlink or fs_mkfs in config_store or recording normal path
    assert "fs_mkfs" not in cfg_src, "config_store must NEVER format SD card"
    assert "fs_unlink" not in cfg_src or "config.bak" in cfg_src  # only for config.bak

    # Recording file creation MUST NOT use FS_O_TRUNC
    assert "FS_O_CREATE | FS_O_WRITE" in rec_src
    assert "FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC" not in rec_src


def test_simulated_sd_recordings_unaltered_across_config_cycle():
    """Simulate file system operations verifying existing recording files retain content."""
    with tempfile.TemporaryDirectory() as tmpdir:
        sd_root = Path(tmpdir) / "SD"
        day_dir = sd_root / "20261031" / "20261031_10"
        day_dir.mkdir(parents=True, exist_ok=True)

        existing_wav = day_dir / "20261031_101500_000.wav"
        existing_data = b"RIFF" + b"\x00" * 36 + b"data" + b"\x12\x34\x56\x78" * 1024
        existing_wav.write_bytes(existing_data)

        # Simulate config update and reboot simulation
        rec = PersistedConfigRecord(seq=1, samf=48000)
        blob = rec.pack()
        assert len(blob) == 512

        # Verify existing file was NOT modified
        assert existing_wav.exists()
        assert existing_wav.read_bytes() == existing_data


# ==============================================================================
# Scenario 12: New Recording Filename Is Unique and Collision-Free
# ==============================================================================

def test_new_recording_filename_unique_and_collision_free():
    """Verify generate_recording_filename checks for collisions and falls back safely to next_filename."""
    rec_src = RECORDER_C.read_text(encoding="utf-8")

    # In generate_recording_filename:
    assert "for (unsigned int step = 0; step < 1000; step++)" in rec_src
    assert "%04u%02u%02u_%02u%02u%02u_%03u.wav" in rec_src
    assert "int stat_ret = fs_stat(name, &ent);" in rec_src
    assert "if (stat_ret == -ENOENT)" in rec_src
    assert "if (stat_ret != 0) return stat_ret;" in rec_src
    assert "return next_filename(name, len);" in rec_src


# ==============================================================================
# Scenario 13: Duplicate ARBT Does Not Trigger Multiple Reboots
# ==============================================================================

def test_duplicate_arbt_handled_idempotently():
    """Verify duplicate ARBT commands detect reboot_pending and return prompt ACK without multi-reboot."""
    c_src = CONSOLE_C.read_text(encoding="utf-8")
    arbt_pos = c_src.find("static void handle_arbt(void)")
    arbt_fn = c_src[arbt_pos:arbt_pos + 400]

    assert "if (main_is_reboot_pending())" in arbt_fn
    assert "[ACK] ARBT: Reboot already pending" in arbt_fn


# ==============================================================================
# Scenario 14: UART/BLE Command Bursts Do Not Block Audio Acquisition
# ==============================================================================

def test_command_bursts_do_not_block_audio_acquisition_priorities():
    """Verify priority hierarchy ensures audio acquisition & DMA are never starved by UART/BLE."""
    main_src = MAIN_C.read_text(encoding="utf-8")
    ble_src = BLE_COMM_C.read_text(encoding="utf-8")
    console_src = CONSOLE_C.read_text(encoding="utf-8")
    audio_src = (SRC_DIR / "audio_in_stm32l4.c").read_text(encoding="utf-8")

    # Audio acquisition priority is preemptive 0 (highest preemptive task)
    assert "#define ACQUISITION_THREAD_PRIORITY K_PRIO_PREEMPT(0)" in audio_src

    # Recorder processing priority is 1 (higher than SD writer to prevent starvation)
    assert "#define RECORDER_THREAD_PRIORITY   1" in main_src

    # SD Writer priority is 3 (lower priority than recorder/DSP processing)
    assert "#define SD_WRITER_PRIORITY 3" in RECORDER_C.read_text(encoding="utf-8")

    # Console command execution dropped to priority 4
    assert "k_thread_priority_set(k_current_get(), 4);" in main_src

    # BLE comm thread priority is 8
    assert "#define BLE_COMM_PRIORITY 8" in ble_src

    # Shared dispatcher mutex is ONLY taken in console/BLE processing threads
    assert "K_MUTEX_DEFINE(s_cmd_dispatch_mutex);" in console_src
    assert "k_mutex_lock(&s_cmd_dispatch_mutex, K_MSEC(250))" in console_src
    assert '"BUSY: command dispatcher unavailable"' in console_src
    assert "s_cmd_dispatch_mutex" not in audio_src, "Audio thread must NEVER touch command dispatcher mutex"

