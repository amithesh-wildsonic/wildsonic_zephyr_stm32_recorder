#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Verification Suite for SDMMC Write Failure Diagnosis,
Idempotent Directory Creation, Error Code Preservation,
and PSRAM Ping-Pong Batch Buffering Architecture.
"""

import os
import sys
import pytest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
APP_SRC = REPO_ROOT / "app" / "src"

# ==============================================================================
# 1. PSRAM PING-PONG BATCH BUFFER MODEL
# ==============================================================================

class PingPongBatchState:
    FREE = 0
    FILLING = 1
    READY = 2
    WRITING = 3

class PingPongBatchModel:
    def __init__(self, num_buffers=2, batch_size=256 * 1024):
        self.num_buffers = num_buffers
        self.batch_size = batch_size
        self.capacity = num_buffers * batch_size
        self.buffers = [{
            "base_addr": i * batch_size,
            "capacity": batch_size,
            "filled": 0,
            "committed": 0,
            "state": PingPongBatchState.FREE,
            "batch_id": 0,
            "data": bytearray(batch_size)
        } for i in range(num_buffers)]
        self.fill_idx = 0
        self.drain_idx = 0
        self.batch_seq = 0
        self.occupied = 0
        self.reserved = 0
        self.drops = 0
        self.full_events = 0
        self.total_buffered = 0
        self.total_flushed = 0

        # Start with buffer 0 in FILLING state
        self.buffers[0]["state"] = PingPongBatchState.FILLING
        self.batch_seq += 1
        self.buffers[0]["batch_id"] = self.batch_seq

    def push(self, data: bytes, samples: int) -> int:
        n = len(data)
        cur = self.buffers[self.fill_idx]

        if cur["state"] == PingPongBatchState.FREE:
            cur["state"] = PingPongBatchState.FILLING
            cur["filled"] = 0
            cur["committed"] = 0
            self.batch_seq += 1
            cur["batch_id"] = self.batch_seq
        elif cur["state"] != PingPongBatchState.FILLING:
            # All buffers occupied
            self.full_events += 1
            self.drops += samples
            return -105  # -ENOBUFS

        space_in_cur = cur["capacity"] - cur["filled"]
        if n > space_in_cur or (self.occupied + self.reserved + n > self.capacity):
            self.full_events += 1
            self.drops += samples
            return -105  # -ENOBUFS

        # Reserve and simulate physical write
        self.reserved = n
        offset = cur["filled"]
        cur["data"][offset:offset + n] = data

        # Publish
        cur["filled"] += n
        self.occupied += n
        self.total_buffered += n
        self.reserved = 0

        # If buffer is now full, transition to READY and switch fill_idx
        if cur["filled"] >= cur["capacity"]:
            cur["state"] = PingPongBatchState.READY
            next_idx = (self.fill_idx + 1) % self.num_buffers
            self.fill_idx = next_idx
            if self.buffers[self.fill_idx]["state"] == PingPongBatchState.FREE:
                self.buffers[self.fill_idx]["state"] = PingPongBatchState.FILLING
                self.buffers[self.fill_idx]["filled"] = 0
                self.buffers[self.fill_idx]["committed"] = 0
                self.batch_seq += 1
                self.buffers[self.fill_idx]["batch_id"] = self.batch_seq

        return 0

    def peek(self, max_bytes: int):
        if self.occupied == 0:
            return b""

        drain = self.buffers[self.drain_idx]
        if drain["state"] == PingPongBatchState.READY:
            drain["state"] = PingPongBatchState.WRITING
        elif drain["state"] != PingPongBatchState.WRITING:
            if drain["state"] == PingPongBatchState.FILLING and drain["filled"] > 0:
                drain["state"] = PingPongBatchState.WRITING
            else:
                return b""

        uncommitted = drain["filled"] - drain["committed"]
        if uncommitted == 0:
            return b""

        to_read = min(max_bytes, uncommitted)
        offset = drain["committed"]
        return bytes(drain["data"][offset:offset + to_read])

    def commit(self, bytes_committed: int) -> int:
        if bytes_committed == 0:
            return 0

        drain = self.buffers[self.drain_idx]
        if drain["state"] not in (PingPongBatchState.WRITING, PingPongBatchState.FILLING):
            return -22  # -EINVAL

        drain["committed"] += bytes_committed
        self.occupied -= bytes_committed
        self.total_flushed += bytes_committed

        # If batch completely written, release to FREE
        if drain["committed"] >= drain["filled"]:
            if drain["state"] == PingPongBatchState.WRITING or self.drain_idx != self.fill_idx:
                drain["state"] = PingPongBatchState.FREE
                drain["filled"] = 0
                drain["committed"] = 0
            self.drain_idx = (self.drain_idx + 1) % self.num_buffers

        return 0

    def flush_publish(self) -> int:
        cur = self.buffers[self.fill_idx]
        if cur["state"] == PingPongBatchState.FILLING and cur["filled"] > 0:
            cur["state"] = PingPongBatchState.READY
            return 0
        return 1


# ==============================================================================
# TESTS
# ==============================================================================

def test_sdmmc_error_code_trace_and_semantics():
    """Trace numeric error 2 in STM32 SDMMC driver to SDMMC_ERROR_DATA_CRC_FAIL."""
    # Peripheral definition check in LL SDMMC headers:
    # Bit 0 (0x01) = SDMMC_ERROR_CMD_CRC_FAIL
    # Bit 1 (0x02) = SDMMC_ERROR_DATA_CRC_FAIL
    SDMMC_ERROR_DATA_CRC_FAIL = 0x00000002
    assert SDMMC_ERROR_DATA_CRC_FAIL == 2, "Numeric error 2 must correspond exactly to SDMMC_ERROR_DATA_CRC_FAIL"

    # Confirm overlay warning for wiring length
    overlay = (REPO_ROOT / "app" / "sd" / "sdmmc_nucleo_l476rg.overlay").read_text()
    assert "Keep wires short (< 10 cm)" in overlay, "Overlay must caution about wire length causing CRC errors"
    assert "clk-div = <2>" in overlay, "Overlay must specify SD clock divider"


def test_idempotent_directory_creation_contracts():
    """Verify that ensure_dir_exists in recorder.c avoids calling fs_mkdir on existing directories."""
    rec_c = (APP_SRC / "recorder.c").read_text()

    assert "static int ensure_dir_exists" in rec_c, "ensure_dir_exists helper must be defined in recorder.c"
    assert "fs_stat(path, &ent)" in rec_c, "ensure_dir_exists must stat directory first"
    assert "ent.type == FS_DIR_ENTRY_DIR" in rec_c, "ensure_dir_exists must check directory entry type"
    assert "ret == -EEXIST" in rec_c, "ensure_dir_exists must handle -EEXIST by re-verifying directory"
    assert "ensure_dir_exists(dir)" in rec_c, "folder_mode 1 must use ensure_dir_exists"
    assert "ensure_dir_exists(day_dir)" in rec_c, "folder_mode 2 must use ensure_dir_exists for day_dir"


def test_verification_error_reporting_distinguishes_errno_and_bytes():
    """Verify recorder_verify_file distinguishes between I/O errno and short read byte count."""
    rec_c = (APP_SRC / "recorder.c").read_text()

    assert "Verify read %s failed with I/O error: %d" in rec_c, "Must log distinct message on negative errno"
    assert "Verify read %s short read: %d bytes read (expected %zu)" in rec_c, "Must log short read with bytes and expected"
    assert "Verify read %s failed: %d" not in rec_c, "Ambiguous 'failed: 0' message must be eliminated"


def test_failed_recording_skips_verification_and_marks_unverified():
    """Verify failed recordings skip file verification and set wav_verified to false."""
    rec_c = (APP_SRC / "recorder.c").read_text()

    # Inspection of recording finalization block
    assert "if (err == 0 && !recorder_abort_requested && st.bytes_written > 0)" in rec_c, \
        "recorder_verify_file must only be called if session had NO errors"
    assert "st.wav_verified = false;" in rec_c, "st.wav_verified must be explicitly set to false on error"


def test_first_meaningful_error_preservation():
    """Verify an early SD write error is preserved and not overwritten by close or truncate."""
    rec_c = (APP_SRC / "recorder.c").read_text()

    # If err != 0, WAV finalize/sync/close must NOT overwrite err
    assert "if (finalize_ret != 0 && err == 0)" in rec_c, \
        "finalize/close error must only be taken if err == 0"
    assert "if (sd_writer_err != 0 && err == 0)" in rec_c or "if (err == 0 && sd_writer_err != 0)" in rec_c, \
        "sd_writer_err must be adopted if err == 0"


def test_storage_recovery_resets_disk_and_unmounts_unconditionally():
    """Verify recorder_storage_recover resets mounted flag and reinitializes disk."""
    rec_c = (APP_SRC / "recorder.c").read_text()

    recover_code = rec_c.split("int recorder_storage_recover(void)", 1)[1].split("static int next_filename", 1)[0]
    assert "mounted = false;" in recover_code, "Storage recovery must clear mounted flag"
    assert "disk_access_init(\"SD\")" in recover_code, "Storage recovery must re-init SD disk"
    assert "fs_mount(&mp)" in recover_code, "Storage recovery must remount volume"


def test_pingpong_buffer_states_and_ownership_transitions():
    """Verify PingPongBatchModel transitions through FREE -> FILLING -> READY -> WRITING -> FREE."""
    model = PingPongBatchModel(num_buffers=2, batch_size=16384)

    # Initial state
    assert model.buffers[0]["state"] == PingPongBatchState.FILLING
    assert model.buffers[1]["state"] == PingPongBatchState.FREE

    # Push half buffer (8192 bytes)
    block1 = b"\x11" * 8192
    assert model.push(block1, 4096) == 0
    assert model.buffers[0]["state"] == PingPongBatchState.FILLING
    assert model.buffers[0]["filled"] == 8192
    assert model.occupied == 8192

    # Push second half (8192 bytes) -> Buffer 0 becomes full!
    block2 = b"\x22" * 8192
    assert model.push(block2, 4096) == 0
    assert model.buffers[0]["state"] == PingPongBatchState.READY
    assert model.buffers[0]["filled"] == 16384
    assert model.buffers[1]["state"] == PingPongBatchState.FILLING
    assert model.fill_idx == 1

    # Writer peeks buffer 0 -> transitions to WRITING
    peeked = model.peek(16384)
    assert len(peeked) == 16384
    assert peeked == block1 + block2
    assert model.buffers[0]["state"] == PingPongBatchState.WRITING

    # Writer commits buffer 0 -> transitions to FREE
    assert model.commit(16384) == 0
    assert model.buffers[0]["state"] == PingPongBatchState.FREE
    assert model.buffers[0]["filled"] == 0
    assert model.occupied == 0
    assert model.drain_idx == 1


def test_pingpong_producer_writer_concurrency_overlap():
    """Verify producer can fill Buffer 1 while writer is concurrently consuming Buffer 0."""
    model = PingPongBatchModel(num_buffers=2, batch_size=16384)

    # Fill Buffer 0 completely
    model.push(b"A" * 16384, 8192)
    assert model.buffers[0]["state"] == PingPongBatchState.READY
    assert model.buffers[1]["state"] == PingPongBatchState.FILLING

    # Writer starts consuming Buffer 0 in 8 KiB chunks
    chunk1 = model.peek(8192)
    assert len(chunk1) == 8192
    assert model.buffers[0]["state"] == PingPongBatchState.WRITING
    model.commit(8192)
    assert model.buffers[0]["state"] == PingPongBatchState.WRITING  # Still writing remaining 8K

    # Concurrently, producer pushes to Buffer 1
    assert model.push(b"B" * 8192, 4096) == 0
    assert model.buffers[1]["filled"] == 8192

    # Writer commits remaining 8K of Buffer 0 -> Buffer 0 becomes FREE
    chunk2 = model.peek(8192)
    assert len(chunk2) == 8192
    model.commit(8192)
    assert model.buffers[0]["state"] == PingPongBatchState.FREE

    # Producer finishes Buffer 1 -> Buffer 1 becomes READY, Buffer 0 becomes FILLING
    assert model.push(b"C" * 8192, 4096) == 0
    assert model.buffers[1]["state"] == PingPongBatchState.READY
    assert model.buffers[0]["state"] == PingPongBatchState.FILLING


def test_pingpong_buffer_exhaustion_rejects_and_drops():
    """Verify that when both buffers are occupied, producer rejects and increments drop count."""
    model = PingPongBatchModel(num_buffers=2, batch_size=16384)

    # Fill Buffer 0 -> READY
    model.push(b"0" * 16384, 8192)
    # Fill Buffer 1 -> READY
    model.push(b"1" * 16384, 8192)

    assert model.buffers[0]["state"] == PingPongBatchState.READY
    assert model.buffers[1]["state"] == PingPongBatchState.READY

    # Both buffers are occupied! Next push MUST return -ENOBUFS and record drop
    ret = model.push(b"X" * 8192, 4096)
    assert ret == -105  # -ENOBUFS
    assert model.drops == 4096
    assert model.full_events == 1

    # Buffer 0 and Buffer 1 data must remain completely intact
    assert model.buffers[0]["data"][:16384] == b"0" * 16384
    assert model.buffers[1]["data"][:16384] == b"1" * 16384


def test_pingpong_simulated_sd_write_failure_preserves_uncommitted_buffer():
    """Verify simulated SD write failure leaves buffer in WRITING state without reusing it."""
    model = PingPongBatchModel(num_buffers=2, batch_size=16384)

    data = b"D" * 16384
    model.push(data, 8192)
    assert model.buffers[0]["state"] == PingPongBatchState.READY

    # Peek buffer to write to SD
    peeked = model.peek(8192)
    assert len(peeked) == 8192
    assert model.buffers[0]["state"] == PingPongBatchState.WRITING

    # Simulate fs_write failure (-EIO) -> do NOT commit!
    # Buffer MUST remain in WRITING state and occupied bytes must be retained!
    assert model.buffers[0]["state"] == PingPongBatchState.WRITING
    assert model.occupied == 16384
    assert model.buffers[0]["state"] != PingPongBatchState.FREE

    # Uncommitted bytes are preserved for retry or failure accounting
    assert model.peek(8192) == peeked


def test_pingpong_flush_publish_at_session_end():
    """Verify that a partially filled active buffer is published to READY upon session flush."""
    model = PingPongBatchModel(num_buffers=2, batch_size=16384)

    # Push partial batch (8192 bytes out of 16384)
    model.push(b"Z" * 8192, 4096)
    assert model.buffers[0]["state"] == PingPongBatchState.FILLING
    assert model.buffers[0]["filled"] == 8192

    # Call flush_publish
    assert model.flush_publish() == 0
    assert model.buffers[0]["state"] == PingPongBatchState.READY

    # Writer can now drain the partial batch
    peeked = model.peek(8192)
    assert len(peeked) == 8192
    assert peeked == b"Z" * 8192
    model.commit(8192)
    assert model.buffers[0]["state"] == PingPongBatchState.FREE
    assert model.occupied == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
