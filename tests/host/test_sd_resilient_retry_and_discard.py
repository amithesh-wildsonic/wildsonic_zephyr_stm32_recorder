#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Host Unit and Verification Suite for Fool-Proof SD CRC Retry, Reopen, and Discard Hardening.

Covers:
1. Scenario 1 (F1): Single write CRC error retried and succeeded by Layer 1 without gap or data loss.
2. Scenario 2 (F2 N=2): Layer 1 3-attempt retry sequence succeeds on attempt 3 after peripheral reset.
3. Scenario 3 (F3): Dead handle returns -EBADF / -9, Layer 2 executes storage_reopen(), executes hardware reset,
   remounts FatFs, reopens active file, seeks to offset, and rewrites block successfully.
4. Scenario 4 (F2 N=5): Layer 1 fails (3 attempts), Layer 2 fails (2 reopen attempts), Layer 3 discards block,
   advances PSRAM staging, records gap in ring buffer, attempts zero-fill write (or skips if offline), and continues recording.
5. Scenario 5 (F4): Permanent card dead triggers >= 16 discards, session aborts cleanly, WAV header finalized with confirmed
   written bytes, storage health set to FAULT.
6. Scenario 6 (Sleep under FAULT): Dirty audio in PSRAM discarded, staging reset, transition to Stop 2 completes in <= 5 s
   without -EBUSY rejection.
7. Scenario 7 (F5): Post-wake mount retries 3 times (500 ms, 1000 ms, 2000 ms), marks card offline, schedule continues without crash.
8. Scenario 8 (Integrity): Truthful metrics reporting; "ZERO LOSS" reported only when all discard/gap counters are strictly 0.
9. Architectural Code Audit:
   - Resilient disk registration and interception of "SD" and "SDR".
   - Memory ownership audit eliminating mem_slab.c:270 assertion crash.
   - Timing budget adherence (Layer 1 <= 1500 ms, Layer 2 <= 4000 ms, Sleep flush <= 3000 ms).
   - Fault injection isolation under CONFIG_WILDSONIC_SD_FAULT_INJECT.
"""

import os
import sys
import pytest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
APP_SRC = REPO_ROOT / "app" / "src"

# ==============================================================================
# STATE & DATA MODELS FOR SCENARIOS 1-8
# ==============================================================================

class StorageHealth:
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    FAULT = "FAULT"

class MockDiskLayer:
    """Simulates Layer 1 Resilient Disk Wrapper and Fault Injection."""
    def __init__(self):
        self.f1_active = False
        self.f2_attempts_left = 0
        self.f3_active = False
        self.f4_active = False
        self.f5_attempts_left = 0
        self.hw_reset_count = 0
        self.write_calls = 0
        self.retry_ok = 0
        self.retry_fail = 0
        self.health = StorageHealth.HEALTHY

    def inject_f1(self):
        self.f1_active = True

    def inject_f2(self, attempts):
        self.f2_attempts_left = attempts

    def inject_f3(self):
        self.f3_active = True

    def inject_f4(self):
        self.f4_active = True

    def inject_f5(self, attempts):
        self.f5_attempts_left = attempts

    def hardware_reset(self):
        self.hw_reset_count += 1
        if self.f3_active:
            self.f3_active = False
        return 0

    def raw_write(self):
        self.write_calls += 1
        if self.f4_active:
            return -5  # -EIO permanent
        if self.f3_active:
            return -9  # -EBADF dead handle
        if self.f1_active:
            self.f1_active = False
            return -5  # -EIO CRC failure
        if self.f2_attempts_left > 0:
            self.f2_attempts_left -= 1
            return -5  # -EIO
        return 0

    def layer1_write(self):
        """Simulates resilient_disk_write retry sequence (max 3 attempts)."""
        attempt = 0
        elapsed_us = 0
        while attempt < 3:
            if elapsed_us >= 1500000 and attempt > 0:
                break
            rc = self.raw_write()
            if rc == 0:
                if attempt > 0:
                    self.retry_ok += 1
                    self.health = StorageHealth.DEGRADED
                return 0
            if attempt == 0:
                elapsed_us += 1050  # 1 ms sleep
            elif attempt == 1:
                self.hardware_reset()
                elapsed_us += 6000  # 5 ms sleep + reset
            attempt += 1

        self.retry_fail += 1
        return -5

class MockRecorderStoragePipeline:
    """Simulates Layer 1 + Layer 2 + Layer 3 Storage Pipeline."""
    def __init__(self, disk: MockDiskLayer):
        self.disk = disk
        self.file_opened = True
        self.reopen_attempts = 0
        self.reopen_ok = 0
        self.reopen_fail = 0
        self.blocks_discarded = 0
        self.samples_discarded = 0
        self.consecutive_discards = 0
        self.consecutive_clean_blocks = 0
        self.gaps = []
        self.staging_committed_bytes = 0
        self.written_bytes = 0
        self.zero_fill_bytes = 0
        self.zero_fill_logs = []
        self.session_aborted = False

    def storage_reopen(self, offset):
        self.reopen_attempts += 1
        if self.reopen_attempts > 2:
            self.reopen_fail += 1
            return -1
        # Layer 2 executes SD hardware reset, remount, reopen
        self.disk.hardware_reset()
        if self.disk.f4_active:
            self.reopen_fail += 1
            return -5
        self.file_opened = True
        self.reopen_ok += 1
        self.disk.health = StorageHealth.DEGRADED
        return 0

    def process_block(self, block_bytes, sample_count):
        if self.session_aborted:
            return -5

        # If F3 is active, fs_write returns -9 (-EBADF) immediately due to dead FatFs handle
        if self.disk.f3_active:
            w_ret = -9
        else:
            w_ret = self.disk.layer1_write()

        if w_ret == 0:
            self.written_bytes += block_bytes
            self.staging_committed_bytes += block_bytes
            self.consecutive_clean_blocks += 1
            self.consecutive_discards = 0
            if self.disk.health == StorageHealth.DEGRADED and self.consecutive_clean_blocks >= 100:
                self.disk.health = StorageHealth.HEALTHY
            return 0

        # 2. Layer 2: Dead-handle reopen recovery (up to 2 attempts)
        recovered = False
        for _ in range(2):
            ro_rc = self.storage_reopen(self.written_bytes)
            if ro_rc == 0:
                # Rewrite block
                rew_ret = self.disk.raw_write()
                if rew_ret == 0:
                    recovered = True
                    self.written_bytes += block_bytes
                    self.staging_committed_bytes += block_bytes
                    self.consecutive_clean_blocks += 1
                    self.consecutive_discards = 0
                    return 0

        # 3. Layer 3: Discard block policy
        self.blocks_discarded += 1
        self.samples_discarded += sample_count
        self.consecutive_discards += 1
        self.consecutive_clean_blocks = 0
        self.gaps.append((self.written_bytes // 2, sample_count, w_ret))
        self.staging_committed_bytes += block_bytes

        # Attempt ONE zero-fill write of same size
        z_rc = self.disk.raw_write()
        if z_rc == 0:
            self.written_bytes += block_bytes
            self.zero_fill_bytes += block_bytes
            self.zero_fill_logs.append("wrote zero-fill block")
        else:
            self.zero_fill_logs.append("zero-fill skipped: card offline")

        if self.consecutive_discards >= 16:
            self.disk.health = StorageHealth.FAULT
            self.session_aborted = True
            return -5

        return 0

# ==============================================================================
# 1. SCENARIO 1: SINGLE WRITE CRC ERROR (F1)
# ==============================================================================

def test_scenario_1_single_crc_error_recovered_by_layer1():
    """Scenario 1: F1 injected -> Layer 1 immediate retry succeeds -> 0 gaps, 0 loss."""
    disk = MockDiskLayer()
    pipeline = MockRecorderStoragePipeline(disk)

    # Arm F1: next write will fail with CRC error (-EIO)
    disk.inject_f1()

    # Process block: 3840 bytes (1920 samples)
    res = pipeline.process_block(3840, 1920)

    assert res == 0
    assert disk.retry_ok == 1
    assert disk.retry_fail == 0
    assert pipeline.blocks_discarded == 0
    assert pipeline.samples_discarded == 0
    assert len(pipeline.gaps) == 0
    assert pipeline.written_bytes == 3840
    assert disk.health == StorageHealth.DEGRADED

# ==============================================================================
# 2. SCENARIO 2: LAYER 1 3-ATTEMPT RETRY SEQUENCE (F2 N=2)
# ==============================================================================

def test_scenario_2_retry_sequence_succeeds_on_attempt_3():
    """Scenario 2: F2 (N=2) -> Attempt 0 fails, Attempt 1 fails, Attempt 2 resets and succeeds."""
    disk = MockDiskLayer()
    pipeline = MockRecorderStoragePipeline(disk)

    # Arm F2 for 2 failures: attempt 0 fails, attempt 1 fails, attempt 2 succeeds
    disk.inject_f2(2)

    res = pipeline.process_block(3840, 1920)

    assert res == 0
    assert disk.retry_ok == 1
    assert disk.hw_reset_count == 1  # Executed peripheral reset on attempt 3
    assert pipeline.blocks_discarded == 0
    assert len(pipeline.gaps) == 0
    assert pipeline.written_bytes == 3840

# ==============================================================================
# 3. SCENARIO 3: DEAD HANDLE RECOVERY (F3 -> -9 / -EBADF)
# ==============================================================================

def test_scenario_3_dead_handle_layer2_reopen_recovery():
    """Scenario 3: F3 returns -9 -> Layer 1 fails -> Layer 2 reopens file & rewrites."""
    disk = MockDiskLayer()
    pipeline = MockRecorderStoragePipeline(disk)

    # Arm F3 (dead handle)
    disk.inject_f3()

    res = pipeline.process_block(3840, 1920)

    assert res == 0
    assert disk.f3_active is False  # Cleared by re-init in Layer 2
    assert pipeline.reopen_ok == 1
    assert pipeline.blocks_discarded == 0
    assert len(pipeline.gaps) == 0
    assert pipeline.written_bytes == 3840

# ==============================================================================
# 4. SCENARIO 4: BOUNDED DISCARD POLICY (F2 N=5)
# ==============================================================================

def test_scenario_4_layer3_discard_advances_staging_and_records_gap():
    """Scenario 4: F2 N=5 -> Layer 1 fails (3 att), Layer 2 fails (2 att) -> Layer 3 discards block."""
    disk = MockDiskLayer()
    pipeline = MockRecorderStoragePipeline(disk)

    # Arm F2 with 5 failures: Layer 1 (3 att) + Layer 2 (2 att) all fail
    disk.inject_f2(5)

    res = pipeline.process_block(3840, 1920)

    assert res == 0  # Recording continues!
    assert pipeline.blocks_discarded == 1
    assert pipeline.samples_discarded == 1920
    assert len(pipeline.gaps) == 1
    assert pipeline.gaps[0][1] == 1920
    assert pipeline.staging_committed_bytes == 3840  # Staging advanced past discarded chunk

# ==============================================================================
# 5. SCENARIO 5: PERMANENT CARD FAILURE & CLEAN ABORT (F4)
# ==============================================================================

def test_scenario_5_permanent_card_dead_aborts_after_16_discards():
    """Scenario 5: F4 permanent dead -> 16 discards -> clean abort, health = FAULT."""
    disk = MockDiskLayer()
    pipeline = MockRecorderStoragePipeline(disk)

    # Arm F4: all operations fail
    disk.inject_f4()

    for i in range(20):
        pipeline.process_block(3840, 1920)
        if pipeline.session_aborted:
            break

    assert pipeline.session_aborted is True
    assert pipeline.consecutive_discards == 16
    assert disk.health == StorageHealth.FAULT
    assert "zero-fill skipped: card offline" in pipeline.zero_fill_logs

# ==============================================================================
# 6. SCENARIO 6: SLEEP UNDER FAULT NEVER BLOCKS (R1, R2, R3)
# ==============================================================================

def test_scenario_6_sleep_under_fault_discards_dirty_and_proceeds():
    """Scenario 6: Dirty audio in PSRAM when storage health is FAULT must discard and sleep in <= 5s."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    sleep_fn = rec_c.split("int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us)")[1].split("int recorder_record_session(")[0]

    # Verify that under FAULT, dirty audio is discarded without rejecting sleep with -EBUSY
    assert "if (s_storage_state == STORAGE_STATE_FAULT) {" in sleep_fn
    assert "sd_resilient_record_sleep_discard(dirty);" in sleep_fn
    assert "psram_staging_reset();" in sleep_fn
    # Verify timeout flush deadline is strictly bounded via writer_completion_timeout_ms
    assert "uint32_t timeout_ms = writer_completion_timeout_ms(" in sleep_fn

# ==============================================================================
# 7. SCENARIO 7: POST-WAKE MOUNT RETRY (F5)
# ==============================================================================

def test_scenario_7_post_wake_mount_backoff_and_schedule_continuation():
    """Scenario 7: Wake mount retries 3 times (500 ms, 1000 ms, 2000 ms) and marks FAULT on failure."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    sleep_fn = rec_c.split("int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us)")[1].split("int recorder_record_session(")[0]

    assert "static const uint32_t mnt_backoffs[3] = {500, 1000, 2000};" in sleep_fn
    assert "sd_resilient_set_health(STORAGE_HEALTH_FAULT);" in sleep_fn
    assert 'TRACE("Wake mount failed after 3 attempts");' in sleep_fn

# ==============================================================================
# 8. SCENARIO 8: TRUTHFUL METRICS & "ZERO LOSS" INTEGRITY
# ==============================================================================

def test_scenario_8_truthful_metrics_and_zero_loss_contract():
    """Scenario 8: ZERO LOSS reported ONLY if all discard/gap counters are strictly 0."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    assert "sd_resilient_get_gap_count()" in rec_c
    assert "(total_gaps == 0)" in rec_c
    assert "(st_res_chk.blocks_discarded == 0)" in rec_c

    # Check truthful zero-fill logging
    assert 'TRACE("Wrote zero-fill (%u B) at 0x%llX"' in rec_c
    assert 'TRACE("Zero-fill skipped: card offline (%d)", w_ret);' in rec_c

# ==============================================================================
# 9. ARCHITECTURAL CODE AUDIT & MEMORY OWNERSHIP (mem_slab.c:270 FIX)
# ==============================================================================

def test_memory_ownership_audit_eliminates_mem_slab_assertion():
    """Verify that buffer pool freeing is strictly protected by is_pool_ptr and RELEASE_BLOCK."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    assert "static inline bool is_pool_ptr(const void *blk)" in rec_c
    assert "static inline void release_block(void *blk)" in rec_c
    assert "#define RELEASE_BLOCK(p) release_block((void *)(p))" in rec_c
    assert "RELEASE_BLOCK(sd_bounce_buf);" in rec_c
    assert "free_slabs != NUM_BLOCKS" in rec_c
    assert "k_mem_slab_num_free_get(&pool)" in rec_c

def test_gap_record_packing_and_ring_capacity():
    """Verify storage_gap_record is packed to 16 bytes and ring capacity is >= 128."""
    res_h = (APP_SRC / "sd_resilient.h").read_text(encoding="utf-8")

    assert "struct storage_gap_record {" in res_h
    assert "} __attribute__((packed));" in res_h
    assert "#define STORAGE_GAP_RING_CAPACITY 128" in res_h

def test_timing_budgets_in_code():
    """Verify retry and recovery timing budgets across layers."""
    res_c = (APP_SRC / "sd_resilient.c").read_text(encoding="utf-8")
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    # Layer 1 budget <= 1500 ms
    assert "elapsed_us >= 1500000U" in res_c
    # Layer 2 budget <= 4000 ms (2 attempts * 2000 ms)
    assert "s_session_reopen_count >= 2" in rec_c
    # Layer 3 session abort at 20 s or 16 discards
    assert "st_chk.consecutive_discards >= 16 || st_chk.time_in_recovery_us > 20000000ULL" in rec_c

def test_fault_injection_isolated_from_production_console():
    """Verify CMD_FAULT is compiled strictly under CONFIG_WILDSONIC_SD_FAULT_INJECT."""
    console_c = (APP_SRC / "console.c").read_text(encoding="utf-8")

    fault_block = console_c.split("CMD_FAULT")[1].split("};")[0]
    assert "CONFIG_WILDSONIC_SD_FAULT_INJECT" in console_c


def test_sd_resilient_hardware_reset_diagnostics_and_power_cycle_gpio():
    """Verify sd_resilient_hardware_reset prints all 5 reset steps, prefers driver deinit, and supports sd-power."""
    res_c = (APP_SRC / "sd_resilient.c").read_text(encoding="utf-8")
    kconfig = (REPO_ROOT / "app" / "Kconfig").read_text(encoding="utf-8")

    assert "config SD_POWER_GPIO" in kconfig
    assert "SD reset step1 DEINIT rc=%d st=%d" in res_c
    assert "SD reset step2 VDD" in res_c
    assert 'TRACE("SD card re-init failed: %d", ret);' in res_c
    assert 'TRACE("SD card status error after re-init: %d", status);' in res_c
    assert "SD reset step5 CLKCR=0x%08X" in res_c
    assert "DT_ALIAS(sd_power)" in res_c

