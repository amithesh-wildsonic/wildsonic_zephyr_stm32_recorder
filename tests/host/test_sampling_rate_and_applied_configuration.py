#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Verification Suite for Sampling Rate Synchronization,
Effective Applied Configuration, WAV Accounting, and Ping-Pong Buffer Lifecycle.
"""

import os
import sys
import re
import pytest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
APP_SRC = REPO_ROOT / "app" / "src"


# ==============================================================================
# 1. SAMPLING RATE & WAV PAYLOAD ACCOUNTING MATHEMATICS
# ==============================================================================

def compute_expected_audio_accounting(sample_rate_hz: int, duration_sec: int, sample_bits: int = 16):
    bytes_per_sample = sample_bits // 8
    expected_samples = sample_rate_hz * duration_sec
    pcm_payload_bytes = expected_samples * bytes_per_sample
    total_wav_bytes = 44 + pcm_payload_bytes  # Standard RIFF WAV header
    byte_rate = sample_rate_hz * 1 * bytes_per_sample  # Mono
    return {
        "expected_samples": expected_samples,
        "pcm_payload_bytes": pcm_payload_bytes,
        "total_wav_bytes": total_wav_bytes,
        "byte_rate": byte_rate,
    }


def test_192khz_10second_recording_accounting():
    """
    User specification: For a genuine 192000 Hz, 16-bit mono, 10-second recording,
    the expected payload is exactly 1,920,000 samples and 3,840,000 bytes,
    excluding the 44-byte WAV header (total WAV size: 3,840,044 bytes).
    """
    acct = compute_expected_audio_accounting(192000, 10, sample_bits=16)
    assert acct["expected_samples"] == 1_920_000
    assert acct["pcm_payload_bytes"] == 3_840_000
    assert acct["total_wav_bytes"] == 3_840_044
    assert acct["byte_rate"] == 384_000


def test_standard_rates_audio_accounting():
    """Verify accounting for all canonical WildSonic sample rates across typical durations."""
    canonical_rates = [16000, 32000, 44100, 48000, 96000, 192000]
    durations = [1, 5, 10, 60, 600]

    for rate in canonical_rates:
        for sec in durations:
            acct = compute_expected_audio_accounting(rate, sec, sample_bits=16)
            assert acct["expected_samples"] == rate * sec
            assert acct["pcm_payload_bytes"] == rate * sec * 2
            assert acct["total_wav_bytes"] == 44 + (rate * sec * 2)


# ==============================================================================
# 2. HARDWARE TIMING & ADC OVERSAMPLING BUDGET EVALUATION
# ==============================================================================

def eval_audio_in_supported(rate_hz: int, ahb_hz: int = 72_000_000, ovs_log2: int = 3, cycles_per_conv: int = 37):
    """
    Simulates audio_in_is_rate_supported() in audio_in_stm32l4.c.
    Must check both discrete rates and physical ADC timing budget.
    """
    canonical_rates = {16000, 32000, 44100, 48000, 96000, 192000}
    if rate_hz not in canonical_rates:
        return False, 0, 0, "Invalid discrete rate"

    tim_hz = ahb_hz  # APB1 prescaler = 1
    div = (tim_hz + rate_hz // 2) // rate_hz
    if div == 0:
        return False, 0, 0, "Timer divider zero"
    actual_rate = tim_hz // div

    conv_ns = (((1 << ovs_log2) * cycles_per_conv + 4) * 1_000_000_000) // ahb_hz
    period_ns = 1_000_000_000 // actual_rate

    if conv_ns * 100 > period_ns * 85:
        return False, actual_rate, div, f"Oversampling budget exceeded ({conv_ns} ns of {period_ns} ns period)"

    return True, actual_rate, div, "OK"


def test_adc_oversampling_budget_at_72mhz():
    """At 72 MHz with 8x oversampling (OVS_LOG2=3), 192 kHz fits into 80.0% of period."""
    supported, actual, div, reason = eval_audio_in_supported(192000, ahb_hz=72_000_000, ovs_log2=3)
    assert supported is True
    assert actual == 192000
    assert div == 375
    assert reason == "OK"


def test_adc_oversampling_budget_rejection_at_low_clock():
    """If AHB clock drops to 48 MHz, 8x oversampling exceeds period (120%) and is rejected."""
    supported, actual, div, reason = eval_audio_in_supported(192000, ahb_hz=48_000_000, ovs_log2=3)
    assert supported is False
    assert "budget exceeded" in reason


def test_adc_oversampling_budget_rejection_at_16x_oversampling():
    """At 16x oversampling (OVS_LOG2=4), conversion cycles (596) exceed 85% of 192 kHz period."""
    supported, actual, div, reason = eval_audio_in_supported(192000, ahb_hz=72_000_000, ovs_log2=4)
    assert supported is False
    assert "budget exceeded" in reason


def test_arbitrary_unsupported_rates_rejected():
    """Non-canonical rates are explicitly rejected."""
    for bad_rate in [8000, 22050, 50000, 100000, 384000]:
        supported, _, _, reason = eval_audio_in_supported(bad_rate)
        assert supported is False
        assert "Invalid discrete rate" in reason


# ==============================================================================
# 3. SOURCE INTEGRATION & SYNCHRONIZATION AUDIT
# ==============================================================================

def test_source_audio_in_checks_timing_budget_in_is_rate_supported():
    """audio_in_is_rate_supported() in audio_in_stm32l4.c must delegate to rate_config_check_ex."""
    src = (APP_SRC / "audio_in_stm32l4.c").read_text(encoding="utf-8")
    src_rc = (APP_SRC / "rate_check.c").read_text(encoding="utf-8")
    assert "bool audio_in_is_rate_supported" in src
    assert "rate_config_check_ex" in src
    # Timing budget checked in pure rate_check module
    assert "period_ns64" in src_rc and "RATE_CHECK_ADC_BUDGET_PCT_MAX" in src_rc
    # audio_in_set_rate must call audio_in_is_rate_supported
    assert "audio_in_is_rate_supported(new_rate_hz" in src


def test_source_console_samf_checks_hardware_set_rate():
    """handle_samf() in console.c must verify feasibility & audio_in_set_rate() return code."""
    src = (APP_SRC / "console.c").read_text(encoding="utf-8")
    assert "handle_samf" in src
    assert "console_check_audio_config(" in src
    # Checks return code of audio_in_set_rate when idle
    assert "int rc = audio_in_set_rate(val);" in src
    assert "if (rc != 0)" in src
    assert "Unsupported sample rate" in src
    # Keeps nominal sample rate in settings
    assert "settings->sample_rate = val;" in src


def test_source_console_sync_staged_to_active():
    """console_sync_staged_to_active() must validate staged config cleanly without overwriting nominal rate."""
    src = (APP_SRC / "console.c").read_text(encoding="utf-8")
    assert "void console_sync_staged_to_active(void)" in src
    assert "rate_config_check_ex(" in src
    assert "settings->sample_rate = g_cfg.sampling_rate;" in src


def test_source_config_store_validates_persisted_rate():
    """unpack_record_to_system() and config_store_init() must validate persisted config via rate_config_check_ex."""
    src = (APP_SRC / "config_store.c").read_text(encoding="utf-8")
    assert "unpack_record_to_system" in src
    assert "config_record_validate_rate(" in src
    assert "Persisted config rejected" in src


def test_source_main_boot_synchronizes_audio_in_rate():
    """main() in main.c must validate before audio init and keep nominal settings.sample_rate."""
    src = (APP_SRC / "main.c").read_text(encoding="utf-8")
    # In main() startup:
    assert "rate_config_check_ex(&boot_p, &boot_rc);" in src
    assert "ret = recorder_init_audio(settings.sample_rate);" in src
    # In recorder_thread_fn() / recorder_reconcile_rate_before_session():
    assert "if (audio_in_requested_rate() == settings.sample_rate)" in src
    assert "audio_in_set_rate(settings.sample_rate);" in src


def test_source_recorder_uses_actual_hardware_rate_and_exact_accounting():
    """recorder_record_session() must check feasibility first and calculate expected samples from actual rate."""
    src = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    assert "rate_config_check_ex(&rcp, &rc_res);" in src
    assert "st.sample_rate = audio_in_rate();" in src
    assert "st.expected_samples = (uint64_t)seconds * st.sample_rate;" in src
    assert "exact_bytes = st.expected_samples * bytes_per_frame;" in src
    assert "wav_fill_header(hdr, st.sample_rate" in src


# ==============================================================================
# 4. PING-PONG BUFFER OWNERSHIP & LIFECYCLE TESTS
# ==============================================================================

class PingPongBatchState:
    FREE = 0
    FILLING = 1
    READY = 2
    WRITING = 3


class MockPingPongManager:
    def __init__(self, num_buffers=2, batch_size=256 * 1024):
        self.num_buffers = num_buffers
        self.batch_size = batch_size
        self.buffers = [{
            "id": i,
            "state": PingPongBatchState.FREE,
            "filled": 0,
            "committed": 0,
            "batch_id": 0,
        } for i in range(num_buffers)]
        self.fill_idx = 0
        self.drain_idx = 0
        self.batch_seq = 1
        self.buffers[0]["state"] = PingPongBatchState.FILLING
        self.buffers[0]["batch_id"] = self.batch_seq
        self.occupied = 0
        self.drops = 0
        self.full_events = 0

    def push_block(self, bytes_count: int, samples_count: int) -> int:
        cur = self.buffers[self.fill_idx]
        if cur["state"] == PingPongBatchState.FREE:
            cur["state"] = PingPongBatchState.FILLING
            cur["filled"] = 0
            cur["committed"] = 0
            self.batch_seq += 1
            cur["batch_id"] = self.batch_seq
        elif cur["state"] != PingPongBatchState.FILLING:
            self.full_events += 1
            self.drops += samples_count
            return -105  # -ENOBUFS

        space = self.batch_size - cur["filled"]
        if bytes_count > space:
            self.full_events += 1
            self.drops += samples_count
            return -105

        cur["filled"] += bytes_count
        self.occupied += bytes_count

        if cur["filled"] >= self.batch_size:
            cur["state"] = PingPongBatchState.READY
            self.fill_idx = (self.fill_idx + 1) % self.num_buffers
            next_b = self.buffers[self.fill_idx]
            if next_b["state"] == PingPongBatchState.FREE:
                next_b["state"] = PingPongBatchState.FILLING
                next_b["filled"] = 0
                next_b["committed"] = 0
                self.batch_seq += 1
                next_b["batch_id"] = self.batch_seq

        return 0

    def peek_batch(self, bytes_req: int):
        drain = self.buffers[self.drain_idx]
        if drain["state"] == PingPongBatchState.READY:
            drain["state"] = PingPongBatchState.WRITING
        elif drain["state"] != PingPongBatchState.WRITING:
            if drain["state"] == PingPongBatchState.FILLING and drain["filled"] > 0:
                drain["state"] = PingPongBatchState.WRITING
            else:
                return 0, 0

        uncommitted = drain["filled"] - drain["committed"]
        to_read = min(bytes_req, uncommitted)
        return to_read, drain["id"]

    def commit_batch(self, bytes_committed: int):
        drain = self.buffers[self.drain_idx]
        if drain["state"] not in (PingPongBatchState.WRITING, PingPongBatchState.FILLING):
            return -22  # -EINVAL

        drain["committed"] += bytes_committed
        self.occupied -= bytes_committed

        if drain["committed"] >= drain["filled"]:
            drain["state"] = PingPongBatchState.FREE
            drain["filled"] = 0
            drain["committed"] = 0
            self.drain_idx = (self.drain_idx + 1) % self.num_buffers
        return 0

    def flush_publish(self):
        cur = self.buffers[self.fill_idx]
        if cur["state"] == PingPongBatchState.FILLING and cur["filled"] > 0:
            cur["state"] = PingPongBatchState.READY
            return 0
        return 1

    def reset(self):
        for b in self.buffers:
            b["state"] = PingPongBatchState.FREE
            b["filled"] = 0
            b["committed"] = 0
        self.fill_idx = 0
        self.drain_idx = 0
        self.occupied = 0
        self.batch_seq += 1
        self.buffers[0]["state"] = PingPongBatchState.FILLING
        self.buffers[0]["batch_id"] = self.batch_seq


def test_pingpong_buffer_exhaustion_and_non_overwrite():
    """Verify that when both 256 KiB buffers are occupied, incoming samples are rejected without overwrite."""
    mgr = MockPingPongManager(num_buffers=2, batch_size=256 * 1024)
    block_bytes = 8192
    block_samples = 4096

    # Fill Buffer 0 completely (32 blocks * 8192 = 256 KiB)
    for _ in range(32):
        assert mgr.push_block(block_bytes, block_samples) == 0

    assert mgr.buffers[0]["state"] == PingPongBatchState.READY
    assert mgr.buffers[1]["state"] == PingPongBatchState.FILLING
    assert mgr.fill_idx == 1

    # Fill Buffer 1 completely
    for _ in range(32):
        assert mgr.push_block(block_bytes, block_samples) == 0

    assert mgr.buffers[1]["state"] == PingPongBatchState.READY
    assert mgr.fill_idx == 0

    # Both buffers are READY. Attempting another push must return -ENOBUFS and drop samples
    ret = mgr.push_block(block_bytes, block_samples)
    assert ret == -105
    assert mgr.drops == block_samples
    assert mgr.full_events == 1
    # Buffer contents must not be overwritten
    assert mgr.buffers[0]["filled"] == 256 * 1024
    assert mgr.buffers[1]["filled"] == 256 * 1024


def test_pingpong_failed_write_preserves_uncommitted_buffer():
    """Verify that a write failure keeps the buffer in WRITING state without committing."""
    mgr = MockPingPongManager(num_buffers=2, batch_size=256 * 1024)
    # Fill Buffer 0
    for _ in range(32):
        mgr.push_block(8192, 4096)

    # Writer begins draining Buffer 0
    to_read, buf_id = mgr.peek_batch(8192)
    assert to_read == 8192
    assert buf_id == 0
    assert mgr.buffers[0]["state"] == PingPongBatchState.WRITING

    # Simulate SD write failure (fs_write fails -> commit is NOT called)
    # The buffer MUST remain in WRITING state with 0 committed bytes
    assert mgr.buffers[0]["state"] == PingPongBatchState.WRITING
    assert mgr.buffers[0]["committed"] == 0
    assert mgr.occupied == 256 * 1024


def test_pingpong_partial_buffer_flushed_at_session_stop():
    """Verify that a partially filled buffer is transitioned to READY on session stop and drained cleanly."""
    mgr = MockPingPongManager(num_buffers=2, batch_size=256 * 1024)
    # Push 3 blocks = 24 KiB (partial buffer)
    for _ in range(3):
        mgr.push_block(8192, 4096)

    assert mgr.buffers[0]["state"] == PingPongBatchState.FILLING
    assert mgr.buffers[0]["filled"] == 24 * 1024

    # Session stop triggers flush_publish
    ret = mgr.flush_publish()
    assert ret == 0
    assert mgr.buffers[0]["state"] == PingPongBatchState.READY

    # Writer drains the 24 KiB
    drained_bytes = 0
    while mgr.occupied > 0:
        chunk, _ = mgr.peek_batch(8192)
        assert chunk > 0
        mgr.commit_batch(chunk)
        drained_bytes += chunk

    assert drained_bytes == 24 * 1024
    assert mgr.occupied == 0
    assert mgr.buffers[0]["state"] == PingPongBatchState.FREE


def test_pingpong_reset_prepares_clean_state_for_next_session():
    """Verify that reset() cleanly resets buffer states between sessions."""
    mgr = MockPingPongManager(num_buffers=2, batch_size=256 * 1024)
    # Fill some data
    mgr.push_block(8192, 4096)
    assert mgr.occupied > 0

    # Reset
    mgr.reset()
    assert mgr.occupied == 0
    assert mgr.fill_idx == 0
    assert mgr.drain_idx == 0
    assert mgr.buffers[0]["state"] == PingPongBatchState.FILLING
    assert mgr.buffers[1]["state"] == PingPongBatchState.FREE

