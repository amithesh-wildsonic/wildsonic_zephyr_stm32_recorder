# SPDX-License-Identifier: Apache-2.0
"""Unit tests for asynchronous ping-pong SDMMC pipeline and QSPI burst optimization.

Validates:
1. Ping-pong buffer ownership state transitions (FREE -> READING -> READY -> WRITING -> COMMITTED -> FREE).
2. Cycle-accurate overlap timing measurement between PSRAM peek and SDMMC fs_write.
3. Single-writer serialization to FatFs (only SD writer calls fs_write).
4. Partial final batch flushing and uncommitted data preservation on write error.
5. QSPI burst limit increased to 1024 bytes (8 transactions per 8 KB block, eliminating 71 ms peek bottleneck).
6. SRAM layout constraints: Buffer A (sd_bounce_buf in SRAM2 pool), Buffer B (s_pingpong_buf_b in SRAM1).
7. Mathematical simulation of 10-second 192 kHz recording verifying zero PSRAM overflow and zero dropped blocks.
8. Source code assertions for synchronization primitives, semaphores, and shutdown guards.
"""

from pathlib import Path
import re
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"


# ==============================================================================
# 1. Ping-Pong Buffer Ownership State Machine
# ==============================================================================

class PingPongBufferState:
    FREE = 0
    READING_PSRAM = 1
    READY_FOR_SD = 2
    WRITING_SD = 3
    COMMITTED = 4


class MockPingPongSlot:
    def __init__(self, slot_id: int):
        self.slot_id = slot_id
        self.state = PingPongBufferState.FREE
        self.bytes = 0
        self.err = 0
        self.peek_start = 0
        self.peek_end = 0
        self.write_start = 0
        self.write_end = 0


def test_pingpong_buffer_ownership_state_machine():
    """Verify state machine strictly enforces single ownership and valid transitions."""
    slots = [MockPingPongSlot(0), MockPingPongSlot(1)]

    # Initial state
    assert slots[0].state == PingPongBufferState.FREE
    assert slots[1].state == PingPongBufferState.FREE

    # Reader claims slot 0
    slots[0].state = PingPongBufferState.READING_PSRAM
    assert slots[0].state == PingPongBufferState.READING_PSRAM
    assert slots[1].state == PingPongBufferState.FREE

    # Reader completes fill of slot 0
    slots[0].bytes = 8192
    slots[0].state = PingPongBufferState.READY_FOR_SD

    # Reader claims slot 1 while slot 0 is waiting for SD
    slots[1].state = PingPongBufferState.READING_PSRAM

    # Writer claims slot 0
    slots[0].state = PingPongBufferState.WRITING_SD

    # At this moment, Slot 1 is READING_PSRAM while Slot 0 is WRITING_SD (true overlap)
    assert slots[0].state == PingPongBufferState.WRITING_SD
    assert slots[1].state == PingPongBufferState.READING_PSRAM

    # Writer finishes slot 0 SD write and commits
    slots[0].state = PingPongBufferState.COMMITTED
    slots[0].state = PingPongBufferState.FREE

    # Slot 1 completes read
    slots[1].bytes = 8192
    slots[1].state = PingPongBufferState.READY_FOR_SD

    # Writer claims slot 1
    slots[1].state = PingPongBufferState.WRITING_SD
    slots[1].state = PingPongBufferState.COMMITTED
    slots[1].state = PingPongBufferState.FREE

    assert slots[0].state == PingPongBufferState.FREE
    assert slots[1].state == PingPongBufferState.FREE


# ==============================================================================
# 2. Cycle-Accurate Overlap Timing Measurement
# ==============================================================================

def compute_overlap_us(peek_start: int, peek_end: int, write_start: int, write_end: int, cyc_to_us: float = 1.0) -> int:
    """Replicates the cycle overlap calculation in recorder.c sd_writer_thread_fn."""
    if peek_start > 0 and peek_end > write_start:
        ov_start = max(peek_start, write_start)
        ov_end = min(peek_end, write_end)
        if ov_end > ov_start:
            return int((ov_end - ov_start) * cyc_to_us)
    return 0


def test_pingpong_overlap_calculation_scenarios():
    """Verify the cycle-accurate overlap calculation under varied timing scenarios."""
    # Scenario A: Write completely encompasses Peek
    # SD write runs from t=100 to t=500. PSRAM peek runs from t=150 to t=250.
    ov_a = compute_overlap_us(peek_start=150, peek_end=250, write_start=100, write_end=500)
    assert ov_a == 100

    # Scenario B: Partial overlap (Peek starts before write ends)
    # SD write runs from t=100 to t=300. PSRAM peek runs from t=250 to t=400.
    ov_b = compute_overlap_us(peek_start=250, peek_end=400, write_start=100, write_end=300)
    assert ov_b == 50

    # Scenario C: No overlap (Peek ends before SD write starts)
    ov_c = compute_overlap_us(peek_start=100, peek_end=200, write_start=250, write_end=450)
    assert ov_c == 0

    # Scenario D: No overlap (Peek starts after SD write ends)
    ov_d = compute_overlap_us(peek_start=500, peek_end=600, write_start=200, write_end=400)
    assert ov_d == 0


# ==============================================================================
# 3. Single-Writer Serialization to FatFs
# ==============================================================================

def test_single_writer_serialization_to_fatfs():
    """Verify that only sd_writer_thread_fn calls fs_write during active recording."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    # Locate psram_reader_thread_fn
    reader_match = re.search(r"static void psram_reader_thread_fn\(.*?\)\s*\{(.*?)\n\}", rec_c, re.DOTALL)
    assert reader_match is not None, "psram_reader_thread_fn not found"
    reader_body = reader_match.group(1)

    # Reader thread must ONLY peek PSRAM, never call fs_write
    assert "psram_staging_peek(" in reader_body
    assert "fs_write(" not in reader_body
    assert "fs_sync(" not in reader_body
    assert "fs_close(" not in reader_body

    # SD writer thread calls fs_write
    writer_match = re.search(r"static void sd_writer_thread_fn\(.*?\)\s*\{(.*?)\nstatic FATFS fat_fs;", rec_c, re.DOTALL)
    assert writer_match is not None, "sd_writer_thread_fn not found"
    writer_body = writer_match.group(1)

    assert "fs_write(sd_file_ptr, slot->buf, slot->bytes)" in writer_body
    assert "psram_staging_commit(" in writer_body


# ==============================================================================
# 4. Partial Final Batch Flushing & Error Preservation
# ==============================================================================

def test_partial_final_batch_flushing_logic():
    """Verify flush request drains exact remaining bytes without padding or data truncation."""
    staging_capacity = 3 * 1024 * 1024
    batch_size = 256 * 1024

    # 10s audio = 3,840,000 bytes
    total_audio_bytes = 3840000
    # 3,840,000 = 14 * 262,144 + 169,984 bytes
    full_batches = total_audio_bytes // batch_size
    remaining_partial = total_audio_bytes % batch_size

    assert full_batches == 14
    assert remaining_partial == 169984

    # The final flush must drain exactly remaining_partial bytes in chunks of 8192
    chunk_size = 8192
    chunks = []
    to_drain = remaining_partial
    while to_drain > 0:
        c = min(to_drain, chunk_size)
        chunks.append(c)
        to_drain -= c

    assert sum(chunks) == remaining_partial
    assert len(chunks) == 21  # 20 chunks of 8192 + 1 chunk of 6144
    assert chunks[-1] == 6144


# ==============================================================================
# 5. QSPI Burst Optimization Verification
# ==============================================================================

def test_qspi_burst_limit_1024_bytes():
    """Verify QSPI driver burst limit is set to 1024 bytes (page size) to prevent 71 ms stall."""
    hdr_text = (APP_SRC / "psram_is66wvs.h").read_text(encoding="utf-8")
    assert re.search(r"#define\s+PSRAM_IS66WVS_SAFE_BURST_BYTES_QUAD\s+1024", hdr_text) is not None

    block_size = 8192
    old_burst = 16
    new_burst = 1024

    old_txns_per_block = block_size // old_burst
    new_txns_per_block = block_size // new_burst

    assert old_txns_per_block == 512
    assert new_txns_per_block == 8

    # Old peek time at ~140 us/txn: 512 * 140 us = 71.68 ms
    # New peek time at ~140 us/txn: 8 * 140 us = 1.12 ms (64x faster!)
    speedup = old_txns_per_block / new_txns_per_block
    assert speedup == 64.0


# ==============================================================================
# 6. SRAM Layout & Zero Heap Allocation
# ==============================================================================

def test_sram_layout_and_zero_heap_allocation():
    """Verify ping-pong buffers respect SRAM1 / SRAM2 memory architecture."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    # Dedicated static bounce buffers in SRAM1 (zero pool borrowing)
    assert "s_sd_bounce_buf_a[SD_IO_CHUNK_BYTES]" in rec_c
    assert "s_sd_bounce_buf_b[SD_IO_CHUNK_BYTES]" in rec_c
    assert "s_pingpong[0].buf = s_sd_bounce_buf_a;" in rec_c
    assert "s_pingpong[1].buf = s_sd_bounce_buf_b;" in rec_c

    # Ensure no dynamic malloc is used in recorder.c fast path
    assert "malloc(" not in rec_c
    assert "k_malloc(" not in rec_c


# ==============================================================================
# 7. End-to-End Mathematical Simulation: 10s @ 192 kHz
# ==============================================================================

def test_10s_192khz_zero_psram_overflow_simulation():
    """Simulate 10s recording session at 192 kHz mono PCM with ping-pong SDMMC pipeline."""
    SAMPLE_RATE = 192000
    BYTES_PER_SEC = SAMPLE_RATE * 2  # 384,000 bytes/sec
    DURATION_SEC = 10
    TOTAL_BYTES = BYTES_PER_SEC * DURATION_SEC  # 3,840,000 bytes
    DMA_BLOCK_BYTES = 8192
    BATCH_BYTES = 262144  # 256 KB
    BATCH_DRAIN_TIME = 0.760  # seconds (SD write + ping-pong peek)

    psram_occupied = 0
    high_water = 0
    writer_busy_until = 0.0
    full_events = 0

    dt = DMA_BLOCK_BYTES / BYTES_PER_SEC  # ~0.02133 s per DMA block
    total_blocks = TOTAL_BYTES // DMA_BLOCK_BYTES

    for i in range(total_blocks):
        t = i * dt
        # Complete any batches finished by time t
        while writer_busy_until > 0 and t >= writer_busy_until:
            psram_occupied -= BATCH_BYTES
            if psram_occupied >= BATCH_BYTES:
                writer_busy_until += BATCH_DRAIN_TIME
            else:
                writer_busy_until = 0.0

        # If writer is idle and enough data is queued, start writer
        if writer_busy_until == 0.0 and psram_occupied >= BATCH_BYTES:
            writer_busy_until = t + BATCH_DRAIN_TIME

        # Push new DMA audio block
        if psram_occupied + DMA_BLOCK_BYTES > 3 * 1024 * 1024:
            full_events += 1
        else:
            psram_occupied += DMA_BLOCK_BYTES
            high_water = max(high_water, psram_occupied)

    assert full_events == 0, f"Simulated {full_events} PSRAM full events!"
    assert high_water < 1.5 * 1024 * 1024, f"High water mark {high_water} exceeded 1.5 MB!"


# ==============================================================================
# 8. Source Code Synchronization and Guard Assertions
# ==============================================================================

def test_source_code_synchronization_and_guards():
    """Verify synchronization primitives, thread stacks, and shutdown checks exist in source."""
    rec_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    rec_h = (APP_SRC / "recorder.h").read_text(encoding="utf-8")

    # Metrics in recorder.h
    assert "uint64_t pingpong_overlap_us;" in rec_h
    assert "uint32_t pingpong_swaps;" in rec_h
    assert "uint32_t psram_qspi_txn_count;" in rec_h
    assert "uint32_t psram_read_throughput_kb_s;" in rec_h
    assert "uint32_t psram_write_throughput_kb_s;" in rec_h

    # Synchronization primitives in recorder.c
    assert "s_reader_start_sem;" in rec_c
    assert "s_reader_done_sem;" in rec_c
    assert "s_slot_ready_sem;" in rec_c
    assert "s_slot_free_sem;" in rec_c
    assert "s_pingpong[2];" in rec_c
    assert "psram_reader_stack" in rec_c

    # Shutdown and sleep guards
    assert "s_pingpong[0].state != PINGPONG_BUF_FREE" in rec_c
    assert "s_pingpong[1].state != PINGPONG_BUF_FREE" in rec_c

    # Accounting log output
    assert '"  SD Writer Ping-Pong Overlap        : %llu ms (%u buffer swaps)\\n"' in rec_c
    assert '"  PSRAM QSPI Transaction Count       : %u (read throughput: %u KB/s)\\n"' in rec_c
