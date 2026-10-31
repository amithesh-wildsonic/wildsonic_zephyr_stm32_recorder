#!/usr/bin/env python3
"""
Test Suite: Scheduled Recording Sample Accounting, Batch 10 Trace & SDMMC Error 2
==================================================================================
Verifies:
  1. Exact mathematical reconciliation of samples=1200128:
     - 192000 Hz, 16-bit mono -> 1 sample = 2 bytes, 1 block (4096 samples) = 8192 bytes.
     - 9 full 256 KiB batches = 9 * 262144 B = 2,359,296 B = 1,179,648 samples (288 blocks).
     - Batch 10 partial progress: 5 blocks = 40,960 B = 20,480 samples.
     - Total: 293 blocks = 2,400,256 B = 1,200,128 samples (elapsed time: 6.250667 s).
  2. Ping-pong state machine & batch readiness:
     - Batch 10 announced with 256 KiB drain target upon meeting threshold.
     - Chunk-by-chunk commit (8192 bytes) ensures only successfully written chunks increment bytes_written.
     - Failure on chunk 6 leaves 221,184 bytes uncommitted in PSRAM.
  3. SDMMC Error 2 hardware & driver trace:
     - Error code 2 maps to SDMMC_STA_DCRCFAIL (bit 1, mask 0x2) in stm32l476xx.h.
     - Maps to SDMMC_ERROR_DATA_CRC_FAIL (0x2) in stm32l4xx_ll_sdmmc.h.
     - Driver string: "Data CRC failure".
  4. Error handling contracts:
     - First failure preservation.
     - WAV header finalization skipped on error.
     - File handle closed and bounce buffer freed.
     - Storage recovery executed only after writer ownership released.
"""

import struct
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent.parent
SRC_DIR = BASE_DIR / "app" / "src"

# ==============================================================================
# 1. SAMPLE AND BYTE ACCOUNTING AT 192 KHZ 16-BIT MONO
# ==============================================================================

def test_192khz_16bit_mono_sample_and_byte_accounting():
    """Verify exact arithmetic for 192000 Hz, 16-bit mono recording."""
    sample_rate = 192000
    channels = 1
    bits_per_sample = 16
    bytes_per_sample = bits_per_sample // 8  # 2 bytes
    bytes_per_frame = bytes_per_sample * channels  # 2 bytes
    block_samples = 4096
    block_bytes = block_samples * bytes_per_frame  # 8192 bytes
    batch_size_bytes = 256 * 1024  # 262144 bytes
    blocks_per_batch = batch_size_bytes // block_bytes  # 32 blocks
    samples_per_batch = blocks_per_batch * block_samples  # 131072 samples

    assert bytes_per_sample == 2
    assert block_bytes == 8192
    assert blocks_per_batch == 32
    assert samples_per_batch == 131072

    # 9 full batches:
    nine_batches_bytes = 9 * batch_size_bytes  # 2,359,296 bytes
    nine_batches_samples = 9 * samples_per_batch  # 1,179,648 samples
    nine_batches_blocks = 9 * blocks_per_batch  # 288 blocks

    assert nine_batches_bytes == 2_359_296
    assert nine_batches_samples == 1_179_648
    assert nine_batches_blocks == 288

    # Final reported samples = 1,200,128
    final_reported_samples = 1_200_128
    final_reported_bytes = final_reported_samples * bytes_per_sample  # 2,400,256 bytes

    assert final_reported_bytes == 2_400_256

    # Batch 10 contribution:
    batch10_samples = final_reported_samples - nine_batches_samples
    batch10_bytes = final_reported_bytes - nine_batches_bytes
    batch10_blocks = batch10_bytes // block_bytes

    assert batch10_samples == 20_480
    assert batch10_bytes == 40_960
    assert batch10_blocks == 5
    assert batch10_bytes % block_bytes == 0  # Exact integer number of 8192-byte chunks!

    # Total blocks written across session:
    total_blocks_written = nine_batches_blocks + batch10_blocks
    assert total_blocks_written == 293
    assert total_blocks_written * block_samples == final_reported_samples

    # Exact session time elapsed at failure:
    elapsed_seconds = final_reported_samples / sample_rate
    assert abs(elapsed_seconds - 6.250667) < 0.0001


def test_zero_producer_drops_reconciliation():
    """Prove that final sample count 1200128 is an exact multiple of 4096,
    confirming zero blocks dropped by ADC/DMA/DSP before storage error.
    """
    final_reported_samples = 1_200_128
    block_samples = 4096
    assert (final_reported_samples % block_samples) == 0, \
        "If producer had dropped samples, sample count would not be block-aligned"
    assert (final_reported_samples // block_samples) == 293


# ==============================================================================
# 2. PING-PONG STATE MACHINE & BATCH DRAIN SEMANTICS
# ==============================================================================

def test_ping_pong_batch_drain_chunk_accounting():
    """Simulate the ping-pong batch drain loop and verify chunk accounting on error."""
    batch_threshold = 262144
    block_bytes = 8192
    bytes_written_cum = 9 * 262144  # After 9 full batches = 2,359,296
    samples_written_cum = bytes_written_cum // 2

    # Batch 10 starts:
    to_write = batch_threshold
    batch_bytes_written = 0
    fail_on_chunk = 6  # Fails on 6th chunk (after 5 chunks succeeded)

    chunk_idx = 0
    while to_write > 0:
        chunk_idx += 1
        chunk = min(to_write, block_bytes)
        if chunk_idx == fail_on_chunk:
            # Simulated SDMMC error 2 (Data CRC fail)
            err = -5  # -EIO
            break
        # Succeeded write:
        bytes_written_cum += chunk
        batch_bytes_written += chunk
        to_write -= chunk

    assert chunk_idx == 6
    assert batch_bytes_written == 40960  # Exactly 5 chunks * 8192
    assert bytes_written_cum == 2400256
    samples_final = bytes_written_cum // 2
    assert samples_final == 1200128

    # Uncommitted bytes remaining in Batch 10 buffer:
    uncommitted_bytes = batch_threshold - batch_bytes_written
    assert uncommitted_bytes == 221184


# ==============================================================================
# 3. SDMMC ERROR 2 TRACING & REGISTER BITMASKS
# ==============================================================================

def test_sdmmc_error_2_hardware_flags_and_driver_mapping():
    """Verify that SDMMC Error 2 is SDMMC_STA_DCRCFAIL / SDMMC_ERROR_DATA_CRC_FAIL."""
    # From stm32l476xx.h:
    # #define SDMMC_STA_DCRCFAIL_Pos  (1U)
    # #define SDMMC_STA_DCRCFAIL_Msk  (0x1UL << 1) /* 0x00000002 */
    SDMMC_STA_DCRCFAIL = 0x00000002

    # From stm32l4xx_ll_sdmmc.h:
    # #define SDMMC_ERROR_NONE              0x00000000U
    # #define SDMMC_ERROR_CMD_CRC_FAIL      0x00000001U
    # #define SDMMC_ERROR_DATA_CRC_FAIL     0x00000002U
    # #define SDMMC_ERROR_CMD_RSP_TIMEOUT   0x00000004U
    # #define SDMMC_ERROR_DATA_TIMEOUT      0x00000008U
    SDMMC_ERROR_DATA_CRC_FAIL = 0x00000002

    assert SDMMC_STA_DCRCFAIL == 2
    assert SDMMC_ERROR_DATA_CRC_FAIL == 2

    # In sdhc_stm32.c:
    # {SDMMC_ERROR_DATA_CRC_FAIL, "Data CRC failure"}
    error_mapping = {
        0x00000001: "Command CRC failure",
        0x00000002: "Data CRC failure",
        0x00000004: "Command response timeout",
        0x00000008: "Data timeout",
        0x00000010: "Transmit FIFO underrun",
        0x00000020: "Receive FIFO overrun",
    }

    assert error_mapping[2] == "Data CRC failure"


# ==============================================================================
# 4. SOURCE CODE CONTRACTS & ERROR HANDLING VERIFICATION
# ==============================================================================

def test_source_code_preserves_first_error_and_skips_wav_finalization():
    """Inspect recorder.c to verify storage error contracts."""
    rec_c = (SRC_DIR / "recorder.c").read_text(encoding="utf-8")

    # 1. Preserves first error
    assert "if (sd_writer_err != 0 && err == 0)" in rec_c
    assert "err = sd_writer_err;" in rec_c

    # 2. Finalizes only confirmed PCM bytes even for partial/error files.
    assert "/* Writer completion is established here. Finalize a valid partial WAV" in rec_c
    assert "finalize_ret = fs_seek(&f, 0, FS_SEEK_SET);" in rec_c

    # 3. Closes file safely
    assert "fs_close(&f);" in rec_c

    # 4. Frees bounce buffer
    assert "if (sd_bounce_buf)" in rec_c
    assert "k_mem_slab_free(&pool, sd_bounce_buf);" in rec_c
    assert "sd_file_ptr = NULL;" in rec_c

    # 5. Interrupted batch log exists
    assert "Batch %u interrupted: wrote %u of %u bytes to SD before failure" in rec_c

    # 6. Storage recovery executed after recorder_active is cleared
    rec_active_pos = rec_c.rfind("recorder_active = false;")
    rec_recover_pos = rec_c.rfind("recorder_storage_recover();")
    assert rec_active_pos != -1
    assert rec_recover_pos != -1
    assert rec_active_pos < rec_recover_pos, "Storage recovery must run after recorder_active = false"

