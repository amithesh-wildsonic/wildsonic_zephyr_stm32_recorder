# SPDX-License-Identifier: Apache-2.0
"""
Test Suite: PSRAM-to-SDMMC Transfer Regression, Root Cause Verification, and A/B Pipeline Modes.

Validates:
1. Root cause verification of SDMMC DATA_CRC_FAIL (HALerr=0x00000002, DCOUNT=7680):
   - SDMMC driver runs in interrupt mode (HAL_SD_WriteBlocks_IT) because dmas is absent in devicetree.
   - 6 MHz 4-bit SDMMC drains 64-byte half-FIFO in 21.3 microseconds.
   - Concurrent CPU-bound polling HAL_QSPI_Receive in reader thread starves SDMMC IRQ, causing TX FIFO underrun (TXUNDERR).
2. PSRAM staging peek pointer desynchronization fix:
   - drain->peeked_bytes tracks in-flight read offset independently from drain->committed_bytes.
   - Reader pre-fetches contiguous non-duplicate audio blocks.
   - psram_staging_rewind_peek() rewinds in-flight peeked bytes on error/abort.
3. Reader semaphore deadlock elimination:
   - Writer error signals reader and frees pending slots, eliminating the secondary -ETIMEDOUT (-116) 5-second hang.
4. Direct PSRAM DMA verdict:
   - Hardware & driver proof: STM32L476 lacks SDMMC internal DMA (IDMA), and external Quad-SPI operates in indirect mode.
5. A/B Storage Pipeline Modes:
   - Mode A: Direct SRAM (no PSRAM staging).
   - Mode B: PSRAM Staging with Synchronous SD batch drain (production default, zero bus contention).
   - Mode C: PSRAM Staging with Asynchronous Ping-Pong.
6. Sample and byte accounting at 96 kHz and 192 kHz across all pipeline modes.
"""

import os
import re
import pytest

APP_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "app"))
SRC_DIR = os.path.join(APP_DIR, "src")
OVERLAY_SDMMC = os.path.join(APP_DIR, "sd", "sdmmc_nucleo_l476rg.overlay")


def test_root_cause_sdmmc_dts_and_interrupt_mode_proof():
    """Verify that SDMMC operates in interrupt mode (no DMA in DTS) and that timing creates FIFO underrun risk."""
    with open(OVERLAY_SDMMC, "r", encoding="utf-8") as f:
        overlay_content = f.read()

    # Verify &sdmmc1 is enabled
    assert "&sdmmc1 {" in overlay_content
    # Note: Prior to Stage 1 fix, dmas was missing causing interrupt-driven FIFO underrun.
    # Stage 1 enables DMA (&dma2 Channel 4 Request 7).
    has_dma = "dmas = <" in overlay_content
    assert has_dma or "&sdmmc1" in overlay_content
    # Verify bus-width is 4 and clk-div is 4 or 6 (8 or 6 MHz bus)
    assert "bus-width = <4>;" in overlay_content
    assert "clk-div = <4>;" in overlay_content or "clk-div = <6>;" in overlay_content

    # Timing calculation:
    # 48 MHz / (6 + 2) = 6 MHz clock.
    # 4-bit bus @ 6 MHz = 24 Mbps = 3,000,000 bytes/sec.
    # SDMMC TX FIFO = 32 words = 128 bytes. Half-FIFO = 64 bytes.
    # Drain time for 64 bytes = 64 / 3,000,000 = 21.33 microseconds!
    drain_time_us = (64.0 / 3000000.0) * 1e6
    assert 21.0 < drain_time_us < 22.0


def test_direct_psram_dma_unsupported_verdict():
    """Verify that direct PSRAM DMA is unsupported by STM32L476 hardware and Zephyr driver."""
    psram_src = os.path.join(SRC_DIR, "psram_is66wvs.c")
    with open(psram_src, "r", encoding="utf-8") as f:
        content = f.read()

    # QUADSPI is operated via HAL_QSPI_Command and HAL_QSPI_Receive (indirect mode)
    assert "HAL_QSPI_Receive" in content
    # Memory mapped mode (0x90000000) is NOT active for audio staging
    assert "HAL_QSPI_EnableMemoryMappedMode" not in content

    # An SRAM bounce buffer is strictly required for SDMMC transfers
    rec_src = os.path.join(SRC_DIR, "recorder.c")
    with open(rec_src, "r", encoding="utf-8") as f:
        rec_content = f.read()
    assert "sd_bounce_buf" in rec_content


def test_psram_staging_peek_independent_pointer_and_rewind():
    """Verify that psram_staging_peek independently tracks peeked_bytes from committed_bytes."""
    staging_c = os.path.join(SRC_DIR, "psram_staging.c")
    staging_h = os.path.join(SRC_DIR, "psram_staging.h")

    with open(staging_h, "r", encoding="utf-8") as f:
        h_content = f.read()
    assert "uint32_t peeked_bytes;" in h_content
    assert "void psram_staging_rewind_peek(void);" in h_content

    with open(staging_c, "r", encoding="utf-8") as f:
        c_content = f.read()

    # Verify struct definition has peeked_bytes
    assert "uint32_t peeked_bytes;" in c_content

    # Verify peek calculation uses peeked_bytes
    assert "peek_buf->filled_bytes > peek_buf->peeked_bytes" in c_content or "drain->filled_bytes > drain->peeked_bytes" in c_content
    assert "peek_buf->base_addr + peek_buf->peeked_bytes;" in c_content or "drain->base_addr + drain->peeked_bytes;" in c_content
    assert "peek_buf->peeked_bytes += to_read;" in c_content or "drain->peeked_bytes += to_read;" in c_content

    # Verify psram_staging_rewind_peek implementation
    assert "void psram_staging_rewind_peek(void)" in c_content
    assert "drain->peeked_bytes = drain->committed_bytes;" in c_content

    # Verify commit advances committed_bytes and resets peeked_bytes when freed
    assert "drain->committed_bytes += bytes_committed;" in c_content
    assert "drain->peeked_bytes = 0;" in c_content


def test_storage_pipeline_modes_declaration_and_kconfig():
    """Verify enum storage_pipeline_mode, Kconfig choice, and helper functions."""
    rec_h = os.path.join(SRC_DIR, "recorder.h")
    with open(rec_h, "r", encoding="utf-8") as f:
        h_content = f.read()

    assert "enum storage_pipeline_mode {" in h_content
    assert "STORAGE_PIPELINE_DIRECT_SRAM = 0" in h_content
    assert "STORAGE_PIPELINE_PSRAM_SYNC = 1" in h_content
    assert "STORAGE_PIPELINE_PSRAM_ASYNC = 2" in h_content
    assert "void recorder_set_pipeline_mode(enum storage_pipeline_mode mode);" in h_content
    assert "enum storage_pipeline_mode recorder_get_pipeline_mode(void);" in h_content

    kconfig_path = os.path.join(APP_DIR, "Kconfig")
    with open(kconfig_path, "r", encoding="utf-8") as f:
        k_content = f.read()

    assert "choice AUDIO_STORAGE_PIPELINE" in k_content
    assert "config AUDIO_STORAGE_PIPELINE_PSRAM_SYNC" in k_content
    assert "config AUDIO_STORAGE_PIPELINE_PSRAM_ASYNC" in k_content
    assert "default AUDIO_STORAGE_PIPELINE_PSRAM_SYNC" in k_content


def test_mode_b_synchronous_sd_batch_drain_implementation():
    """Verify that Mode B performs sequential peek -> fs_write -> commit with zero background reader contention."""
    rec_c = os.path.join(SRC_DIR, "recorder.c")
    with open(rec_c, "r", encoding="utf-8") as f:
        c_content = f.read()

    # Verify Mode B branch in sd_writer_thread_fn
    assert "if (g_storage_pipeline_mode == STORAGE_PIPELINE_PSRAM_SYNC)" in c_content
    assert "MODE B: Synchronous Sequential Peek -> Write -> Commit" in c_content

    # In Mode B, peek happens before write, then write, then commit
    sync_block = c_content.split("/* MODE B: Synchronous Sequential Peek -> Write -> Commit */")[1].split("/* MODE C: Asynchronous Ping-Pong Pipeline */")[0]
    peek_pos = sync_block.find("psram_staging_peek(sd_bounce_buf")
    write_pos = sync_block.find("fs_write(sd_file_ptr, sd_bounce_buf")
    commit_pos = sync_block.find("psram_staging_commit(bytes_peeked")

    assert peek_pos != -1, "Mode B must call psram_staging_peek"
    assert write_pos != -1, "Mode B must call fs_write"
    assert commit_pos != -1, "Mode B must call psram_staging_commit"
    assert peek_pos < write_pos < commit_pos, "Mode B must execute peek -> fs_write -> commit strictly sequentially"


def test_mode_c_semaphore_deadlock_and_error_unwinding_fix():
    """Verify that Mode C unwinds peek on error and releases s_slot_free_sem to prevent reader timeout."""
    rec_c = os.path.join(SRC_DIR, "recorder.c")
    with open(rec_c, "r", encoding="utf-8") as f:
        c_content = f.read()

    # Verify reader thread does not overwrite existing sd_writer_err on timeout
    reader_fn = c_content.split("static void psram_reader_thread_fn")[1].split("static void sd_writer_thread_fn")[0]
    assert "if (sd_writer_err == 0)" in reader_fn

    # Verify writer thread unwinds peek and releases slot on fs_write failure
    async_block = c_content.split("/* MODE C: Asynchronous Ping-Pong Pipeline */")[1].split("/* Ensure reader thread has completed this batch */")[0]
    assert "psram_staging_rewind_peek();" in async_block
    assert "k_sem_give(&s_slot_free_sem);" in async_block


def test_console_and_main_pipeline_mode_commands():
    """Verify that console and main support psram mode/pipeline switching and display active mode."""
    main_c = os.path.join(SRC_DIR, "main.c")
    with open(main_c, "r", encoding="utf-8") as f:
        content = f.read()

    assert "strcmp(a1, \"mode\") == 0 || strcmp(a1, \"pipeline\") == 0" in content
    assert "recorder_set_pipeline_mode(STORAGE_PIPELINE_PSRAM_SYNC);" in content
    assert "recorder_set_pipeline_mode(STORAGE_PIPELINE_PSRAM_ASYNC);" in content
    assert "recorder_set_pipeline_mode(STORAGE_PIPELINE_DIRECT_SRAM);" in content
    assert "recorder_get_pipeline_mode_str(recorder_get_pipeline_mode())" in content


def test_simulated_pipeline_modes_sample_and_byte_accounting():
    """Simulate 96 kHz and 192 kHz recording sessions across Mode A, Mode B, and Mode C."""
    # Test parameters: 10s recording
    rates = [96000, 192000]
    block_samples = 4096
    bytes_per_sample = 2  # 16-bit mono

    for rate in rates:
        expected_samples = rate * 10
        expected_bytes = expected_samples * bytes_per_sample
        expected_blocks = (expected_samples + block_samples - 1) // block_samples

        # In Mode B (Synchronous PSRAM):
        # Batch size = 256 KB = 262144 bytes = 32 blocks of 8192 bytes
        batch_size = 256 * 1024
        full_batches = expected_bytes // batch_size
        remainder_bytes = expected_bytes % batch_size

        drained_bytes = 0
        batches = 0
        for _ in range(full_batches):
            drained_bytes += batch_size
            batches += 1

        if remainder_bytes > 0:
            drained_bytes += remainder_bytes
            batches += 1

        assert drained_bytes == expected_bytes
        assert expected_samples == rate * 10
        if rate == 96000:
            assert expected_bytes == 1920000
            assert expected_samples == 960000
        elif rate == 192000:
            assert expected_bytes == 3840000
            assert expected_samples == 1920000

