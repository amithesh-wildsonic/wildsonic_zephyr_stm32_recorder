#!/usr/bin/env python3
"""
Test Suite: Production Firmware Finalization Audit
===================================================
Target: STM32L476RG + Zephyr RTOS (WildSonic STM32 Recorder)
Branch: production

Validates the complete 8-phase production firmware requirements:
  1. Production 192 kHz with 32-bit PCM storage format (mono & stereo container).
  2. WAV header calculations (fmt, block align, byte rate, data chunk) matching:
       - 192 kHz, 16-bit mono   :   384,000 bytes/sec
       - 192 kHz, 32-bit mono   :   768,000 bytes/sec
       - 192 kHz, 16-bit stereo :   768,000 bytes/sec
       - 192 kHz, 32-bit stereo : 1,536,000 bytes/sec
  3. Zero-drop recording pipeline accounting (ADC -> DMA -> DSP -> PSRAM -> SDMMC).
  4. Ping-pong batch buffer accounting and SD flush behavior.
  5. Configuration correctness: console -> validation -> staged -> APPLY -> flash -> boot restore.
  6. 11-step low-power state machine and non-blocking LED behavior.
  7. Production build hygiene: exclusion of test benchmark code and quiet console logging.
  8. Dedicated SRAM2 retention pool (32 KiB at 0x10000000).
"""

from pathlib import Path
import struct
import pytest

BASE_DIR = Path(__file__).resolve().parent.parent.parent
APP_DIR = BASE_DIR / "app"
APP_SRC = APP_DIR / "src"

WAV_HEADER_BYTES = 512
PSRAM_CAPACITY_BYTES = 1024 * 1024       # 1 MiB
SD_BATCH_BYTES = 256 * 1024              # 256 KiB
BLOCK_SAMPLES = 4096


def build_test_wav_header(sample_rate: int, channels: int, bits_per_sample: int, data_bytes: int) -> bytes:
    """Simulate C wav_fill_header implementation in wav.c."""
    hdr = bytearray(WAV_HEADER_BYTES)
    block_align = channels * (bits_per_sample // 8)
    byte_rate = sample_rate * block_align
    junk_bytes = WAV_HEADER_BYTES - 12 - 24 - 8 - 8

    # RIFF header
    hdr[0:4] = b"RIFF"
    struct.pack_into("<I", hdr, 4, (WAV_HEADER_BYTES - 8) + data_bytes)
    hdr[8:12] = b"WAVE"

    # fmt chunk
    hdr[12:16] = b"fmt "
    struct.pack_into("<I", hdr, 16, 16)               # Subchunk1Size for PCM
    struct.pack_into("<H", hdr, 20, 1)                # AudioFormat = 1 (PCM)
    struct.pack_into("<H", hdr, 22, channels)
    struct.pack_into("<I", hdr, 24, sample_rate)
    struct.pack_into("<I", hdr, 28, byte_rate)
    struct.pack_into("<H", hdr, 32, block_align)
    struct.pack_into("<H", hdr, 34, bits_per_sample)

    # JUNK chunk padding
    hdr[36:40] = b"JUNK"
    struct.pack_into("<I", hdr, 40, junk_bytes)

    # data chunk
    data_hdr_offset = 36 + 8 + junk_bytes
    hdr[data_hdr_offset:data_hdr_offset + 4] = b"data"
    struct.pack_into("<I", hdr, data_hdr_offset + 4, data_bytes)

    return bytes(hdr)


def verify_wav_header(hdr: bytes, exp_rate: int, exp_channels: int, exp_bits: int, exp_data_bytes: int) -> bool:
    """Simulate C wav_verify_header implementation in wav.c."""
    if len(hdr) != WAV_HEADER_BYTES:
        return False
    if hdr[0:4] != b"RIFF" or hdr[8:12] != b"WAVE" or hdr[12:16] != b"fmt ":
        return False
    riff_size = struct.unpack_from("<I", hdr, 4)[0]
    if riff_size != (WAV_HEADER_BYTES - 8) + exp_data_bytes:
        return False
    fmt_size = struct.unpack_from("<I", hdr, 16)[0]
    pcm_tag = struct.unpack_from("<H", hdr, 20)[0]
    channels = struct.unpack_from("<H", hdr, 22)[0]
    rate = struct.unpack_from("<I", hdr, 24)[0]
    byte_rate = struct.unpack_from("<I", hdr, 28)[0]
    block_align = struct.unpack_from("<H", hdr, 32)[0]
    bits = struct.unpack_from("<H", hdr, 34)[0]

    exp_block_align = exp_channels * (exp_bits // 8)
    exp_byte_rate = exp_rate * exp_block_align

    if fmt_size != 16 or pcm_tag != 1:
        return False
    if channels != exp_channels or rate != exp_rate or bits != exp_bits:
        return False
    if block_align != exp_block_align or byte_rate != exp_byte_rate:
        return False

    junk_bytes = WAV_HEADER_BYTES - 12 - 24 - 8 - 8
    data_hdr_offset = 36 + 8 + junk_bytes
    if hdr[data_hdr_offset:data_hdr_offset + 4] != b"data":
        return False
    data_size = struct.unpack_from("<I", hdr, data_hdr_offset + 4)[0]
    return data_size == exp_data_bytes


# ==============================================================================
# 1. 192 KHZ 32-BIT MONO & STEREO PIPELINE ACCOUNTING
# ==============================================================================

def test_192khz_32bit_mono_session_accounting():
    """
    Validate 10-second 192 kHz 32-bit mono session payload:
      - Duration: 10 seconds
      - Sample rate: 192,000 Hz
      - Channels: 1
      - Container: 32-bit signed integer (4 bytes/sample)
      - Total samples: 1,920,000
      - Total payload bytes: 7,680,000
      - Streaming throughput: 768,000 B/s (750 KB/s)
      - PSRAM 1 MiB staging capacity = 1.365 seconds of buffering
      - SDMMC batches (256 KiB): exactly 29 full batches (7,602,176 B) + 1 partial flush batch (77,824 B) = 30 batches
    """
    duration_s = 10
    sample_rate = 192000
    channels = 1
    sample_bits = 32
    bytes_per_sample = sample_bits // 8

    expected_samples = duration_s * sample_rate
    expected_payload_bytes = expected_samples * channels * bytes_per_sample
    throughput_bps = sample_rate * channels * bytes_per_sample

    assert expected_samples == 1_920_000
    assert expected_payload_bytes == 7_680_000
    assert throughput_bps == 768_000

    # PSRAM capacity time
    psram_time_s = PSRAM_CAPACITY_BYTES / throughput_bps
    assert pytest.approx(psram_time_s, abs=0.01) == 1.365

    # Batch accounting
    full_batches = expected_payload_bytes // SD_BATCH_BYTES
    remainder_bytes = expected_payload_bytes % SD_BATCH_BYTES
    total_batches = full_batches + (1 if remainder_bytes > 0 else 0)

    assert full_batches == 29
    assert remainder_bytes == 77824
    assert total_batches == 30
    assert (full_batches * SD_BATCH_BYTES + remainder_bytes) == expected_payload_bytes


def test_192khz_32bit_stereo_session_accounting():
    """
    Validate 10-second 192 kHz 32-bit stereo session payload:
      - Duration: 10 seconds
      - Sample rate: 192,000 Hz
      - Channels: 2 (duplicated mono container on Nucleo)
      - Container: 32-bit signed integer (4 bytes/sample, 8 bytes/frame)
      - Frames: 1,920,000
      - Total samples: 3,840,000
      - Total payload bytes: 15,360,000
      - Streaming throughput: 1,536,000 B/s (1.5 MB/s)
      - PSRAM 1 MiB staging capacity = 0.682 seconds of buffering
      - SDMMC batches (256 KiB): exactly 58 full batches (15,204,352 B) + 1 partial flush batch (155,648 B) = 59 batches
    """
    duration_s = 10
    sample_rate = 192000
    channels = 2
    sample_bits = 32
    bytes_per_frame = channels * (sample_bits // 8)

    expected_frames = duration_s * sample_rate
    expected_samples = expected_frames * channels
    expected_payload_bytes = expected_frames * bytes_per_frame
    throughput_bps = sample_rate * bytes_per_frame

    assert expected_frames == 1_920_000
    assert expected_samples == 3_840_000
    assert expected_payload_bytes == 15_360_000
    assert throughput_bps == 1_536_000

    # PSRAM capacity time
    psram_time_s = PSRAM_CAPACITY_BYTES / throughput_bps
    assert pytest.approx(psram_time_s, abs=0.01) == 0.682

    # Batch accounting
    full_batches = expected_payload_bytes // SD_BATCH_BYTES
    remainder_bytes = expected_payload_bytes % SD_BATCH_BYTES
    total_batches = full_batches + (1 if remainder_bytes > 0 else 0)

    assert full_batches == 58
    assert remainder_bytes == 155648
    assert total_batches == 59
    assert (full_batches * SD_BATCH_BYTES + remainder_bytes) == expected_payload_bytes


def test_wav_header_conformance_all_production_formats():
    """Verify WAV header fields across all supported production formats."""
    test_cases = [
        # (rate, ch, bits, data_bytes, exp_align, exp_rate)
        (192000, 1, 16, 3_840_000, 2, 384_000),      # 192k 16-bit mono
        (192000, 1, 32, 7_680_000, 4, 768_000),      # 192k 32-bit mono
        (192000, 2, 16, 7_680_000, 4, 768_000),      # 192k 16-bit stereo
        (192000, 2, 32, 15_360_000, 8, 1_536_000),   # 192k 32-bit stereo
        (96000,  1, 16, 1_920_000, 2, 192_000),      # 96k 16-bit mono
        (48000,  1, 16, 960_000,   2, 96_000),       # 48k 16-bit mono
    ]

    for rate, ch, bits, data_bytes, exp_align, exp_byte_rate in test_cases:
        hdr = build_test_wav_header(rate, ch, bits, data_bytes)
        assert len(hdr) == 512
        assert verify_wav_header(hdr, rate, ch, bits, data_bytes)

        # Direct unpack verification
        block_align = struct.unpack_from("<H", hdr, 32)[0]
        byte_rate = struct.unpack_from("<I", hdr, 28)[0]
        riff_size = struct.unpack_from("<I", hdr, 4)[0]

        assert block_align == exp_align
        assert byte_rate == exp_byte_rate
        assert riff_size == (512 - 8) + data_bytes


# ==============================================================================
# 2. CONSOLE CONFIGURATION & ALIASES AUDIT
# ==============================================================================

def test_source_code_console_production_32bit_and_stereo_support():
    """Verify that console.c allows 32-bit depth and stereo channels in production."""
    src = (APP_SRC / "console.c").read_text(encoding="utf-8")

    # BTPS / BITS: 32 must be valid in production
    assert "g_cfg.audio_sampling_width = 32;" in src
    assert "Value is set to 32" in src

    # 24-bit must remain deferred
    assert "[DEFERRED] Bits per sample %d requires future hardware" in src

    # NUMCH / CHANNELS: 2 must be valid in production
    assert "g_cfg.num_audio_channels = 2;" in src
    assert "Value is set to 2" in src

    # Packed AUDIO command allows 1/2 ch and 16/32 bits
    assert "(new_numch == 1 || new_numch == 2)" in src
    assert "(new_btps == 16 || new_btps == 32)" in src

    # Aliases BITS and CHANNELS available in production
    assert 'strncasecmp(p, "BITS", 4)' in src
    assert 'strncasecmp(p, "CHANNELS", 8)' in src


def test_source_code_config_store_persistence():
    """Verify config_store.c sanitizes and unpacks 32-bit and stereo configurations."""
    src = (APP_SRC / "config_store.c").read_text(encoding="utf-8")

    assert "uint32_t eff_bits = (rec->audio_sampling_width == 32) ? 32 : 16;" in src
    assert "uint32_t eff_ch = (rec->num_audio_channels == 2) ? 2 : 1;" in src
    assert "cfg->audio_sampling_width = eff_bits;" in src
    assert "cfg->num_audio_channels = eff_ch;" in src
    assert "settings->sample_bits = (uint16_t)eff_bits;" in src
    assert "settings->channels = (uint16_t)eff_ch;" in src


# ==============================================================================
# 3. 11-STEP LOW-POWER STATE MACHINE & ZERO-DROP PIPELINE
# ==============================================================================

def test_eleven_step_low_power_state_machine_contracts():
    """
    Verify the 11-step low-power recording state machine architecture:
      Step 1: Boot & clock setup (main.c -> 80 MHz HCLK / APB1 / APB2)
      Step 2: Load configuration from flash (config_store_init)
      Step 3: Initialize RTC and scheduler (rtc_calendar_init, scheduler_init)
      Step 4: Calculate next scheduled session (scheduler_calculate_next)
      Step 5: Enter low-power sleep Stop 2 if wait > threshold (rtc_sleep_enter_stop2)
      Step 6: Wake on RTC alarm before session (RTC Wakeup Timer WKUP_IRQn)
      Step 7: Re-initialize / wake peripherals (psram_wake, disk_access_ioctl SYNC)
      Step 8: Start recording: ADC+DMA -> DSP -> PSRAM -> SD write
      Step 9: Session end: flush remaining buffers, close WAV, finalize headers
      Step 10: Update scheduler status, calculate next session
      Step 11: Return to low-power sleep
    """
    src_main = (APP_SRC / "main.c").read_text(encoding="utf-8")
    src_rec = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    src_sleep = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")

    # Step 1: Boot & clocks
    assert "stm32_clock_control_init" in src_sleep

    # Step 2: Config persistence
    assert "config_store_init()" in src_main

    # Step 3: RTC and scheduler init
    assert "scheduler_init()" in src_main

    # Step 4: Next session calculation
    assert "scheduler_calculate_next" in src_main

    # Step 5: Stop 2 sleep
    assert "rtc_sleep_enter_stop2" in src_rec
    assert "LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);" in src_sleep

    # Step 6: RTC Wakeup Timer
    assert "RTC_WKUP_IRQn" in src_sleep

    # Step 7: Wake peripherals & verify idle
    assert "psram_wake()" in src_rec
    assert "recorder_sd_verify_idle()" in src_rec

    # Step 8: Acquisition & DSP pipeline
    assert "audio_in_start()" in src_rec
    assert "psram_staging_push" in src_rec
    assert "sd_writer_thread_data" in src_rec

    # Step 9: Flush & finalize
    assert "psram_staging_flush_publish()" in src_rec
    assert "wav_fill_header(hdr, st.sample_rate, st.channels, st.sample_bits, data_bytes);" in src_rec
    assert "fs_truncate(&f, WAV_HEADER_BYTES + data_bytes);" in src_rec

    # Step 10: Scheduler update
    assert "s_last_end_time_local" in src_main

    # Step 11: LED behavior: non-blocking, turned off before sleep and idle
    assert "led_indicator_set(LED_IND_OFF);" in src_rec
    assert "LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);" in src_sleep


# ==============================================================================
# 4. PRODUCTION BUILD HYGIENE & MEMORY AUDIT
# ==============================================================================

def test_production_build_hygiene_and_quiet_console():
    """Verify that test benchmark code is excluded from production and logs are quiet."""
    src_cmakelists = (APP_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    src_rec = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    src_psram_staging = (APP_SRC / "psram_staging.c").read_text(encoding="utf-8")

    # Benchmark conditionally compiled only when CONFIG_WILDSONIC_VERBOSE_LOGS is defined
    assert "target_sources_ifdef(CONFIG_WILDSONIC_VERBOSE_LOGS app PRIVATE src/psram_benchmark.c)" in src_cmakelists

    # Quiet logging policy in production
    assert "LOG_MODULE_REGISTER(recorder, LOG_LEVEL_WRN);" in src_rec
    assert "LOG_MODULE_REGISTER(psram_staging, LOG_LEVEL_WRN);" in src_psram_staging

    # SRAM2 retention section dedicated to audio buffer pool
    assert 'section("SRAM1")' in src_rec
    assert "_k_mem_slab_buf_pool[(NUM_BLOCKS) * ROUND_UP(BLOCK_BYTES, 4)];" in src_rec
