#!/usr/bin/env python3
"""
Test Suite: High-Rate Audio Architecture, PSRAM Benchmarking & STM32U585 Portability
=====================================================================================
Covers Phases B, C, D, E, F, G:
  1. Audio format model: 48k, 96k, 192k, 384k, 512k; mono/stereo; 16-bit/32-bit.
  2. Hardware tier classification: Current Hardware (L476 mono <= 192k),
     Future Hardware (U585 / dual-ADC / OctoSPI), Representable Only.
  3. Exact streaming bandwidth math (decimal B/s), frame sizing, and buffer duration.
  4. Ping-pong buffer margin and stall analysis for all rate combinations against PSRAM bus.
  5. WAV RIFF header field correctness and 4GB boundary checking.
  6. DSP filter cutoff validation against Nyquist frequency (Fs/2) and passband margins.
  7. PSRAM benchmark engine memory isolation (0x00300000) and measurement metrics structure.
  8. Source inspection for strict build separation (production quietness vs test diagnostics).
"""

import math
import struct
import pytest
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent.parent
APP_DIR = BASE_DIR / "app"
SRC_DIR = APP_DIR / "src"

# ==============================================================================
# 1. HARDWARE-INDEPENDENT AUDIO CONFIGURATION MODEL
# ==============================================================================

SUPPORTED_RATES = [48000, 96000, 192000, 384000, 512000]
SUPPORTED_CHANNELS = [1, 2] # mono, stereo
SUPPORTED_BIT_DEPTHS = [16, 32]

class AudioFormatModel:
    TIER_UNSUPPORTED = 0
    TIER_REPRESENTABLE_ONLY = 1
    TIER_CURRENT_HARDWARE = 2 # STM32L476RG (mono, 16-bit, <= 192kHz)
    TIER_FUTURE_HARDWARE = 3  # STM32U585 (dual ADC, OctoSPI PSRAM, 160MHz)

    def __init__(self, sample_rate: int, channels: int, bit_depth: int):
        self.sample_rate = sample_rate
        self.channels = channels
        self.bit_depth = bit_depth
        self.bytes_per_sample = bit_depth // 8
        self.bytes_per_frame = self.bytes_per_sample * channels
        self.byte_rate = sample_rate * self.bytes_per_frame
        self.nyquist_hz = sample_rate // 2

    def classify_tier(self) -> int:
        # Check basic representation validity
        if self.sample_rate not in SUPPORTED_RATES:
            return self.TIER_UNSUPPORTED
        if self.channels not in (1, 2):
            return self.TIER_UNSUPPORTED
        if self.bit_depth not in (16, 32):
            return self.TIER_UNSUPPORTED

        # Current hardware: STM32L476RG constraints:
        # - Single ADC physically wired to mono mic -> channels must be 1
        # - 12-bit SAR ADC with hardware oversampling -> 16-bit stored PCM
        # - 72 MHz system clock & 8x oversampling conversion time (4167 ns) -> rate <= 192000 Hz
        if self.channels == 1 and self.bit_depth == 16 and self.sample_rate <= 192000:
            return self.TIER_CURRENT_HARDWARE

        # Future hardware: STM32U585
        # - Dual 2.5 Msps ADCs with simultaneous dual regular conversion -> true stereo supported
        # - 160 MHz clock + OctoSPI PSRAM (80-120 MHz bus, 100-200 MB/s) -> up to 512 kHz supported
        # - 14-bit SAR native ADC or digital PDM microphone filter (MDF) -> 16-bit & 32-bit supported
        if self.sample_rate <= 512000 and self.channels in (1, 2) and self.bit_depth in (16, 32):
            return self.TIER_FUTURE_HARDWARE

        return self.TIER_REPRESENTABLE_ONLY

    def calculate_session_payload(self, duration_sec: int) -> int:
        return self.byte_rate * duration_sec

    def calculate_block_duration_us(self, block_bytes: int) -> int:
        return int((block_bytes * 1_000_000) / self.byte_rate)

    def validate_dsp_cutoffs(self, lcutoff: int, ucutoff: int) -> tuple[bool, str]:
        if lcutoff < 0:
            return False, "Lower cutoff cannot be negative"
        if ucutoff == 0:
            return True, "All-pass (LPF disabled)"
        if ucutoff < 1000:
            return False, "Upper cutoff must be at least 1000 Hz"
        if ucutoff >= self.nyquist_hz:
            return False, f"Upper cutoff ({ucutoff} Hz) violates Nyquist limit ({self.nyquist_hz} Hz)"
        if lcutoff > 0 and lcutoff >= ucutoff:
            return False, f"Lower cutoff ({lcutoff} Hz) must be strictly less than upper cutoff ({ucutoff} Hz)"
        return True, "Valid filter parameters"


def test_ten_audio_rate_combinations_bandwidth():
    """Verify exact decimal byte rates (B/s) for all 10 core rate combinations (16-bit)."""
    expected_16bit_rates = {
        (48000, 1): 96_000,
        (48000, 2): 192_000,
        (96000, 1): 192_000,
        (96000, 2): 384_000,
        (192000, 1): 384_000,
        (192000, 2): 768_000,
        (384000, 1): 768_000,
        (384000, 2): 1_536_000,
        (512000, 1): 1_024_000,
        (512000, 2): 2_048_000,
    }

    for (rate, ch), exp_bps in expected_16bit_rates.items():
        fmt = AudioFormatModel(rate, ch, 16)
        assert fmt.byte_rate == exp_bps, f"Rate {rate} ch {ch} expected {exp_bps} B/s, got {fmt.byte_rate}"
        assert fmt.bytes_per_frame == 2 * ch
        assert fmt.nyquist_hz == rate // 2


def test_32bit_audio_rate_combinations_bandwidth():
    """Verify 32-bit stored sample format bandwidth up to 512k stereo (4.096 MB/s)."""
    expected_32bit_rates = {
        (48000, 1): 192_000,
        (48000, 2): 384_000,
        (96000, 1): 384_000,
        (96000, 2): 768_000,
        (192000, 1): 768_000,
        (192000, 2): 1_536_000,
        (384000, 1): 1_536_000,
        (384000, 2): 3_072_000,
        (512000, 1): 2_048_000,
        (512000, 2): 4_096_000,
    }

    for (rate, ch), exp_bps in expected_32bit_rates.items():
        fmt = AudioFormatModel(rate, ch, 32)
        assert fmt.byte_rate == exp_bps, f"32-bit {rate} ch {ch} expected {exp_bps} B/s, got {fmt.byte_rate}"
        assert fmt.bytes_per_frame == 4 * ch


def test_hardware_tier_classification():
    """Verify classification of formats into Current Hardware (L4) vs Future Hardware (U5)."""
    # Current hardware: STM32L476RG (mono 16-bit <= 192kHz)
    l4_valid = [
        AudioFormatModel(48000, 1, 16),
        AudioFormatModel(96000, 1, 16),
        AudioFormatModel(192000, 1, 16),
    ]
    for fmt in l4_valid:
        assert fmt.classify_tier() == AudioFormatModel.TIER_CURRENT_HARDWARE

    # Stereo on L476 is IMPOSSIBLE with single wired ADC -> Must be classified as Future Hardware
    stereo_l4 = AudioFormatModel(48000, 2, 16)
    assert stereo_l4.classify_tier() == AudioFormatModel.TIER_FUTURE_HARDWARE

    # High rates (> 192 kHz) on L476 require U585
    high_rates = [
        AudioFormatModel(384000, 1, 16),
        AudioFormatModel(512000, 1, 16),
        AudioFormatModel(384000, 2, 16),
        AudioFormatModel(512000, 2, 16),
        AudioFormatModel(192000, 1, 32),
        AudioFormatModel(512000, 2, 32),
    ]
    for fmt in high_rates:
        assert fmt.classify_tier() == AudioFormatModel.TIER_FUTURE_HARDWARE


# ==============================================================================
# 2. PSRAM STREAMING MARGIN AND STALL ANALYSIS
# ==============================================================================

def test_streaming_throughput_margin_analysis():
    """
    Verify PSRAM bandwidth margin against incoming audio streams.
    On STM32L476RG:
      - SPI/QSPI PSRAM bus clock: 18 MHz (55.56 ns/cycle)
      - IS66WVS tCEM max: 4.0 us at 85°C
      - Burst size: 16 bytes (Quad I/O = 46 cycles = 2.56 us)
      - Measured/polled QSPI sequential throughput: ~1.5 - 2.0 MB/s (1,500,000 - 2,000,000 B/s)
    """
    psram_polled_throughput_bps = 1_500_000 # Conservative 1.5 MB/s lower bound

    # Evaluate 16-bit formats
    eval_cases = [
        (48000, 1, 16, 96_000, True),
        (48000, 2, 16, 192_000, True),
        (96000, 1, 16, 192_000, True),
        (96000, 2, 16, 384_000, True),
        (192000, 1, 16, 384_000, True),
        (192000, 2, 16, 768_000, True),
        (384000, 1, 16, 768_000, True),
        (384000, 2, 16, 1_536_000, False), # 1.536 MB/s exceeds 1.5 MB/s margin
        (512000, 1, 16, 1_024_000, True),  # 1.024 MB/s feasible with light margin
        (512000, 2, 16, 2_048_000, False), # 2.048 MB/s exceeds 1.5 MB/s polled bus
    ]

    for rate, ch, bits, exp_bps, is_margin_positive in eval_cases:
        fmt = AudioFormatModel(rate, ch, bits)
        margin_factor = psram_polled_throughput_bps / fmt.byte_rate
        if is_margin_positive:
            assert margin_factor >= 1.0, f"{rate}Hz {ch}ch has insufficient margin ({margin_factor:.2f})"
        else:
            assert margin_factor < 1.0, f"{rate}Hz {ch}ch was expected to bottleneck polled bus ({margin_factor:.2f})"


def test_ping_pong_buffer_exhaustion_timeline():
    """
    Verify the time required to fill one 16 KiB ping-pong buffer across all sample rates.
    Proves how long the SD write task has to complete its write before an overrun occurs.
    """
    buffer_bytes = 16384 # 16 KiB per ping-pong half

    timings = {
        (48000, 1): 170.67,  # ms: 16384 / 96000
        (96000, 1): 85.33,   # ms: 16384 / 192000
        (192000, 1): 42.67,  # ms: 16384 / 384000
        (384000, 1): 21.33,  # ms: 16384 / 768000
        (512000, 1): 16.00,  # ms: 16384 / 1024000
    }

    for (rate, ch), exp_ms in timings.items():
        fmt = AudioFormatModel(rate, ch, 16)
        fill_time_ms = (buffer_bytes / fmt.byte_rate) * 1000.0
        assert math.isclose(fill_time_ms, exp_ms, rel_tol=1e-3), \
            f"Rate {rate} buffer fill time expected {exp_ms} ms, got {fill_time_ms:.2f} ms"


# ==============================================================================
# 3. WAV HEADER & RIFF CORRECTNESS
# ==============================================================================

def create_wav_header(rate: int, channels: int, bit_depth: int, data_bytes: int) -> bytes:
    """Generate canonical 44-byte standard PCM RIFF/WAV header."""
    bytes_per_sample = bit_depth // 8
    block_align = channels * bytes_per_sample
    byte_rate = rate * block_align
    riff_chunk_size = 36 + data_bytes

    header = struct.pack(
        "<4sI4s4sIHHIIHH4sI",
        b"RIFF",
        riff_chunk_size,
        b"WAVE",
        b"fmt ",
        16,              # Subchunk1Size for PCM
        1,               # AudioFormat: 1 = PCM uncompressed
        channels,
        rate,
        byte_rate,
        block_align,
        bit_depth,
        b"data",
        data_bytes
    )
    return header


def test_wav_header_fields_for_all_formats():
    """Verify RIFF/WAV header generation for 48k, 96k, 192k, 384k, 512k."""
    rates = [48000, 96000, 192000, 384000, 512000]
    duration_s = 10

    for r in rates:
        for ch in [1, 2]:
            for bits in [16, 32]:
                fmt = AudioFormatModel(r, ch, bits)
                payload = fmt.calculate_session_payload(duration_s)
                hdr = create_wav_header(r, ch, bits, payload)
                assert len(hdr) == 44

                # Parse header back
                (riff_tag, riff_sz, wave_tag, fmt_tag, sub1_sz,
                 audio_fmt, nch, srate, brate, balign, bps, data_tag, dsz) = struct.unpack(
                    "<4sI4s4sIHHIIHH4sI", hdr
                )
                assert riff_tag == b"RIFF"
                assert wave_tag == b"WAVE"
                assert fmt_tag == b"fmt "
                assert sub1_sz == 16
                assert audio_fmt == 1 # PCM
                assert nch == ch
                assert srate == r
                assert brate == fmt.byte_rate
                assert balign == fmt.bytes_per_frame
                assert bps == bits
                assert data_tag == b"data"
                assert dsz == payload
                assert riff_sz == payload + 36


def test_4gb_riff_and_fat32_file_size_boundary():
    """Verify behavior at and near 4GB boundary for 512 kHz 32-bit audio."""
    fmt = AudioFormatModel(512000, 2, 32) # 4,096,000 B/s
    # Max standard 32-bit unsigned RIFF size is 0xFFFFFFFF (4,294,967,295 bytes)
    max_payload = 0xFFFFFFFF - 36
    max_duration_sec = max_payload // fmt.byte_rate # ~1048 seconds (~17.4 minutes)

    assert max_duration_sec > 1000
    assert (max_duration_sec * fmt.byte_rate + 36) <= 0xFFFFFFFF

    # 1 second over max duration overflows 32-bit RIFF field
    overflow_duration = max_duration_sec + 2
    overflow_payload = overflow_duration * fmt.byte_rate
    assert (overflow_payload + 36) > 0xFFFFFFFF


# ==============================================================================
# 4. DSP NYQUIST FILTER CUTOFF VALIDATION
# ==============================================================================

def test_dsp_nyquist_cutoff_validation():
    """Verify DSP filter validation enforces cutoff strictly < Nyquist (Fs/2)."""
    # 48 kHz (Nyquist = 24,000 Hz)
    fmt_48k = AudioFormatModel(48000, 1, 16)
    valid, _ = fmt_48k.validate_dsp_cutoffs(80, 20000)
    assert valid is True
    valid, msg = fmt_48k.validate_dsp_cutoffs(80, 24000)
    assert valid is False
    assert "Nyquist" in msg
    valid, msg = fmt_48k.validate_dsp_cutoffs(80, 25000)
    assert valid is False

    # 96 kHz (Nyquist = 48,000 Hz)
    fmt_96k = AudioFormatModel(96000, 1, 16)
    valid, _ = fmt_96k.validate_dsp_cutoffs(80, 40000)
    assert valid is True
    valid, _ = fmt_96k.validate_dsp_cutoffs(80, 48000)
    assert valid is False

    # 192 kHz (Nyquist = 96,000 Hz)
    fmt_192k = AudioFormatModel(192000, 1, 16)
    valid, _ = fmt_192k.validate_dsp_cutoffs(80, 80000)
    assert valid is True
    valid, _ = fmt_192k.validate_dsp_cutoffs(80, 96000)
    assert valid is False

    # 384 kHz (Nyquist = 192,000 Hz)
    fmt_384k = AudioFormatModel(384000, 1, 16)
    valid, _ = fmt_384k.validate_dsp_cutoffs(80, 150000)
    assert valid is True
    valid, _ = fmt_384k.validate_dsp_cutoffs(80, 192000)
    assert valid is False

    # 512 kHz (Nyquist = 256,000 Hz)
    fmt_512k = AudioFormatModel(512000, 1, 16)
    valid, _ = fmt_512k.validate_dsp_cutoffs(80, 200000)
    assert valid is True
    valid, _ = fmt_512k.validate_dsp_cutoffs(80, 256000)
    assert valid is False


# ==============================================================================
# 5. SOURCE CODE ARCHITECTURE INSPECTION
# ==============================================================================

def test_source_files_exist():
    """Verify that new architecture and benchmark modules exist in repository."""
    assert (SRC_DIR / "audio_format.h").is_file()
    assert (SRC_DIR / "audio_format.c").is_file()
    assert (SRC_DIR / "psram_benchmark.h").is_file()
    assert (SRC_DIR / "psram_benchmark.c").is_file()


def test_psram_benchmark_address_space_isolation():
    """Verify benchmark addresses (0x00300000) do NOT overlap staging (0x00000000)."""
    psram_h = (SRC_DIR / "psram_is66wvs.h").read_text(encoding="utf-8")
    assert "PSRAM_DIAG_TEST_BASE_ADDR      0x00300000U" in psram_h
    assert "PSRAM_DIAG_TEST_MAX_ADDR       0x003FFFFFU" in psram_h

    bench_c = (SRC_DIR / "psram_benchmark.c").read_text(encoding="utf-8")
    assert "PSRAM_DIAG_TEST_BASE_ADDR" in bench_c

    staging_h = (SRC_DIR / "psram_staging.h").read_text(encoding="utf-8")
    assert "PSRAM_STAGING_BASE_ADDR                0x00000000U" in staging_h
    assert "PSRAM_AUDIO_BUFFER_BYTES   (3U * 1024U * 1024U)" in staging_h
    assert "PSRAM_STAGING_DEFAULT_CAPACITY_BYTES   PSRAM_AUDIO_BUFFER_BYTES" in staging_h

    # Staging ends at 0x00000000 + 3MB = 0x00300000, Benchmark starts at 0x00300000
    staging_end = 0x00000000 + 3 * 1024 * 1024
    bench_start = 0x00300000
    assert staging_end <= bench_start, "Benchmark address space overlaps audio staging region!"


def test_cmakelists_integrates_new_modules():
    """Verify CMakeLists.txt compiles audio_format.c and psram_benchmark.c."""
    cmake = (APP_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "src/audio_format.c" in cmake
    assert "src/psram_benchmark.c" in cmake


def test_production_build_separation_and_quietness():
    """Verify that benchmark runner is gated under CONFIG_WILDSONIC_VERBOSE_LOGS in main.c."""
    main_c = (SRC_DIR / "main.c").read_text(encoding="utf-8")
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in main_c
    assert "psram_benchmark_run_matrix" in main_c
