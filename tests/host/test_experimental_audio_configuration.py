"""Host unit tests for TEST-ONLY Experimental Audio Configuration.

Verifies:
- Phase 1: Test-only console configuration support (SAMF 48k/96k/192k/384k/512k, BITS 16/32, CHANNELS 1/2, SOURCE)
  with explicit warning banners and quiet production exclusion.
- Phase 2: Requested vs actual hardware rates, TIM6 divider/auto-reload math at 80 MHz, WAV header rate.
- Phase 3: Adaptive ADC hardware oversampling (8x for <=192k, 2x for 384k, 1x/off for 512k) and synthetic source.
- Phase 4: 32-bit PCM container scaling (int16_t << 16), RIFF WAV fields, and 4-byte payload accounting.
- Phase 5: Stereo format emulation (mono duplication, synthetic L=1kHz/R=2kHz dual-tone, 2-channel WAV header).
- Phase 6: Pipeline protection, exact byte formula, overflow abort (-ENOBUFS), skip WAV finalization on error.
- Phase 7: Production build quietness and strict hardware validation preservation.
"""

import math
import struct
from pathlib import Path
import pytest

APP_DIR = Path(__file__).resolve().parent.parent.parent / "app"
APP_SRC = APP_DIR / "src"

# ==============================================================================
# Helper math & reference definitions
# ==============================================================================

APB1_CLOCK_HZ = 80_000_000
WAV_HEADER_BYTES = 512


def calc_tim6_rate(requested_hz: int) -> tuple[int, int, int]:
    """Calculate (divider, ARR, actual_hz) for TIM6 TRGO trigger at 80 MHz APB1."""
    div = (APB1_CLOCK_HZ + requested_hz // 2) // requested_hz
    arr = div - 1
    actual_hz = APB1_CLOCK_HZ // div
    return div, arr, actual_hz


def calc_adc_timing_budget(rate_hz: int, ovs_log2: int) -> tuple[float, float, float]:
    """Calculate conversion time (us), trigger period (us), and duty cycle percentage."""
    cycles_per_conv = 37
    conv_cycles = (1 << ovs_log2) * cycles_per_conv + 4
    conv_us = conv_cycles / 80.0
    _, _, actual_hz = calc_tim6_rate(rate_hz)
    period_us = 1_000_000.0 / actual_hz
    duty_pct = (conv_us / period_us) * 100.0
    return conv_us, period_us, duty_pct


def build_wav_header(sample_rate: int, channels: int, bits_per_sample: int, data_bytes: int) -> bytes:
    """Build canonical 512-byte sector-aligned WAV header."""
    block_align = channels * (bits_per_sample // 8)
    byte_rate = sample_rate * block_align
    junk_len = WAV_HEADER_BYTES - 12 - 24 - 8 - 8  # 460 bytes

    hdr = bytearray(WAV_HEADER_BYTES)
    # RIFF chunk
    hdr[0:4] = b"RIFF"
    struct.pack_into("<I", hdr, 4, (WAV_HEADER_BYTES - 8) + data_bytes)
    hdr[8:12] = b"WAVE"
    # fmt chunk
    hdr[12:16] = b"fmt "
    struct.pack_into("<I", hdr, 16, 16)
    struct.pack_into("<H", hdr, 20, 1)  # PCM format
    struct.pack_into("<H", hdr, 22, channels)
    struct.pack_into("<I", hdr, 24, sample_rate)
    struct.pack_into("<I", hdr, 28, byte_rate)
    struct.pack_into("<H", hdr, 32, block_align)
    struct.pack_into("<H", hdr, 34, bits_per_sample)
    # JUNK chunk padding
    hdr[36:40] = b"JUNK"
    struct.pack_into("<I", hdr, 40, junk_len)
    # data chunk header
    data_hdr_off = 36 + 8 + junk_len
    hdr[data_hdr_off:data_hdr_off + 4] = b"data"
    struct.pack_into("<I", hdr, data_hdr_off + 4, data_bytes)
    return bytes(hdr)


def verify_wav_header(hdr: bytes, exp_rate: int, exp_channels: int, exp_bits: int, exp_data_bytes: int) -> bool:
    """Verify WAV header fields against expected format parameters."""
    if len(hdr) != WAV_HEADER_BYTES:
        return False
    if hdr[0:4] != b"RIFF" or hdr[8:12] != b"WAVE" or hdr[12:16] != b"fmt ":
        return False
    riff_len = struct.unpack_from("<I", hdr, 4)[0]
    if riff_len != (WAV_HEADER_BYTES - 8) + exp_data_bytes:
        return False
    channels = struct.unpack_from("<H", hdr, 22)[0]
    rate = struct.unpack_from("<I", hdr, 24)[0]
    byte_rate = struct.unpack_from("<I", hdr, 28)[0]
    block_align = struct.unpack_from("<H", hdr, 32)[0]
    bits = struct.unpack_from("<H", hdr, 34)[0]
    exp_align = exp_channels * (exp_bits // 8)
    exp_byte_rate = exp_rate * exp_align
    if channels != exp_channels or rate != exp_rate or bits != exp_bits:
        return False
    if block_align != exp_align or byte_rate != exp_byte_rate:
        return False
    data_off = WAV_HEADER_BYTES - 8
    if hdr[data_off:data_off + 4] != b"data":
        return False
    data_bytes = struct.unpack_from("<I", hdr, data_off + 4)[0]
    return data_bytes == exp_data_bytes


# ==============================================================================
# Phase 1 & 2: Console Configuration, Aliases, Timer Rate Arithmetic
# ==============================================================================

def test_tim6_rate_arithmetic_all_rates():
    """Verify TIM6 integer divider and auto-reload math for standard and experimental rates."""
    # 48 kHz
    div_48k, arr_48k, act_48k = calc_tim6_rate(48000)
    assert div_48k == 1667
    assert arr_48k == 1666
    assert act_48k == 47990
    assert abs(act_48k - 48000) <= 10

    # 96 kHz
    div_96k, arr_96k, act_96k = calc_tim6_rate(96000)
    assert div_96k == 833
    assert arr_96k == 832
    assert act_96k == 96038
    assert abs(act_96k - 96000) <= 40

    # 192 kHz
    div_192k, arr_192k, act_192k = calc_tim6_rate(192000)
    assert div_192k == 417
    assert arr_192k == 416
    assert act_192k == 191846
    assert abs(act_192k - 192000) <= 160

    # 384 kHz (experimental)
    div_384k, arr_384k, act_384k = calc_tim6_rate(384000)
    assert div_384k == 208
    assert arr_384k == 207
    assert act_384k == 384615
    assert (act_384k - 384000) == 615

    # 512 kHz (experimental)
    div_512k, arr_512k, act_512k = calc_tim6_rate(512000)
    assert div_512k == 156
    assert arr_512k == 155
    assert act_512k == 512820
    assert (act_512k - 512000) == 820


def test_console_source_command_handlers_and_aliases():
    """Inspect console.c to verify experimental commands, aliases, and warning banners."""
    src = (APP_SRC / "console.c").read_text(encoding="utf-8")

    # Command aliases in console_process_line
    assert 'strncasecmp(p, "BITS", 4)' in src
    assert 'strncasecmp(p, "CHANNELS", 8)' in src
    assert 'strncasecmp(p, "SOURCE", 6)' in src

    # Capture source handler
    assert "handle_source(" in src
    assert "AUDIO_SRC_PHYSICAL_MIC" in src
    assert "AUDIO_SRC_STEREO_DUPLICATE" in src
    assert "AUDIO_SRC_SYNTHETIC" in src

    # Warning banners
    assert "WARNING: Experimental audio sampling rate configured" in src
    assert "WARNING: Experimental 32-bit PCM storage format configured" in src
    assert "WARNING: Experimental stereo channel emulation configured" in src
    assert "Target production platform: STM32U585" in src


def test_production_build_excludes_experimental_settings():
    """Verify that under PRODUCTION (CONFIG_WILDSONIC_VERBOSE_LOGS not defined), experimental features are gated."""
    src_console = (APP_SRC / "console.c").read_text(encoding="utf-8")
    src_audio_in = (APP_SRC / "audio_in_stm32l4.c").read_text(encoding="utf-8")

    # Console handlers must gate experimental paths under CONFIG_WILDSONIC_VERBOSE_LOGS
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in src_console
    assert "[DEFERRED] Bits per sample 24" in src_console or "[DEFERRED] Bits per sample" in src_console

    # Production supports 192k 32-bit mono and stereo (duplicated mono container)
    assert "g_cfg.num_audio_channels = 2" in src_console
    assert "g_cfg.audio_sampling_width = 32" in src_console

    # audio_in_is_rate_supported must restrict 384k and 512k in production
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in src_audio_in
    assert "rate_hz != 384000 && rate_hz != 512000" in src_audio_in


# ==============================================================================
# Phase 3: Adaptive Hardware Oversampling Timing Budget & Synthetic Generator
# ==============================================================================

def test_adc_adaptive_oversampling_budget_at_80mhz():
    """Verify adaptive oversampling selection and compliance with < 85% trigger period constraint."""
    # 48k: 8x oversampling (OVS_LOG2 = 3)
    conv_us, per_us, duty_pct = calc_adc_timing_budget(48000, 3)
    assert conv_us == pytest.approx(3.75, abs=0.01)
    assert duty_pct < 20.0

    # 192k: 8x oversampling (OVS_LOG2 = 3)
    conv_us, per_us, duty_pct = calc_adc_timing_budget(192000, 3)
    assert conv_us == pytest.approx(3.75, abs=0.01)
    assert per_us == pytest.approx(5.2125, abs=0.01)
    assert duty_pct < 73.0  # Safe < 85%

    # 384k: 8x oversampling would exceed trigger period (3.75 us > 2.60 us)
    _, per_384, _ = calc_adc_timing_budget(384000, 3)
    assert per_384 == pytest.approx(2.60, abs=0.01)
    # Adaptive oversampling to 2x (OVS_LOG2 = 1)
    conv_384_2x, per_384, duty_384 = calc_adc_timing_budget(384000, 1)
    assert conv_384_2x == pytest.approx(0.975, abs=0.01)
    assert duty_384 == pytest.approx(37.5, abs=0.5)
    assert duty_384 < 85.0  # Fully safe within budget!

    # 512k: 1x oversampling / off (OVS_LOG2 = 0)
    conv_512_1x, per_512, duty_512 = calc_adc_timing_budget(512000, 0)
    assert conv_512_1x == pytest.approx(0.5125, abs=0.01)
    assert per_512 == pytest.approx(1.95, abs=0.01)
    assert duty_512 == pytest.approx(26.28, abs=0.5)
    assert duty_512 < 85.0  # Fully safe within budget!


def test_synthetic_sample_generator_purity():
    """Verify synthetic 1 kHz sine tone generation and scaling."""
    rate = 192000
    freq = 1000.0
    samples = []
    for n in range(rate // 100):  # 10 ms = 10 full cycles
        phase = 2.0 * math.pi * freq * (n / rate)
        s16 = int(math.sin(phase) * 16384.0)
        samples.append(s16)

    assert min(samples) >= -16384
    assert max(samples) <= 16384
    # Zero crossing check
    assert abs(samples[0]) <= 1
    # Quarter cycle peak (sample index = 192000 / 4000 = 48)
    assert samples[48] == pytest.approx(16384, abs=5)


# ==============================================================================
# Phase 4: 32-bit PCM Container Scaling and WAV Header
# ==============================================================================

def test_32bit_pcm_container_scaling():
    """Verify explicit (int16_t << 16) container scaling to 32-bit signed PCM."""
    # Test values across full int16 dynamic range
    test_vals = [-32768, -16384, -1, 0, 1, 16384, 32767]
    for s16 in test_vals:
        s32 = s16 << 16
        # Verify 32-bit signed integer bounds
        assert -2147483648 <= s32 <= 2147483647
        # Verify upper 16 bits match s16 exactly
        recovered_s16 = s32 >> 16
        assert recovered_s16 == s16
        # Verify lower 16 bits are cleanly zeroed
        assert (s32 & 0xFFFF) == 0


def test_32bit_mono_wav_header_and_payload_accounting():
    """Verify 32-bit mono WAV header creation, byte rate, block align, and verification."""
    duration_s = 10
    rate = 192000
    channels = 1
    bits = 32
    samples = duration_s * rate
    bytes_per_sample = bits // 8
    payload_bytes = samples * channels * bytes_per_sample  # 1,920,000 * 4 = 7,680,000 bytes

    hdr = build_wav_header(rate, channels, bits, payload_bytes)
    assert verify_wav_header(hdr, rate, channels, bits, payload_bytes) is True
    # Verify exact fields in header
    block_align = struct.unpack_from("<H", hdr, 32)[0]
    byte_rate = struct.unpack_from("<I", hdr, 28)[0]
    assert block_align == 4
    assert byte_rate == rate * 4
    assert len(hdr) == 512


# ==============================================================================
# Phase 5: Stereo Format Emulation & Dual-Tone Synthetic Generator
# ==============================================================================

def test_stereo_mono_duplication():
    """Verify mono sample duplication across Left and Right channels."""
    mono_block = [100, -200, 300, -400, 500]
    stereo_block = []
    for s in mono_block:
        stereo_block.append(s)  # Left
        stereo_block.append(s)  # Right

    assert len(stereo_block) == len(mono_block) * 2
    for i, s in enumerate(mono_block):
        assert stereo_block[2 * i] == s
        assert stereo_block[2 * i + 1] == s


def test_stereo_synthetic_dual_tone_separation():
    """Verify synthetic Left=1kHz, Right=2kHz dual-tone generates distinct orthogonal signals."""
    rate = 192000
    frames = 1920  # 10 ms
    left_ch = []
    right_ch = []

    for n in range(frames):
        p1 = 2.0 * math.pi * 1000.0 * (n / rate)
        p2 = 2.0 * math.pi * 2000.0 * (n / rate)
        left_ch.append(int(math.sin(p1) * 16384.0))
        right_ch.append(int(math.sin(p2) * 16384.0))

    assert len(left_ch) == frames
    assert len(right_ch) == frames
    # Signals must be distinct
    assert left_ch != right_ch
    # Right channel has twice the frequency of Left channel
    # At quarter cycle of 1 kHz (n=48): Left is peak, Right (2 kHz) is at half cycle (zero)
    assert left_ch[48] == pytest.approx(16384, abs=5)
    assert right_ch[48] == pytest.approx(0, abs=5)


def test_stereo_16bit_and_32bit_wav_headers():
    """Verify 16-bit and 32-bit stereo WAV headers."""
    rate = 192000
    channels = 2

    # 16-bit stereo: 10s -> 1,920,000 frames * 4 bytes = 7,680,000 bytes
    payload_16 = 10 * rate * channels * 2
    hdr_16 = build_wav_header(rate, channels, 16, payload_16)
    assert verify_wav_header(hdr_16, rate, channels, 16, payload_16) is True
    assert struct.unpack_from("<H", hdr_16, 32)[0] == 4  # BlockAlign = 2 * 2 = 4
    assert struct.unpack_from("<I", hdr_16, 28)[0] == rate * 4  # ByteRate = rate * 4

    # 32-bit stereo: 10s -> 1,920,000 frames * 8 bytes = 15,360,000 bytes
    payload_32 = 10 * rate * channels * 4
    hdr_32 = build_wav_header(rate, channels, 32, payload_32)
    assert verify_wav_header(hdr_32, rate, channels, 32, payload_32) is True
    assert struct.unpack_from("<H", hdr_32, 32)[0] == 8  # BlockAlign = 2 * 4 = 8
    assert struct.unpack_from("<I", hdr_32, 28)[0] == rate * 8  # ByteRate = rate * 8


# ==============================================================================
# Phase 6: Pipeline Protection, Exact Accounting, Failure Recovery
# ==============================================================================

def test_high_rate_experimental_accounting_matrix():
    """Verify exact sample and byte calculations for all experimental rate combinations."""
    combos = [
        # (rate, channels, bits, duration_s, expected_samples, expected_bytes)
        (48000, 1, 16, 10, 480000, 960000),
        (96000, 1, 16, 10, 960000, 1920000),
        (192000, 1, 16, 10, 1920000, 3840000),
        (384615, 1, 16, 10, 3846150, 7692300),   # 384k TIM6 actual
        (512820, 1, 16, 10, 5128200, 10256400),  # 512k TIM6 actual
        (192000, 1, 32, 10, 1920000, 7680000),   # 192k 32-bit mono
        (192000, 2, 16, 10, 1920000, 7680000),   # 192k 16-bit stereo
        (192000, 2, 32, 10, 1920000, 15360000),  # 192k 32-bit stereo
        (384615, 2, 32, 10, 3846150, 30769200),  # 384k 32-bit stereo
        (512820, 2, 32, 10, 5128200, 41025600),  # 512k 32-bit stereo
    ]

    for rate, ch, bits, dur, exp_samples, exp_bytes in combos:
        bytes_per_frame = ch * (bits // 8)
        calc_samples = dur * rate
        calc_bytes = calc_samples * bytes_per_frame
        assert calc_samples == exp_samples
        assert calc_bytes == exp_bytes


def test_pipeline_failure_preservation_and_skip_wav_finalization():
    """Verify that on PSRAM full (-ENOBUFS) or SD write failure, recording aborts cleanly and skips WAV finalization."""
    src_rec = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    # Error handling and abort
    assert "err = -ENOBUFS;" in src_rec
    assert "LOG_WRN(\"PSRAM staging buffer full!" in src_rec

    # Must skip WAV finalization if error occurred
    assert "if (err == 0 && sd_writer_err == 0)" in src_rec
    assert "LOG_WRN(\"Skipping WAV header finalization due to previous storage error (%d)\"" in src_rec

    # File verification must be skipped if error occurred
    assert "if (err == 0 && !recorder_abort_requested && st.bytes_written > 0)" in src_rec
    assert "st.wav_verified = false;" in src_rec


# ==============================================================================
# Phase 7: Source Code Architecture & Parity Inspections
# ==============================================================================

def test_source_code_architecture_inspections():
    """Verify all contracts, function signatures, and data model fields in source code."""
    src_hdr = (APP_SRC / "audio_in.h").read_text(encoding="utf-8")
    src_fmt = (APP_SRC / "audio_format.h").read_text(encoding="utf-8")
    src_main_h = (APP_SRC / "main.h").read_text(encoding="utf-8")
    src_rec = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    # audio_in.h declarations
    assert "audio_in_requested_rate" in src_hdr
    assert "audio_in_set_capture_source" in src_hdr
    assert "audio_in_get_capture_source" in src_hdr
    assert "audio_in_get_ovs_ratio" in src_hdr

    # audio_format.h declarations
    assert "audio_format_is_experimental" in src_fmt

    # main.h settings struct
    assert "uint16_t sample_bits;" in src_main_h
    assert "uint16_t channels;" in src_main_h
    assert "enum audio_capture_source capture_source;" in src_main_h

    # recorder.c verification function
    assert "recorder_verify_file_format(" in src_rec
    assert "audio_format_is_experimental(st.sample_rate, st.channels, st.sample_bits)" in src_rec
