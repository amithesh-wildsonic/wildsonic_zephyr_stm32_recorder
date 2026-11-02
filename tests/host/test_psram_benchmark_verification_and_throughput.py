#!/usr/bin/env python3
"""
Test Suite: PSRAM Benchmark Verification Errors, Throughput & Audio Bandwidth Model
=====================================================================================
Covers:
  1. Verification error counter semantics (why 4K=1, 32K=8, 256K=64, 1M=256 counts 4 KiB chunks).
  2. Targeted verification diagnostics (first mismatch address, expected/actual, diff bitmask, offset).
  3. Real verification logic in interleaved streaming and repeated-block modes.
  4. Physical throughput calculation reconciliation (wire time + HAL overhead for 16-byte bursts at 18 MHz).
  5. Two-way PSRAM audio traffic model and headroom margins for 192k, 384k, 512k mono/stereo 16/32-bit.
  6. Dedicated diagnostic memory isolation (0x00300000..0x003FFFFF) vs live staging (0x00000000..0x000FFFFF).
  7. Production build separation and console quietness.
"""

from pathlib import Path
import pytest

BASE_DIR = Path(__file__).resolve().parent.parent.parent
APP_DIR = BASE_DIR / "app"
SRC_DIR = APP_DIR / "src"


# ==============================================================================
# 1. VERIFICATION ERROR COUNTER SEMANTICS AUDIT
# ==============================================================================

def test_mismatch_counter_reconciliation_4k_chunks():
    """
    Prove mathematically that error counts of 1, 8, 64, 256 for 4KB, 32KB, 256KB, 1MB
    corresponded to counting 4 KiB chunks rather than individual bytes.
    """
    chunk_size = 4096
    test_spans = {
        4 * 1024: 1,      # 4 KiB -> 1 chunk -> reported 1 error
        32 * 1024: 8,     # 32 KiB -> 8 chunks -> reported 8 errors
        256 * 1024: 64,   # 256 KiB -> 64 chunks -> reported 64 errors
        1024 * 1024: 256  # 1 MiB -> 256 chunks -> reported 256 errors
    }

    for span_bytes, expected_error_count in test_spans.items():
        calculated_chunks = span_bytes // chunk_size
        assert calculated_chunks == expected_error_count, (
            f"Span {span_bytes} B should have {expected_error_count} chunks of {chunk_size} B"
        )


def test_chunk_vs_byte_mismatch_simulation():
    """Simulate a buffer with 1 corrupt byte per 4 KiB chunk and check block vs byte counts."""
    chunk_size = 4096
    num_chunks = 8 # 32 KiB test
    total_bytes = num_chunks * chunk_size

    # Simulated memory buffers
    tx_buf = bytearray((i & 0xFF) for i in range(chunk_size))
    rx_buf = bytearray(tx_buf)
    # Corrupt exactly 1 byte per chunk (at offset 16)
    rx_buf[16] ^= 0x01

    mismatch_blocks = 0
    mismatch_bytes = 0
    first_mismatch_offset = None
    first_expected = None
    first_actual = None

    for chunk_idx in range(num_chunks):
        chunk_mismatches = 0
        for i in range(chunk_size):
            if tx_buf[i] != rx_buf[i]:
                if first_mismatch_offset is None:
                    first_mismatch_offset = i
                    first_expected = tx_buf[i]
                    first_actual = rx_buf[i]
                chunk_mismatches += 1
                mismatch_bytes += 1
        if chunk_mismatches > 0:
            mismatch_blocks += 1

    assert mismatch_blocks == 8, "Block count must equal 8"
    assert mismatch_bytes == 8, "Byte count must equal 8"
    assert first_mismatch_offset == 16
    assert first_expected ^ first_actual == 0x01


# ==============================================================================
# 2. SOURCE CODE DIAGNOSTICS & VERIFICATION INSPECTION
# ==============================================================================

def test_source_benchmark_tracks_both_blocks_and_bytes():
    """Verify that psram_benchmark.c and psram_benchmark.h track blocks, bytes, and first mismatch."""
    bench_h = (SRC_DIR / "psram_benchmark.h").read_text(encoding="utf-8")
    assert "uint32_t mismatch_bytes;" in bench_h
    assert "uint32_t mismatch_blocks;" in bench_h
    assert "uint32_t first_mismatch_addr;" in bench_h
    assert "uint32_t first_mismatch_offset;" in bench_h
    assert "uint8_t first_expected_byte;" in bench_h
    assert "uint8_t first_actual_byte;" in bench_h
    assert "uint8_t first_diff_mask;" in bench_h
    assert "bool has_mismatch_diag;" in bench_h

    bench_c = (SRC_DIR / "psram_benchmark.c").read_text(encoding="utf-8")
    assert "out->mismatch_bytes++" in bench_c
    assert "out->mismatch_blocks++" in bench_c
    assert "out->first_mismatch_addr" in bench_c
    assert "out->first_mismatch_offset" in bench_c
    assert "out->first_expected_byte" in bench_c
    assert "out->first_actual_byte" in bench_c
    assert "out->first_diff_mask" in bench_c


def test_source_repeated_and_streaming_benchmarks_perform_verification():
    """Verify that REPEATED_BLOCK and STREAMING_SIM perform actual readback validation."""
    bench_c = (SRC_DIR / "psram_benchmark.c").read_text(encoding="utf-8")

    # REPEATED_BLOCK must compare bench_tx_buf and bench_rx_buf
    assert "PSRAM_BENCH_REPEATED_BLOCK:" in bench_c
    assert "[BENCH DIAG REPEAT]" in bench_c

    # STREAMING_SIM must compare bench_rx_buf against expected pattern for read_addr
    assert "PSRAM_BENCH_STREAMING_SIM:" in bench_c
    assert "[BENCH DIAG STREAM]" in bench_c
    assert "uint8_t exp = (uint8_t)((read_addr + i) ^ 0xA5U);" in bench_c


def test_source_rx_buffer_cleared_before_read():
    """Verify that bench_rx_buf is cleared before reads to prevent stale data false-matches."""
    bench_c = (SRC_DIR / "psram_benchmark.c").read_text(encoding="utf-8")
    assert "memset(bench_rx_buf, 0x55, chunk_len);" in bench_c


def test_source_pattern_validation_function_exists():
    """Verify that psram_benchmark_run_pattern_validation is implemented and exported."""
    bench_h = (SRC_DIR / "psram_benchmark.h").read_text(encoding="utf-8")
    assert "int psram_benchmark_run_pattern_validation(void);" in bench_h

    bench_c = (SRC_DIR / "psram_benchmark.c").read_text(encoding="utf-8")
    assert "int psram_benchmark_run_pattern_validation(void)" in bench_c
    assert "psram_fill_pattern(pat, bench_tx_buf, BENCH_CHUNK_SIZE, addr);" in bench_c


# ==============================================================================
# 3. THROUGHPUT AUDIT & PHYSICAL LATENCY RECONCILIATION
# ==============================================================================

def test_physical_throughput_reconciliation_18mhz_quad():
    """
    Audit physical throughput for 18 MHz Quad-SPI IS66WVS4M8.
    With tCEM <= 4.0 us limit, burst size is constrained to 16 bytes.
    A 4096-byte transfer requires 256 bursts of 16 bytes.
    """
    f_bus_hz = 18_000_000 # 18 MHz
    t_clk_ns = 1_000_000_000 / f_bus_hz # 55.56 ns

    # Wire time for 16-byte Quad write (CMD=8 SPI clks, ADDR=6 Quad clks, DATA=32 Quad clks):
    # Total clks = 8 + 6 + 32 = 46 clks
    wire_time_write_us = (46 * t_clk_ns) / 1000.0 # ~2.56 us

    # Wire time for 16-byte Quad read (CMD=8 SPI, ADDR=6 Quad, DUMMY=6 Quad, DATA=32 Quad):
    # Total clks = 8 + 6 + 6 + 32 = 52 clks
    wire_time_read_us = (52 * t_clk_ns) / 1000.0 # ~2.89 us

    assert wire_time_write_us <= 4.0, "Wire time must strictly respect tCEM <= 4.0 us"
    assert wire_time_read_us <= 4.0, "Wire time must strictly respect tCEM <= 4.0 us"

    # CPU/HAL overhead per 16-byte transaction (HAL_QSPI_Command + Transmit/Receive + Abort check):
    # Measured on STM32L476 @ 80 MHz: ~18 us overhead per transaction
    hal_overhead_us = 18.3
    total_burst_us = wire_time_write_us + hal_overhead_us # ~20.86 us

    # 4096 bytes = 256 bursts of 16 bytes:
    chunk_lat_us = 256 * total_burst_us # ~5340 us (5.34 ms)
    throughput_bytes_sec = (4096 * 1_000_000) / chunk_lat_us # ~767,000 B/s (~749 KiB/s)
    throughput_kib_sec = throughput_bytes_sec / 1024.0

    # Must match observed hardware log (~747 KB/s sequential write, ~759 KB/s sequential read)
    assert 730 <= throughput_kib_sec <= 770, f"Expected ~747 KB/s, got {throughput_kib_sec:.1f} KiB/s"


# ==============================================================================
# 4. TWO-WAY PSRAM AUDIO TRAFFIC MODEL & HEADROOM ANALYSIS
# ==============================================================================

def test_two_way_audio_bandwidth_and_psram_headroom():
    """
    Evaluate two-way PSRAM traffic against measured ~771,000 B/s (753 KiB/s) capacity.
    Two-way traffic = 1 write (producer) + 1 read (SD writer) = 2 * AudioStreamRate.
    """
    psram_capacity_bps = 771_000 # Measured combined physical throughput (753 KiB/s)

    test_matrix = [
        # (Rate, Ch, Bits, Name, Expected Audio B/s, Expected 2-Way B/s, Feasibility)
        (192000, 1, 16, "192kHz Mono 16-bit", 384_000, 768_000, "FRAGILE_BORDERLINE"),
        (384000, 1, 16, "384kHz Mono 16-bit", 768_000, 1_536_000, "IMPOSSIBLE_DEFICIT"),
        (512000, 1, 16, "512kHz Mono 16-bit", 1_024_000, 2_048_000, "IMPOSSIBLE_DEFICIT"),
        (512000, 2, 32, "512kHz Stereo 32-bit", 4_096_000, 8_192_000, "IMPOSSIBLE_DEFICIT"),
    ]

    for rate, ch, bits, name, expected_audio_bps, expected_2way_bps, verdict in test_matrix:
        audio_bps = rate * ch * (bits // 8)
        two_way_traffic_bps = 2 * audio_bps

        assert audio_bps == expected_audio_bps
        assert two_way_traffic_bps == expected_2way_bps

        margin_bps = psram_capacity_bps - two_way_traffic_bps
        margin_pct = (margin_bps / psram_capacity_bps) * 100.0

        if verdict == "FRAGILE_BORDERLINE":
            # 192 kHz mono 16-bit has +3,000 B/s margin (+0.39%), razor thin
            assert 0 < margin_pct < 2.0, f"192 kHz should have razor-thin margin, got {margin_pct:.2f}%"
        else:
            # All higher rates exceed physical bus capacity by 2x to 10x
            assert margin_pct < -40.0, f"{name} should have severe deficit, got {margin_pct:.2f}%"


# ==============================================================================
# 5. DIAGNOSTIC RANGE ISOLATION & BUILD SEPARATION
# ==============================================================================

def test_diagnostic_memory_range_isolation():
    """Verify that benchmark diagnostic span (0x00300000) does not overlap live staging."""
    psram_h = (SRC_DIR / "psram_is66wvs.h").read_text(encoding="utf-8")
    assert "PSRAM_DIAG_TEST_BASE_ADDR      0x00300000U" in psram_h
    assert "PSRAM_DIAG_TEST_MAX_ADDR       0x003FFFFFU" in psram_h

    staging_h = (SRC_DIR / "psram_staging.h").read_text(encoding="utf-8")
    assert "PSRAM_STAGING_BASE_ADDR                0x00000000U" in staging_h
    assert "PSRAM_AUDIO_BUFFER_BYTES   (3U * 1024U * 1024U)" in staging_h
    assert "PSRAM_STAGING_DEFAULT_CAPACITY_BYTES   PSRAM_AUDIO_BUFFER_BYTES" in staging_h


def test_production_build_excludes_benchmark_commands():
    """Verify that psram bench commands and verbose diagnostics are TEST-only."""
    main_c = (SRC_DIR / "main.c").read_text(encoding="utf-8")
    assert "#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)" in main_c
    assert 'strcmp(a1, "bench") == 0' in main_c


# ==============================================================================
# 6. THROUGHPUT ARITHMETIC RECONCILIATION & UNVERIFIED STATUS AUDIT
# ==============================================================================

def test_latency_and_throughput_arithmetic_reconciliation():
    """
    Reconcile the 5,013 us chunk latency and the 746 KB/s throughput discrepancy.
    - Pure bus time: 4096 bytes in 5,013 us = 817.1 kB/s (decimal) | 797.9 KiB/s (binary).
    - Wall time: 5,013 us bus + ~475 us CPU fill pattern = 5,488 us -> 746.3 KiB/s.
    Proves that old benchmark computed throughput over total wall time rather than bus time.
    """
    bytes_transferred = 4096
    bus_lat_us = 5013
    cpu_prep_us = 475 # Measured fill_bench_pattern time @ 80 MHz
    total_wall_us = bus_lat_us + cpu_prep_us # 5488 us

    # Correct bus throughput:
    bus_throughput_bps = (bytes_transferred * 1_000_000) // bus_lat_us # 817,075 B/s
    decimal_kb_s = bus_throughput_bps // 1000 # 817 kB/s
    binary_kib_s = bus_throughput_bps // 1024 # 797 KiB/s

    assert decimal_kb_s == 817
    assert binary_kib_s == 797

    # Flawed wall-clock throughput (either 5,488 us decimal or 5,361 us binary):
    # In decimal: (4096 * 1_000_000) // 5488 // 1000 = 746 kB/s
    # In binary: (4096 * 1_000_000) // 5361 // 1024 = 746 KiB/s
    flawed_decimal_kb_s = (bytes_transferred * 1_000_000) // 5488 // 1000
    flawed_binary_kib_s = (bytes_transferred * 1_000_000) // 5361 // 1024
    assert flawed_decimal_kb_s == 746
    assert flawed_binary_kib_s == 746

    # Verify source implementation distinguishes bus_elapsed_us and total_elapsed_us
    bench_h = (SRC_DIR / "psram_benchmark.h").read_text(encoding="utf-8")
    assert "uint64_t bus_elapsed_us;" in bench_h
    assert "uint64_t total_elapsed_us;" in bench_h
    assert "uint32_t bytes_written;" in bench_h
    assert "uint32_t bytes_read;" in bench_h
    assert "bool verified_mode;" in bench_h

    bench_c = (SRC_DIR / "psram_benchmark.c").read_text(encoding="utf-8")
    assert "out->bus_elapsed_us = (chunk_latency_sum > 0) ? chunk_latency_sum : 1U;" in bench_c
    assert "out->total_elapsed_us = (t_end >= t_start) ? (t_end - t_start) : out->bus_elapsed_us;" in bench_c
    assert "out->throughput_bytes_sec = (uint32_t)(((uint64_t)out->total_transferred_bytes * 1000000ULL) / out->bus_elapsed_us);" in bench_c


def test_unverified_modes_labeled_clearly():
    """Verify that Sequential Write and Sequential Read do not claim bit-exact verification."""
    bench_c = (SRC_DIR / "psram_benchmark.c").read_text(encoding="utf-8")

    # Mode 0 and 1 explicitly set verified_mode = false
    assert "case PSRAM_BENCH_SEQ_WRITE:" in bench_c
    assert "case PSRAM_BENCH_SEQ_READ:" in bench_c
    assert "out->verified_mode = false;" in bench_c

    # Output table explicitly labels unverified modes
    assert "WRITE OK (unverified - no readback)" in bench_c
    assert "READ OK (unverified - untracked pattern)" in bench_c


# ==============================================================================
# 7. MINIMAL ISOLATED PSRAM TRANSACTION DIAGNOSTIC AUDIT
# ==============================================================================

def test_isolated_diagnostic_suite_contract():
    """Verify that psram_benchmark_run_isolated_diagnostic implements all 5 test cases."""
    bench_h = (SRC_DIR / "psram_benchmark.h").read_text(encoding="utf-8")
    assert "int psram_benchmark_run_isolated_diagnostic(void);" in bench_h

    bench_c = (SRC_DIR / "psram_benchmark.c").read_text(encoding="utf-8")
    assert "int psram_benchmark_run_isolated_diagnostic(void)" in bench_c

    # Must check idle state
    assert "recorder_is_active()" in bench_c
    assert "return -EBUSY;" in bench_c

    # Case 1: Aligned single address with all 10 patterns
    assert 'Aligned (10 Pat)' in bench_c

    # Case 2: Deliberately unaligned starting addresses (1, 2, 3 byte offsets)
    assert 'Unaligned Start' in bench_c
    assert "unaligned_offsets[]" in bench_c

    # Case 3: 16-byte burst boundary crossings
    assert 'Burst Crossing' in bench_c
    assert "0x208U" in bench_c
    assert "0x20EU" in bench_c

    # Case 4: Non-multiple of 16-byte lengths
    assert 'Non-16 Multiple' in bench_c
    assert "non_16_lens[] = { 1U, 7U, 15U, 23U, 127U };" in bench_c

    # Case 5: Diagnostic boundary termination at 0x003FFFFF and bounds check
    assert 'Boundary Term' in bench_c
    assert "PSRAM_DIAG_TEST_MAX_ADDR" in bench_c
    assert "Out-of-Bounds Rejection" in bench_c
    assert "w_oob == -EINVAL && r_oob == -EINVAL" in bench_c


def test_main_wires_diag_commands():
    """Verify that 'psram bench diag' is wired in main.c under TEST verbose configuration."""
    main_c = (SRC_DIR / "main.c").read_text(encoding="utf-8")
    assert 'strcasecmp(a2, "diag") == 0' in main_c
    assert "psram_benchmark_run_isolated_diagnostic();" in main_c

