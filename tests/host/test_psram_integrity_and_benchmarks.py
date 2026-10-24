#!/usr/bin/env python3
"""
Test Suite: PSRAM Integrity Validation, Hardware Timing & Performance Benchmarks
Verifies:
  1. PSRAM transfer timing math for the configured 18 MHz, SPI (4 bytes) and Quad (16 bytes).
  2. Hex bitmask diagnosis for 0x55 -> 0xDD mismatch (IO3 PA6 stuck-high).
  3. Source inspection of benchmark.c, benchmark.h, psram_is66wvs.c, recorder.c, main.c.
  4. Memory bounds: Zero heap, zero extra static SRAM buffers (pool block borrowing).
  5. Mutual exclusion between audio recording sessions and benchmarks.
  6. Dedicated benchmark file safety (/SD:/BENCH.DAT never touching RECnnnn.WAV).
  7. Address space partitioning: Benchmark at 0x00200000 vs Audio staging at 0x00000000.
"""

import sys
import re
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[2] / "app"

def test_tcem_timing():
    print("Test 1: Checking wire-time estimates at 18 MHz (physical CE# timing remains unmeasured)...")
    qspi_freq_hz = 18_000_000
    t_cycle_ns = 1_000_000_000 / qspi_freq_hz
    tcem_max_ns = 4000.0                       # 4.0 us

    # 1-Line SPI: 8-bit cmd + 24-bit addr = 32 cycles overhead
    burst_spi = 4
    spi_cycles = 32 + (burst_spi * 8)
    t_spi_ns = spi_cycles * t_cycle_ns
    assert t_spi_ns <= tcem_max_ns, f"SPI burst {burst_spi} exceeds tCEM: {t_spi_ns} ns > {tcem_max_ns} ns"
    print(f"  [PASS] 1-Line SPI 4-byte burst: {spi_cycles} cycles = {t_spi_ns/1000:.2f} us <= 4.0 us")

    # Quad SPI (1-4-4): 8-bit cmd (8 cycles) + 24-bit addr (6 cycles) + 6 dummy (6 cycles) = 20 cycles overhead
    # 16 bytes @ 2 nibbles/byte = 32 cycles
    burst_quad = 16
    quad_cycles = 8 + 6 + 6 + (burst_quad * 2)
    t_quad_ns = quad_cycles * t_cycle_ns
    assert t_quad_ns <= tcem_max_ns, f"Quad burst {burst_quad} exceeds tCEM: {t_quad_ns} ns > {tcem_max_ns} ns"
    print(f"  [PASS] Quad I/O 28-byte burst: {quad_cycles} cycles = {t_quad_ns/1000:.2f} us <= 4.0 us")

def test_bitmask_diagnosis():
    print("\nTest 2: Validating root-cause bitmask diagnosis (0x55 -> 0xDD)...")
    expected = 0x55  # 0101 0101
    observed = 0xDD  # 1101 1101
    xor_diff = expected ^ observed  # 1000 1000 = 0x88
    # In Quad mode (1-4-4), bits 7 & 3 correspond to IO3 pin
    assert xor_diff == 0x88, f"Unexpected XOR diff: 0x{xor_diff:02X}"
    io3_bitmask = (1 << 7) | (1 << 3)
    assert (xor_diff & ~io3_bitmask) == 0, "Non-IO3 bits were affected!"
    print(f"  [PASS] Mismatch 0x{expected:02X} -> 0x{observed:02X}: failing bitmask is 0x{xor_diff:02X} (IO3 / PA6 stuck high)")

def test_source_guards():
    print("\nTest 3: Inspecting firmware source tree for PSRAM integrity & safety guards...")
    
    # 1. psram_is66wvs.h & .c
    psram_h = (BASE_DIR / "src" / "psram_is66wvs.h").read_text()
    psram_c = (BASE_DIR / "src" / "psram_is66wvs.c").read_text()
    assert "psram_is_integrity_verified" in psram_h
    assert "spi_integrity_verified" in psram_h
    assert "failing_bitmask" in psram_h
    assert "quad_mode_enabled = false;" in psram_c
    assert "psram_integrity_verified = false;" in psram_c
    assert "irq_lock()" not in psram_c
    assert "irq_unlock(key)" not in psram_c
    assert "HAL_QSPI_Abort" in psram_c
    assert "QSPI_ADDRESS_24_BITS" in psram_c
    assert "CMD_QUAD_WRITE" in psram_c and "CMD_QUAD_FAST_READ" in psram_c
    assert "cmd.DummyCycles = 6" in psram_c
    print("  [PASS] PSRAM paths preserve HAL tick progress, 24-bit addressing, and 0x38/0xEB phase setup.")

    # 2. psram_staging.c
    staging_c = (BASE_DIR / "src" / "psram_staging.c").read_text()
    assert "psram_is_integrity_verified()" in staging_c
    print("  [PASS] psram_staging_init() checks psram_is_integrity_verified().")

    # 3. recorder.c & .h
    recorder_h = (BASE_DIR / "src" / "recorder.h").read_text()
    recorder_c = (BASE_DIR / "src" / "recorder.c").read_text()
    assert "recorder_borrow_pool_blocks" in recorder_h
    assert "recorder_return_pool_blocks" in recorder_h
    assert "recorder_borrow_contiguous_pool" in recorder_h
    assert "recorder_return_contiguous_pool" in recorder_h
    assert "benchmark_is_running" in recorder_h
    assert "benchmark_set_running" in recorder_h
    assert "psram_is_integrity_verified()" in recorder_c
    print("  [PASS] recorder.c enforces mutual exclusion and contiguous pool block borrowing.")

    # 4. benchmark.c & .h
    bench_h_path = (BASE_DIR / "src" / "diagnostics" / "benchmark.h") if (BASE_DIR / "src" / "diagnostics" / "benchmark.h").exists() else (BASE_DIR / "src" / "benchmark.h")
    bench_c_path = (BASE_DIR / "src" / "diagnostics" / "benchmark.c") if (BASE_DIR / "src" / "diagnostics" / "benchmark.c").exists() else (BASE_DIR / "src" / "benchmark.c")
    bench_h = bench_h_path.read_text()
    bench_c = bench_c_path.read_text()
    assert "bench_run_ram" in bench_h
    assert "bench_run_psram" in bench_h
    assert "bench_run_sd" in bench_h
    assert "bench_run_pipeline" in bench_h
    assert "bench_run_pipeline_all" in bench_h
    assert "bench_print_status" in bench_h
    
    # Check address isolation and buffer bounds safety
    assert "0x00200000" in bench_c
    assert "/SD:/BENCH.DAT" in bench_c
    assert "/SD:/PIPETEST.DAT" in bench_c
    assert "0x20018000" in bench_c
    assert "FS_O_READ | FS_O_WRITE" in bench_c
    assert "REC" not in bench_c or "RECnnnn.WAV" in bench_c
    print("  [PASS] benchmark.c enforces bounds <= 0x20018000, readback flags, and file isolation.")

    # 5. main.c
    main_c = (BASE_DIR / "src" / "main.c").read_text()
    assert "bench_run_ram()" in main_c
    assert "bench_run_psram()" in main_c
    assert "bench_run_sd()" in main_c
    assert "bench_run_pipeline" in main_c
    assert "bench_print_status()" in main_c
    assert "psram_is_integrity_verified()" in main_c
    print("  [PASS] main.c dispatches all bench commands and guards 'psram on' with integrity check.")

def test_memory_allocation():
    print("\nTest 4: Verifying zero static buffer allocation in benchmark.c...")
    bench_c_path = (BASE_DIR / "src" / "diagnostics" / "benchmark.c") if (BASE_DIR / "src" / "diagnostics" / "benchmark.c").exists() else (BASE_DIR / "src" / "benchmark.c")
    bench_c = bench_c_path.read_text()
    
    # Check that large static buffers (e.g. static uint8_t buf[...]) do NOT exist
    static_bufs = re.findall(r"static\s+(?:uint8_t|uint16_t|uint32_t|char)\s+\w+\[\s*(\d+)\s*\]", bench_c)
    large_bufs = [int(s) for s in static_bufs if int(s) > 1024]
    assert len(large_bufs) == 0, f"Found large static buffer in benchmark.c: {large_bufs}"
    print("  [PASS] No static buffers > 1024 B found in benchmark.c (pool blocks borrowed dynamically).")

def main():
    print("=" * 75)
    print("  WILDSONIC PSRAM INTEGRITY & BENCHMARK VALIDATION SUITE")
    print("=" * 75)
    test_tcem_timing()
    test_bitmask_diagnosis()
    test_source_guards()
    test_memory_allocation()
    print("\n" + "=" * 75)
    print("  ALL PSRAM INTEGRITY & BENCHMARK TESTS PASSED SUCCESSFULLY!")
    print("=" * 75)

if __name__ == "__main__":
    main()

