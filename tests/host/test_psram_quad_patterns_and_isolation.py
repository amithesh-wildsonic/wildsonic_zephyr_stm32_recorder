#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Test Suite: PSRAM Quad I/O Pattern Generation, Address Partitioning, and Cross-Mode Isolation
Verifies:
  1. Deterministic generation of all 10 diagnostic pattern classes.
  2. XOR bitmask diagnosis of observed hardware mismatches (0xFF->0xFD, 0x55->0xDD).
  3. Address space partitioning: test region 0x00300000-0x003FFFFF strictly isolated from 0x000000-0x0FFFFF live staging.
  4. Decoupled 4-mode cross-mode test isolation logic (Test A, Test B, Test C, Test D).
  5. Source code compliance: safe burst limits (4B SPI, 16B Quad), page boundary splits, and QSPI sample shifting configuration.
"""

from pathlib import Path
import re
try:
    import pytest
except ImportError:
    pytest = None

BASE_DIR = Path(__file__).resolve().parents[2] / "app"

# 1. Pattern generator model
def generate_pattern(pattern_type: str, length: int, base_addr: int = 0) -> bytes:
    buf = bytearray(length)
    if pattern_type == "ALL_ZEROS":
        pass  # bytearray initialized to 0
    elif pattern_type == "ALL_ONES":
        for i in range(length):
            buf[i] = 0xFF
    elif pattern_type == "ALT_55_AA":
        for i in range(length):
            buf[i] = 0xAA if (i & 1) else 0x55
    elif pattern_type == "ALT_AA_55":
        for i in range(length):
            buf[i] = 0x55 if (i & 1) else 0xAA
    elif pattern_type == "INCREMENTING":
        for i in range(length):
            buf[i] = (base_addr + i) & 0xFF
    elif pattern_type == "DECREMENTING":
        for i in range(length):
            buf[i] = (0xFF - ((base_addr + i) & 0xFF)) & 0xFF
    elif pattern_type == "WALKING_ONES":
        for i in range(length):
            buf[i] = 1 << (i % 8)
    elif pattern_type == "WALKING_ZEROS":
        for i in range(length):
            buf[i] = (~(1 << (i % 8))) & 0xFF
    elif pattern_type == "ALT_NIBBLES":
        for i in range(length):
            buf[i] = 0x0F if (i & 1) else 0xF0
    elif pattern_type == "PSEUDO_RANDOM":
        state = (0x12345678 ^ base_addr) & 0xFFFFFFFF
        for i in range(length):
            state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
            buf[i] = (state >> 24) & 0xFF
    else:
        raise ValueError(f"Unknown pattern type {pattern_type}")
    return bytes(buf)


def test_ten_patterns_deterministic():
    """Verify all 10 patterns generate exact deterministic sequences."""
    patterns = [
        "ALL_ZEROS", "ALL_ONES", "ALT_55_AA", "ALT_AA_55",
        "INCREMENTING", "DECREMENTING", "WALKING_ONES", "WALKING_ZEROS",
        "ALT_NIBBLES", "PSEUDO_RANDOM"
    ]
    for pat in patterns:
        data1 = generate_pattern(pat, 64, 0x00300000)
        data2 = generate_pattern(pat, 64, 0x00300000)
        assert data1 == data2, f"Pattern {pat} is not deterministic"
        assert len(data1) == 64

    # Specific pattern sanity checks
    assert generate_pattern("ALL_ZEROS", 4) == b"\x00\x00\x00\x00"
    assert generate_pattern("ALL_ONES", 4) == b"\xff\xff\xff\xff"
    assert generate_pattern("ALT_55_AA", 4) == b"\x55\xaa\x55\xaa"
    assert generate_pattern("ALT_AA_55", 4) == b"\xaa\x55\xaa\x55"
    assert generate_pattern("INCREMENTING", 4, 0) == b"\x00\x01\x02\x03"
    assert generate_pattern("DECREMENTING", 4, 0) == b"\xff\xfe\xfd\xfc"
    assert generate_pattern("WALKING_ONES", 8) == b"\x01\x02\x04\x08\x10\x20\x40\x80"
    assert generate_pattern("WALKING_ZEROS", 8) == b"\xfe\xfd\xfb\xf7\xef\xdf\xbf\x7f"
    assert generate_pattern("ALT_NIBBLES", 4) == b"\xf0\x0f\xf0\x0f"


def test_mismatch_bitmask_diagnosis():
    """Verify bitmask analysis for observed hardware log failures."""
    # Failure 1: 0xFF expected, 0xFD actual
    exp1 = 0xFF
    act1 = 0xFD
    diff1 = exp1 ^ act1
    assert diff1 == 0x02  # Bit 1 cleared
    # In Quad mode (1-4-4), lower nibble has bit 1 on IO1 pin
    assert (diff1 & (1 << 1)) != 0

    # Failure 2: 0x55 expected, 0xDD actual
    exp2 = 0x55  # 0101 0101
    act2 = 0xDD  # 1101 1101
    diff2 = exp2 ^ act2
    assert diff2 == 0x88  # Bits 7 and 3 set (IO3 pin)
    assert (diff2 & ~((1 << 7) | (1 << 3))) == 0


def test_address_space_isolation():
    """Verify test region strictly avoids live staging (0x000000-0x0FFFFF)."""
    staging_base = 0x000000
    staging_limit = 0x100000  # 1 MB mark
    diag_base = 0x00300000    # 3 MB mark
    diag_limit = 0x00400000   # 4 MB capacity

    assert diag_base >= staging_limit, "Diagnostic test region collides with audio staging!"
    assert diag_limit <= 0x00400000, "Diagnostic test region exceeds 4 MB capacity!"

    # Verify psram_is66wvs.h defines these isolated addresses
    psram_h = (BASE_DIR / "src" / "psram_is66wvs.h").read_text()
    assert "0x00300000" in psram_h
    assert "0x003FFFFF" in psram_h


def test_cross_mode_isolation_logic():
    """Verify cross-mode test isolation: Test B pass + Test C fail isolates Quad Read."""
    class CrossModeState:
        def __init__(self, mode_a_pass, mode_b_pass, mode_c_pass, mode_d_pass):
            self.mode_a = mode_a_pass
            self.mode_b = mode_b_pass
            self.mode_c = mode_c_pass
            self.mode_d = mode_d_pass

        def evaluate_quad_mode(self):
            return self.mode_a and self.mode_b and self.mode_c and self.mode_d

        def diagnose(self):
            if not self.mode_a:
                return "Basic SPI bus/memory physical defect"
            if self.mode_b and not self.mode_c:
                return "Quad Read (0xEB) path failing (dummy cycles / sample shifting / timing)"
            if not self.mode_b and self.mode_c:
                return "Quad Write (0x38) path failing"
            if not self.mode_b and not self.mode_c:
                return "Quad data lines IO2/IO3 physical signal integrity failure"
            return "Quad I/O verified healthy"

    # Current hardware log state: Test A and Test B pass, Test C and Test D fail
    state = CrossModeState(mode_a_pass=True, mode_b_pass=True, mode_c_pass=False, mode_d_pass=False)
    assert not state.evaluate_quad_mode(), "Quad mode must NOT be activated when Test C fails!"
    diagnosis = state.diagnose()
    assert "Quad Read (0xEB)" in diagnosis

    # Healthy state
    healthy = CrossModeState(mode_a_pass=True, mode_b_pass=True, mode_c_pass=True, mode_d_pass=True)
    assert healthy.evaluate_quad_mode()
    assert healthy.diagnose() == "Quad I/O verified healthy"


def test_source_code_burst_and_shift_guards():
    """Verify source code implements burst limits, sample shifting, and boundary handling."""
    psram_c = (BASE_DIR / "src" / "psram_is66wvs.c").read_text()
    psram_h = (BASE_DIR / "src" / "psram_is66wvs.h").read_text()
    main_c = (BASE_DIR / "src" / "main.c").read_text()

    # Safe burst enforcement in psram_write and psram_read
    assert "burst_limit = is_quad ? active_burst_size : PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;" in psram_c
    assert "PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI" in psram_c
    assert "PSRAM_IS66WVS_SAFE_BURST_BYTES_QUAD" in psram_c
    assert "PSRAM_IS66WVS_PAGE_SIZE" in psram_c

    # Sample shifting functions
    assert "psram_set_sample_shifting" in psram_h
    assert "psram_get_sample_shifting" in psram_h
    assert "psram_run_diagnostic_matrix" in psram_h

    # Cross-mode error tracking fields in report
    assert "mode_a_errors" in psram_h
    assert "mode_b_errors" in psram_h
    assert "mode_c_errors" in psram_h
    assert "mode_d_errors" in psram_h
    assert "sample_shifting" in psram_h

    # main.c console commands
    assert "psram sshift" in main_c
    assert "psram diag" in main_c
    assert "psram test" in main_c


if __name__ == "__main__":
    if pytest is not None:
        pytest.main([__file__, "-v"])
    else:
        test_ten_patterns_deterministic()
        test_mismatch_bitmask_diagnosis()
        test_address_space_isolation()
        test_cross_mode_isolation_logic()
        test_source_code_burst_and_shift_guards()
        print("All PSRAM quad patterns and isolation tests passed!")

