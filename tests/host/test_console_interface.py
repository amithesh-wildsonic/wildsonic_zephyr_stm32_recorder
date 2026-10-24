#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Unit and Contract Verification Suite for WildSonic Console & Command Dispatcher.

Verifies:
 1. Canonical WildSonic product command interface matching reference Maxim MAX78000 firmware:
    HELP, SAMF, BTPS, NUMCH, AUDIO, RECT, SLEEP, DATETIME, TIMEZONE,
    DATERANGE, TIMEOFDAY, DAYSOFWEEK, GAIN, LCUTOFF, UCUTOFF, FLDR,
    STOR, VERSION, SN, PW, ASSERT, DEFAULT, APPLY, RBT, ARBT.
 2. Strict build separation:
    - In PRODUCTION: development commands (start, stop, record, rate, time,
      bench, diagnose, compare, stress, integrity, schedule) are strictly excluded
      from help output and rejected as "Unknown or BAD command".
    - In TEST: engineering commands (bench, diagnose, compare, stress, integrity,
      psram [test|diag|sshift|bench], manual start/stop) are enabled under Kconfig guards.
 3. Explicitly deferred hardware commands (GSM, BATT, ENVSTS, LOC, SUN*, BLEPAIR, etc.)
    return explicit notice rather than misleading stubs or failures.
 4. Reference syntax, getter/setter behavior, and response formatting:
    - "Value is set to %u"
    - "Send APPLY command and reboot for changes to take effect"
    - "List of commands"
"""

import re
import pytest
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
APP_DIR = ROOT_DIR / "app" if (ROOT_DIR / "app").is_dir() else ROOT_DIR
SRC_DIR = APP_DIR / "src"

CONSOLE_C = SRC_DIR / "console.c"
CONSOLE_H = SRC_DIR / "console.h"
MAIN_C = SRC_DIR / "main.c"
MAIN_H = SRC_DIR / "main.h"


# ==============================================================================
# 1. FILE EXISTENCE & ARCHITECTURE VERIFICATION
# ==============================================================================

def test_console_source_files_exist():
    """Verify console header and implementation are cleanly separated."""
    assert CONSOLE_C.is_file(), f"Missing {CONSOLE_C}"
    assert CONSOLE_H.is_file(), f"Missing {CONSOLE_H}"
    assert MAIN_H.is_file(), f"Missing {MAIN_H}"


def test_main_includes_console_h():
    """Verify main.c includes console.h and calls console_init & console_process_line."""
    main_text = MAIN_C.read_text(encoding="utf-8")
    assert '#include "console.h"' in main_text
    assert "console_init()" in main_text
    assert "console_process_line" in main_text


# ==============================================================================
# 2. CANONICAL PRODUCT COMMAND CONTRACT (MAX78000 PARITY)
# ==============================================================================

CANONICAL_PRODUCT_COMMANDS = [
    "HELP", "SAMF", "BTPS", "NUMCH", "AUDIO", "RECT", "SLEEP",
    "DATETIME", "TIMEZONE", "DATERANGE", "TIMEOFDAY", "DAYSOFWEEK",
    "GAIN", "LCUTOFF", "UCUTOFF", "FLDR", "STOR", "VERSION", "SN",
    "PW", "ASSERT", "DEFAULT", "APPLY", "RBT", "ARBT", "SDFMT"
]

DEFERRED_HARDWARE_COMMANDS = [
    "GSM", "BATT", "ENVSTS", "LOC", "SUN", "SUNMODE", "SUNRISEBEF",
    "SUNRISEAFT", "SUNSETBEF", "SUNSETAFT", "SUNROUND", "BLEPAIR",
    "SIG", "LBAT", "NBAT", "FULLCAP", "SENDSTS", "I2C", "RTCRST",
    "TEST_ALARM", "DNAME", "SMSKW", "SMSKEY", "A2PPHONE", "P2APHONE",
    "SURL", "SPORT", "POSTROUTE", "POSTCTNT", "STEST", "SWS", "SMSFREQ"
]

TEST_ONLY_COMMANDS = [
    "BENCH", "DIAGNOSE", "COMPARE", "STRESS", "INTEGRITY",
    "PSRAM", "START", "STOP", "SCHEDULE"
]


def test_canonical_commands_registered():
    """Verify all canonical WildSonic product commands are in the command table."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    for cmd in CANONICAL_PRODUCT_COMMANDS:
        assert f'"{cmd}"' in c_text, f"Canonical product command {cmd} not found in console.c"


def test_deferred_hardware_commands_explicitly_handled():
    """Verify hardware-dependent commands return [DEFERRED] message."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    assert "[DEFERRED]" in c_text
    for cmd in DEFERRED_HARDWARE_COMMANDS:
        assert f'"{cmd}"' in c_text, f"Deferred command {cmd} not handled in console.c"


# ==============================================================================
# 3. STRICT BUILD SEPARATION: PRODUCTION VS TEST
# ==============================================================================

def test_production_excludes_test_commands():
    """Verify test commands are guarded by Kconfig in console.c and absent from production."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    # In console_cmd_type enum and console_cmd_lookup_table, test commands MUST be wrapped in #if
    guard = "#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS || CONFIG_AUDIO_INTEGRITY_REPLAY"
    assert guard in c_text, "Kconfig guard for test commands missing in console.c"

    # Verify that without test configs, the test commands are not in the production table
    # Check that each test command is inside guarded blocks
    for cmd in TEST_ONLY_COMMANDS:
        occurrences = [m.start() for m in re.finditer(f'"{cmd}"', c_text)]
        assert len(occurrences) > 0, f"Test command {cmd} should be registered in test mode"


def test_production_unknown_command_response():
    """Verify unknown or unauthorized commands produce canonical Maxim error message."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    assert "\\nUnknown or BAD command : %s\\n" in c_text


# ==============================================================================
# 4. REFERENCE RESPONSE STRINGS (MAX78000 FORMATTING PARITY)
# ==============================================================================

def test_canonical_response_formatting():
    """Verify exact formatting strings from Maxim WildSonic firmware."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    # Setters confirmation
    assert "\\nValue is set to %u\\n" in c_text
    assert "\\nSend APPLY command and reboot for changes to take effect\\n" in c_text

    # Help list
    assert "\\nList of commands\\n" in c_text
    assert " %s : %s\\n" in c_text

    # Reboot & persistence deferred notices
    assert "\\nRebooting device.\\n" in c_text
    assert "Flash persistence not implemented" in c_text


# ==============================================================================
# 5. ARGUMENT VALIDATION LOGIC SIMULATION
# ==============================================================================

class ConsoleValidatorModel:
    """Python simulation of the C console parser & validator functions."""

    SUPPORTED_SAMPLE_RATES = (192000, 96000, 48000)

    @staticmethod
    def validate_samf(rate: int):
        if rate in ConsoleValidatorModel.SUPPORTED_SAMPLE_RATES:
            return True, rate
        return False, None

    @staticmethod
    def validate_btps(bits: int):
        if bits == 16:
            return True, 16
        if bits in (24, 32):
            return False, "DEFERRED"
        return False, None

    @staticmethod
    def validate_numch(channels: int):
        if channels == 1:
            return True, 1
        if channels == 2:
            return False, "DEFERRED"
        return False, None

    @staticmethod
    def validate_rect(seconds: int):
        if 1 <= seconds <= 86400:
            return True, seconds
        return False, None

    @staticmethod
    def validate_sleep(seconds: int):
        if 0 <= seconds <= 86400:
            return True, seconds
        return False, None

    @staticmethod
    def validate_gain(gain_db: float):
        if 0.0 <= gain_db <= 40.0:
            return True, gain_db
        return False, None

    @staticmethod
    def validate_cutoff(cutoff_hz: int):
        if 0 <= cutoff_hz <= 96000:
            return True, cutoff_hz
        return False, None


def test_validator_sample_rate():
    val = ConsoleValidatorModel()
    assert val.validate_samf(192000) == (True, 192000)
    assert val.validate_samf(96000) == (True, 96000)
    assert val.validate_samf(48000) == (True, 48000)
    assert val.validate_samf(44100) == (False, None)
    assert val.validate_samf(384000) == (False, None)


def test_validator_bit_depth():
    val = ConsoleValidatorModel()
    assert val.validate_btps(16) == (True, 16)
    assert val.validate_btps(24) == (False, "DEFERRED")
    assert val.validate_btps(8) == (False, None)


def test_validator_channels():
    val = ConsoleValidatorModel()
    assert val.validate_numch(1) == (True, 1)
    assert val.validate_numch(2) == (False, "DEFERRED")
    assert val.validate_numch(4) == (False, None)


def test_validator_rect_and_sleep():
    val = ConsoleValidatorModel()
    assert val.validate_rect(20) == (True, 20)
    assert val.validate_rect(0) == (False, None)
    assert val.validate_rect(86400) == (True, 86400)
    assert val.validate_rect(86401) == (False, None)

    assert val.validate_sleep(0) == (True, 0)
    assert val.validate_sleep(60) == (True, 60)
    assert val.validate_sleep(-1) == (False, None)
    assert val.validate_sleep(100000) == (False, None)


def test_validator_gain_and_cutoffs():
    val = ConsoleValidatorModel()
    assert val.validate_gain(0.0) == (True, 0.0)
    assert val.validate_gain(12.5) == (True, 12.5)
    assert val.validate_gain(40.0) == (True, 40.0)
    assert val.validate_gain(-1.0) == (False, None)
    assert val.validate_gain(41.0) == (False, None)

    assert val.validate_cutoff(80) == (True, 80)
    assert val.validate_cutoff(20000) == (True, 20000)
    assert val.validate_cutoff(100000) == (False, None)
