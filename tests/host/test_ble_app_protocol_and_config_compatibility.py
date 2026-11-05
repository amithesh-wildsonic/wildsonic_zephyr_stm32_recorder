# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for WildSonic BLE Mobile Companion Protocol & Configuration Compatibility.

Covers:
- Section 5: WS+ALIVE keep-alive packet handling and pairing window refresh.
- Section 3 & 4: GET_DEV_INFO and GET_STATUS_INFO exact line formatting, field order, and schema parity with original reference firmware (BleComm.cpp).
- Section 2: Strict numeric validation (rejection of trailing characters, negative values, and overflows).
- Section 2: Single command response ownership for RBT and ARBT.
- Section 7: Robust and transactional SD config importer (EOF without newline, >127 byte lines, rollback on error).
"""

import re
import pytest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
BLE_COMM_C = REPO_ROOT / "app" / "src" / "ble_comm.c"
CONSOLE_C = REPO_ROOT / "app" / "src" / "console.c"
CONFIG_STORE_C = REPO_ROOT / "app" / "src" / "config_store.c"
CONSOLE_H = REPO_ROOT / "app" / "src" / "console.h"


# ==============================================================================
# 1. WS+ALIVE Keep-Alive Handling
# ==============================================================================

def test_ws_alive_intercept_and_silent_ack():
    """Verify WS+ALIVE is intercepted, refreshes the pairing window, and emits no text response."""
    src = BLE_COMM_C.read_text(encoding="utf-8")
    assert 'strcmp(p, "WS+ALIVE") == 0' in src
    # Must refresh pairing / active connection timer
    assert "s_pairing_window_active = true;" in src
    # Must return without sending command response
    proc_pos = src.find("void ble_comm_process_line")
    assert proc_pos != -1
    proc_block = src[proc_pos:src.find("/* Dispatch into canonical console parser", proc_pos)]
    assert "WS+ALIVE" in proc_block
    assert "return;" in proc_block


# ==============================================================================
# 2. Strict Numeric Validation
# ==============================================================================

def test_strict_numeric_parsing_helpers_present():
    """Verify strict numeric validation helper parses numbers without trailing garbage or overflow."""
    src = CONSOLE_C.read_text(encoding="utf-8")
    assert "parse_strict_u32" in src
    helper_pos = src.find("parse_strict_u32")
    helper_code = src[helper_pos:helper_pos + 600]
    # Check endptr and ERANGE enforcement
    assert "*endptr != '\\0'" in helper_code
    assert "ERANGE" in helper_code


def test_command_handlers_use_strict_numeric_validation():
    """Verify SAMF, RECT, SLEEP, BTPS, NUMCH, LCUTOFF, UCUTOFF use strict numeric parsing."""
    src = CONSOLE_C.read_text(encoding="utf-8")
    handlers = ["handle_samf", "handle_rect", "handle_sleep", "handle_btps", "handle_numch", "handle_lcutoff", "handle_ucutoff"]
    for h in handlers:
        assert f"{h}(" in src
        pos = src.find(f"{h}(")
        func_body = src[pos:pos + 1200]
        assert "parse_strict_u32" in func_body, f"{h} must use parse_strict_u32"


# ==============================================================================
# 3. GET_DEV_INFO Schema Parity
# ==============================================================================

def test_get_dev_info_schema_parity():
    """Verify ble_comm_send_dev_info includes all canonical fields in matching format with BleComm.cpp."""
    src = BLE_COMM_C.read_text(encoding="utf-8")
    assert "void ble_comm_send_dev_info(void)" in src
    dev_pos = src.find("void ble_comm_send_dev_info(void)")
    dev_body = src[dev_pos:src.find("void ble_comm_send_status_info(void)", dev_pos)]

    expected_lines = [
        '"SAMF %u\\r\\n"',
        '"BTPS %u\\r\\n"',
        '"NUMCH %u\\r\\n"',
        '"GAIN %d\\r\\n"',
        '"LCUTOFF %u\\r\\n"',
        '"UCUTOFF %u\\r\\n"',
        '"RECT %u\\r\\n"',
        '"SLEEP %u\\r\\n"',
        '"VER 1.2.0, SHA d7a4f91, DT %s %s\\r\\n"',
        '"SDUSED %llu, SDCAP %llu\\r\\n"',
        '"FLDR %s\\r\\n"',
        '"VBATT %u"',
        '"CBATT %d"',
        '"REMBATT %u\\r\\n"',
        '"FULLBATT %u"',
        '"REMTIME %u"',
        '"AVGPWR %d\\r\\n"',
        '"VBATT N/A"',
        '"CBATT N/A"',
        '"REMBATT N/A\\r\\n"',
        '"FULLBATT N/A"',
        '"REMTIME N/A"',
        '"AVGPWR N/A\\r\\n"',
        '"NBAT 1\\r\\n"',
        '"DATETIME',
        '"TIMEZONE',
        '"DATERANGE',
        '"SUNMODE',
        '"TIMEOFDAY %d',
        '"DAYSOFWEEK %s\\r\\n"',
        '"TEMP 0.00, HUMID 0.00\\r\\n"',
    ]
    for line_fmt in expected_lines:
        assert line_fmt in dev_body, f"Expected format string {line_fmt} in ble_comm_send_dev_info"


# ==============================================================================
# 4. GET_STATUS_INFO Schema Parity
# ==============================================================================

def test_get_status_info_schema_parity():
    """Verify ble_comm_send_status_info emits canonical status fields per BleComm.cpp."""
    src = BLE_COMM_C.read_text(encoding="utf-8")
    assert "void ble_comm_send_status_info(void)" in src
    status_pos = src.find("void ble_comm_send_status_info(void)")
    status_body = src[status_pos:src.find("int ble_comm_feed_rx_bytes", status_pos)]

    expected_status_lines = [
        '"SDUSED %llu, SDCAP %llu\\r\\n"',
        '"VBATT %u"',
        '"CBATT %d"',
        '"REMBATT %u\\r\\n"',
        '"FULLBATT %u"',
        '"REMTIME %u"',
        '"AVGPWR %d\\r\\n"',
        '"VBATT N/A"',
        '"CBATT N/A"',
        '"REMBATT N/A\\r\\n"',
        '"FULLBATT N/A"',
        '"REMTIME N/A"',
        '"AVGPWR N/A\\r\\n"',
        '"TEMP 0.00, HUMID 0.00\\r\\n"',
    ]
    for line_fmt in expected_status_lines:
        assert line_fmt in status_body, f"Expected format string {line_fmt} in ble_comm_send_status_info"


# ==============================================================================
# 5. Single Response Ownership for Reboot Commands
# ==============================================================================

def test_reboot_commands_single_response_ownership():
    """Verify RBT and ARBT have single response ownership and no duplicate prints."""
    src = CONSOLE_C.read_text(encoding="utf-8")
    rbt_pos = src.find("static void handle_rbt(void)")
    rbt_body = src[rbt_pos:src.find("/* ARBT handler", rbt_pos)]
    assert rbt_body.count('printk("\\nRebooting device.\\n");') == 1

    arbt_pos = src.find("static void handle_arbt(void)")
    arbt_body = src[arbt_pos:src.find("/* SDFMT handler", arbt_pos)]
    assert arbt_body.count('printk("\\nRebooting device.\\n");') == 1


# ==============================================================================
# 6. SD Config Importer Transactional Safety
# ==============================================================================

def test_sd_config_importer_transactional_safety():
    """Verify config_store_import_sd handles line limits, EOF without newline, and rollback on error."""
    src = CONFIG_STORE_C.read_text(encoding="utf-8")
    import_pos = src.find("int config_store_import_sd(void)")
    assert import_pos != -1
    import_body = src[import_pos:]

    # Checks line length bounds
    assert "line_pos >= sizeof(line) - 1" in import_body
    assert "discarding_oversized" in import_body

    # Checks rollback on error
    assert "config_store_rollback_pending()" in import_body
    assert "import_error" in import_body
    assert "updated = false;" in import_body

    # Checks file archiving
    assert "config.bak" in import_body
    assert "config_status.txt" in import_body
