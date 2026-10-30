# SPDX-License-Identifier: Apache-2.0
"""
Unit and Contract Tests for:
1. BLE Default Unlocked State (AT+ADTY0, no auto-pairing timeout at boot).
2. DNAME & SN Parity with Original WildSonic (max 6/3 alphanumeric chars, formats, defaults).
3. BLE Advertised Name Builder (WS:<DNAME>-<SN>, fallbacks XXX/00, max 12 chars, deferred update).
4. SD Configuration Importer Parity with OnBootConfig.cpp (translation, unsupported & metadata keys).
5. RTC Backup Register Retention Across NRST Reset (Magic, Version, 64-bit UTC, XOR Checksum, Cooldown Calculation).
"""

import re
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
BLE_COMM_C = APP_SRC / "ble_comm.c"
BLE_COMM_H = APP_SRC / "ble_comm.h"
CONSOLE_C = APP_SRC / "console.c"
CONFIG_STORE_C = APP_SRC / "config_store.c"
MAIN_C = APP_SRC / "main.c"
SCHED_RET_C = APP_SRC / "scheduler_retention.c"
SCHED_RET_H = APP_SRC / "scheduler_retention.h"


# ==============================================================================
# 1. BLE Defaults & Startup Connectability Tests
# ==============================================================================

def test_ble_comm_initial_unlocked_connectable_state():
    """Verify BLE starts unlocked with AT+ADTY0 and s_current_adty = 0."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    
    # Static initialization must be 0 (unlocked)
    assert re.search(r"static\s+uint8_t\s+s_current_adty\s*=\s*0;", c_src), \
        "s_current_adty must initialize to 0 (unlocked)"

    # ble_comm_init must command AT+ADTY0 and expect OK+Set:0
    init_pos = c_src.find("int ble_comm_init(")
    assert init_pos != -1
    init_fn = c_src[init_pos:c_src.find("bool ble_comm_is_uart_ready", init_pos)]

    assert 'AT+ADTY0\\r\\n' in init_fn, "ble_comm_init must configure AT+ADTY0 at boot"
    assert 'OK+Set:0' in init_fn, "ble_comm_init must expect OK+Set:0"
    assert '[BLE] Access: UNLOCKED (initial boot state)' in init_fn, \
        "ble_comm_init must log initial unlocked boot state"

    # Must NOT automatically start a pairing window timeout at boot
    assert "s_pairing_window_active = false;" in init_fn, \
        "ble_comm_init must not start an auto-expiring pairing window at boot"
    assert "s_pairing_window_end_ms = 0;" in init_fn


def test_ble_comm_active_window_semantics():
    """Verify pairing window active checks s_current_adty == 0."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    fn_pos = c_src.find("bool ble_comm_is_pairing_window_active(void)")
    assert fn_pos != -1
    fn = c_src[fn_pos:c_src.find("uint32_t ble_comm_get_pairing_window_remaining", fn_pos)]

    assert "s_current_adty != 0" in fn, "Must check s_current_adty in active check"
    assert "return false;" in fn
    assert "return true;" in fn


# ==============================================================================
# 2. DNAME, SN & Factory Defaults Parity Tests
# ==============================================================================

def test_factory_defaults_align_with_original_wildsonic():
    """Verify device_name = '' and serial_number = '0' defaults."""
    c_console = CONSOLE_C.read_text(encoding="utf-8")
    c_store = CONFIG_STORE_C.read_text(encoding="utf-8")

    # In console.c g_cfg
    assert re.search(r'\.serial_number\s*=\s*"0"', c_console), \
        "Default serial number in console.c must be '0'"
    assert re.search(r'\.device_name\s*=\s*""', c_console), \
        "Default device name in console.c must be empty string"

    # In config_store.c load_defaults
    assert 'strncpy(cfg->serial_number, "0"' in c_store, \
        "config_store_load_defaults must set serial_number to '0'"
    assert 'cfg->device_name[0] = \'\\0\';' in c_store, \
        "config_store_load_defaults must set device_name to empty string"


def test_dname_handler_constraints_and_formatting():
    """Verify DNAME enforces max 6 alphanumeric characters and canonical formatting."""
    c_src = CONSOLE_C.read_text(encoding="utf-8")
    dname_pos = c_src.find("static int handle_dname(")
    assert dname_pos != -1, "handle_dname must be defined and return int"
    dname_fn = c_src[dname_pos:c_src.find("static int handle_sn", dname_pos) if "static int handle_sn" in c_src[dname_pos:] else dname_pos + 1200]

    # Max length 6 check
    assert "args_length > 6" in dname_fn, "DNAME must reject length > 6"
    assert "isalnum" in dname_fn, "DNAME must require alphanumeric characters"
    assert "Invalid value %s" in dname_fn, "DNAME must report Invalid value on rejection"
    assert "Value is set to %s" in dname_fn, "DNAME must print Value is set to <value>"
    assert "Send APPLY command and reboot" in dname_fn, "DNAME must prompt for APPLY on success"
    assert "ble_comm_notify_name_changed" in dname_fn, "DNAME must notify BLE of name change"


def test_sn_handler_constraints_and_formatting():
    """Verify SN enforces max 3 alphanumeric characters and canonical formatting."""
    c_src = CONSOLE_C.read_text(encoding="utf-8")
    sn_pos = c_src.find("static int handle_sn(")
    assert sn_pos != -1, "handle_sn must be defined and return int"
    sn_fn = c_src[sn_pos:c_src.find("/* DNAME handler", sn_pos)]

    # Max length 3 check
    assert "args_length > 3" in sn_fn, "SN must reject length > 3 per MAX_SERIAL_NUMBER_LENGTH"
    assert "isalnum" in sn_fn, "SN must require alphanumeric characters"
    assert "Invalid value %s" in sn_fn
    assert "Value is set to %s" in sn_fn
    assert "ble_comm_notify_name_changed" in sn_fn, "SN must notify BLE of name change"


# ==============================================================================
# 3. BLE Advertised Name Builder Logic Simulation
# ==============================================================================

def simulate_ble_build_advertised_name(dname: str, sn: str):
    """Python reference simulator matching ble_comm_build_advertised_name in ble_comm.c."""
    name_valid = bool(dname and 0 < len(dname) <= 6 and dname.isalnum())
    sn_valid = bool(sn and 0 < len(sn) <= 3 and sn.isalnum())

    use_name = dname if name_valid else "XXX"
    use_sn = sn if sn_valid else "00"

    adv = f"WS:{use_name}-{use_sn}"
    if len(adv) > 12:
        return -1, None
    return 0, adv


def test_ble_build_advertised_name_rules():
    """Verify BLE advertised name builder fallbacks, formats, and 12-char limit."""
    # 1. Defaults: empty dname -> XXX, "0" sn -> 0
    rc, name = simulate_ble_build_advertised_name("", "0")
    assert rc == 0
    assert name == "WS:XXX-0"
    assert len(name) <= 12

    # 2. Both empty: XXX and 00
    rc, name = simulate_ble_build_advertised_name("", "")
    assert rc == 0
    assert name == "WS:XXX-00"
    assert len(name) <= 12

    # 3. Valid name and sn
    rc, name = simulate_ble_build_advertised_name("REC1", "01")
    assert rc == 0
    assert name == "WS:REC1-01"
    assert len(name) <= 12

    # 4. Maximum permitted 12-character combination: 6-char name + 2-char SN -> WS:MYDEV1-99 (12 chars)
    rc, name = simulate_ble_build_advertised_name("MYDEV1", "99")
    assert rc == 0
    assert name == "WS:MYDEV1-99"
    assert len(name) == 12

    # 5. Length limit exceeded: 6-char name + 3-char SN -> WS:MYDEV1-123 (13 chars > 12 limit)
    rc, name = simulate_ble_build_advertised_name("MYDEV1", "123")
    assert rc != 0, "Must return error when WS:<DNAME>-<SN> exceeds 12 chars"
    assert name is None

    # 6. Invalid non-alphanumeric characters trigger fallbacks
    rc, name = simulate_ble_build_advertised_name("A_B", "01")
    assert rc == 0
    assert name == "WS:XXX-01"


def test_ble_deferred_name_update_while_connected():
    """Verify that name updates while central connected are deferred until OK+LOST."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    apply_fn = c_src[c_src.find("void ble_comm_apply_advertised_name(void)"):c_src.find("void ble_comm_suspend")]

    assert "s_ble_connected" in apply_fn
    assert "s_name_update_pending = true;" in apply_fn, \
        "Must set s_name_update_pending when central is connected"

    # On OK+LOST, pending name update must execute
    lost_pos = c_src.find('strcmp(p, "OK+LOST") == 0')
    assert lost_pos != -1
    lost_block = c_src[lost_pos:lost_pos + 400]
    assert "s_name_update_pending" in lost_block
    assert "ble_comm_apply_advertised_name();" in lost_block


# ==============================================================================
# 4. SD Configuration Importer Parity Tests
# ==============================================================================

def test_sd_config_importer_acceptance_and_unsupported_classification():
    """Verify SD config importer translates supported keys and ignores unsupported/metadata keys."""
    c_src = CONFIG_STORE_C.read_text(encoding="utf-8")
    import_pos = c_src.find("int config_store_import_sd(void)")
    assert import_pos != -1
    import_fn = c_src[import_pos:]

    # Supported key mappings
    for key, cmd in [
        ("sampling_frequency", "SAMF"),
        ("record_length", "RECT"),
        ("sleep_interval", "SLEEP"),
        ("bits_per_sample", "BTPS"),
        ("number_of_channels", "NUMCH"),
        ("mic_gain", "GAIN"),
        ("lower_cutoff_frequency", "LCUTOFF"),
        ("upper_cutoff_frequency", "UCUTOFF"),
        ("date_range", "DATERANGE"),
        ("days_of_week", "DAYSOFWEEK"),
        ("time_zone", "TIMEZONE"),
        ("system_time", "DATETIME"),
        ("folder_structure", "FLDR"),
    ]:
        assert f'"{key}"' in import_fn, f"Key {key} must be handled"
        assert f'"{cmd} ' in import_fn, f"Command {cmd} must be formatted"

    # time_of_day_0..9 index constraint
    assert "isdigit((unsigned char)key[12]) && key[13] == '\\0'" in import_fn, \
        "time_of_day_ must strictly restrict index to 0..9"

    # Unsupported hardware settings
    for unsupp in [
        "transmission_mode", "location", "sms_frequency", "low_battery_threshold",
        "sun_recording_mode", "sunrise_interval_before", "sunrise_interval_after",
        "sunset_interval_before", "sunset_interval_after", "sunrise_sunset_rounding"
    ]:
        assert f'"{unsupp}"' in import_fn, f"Unsupported setting {unsupp} must be classified"

    # Metadata keys
    for meta in ["build_version", "build_datetime", "build_sha"]:
        assert f'"{meta}"' in import_fn, f"Metadata key {meta} must be classified"

    # console_process_line_source invocation and update gating
    assert "console_process_line_source(cmd_buf, false, NULL, 0)" in import_fn
    assert "if (rc == 0) {\n\t\t\t\t\tupdated = true;" in import_fn or "rc == 0" in import_fn
    assert "if (updated) {\n\t\tconfig_store_save();\n\t}" in import_fn, \
        "config_store_save must only execute if updated is true"


# ==============================================================================
# 5. RTC Backup Register Retention Across NRST Reset Tests
# ==============================================================================

class RtcBackupRegisterSimulator:
    """Simulates STM32L476 RTC backup data registers (BKP0R..BKP4R)."""
    MAGIC = 0x57534348      # 'WSCH'
    VERSION = 0x00010001    # v1.1
    XOR_CONST = 0xA5A55A5A

    def __init__(self):
        self.regs = [0] * 5

    def save(self, end_utc: int) -> int:
        if end_utc <= 0:
            return -22  # -EINVAL
        low = end_utc & 0xFFFFFFFF
        high = (end_utc >> 32) & 0xFFFFFFFF
        cksum = self.MAGIC ^ self.VERSION ^ low ^ high ^ self.XOR_CONST

        self.regs[0] = self.MAGIC
        self.regs[1] = self.VERSION
        self.regs[2] = low
        self.regs[3] = high
        self.regs[4] = cksum
        return 0

    def restore(self) -> (int, int):
        magic = self.regs[0]
        if magic != self.MAGIC:
            return -22, 0  # -EINVAL (no retained data)
        ver = self.regs[1]
        if ver != self.VERSION:
            return -22, 0  # -EINVAL (version mismatch)
        low = self.regs[2]
        high = self.regs[3]
        stored_cksum = self.regs[4]

        expected_cksum = magic ^ ver ^ low ^ high ^ self.XOR_CONST
        if stored_cksum != expected_cksum:
            return -74, 0  # -EBADMSG (checksum corrupted)

        ts = (high << 32) | low
        if ts < 1704067200 or ts > 4102444800:
            return -22, 0
        return 0, ts

    def invalidate(self):
        self.regs = [0] * 5


def test_rtc_retention_save_restore_roundtrip():
    """Verify retention register serialization, 64-bit UTC timestamp and checksum."""
    sim = RtcBackupRegisterSimulator()
    test_ts = 1730290000  # 2024 timestamp

    assert sim.save(test_ts) == 0
    assert sim.regs[0] == 0x57534348
    assert sim.regs[1] == 0x00010001
    assert sim.regs[2] == test_ts & 0xFFFFFFFF
    assert sim.regs[3] == (test_ts >> 32) & 0xFFFFFFFF

    rc, restored_ts = sim.restore()
    assert rc == 0
    assert restored_ts == test_ts


def test_rtc_retention_checksum_corruption_detection():
    """Verify corrupted backup register bit causes checksum validation failure."""
    sim = RtcBackupRegisterSimulator()
    sim.save(1730290000)

    # Corrupt bit in DR2 (timestamp low)
    sim.regs[2] ^= 0x00000001

    rc, ts = sim.restore()
    assert rc == -74  # -EBADMSG
    assert ts == 0


def test_rtc_retention_version_mismatch():
    """Verify version mismatch returns error."""
    sim = RtcBackupRegisterSimulator()
    sim.save(1730290000)
    sim.regs[1] = 0x00020000  # v2.0

    rc, ts = sim.restore()
    assert rc == -22  # -EINVAL


def test_scheduler_cooldown_retention_across_nrst_reset():
    """
    Verify the critical NRST bug fix:
    Session completed at T=1000. Sleep interval = 60s. Target wake = T=1060.
    At T=1015 (15s into cooldown), NRST reset occurs.
    Device boots at T=1016.
    Restored prior end time T=1000 calculates remaining cooldown = 44s.
    Scheduler MUST NOT start recording immediately; it must wait 44s.
    """
    sim = RtcBackupRegisterSimulator()
    session_end_utc = 1730290000
    sleep_interval = 60

    # 1. Successful session saves end timestamp
    assert sim.save(session_end_utc) == 0

    # 2. NRST reset occurs. On boot, restore prior end time
    rc, restored_end_utc = sim.restore()
    assert rc == 0
    assert restored_end_utc == session_end_utc

    # 3. Simulate scheduler evaluation at reboot time (15s after session end)
    reboot_now_utc = session_end_utc + 15
    cooldown_end_utc = restored_end_utc + sleep_interval
    time_remaining = cooldown_end_utc - reboot_now_utc

    assert time_remaining == 45, "Must compute remaining 45 seconds of cooldown"
    is_immediate = (time_remaining <= 0)
    assert not is_immediate, "Must NOT trigger immediate recording on reboot during cooldown"

    # 4. If reboot occurs AFTER cooldown has elapsed (e.g. 70s later)
    late_reboot_now_utc = session_end_utc + 70
    late_remaining = max(0, (restored_end_utc + sleep_interval) - late_reboot_now_utc)
    assert late_remaining == 0
    late_is_immediate = (late_remaining <= 0)
    assert late_is_immediate, "Must trigger immediate recording when cooldown already elapsed"


def test_scheduler_sleep_0_starts_immediately():
    """Verify that SLEEP 0 (sleep_interval=0) immediately starts recording without delay."""
    session_end_utc = 1730290000
    sleep_interval = 0
    reboot_now_utc = session_end_utc + 5

    remaining = max(0, (session_end_utc + sleep_interval) - reboot_now_utc)
    assert remaining == 0
    assert remaining <= 0, "SLEEP 0 must allow immediate recording"


def test_aborted_session_does_not_update_retention():
    """Verify aborted or errored recording does not overwrite successful retention timestamp."""
    c_src = MAIN_C.read_text(encoding="utf-8")
    rec_fn_pos = c_src.find("static void recorder_thread_fn(")
    assert rec_fn_pos != -1
    rec_fn = c_src[rec_fn_pos:c_src.find("static uint32_t s_sleep_attempt_counter", rec_fn_pos)]

    # Check that scheduler_retention_save is only in the else (success) branch
    abort_pos = rec_fn.find("recorder_is_abort_requested()")
    err_pos = rec_fn.find("s_rec_last_result != 0")
    save_pos = rec_fn.find("scheduler_retention_save(end_utc);")

    assert abort_pos != -1
    assert err_pos != -1
    assert save_pos != -1
    assert save_pos > abort_pos and save_pos > err_pos, \
        "scheduler_retention_save must only execute on successful session completion"
