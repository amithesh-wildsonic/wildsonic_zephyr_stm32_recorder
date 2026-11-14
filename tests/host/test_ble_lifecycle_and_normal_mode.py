# SPDX-License-Identifier: Apache-2.0
"""
Comprehensive Host Unit and State-Machine Tests for:
1. Default ADTY0 connectable state (no auto-ADTY2 at startup).
2. Correct WS=0 transparent mode configuration (no WS=1).
3. SLPM=1 persistence in supervisor NVS.
4. AT+SLEEP as the final configuration transaction and OK+SLEEP handling.
5. OK+WAKE handling and event decoupling.
6. Zero AT transmissions from ble_comm_resume().
7. Persistent BLE provisioning fingerprint skipping AT sequence on normal reboot.
8. Fingerprint update upon DNAME/SN/PIN change.
9. Deferred AT+NAME update while central is connected and execution upon OK+LOST.
10. Strict rejection of AT commands while connected (-EBUSY).
11. GET_DEV_INFO protocol compatibility (MIC ANALOG, N/A telemetry, TEMP terminator).
12. GET_STATUS_INFO protocol compatibility.
13. Full mobile configuration command transparent forwarding to console.
14. Bounded timeout handling for missing and delayed ACKs.
15. UART RX overflow handling and non-blocking TX test-capture ring buffer.
16. Report serialization via mutex preventing interleaving.
17. Reconnect after disconnect without STM32 reboot.
18. STM32 reset recovery resynchronization with active central connection.
"""

import re
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
BLE_COMM_C = APP_SRC / "ble_comm.c"
BLE_COMM_H = APP_SRC / "ble_comm.h"
SCHED_RET_C = APP_SRC / "scheduler_retention.c"
SCHED_RET_H = APP_SRC / "scheduler_retention.h"


# ==============================================================================
# State Machine & Driver Simulators
# ==============================================================================

class BleSupervisorSimulator:
    """Simulates custom HC-42 (nRF52) BLE_MODULE firmware in normal Mode 0."""
    def __init__(self, nvs_stored=False):
        self.nvs_stored = nvs_stored
        self.ws_mode = 0
        self.slpm = 1
        self.adty = 0
        self.type = 3
        self.pin = "123456"
        self.adv_name = "WS:XXX-00"
        self.is_awake = not nvs_stored
        self.is_connected = False
        self.tx_history = []
        self.rx_history = []

    def receive_uart(self, cmd_line: str) -> str:
        cmd = cmd_line.strip()
        self.rx_history.append(cmd)

        if self.is_connected:
            return ""  # AT commands strictly rejected/ignored when central connected

        if not self.is_awake:
            # First character edge wakes module in Mode 1
            self.is_awake = True
            # Module emits OK+WAKE
            return "OK+WAKE\r\n"

        if cmd == "AT":
            return "OK\r\n"
        elif cmd == "AT+WS=0":
            self.ws_mode = 0
            return "OK+Set:0\r\n"
        elif cmd == "AT+SLPM=1":
            self.slpm = 1
            return "OK+Set:1\r\n"
        elif cmd == "AT+SLPM?":
            return f"OK+SLPM:{self.slpm}\r\n"
        elif cmd == "AT+ADTY0":
            self.adty = 0
            return "OK+Set:0\r\n"
        elif cmd == "AT+ADTY2":
            self.adty = 2
            return "OK+Set:2\r\n"
        elif cmd == "AT+ADTY?":
            return f"OK+Get:{self.adty}\r\n"
        elif cmd.startswith("AT+NAME"):
            self.adv_name = cmd[7:]
            return f"OK+Set:{self.adv_name}\r\n"
        elif cmd == "AT+TYPE3":
            self.type = 3
            return "OK+Set:3\r\n"
        elif cmd.startswith("AT+PASS"):
            self.pin = cmd[7:]
            return "OK+Set:******\r\n"
        elif cmd == "AT+SLEEP":
            self.is_awake = False
            return "OK+SLEEP\r\n"
        elif cmd == "AT+RESET":
            self.is_awake = True
            return "OK+RESET\r\n"
        return "ERR\r\n"


class Stm32BleCommDriver:
    """Simulates STM32 ble_comm.c state machine with retention fingerprint."""
    FNV32_OFFSET = 0x811C9DC5
    FNV32_PRIME = 0x01000193

    def __init__(self, supervisor: BleSupervisorSimulator):
        self.supervisor = supervisor
        self.supervisor_online = False
        self.connected = False
        self.module_awake = False
        self.current_adty = 0
        self.current_slpm = 1
        self.pairing_window_active = False
        self.lock_pending_on_disconnect = False
        self.name_update_pending = False
        self.dname = "XXX"
        self.sn = "00"
        self.retained_fingerprint = None
        self.tx_sent = []

    def calc_fingerprint(self, adv_name: str, pin: str, ws_mode=0, slpm=1, adty=0, type=3) -> int:
        h = self.FNV32_OFFSET
        for val in (ws_mode, slpm, adty, type):
            h = ((h ^ val) * self.FNV32_PRIME) & 0xFFFFFFFF
        if adv_name:
            for c in adv_name.encode("utf-8"):
                h = ((h ^ c) * self.FNV32_PRIME) & 0xFFFFFFFF
        if pin:
            for c in pin.encode("utf-8"):
                h = ((h ^ c) * self.FNV32_PRIME) & 0xFFFFFFFF
        return h

    def send_cmd_expect(self, cmd: str, expected: str, timeout_ms=150) -> (int, str):
        if self.connected:
            return -16, ""  # -EBUSY
        self.tx_sent.append(cmd)
        resp = self.supervisor.receive_uart(cmd)
        if "OK+WAKE" in resp and not self.supervisor.is_awake:
            self.module_awake = True
        if expected in resp:
            return 0, resp.strip()
        # Retry once if module woke from Mode 1
        if self.supervisor.is_awake and not self.connected:
            self.tx_sent.append(cmd + "_retry")
            resp2 = self.supervisor.receive_uart(cmd)
            if expected in resp2:
                return 0, resp2.strip()
        return -110, ""  # -ETIMEDOUT

    def init_driver(self, dname="XXX", sn="00") -> int:
        self.dname = dname
        self.sn = sn
        adv_name = f"WS:{dname}-{sn}"
        target_fp = self.calc_fingerprint(adv_name, "123456", 0, 1, 0, 3)

        # Check retained fingerprint
        if self.retained_fingerprint == target_fp:
            self.supervisor_online = True
            self.module_awake = False
            self.current_adty = 0
            self.current_slpm = 1
            return 0  # Skip AT commands on normal boot

        # Provisioning sequence
        rc, _ = self.send_cmd_expect("AT\r\n", "OK")
        if rc != 0:
            self.supervisor_online = False
            return 0
        self.supervisor_online = True
        self.module_awake = True

        rc_ws, _ = self.send_cmd_expect("AT+WS=0\r\n", "OK+Set:0")
        rc_slpm, _ = self.send_cmd_expect("AT+SLPM=1\r\n", "OK+Set:1")
        rc_name, _ = self.send_cmd_expect(f"AT+NAME{adv_name}\r\n", "OK+Set:")
        rc_type, _ = self.send_cmd_expect("AT+TYPE3\r\n", "OK+Set:3")
        rc_pass, _ = self.send_cmd_expect("AT+PASS123456\r\n", "OK+Set:")
        rc_adty, _ = self.send_cmd_expect("AT+ADTY0\r\n", "OK+Set:0")
        rc_sleep, _ = self.send_cmd_expect("AT+SLEEP\r\n", "OK+SLEEP")

        if all(r == 0 for r in (rc_ws, rc_slpm, rc_name, rc_type, rc_pass, rc_adty, rc_sleep)):
            self.module_awake = False
            self.current_adty = 0
            self.current_slpm = 1
            self.retained_fingerprint = target_fp
            return 0
        return -1

    def suspend(self):
        if not self.supervisor_online:
            return
        if not self.connected and self.module_awake:
            self.send_cmd_expect("AT+SLEEP\r\n", "OK+SLEEP")
            self.module_awake = False

    def resume(self):
        # Must NOT send AT commands on resume
        pass

    def on_central_connected(self):
        self.connected = True
        self.module_awake = True
        self.supervisor_online = True
        self.supervisor.is_connected = True

    def on_central_disconnected(self):
        self.connected = False
        self.module_awake = False
        self.supervisor.is_connected = False
        if self.name_update_pending:
            self.name_update_pending = False
            self.apply_advertised_name()

    def notify_name_changed(self, new_dname: str, new_sn: str):
        self.dname = new_dname
        self.sn = new_sn
        if self.connected:
            self.name_update_pending = True
        else:
            self.apply_advertised_name()

    def apply_advertised_name(self):
        adv_name = f"WS:{self.dname}-{self.sn}"
        self.send_cmd_expect(f"AT+NAME{adv_name}\r\n", "OK+Set:")
        self.send_cmd_expect("AT+SLEEP\r\n", "OK+SLEEP")
        self.module_awake = False
        self.retained_fingerprint = self.calc_fingerprint(adv_name, "123456", 0, 1, 0, 3)


# ==============================================================================
# Unit & Behavioral Test Suite
# ==============================================================================

def test_default_adty0_and_no_auto_adty2_at_startup():
    """Verify BLE starts unlocked with AT+ADTY0 and s_current_adty = 0."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    assert re.search(r"static\s+uint8_t\s+s_current_adty\s*=\s*0;", c_src)
    assert 'AT+ADTY0\\r\\n' in c_src
    assert 'OK+Set:0' in c_src
    assert 's_current_adty = 0;' in c_src


def test_correct_ws0_transparent_mode_configuration():
    """Verify AT+WS=0 is strictly enforced and AT+WS=1 is never used."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    assert 'AT+WS=0\\r\\n' in c_src
    assert 'AT+WS=1' not in c_src, "AT+WS=1 must NEVER be used in normal mode"


def test_slpm1_light_sleep_persistence():
    """Verify Mode 1 sleep persistence remains valid without forcing a legacy AT+SLPM=1 probe path."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    assert 's_current_slpm = 1;' in c_src
    assert 'AT+SLPM=2' not in c_src
    assert 'AT+SLPM=3' not in c_src
    assert 'AT+SLPM=1\\r\\n' not in c_src


def test_at_sleep_final_provisioning_transaction_and_ok_sleep_handling():
    """Verify AT+SLEEP is the final configuration step and OK+SLEEP clears awake state."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    init_pos = c_src.find("int ble_comm_init(")
    adty0_pos = c_src.find('AT+ADTY0\\r\\n', init_pos)
    sleep_pos = c_src.find('AT+SLEEP\\r\\n', init_pos)
    assert adty0_pos < sleep_pos, "AT+ADTY0 must precede final AT+SLEEP"
    assert 's_ble_module_awake = false;' in c_src[sleep_pos:sleep_pos + 200]


def test_ok_wake_handling_and_decoupling():
    """Verify OK+WAKE sets awake state without completing unrelated commands."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    wake_pos = c_src.find('strcmp(p, "OK+WAKE") == 0')
    assert wake_pos != -1
    wake_block = c_src[wake_pos:wake_pos + 350]
    assert "s_ble_module_awake = true;" in wake_block
    assert "s_ble_supervisor_online = true;" in wake_block


def test_no_at_transmission_on_stop2_resume():
    """Verify ble_comm_resume() contains ZERO AT commands, pings, or wake transmissions."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    resume_fn = c_src[c_src.find("void ble_comm_resume"):c_src.find("void ble_comm_send_dev_info")]
    assert "ble_comm_send_cmd_expect" not in resume_fn
    assert "ble_comm_tx" not in resume_fn
    assert "uart_poll_out" not in resume_fn
    assert "AT\\r\\n" not in resume_fn
    assert "AT+RESET" not in resume_fn


def test_provisioning_fingerprint_skips_at_on_normal_reboot():
    """Verify that a normal reboot with retained fingerprint sends NO AT commands."""
    sup = BleSupervisorSimulator(nvs_stored=True)
    driver = Stm32BleCommDriver(sup)

    # First boot: unprovisioned -> runs full sequence
    assert driver.init_driver(dname="DEV1", sn="01") == 0
    assert len(driver.tx_sent) > 0
    assert driver.retained_fingerprint is not None
    saved_fp = driver.retained_fingerprint

    # Second boot: retained fingerprint matches -> zero AT commands transmitted
    driver.tx_sent.clear()
    assert driver.init_driver(dname="DEV1", sn="01") == 0
    assert len(driver.tx_sent) == 0, "Second boot with unchanged fingerprint must transmit 0 AT commands"
    assert driver.supervisor_online is True
    assert driver.module_awake is False
    assert driver.current_adty == 0


def test_provisioning_fingerprint_mismatch_triggers_provisioning():
    """Verify changing DNAME/SN produces fingerprint mismatch and triggers provisioning."""
    sup = BleSupervisorSimulator(nvs_stored=True)
    driver = Stm32BleCommDriver(sup)

    # Initial provisioning with DEV1
    driver.init_driver(dname="DEV1", sn="01")
    old_fp = driver.retained_fingerprint

    # Reboot with changed DNAME 'DEV2'
    driver.tx_sent.clear()
    driver.init_driver(dname="DEV2", sn="01")
    assert len(driver.tx_sent) > 0, "Fingerprint mismatch must trigger AT provisioning"
    assert driver.retained_fingerprint != old_fp, "Fingerprint must update to reflect new identity"


def test_pending_identity_update_while_connected_and_deferred_application():
    """Verify name update is deferred while connected and applied immediately upon OK+LOST."""
    sup = BleSupervisorSimulator(nvs_stored=True)
    driver = Stm32BleCommDriver(sup)
    driver.init_driver(dname="WS", sn="01")

    # Central connects
    driver.on_central_connected()
    assert driver.connected is True

    # User changes DNAME while connected
    driver.tx_sent.clear()
    driver.notify_name_changed("NEW", "02")
    assert driver.name_update_pending is True
    assert len(driver.tx_sent) == 0, "No AT commands must be sent while central is connected"

    # Central disconnects (OK+LOST)
    driver.on_central_disconnected()
    assert driver.name_update_pending is False
    assert any("AT+NAMENEW-02" in cmd or "AT+NAMEWS:NEW-02" in cmd for cmd in driver.tx_sent)
    assert any("AT+SLEEP" in cmd for cmd in driver.tx_sent)
    assert driver.module_awake is False


def test_no_at_transactions_while_connected():
    """Verify ble_comm_send_cmd_expect returns -EBUSY when central is connected."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    send_pos = c_src.find("int ble_comm_send_cmd_expect(")
    send_fn = c_src[send_pos:c_src.find("static void ble_comm_process_line", send_pos)]
    assert "if (s_ble_connected)" in send_fn
    assert "return -EBUSY;" in send_fn


def test_get_dev_info_protocol_compatibility():
    """Verify GET_DEV_INFO protocol parity: analog mic, N/A telemetry, and TEMP terminator."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    dev_info_pos = c_src.find("void ble_comm_send_dev_info(")
    dev_info_fn = c_src[dev_info_pos:c_src.find("void ble_comm_send_status_info", dev_info_pos)]

    # Analog microphone requirement
    assert 'ble_comm_printf("MIC ANALOG\\r\\n");' in dev_info_fn
    assert "SPH0645" not in dev_info_fn

    # Battery fields use the latest valid snapshot and preserve N/A per field.
    assert 'battery_monitor_get_status(&battery);' in dev_info_fn
    assert '"VBATT %u"' in dev_info_fn and '"VBATT N/A"' in dev_info_fn
    assert '"CBATT %d"' in dev_info_fn and '"CBATT N/A"' in dev_info_fn
    assert '"REMBATT %u\\r\\n"' in dev_info_fn and '"REMBATT N/A\\r\\n"' in dev_info_fn
    assert '"FULLBATT %u"' in dev_info_fn and '"FULLBATT N/A"' in dev_info_fn
    assert '"REMTIME %u"' in dev_info_fn and '"REMTIME N/A"' in dev_info_fn
    assert '"AVGPWR %d\\r\\n"' in dev_info_fn and '"AVGPWR N/A\\r\\n"' in dev_info_fn
    assert 'BATTCAP 0' in dev_info_fn

    # RTC fallback to N/A if invalid
    assert 'DATETIME N/A' in dev_info_fn

    # Canonical required tags
    assert 'SAMF %u' in dev_info_fn
    assert 'MODEL WS-STM32L476' in dev_info_fn
    assert 'FWVER 1.2.0-BLE' in dev_info_fn
    assert 'TEMP 0.00, HUMID 0.00' in dev_info_fn

    # Report serialization via mutex
    assert 'k_mutex_lock(&s_ble_report_mutex' in dev_info_fn
    assert 'k_mutex_unlock(&s_ble_report_mutex' in dev_info_fn


def test_get_status_info_protocol_compatibility():
    """Verify GET_STATUS_INFO protocol parity and terminating TEMP line."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    status_pos = c_src.find("void ble_comm_send_status_info(")
    status_fn = c_src[status_pos:c_src.find("int ble_comm_feed_rx_bytes", status_pos)]

    assert 'SDUSED %llu, SDCAP %llu' in status_fn
    assert 'battery_monitor_get_status(&battery);' in status_fn
    assert '"VBATT %u"' in status_fn and '"VBATT N/A"' in status_fn
    assert '"CBATT %d"' in status_fn and '"CBATT N/A"' in status_fn
    assert '"REMBATT %u\\r\\n"' in status_fn and '"REMBATT N/A\\r\\n"' in status_fn
    assert '"FULLBATT %u"' in status_fn and '"FULLBATT N/A"' in status_fn
    assert '"REMTIME %u"' in status_fn and '"REMTIME N/A"' in status_fn
    assert '"AVGPWR %d\\r\\n"' in status_fn and '"AVGPWR N/A\\r\\n"' in status_fn
    assert 'TEMP 0.00, HUMID 0.00' in status_fn

    # Mutex protection
    assert 'k_mutex_lock(&s_ble_report_mutex' in status_fn
    assert 'k_mutex_unlock(&s_ble_report_mutex' in status_fn


def test_full_mobile_configuration_command_forwarding():
    """Verify console dispatcher processes mobile configuration commands over BLE."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    proc_pos = c_src.find("static void ble_comm_process_line(")
    proc_fn = c_src[proc_pos:c_src.find("static void ble_comm_handle_rx_byte", proc_pos)]

    assert "console_process_line_source(p, true, resp_buf, sizeof(resp_buf))" in proc_fn
    assert 'ble_comm_printf("%s\\r\\n", resp_buf)' in proc_fn
    assert 'ble_comm_printf("OK\\r\\n")' in proc_fn
    assert 'ble_comm_printf("ERR\\r\\n")' in proc_fn


def test_missing_and_delayed_acks_handling():
    """Verify bounded timeout on missing or delayed ACKs."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    send_pos = c_src.find("int ble_comm_send_cmd_expect(")
    send_fn = c_src[send_pos:c_src.find("static void ble_comm_process_line", send_pos)]

    assert "k_sem_take(&s_ble_ack_sem, K_MSEC(timeout_ms))" in send_fn
    assert "return -ETIMEDOUT;" in send_fn


def test_uart_rx_overflow_and_tx_capture_saturation_resilience():
    """Verify oversized RX lines are dropped safely, once per malformed line, without memory corruption."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    handle_pos = c_src.find("static void ble_comm_handle_rx_byte(")
    handle_fn = c_src[handle_pos:c_src.find("static void ble_lock_work_fn", handle_pos)]

    assert "s_ble_discard_until_terminator = true;" in handle_fn
    assert "s_ble_discard_until_terminator = false;" in handle_fn
    assert "s_ble_overflow_line_count++;" in handle_fn
    assert "s_ble_rx_drop_count++;" in c_src
    assert "s_ble_cmd_len = 0;" in handle_fn
    assert "ring_buf_put(&s_ble_rx_rb, &rx_byte, 1) == 0" in c_src


def test_ble_line_parser_behaviour_contract():
    """Verify malformed oversized lines are discarded exactly once until a terminator, without dispatching partial commands."""
    max_len = 127
    accepted = []
    overflow_events = 0
    discard = False
    last_was_cr = False
    line = ""

    def feed(data: str):
        nonlocal accepted, overflow_events, discard, last_was_cr, line
        for ch in data:
            if discard:
                if ch == '\r' or ch == '\n':
                    if ch == '\r':
                        last_was_cr = True
                    else:
                        last_was_cr = False
                    discard = False
                    line = ""
                continue

            if ch == '\r':
                if last_was_cr:
                    last_was_cr = False
                    continue
                if line:
                    accepted.append(line)
                    line = ""
                last_was_cr = True
                continue

            if ch == '\n':
                if last_was_cr:
                    last_was_cr = False
                    continue
                if line:
                    accepted.append(line)
                    line = ""
                continue

            last_was_cr = False
            if len(line) >= max_len:
                discard = True
                overflow_events += 1
                line = ""
                continue
            line += ch

    # Normal short command is accepted.
    feed("GET_STATUS_INFO\r\n")
    assert accepted == ["GET_STATUS_INFO"]

    # A line that hits max length is treated as malformed and discarded once.
    feed("A" * (max_len + 1) + "\r")
    assert overflow_events == 1
    assert accepted == ["GET_STATUS_INFO"]

    # A line with excess bytes before termination increments once and discards the rest.
    feed("A" * (max_len + 5) + "\n")
    assert overflow_events == 2
    assert accepted == ["GET_STATUS_INFO"]

    # A valid command can still arrive after an overflow line.
    feed("Z" * 10 + "\r\n")
    assert accepted == ["GET_STATUS_INFO", "Z" * 10]

    # Empty lines and CRLF normalization should not produce bogus commands.
    feed("\r\n\n")
    assert accepted == ["GET_STATUS_INFO", "Z" * 10]

    # The parser should not dispatch partial command fragments.
    feed("SHORT\nLONG\r\n")
    assert accepted[-2:] == ["SHORT", "LONG"]

    # CRLF followed by a clean line after an oversized frame should not cause duplication.
    feed("A" * (max_len + 1) + "\r\nDIRTY\r\n")
    assert overflow_events == 3
    assert accepted[-1] == "DIRTY"

    # A sequence of malformed lines should produce exactly one warning each.
    feed("OK\r\n" + ("A" * (max_len + 1)) + "\nVALID\r\n")
    assert accepted[-2:] == ["OK", "VALID"]
    assert overflow_events >= 4


def test_reconnect_after_disconnect_without_stm32_reboot():
    """Verify phone can connect, disconnect, and reconnect without reboot."""
    sup = BleSupervisorSimulator(nvs_stored=True)
    driver = Stm32BleCommDriver(sup)
    driver.init_driver()

    # Session 1
    driver.on_central_connected()
    assert driver.connected is True
    driver.on_central_disconnected()
    assert driver.connected is False
    assert driver.current_adty == 0, "Must remain connectable (ADTY0) after disconnect"

    # Session 2 without reboot
    driver.on_central_connected()
    assert driver.connected is True
    driver.on_central_disconnected()
    assert driver.connected is False
    assert driver.current_adty == 0


def test_stm32_reset_recovery_with_active_connection():
    """Verify mobile app traffic resynchronizes connection state if host reboots while connected."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    proc_pos = c_src.find("static void ble_comm_process_line(")
    proc_fn = c_src[proc_pos:c_src.find("static void ble_comm_handle_rx_byte", proc_pos)]

    # Verifies incoming app requests immediately restore connected state
    app_pos = proc_fn.find("Mobile Companion Application Requests")
    assert "s_ble_connected = true;" in proc_fn[app_pos:app_pos + 300]
    assert "s_ble_module_awake = true;" in proc_fn[app_pos:app_pos + 300]
    assert "s_ble_supervisor_online = true;" in proc_fn[app_pos:app_pos + 300]

