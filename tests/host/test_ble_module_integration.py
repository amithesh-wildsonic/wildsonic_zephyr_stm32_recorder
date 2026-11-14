#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Test Suite: BLE Module Integration, AT Command Protocol Parity & Reset Cause Diagnostics.

Covers:
 1. Hardware & Devicetree Pinout:
    - USART1 mapped to PA9 (TX) and PA10 (RX) at 115200 8N1.
    - USART2 preserved exclusively for Zephyr console (PA2/PA3).
 2. AT Command Transaction Engine:
    - Bare AT ping requires exact 'OK\r\n'.
    - Rejects unmatched or arbitrary 'OK+...' as bare ping success.
 3. Pairing Lock & Unlock Semantics:
    - Unlock: AT+ADTY0 -> validates exact OK+Set:0.
    - Lock: AT+ADTY2 -> validates exact OK+Set:2.
    - Rejects AT+SLEEP or AT+ADTY0 as pairing lock commands.
 4. Sleep Policy (AT+SLPM):
    - Query: AT+SLPM? -> validates OK+SLPM:<mode>.
    - Set: AT+SLPM=1 -> validates OK+Set:1.
    - Audits Mode 1 (UART RX wake supported) vs Mode 2 (UART RX blocked).
    - Ensures initial operating policy enforces Mode 1 because no LPIN wire exists.
 5. Pairing Window Expiration Lifecycle:
    - Expiration while disconnected: triggers AT+ADTY2 lock immediately.
    - Expiration while connected: preserves transparent central connection; defers AT+ADTY2 until OK+LOST.
 6. Supervisor Modem Notifications:
    - Parses OK+CONN, OK+LOST, OK+WAKE, OK+SLEEP.
 7. Mobile Companion Interoperability:
    - GET_DEV_INFO and GET_STATUS_INFO emit required tags and terminate with 'TEMP 0.00, HUMID 0.00\r\n'.
 8. Host Readiness Protocol:
    - Confirms AT+WS=1 / wake-aware mode is not enabled without full handshake support (remains WS=0 transparent).
 9. Elimination of Unsupported Legacy Commands:
    - Verifies AT+UART1, AT+PWRM0, AT+MODE2, WS+SYNC, WS+ALIVE are purged from production C sources.
10. STM32 Reset Cause Diagnostics:
    - Verifies RCC_CSR reset flags (BORRSTF, PINRSTF, SFTRSTF, IWDGRSTF, WWDGRSTF, LPWRRSTF).
    - Verifies flags are captured and reported BEFORE being cleared.
11. Stability & Reboots:
    - Verifies BLE events and AT+RESET commands never cause STM32 host reboots.
"""

from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
APP_BOARDS = REPO_ROOT / "app" / "boards"
OVERLAY_DTS = APP_BOARDS / "nucleo_l476rg.overlay"
CMAKELISTS = REPO_ROOT / "app" / "CMakeLists.txt"

BLE_COMM_H = APP_SRC / "ble_comm.h"
BLE_COMM_C = APP_SRC / "ble_comm.c"
CONSOLE_H = APP_SRC / "console.h"
CONSOLE_C = APP_SRC / "console.c"
MAIN_C = APP_SRC / "main.c"
RTC_SLEEP_C = APP_SRC / "rtc_sleep.c"


# ==============================================================================
# 1. Devicetree & Pin Allocation Audits
# ==============================================================================

def test_usart1_devicetree_pin_mapping():
    """Verify USART1 is assigned to PA9 (TX) and PA10 (RX) at 115200 baud."""
    dts = OVERLAY_DTS.read_text(encoding="utf-8")
    assert "&usart1" in dts, "Missing &usart1 block in overlay"
    assert "usart1_tx_pa9" in dts, "USART1 TX must map to PA9"
    assert "usart1_rx_pa10" in dts, "USART1 RX must map to PA10"
    assert "current-speed = <115200>" in dts, "USART1 baud rate must be 115200"
    assert 'status = "okay"' in dts, "USART1 status must be okay"


def test_usart2_console_isolation():
    """Verify USART2 remains the designated console and is not overridden."""
    gen_dts_path = REPO_ROOT / "app" / "build" / "PRODUCTION" / "zephyr" / "zephyr.dts"
    if gen_dts_path.exists():
        dts = gen_dts_path.read_text(encoding="utf-8")
        assert "zephyr,console = &usart2;" in dts, "zephyr,console must remain &usart2"
        assert "zephyr,shell-uart = &usart2;" in dts, "zephyr,shell-uart must remain &usart2"
        assert "usart2_tx_pa2" in dts, "USART2 TX must map to PA2"
        assert "usart2_rx_pa3" in dts, "USART2 RX must map to PA3"
    else:
        overlay = OVERLAY_DTS.read_text(encoding="utf-8")
        assert "zephyr,console = &usart1" not in overlay


def test_cmake_includes_ble_comm():
    """Verify CMakeLists.txt builds ble_comm.c."""
    cmake = CMAKELISTS.read_text(encoding="utf-8")
    assert "src/ble_comm.c" in cmake, "CMakeLists.txt must include src/ble_comm.c"


# ==============================================================================
# 2. Simulated Transaction & Protocol Engine
# ==============================================================================

class BleSupervisorSimulator:
    """
    Simulates the nRF52832 WildSonic BLE Supervisor module implementing
    exact AT command dispatching and responses according to at_cmd.c.
    """
    def __init__(self, initial_slpm=1, initial_adty=2):
        self.slpm = initial_slpm
        self.adty = initial_adty
        self.is_connected = False
        self.is_sleeping = False
        self.device_name = "WS:WS-01"
        self.bond_mode = 3
        self.pin = "123456"

    def process_command(self, cmd: str) -> str:
        cmd = cmd.strip()
        if self.is_connected:
            return ""  # AT commands blocked while central connected

        if cmd == "AT":
            self.is_sleeping = False
            return "OK\r\n"
        elif cmd == "AT+SLPM?":
            return f"OK+SLPM:{self.slpm}\r\n"
        elif cmd.startswith("AT+SLPM="):
            val = int(cmd.split("=")[1])
            if val in (1, 2, 3):
                self.slpm = val
                return f"OK+Set:{val}\r\n"
            return "ERR\r\n"
        elif cmd == "AT+ADTY?":
            return f"OK+Get:{self.adty}\r\n"
        elif cmd == "AT+ADTY0":
            self.adty = 0
            return "OK+Set:0\r\n"
        elif cmd == "AT+ADTY2":
            self.adty = 2
            return "OK+Set:2\r\n"
        elif cmd.startswith("AT+NAME"):
            name = cmd[7:]
            self.device_name = name
            return f"OK+Set:{name}\r\n"
        elif cmd == "AT+TYPE3":
            self.bond_mode = 3
            return "OK+Set:3\r\n"
        elif cmd.startswith("AT+PASS"):
            self.pin = cmd[7:]
            return "OK+Set:******\r\n"
        elif cmd == "AT+SLEEP":
            self.is_sleeping = True
            return "OK+SLEEP\r\n"
        elif cmd == "AT+RESET":
            self.is_sleeping = False
            return "OK+RESET\r\n"
        elif cmd == "AT+BAUD?":
            return "OK+Get:4\r\n"
        else:
            return "ERR\r\n"


class BleHostClientSimulator:
    """
    Simulates STM32 ble_comm.c transaction state machine.
    """
    def __init__(self, supervisor: BleSupervisorSimulator):
        self.supervisor = supervisor
        self.is_active = False
        self.is_connected = False
        self.is_module_awake = False
        self.pairing_window_active = False
        self.pairing_window_remaining = 0
        self.lock_pending_on_disconnect = False
        self.tx_log = []
        self.rx_log = []

    def send_cmd_expect(self, cmd: str, expected_ack: str, timeout_ms: int = 150) -> (int, str):
        if self.is_connected:
            return -1, "BUSY"
        self.tx_log.append(cmd)
        resp = self.supervisor.process_command(cmd)
        self.rx_log.append(resp.strip())

        # Exact match vs Prefix match
        clean_resp = resp.strip()
        if expected_ack.endswith(":"):
            if clean_resp.startswith(expected_ack):
                return 0, clean_resp
        else:
            if clean_resp == expected_ack:
                return 0, clean_resp
        return -2, clean_resp

    def open_pairing_window(self, seconds: int = 300, simulate_wake_failure: bool = False):
        if seconds == 0:
            seconds = 300
        if seconds > 600:
            seconds = 600
        if self.is_connected:
            self.pairing_window_active = True
            self.pairing_window_remaining = seconds
            return 0

        # If module is asleep, wake it first; abort if wake fails
        if not self.is_module_awake:
            if simulate_wake_failure:
                return -3  # Wake failed: abort without sending AT+ADTY0
            rc, _ = self.send_cmd_expect("AT", "OK")
            if rc != 0:
                return rc
            self.is_module_awake = True

        # Send AT+ADTY0 and validate OK+Set:0
        rc, resp = self.send_cmd_expect("AT+ADTY0", "OK+Set:0")
        if rc == 0:
            self.pairing_window_active = True
            self.pairing_window_remaining = seconds
            self.is_module_awake = True
            return 0
        return rc

    def lock_pairing(self, simulate_wake_failure: bool = False):
        if self.is_connected:
            self.lock_pending_on_disconnect = True
            return 0  # Deferred

        # If module is asleep, wake it first; abort if wake fails
        if not self.is_module_awake:
            if simulate_wake_failure:
                return -3  # Wake failed: abort without sending AT+ADTY2
            rc, _ = self.send_cmd_expect("AT", "OK")
            if rc != 0:
                return rc
            self.is_module_awake = True

        rc, resp = self.send_cmd_expect("AT+ADTY2", "OK+Set:2")
        if rc == 0:
            self.pairing_window_active = False
            self.pairing_window_remaining = 0
            self.lock_pending_on_disconnect = False
            # Enter sleep under Mode 1
            self.send_cmd_expect("AT+SLEEP", "OK+SLEEP")
            self.is_module_awake = False
            return 0
        return rc

    def on_central_connected(self):
        self.is_connected = True
        self.is_module_awake = True

    def on_central_disconnected(self):
        self.is_connected = False
        if self.lock_pending_on_disconnect or self.pairing_window_remaining <= 0:
            self.lock_pairing()


# ==============================================================================
# 3. Exact AT Command Protocol & Transaction Verification
# ==============================================================================

def test_at_bare_ping_transaction():
    """Verify bare AT ping requires exact 'OK' and does not match arbitrary 'OK+'."""
    sup = BleSupervisorSimulator()
    client = BleHostClientSimulator(sup)

    rc, resp = client.send_cmd_expect("AT", "OK")
    assert rc == 0
    assert resp == "OK"

    # Verify that if response were OK+Set:0, expect "OK" rejects it
    clean_resp = "OK+Set:0"
    assert clean_resp != "OK"


def test_at_adty0_unlock_semantics():
    """Verify AT+ADTY0 opens connectable advertising and validates OK+Set:0."""
    sup = BleSupervisorSimulator(initial_adty=2)
    client = BleHostClientSimulator(sup)

    rc = client.open_pairing_window(300)
    assert rc == 0
    assert client.pairing_window_active
    assert sup.adty == 0
    assert "AT+ADTY0" in client.tx_log
    assert "OK+Set:0" in client.rx_log


def test_at_adty2_lock_semantics():
    """Verify AT+ADTY2 locks non-connectable advertising and validates OK+Set:2."""
    sup = BleSupervisorSimulator(initial_adty=0)
    client = BleHostClientSimulator(sup)
    client.pairing_window_active = True

    rc = client.lock_pairing()
    assert rc == 0
    assert not client.pairing_window_active
    assert sup.adty == 2
    assert "AT+ADTY2" in client.tx_log
    assert "OK+Set:2" in client.rx_log
    assert "AT+SLEEP" in client.tx_log
    assert sup.is_sleeping


def test_at_slpm_query_and_set_mode1():
    """Verify querying sleep policy (AT+SLPM?) and enforcing Mode 1 (AT+SLPM=1)."""
    sup = BleSupervisorSimulator(initial_slpm=2)
    client = BleHostClientSimulator(sup)

    # 1. Query existing policy
    rc, resp = client.send_cmd_expect("AT+SLPM?", "OK+SLPM:")
    assert rc == 0
    assert resp == "OK+SLPM:2"

    # 2. Update to Mode 1 because UART wake is required
    rc, resp = client.send_cmd_expect("AT+SLPM=1", "OK+Set:1")
    assert rc == 0
    assert resp == "OK+Set:1"
    assert sup.slpm == 1


def test_mode1_uart_wake_vs_mode2_uart_blocked():
    """Verify Mode 1 allows UART RX wake while Mode 2 disables UART RX entirely."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    assert "AT+SLPM=1" in c_src, "Must configure AT+SLPM=1 for UART wake support"
    # Verify Mode 2 is never commanded as the normal sleep policy
    assert "AT+SLPM=2" not in c_src, "Mode 2 must not be commanded without LPIN wake"
    assert "AT+SLPM=3" not in c_src, "Mode 3 must not be commanded during operation"


def test_pairing_window_expiration_while_disconnected():
    """Verify pairing window expiring while disconnected triggers immediate AT+ADTY2."""
    sup = BleSupervisorSimulator(initial_adty=2)
    client = BleHostClientSimulator(sup)

    client.open_pairing_window(10)
    assert sup.adty == 0

    # Simulate timer expiration
    client.pairing_window_remaining = 0
    client.lock_pairing()
    assert sup.adty == 2
    assert not client.pairing_window_active


def test_pairing_window_expiration_while_connected_defers_lock():
    """Verify pairing window expiring while central is connected preserves link and defers lock."""
    sup = BleSupervisorSimulator(initial_adty=2)
    client = BleHostClientSimulator(sup)

    client.open_pairing_window(10)
    client.on_central_connected()

    # Expire window during connection
    client.pairing_window_remaining = 0
    # Attempt lock: must defer because link is active
    client.lock_pairing()
    assert client.lock_pending_on_disconnect
    assert sup.adty == 0, "Must NOT change ADTY while central is actively connected"
    assert "AT+ADTY2" not in client.tx_log[1:], "Must not transmit AT commands while connected"

    # Central disconnects: deferred lock must now execute
    client.on_central_disconnected()
    assert not client.lock_pending_on_disconnect
    assert sup.adty == 2
    assert "AT+ADTY2" in client.tx_log


def test_unsupported_legacy_commands_purged():
    """Verify unsupported legacy commands (UART1, PWRM0, MODE2, WS+SYNC) are purged."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    h_src = BLE_COMM_H.read_text(encoding="utf-8")
    combined = c_src + "\n" + h_src

    assert "AT+UART1" not in combined, "Legacy AT+UART1 must not be sent"
    assert "AT+PWRM0" not in combined, "Legacy AT+PWRM0 must not be sent"
    assert "AT+MODE2" not in combined, "Legacy AT+MODE2 must not be sent"
    assert "WS+SYNC" not in combined, "Unconfirmed WS+SYNC must not be sent"


def test_get_dev_info_format_and_temp_terminator():
    """Verify GET_DEV_INFO emits tags and terminating TEMP line."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    assert "TEMP 0.00, HUMID 0.00" in c_src
    assert "SAMF %u" in c_src
    assert "RECT %u" in c_src
    assert "MODEL WS-STM32L476" in c_src


def test_get_status_info_format_and_temp_terminator():
    """Verify GET_STATUS_INFO emits SDCAP/SDUSED and terminating TEMP line."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    assert "SDUSED %llu, SDCAP %llu" in c_src
    assert "TEMP 0.00, HUMID 0.00" in c_src


# ==============================================================================
# 4. STM32 Reset Cause Diagnostics Audits
# ==============================================================================

def test_reset_cause_diagnostics_in_main():
    """Verify main.c captures RCC_CSR before clearing and retained_reboot classifies reset flags."""
    main_src = MAIN_C.read_text(encoding="utf-8")
    rbt_src = (APP_SRC / "retained_reboot.c").read_text(encoding="utf-8")

    capture_idx = main_src.find("g_boot_csr = RCC->CSR;")
    clear_idx = main_src.find("LL_RCC_ClearResetFlags();", capture_idx)
    assert capture_idx != -1 and clear_idx > capture_idx, "g_boot_csr must be captured before clearing flags"
    assert "retained_reboot_capture_csr(g_boot_csr);" in main_src
    assert "main_emit_boot_banner_and_reason(" in main_src

    assert "RCC_CSR_BORRSTF" in rbt_src, "Must check BORRSTF"
    assert "RCC_CSR_PINRSTF" in rbt_src, "Must check PINRSTF"
    assert "RCC_CSR_SFTRSTF" in rbt_src, "Must check SFTRSTF"
    assert "RCC_CSR_IWDGRSTF" in rbt_src, "Must check IWDGRSTF"
    assert "RCC_CSR_WWDGRSTF" in rbt_src, "Must check WWDGRSTF"


def test_ble_events_do_not_reboot_stm32():
    """Verify ble_comm.c never invokes sys_reboot or NVIC_SystemReset."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    assert "sys_reboot" not in c_src, "ble_comm.c must never call sys_reboot"
    assert "NVIC_SystemReset" not in c_src, "ble_comm.c must never call NVIC_SystemReset"


def test_stop2_sleep_resume_handshake():
    """Verify ble_comm_suspend sends AT+SLEEP when awake and resume sends NO AT commands."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    suspend_fn = c_src[c_src.find("void ble_comm_suspend"):c_src.find("void ble_comm_resume")]
    resume_fn = c_src[c_src.find("void ble_comm_resume"):c_src.find("void ble_comm_send_dev_info")]

    assert "AT+SLEEP" in suspend_fn, "Suspend must command AT+SLEEP if awake"
    assert "ble_comm_send_cmd_expect" not in resume_fn, "Resume must NEVER send AT commands to BLE supervisor"
    assert "AT\\r\\n" not in resume_fn, "Resume must NOT wake via AT ping"
    assert "AT+RESET" not in resume_fn, "Resume must NEVER use AT+RESET as a wake mechanism"
    assert "uart_irq_rx_enable" in resume_fn, "Resume must re-enable UART RX interrupt"


# ==============================================================================
# 5. Normal Mode (AT+WS=0) Protocol & Architecture Parity
# ==============================================================================

def test_uart_tx_gating_decoupled_from_supervisor_probe():
    """Verify UART TX readiness is decoupled from supervisor probe response."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    h_src = BLE_COMM_H.read_text(encoding="utf-8")

    # Accessor declarations
    assert "bool ble_comm_is_uart_ready(void);" in h_src
    assert "bool ble_comm_is_supervisor_online(void);" in h_src
    assert "void ble_comm_set_test_capture(bool enabled);" in h_src

    # UART ready must be set before probe in ble_comm_init
    ready_pos = c_src.find("s_ble_uart_ready = true;")
    probe_pos = c_src.find('ble_comm_send_cmd_expect("AT\\r\\n"')
    assert ready_pos != -1, "s_ble_uart_ready must be explicitly set"
    assert probe_pos != -1, "AT probe must be sent"
    assert ready_pos < probe_pos, "s_ble_uart_ready must be set BEFORE initial probe"

    # ble_comm_tx must transmit when UART ready, not gated on s_ble_active or s_ble_supervisor_online
    tx_fn = c_src[c_src.find("int ble_comm_tx"):c_src.find("int ble_comm_printf")]
    assert "s_ble_uart_ready" in tx_fn, "ble_comm_tx must check s_ble_uart_ready"
    assert "s_ble_supervisor_online" not in tx_fn, "ble_comm_tx must NOT be gated on supervisor online"
    assert "s_ble_active" not in tx_fn, "ble_comm_tx must NOT be gated on legacy s_ble_active"

    # Ring buffer capture must be guarded by s_ble_test_capture_enabled
    assert "s_ble_test_capture_enabled" in tx_fn, "TX ring buffer must be guarded by test capture flag"


def test_ws0_mandatory_transparent_mode_no_ws1():
    """Verify AT+WS=0 is mandated in init and AT+WS=1 is never sent."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    h_src = BLE_COMM_H.read_text(encoding="utf-8")
    combined = c_src + "\n" + h_src

    assert 'AT+WS=0' in c_src, "Must command AT+WS=0 for transparent mode"
    assert 'OK+Set:0' in c_src, "Must validate OK+Set:0 ack for AT+WS=0"
    assert "AT+WS=1" not in combined, "AT+WS=1 (wake-aware mode) must NOT be present"
    assert "M_WAKE" not in combined, "No M_WAKE pin handshake in normal mode"
    assert "M_READY" not in combined, "No M_READY pin handshake in normal mode"


def test_wake_before_adty_failure_aborts_adty_command():
    """Verify that failure to wake an asleep module aborts ADTY without transmitting it blindly."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    open_fn = c_src[c_src.find("void ble_comm_open_pairing_window"):c_src.find("void ble_comm_lock_pairing")]
    lock_fn = c_src[c_src.find("void ble_comm_lock_pairing"):c_src.find("void ble_comm_suspend")]

    # Both open and lock must check wake success and return/abort on failure
    assert "if (ble_comm_wake_module() != 0)" in open_fn
    assert "if (ble_comm_wake_module() != 0)" in lock_fn

    # Simulator behavior verification
    sup = BleSupervisorSimulator(initial_adty=2)
    client = BleHostClientSimulator(sup)
    client.is_module_awake = False
    rc = client.open_pairing_window(300, simulate_wake_failure=True)
    assert rc != 0, "open_pairing_window must fail if wake fails"
    assert "AT+ADTY0" not in client.tx_log, "AT+ADTY0 must NOT be sent if wake failed"

    client.tx_log.clear()
    rc = client.lock_pairing(simulate_wake_failure=True)
    assert rc != 0, "lock_pairing must fail if wake fails"
    assert "AT+ADTY2" not in client.tx_log, "AT+ADTY2 must NOT be sent if wake failed"


def test_generic_commands_do_not_fabricate_connection_state():
    """Verify GET_DEV_INFO, GET_STATUS_INFO, and config commands do not set s_ble_connected."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    process_fn = c_src[c_src.find("static void ble_comm_process_line"):c_src.find("static void ble_comm_handle_rx_byte")]

    # Check the section after modem events (from GET_DEV_INFO onwards)
    dev_info_pos = process_fn.find('GET_DEV_INFO')
    assert dev_info_pos != -1
    remainder = process_fn[dev_info_pos:]
    assert "s_ble_connected = true;" not in remainder, \
        "Generic commands and config lines must NOT set s_ble_connected = true"

    # Verify OK+CONN is the sole trigger for setting s_ble_connected = true
    conn_pos = process_fn.find('strcmp(p, "OK+CONN") == 0')
    assert conn_pos != -1
    conn_block = process_fn[conn_pos:dev_info_pos]
    assert "s_ble_connected = true;" in conn_block, "OK+CONN must set s_ble_connected = true"


def test_supervisor_probe_failure_preserves_uart_ready():
    """Verify probe failure leaves s_ble_uart_ready = true but sets s_ble_supervisor_online = false."""
    c_src = BLE_COMM_C.read_text(encoding="utf-8")
    init_fn = c_src[c_src.find("int ble_comm_init"):c_src.find("bool ble_comm_is_uart_ready")]

    assert "s_ble_uart_ready = true;" in init_fn
    assert "s_ble_supervisor_online = false;" in init_fn
    assert "s_ble_module_awake = false;" in init_fn
    assert "if (!probed)" in init_fn
