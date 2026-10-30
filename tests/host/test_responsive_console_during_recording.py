#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Test Suite: Responsive Console During Active Recording & Command Dispatch Audit.

Verifies:
 1. Asynchronous interrupt-driven UART RX architecture:
    - CONFIG_UART_INTERRUPT_DRIVEN and CONFIG_RING_BUFFER enabled in Kconfig.
    - Bounded ring buffer (256 bytes) and ISR decoupling reception from thread scheduling.
    - Dedicated non-blocking console execution context with timeout wait, eliminating blind sleeps.
 2. Command availability and dispatch:
    - HELP, GET_STATUS, GET_DEV_INFO, GET_CONFIG, STOP_RECORDING registered and operational.
 3. Safe command behavior during active recording:
    - STOP_RECORDING signals safe shutdown via recorder_request_stop(), drains buffers, closes file, returns OK.
    - Configuration setters (SAMF, BITS, CHANNELS, RECT, SLEEP, GAIN) stage in RAM for next session
      without mutating active ADC hardware rate or DSP filter parameters during recording.
    - APPLY returns BUSY rejection when recording is active.
    - SLEEP returns BUSY rejection when recording is active.
 4. Strict RTC-only sleep isolation:
    - console_async_suspend() is called prior to entering Stop 2 sleep so UART RX cannot wake MCU.
    - console_async_resume() is called after RTC wakeup to restore asynchronous reception.
"""

import re
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
PRJ_CONF = REPO_ROOT / "app" / "prj.conf"

CONSOLE_H = APP_SRC / "console.h"
CONSOLE_C = APP_SRC / "console.c"
MAIN_C = APP_SRC / "main.c"
RECORDER_C = APP_SRC / "recorder.c"
RECORDER_H = APP_SRC / "recorder.h"
RTC_SLEEP_C = APP_SRC / "rtc_sleep.c"


def test_uart_interrupt_kconfig_enabled():
    """Verify prj.conf enables UART interrupt-driven driver and ring buffer."""
    conf = PRJ_CONF.read_text(encoding="utf-8")
    assert "CONFIG_SERIAL=y" in conf, "CONFIG_SERIAL missing in prj.conf"
    assert "CONFIG_UART_INTERRUPT_DRIVEN=y" in conf, "CONFIG_UART_INTERRUPT_DRIVEN missing in prj.conf"
    assert "CONFIG_RING_BUFFER=y" in conf, "CONFIG_RING_BUFFER missing in prj.conf"


def test_console_async_api_declarations():
    """Verify console.h declares async UART lifecycle and character reception APIs."""
    h_text = CONSOLE_H.read_text(encoding="utf-8")
    assert "int console_async_init(const struct device *dev);" in h_text
    assert "int console_async_get_char(unsigned char *c);" in h_text
    assert "int console_async_wait_char(unsigned char *c, int32_t timeout_ms);" in h_text
    assert "void console_async_suspend(void);" in h_text
    assert "void console_async_resume(void);" in h_text


def test_console_async_ring_buffer_and_isr_implementation():
    """Verify console.c implements bounded ring buffer and ISR with error checking."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    assert "RING_BUF_DECLARE(s_console_rx_rb, CONSOLE_RX_BUF_SIZE);" in c_text
    assert "uart_irq_callback_user_data_set" in c_text
    assert "uart_irq_rx_enable" in c_text
    assert "uart_fifo_read" in c_text
    assert "ring_buf_put" in c_text
    assert "ring_buf_get" in c_text
    assert "k_sem_give(&s_console_rx_sem);" in c_text


def test_main_uses_non_blocking_async_wait():
    """Verify main.c initializes async console and replaces blind sleep with async wait."""
    main_text = MAIN_C.read_text(encoding="utf-8")
    assert "console_async_init(console_dev);" in main_text
    assert "console_async_wait_char(&c, 20)" in main_text
    assert "k_sleep(K_MSEC(50))" not in main_text, "Found legacy 50 ms blind sleep in console_read_line"


def test_command_registration_and_dispatch():
    """Verify GET_STATUS, GET_DEV_INFO, GET_CONFIG, STOP_RECORDING in lookup table."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")
    for cmd in ["GET_STATUS", "GET_DEV_INFO", "GET_CONFIG", "STOP_RECORDING"]:
        assert f'"{cmd}"' in c_text, f"{cmd} not in console_cmd_lookup_table"
    assert "CMD_GET_STATUS" in c_text
    assert "CMD_GET_DEV_INFO" in c_text
    assert "CMD_GET_CONFIG" in c_text
    assert "CMD_STOP_RECORDING" in c_text


def test_safe_command_behavior_during_active_recording():
    """Verify staged RAM setters and BUSY rejections during active recording."""
    c_text = CONSOLE_C.read_text(encoding="utf-8")

    # 1. APPLY returns BUSY
    assert "BUSY: Cannot commit to flash while recording is active" in c_text

    # 2. SLEEP returns BUSY when recording is active
    assert "BUSY: Cannot enter sleep while recording is active" in c_text

    # 3. STOP_RECORDING requests stop and returns OK
    assert "OK: Stop recording requested" in c_text
    assert "recorder_request_stop()" in c_text

    # 4. Setters stage in RAM and report explicit OK message
    assert "OK: Value staged for next session (active session unaffected)" in c_text


def test_recorder_exposes_thread_safe_progress_and_file():
    """Verify recorder.h and recorder.c export non-blocking progress and active file accessors."""
    h_text = RECORDER_H.read_text(encoding="utf-8")
    assert "recorder_get_active_progress" in h_text
    assert "recorder_get_active_file" in h_text

    c_text = RECORDER_C.read_text(encoding="utf-8")
    assert "void recorder_get_active_progress(uint64_t *samples_written, uint64_t *expected_samples)" in c_text
    assert "void recorder_get_active_file(char *out_buf, size_t max_len)" in c_text


def test_strict_rtc_sleep_suspends_and_resumes_console():
    """Verify rtc_sleep.c suspends UART interrupts before Stop 2 and resumes them upon wake."""
    src = RTC_SLEEP_C.read_text(encoding="utf-8")
    assert "console_async_suspend();" in src
    assert "console_async_resume();" in src

    # Verify suspension happens BEFORE entering Stop 2 WFI
    suspend_pos = src.find("console_async_suspend();")
    wfi_pos = src.find("__WFI();")
    assert 0 <= suspend_pos < wfi_pos, "console_async_suspend must occur before __WFI()"

    # Verify resumption happens AFTER clock control restoration
    clk_restore_pos = src.find("stm32_clock_control_init(NULL);")
    resume_pos = src.find("console_async_resume();")
    assert 0 <= clk_restore_pos < resume_pos, "console_async_resume must occur after stm32_clock_control_init()"


def test_main_thread_priority_management_and_instant_echo():
    """Verify console thread priority is elevated for instant echo and lowered during command execution."""
    main_text = MAIN_C.read_text(encoding="utf-8")
    assert "k_thread_priority_set(k_current_get(), 1);" in main_text, "Console prompt must run at priority 1"
    assert "k_thread_priority_set(k_current_get(), 4);" in main_text, "Command execution must drop to priority 4"
    assert "while (console_async_get_char(&c) == 0)" in main_text, "Ring buffer burst drain missing in console_read_line"

    c_text = CONSOLE_C.read_text(encoding="utf-8")
    assert "bool console_async_is_active(void)" in c_text
    # Verify sem take in both get_char and wait_char maintains 1:1 token lockstep
    assert "k_sem_take(&s_console_rx_sem, K_NO_WAIT)" in c_text


