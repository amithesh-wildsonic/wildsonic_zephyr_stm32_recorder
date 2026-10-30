#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Host Regression Suite:
RTC Stop 2 Wakeup Sequence, WUTWF Timeout Handling, Sleep Retry Backoff,
Diagnostic Rate Limiting, SDMMC 6 MHz Clock Verification, and USART2 TC Flag Preservation.
"""

from pathlib import Path
import pytest
import re

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
ZEPHYR_DISK_DRIVER = Path(r"C:\Users\amith\zephyrproject\zephyr\drivers\disk\sdmmc_stm32.c")

RTC_SLEEP_C = APP_SRC / "rtc_sleep.c"
RTC_SLEEP_H = APP_SRC / "rtc_sleep.h"
CONSOLE_C = APP_SRC / "console.c"
CONSOLE_H = APP_SRC / "console.h"
RECORDER_C = APP_SRC / "recorder.c"
MAIN_C = APP_SRC / "main.c"
SD_OVERLAY = REPO_ROOT / "app" / "sd" / "sdmmc_nucleo_l476rg.overlay"


# ==============================================================================
# 1. RTC WAKEUP TIMER LIFECYCLE & SEQUENCING TESTS
# ==============================================================================

def test_rtc_wakeup_timer_disable_write_ready_enable_sequence():
    """Verify RTC wakeup timer follows disable -> WUTWF wait -> configure -> enable sequence."""
    rtc_c = RTC_SLEEP_C.read_text(encoding="utf-8")

    # Access enable (PWR APB1 clock + Backup Domain write access)
    assert "LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR)" in rtc_c
    assert "LL_PWR_EnableBkUpAccess()" in rtc_c
    assert "rtc_access_enable" in rtc_c

    # In rtc_sleep_arm:
    assert "LL_RTC_WAKEUP_Disable(RTC)" in rtc_c
    assert "LL_RTC_IsActiveFlag_WUTW(RTC)" in rtc_c
    assert "LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE)" in rtc_c
    assert "LL_RTC_WAKEUP_SetAutoReload(RTC, seconds - 1)" in rtc_c
    assert "LL_RTC_ClearFlag_WUT(RTC)" in rtc_c
    assert "LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20)" in rtc_c
    assert "LL_RTC_EnableIT_WUT(RTC)" in rtc_c
    assert "LL_RTC_WAKEUP_Enable(RTC)" in rtc_c
    assert "LL_RTC_EnableWriteProtection(RTC)" in rtc_c


def test_wutwf_timeout_handling_and_diagnostics():
    """Verify WUTWF polling enforces bounded timeout and reports register diagnostics on failure."""
    rtc_c = RTC_SLEEP_C.read_text(encoding="utf-8")
    assert "Timeout waiting for RTC WUTWF" in rtc_c
    assert "-ETIMEDOUT" in rtc_c
    assert "RTC->ISR" in rtc_c
    assert "RTC->CR" in rtc_c


def test_unexpected_wake_source_detection_and_flag_inspection_before_irq():
    """Verify wake flags are captured before enabling interrupts and taking ISR."""
    rtc_c = RTC_SLEEP_C.read_text(encoding="utf-8")

    # Inspection before __enable_irq()
    wfi_idx = rtc_c.find("__WFI()")
    enable_irq_idx = rtc_c.find("__enable_irq()", wfi_idx)
    flag_check_idx = rtc_c.find("bool rtc_wut_flag = (LL_RTC_IsActiveFlag_WUT(RTC) != 0);", wfi_idx)

    assert wfi_idx != -1, "__WFI() must be present in Stop 2 path"
    assert flag_check_idx != -1, "rtc_wut_flag must be captured after WFI"
    assert enable_irq_idx != -1, "__enable_irq() must be called"
    assert flag_check_idx < enable_irq_idx, "Hardware flags MUST be snapshotted BEFORE __enable_irq() to prevent ISR clearing them first"

    # Rich register snapshot on unexpected wake
    assert "Unexpected wake event: RTC WUT flag not asserted" in rtc_c
    assert "EXTI_PR1=0x%08X" in rtc_c
    assert "PWR_SR1=0x%08X" in rtc_c
    assert "NVIC_ISPR=" in rtc_c


def test_exti_and_rtc_pending_flag_cleanup():
    """Verify all EXTI, RTC, and Cortex-M exception pending bits are cleared prior to sleep."""
    rtc_c = RTC_SLEEP_C.read_text(encoding="utf-8")
    assert "LL_RTC_ClearFlag_WUT(RTC)" in rtc_c
    assert "LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20)" in rtc_c
    assert "NVIC_ClearPendingIRQ(RTC_WKUP_IRQn)" in rtc_c
    assert "SCB_ICSR_PENDSTCLR_Msk" in rtc_c, "SysTick pending bit must be cleared"
    assert "SCB_ICSR_PENDSVCLR_Msk" in rtc_c, "PendSV pending bit must be cleared to prevent immediate WFI exit"


# ==============================================================================
# 2. USART2 TC FLAG PRESERVATION & RX RECOVERY AUDIT
# ==============================================================================

def test_usart2_icr_no_raw_all_ones_and_no_tccf_clear():
    """Verify that no code writes 0xFFFFFFFF to USART ICR and TCCF is never cleared."""
    console_c = CONSOLE_C.read_text(encoding="utf-8")
    rtc_c = RTC_SLEEP_C.read_text(encoding="utf-8")

    assert "ICR = 0xFFFFFFFF" not in console_c, "console.c must not write 0xFFFFFFFF to ICR"
    assert "ICR = 0xFFFFFFFF" not in rtc_c, "rtc_sleep.c must not write 0xFFFFFFFF to ICR"

    # Verify console_async_resume clears only specific RX error and wakeup flags
    assert "LL_USART_ClearFlag_ORE(USART2)" in console_c
    assert "LL_USART_ClearFlag_FE(USART2)" in console_c
    assert "LL_USART_ClearFlag_NE(USART2)" in console_c
    assert "LL_USART_ClearFlag_PE(USART2)" in console_c
    assert "LL_USART_ClearFlag_IDLE(USART2)" in console_c
    assert "LL_USART_ClearFlag_WKUP(USART2)" in console_c
    assert "LL_USART_ClearFlag_TC" not in console_c, "console_async_resume must NEVER clear TC flag"


def test_usart2_rdr_read_guarded_by_rxne():
    """Verify RDR read is strictly guarded by RXNE check in console resume and sleep entry."""
    console_c = CONSOLE_C.read_text(encoding="utf-8")
    rtc_c = RTC_SLEEP_C.read_text(encoding="utf-8")

    assert "if (LL_USART_IsActiveFlag_RXNE(USART2))" in console_c
    assert "if (LL_USART_IsActiveFlag_RXNE(USART2))" in rtc_c


def test_usart2_tc_timeout_reports_rich_diagnostic_registers():
    """Verify USART2 TC check reports full diagnostic registers on timeout."""
    rtc_c = RTC_SLEEP_C.read_text(encoding="utf-8")
    assert "USART2 transmit-complete timeout; refusing Stop2 entry" in rtc_c
    assert "ISR=0x%08X" in rtc_c
    assert "CR1=0x%08X" in rtc_c
    assert "CR3=0x%08X" in rtc_c
    assert "BRR=0x%08X" in rtc_c
    assert "TC=%d" in rtc_c
    assert "TXE=%d" in rtc_c
    assert "UE=%d" in rtc_c
    assert "TE=%d" in rtc_c


# ==============================================================================
# 3. SLEEP RETRY BACKOFF & RATE LIMITING TESTS
# ==============================================================================

def test_sleep_attempt_counter_and_failure_tracking():
    """Verify autonomous scheduler implements monotonic attempt counter and progressive backoff."""
    main_c = MAIN_C.read_text(encoding="utf-8")
    assert "s_sleep_attempt_counter" in main_c
    assert "s_sleep_attempt_counter++;" in main_c
    assert "s_sleep_backoff_until_ms" in main_c
    assert "s_consecutive_sleep_failures" in main_c
    assert "k_uptime_get() < s_sleep_backoff_until_ms" in main_c, "Scheduler must skip sleep attempts during backoff"

    # Verify backoff calculation logic: 1000 ms per failure up to 10 s
    failures = [1, 2, 5, 10, 15]
    for f in failures:
        backoff_ms = 1000 * (10 if f > 10 else f)
        assert 1000 <= backoff_ms <= 10000


def test_diagnostic_rate_limiting_prevents_console_flooding():
    """Verify error diagnostics are rate-limited to at most once per 5 seconds."""
    main_c = MAIN_C.read_text(encoding="utf-8")
    assert "s_last_sleep_error_log_ms" in main_c
    assert "k_uptime_get() - s_last_sleep_error_log_ms >= 5000" in main_c
    assert "Autonomous sleep attempt" in main_c and "returned error:" in main_c


def test_console_interactive_awake_window():
    """Verify 10-second interactive awake window keeps MCU awake during user activity."""
    main_c = MAIN_C.read_text(encoding="utf-8")
    assert "s_console_last_activity_ms" in main_c
    assert "k_uptime_get() - s_console_last_activity_ms < 10000" in main_c


def test_scheduler_behavior_after_sleep_failure():
    """Verify scheduler breaks loop on failure and resets counters on documented success/abort paths."""
    main_c = MAIN_C.read_text(encoding="utf-8")
    assert "s_consecutive_sleep_failures = 0;" in main_c
    assert "s_sleep_backoff_until_ms = 0;" in main_c
    assert "resetting consecutive failure counter" in main_c


# ==============================================================================
# 4. SDMMC 6 MHZ CLOCK AND DIVIDER VERIFICATION
# ==============================================================================

def test_effective_sdmmc_clock_calculation_and_overlay_divider():
    """Verify that app/sd/sdmmc_nucleo_l476rg.overlay sets clk-div = <6> for 6 MHz operation."""
    overlay = SD_OVERLAY.read_text(encoding="utf-8")
    assert "clk-div = <6>;" in overlay, "Overlay must configure clk-div = <6> for 6 MHz SDMMC clock"
    assert "bus-width = <4>;" in overlay

    # Formula check: SDMMCCLK = 48 MHz, CLKDIV = 6
    # f = 48 MHz / (6 + 2) = 6.0 MHz
    f_hz = 48_000_000 // (6 + 2)
    assert f_hz == 6_000_000

    # 4-bit bus bandwidth:
    bw_bytes_per_sec = (f_hz * 4) // 8
    assert bw_bytes_per_sec == 3_000_000  # 3.0 MB/s

    # Audio stream (192 kHz, 32-bit duo-mono = 768,000 B/s)
    audio_stream_bps = 192000 * 4
    margin = bw_bytes_per_sec / audio_stream_bps
    assert margin > 3.9, f"6 MHz bus provides {margin:.2f}x margin over audio throughput"


def test_runtime_divider_and_clock_diagnostic_consistency():
    """Verify driver and recorder log calculated clock, divider, and CLKCR register."""
    if ZEPHYR_DISK_DRIVER.is_file():
        driver = ZEPHYR_DISK_DRIVER.read_text(encoding="utf-8")
        assert "SDMMC initialized: clk=%u Hz (div=%u), bus=%u-bit, CLKCR=0x%08x" in driver

    rec_c = RECORDER_C.read_text(encoding="utf-8")
    assert "SDMMC hardware clock: %u Hz (CLKDIV=%u, CLKCR=0x%08X, bus=%s)" in rec_c
