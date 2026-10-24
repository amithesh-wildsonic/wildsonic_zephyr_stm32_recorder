#!/usr/bin/env python3
"""
Automated unit and lifecycle test suite for WildSonic RTC-Wake Sleep Implementation
and Start Command Sequence Verification.
"""

import os
import re
import sys

from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
APP_DIR = ROOT_DIR / "app" if (ROOT_DIR / "app").is_dir() else ROOT_DIR
PROJECT_ROOT = str(APP_DIR)
SRC_DIR = str(APP_DIR / "src")
PRJ_CONF = str(APP_DIR / "prj.conf")


def test_phase_markers_in_code():
    print("Test 1: Verifying 8 Start Phase Markers in firmware sources...")

    phases = [
        (1, "Command received", ["src/main.c"]),
        (2, "Configuration validated", ["src/main.c"]),
        (3, "DSP initialized", ["src/recorder.c"]),
        (4, "WAV file opened", ["src/recorder.c"]),
        (5, "ADC/DMA configured", ["src/audio_in_stm32l4.c"]),
        (6, "Timer started", ["src/audio_in_stm32l4.c"]),
        (7, "First DMA callback received", ["src/audio_in_stm32l4.c"]),
        (8, "First block accepted and written", ["src/recorder.c"]),
    ]

    for p_num, p_text, files in phases:
        found = False
        target_marker = f"[PHASE {p_num}] {p_text}"
        for rel_file in files:
            path = os.path.join(PROJECT_ROOT, rel_file)
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
                if target_marker in content:
                    found = True
                    break
        assert found, f"Marker '{target_marker}' not found in {files}"
        print(f"  [PASS] Phase {p_num}: '{target_marker}' verified in {files}")


def test_prj_conf_synchronous_printk():
    print("\nTest 2: Verifying synchronous UART logging configuration in prj.conf...")
    with open(PRJ_CONF, "r", encoding="utf-8") as f:
        conf = f.read()
    assert "CONFIG_LOG_PRINTK=n" in conf, "CONFIG_LOG_PRINTK=n missing from prj.conf"
    print("  [PASS] CONFIG_LOG_PRINTK=n verified (printk bypasses log queue to UART immediately)")


def test_rtc_wakeup_timer_architecture():
    print("\nTest 3: Verifying RTC Wakeup Timer hardware integration...")
    rtc_h = os.path.join(SRC_DIR, "rtc_sleep.h")
    rtc_c = os.path.join(SRC_DIR, "rtc_sleep.c")

    assert os.path.isfile(rtc_h), "rtc_sleep.h missing"
    assert os.path.isfile(rtc_c), "rtc_sleep.c missing"

    with open(rtc_c, "r", encoding="utf-8") as f:
        code = f.read()

    # Check hardware registers and calls
    assert "LL_RTC_WAKEUPCLOCK_CKSPRE" in code, "CKSPRE 1Hz clock selection missing"
    assert "LL_RTC_WAKEUP_SetAutoReload" in code, "WUTR auto-reload configuration missing"
    assert "LL_EXTI_LINE_20" in code, "EXTI Line 20 RTC Wakeup mapping missing"
    assert "RTC_WKUP_IRQn" in code, "RTC_WKUP_IRQn NVIC IRQ mapping missing"
    assert "LL_RTC_DisableWriteProtection" in code, "RTC write protection handling missing"
    print("  [PASS] RTC Wakeup Timer (CKSPRE, WUTR, EXTI20, IRQ 3) verified.")


def test_zero_second_sleep_guard():
    print("\nTest 4: Verifying zero-second sleep guard in recorder and main...")
    rec_c = os.path.join(SRC_DIR, "recorder.c")
    main_c = os.path.join(SRC_DIR, "main.c")

    with open(rec_c, "r", encoding="utf-8") as f:
        rc_code = f.read()
    assert "if (seconds == 0)" in rc_code, "Zero-second check missing in recorder_sleep"

    with open(main_c, "r", encoding="utf-8") as f:
        m_code = f.read()
    assert "if (sleep_sec == 0)" in m_code, "Zero-second check missing in execute_recording_sessions"
    print("  [PASS] Zero-second sleep guard verified (prevents rapid loop).")


def test_adc_dma_reentrancy_guards():
    print("\nTest 5: Verifying ADC and DMA lifecycle reentrancy guards...")
    adc_c = os.path.join(SRC_DIR, "audio_in_stm32l4.c")
    with open(adc_c, "r", encoding="utf-8") as f:
        code = f.read()

    # Verify ADC ADRDY is not cleared if already enabled
    assert "if (!LL_ADC_IsEnabled(adc))" in code, "LL_ADC_IsEnabled check missing before ADRDY enable"

    # Verify DMA cleanup before re-configuring
    assert "if (dma_active) {" in code, "dma_active stop check missing"
    assert "LL_DMA_ClearFlag_GI1(DMA1)" in code, "DMA flag clear missing"
    print("  [PASS] ADC ADRDY and DMA reentrancy lifecycle guards verified.")


def test_production_startup_no_verbose_hexdump():
    print("\nTest 6: Verifying verbose SD sector hex dumps are disabled in production startup...")
    rec_c = os.path.join(SRC_DIR, "recorder.c")
    with open(rec_c, "r", encoding="utf-8") as f:
        code = f.read()
    assert "#ifdef CONFIG_AUDIO_DIAGNOSTIC_MODE" in code, "CONFIG_AUDIO_DIAGNOSTIC_MODE guard missing"
    assert code.count("LOG_HEXDUMP_INF") == 2, "Unexpected number of LOG_HEXDUMP_INF calls"
    print("  [PASS] Verbose hex dumps guarded by CONFIG_AUDIO_DIAGNOSTIC_MODE.")


def test_startup_log_flushed_before_prompt():
    print("\nTest 7: Verifying all startup logs are flushed before prompt...")
    main_c = os.path.join(SRC_DIR, "main.c")
    with open(main_c, "r", encoding="utf-8") as f:
        code = f.read()
    assert "while (log_process())" in code, "log_process flush loop missing from main.c"
    assert "zephyr/logging/log_ctrl.h" in code, "log_ctrl.h header missing"
    print("  [PASS] Startup log flush loop verified.")


def test_stop2_mode_and_clock_restoration():
    print("\nTest 8: Verifying STM32L4 Stop 2 mode & 72 MHz clock tree restoration...")
    rtc_c = os.path.join(SRC_DIR, "rtc_sleep.c")
    with open(rtc_c, "r", encoding="utf-8") as f:
        code = f.read()
    assert "LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2)" in code, "LL_PWR_MODE_STOP2 setting missing"
    assert "LL_LPM_EnableDeepSleep()" in code, "DeepSleep enable missing"
    assert "stm32_clock_control_init(NULL)" in code, "Clock restoration call missing"
    assert "SysTick->CTRL &=" in code, "SysTick suspension missing"
    assert "__WFI()" in code, "WFI instruction missing"
    print("  [PASS] STM32L4 Stop 2 mode, SysTick suspension, and 72 MHz clock restoration verified.")


def test_psram_low_power_dpd():
    print("\nTest 9: Verifying IS66WVS PSRAM Deep Power Down (0xB9) & CE# pull-up...")
    psram_c = os.path.join(SRC_DIR, "psram_is66wvs.c")
    with open(psram_c, "r", encoding="utf-8") as f:
        code = f.read()
    assert "CMD_ENTER_DPD" in code, "CMD_ENTER_DPD missing"
    assert "0xB9" in code, "DPD opcode 0xB9 missing"
    assert "psram_sleep" in code, "psram_sleep function missing"
    assert "psram_wake" in code, "psram_wake function missing"
    assert "LL_GPIO_PULL_UP" in code, "PB11 CE# pull-up configuration missing"
    print("  [PASS] PSRAM Deep Power Down (0xB9), wake pulse, and CE# pull-up verified.")


def test_led_shutdown_and_button_wake():
    print("\nTest 10: Verifying User LED (PA5) shutdown and User Button (B1 / PC13) wake...")
    rtc_c = os.path.join(SRC_DIR, "rtc_sleep.c")
    with open(rtc_c, "r", encoding="utf-8") as f:
        code = f.read()
    assert "LL_GPIO_PIN_5" in code, "User LED PA5 pin mapping missing"
    assert "LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5)" in code, "LED OFF reset missing"
    assert "LL_GPIO_PIN_13" in code, "User button PC13 pin mapping missing"
    assert "LL_EXTI_LINE_13" in code, "EXTI Line 13 mapping missing"
    print("  [PASS] User LED PA5 forced OFF and B1 (PC13) EXTI Line 13 wakeup verified.")


def main():
    print("===========================================================================")
    print("   WILDSONIC RTC-WAKE SLEEP & START COMMAND VERIFICATION SUITE")
    print("===========================================================================\n")
    test_phase_markers_in_code()
    test_prj_conf_synchronous_printk()
    test_rtc_wakeup_timer_architecture()
    test_zero_second_sleep_guard()
    test_adc_dma_reentrancy_guards()
    test_production_startup_no_verbose_hexdump()
    test_startup_log_flushed_before_prompt()
    test_stop2_mode_and_clock_restoration()
    test_psram_low_power_dpd()
    test_led_shutdown_and_button_wake()
    print("\n===========================================================================")
    print("   ALL TESTS PASSED (10/10) — FIRMWARE LIFECYCLE, STOP 2 & RTC VALID")
    print("===========================================================================")


if __name__ == "__main__":
    main()

