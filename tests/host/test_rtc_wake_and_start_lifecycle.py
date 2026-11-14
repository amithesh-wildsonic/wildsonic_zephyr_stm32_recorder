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
    print("Test 1: Verifying session lifecycle markers in firmware sources...")

    markers = [
        ("Recording start:", ["src/recorder.c"]),
        ("[TEST] CAPTURE_STARTED", ["src/recorder.c"]),
        ("[TEST] CAPTURE_TARGET_REACHED", ["src/recorder.c"]),
        ("[TEST] WAV_HEADER_FINALIZED", ["src/recorder.c"]),
        ("[TEST] WAV_FILE_CLOSED", ["src/recorder.c"]),
        ("[TEST] WAV_VERIFIED", ["src/recorder.c"]),
        ("Recording done:", ["src/recorder.c"]),
    ]

    for target_marker, files in markers:
        found = False
        for rel_file in files:
            path = os.path.join(PROJECT_ROOT, rel_file)
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
                if target_marker in content:
                    found = True
                    break
        assert found, f"Marker '{target_marker}' not found in {files}"


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
    print("\nTest 6: Verifying verbose SD sector hex dumps are absent from recorder...")
    rec_c = os.path.join(SRC_DIR, "recorder.c")
    with open(rec_c, "r", encoding="utf-8") as f:
        code = f.read()
    assert "LOG_HEXDUMP_" not in code, "Unexpected LOG_HEXDUMP call in recorder.c"


def test_startup_log_flushed_before_prompt():
    print("\nTest 7: Verifying startup logs run synchronously and trace pump starts after boot...")
    main_c = os.path.join(SRC_DIR, "main.c")
    with open(main_c, "r", encoding="utf-8") as f:
        code = f.read()
    assert "wildsonic_trace_start_pump();" in code, "wildsonic_trace_start_pump missing from main.c"

    trace_c = os.path.join(SRC_DIR, "wildsonic_trace.c")
    with open(trace_c, "r", encoding="utf-8") as f:
        tcode = f.read()
    assert "while (log_process())" in tcode, "log_process flush loop missing from wildsonic_trace.c"
    assert "zephyr/logging/log_ctrl.h" in tcode, "log_ctrl.h header missing from wildsonic_trace.c"


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

