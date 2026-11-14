#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Host Verification Suite for:
Critical Fix: STM32L476 Recording -> SD Flush -> Stop 2 -> RTC Wake -> Next Recording

Validates all 12 core requirements and 23 diagnostic test milestones:
  1. Thread Priority Hierarchy & Inversion Fix:
     Acquisition (0), Recorder (1), SD Writer (3), Scheduler (4), Console (1/4).
  2. Hot-Path Native PCM & Fine-Grained Drop Accounting:
     dma_queue_full_drops, slab_alloc_drops, half_unstable_drops,
     half_overwritten_drops, queue_submit_drops, worker_deadline_drops,
     boundary_discarded_samples.
  3. DSP Stages Optimization & Nyquist Cutoff Validation.
  4. RTC Shadow Register Hardware Synchronization (LL_RTC_ClearFlag_RS & wait RSF).
  5. RTC WUT Disarm & Stale Flag Cleanup at Startup (Prevents capture interruptions).
  6. Targeted Interrupt Masking & EXTI Line 20 / NVIC Pending preservation.
  7. DBGMCU Stop Mode Retention (DBGMCU_CR_DBG_STOP prevents NRST reset on debugger target loss).
  8. Explicit 115200 8N1 UART Clock & Configuration Restoration Post-Wake.
  9. Power State Machine (REC_PWR_ACTIVE -> RECORDING -> FINALIZING -> SLEEP_PREP -> ARMED -> STOP2 -> RECOVERY).
  10. Sleep Interval measured from true RTC completion timestamp after WAV closure.
  11. Dedicated Autonomous Scheduler Thread (decoupled from console idle loop).
  12. Presence of all 23 structured [TEST] diagnostic milestone tags.
"""

from pathlib import Path
import pytest

ROOT_DIR = Path(__file__).resolve().parents[2]
APP_SRC = (ROOT_DIR / "app" / "src") if (ROOT_DIR / "app" / "src").is_dir() else (ROOT_DIR / "src")

AUDIO_IN_H = APP_SRC / "audio_in.h"
AUDIO_IN_C = APP_SRC / "audio_in_stm32l4.c"
DSP_C = APP_SRC / "dsp.c"
MAIN_C = APP_SRC / "main.c"
RECORDER_H = APP_SRC / "recorder.h"
RECORDER_C = APP_SRC / "recorder.c"
RTC_CALENDAR_H = APP_SRC / "rtc_calendar.h"
RTC_CALENDAR_C = APP_SRC / "rtc_calendar.c"
RTC_SLEEP_C = APP_SRC / "rtc_sleep.c"
CONSOLE_C = APP_SRC / "console.c"
BLE_COMM_C = APP_SRC / "ble_comm.c"


def test_thread_priority_hierarchy_inversion_fix():
    """Requirement 1: Acquisition (0) > Recorder (1) > SD Writer (3) > Scheduler (4)."""
    audio_code = AUDIO_IN_C.read_text(encoding="utf-8")
    main_code = MAIN_C.read_text(encoding="utf-8")
    rec_code = RECORDER_C.read_text(encoding="utf-8")

    assert "#define ACQUISITION_THREAD_PRIORITY K_PRIO_PREEMPT(0)" in audio_code
    assert "#define RECORDER_THREAD_PRIORITY   1" in main_code
    assert "#define SD_WRITER_PRIORITY 3" in rec_code
    assert "#define SCHEDULER_THREAD_PRIORITY   4" in main_code
    assert "k_thread_priority_set(k_current_get(), 1);" in main_code
    assert "k_thread_priority_set(k_current_get(), 4);" in main_code


def test_fine_grained_drop_counters_and_source_sample_tracking():
    """Requirement 2: Specific drop counters distinguish DMA, slab, half-buffer, submit, worker drops."""
    audio_h = AUDIO_IN_H.read_text(encoding="utf-8")
    audio_c = AUDIO_IN_C.read_text(encoding="utf-8")

    counters = [
        "dma_queue_full_drops",
        "slab_alloc_drops",
        "half_unstable_drops",
        "half_overwritten_drops",
        "queue_submit_drops",
        "worker_deadline_drops",
        "boundary_discarded_samples",
    ]
    for c in counters:
        assert c in audio_h, f"Missing {c} in audio_in.h"
        assert c in audio_c, f"Missing {c} in audio_in_stm32l4.c"

    assert "uint64_t audio_in_get_source_samples(void);" in audio_h
    assert "uint64_t audio_in_get_source_samples(void)" in audio_c


def test_dsp_nyquist_boundary_and_bypass_optimization():
    """Requirement 3: Validate DSP LPF below Nyquist (0.49*rate) and early bypass when unneeded."""
    dsp_code = DSP_C.read_text(encoding="utf-8")

    assert "float nyquist_bound = (float)sample_rate_hz * 0.49f;" in dsp_code
    assert "if ((float)cfg->lpf_cutoff_hz >= nyquist_bound)" in dsp_code
    assert "if (num_active > 0U || gain != 1.0f)" in dsp_code


def test_rtc_shadow_register_rsf_synchronization():
    """Requirement 4: Wait for hardware RSF synchronization before reading shadow registers."""
    rtc_code = RTC_CALENDAR_C.read_text(encoding="utf-8")
    rtc_h = RTC_CALENDAR_H.read_text(encoding="utf-8")

    assert "LL_RTC_ClearFlag_RS(RTC);" in rtc_code
    assert "LL_RTC_IsActiveFlag_RS(RTC)" in rtc_code
    assert "void rtc_calendar_log_snapshot(const char *phase, int64_t target_utc);" in rtc_h
    assert "void rtc_calendar_log_snapshot(const char *phase, int64_t target_utc)" in rtc_code


def test_rtc_wut_disarm_and_stale_flag_cleanup_at_startup():
    """Requirement 5: Disarm and clear WUT at startup to prevent lingering interrupts during capture."""
    rtc_sleep = RTC_SLEEP_C.read_text(encoding="utf-8")
    init_fn = rtc_sleep.split("int rtc_sleep_init(void)")[1].split("int rtc_sleep_arm")[0]

    assert "LL_RTC_WAKEUP_Disable(RTC);" in init_fn
    assert "LL_RTC_DisableIT_WUT(RTC);" in init_fn
    assert "LL_RTC_ClearFlag_WUT(RTC);" in init_fn
    assert "LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);" in init_fn
    assert "NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);" in init_fn


def test_dbgmcu_stop_mode_retention_and_targeted_masking():
    """Requirement 6 & 7: Set DBGMCU_CR_DBG_STOP and avoid destructive NVIC/EXTI wiping."""
    rtc_sleep = RTC_SLEEP_C.read_text(encoding="utf-8")

    assert "DBGMCU->CR |= DBGMCU_CR_DBG_STOP;" in rtc_sleep
    # Verify we do NOT indiscriminately wipe NVIC ICPR or all EXTI PR bits
    assert "NVIC->ICPR[0] = 0xFFFFFFFF;" not in rtc_sleep
    assert "NVIC->ICPR[1] = 0xFFFFFFFF;" not in rtc_sleep
    assert "EXTI->PR1 = 0xFFFFFFFF;" not in rtc_sleep


def test_uart_clock_and_configuration_restoration():
    """Requirement 8: Explicit uart_configure 115200 8N1 on console and BLE post-wake resume."""
    console_code = CONSOLE_C.read_text(encoding="utf-8")
    ble_code = BLE_COMM_C.read_text(encoding="utf-8")

    assert "baudrate = 115200" in console_code
    assert "uart_configure(s_console_uart_dev, &ucfg);" in console_code

    assert "baudrate = 115200" in ble_code
    assert "uart_configure(s_ble_uart_dev, &bcfg);" in ble_code


def test_power_state_machine_transitions():
    """Requirement 9: Formal power lifecycle states tracked and transitions audited."""
    rec_h = RECORDER_H.read_text(encoding="utf-8")
    rec_c = RECORDER_C.read_text(encoding="utf-8")

    states = [
        "REC_PWR_ACTIVE",
        "REC_PWR_RECORDING",
        "REC_PWR_FINALIZING",
        "REC_PWR_SLEEP_PREP",
        "REC_PWR_RTC_ARMED",
        "REC_PWR_STOP2",
        "REC_PWR_WAKE_RECOVERY",
    ]
    for s in states:
        assert s in rec_h, f"Missing state {s} in recorder.h"
        assert s in rec_c, f"Missing state {s} transition in recorder.c"

    assert "enum recorder_power_state recorder_get_power_state(void);" in rec_h
    assert "enum recorder_power_state recorder_get_power_state(void)" in rec_c


def test_sleep_interval_measured_from_true_rtc_completion():
    """Requirement 10: Sleep interval measured from actual WAV finalization completion, not nominal start."""
    main_code = MAIN_C.read_text(encoding="utf-8")

    rec_thread = main_code.split("static void recorder_thread_fn(")[1].split("static uint32_t s_sleep_attempt_counter")[0]
    assert "struct rtc_calendar_time tend;" in rec_thread
    assert "end_utc = scheduler_calendar_to_epoch(&tend);" in rec_thread
    assert "s_last_end_time_local = end_utc + s_rec_req.tz_offset;" in rec_thread


def test_dedicated_scheduler_thread_decoupled_from_console():
    """Requirement 11: Dedicated scheduler thread runs independently from console prompt."""
    main_code = MAIN_C.read_text(encoding="utf-8")

    assert "#define SCHEDULER_THREAD_STACK_SIZE 2048" in main_code
    assert "#define SCHEDULER_THREAD_PRIORITY   4" in main_code
    assert "scheduler_tid = k_thread_create(" in main_code
    assert "scheduler_thread_fn" in main_code

    # console_read_line must not service scheduler
    console_read_line = main_code.split("static bool console_read_line(")[1].split("static bool prompt_read_line")[0]
    assert "main_scheduler_service_step();" not in console_read_line


def test_presence_of_all_twenty_three_milestone_tags():
    """Requirement 12: Verify all 23 structured diagnostic test milestone messages exist in code."""
    all_code = "\n".join([
        (APP_SRC / "audio_in_stm32l4.c").read_text(encoding="utf-8"),
        (APP_SRC / "recorder.c").read_text(encoding="utf-8"),
        (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8"),
        (APP_SRC / "main.c").read_text(encoding="utf-8"),
    ])

    milestones = [
        "[TEST] CAPTURE_STARTED",
        "[TEST] TIM6_STOPPED",
        "[TEST] ADC_STOPPED",
        "[TEST] DMA_WORKER_DRAINED",
        "[TEST] DMA_EVENTS_DRAINED",
        "[TEST] CAPTURE_TARGET_REACHED",
        "[TEST] AUDIO_QUEUES_EMPTY",
        "[TEST] PSRAM_FLUSH_COMPLETE",
        "[TEST] SD_WRITER_COMPLETE",
        "[TEST] WAV_HEADER_FINALIZED",
        "[TEST] WAV_FILE_CLOSED",
        "[TEST] WAV_VERIFIED",
        "[TEST] SDMMC_IDLE",
        "RECORDING ACCOUNTING SUMMARY:",
        "Accounting: acquired %llu",
        "[TEST] SLEEP_INTERVAL_STARTED:",
        "[TEST] RTC_WUT_ARMED:",
        "[TEST] STOP2_EXITED",
        "[TEST] RTC_WAKE_ISR",
        "[TEST] CLOCKS_RESTORED",
        "[TEST] UART_RESTORED",
        "[TEST] RTC_CALENDAR_ADVANCED:",
        "[TEST] SCHEDULER_RESUMED",
        "[TEST] NEXT_RECORDING_STARTED:",
    ]

    for m in milestones:
        assert m in all_code, f"Missing milestone tag: {m}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
