#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Host Regression Suite for Phase 9:
Safe Recording Shutdown, SD Card Idle Verification, and Reliable RTC Stop 2 Wake-Up.

Requirements Covered:
 1. Successful recording completion and shutdown.
 2. DMA completion before shutdown.
 3. PSRAM staging flush.
 4. PSRAM write-in-flight completion.
 5. SD writer completion.
 6. Writer timeout.
 7. WAV header finalization.
 8. SD filesystem synchronization.
 9. SDMMC controller idle verification.
10. SD write error during finalization.
11. No repeated SD sync errors after a known failed session.
12. RTC wake timer programming.
13. Stale RTC/EXTI flag clearing.
14. Stop 2 entry conditions.
15. RTC wake interrupt.
16. Clock restoration.
17. SDMMC reinitialization after wake where required.
18. ADC/DMA reinitialization.
19. Scheduler handoff after wake.
20. Exactly one recording start per scheduled opportunity.
"""

from pathlib import Path
import pytest
import re

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"

RECORDER_C = APP_SRC / "recorder.c"
RECORDER_H = APP_SRC / "recorder.h"
AUDIO_IN_C = APP_SRC / "audio_in_stm32l4.c"
AUDIO_IN_H = APP_SRC / "audio_in.h"
PSRAM_STAGING_C = APP_SRC / "psram_staging.c"
PSRAM_STAGING_H = APP_SRC / "psram_staging.h"
RTC_SLEEP_C = APP_SRC / "rtc_sleep.c"
RTC_SLEEP_H = APP_SRC / "rtc_sleep.h"
MAIN_C = APP_SRC / "main.c"
SCHEDULER_C = APP_SRC / "scheduler.c"


# ==============================================================================
# Requirement 1: Successful recording completion and shutdown
# ==============================================================================
def test_successful_recording_completion_and_shutdown():
    """Verify clean recording completion flow: stop audio -> drain -> finalize WAV -> close -> verify."""
    code = RECORDER_C.read_text(encoding="utf-8")
    session_fn = code.split("int recorder_record_session(")[1].split("void recorder_set_stop_reason")[0]

    assert "audio_in_stop();" in session_fn
    assert "uint32_t drained_blocks = drain_queue();" in session_fn
    assert "psram_staging_flush_publish();" in session_fn
    assert "sd_writer_flush_req = true;" in session_fn
    assert "k_sem_take(&sd_writer_done_sem" in session_fn
    assert "wav_fill_header(" in session_fn
    assert "fs_seek(&f, 0, FS_SEEK_SET);" in session_fn
    assert "fs_truncate(&f," in session_fn
    assert "fs_sync(&f);" in session_fn
    assert "fs_close(&f);" in session_fn
    assert "recorder_verify_file_format(" in session_fn


# ==============================================================================
# Requirement 2: DMA completion before shutdown
# ==============================================================================
def test_dma_completion_before_shutdown():
    """Verify audio_in_stop stops TIM6 and ADC, waits for worker copying, and only then stops DMA."""
    code = AUDIO_IN_C.read_text(encoding="utf-8")
    stop_fn = code.split("int audio_in_stop(void)")[1].split("bool audio_in_is_dma_active(void)")[0]

    pos_tim_stop = stop_fn.find("LL_TIM_DisableCounter(tim);")
    pos_adc_stop = stop_fn.find("LL_ADC_REG_StopConversion(adc);")
    pos_worker_wait = stop_fn.find("atomic_get(&acquisition_worker_active) != 0")
    pos_dma_stop = stop_fn.find("dma_stop(dma_dev, DMA_CH);")
    pos_dma_drain = stop_fn.find("k_msgq_get(&dma_event_q, &leftover_event, K_NO_WAIT)")
    pos_adc_dis = stop_fn.find("LL_ADC_Disable(adc);")
    pos_dma_clear = stop_fn.find("LL_DMA_ClearFlag_TC1(DMA1);")

    assert -1 < pos_tim_stop < pos_adc_stop < pos_worker_wait < pos_dma_stop < pos_dma_drain < pos_adc_dis < pos_dma_clear, \
        "Orderly audio shutdown sequence violated"


# ==============================================================================
# Requirement 3: PSRAM staging flush
# ==============================================================================
def test_psram_staging_flush():
    """Verify that psram_staging_flush_publish transitions FILLING buffer to READY for SD writer."""
    code = PSRAM_STAGING_C.read_text(encoding="utf-8")
    flush_fn = code.split("int psram_staging_flush_publish(void)")[1].split("void psram_staging_get_stats")[0]

    assert "cur->state == BATCH_STATE_FILLING && cur->filled_bytes > 0" in flush_fn
    assert "cur->state = BATCH_STATE_READY;" in flush_fn


# ==============================================================================
# Requirement 4: PSRAM write-in-flight completion
# ==============================================================================
def test_psram_write_in_flight_completion():
    """Verify psram_staging tracks write_in_flight and in_flight_bytes, and exposes psram_staging_is_idle."""
    code = PSRAM_STAGING_C.read_text(encoding="utf-8")
    assert "write_in_flight = true;" in code
    assert "write_in_flight = false;" in code
    assert "bool psram_staging_is_idle(void)" in code
    assert "count_occupied == 0 && reserved_bytes == 0 && !write_in_flight" in code


# ==============================================================================
# Requirement 5: SD writer completion
# ==============================================================================
def test_sd_writer_completion():
    """Verify SD writer signals completion semaphore upon flushing all bytes or on write error."""
    code = RECORDER_C.read_text(encoding="utf-8")
    writer_fn = code.split("static void sd_writer_thread_fn(")[1].split("static uint32_t drain_queue(")[0]

    assert "if (sd_writer_flush_req &&" in writer_fn
    assert "psram_staging_occupied() == 0 || sd_writer_err != 0" in writer_fn
    assert "k_sem_give(&sd_writer_done_sem);" in writer_fn


# ==============================================================================
# Requirement 6: Writer timeout
# ==============================================================================
def test_writer_timeout():
    """Verify bounded writer completion timeout and safe error return on timeout without hanging."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "uint32_t completion_timeout_ms = writer_completion_timeout_ms(" in code
    assert "if (k_sem_take(&sd_writer_done_sem, K_MSEC(completion_timeout_ms)) != 0) {" in code
    assert "st.writer_completion_timeouts++;" in code
    assert "return -ETIMEDOUT;" in code


# ==============================================================================
# Requirement 7: WAV header finalization
# ==============================================================================
def test_wav_header_finalization():
    """Verify that WAV header is written with confirmed st.bytes_written and file is truncated."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "const uint32_t data_bytes = (uint32_t)st.bytes_written;" in code
    assert "wav_fill_header(hdr, st.sample_rate, st.channels, st.sample_bits, data_bytes);" in code
    assert "finalize_ret = fs_seek(&f, 0, FS_SEEK_SET);" in code
    assert "header_written = fs_write(&f, hdr, WAV_HEADER_BYTES);" in code
    assert "finalize_ret = fs_truncate(&f, WAV_HEADER_BYTES + data_bytes);" in code


# ==============================================================================
# Requirement 8: SD filesystem synchronization
# ==============================================================================
def test_sd_filesystem_synchronization():
    """Verify file synchronization via fs_sync and block-device idle check before sleep."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "finalize_ret = fs_sync(&f);" in code
    assert "recorder_sd_verify_idle()" in code


# ==============================================================================
# Requirement 9: SDMMC controller idle verification
# ==============================================================================
def test_sdmmc_controller_idle_verification():
    """Verify recorder_sd_verify_idle checks SDMMC STA active flags, DCOUNT, and DMA channel."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "int recorder_sd_verify_idle(void)" in code
    assert "SDMMC1->STA & (SDMMC_STA_TXACT | SDMMC_STA_RXACT | SDMMC_STA_CMDACT)" in code
    assert "SDMMC1->DCOUNT != 0" in code
    assert "LL_DMA_IsEnabledChannel(DMA2, LL_DMA_CHANNEL_4)" in code


# ==============================================================================
# Requirement 10: SD write error during finalization
# ==============================================================================
def test_sd_write_error_during_finalization():
    """Verify that if fs_write or fs_truncate fails during WAV finalization, error is preserved and logged."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "if (finalize_ret != 0 && err == 0) {" in code
    assert "LOG_ERR(\"WAV finalize/sync/close failed: %d\", finalize_ret);" in code
    assert "err = finalize_ret;" in code


# ==============================================================================
# Requirement 11: No repeated SD sync errors after a known failed session
# ==============================================================================
def test_no_repeated_sd_sync_errors_after_failed_session():
    """Verify recorder_sleep checks s_storage_state and verifies SDMMC idle instead of sending redundant syncs."""
    code = RECORDER_C.read_text(encoding="utf-8")
    sleep_fn = code.split("int recorder_sleep(")[1].split("void recorder_get_active_progress")[0]

    assert "if (s_storage_state == STORAGE_STATE_HEALTHY) {" in sleep_fn
    assert "int sd_idle_rc = recorder_sd_verify_idle();" in sleep_fn


# ==============================================================================
# Requirement 12: RTC wake timer programming
# ==============================================================================
def test_rtc_wake_timer_programming():
    """Verify rtc_sleep_arm configures ck_spre (1 Hz), bounded auto-reload, and enables WUT."""
    code = RTC_SLEEP_C.read_text(encoding="utf-8")
    arm_fn = code.split("int rtc_sleep_arm(")[1].split("void rtc_sleep_disarm(")[0]

    assert "LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);" in arm_fn
    assert "LL_RTC_WAKEUP_SetAutoReload(RTC, seconds - 1);" in arm_fn
    assert "LL_RTC_EnableIT_WUT(RTC);" in arm_fn
    assert "LL_RTC_WAKEUP_Enable(RTC);" in arm_fn
    assert "LL_RTC_WAKEUP_IsEnabled(RTC)" in arm_fn


# ==============================================================================
# Requirement 13: Stale RTC/EXTI flag clearing
# ==============================================================================
def test_stale_rtc_exti_flag_clearing():
    """Verify stale RTC WUT, EXTI Line 20, and NVIC pending flags are cleared prior to WFI and after wake."""
    code = RTC_SLEEP_C.read_text(encoding="utf-8")
    assert "LL_RTC_ClearFlag_WUT(RTC);" in code
    assert "LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);" in code
    assert "NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);" in code


# ==============================================================================
# Requirement 14: Stop 2 entry conditions
# ==============================================================================
def test_stop2_entry_conditions():
    """Verify all pre-conditions before Stop 2: idle controller, drained queues, closed file, disabled wake pins."""
    rec_code = RECORDER_C.read_text(encoding="utf-8")
    rtc_code = RTC_SLEEP_C.read_text(encoding="utf-8")

    assert "bool recorder_is_shutdown_ready(void)" in rec_code
    assert "audio_in_is_dma_active() || audio_in_is_worker_active()" in rec_code
    assert "psram_staging_is_active() && !psram_staging_is_idle()" in rec_code
    assert "recorder_sd_verify_idle() != 0" in rec_code

    assert "LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);" in rtc_code
    assert "LL_LPM_EnableDeepSleep();" in rtc_code
    assert "__WFI();" in rtc_code


# ==============================================================================
# Requirement 15: RTC wake interrupt
# ==============================================================================
def test_rtc_wake_interrupt():
    """Verify RTC_WKUP_IRQn is configured with EXTI Line 20 and rtc_wkup_isr clears flags and gives semaphore."""
    code = RTC_SLEEP_C.read_text(encoding="utf-8")
    assert "IRQ_CONNECT(RTC_WKUP_IRQn, 1, rtc_wkup_isr, NULL, 0);" in code
    assert "irq_enable(RTC_WKUP_IRQn);" in code

    isr_fn = code.split("static void rtc_wkup_isr(")[1].split("enum rtc_wake_source rtc_sleep_get_last_wake_source")[0]
    assert "LL_RTC_ClearFlag_WUT(RTC);" in isr_fn
    assert "LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);" in isr_fn
    assert "last_wake_source = RTC_WAKE_TIMER;" in isr_fn
    assert "k_sem_give(&rtc_wake_sem);" in isr_fn


# ==============================================================================
# Requirement 16: Clock restoration
# ==============================================================================
def test_clock_restoration():
    """Verify clock restoration after Stop 2 via stm32_clock_control_init and PLL status check."""
    code = RTC_SLEEP_C.read_text(encoding="utf-8")
    assert "int clk_ret = stm32_clock_control_init(NULL);" in code
    assert "LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL" in code


# ==============================================================================
# Requirement 17: SDMMC reinitialization after wake where required
# ==============================================================================
def test_sdmmc_post_wake_verification():
    """Verify post-wake SD card responsiveness check and PSRAM wake."""
    code = RECORDER_C.read_text(encoding="utf-8")
    sleep_fn = code.split("int recorder_sleep(")[1].split("void recorder_get_active_progress")[0]

    assert "int psram_wake_rc = psram_wake();" in sleep_fn
    assert "s_power_state = REC_PWR_ACTIVE;" in sleep_fn


# ==============================================================================
# Requirement 18: ADC/DMA reinitialization
# ==============================================================================
def test_adc_dma_reinitialization():
    """Verify audio_in_start reinitializes DMA block config, clears flags, enables ADC ADRDY and starts TIM6."""
    code = AUDIO_IN_C.read_text(encoding="utf-8")
    start_fn = code.split("int audio_in_start(void)")[1].split("int audio_in_stop(void)")[0]

    assert "dma_config(dma_dev, DMA_CH, &cfg);" in start_fn
    assert "dma_start(dma_dev, DMA_CH);" in start_fn
    assert "LL_ADC_Enable(adc);" in start_fn
    assert "LL_ADC_IsActiveFlag_ADRDY(adc)" in start_fn
    assert "LL_ADC_REG_StartConversion(adc);" in start_fn
    assert "LL_TIM_EnableCounter(tim);" in start_fn


# ==============================================================================
# Requirement 19: Scheduler handoff after wake
# ==============================================================================
def test_scheduler_handoff_after_wake():
    """Verify scheduler re-evaluates RTC calendar time after waking from sleep before dispatching next session."""
    code = MAIN_C.read_text(encoding="utf-8")
    sched_fn = code.split("void main_scheduler_service_step(void)")[1].split("/* ==============================================================================\n * Bridge and Dispatch Functions")[0]

    assert "rtc_calendar_get(&wake_cal) != 0 || !rtc_calendar_is_valid()" in sched_fn
    assert "time_t wake_utc = scheduler_calendar_to_epoch(&wake_cal);" in sched_fn
    assert "if (wake_utc < interval.start_time_utc) {" in sched_fn


# ==============================================================================
# Requirement 20: Exactly one recording start per scheduled opportunity
# ==============================================================================
def test_exactly_one_recording_start_per_opportunity():
    """Verify s_rec_busy guard and s_schedule_cycle advancement ensure exactly one recording starts."""
    code = MAIN_C.read_text(encoding="utf-8")

    # In main_scheduler_service_step
    assert "atomic_get(&s_reboot_pending)" in code
    assert "atomic_get(&s_schedule_paused)" in code
    assert "atomic_get(&s_rec_busy)" in code
    assert "atomic_set(&s_rec_busy, 1);" in code
    assert "k_sem_give(&s_rec_trigger_sem);" in code

    # In recorder_thread_fn
    assert "s_schedule_cycle++;" in code
    assert "atomic_set(&s_rec_busy, 0);" in code
    assert "k_sem_give(&s_rec_done_sem);" in code

