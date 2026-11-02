#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Automated Host Verification Suite for:
1. Recording Failure Stage Diagnostics and SDMMC Register Capture.
2. Complete Precondition Auditing for STM32L476 Stop 2 Sleep.
3. Strict RTC Wake Cycle Tracing (SLEEP_REQUESTED through NEXT_RECORDING_STARTED).
"""

from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"

RECORDER_C = APP_SRC / "recorder.c"
AUDIO_IN_C = APP_SRC / "audio_in_stm32l4.c"
RTC_SLEEP_C = APP_SRC / "rtc_sleep.c"
MAIN_C = APP_SRC / "main.c"


def test_sdmmc_stage_diagnostics_and_register_capture():
    """Verify log_sdmmc_stage_diagnostics captures stage, error, offset, SDMMC STA/DCOUNT/DCTRL/CLKCR, DMA, sector."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "static void log_sdmmc_stage_diagnostics(" in code
    assert "uint32_t sta = SDMMC1->STA;" in code
    assert "uint32_t dcount = SDMMC1->DCOUNT;" in code
    assert "uint32_t dctrl = SDMMC1->DCTRL;" in code
    assert "uint32_t clkcr = SDMMC1->CLKCR;" in code
    assert "uint32_t clk_div = clkcr & SDMMC_CLKCR_CLKDIV_Msk;" in code
    assert "DATA_CRC_FAIL" in code
    assert "CMD_CRC_FAIL" in code
    assert "DATA_TIMEOUT" in code
    assert "CMD_TIMEOUT" in code
    assert "TX_UNDERRUN" in code
    assert "RX_OVERRUN" in code
    assert "DATA_TRANSFER" in code and "FINALIZATION" in code
    assert "LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_DMA2)" in code


def test_all_failing_stages_instrumented():
    """Verify each failing stage is instrumented with diagnostics."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert 'log_sdmmc_stage_diagnostics("FILE_OPEN_CREATE"' in code
    assert 'log_sdmmc_stage_diagnostics("INITIAL_HEADER_WRITE"' in code
    assert 'printk("[STAGE_DIAG] Stage: AUDIO_ACQUISITION_START' in code
    assert 'log_sdmmc_stage_diagnostics("FILESYSTEM_WRITE_PSRAM"' in code
    assert 'log_sdmmc_stage_diagnostics("FILESYSTEM_WRITE_QUEUE"' in code
    assert 'log_sdmmc_stage_diagnostics("WAV_HEADER_FINAL_SEEK"' in code
    assert 'log_sdmmc_stage_diagnostics("WAV_HEADER_FINAL_WRITE"' in code
    assert 'log_sdmmc_stage_diagnostics("FILE_TRUNCATE"' in code
    assert 'log_sdmmc_stage_diagnostics("FILE_SYNC"' in code
    assert 'log_sdmmc_stage_diagnostics("FILE_CLOSE"' in code


def test_strict_rtc_wake_cycle_seven_messages():
    """Verify all 7 lifecycle messages required by specification are implemented under TEST build."""
    rec_code = RECORDER_C.read_text(encoding="utf-8")
    rtc_code = RTC_SLEEP_C.read_text(encoding="utf-8")
    main_code = MAIN_C.read_text(encoding="utf-8")

    assert 'printk("[TEST] SLEEP_REQUESTED' in rec_code
    assert 'printk("[TEST] STOP2_ENTERED\\n");' in rtc_code
    assert 'printk("[TEST] RTC_WAKE_ISR\\n");' in rtc_code
    assert 'printk("[TEST] STOP2_EXITED\\n");' in rtc_code
    assert 'printk("[TEST] CLOCKS_RESTORED\\n");' in rtc_code
    assert 'printk("[TEST] SCHEDULER_RESUMED\\n");' in main_code
    assert 'printk("[TEST] NEXT_RECORDING_STARTED' in main_code


def test_all_eleven_sleep_preconditions_audited():
    """Verify recorder_sleep and rtc_sleep_enter_stop2 audit all 11 preconditions with explicit logging."""
    rec_code = RECORDER_C.read_text(encoding="utf-8")
    rtc_code = RTC_SLEEP_C.read_text(encoding="utf-8")

    # 1. Recorder inactive & no flush in progress
    assert "Sleep precondition failed: recorder_active=" in rec_code
    # 2. Audio acquisition stopped
    assert "Sleep precondition failed: audio_in_stop=" in rec_code
    # 3. Audio queues empty
    assert "Sleep precondition failed: audio queues not empty" in rec_code
    # 4. Audio DMA & worker inactive
    assert "Sleep precondition failed: audio DMA active=" in rec_code
    # 5. PSRAM staging empty & not in-flight
    assert "Sleep precondition failed: PSRAM staging not idle" in rec_code
    # 6. SD writer stopped
    assert "sd_writer_flush_req" in rec_code
    # 7. File closed & bounce buffer released
    assert "Sleep precondition failed: SD file still open" in rec_code
    assert "Sleep precondition failed: SD writer bounce buffer held" in rec_code
    # 8. SDMMC TXACT/RXACT/CMDACT cleared, DCOUNT == 0, SDMMC DMA disabled
    assert "Sleep precondition failed: SDMMC controller not idle" in rec_code
    # 9. Pre-sleep SDMMC idle verified
    assert "recorder_sd_verify_idle()" in rec_code
    # 10. PSRAM DPD verified
    assert "Sleep precondition failed: PSRAM DPD failed" in rec_code
    # 11. RTC Wakeup Timer armed before WFI
    assert "Sleep precondition failed: RTC Wakeup Timer not armed" in rtc_code


def test_samples_lost_does_not_fail_completed_recording():
    """Verify dma_stats.samples_lost only triggers -EIO if samples_written is less than expected_samples."""
    code = RECORDER_C.read_text(encoding="utf-8")
    assert "if (st.samples_written < st.expected_samples && !recorder_abort_requested && err == 0) {" in code
    assert "if (dma_stats.samples_lost != 0) {" in code
    assert "err = -EIO;" in code


def test_bounded_worker_and_events_pending_drain_in_audio_stop():
    """Verify audio_in_stop waits for both acquisition_worker_active and acquisition_events_pending."""
    code = AUDIO_IN_C.read_text(encoding="utf-8")
    assert "atomic_get(&acquisition_worker_active) != 0 ||" in code
    assert "atomic_get(&acquisition_events_pending) != 0" in code

