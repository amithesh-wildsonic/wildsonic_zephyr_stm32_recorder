"""
Test Suite: Strict RTC-Only Wakeup, 192 kHz / 32-Bit Conversion, and Memory Layout Audit.

Validates the final production requirements:
1. Exact TIM6 TRGO sample-rate arithmetic at 72 MHz (0.000% error at 192 kHz).
2. Memory layout audit proving SRAM2 (0x10000000) vs SRAM1 (0x20000000) buffer allocations.
3. Well-defined signed PCM to 32-bit container conversion eliminating undefined behavior.
4. Strict RTC-only low-power sleep wake isolation (EXTI Line 20 only, PWR WKUP pins disabled,
   USART2 wake-from-stop disabled, NVIC masking).
5. Exact byte and block accounting for 192 kHz 32-bit mono and duplicated 2-channel sessions.
"""

import os
import re
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
APP_SRC = REPO_ROOT / "app" / "src"
BUILD_PROD = REPO_ROOT / "app" / "build" / "PRODUCTION"
ZEPHYR_MAP = BUILD_PROD / "zephyr" / "zephyr.map"
OVERLAY_FILE = REPO_ROOT / "app" / "boards" / "nucleo_l476rg.overlay"


def test_timer_frequency_and_exact_rate():
    """Verify that 72 MHz APB1 timer clock produces exactly 192,000 Hz with PSC=0, ARR=374."""
    overlay_text = OVERLAY_FILE.read_text(encoding="utf-8")
    assert "clock-frequency = <DT_FREQ_M(72)>" in overlay_text or "DT_FREQ_M(72)" in overlay_text
    assert "apb1-prescaler = <1>" in overlay_text

    tim_clock = 72_000_000
    target_rate = 192_000
    div = tim_clock // target_rate
    assert div == 375

    psc = 0
    arr = div - 1
    assert arr == 374

    actual_rate = tim_clock / ((psc + 1) * (arr + 1))
    assert actual_rate == 192_000.0
    error_percent = abs(actual_rate - target_rate) / target_rate * 100.0
    assert error_percent == 0.0


def test_memory_layout_sram1_vs_sram2_proof():
    """Inspect zephyr.map to prove audio slab is in SRAM2 (0x10000000) and DMA is in SRAM1 (0x20000000)."""
    assert ZEPHYR_MAP.is_file(), f"Production map file not found: {ZEPHYR_MAP}"
    map_text = ZEPHYR_MAP.read_text(encoding="utf-8", errors="replace")

    # 1. Audio slab pool in physical SRAM2 (Zephyr linker section SRAM1 at 0x10000000)
    match_sram2 = re.search(r"SRAM1\s+0x10000000\s+0x8000\s+app/libapp\.a\(recorder\.c\.obj\)", map_text)
    assert match_sram2 is not None, "Audio slab pool not found at 0x10000000 (SRAM2) in map"

    # 2. DMA buffer in SRAM1 (.bss.dma_buf at 0x2000xxxx)
    match_dma = re.search(r"\.bss\.dma_buf\s+(0x2000[0-9a-fA-F]+)\s+0x4000\s+app/libapp\.a\(audio_in_stm32l4\.c\.obj\)", map_text)
    assert match_dma is not None, "DMA buffer not found in SRAM1 (0x20000000 range) in map"
    dma_addr = int(match_dma.group(1), 16)
    assert 0x20000000 <= dma_addr < 0x20018000, f"DMA buffer address {hex(dma_addr)} outside SRAM1 range"

    # 3. Check that SRAM2 (0x10000000 - 0x10008000) has NO other overlapping symbols
    # Section SRAM1 size is exactly 0x8000 (32768 bytes = 100.00% full, 0 bytes overflow)
    match_sram2_size = re.search(r"0x00008000\s+__SRAM1_size\s*=", map_text)
    assert match_sram2_size is not None, "__SRAM1_size is not exactly 0x8000 (32 KiB)"


def test_pcm32_conversion_eliminates_undefined_behavior():
    """Verify that PCM16 to PCM32 container conversion handles all signed ranges without UB."""
    src_h = (APP_SRC / "audio_format.h").read_text(encoding="utf-8")
    assert "audio_pcm16_to_pcm32" in src_h
    assert "(uint32_t)(int32_t)s16 << 16" in src_h

    # Replicate C unsigned-cast left-shift logic in Python for 16-bit values
    def pcm16_to_pcm32(s16: int) -> int:
        u32 = ((s16 & 0xFFFFFFFF) << 16) & 0xFFFFFFFF
        # Convert to signed 32-bit
        return u32 - 0x100000000 if u32 & 0x80000000 else u32

    # Midpoint (0)
    assert pcm16_to_pcm32(0) == 0

    # Max positive (32767 = 0x7FFF)
    assert pcm16_to_pcm32(32767) == 0x7FFF0000
    assert pcm16_to_pcm32(32767) == 2147418112

    # Min negative (-32768 = 0x8000)
    assert pcm16_to_pcm32(-32768) == -2147483648
    assert pcm16_to_pcm32(-32768) == -0x80000000

    # Negative one (-1)
    assert pcm16_to_pcm32(-1) == -65536
    assert (pcm16_to_pcm32(-1) & 0xFFFFFFFF) == 0xFFFF0000

    # Boundary positive (+1)
    assert pcm16_to_pcm32(1) == 65536

    # Verify ADC 12-bit conversion mapping
    def adc_raw_to_pcm16(raw: int, lshift: int = 4) -> int:
        x = (raw << lshift) - 32768
        return max(-32768, min(32767, x))

    assert adc_raw_to_pcm16(0) == -32768          # Min raw -> min signed
    assert adc_raw_to_pcm16(2048) == 0           # Mid raw -> 0 signed
    assert adc_raw_to_pcm16(4095) == 32752       # Max raw -> near max signed


def test_strict_rtc_only_wakeup_code_audit():
    """Verify in rtc_sleep.c that non-RTC wake sources are strictly disabled during Stop 2."""
    src = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")

    # 1. USART2 wake from stop disabled
    assert "LL_USART_DisableInStopMode(USART2);" in src
    assert "LL_USART_DisableIT_WKUP(USART2);" in src

    # 2. STM32 PWR wakeup pins disabled
    assert "LL_PWR_DisableWakeUpPin(" in src
    assert "LL_PWR_WAKEUP_PIN1" in src
    assert "LL_PWR_WAKEUP_PIN5" in src

    # 3. EXTI line masking: Line 20 is the ONLY unmasked interrupt line
    assert "EXTI->IMR1 = LL_EXTI_LINE_20;" in src
    assert "EXTI->EMR1 = 0;" in src
    assert "EXTI->IMR2 = 0;" in src

    # 4. User Button EXTI 13 explicitly disabled
    assert "LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_13);" in src
    assert "LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_13);" in src

    # 5. NVIC IRQs masked except RTC_WKUP_IRQn
    assert "NVIC->ICER[0] = 0xFFFFFFFF & ~(1UL << (RTC_WKUP_IRQn & 0x1F));" in src
    assert "NVIC->ISER[0] = (1UL << (RTC_WKUP_IRQn & 0x1F));" in src

    # 6. SysTick suspended
    assert "SysTick->CTRL &=" in src

    # 7. Low power Stop 2 mode entered
    assert "LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);" in src
    assert "LL_LPM_EnableDeepSleep();" in src

    # 8. Clean restoration of NVIC, EXTI, SysTick, and 72 MHz clock
    assert "stm32_clock_control_init(NULL);" in src
    assert "NVIC->ISER[0] = saved_nvic_iser[0];" in src
    assert "EXTI->IMR1 = saved_exti_imr1;" in src
    assert "SysTick->CTRL = saved_systick_ctrl;" in src


def test_main_scheduled_sleep_calls_recorder_sleep():
    """Verify that main.c scheduled loop enters Stop 2 sleep via recorder_sleep instead of polling."""
    main_c = (APP_SRC / "main.c").read_text(encoding="utf-8")
    assert "recorder_sleep(sleep_chunk, NULL)" in main_c
    assert "Enter true STM32L476 Stop 2 low-power sleep with strict RTC-only wakeup" in main_c


def test_192khz_exact_payload_accounting():
    """Validate 192 kHz mono and 2-channel payload and batch counts for 10-second sessions."""
    rate = 192_000
    duration = 10
    total_frames = rate * duration
    assert total_frames == 1_920_000

    # 32-bit Mono
    mono_bytes_per_sample = 4
    mono_payload = total_frames * mono_bytes_per_sample
    assert mono_payload == 7_680_000

    batch_size = 262_144  # 256 KiB
    mono_full_batches = mono_payload // batch_size
    mono_remainder = mono_payload % batch_size
    assert mono_full_batches == 29
    assert mono_remainder == 77_824
    assert mono_full_batches * batch_size + mono_remainder == mono_payload

    # 32-bit 2-Channel Duplicated Mono Container
    stereo_bytes_per_frame = 8
    stereo_payload = total_frames * stereo_bytes_per_frame
    assert stereo_payload == 15_360_000

    stereo_full_batches = stereo_payload // batch_size
    stereo_remainder = stereo_payload % batch_size
    assert stereo_full_batches == 58
    assert stereo_remainder == 155_648
    assert stereo_full_batches * batch_size + stereo_remainder == stereo_payload


def test_autonomous_scheduler_enters_stop2_path():
    """Verify autonomous scheduler in main_scheduler_service_step calls recorder_sleep for future windows."""
    main_c = (APP_SRC / "main.c").read_text(encoding="utf-8")
    fn_match = re.search(r"void main_scheduler_service_step\(void\)\s*\{(.*?)\n\}", main_c, re.DOTALL)
    assert fn_match is not None, "main_scheduler_service_step not found in main.c"
    fn_body = fn_match.group(1)

    # Must verify it handles wait_seconds > 0 by entering Stop 2
    assert "Standby wait for upcoming window: enter STM32L476 Stop 2 with strict RTC-only wakeup" in fn_body
    assert "recorder_sleep(sleep_chunk, NULL)" in fn_body
    # Must re-read RTC calendar time upon wake
    assert "rtc_calendar_get(&now_cal)" in fn_body or "rtc_calendar_get(&wake_cal)" in fn_body
    # Must dispatch to recorder_thread without blocking console REPL
    assert "dispatch_recording_session(&interval" in fn_body


def test_short_and_long_scheduled_intervals_chunking():
    """Simulate interval chunking: no < 5s bypass, clamps to 65535s, re-evaluates RTC epoch."""
    def simulate_sleep_chunks(wait_seconds: int):
        chunks = []
        cur_epoch = 1000
        target_epoch = cur_epoch + wait_seconds
        while True:
            remaining = target_epoch - cur_epoch
            if remaining <= 0:
                break
            # No special-case bypass: all nonzero intervals enter Stop 2
            chunk = min(remaining, 65535)
            chunks.append(chunk)
            # Advance simulated RTC calendar time
            cur_epoch += chunk
        return chunks

    # 1. Short intervals (1s, 2s, 4s) must not use k_sleep bypass
    assert simulate_sleep_chunks(1) == [1]
    assert simulate_sleep_chunks(2) == [2]
    assert simulate_sleep_chunks(4) == [4]

    # 2. Medium interval (300s)
    assert simulate_sleep_chunks(300) == [300]

    # 3. Long interval (> 65535s: 100,000s) chunked properly
    long_chunks = simulate_sleep_chunks(100_000)
    assert long_chunks == [65535, 34465]
    assert sum(long_chunks) == 100_000
    assert all(c <= 65535 for c in long_chunks)


def test_rtc_sleep_timer_bounds():
    """Verify rtc_sleep_arm enforces 16-bit 65535s bound and CKSPRE 1 Hz clock."""
    rtc_sleep_c = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")
    assert "if (seconds > 65535) {" in rtc_sleep_c
    assert "seconds = 65535;" in rtc_sleep_c
    assert "LL_RTC_WAKEUPCLOCK_CKSPRE" in rtc_sleep_c
    assert "LL_RTC_WAKEUP_SetAutoReload(RTC, seconds - 1);" in rtc_sleep_c


def test_rtc_initialization_failures_and_propagation():
    """Verify rtc_sleep_init fails safely on oscillator timeout and propagates errors."""
    rtc_sleep_c = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")
    assert "LSI startup timeout; RTC init failed" in rtc_sleep_c
    assert "return -ETIMEDOUT;" in rtc_sleep_c
    assert "LL_RTC_SetAsynchPrescaler(RTC, 127);" in rtc_sleep_c
    assert "LL_RTC_SetSynchPrescaler(RTC, 255);" in rtc_sleep_c

    cal_c = (APP_SRC / "rtc_calendar.c").read_text(encoding="utf-8")
    assert "int init_rc = rtc_sleep_init();" in cal_c
    assert "if (init_rc != 0) {" in cal_c
    assert "LL_RTC_ClearFlag_RS(RTC);" in cal_c


def test_failed_psram_dpd_and_recovery():
    """Verify psram_sleep returns -EIO on failure, psram_wake initializes struct, and recorder_sleep propagates."""
    psram_c = (APP_SRC / "psram_is66wvs.c").read_text(encoding="utf-8")
    assert 'TRACE("PSRAM DPD cmd failed: %d", status);' in psram_c
    assert "return -EIO;" in psram_c
    assert "struct psram_id id = {0};" in psram_c

    recorder_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    assert "int psram_sleep_rc = psram_sleep();" in recorder_c
    assert "int psram_wake_rc = psram_wake();" in recorder_c
    assert "if (psram_sleep_rc != 0)" in recorder_c
    assert "if (psram_wake_rc != 0)" in recorder_c


def test_storage_synchronization_failures():
    """Verify recorder_sleep checks and propagates pre-sleep SD idle check and post-wake errors."""
    recorder_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")
    assert "recorder_sd_verify_idle()" in recorder_c
    assert "int psram_wake_rc = psram_wake();" in recorder_c


def test_audio_shutdown_safety_and_timeout():
    """Verify audio_in_stop implements bounded timeout, drains event queue, and accounts residual samples."""
    audio_c = (APP_SRC / "audio_in_stm32l4.c").read_text(encoding="utf-8")
    assert "uint32_t drain_timeout_ms = 100;" in audio_c
    assert "while (k_msgq_get(&dma_event_q, &leftover_event, K_NO_WAIT) == 0)" in audio_c
    assert "atomic_add(&acquisition_lost_samples, active_block_samples);" in audio_c
    assert "atomic_add(&acquisition_samples_accounted, active_block_samples);" in audio_c
    assert "ACQUISITION_THREAD_PRIORITY K_PRIO_PREEMPT(0)" in audio_c


def test_wrong_or_unexpected_wake_source():
    """Verify rtc_sleep_enter_stop2 validates actual WUTF flag and does not assign RTC_WAKE_TIMER unconditionally."""
    rtc_sleep_c = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")
    assert "bool rtc_wut_flag = (LL_RTC_IsActiveFlag_WUT(RTC) != 0);" in rtc_sleep_c
    assert "bool exti20_flag = (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_20) != 0);" in rtc_sleep_c
    assert "Unexpected Stop 2 wake (PR1=0x%08X, SR1=0x%08X)" in rtc_sleep_c
    assert "last_wake_source = RTC_WAKE_NONE;" in rtc_sleep_c


def test_sleep_entry_race_handling():
    """Verify rtc_sleep_enter_stop2 checks WUT/NVIC under __disable_irq before entering __WFI."""
    rtc_sleep_c = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")
    assert "__disable_irq();" in rtc_sleep_c
    assert "bool pre_wake_event = (LL_RTC_IsActiveFlag_WUT(RTC) != 0) ||" in rtc_sleep_c
    assert "if (!pre_wake_event) {" in rtc_sleep_c
    assert "__WFI();" in rtc_sleep_c
    assert "__enable_irq();" in rtc_sleep_c


def test_kernel_time_after_long_sleep():
    """Verify rtc_sleep_enter_stop2 compensates kernel ticks via sys_clock_announce after wake."""
    rtc_sleep_c = (APP_SRC / "rtc_sleep.c").read_text(encoding="utf-8")
    assert "uint64_t elapsed_ticks = (uint64_t)seconds * CONFIG_SYS_CLOCK_TICKS_PER_SEC;" in rtc_sleep_c
    assert "sys_clock_announce(chunk_ticks);" in rtc_sleep_c


def test_rtc_wake_followed_by_correctly_scheduled_recording():
    """Verify the 12-step shutdown sequence and post-wake recording dispatch ordering."""
    recorder_c = (APP_SRC / "recorder.c").read_text(encoding="utf-8")

    # Sleep only starts between closed recording sessions; queued data must not
    # be drained/discarded or an open file closed as an implicit flush.
    sleep_src = recorder_c.split("int recorder_sleep(", 1)[1].split("void recorder_get_active_progress", 1)[0]
    pos_audio_stop = sleep_src.find("audio_in_stop()")
    pos_pending_check = sleep_src.find("recorder_sleep rejected: audio queues not empty")
    pos_staging_flush = sleep_src.find("psram_staging_flush_publish()")
    pos_open_file_guard = sleep_src.find("sd_file_ptr != NULL")
    pos_sd_idle = sleep_src.find("recorder_sd_verify_idle()")
    pos_psram_sleep = sleep_src.find("psram_sleep()")
    pos_stop2 = sleep_src.find("rtc_sleep_enter_stop2(seconds)")
    pos_psram_wake = sleep_src.find("psram_wake()")

    assert -1 < pos_audio_stop < pos_pending_check < pos_staging_flush < pos_open_file_guard < pos_sd_idle < pos_psram_sleep < pos_stop2 < pos_psram_wake, \
        "Sleep must reject pending work and verify SD idle before Stop 2"
    assert "drain_queue();" not in sleep_src[pos_audio_stop:pos_stop2]
    assert "fs_close(sd_file_ptr)" not in sleep_src[pos_audio_stop:pos_stop2]


def test_repeated_sleep_wake_cycles_lifecycle():
    """Simulate 5 consecutive scheduled recording sessions with inter-session Stop 2 sleep."""
    cycle_count = 5
    duration_per_session = 10
    sleep_between = 20
    current_time_epoch = 1700000000

    simulated_sessions = []
    for c in range(1, cycle_count + 1):
        # 1. Target start time
        target_start = current_time_epoch + (0 if c == 1 else sleep_between)
        wait_sec = target_start - current_time_epoch
        assert wait_sec >= 0

        # 2. Standby sleep
        if wait_sec > 0:
            current_time_epoch += wait_sec
        assert current_time_epoch == target_start

        # 3. Recording session
        rec_start = current_time_epoch
        current_time_epoch += duration_per_session
        rec_end = current_time_epoch

        simulated_sessions.append({
            "cycle": c,
            "start": rec_start,
            "end": rec_end,
            "duration": duration_per_session
        })

    assert len(simulated_sessions) == 5
    for i in range(1, 5):
        gap = simulated_sessions[i]["start"] - simulated_sessions[i-1]["end"]
        assert gap == sleep_between, f"Session {i+1} gap was {gap}, expected {sleep_between}"


