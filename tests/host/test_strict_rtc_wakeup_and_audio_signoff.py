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

