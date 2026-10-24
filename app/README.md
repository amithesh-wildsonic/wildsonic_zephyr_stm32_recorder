# WildSonic STM32 Firmware Application

This directory contains the Zephyr RTOS application firmware for the WildSonic STM32L476RG Audio Recorder.

For complete developer documentation, environment setup, wiring, build commands, and operating instructions, refer to the [Main Developer Guide (`../README.md`)](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/README.md).

---

## Directory Structure

- `boards/`: Devicetree overlays and board configuration for `nucleo_l476rg`.
- `configs/`: Modular configuration fragments for test and diagnostic builds:
  - `test_benchmark.conf`: Throughput benchmark suite (`CONFIG_AUDIO_BENCHMARK=y`).
  - `test_diagnostics.conf`: Real-time signal stage tapping (`CONFIG_AUDIO_STAGE_DIAGNOSTICS=y`).
  - `test_integrity_replay.conf`: Deterministic PCM replay and CRC readback (`CONFIG_AUDIO_INTEGRITY_REPLAY=y`).
  - `test_synth.conf`: Synthetic tone generator backend (`CONFIG_AUDIO_SRC_SYNTH=y`).
- `sd/`: Storage bus devicetree overlays (`sdmmc_nucleo_l476rg.overlay`, `spi_nucleo_l476rg.overlay`).
- `src/`: Core C firmware implementation:
  - `audio_in.h`: Hardware audio acquisition backend abstraction interface.
  - `audio_in_stm32l4.c`: STM32L476 hardware-timed TIM6 TRGO + ADC1 + DMA1 capture engine.
  - `audio_in_synth.c`: Deterministic software sine tone generator backend.
  - `diagnostics/`: Isolated benchmarking routines (`benchmark.c`, `benchmark.h`).
  - `dsp.c`, `dsp.h`: 5-stage biquad digital signal processing pipeline and saturating gain.
  - `format_sd.c`: Standalone one-shot SD card formatting utility.
  - `main.c`: System bringup, shell REPL, and interactive configuration console.
  - `psram_is66wvs.c`, `psram_is66wvs.h`: ISSI IS66WVS4M8 QUADSPI Pseudo-SRAM driver.
  - `psram_staging.c`, `psram_staging.h`: Circular staging ring buffer implementation.
  - `recorder.c`, `recorder.h`: Multi-session WAV writer engine and lifecycle coordinator.
  - `rtc_calendar.c`, `rtc_calendar.h`: Hardware RTC calendar timestamping driver.
  - `rtc_sleep.c`, `rtc_sleep.h`: Stop 2 low-power sleep and wakeup manager.
  - `wav.c`, `wav.h`: Canonical RIFF/WAVE header generation and validation.
- `CMakeLists.txt`: Application build system script.
- `Kconfig`: Application-level Kconfig options.
- `prj.conf`: Default production Zephyr configuration.

---

## Quick Build Commands

```powershell
# Production Firmware (from repository root)
west build -b nucleo_l476rg app -d build/prod -p always -- -DSD_BUS=sdmmc

# Flash target
west flash -d build/prod
```

For complete technical specifications, refer to:
- Architecture & Pipeline: [`../docs/ARCHITECTURE.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/ARCHITECTURE.md)
- Hardware & Pinouts: [`../docs/HARDWARE_WIRING.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/HARDWARE_WIRING.md)
- MicroSD Formatting: [`../docs/FORMAT_SD_CARD.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/FORMAT_SD_CARD.md)
- PSRAM & Storage Benchmarks: [`../docs/BENCHMARK_VALIDATION.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/BENCHMARK_VALIDATION.md)
