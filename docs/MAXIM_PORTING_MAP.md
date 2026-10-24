# WildSonic Maxim-to-STM32 Firmware Porting Map and Architecture Audit

**Target Platform**: STMicroelectronics NUCLEO-L476RG (STM32L476RGT6, ARM Cortex-M4 @ 72 MHz)  
**RTOS**: Zephyr RTOS v4.5.0-rc2  
**Source Baseline**: Maxim MAX78000 WildSonic Firmware (`WildSonic_Firmware-feature-dev.zip`)  
**Document Status**: Phase 1 Audit & Porting Map Complete (Pre-Implementation Review)

---

## 1. Executive Summary & Architecture Comparison

The original WildSonic acoustic recorder was developed for the **Maxim Integrated (now Analog Devices) MAX78000**, a dual-core microcontroller (ARM Cortex-M4 primary core + RISC-V coprocessor + CNN hardware accelerator). The current project ports the WildSonic firmware to the **STMicroelectronics STM32L476RG** running **Zephyr RTOS**.

The STM32L476RG platform already features a functional audio acquisition, DSP filtering, PSRAM staging, and FAT32/exFAT WAV recording engine on Zephyr RTOS. The goal of this port is to progressively adopt the production feature set, scheduling engine, and configuration architecture of the original WildSonic system while preserving working STM32 audio drivers and maintaining strict build separation.

### Subsystem Comparison Summary

| Subsystem | Original Maxim MAX78000 | Zephyr STM32L476RG Target | Porting Strategy |
|---|---|---|---|
| **Core Architecture** | Dual-core (Cortex-M4 + RISC-V + CNN); custom bare-metal FreeRTOS-like task runner | Single Cortex-M4 @ 72 MHz running Zephyr RTOS threads | Map tasks into Zephyr cooperative/preemptive threads; no CNN/RISC-V hardware needed for core acoustic recording |
| **Audio Acquisition** | I2S peripheral reading digital microphone / external audio codec | TIM6 hardware TRGO trigger + 12-bit ADC1 (8x hardware oversampling, 15-bit ENOB) + DMA1 circular buffer | Keep existing STM32 ADC1/DMA1 pipeline; do not rewrite audio capture |
| **Digital Signal Processing** | Hardware filters / biquads via codec and software | 5-stage software biquad cascade (DC blocker, 80 Hz HPF, mains notch, 20 kHz LPF) with saturating gain | Retain existing optimized STM32 DSP pipeline (`dsp.c`) |
| **Intermediate Storage** | On-chip / off-chip OCM (Off-Chip Memory) circular pool | 1024 KiB ISSI IS66WVS4M8 QUADSPI PSRAM circular staging buffer with 256 KiB batch writes | Retain existing PSRAM staging driver (`psram_is66wvs.c`, `psram_staging.c`) |
| **Persistent Storage** | FatFs SD Card over SPI/SDHC | FatFs (ELM) SD Card over native 4-bit SDMMC1 (`/SD:`) | Retain existing STM32 SDMMC1 driver |
| **Real-Time Clock** | Dual RTC: internal MAX78000 sub-second RTC + external MCP7940N I2C RTC with battery backup | STM32L476 on-chip hardware RTC calendar with BCD shadow registers (TR/DR), subsecond register (SSR), and VBAT pin | Use STM32 internal RTC exclusively; eliminate external MCP7940N dependency |
| **Power Management & Sleep** | GPIO 1.7 power latch controlled by MCP7940N Alarm 0 output (cuts VDD to SoC) | STM32L476 Stop 2 low-power mode with RTC Wakeup Timer (1 Hz ck_spre) and EXTI button wakeup | Use STM32 Stop 2 mode (`rtc_sleep.c`); retain SRAM1 and RTC registers across sleep intervals |
| **Power Supply & Fuel Gauge** | TI BQ25700A SMBus PMIC + TI BQ34110PWR I2C Fuel Gauge | Direct 3.3V / 5V USB / battery supply via NUCLEO VIN; internal ADC supply monitoring | Abstract battery commands; fuel gauge commands become hardware-specific / mockable |
| **Communications** | 2G/3G/4G GSM modem (SIMCOM AT commands) + BLE | None on baseline NUCLEO board; serial console REPL via ST-LINK VCP | Exclude cellular modem commands from baseline STM32 build; keep console REPL |
| **Task Scheduling** | Complex time-of-day, day-of-week, date range, and solar calculator in `AudioTask.cpp` | Simple cycle loop (`cycles`, `record`, `sleep`) in `main.c` | **Phase 2 priority**: Port calendar, time-of-day, day-of-week, and sleep interval scheduling from `AudioTask.cpp` |
| **Device Configuration** | 512-byte `config_page_st` stored in dual ping-pong flash pages with CRC32; SD boot config `/config.ini` | In-memory `app_settings` struct in `main.c` reset on power cycle | Port `config_page_st` layout, flash wear-leveling driver, and `/config.ini` parser in subsequent phase |

---

## 2. Critical Build Separation Architecture

To ensure firmware stability and safety, the project enforces strict separation between **PRODUCTION** and **TEST** builds:

### Production Build (`build/prod`)
- Focuses exclusively on field deployment: autonomous scheduled acoustic recording, low-power Stop 2 sleep, SD card management, and production CLI configuration.
- **Enabled Components**:
  - `CONFIG_AUDIO_SRC_ADC_STM32L4=y`: Physical analog front-end acquisition (MAX4466 via ADC1 + DMA1).
  - `CONFIG_AUDIO_DSP_MODE_FILTERED=y`: 5-stage biquad filtering cascade.
  - `CONFIG_AUDIO_PSRAM_STAGING=y`: PSRAM circular staging buffer for SD write decoupling.
  - Production CLI commands: `help`, `show`, `status`, `start`, `stop`, `rate`, `record`, `sleep`, `cycles`, `gain`, `filter`, `mode`, `time`, `psram`, `default`, `monitor`.
- **Prohibited Components**:
  - No synthetic audio generators (`CONFIG_AUDIO_SRC_SYNTH=n`).
  - No throughput sweeps or memory benchmarking code (`CONFIG_AUDIO_BENCHMARK=n`).
  - No raw signal stage tapping diagnostics (`CONFIG_AUDIO_STAGE_DIAGNOSTICS=n`).
  - No deterministic PCM injection / replay (`CONFIG_AUDIO_INTEGRITY_REPLAY=n`).
  - No destructive formatting commands in the main firmware.

### Test Build (`build/test` / `build/bench` / `build/diag` / `build/integrity`)
- Contains diagnostic harnesses, integrity verification, and stress test commands.
- **Isolated Capabilities**:
  - `CONFIG_AUDIO_BENCHMARK=y`: Exposes `bench ram`, `bench psram`, `bench sd` commands.
  - `CONFIG_AUDIO_STAGE_DIAGNOSTICS=y`: Exposes `diagnose`, `compare`, and `stress` commands.
  - `CONFIG_AUDIO_INTEGRITY_REPLAY=y`: Exposes `integrity replay <sec>` and `integrity adc <sec>` commands.
  - `CONFIG_AUDIO_SRC_SYNTH=y`: Bypasses ADC hardware using a deterministic sine generator.
  - Dedicated formatter target (`build/format` with `format_sd.c`).

---

## 3. Comprehensive Command and Feature Mapping Table

Below is the complete audit and classification of all 59 console commands from `app/console.cpp`, configuration structures from `device_config.hpp`, boot mechanisms from `OnBootConfig.cpp`, and scheduling algorithms from `AudioTask.cpp`.

### Classification Key:
- **PROD**: Core production feature targeted for STM32 production firmware.
- **TEST**: Test-only / diagnostic feature restricted to test builds.
- **HW_SPECIFIC**: Hardware-specific to Maxim carrier board (PMIC, Fuel Gauge, External MCP7940N RTC, Power Latch). Requires abstraction or stubbing on STM32.
- **NOT_APPLICABLE**: Cellular / GSM / BLE features not present on the STM32 NUCLEO target.
- **DEFERRED**: Planned for a future milestone (e.g. Solar sunrise/sunset calculations requiring location models).

| Original Maxim Command / Feature | Source File & Relevant Function | Original Maxim Behavior | Existing STM32 Implementation | Target Hardware / Zephyr Dependency | Classification | Proposed Porting Action | Validation Test |
|---|---|---|---|---|---|---|---|
| **HELP** (`HELP`, `?`) | `app/console.cpp`<br>`display_console_help()` | Formats and prints command list with descriptions | Implemented in `main.c`<br>`print_help()` | Zephyr Console UART | **PROD** | Update help menu as new commands are ported; keep production commands separated from test commands | Host inspection + console test |
| **SAMF** | `app/console.cpp`<br>`set_sampling_frequency()` | Sets/gets audio sampling rate in Hz (16k, 32k, 44.1k, 48k, 96k, 192k) | Implemented in `main.c`<br>`rate` / `samf` | STM32 TIM6 TRGO + ADC1 | **PROD** | Retain existing rate handler; align validation limits with Maxim values | Target rate sweep + host test |
| **BTPS** | `app/console.cpp`<br>`set_sample_width()` | Sets/gets sample bit width (16, 24, 32 bits) | Configured at compile time via `CONFIG_AUDIO_SAMPLE_BITS` (16 or 32) | STM32 ADC1 (16/32-bit container) | **PROD** | Keep compile-time Kconfig for memory efficiency; add runtime getter `btps` returning active bit width | Host Kconfig test |
| **NUMCH** | `app/console.cpp`<br>`set_num_audio_channels()` | Sets/gets channel count (1=mono, 2=stereo) | Fixed at 1 (mono) in STM32 ADC pipeline | STM32 ADC1 single analog channel PA0 | **PROD** | Add getter returning 1; reject attempts to set stereo until second ADC channel is wired | Host test |
| **AUDIO** | `app/console.cpp`<br>`set_audio_parameters()` | Convenience compound command: `AUDIO <samf> <btps> <numch>` | Not implemented as compound command | STM32 Audio Pipeline | **PROD** | Add compound parser invoking rate and bit-width handlers | CLI parser unit test |
| **GAIN** | `app/console.cpp`<br>`set_audio_gain()` | Sets/gets codec analog gain in dB (-12.0 dB to +30.0 dB) | Implemented in `main.c`<br>`gain` (software digital multiplier 0.1x to 16.0x) | Software DSP multiplier (`dsp.c`) | **PROD** | Support both linear multiplier and decibel notation (`gain +6dB` or `gain 2.0`) | DSP simulation test |
| **LCUTOFF** | `app/console.cpp`<br>`set_lower_cutoff_frequency()` | Sets/gets high-pass filter cutoff in Hz | Implemented in `main.c`<br>`filter hpf <Hz>` | 2nd-order Butterworth HPF (`dsp.c`) | **PROD** | Alias `lcutoff` to existing `filter hpf` handler; range 10 Hz to 2000 Hz | DSP frequency response test |
| **UCUTOFF** | `app/console.cpp`<br>`set_upper_cutoff_frequency()` | Sets/gets low-pass anti-aliasing cutoff in Hz | Implemented in `main.c`<br>`filter lpf <Hz>` | 4th-order Butterworth LPF (`dsp.c`) | **PROD** | Alias `ucutoff` to existing `filter lpf` handler; range 1 kHz to 40 kHz | DSP frequency response test |
| **RECT** | `app/console.cpp`<br>`set_recording_length()` | Sets/gets recording session duration in seconds (1 to 86400) | Implemented in `main.c`<br>`record` / `rect` | Audio WAV writer engine (`recorder.c`) | **PROD** | Retain existing handler; integrates with Phase 2 scheduler | Duration test |
| **SLEEP** | `app/console.cpp`<br>`set_recording_sleep_time()` | Sets/gets pause interval between recording sessions in seconds | Implemented in `main.c`<br>`sleep` | RTC Wakeup Timer + Stop 2 sleep (`rtc_sleep.c`) | **PROD** | Retain existing handler; integrates with Phase 2 scheduler calculations | Sleep timing test |
| **FLDR** | `app/console.cpp`<br>`set_folder_structure()` | Sets folder organization: `flat`, `daily`, or `hourly` | Currently flat root `/SD:/` | Zephyr FatFs directory operations | **PROD** | Implement directory creation in `recorder.c` based on active RTC calendar date/time | Host FS directory test |
| **STOR** | `app/console.cpp`<br>`get_sd_card_storage_status()` | Queries SD total capacity, bytes used, and bytes free | Partial in `status` command | Zephyr `fs_statvfs()` | **PROD** | Expose standalone `stor` command querying sector size, total bytes, and free bytes via `fs_statvfs()` | SD status query test |
| **DATETIME** | `app/console.cpp`<br>`set_start_datetime()` | Sets/gets system date, time, and optional timezone offset: `YYYY-MM-DD HH:MM:SS [[+-]ZZZZ]` | Implemented in `main.c`<br>`time set` / `time show` (UTC without timezone) | STM32 on-chip RTC calendar (`rtc_calendar.c`) | **PROD** | Extend `time set` parser to support optional `[+-]ZZZZ` timezone offset and store in RTC structure | RTC calendar unit test |
| **TIMEZONE** | `app/console.cpp`<br>`set_time_zone()` | Sets/gets timezone offset in format `[[+-]ZZZZ]` | Not implemented | RTC time conversion math | **PROD** | Add `timezone` command; stores signed seconds offset (-43200 to +50400) | Timezone conversion unit test |
| **DATERANGE** | `app/console.cpp`<br>`set_begin_end_date()` | Sets/gets recording calendar date window: `YYYY/MM/DD-YYYY/MM/DD` | Not implemented | Scheduler module (`scheduler.c`) | **PROD (Phase 2)** | Implement start/end date range parsing, validation, and boundary checking | Scheduler boundary test |
| **TIMEOFDAY** | `app/console.cpp`<br>`set_begin_end_time()` | Sets/gets up to 10 time-of-day recording windows: `[index] HH:MM:SS-HH:MM:SS` | Not implemented | Scheduler module (`scheduler.c`) | **PROD (Phase 2)** | Implement multi-range time-of-day storage, ascending sort, and overlap merging | Scheduler range merge test |
| **DAYSOFWEEK** | `app/console.cpp`<br>`set_recording_days_of_week()` | Sets/gets active recording days: `"MTWRFSU"` bitmask (0x7F = all) | Not implemented | Scheduler module (`scheduler.c`) | **PROD (Phase 2)** | Implement day-of-week mask parsing, string formatting, and day matching | Weekday mask test |
| **SN** | `app/console.cpp`<br>`set_serial_number()` | Sets/gets hardware serial number string (up to 32 chars) | Not implemented | Persistent flash config / MCU UID | **PROD** | Default to reading STM32 96-bit Unique Device ID (UID) if unset; allow user override | UID read test |
| **DNAME** | `app/console.cpp`<br>`set_device_name()` | Sets/gets device name string | Not implemented | Persistent flash config | **PROD** | Store in persistent configuration structure | String parser test |
| **PW** | `app/console.cpp`<br>`set_device_password()` | Sets/gets server authentication password | Not implemented | Persistent flash config | **NOT_APPLICABLE** | Exclude or stub until cloud communications are defined | N/A |
| **APPLY** | `app/console.cpp`<br>`apply_settings()` | Commits staged RAM configuration to non-volatile flash | In-memory only; settings lost on power cycle | Zephyr Flash API / NVS (`flash_map.h`) | **PROD (Phase 4)** | Port dual-page wear-leveling flash storage driver to STM32 internal Flash | Flash commit test |
| **RBT** | `app/console.cpp`<br>`reboot()` | Executes software reset | Not implemented | Zephyr `sys_reboot(SYS_REBOOT_COLD)` | **PROD** | Add `rbt` command invoking `sys_reboot()` | Reboot test |
| **ARBT** | `app/console.cpp`<br>`apply_and_reboot()` | Commits config to flash and reboots | Not implemented | Zephyr Flash + `sys_reboot()` | **PROD (Phase 4)** | Combine flash write and system reboot | Apply and reboot test |
| **DEFAULT** | `app/console.cpp`<br>`set_default_config()` | Resets all settings to factory defaults | Implemented in `main.c`<br>`default` | In-memory settings struct | **PROD** | Reset configuration to compiled defaults; invalidate flash page | Defaults test |
| **VERSION** | `app/console.cpp`<br>`fw_version_info()` | Prints firmware version, build date, and git SHA | Boot banner in `main.c` | Zephyr build info | **PROD** | Add `version` command printing structured FW version, compiler, Zephyr version, and build timestamp | Version query test |
| **ASSERT** | `app/console.cpp`<br>`assert_info()` | Queries stored assertion failure log from previous crash | Not implemented | Zephyr crash dump / RAM log | **PROD** | Read crash dump retained in uninitialized SRAM2 region | Crash dump test |
| **OnBootConfig** | `app/OnBootConfig.cpp`<br>`read_config_from_sd_card()` | Reads `/config.ini` from SD on boot, applies commands, writes `/config_status.txt`, deletes `/config.ini` | Not implemented | Zephyr FatFs file I/O | **PROD (Phase 4)** | Implement boot scanner for `/SD:/config.ini`, pipe to command dispatcher, generate `/SD:/config_status.txt` | SD config ingestion test |
| **BATT** | `app/console.cpp`<br>`get_battery_status()` | Queries battery voltage (mV), current (mA), remaining %, temp from TI BQ25700A/BQ34110 | Not implemented | Hardware PMIC / Fuel Gauge | **HW_SPECIFIC** | NUCLEO target lacks TI PMIC. Stub with STM32 internal VREFINT / VBAT ADC measurement | Battery query test |
| **FULLCAP** | `app/console.cpp`<br>`set_fuel_gauge_full_charge_capacity()` | Programs TI BQ34110 learned full capacity register | Not implemented | TI BQ34110 I2C | **HW_SPECIFIC** | Mark not applicable to NUCLEO hardware; preserve configuration parameter in config struct | N/A |
| **LBAT** | `app/console.cpp`<br>`set_low_battery_voltage_cutoff()` | Sets low battery warning threshold in mV | Not implemented | Battery monitor | **HW_SPECIFIC** | Store threshold for internal ADC battery monitor | Threshold test |
| **NBAT** | `app/console.cpp`<br>`set_number_of_cells_used()` | Sets battery cell count (1S, 2S, 3S) | Not implemented | Battery monitor | **HW_SPECIFIC** | Store cell count parameter | Cell count test |
| **SDFMT** | `app/console.cpp`<br>`sd_card_format()` | Unmounts and formats SD card volume | Dedicated standalone target `build/format` (`format_sd.c`) | FatFs `f_mkfs` | **PROD / TEST** | Retain as isolated target or require double confirmation before executing in production | Formatter safety test |
| **GSM** | `app/console.cpp`<br>`set_gsm_transmission_mode()` | Sets 2G/3G/4G transmission mode | Not implemented | Cellular Modem | **NOT_APPLICABLE** | NUCLEO target lacks cellular hardware | N/A |
| **SMSFREQ** | `app/console.cpp`<br>`set_sms_upload_frequency()` | Sets SMS status upload interval in seconds | Not implemented | Cellular Modem | **NOT_APPLICABLE** | NUCLEO target lacks cellular hardware | N/A |
| **SIG** | `app/console.cpp`<br>`get_gsm_signal_quality()` | Queries modem CSQ signal level | Not implemented | Cellular Modem | **NOT_APPLICABLE** | NUCLEO target lacks cellular hardware | N/A |
| **SENDSTS** | `app/console.cpp`<br>`send_status_now()` | Transmits status packet via modem | Not implemented | Cellular Modem | **NOT_APPLICABLE** | NUCLEO target lacks cellular hardware | N/A |
| **A2PPHONE**, **P2APHONE** | `app/console.cpp` | Server and device phone numbers | Not implemented | Cellular Modem | **NOT_APPLICABLE** | NUCLEO target lacks cellular hardware | N/A |
| **SURL**, **SPORT**, **POSTROUTE**, **POSTCTNT**, **STEST**, **SWS** | `app/console.cpp` | Cloud server HTTP endpoints | Not implemented | Cellular Modem / TCP | **NOT_APPLICABLE** | NUCLEO target lacks cellular hardware | N/A |
| **SMSKW**, **SMSKEY** | `app/console.cpp` | SMS manufacturing authorization keys | Not implemented | Cellular Modem | **NOT_APPLICABLE** | NUCLEO target lacks cellular hardware | N/A |
| **BLEPAIR** | `app/console.cpp`<br>`set_ble_pairing_window()` | Opens BLE advertising window | Not implemented | BLE Controller | **NOT_APPLICABLE** | NUCLEO-L476RG lacks BLE radio | N/A |
| **ENVSTS** | `app/console.cpp`<br>`get_environment_status()` | Reads temperature & humidity from I2C sensor | Not implemented | I2C sensor (e.g. SHTC3) | **HW_SPECIFIC** | Stub or connect external I2C sensor via Morpho pins if required | Sensor query test |
| **LOC** | `app/console.cpp`<br>`set_location_info()` | Sets/gets GPS latitude and longitude (scaled by 1e7) | Not implemented | Solar calculator | **DEFERRED** | Store coordinates in config page; calculate solar ephemeris in future phase | Coordinate validation test |
| **SUN**, **SUNMODE**, **SUNRISEBEF**, **SUNRISEAFT**, **SUNSETBEF**, **SUNSETAFT**, **SUNROUND** | `app/console.cpp`<br>`AudioTask.cpp` | Solar astronomical calculation and scheduling around dawn/dusk | Not implemented | Solar calculator algorithms | **DEFERRED** | Defer until core time-scheduled recording is fully validated and location dependencies mapped | Solar math test |
| **I2C** | `app/console.cpp`<br>`handle_i2c_command()` | I2C development bus scan and register read/write | Not implemented | Zephyr I2C driver | **TEST** | Confine to test builds if hardware debugging is needed | I2C scan test |
| **RTCRST** | `app/console.cpp`<br>`reset_rtc_time()` | Resets RTC time to 2025-01-01 00:00:00 UTC | Implemented as `time set 2025-01-01 00:00:00` | STM32 RTC calendar | **TEST** | Restrict fixed-date reset to test builds | RTC set test |
| **TEST_ALARM** | `app/console.cpp`<br>`test_alarm_cmd()` | Toggles GPIO 1.7 power latch and triggers MCP7940N alarm | Not implemented | Maxim hardware latch | **HW_SPECIFIC / TEST** | Maxim-specific power latch test; not applicable to STM32 | N/A |
| **Wildly** | `app/console.cpp` | Dummy command; ignored | Not implemented | N/A | **NOT_APPLICABLE** | Ignore | N/A |
| **bench** | STM32 `main.c`, `benchmark.c` | RAM, PSRAM, and SDMMC throughput sweep benchmarks | Implemented in STM32 TEST build | STM32 hardware | **TEST** | Retain in TEST build only (`CONFIG_AUDIO_BENCHMARK=y`) | Host pytest suite |
| **diagnose** / **diag** | STM32 `main.c`, `recorder.c` | Real-time signal stage tapping (Raw, DC, HPF, LPF, PCM) | Implemented in STM32 TEST build | STM32 DSP pipeline | **TEST** | Retain in TEST build only (`CONFIG_AUDIO_STAGE_DIAGNOSTICS=y`) | Host pytest suite |
| **compare** | STM32 `main.c` | Alternates Cycle 1 Raw vs Cycle 2 Filtered for noise analysis | Implemented in STM32 TEST build | STM32 DSP pipeline | **TEST** | Retain in TEST build only (`CONFIG_AUDIO_STAGE_DIAGNOSTICS=y`) | Host pytest suite |
| **stress** | STM32 `main.c` | Rapid 3 cycles of 5s record / 3s sleep | Implemented in STM32 TEST build | STM32 recorder engine | **TEST** | Retain in TEST build only (`CONFIG_AUDIO_STAGE_DIAGNOSTICS=y`) | Host pytest suite |
| **integrity** | STM32 `main.c`, `recorder.c` | Deterministic PCM pattern replay and readback CRC audit | Implemented in STM32 TEST build | STM32 audio pipeline | **TEST** | Retain in TEST build only (`CONFIG_AUDIO_INTEGRITY_REPLAY=y`) | Host pytest suite |

---

## 4. Subsystem Audits & Implementation Gap Analysis

### 4.1 Command Parser & Console (`app/console.cpp` vs STM32 `app/src/main.c`)
- **Maxim Implementation**: Fixed-length command lookup table with enum translation (`console_config_cmd_t`). Tokenizes input into command verb and argument string (`p_args`). Handlers parse arguments with custom regex (`re.h`) and standard library functions.
- **STM32 Implementation**: Interactive REPL loop in `main.c` using polling UART I/O (`console_poll_char`, `console_read_line`). Verbs are evaluated using `strcmp()`.
- **Gap & Action**:
  - STM32 REPL is non-blocking to characters but blocks waiting for full lines during idle console.
  - Command handlers in `main.c` should be modularized into a cleaner dispatch table.
  - Add missing production commands: `samf` (alias for `rate`), `btps`, `numch`, `audio`, `lcutoff`, `ucutoff`, `fldr`, `stor`, `daterange`, `timeofday`, `daysofweek`, `timezone`, `sn`, `version`, `rbt`.
  - Maintain `#if CONFIG_AUDIO_...` guards on all test commands so they are stripped from `build/prod`.

### 4.2 On-Boot SD Card Configuration (`app/OnBootConfig.cpp`)
- **Maxim Implementation**: On startup, checks for `/config.ini` in the SD card root. Reads chunk by chunk, parses `key = value` lines, translates keys to console command strings, and feeds them to `new_console_command_handler()`. Then generates `/config_status.txt` reporting active parameters, and deletes `/config.ini`. If `/config.ini` does not exist, generates `/config_status.txt` anyway.
- **STM32 Implementation**: Does not currently look for `/SD:/config.ini`.
- **Gap & Action**:
  - Planned for **Phase 4**. The architecture is very clean and portable: reading `/SD:/config.ini` and translating lines directly into console commands reuses the exact same validation logic as the serial console!

### 4.3 Persistent Device Configuration (`lib/config/device_config.cpp` & `.hpp`)
- **Maxim Implementation**: `DeviceConfig::config_page_st` is a fixed 512-byte structure containing magic number `0x13245768`, layout version, sequence number, checksum, audio settings, schedule structures, and calibration data. Flash wear-leveling uses the top two flash sectors (ping-pong sequence numbers).
- **STM32 Implementation**: Currently holds settings in a runtime static struct `settings` in RAM.
- **Gap & Action**:
  - Planned for **Phase 4**. On STM32L476RG (1 MB Flash arranged as two 512 KB banks of 2 KB pages), the last two 2 KB flash pages (Pages 254 and 255 at addresses `0x080FE000` and `0x080FF800`) will be allocated for dual ping-pong config pages.

### 4.4 Scheduling & Audio Lifecycle (`app/AudioTask.cpp` & `.hpp`)
- **Maxim Implementation**: The scheduler in `AudioTask.cpp` implements the authoritative logic for autonomous recording:
  1. `calculate_next_record_length()`:
     - Converts current UTC time to local time: `current_time_local = current_time + time_zone_offset`.
     - Validates start and end calendar dates (`start_date`, `end_date`).
     - Evaluates day-of-week bitmask (`day_of_week_mask`, bits 0..6 for Mon..Sun) via `find_record_interval_from_day_of_week_mask()`.
     - Evaluates multi-window time-of-day ranges (`time_of_day_range_st`, up to 10 slots sorted ascending) via `find_record_interval_from_time_of_day_ranges()`.
     - Enforces minimum sleep spacing: `last_calculated_end_time_local + sleep_interval`.
     - Clips active window to `record_length` if `sleep_interval > 0`.
     - Converts scheduled start time back to UTC.
  2. Sleep Trigger:
     - If `current_time < record_interval.start_time` and `sleep_seconds > MIN_SLEEP_GAP_SECONDS` (5s), programs wake time `record_interval.start_time - EARLY_WAKEUP_SECONDS` (2s early) and initiates low-power shutdown.
- **STM32 Implementation**: Currently executes simple iterative cycles (`for (cycle = 1; cycle <= max_cycles; cycle++)`) with fixed `record_sec` and `sleep_sec`.
- **Gap & Action**:
  - **This is the exact priority for Phase 2**. The scheduling calculation algorithms will be extracted into a modular `scheduler.c` / `scheduler.h` module and integrated into the STM32 recording lifecycle.

### 4.5 Real-Time Clock & Timekeeping (`w_hal_rtc.hpp` / `.cpp` vs STM32)
- **Maxim Implementation**: Required an external I2C RTC (MCP7940N) for battery-backed calendar timekeeping and wake alarms, plus internal MAX78000 RTC for sub-second interval timing.
- **STM32 Implementation**: STM32L476RG includes a sophisticated on-chip hardware RTC calendar (`rtc_calendar.c`) with subsecond resolution (SSR down to fractions of a millisecond), shadow registers `RTC_TR` and `RTC_DR`, battery backup (`VBAT`), and on-chip Wakeup Timer (`rtc_sleep.c`).
- **Gap & Action**:
  - **Zero external RTC hardware needed**. The STM32 on-chip RTC meets and exceeds the capabilities of the Maxim dual-RTC setup.
  - Phase 2 will add timezone offset handling to convert between UTC and local time for schedule comparisons.

### 4.6 Power Management & Sleep (`w_hal_pm.cpp` vs STM32 `rtc_sleep.c`)
- **Maxim Implementation**: Maxim carrier used TI BQ25700A charger and TI BQ34110 fuel gauge. Sleep was achieved by setting an alarm on the MCP7940N and releasing GPIO 1.7 (power latch), cutting power to the MAX78000.
- **STM32 Implementation**: STM32 enters Stop 2 mode (`rtc_sleep_enter_stop2()`), halting CPU core, ADC, and SDMMC clocks while retaining SRAM1, SRAM2, and backup domain registers. Wakeup occurs via RTC Wakeup Timer or User Button B1 (PC13).
- **Gap & Action**:
  - Stop 2 mode is already implemented and validated in `rtc_sleep.c`.
  - In Phase 3, the scheduler will compute the exact seconds to sleep until the next scheduled session and invoke `recorder_sleep()`.

---

## 5. Phase 2 Implementation Plan — Time-Scheduled Recording

Phase 2 implements the first functional porting feature: **time-scheduled recording using the STM32L476RG on-chip RTC**, using `AudioTask.cpp` as the behavioral reference.

### 5.1 Architecture & Pipeline Preservation
- **Preserve Existing Pipeline**: The ADC1 + DMA1 capture engine (`audio_in_stm32l4.c`), biquad DSP cascade (`dsp.c`), PSRAM staging buffer (`psram_staging.c`), and WAV file writer (`recorder.c`) remain 100% untouched.
- **Trigger Existing Lifecycle**: The scheduler evaluates when the next recording should start, arms timers or waits, and invokes the existing `recorder_record_session(duration, cycle, mode, &stats)`.

### 5.2 Schedule Configuration Parameters
The scheduler will support:
1. `time_zone_offset`: Signed integer in seconds (-43200 to +50400, e.g. `+19800` for +05:30).
2. `start_date` and `end_date`: Calendar bounds (year 2000..2099, month 1..12, day 1..31).
3. `day_of_week_mask`: 7-bit bitmask (bit 0 = Monday, bit 1 = Tuesday, ..., bit 6 = Sunday; `0x7F` = all days).
4. `time_of_day_range`: Array of up to 4 non-overlapping intervals within a 24-hour day (`start_second` to `end_second`, 0 to 86400).
5. `record_length`: Recording duration in seconds.
6. `sleep_interval`: Mandatory quiet interval between sessions in seconds.

### 5.3 Boundary Case Handling
- **Invalid RTC Time**: If the STM32 RTC calendar has not been programmed (`rtc_calendar_is_valid() == false`), scheduled recording refuses to arm and prompts the user to program the RTC via `time set <YYYY-MM-DD HH:MM:SS>`.
- **Overdue Schedule**: If the configured `start_date` or `time_of_day` is in the past, but the active window is still open, recording begins immediately for the remainder of the window. If the window has elapsed, the scheduler advances to the next valid window.
- **Past End Date**: If the current time exceeds `end_date`, the scheduler terminates cleanly without recording.
- **User Abort**: Console input (`'q'`, `Ctrl+C`, `ESC`, or `"stop"`) immediately aborts waiting or active recording and returns to the REPL prompt.
- **Missed Events**: If the device boots after a scheduled window, it does not record historical data; it recalculates the next future interval.

### 5.4 Test & Verification Strategy
1. **Host Unit Tests (`tests/host/test_scheduler_logic.py`)**:
   - Verify day-of-week bitmask arithmetic and rollover across Sunday -> Monday.
   - Verify sorting and merging of multiple time-of-day ranges.
   - Verify date range bounding (before start, inside window, after end).
   - Verify local-time to UTC timezone offset conversions.
   - Verify sleep spacing enforcement between consecutive sessions.
2. **Firmware Target Verification**:
   - Both `PRODUCTION` and `TEST` builds must compile cleanly with zero warnings.
   - Verify that all 35 existing host tests continue to pass.
   - Interactive CLI validation of scheduling commands and scheduled recording execution.

---

## 6. Subsequent Phases Roadmap

### Phase 3 — Autonomous Low-Power Sleep & RTC Alarm Wakeup
- Integrate `rtc_sleep_enter_stop2()` into the scheduler.
- Compute sleep duration = `next_start_time - current_time - EARLY_WAKEUP_SECONDS`.
- If sleep duration > 5 seconds, power down peripherals, put PSRAM into Deep Power Down, and enter STM32 Stop 2 mode.
- Wake up via RTC Wakeup Timer, restore 72 MHz PLL clock tree, re-initialize SDMMC, and begin recording on time.

### Phase 4 — Configuration Model, Flash Persistence & On-Boot SD Config
- Define 512-byte packed `device_config_t` structure matching `config_page_st`.
- Implement dual ping-pong wear-leveling flash driver in top STM32 flash pages (Pages 254-255).
- Port `APPLY`, `RBT`, `ARBT`, and `DEFAULT` commands.
- Implement `/SD:/config.ini` ingestion on boot, parameter validation, and `/SD:/config_status.txt` generation.

### Phase 5 — Production Command Set Expansion & Hardening
- Complete remaining production CLI commands (`samf`, `btps`, `numch`, `audio`, `lcutoff`, `ucutoff`, `fldr`, `stor`, `sn`, `version`).
- Enforce strict build separation: ensure zero test symbols or test commands are linked into production binaries.
- Final multi-day hardware soak testing on NUCLEO-L476RG.

