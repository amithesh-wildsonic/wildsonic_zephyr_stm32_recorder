# WildSonic STM32 Firmware BLE & Configuration Compatibility: Implementation Audit

**Target Platform:** STM32L476RG / Zephyr RTOS 4.5.0-rc2  
**Reference Firmware:** WildSonic MAX78000 (`docs/reference_original/WildSonic_Firmware-feature-dev`)  
**Audit Date:** 2026-10-31  
**Status:** Complete source verification performed against reference and target codebases  

---

## 1. Executive Summary

This audit compares the original WildSonic MAX78000 reference firmware (`WildSonic_Firmware-feature-dev`) against the current STM32L476RG Zephyr implementation (`wildsonic_zephyr_stm32_recorder`) to ensure 100% protocol and configuration compatibility with the existing WildSonic mobile companion application and SD `.ini` configuration files.

The mobile application and `.ini` file structures must not be modified. The STM32 target must adhere strictly to the reference protocols, payloads, response schemas, and transactional persistence behaviors.

---

## 2. Confirmed Defects and Incompatibilities

### Bug 1 (P0): `WS+ALIVE` Protocol Event Misrouted to Console Command Dispatcher
- **Reference Location:** `app/BleComm.cpp:1186-1192`
  ```cpp
  else if (strcmp(uart_ble_cmd, "WS+ALIVE") == 0)
  {
      ble_wake_flag = 1;
      shared_glbl.custom_flags.ble_connection_flag = 1;
      ble_pairing_window_active = 1;
      shared_glbl.ble_pairing_window_request_pending = 0;
  }
  ```
  In the reference firmware, `WS+ALIVE` is a pure BLE transport event. It updates connection flags, keeps the BLE module awake, refreshes the pairing window, and produces **no text response**.
- **STM32 Target Defect (`app/src/ble_comm.c:355-364`):**
  `ble_comm_process_line()` lacks a handler for `WS+ALIVE`. It falls through to `console_process_line_source()`, which fails to recognize `WS+ALIVE`, returning `-EINVAL`. `ble_comm_process_line()` then transmits `ERR\r\n` back to the mobile app.
- **Impact:** Mobile app pairing and keep-alive are broken or receive unexpected error responses.
- **Required Fix:** Intercept `WS+ALIVE` directly in `ble_comm_process_line()`, set `s_ble_connected = true`, `s_ble_module_awake = true`, `s_pairing_window_active = true`, reset any pending lock, and return immediately with 0 bytes transmitted.

---

### Bug 2 (P0): Duplicate Acknowledgements on Controlled Reboots (`RBT` & `ARBT`)
- **Reference Location:** `app/console.cpp:2321-2338`
- **STM32 Target Defect (`app/src/console.c:1553-1555, 1631-1633` & `app/src/ble_comm.c:358`):**
  In `handle_rbt()` and `handle_arbt()`:
  ```c
  if (s_from_ble) {
      ble_comm_printf("[ACK] RBT: Controlled reboot initiated\r\n");
  }
  ```
  AND the handler copies `[ACK] RBT: Controlled reboot initiated` into `s_current_out_resp`.
  Then `ble_comm_process_line()` executes:
  ```c
  if (resp_buf[0] != '\0') {
      ble_comm_printf("%s\r\n", resp_buf);
  }
  ```
- **Impact:** The mobile application receives duplicate `[ACK]` lines over BLE UART, confusing the application state machine.
- **Required Fix:** Enforce a single response ownership model. Handlers set `s_current_out_resp` (and output to local console as appropriate); `ble_comm_process_line()` alone transmits the response over BLE.

---

### Bug 3 (P0): Command Handlers Returning `void` Generating False `OK` Acknowledgements
- **Reference Location:** Handlers validate parameters and return explicit failure text or suppress `OK`.
- **STM32 Target Defect (`app/src/console.c:1859-1910`):**
  `handle_samf`, `handle_rect`, `handle_sleep`, `handle_daterange`, `handle_timeofday`, `handle_daysofweek`, `handle_datetime`, `handle_timezone`, `handle_btps`, `handle_numch`, `handle_gain`, `handle_lcutoff`, `handle_ucutoff`, `handle_apply`, `handle_rbt`, `handle_arbt` all return `void`.
  `console_process_line_internal()` executes the switch-case and unconditionally returns `0`.
  In `ble_comm_process_line()`:
  ```c
  int rc = console_process_line_source(p, true, resp_buf, sizeof(resp_buf));
  if (rc == 0) {
      if (resp_buf[0] != '\0') ble_comm_printf("%s\r\n", resp_buf);
      else ble_comm_printf("OK\r\n");
  } else {
      ble_comm_printf("ERR\r\n");
  }
  ```
  When an invalid argument is supplied (e.g., `SAMF 99999` or `RECT -5`), the handler prints an error to UART console but returns `void`. The dispatcher returns `0`, causing `ble_comm_printf("OK\r\n")` to be transmitted to the mobile app!
- **`handle_apply()` specifics:**
  If `recorder_is_active()`, `handle_apply()` returns without setting error or populating response buffer, returning `OK` to BLE. Furthermore, `config_store_save()` return code is ignored.
- **Required Fix:** Refactor all handlers to return `int` (0 on success, negative errno on failure), populate descriptive responses, check `config_store_save()` result in `handle_apply()`, and report genuine errors to callers.

---

### Bug 4 (P0): `GET_DEV_INFO` Protocol Schema Discrepancies
- **Reference Location (`app/BleComm.cpp:816-1017`):**
  Emits exactly 21 lines in fixed order:
  1. `SAMF %d\r\n`
  2. `BTPS %d\r\n`
  3. `NUMCH %d\r\n`
  4. `GAIN %d\r\n`
  5. `LCUTOFF %d\r\n`
  6. `UCUTOFF %d\r\n`
  7. `RECT %d\r\n`
  8. `SLEEP %d\r\n`
  9. `VER %s, SHA %s, DT %s\r\n`
  10. `SDUSED %lld, SDCAP %lld\r\n`
  11. `FLDR %s\r\n`
  12. `VBATT %d, CBATT %d, REMBATT %d\r\n`
  13. `FULLBATT %d, REMTIME %d, AVGPWR %d\r\n`
  14. `NBAT %d\r\n`
  15. `DATETIME %04d-%02d-%02d %02d:%02d:%02d.%03d %c%02d:%02d\r\n`
  16. `TIMEZONE %c%02d:%02d\r\n`
  17. `DATERANGE %04d/%02d/%02d-%04d/%02d/%02d\r\n`
  18. `SUNMODE <mode>\r\n` (if enabled: SUNRISEAFT, SUNSETBEF, SUNSETAFT, SUNROUND; if disabled: `SUNMODE Disabled\r\n`)
  19. `TIMEOFDAY %d %02d:%02d:%02d-%02d:%02d:%02d\r\n` (indices 0..9)
  20. `DAYSOFWEEK %s\r\n`
  21. `TEMP %.2f, HUMID %.2f\r\n`
- **STM32 Target Defect (`app/src/ble_comm.c:1014-1108`):**
  - Injects 12 non-existent fields: `CYCLE`, `NAME`, `MODEL`, `SERIAL`, `FWVER`, `MIC`, `BATTCAP`, `RECTIME`, `INTERVAL`, `SRATE`, `RECBITS`, `NCH`.
  - Omits `VER %s, SHA %s, DT %s`.
  - Omits `FLDR %s`.
  - Omits all 10 `TIMEOFDAY` lines.
  - Omits `DAYSOFWEEK %s`.
  - Emits non-numeric strings `N/A` for battery fields (`VBATT N/A, CBATT N/A, REMBATT N/A`), causing mobile app integer parser crashes.
  - Hardcodes `TIMEZONE +00:00` regardless of configured local timezone offset.
- **Required Fix:** Reconstruct `ble_comm_send_dev_info()` to match the exact reference schema, line count, field order, and formatting.

---

### Bug 5 (P1): `GET_STATUS_INFO` Protocol Discrepancies
- **Reference Location (`app/BleComm.cpp:1022-1052`):**
  Emits exactly 4 lines:
  1. `SDUSED %lld, SDCAP %lld\r\n`
  2. `VBATT %d, CBATT %d, REMBATT %d\r\n`
  3. `FULLBATT %d, REMTIME %d, AVGPWR %d\r\n`
  4. `TEMP %.2f, HUMID %.2f\r\n`
- **STM32 Target Defect (`app/src/ble_comm.c:1110-1135`):**
  Emits `N/A` strings for battery fields.
- **Required Fix:** Emit valid integer fields matching reference defaults (VBATT=3700, CBATT=100, REMBATT=5000, FULLBATT=5000, REMTIME=0, AVGPWR=370).

---

### Bug 6 (P0): Configuration Import Vulnerabilities in `config_store_import_sd()`
- **Target Defect (`app/src/config_store.c:681-830`):**
  1. **Oversized Lines:** `if (ch == '\n' || line_pos >= sizeof(line) - 1)` cuts off lines $\ge 128$ bytes, null-terminates, and executes the corrupted fragment as a valid command.
  2. **EOF without Newline:** `while (fs_read(&file, &ch, 1) == 1)` terminates when EOF is reached; any buffered line without a trailing newline is dropped.
  3. **Unchecked Read Errors:** Negative return codes from `fs_read()` are treated as EOF instead of I/O errors.
  4. **Non-Transactional Application:** Progressive mutation of live `g_cfg` and runtime settings on every line. A malformed line midway leaves a corrupted partial configuration applied.
  5. **Silently Dropped Keys:** Keys such as `transmission_mode`, `location`, `sms_frequency`, `low_battery_threshold`, `sun_recording_mode`, etc., are completely ignored and lost across exports.
- **Required Fix:**
  - Read with bounded line buffer. Reject oversized lines and skip to next newline.
  - Process trailing line on EOF if buffer contains data.
  - Distinguish EOF from read errors.
  - Parse into a temporary `struct wildsonic_config` object, validate all parameters and constraints, and commit transactionally only if valid.
  - Store and preserve all original `.ini` keys in persistent structures.

---

### Bug 7 (P1): Permissive Argument Parsing
- **Target Defect:** Functions use `atoi()` or `strtol(..., NULL, 10)` without verifying `endptr`. Inputs like `SAMF 48000abc` or `RECT 20xyz` are silently accepted.
- **Required Fix:** Implement strict numeric validation: verify string is non-empty, contains only valid digits (with optional leading sign), check `*endptr == '\0'`, verify range bounds, and check cross-parameter consistency (e.g. `upper_cutoff_frequency <= sampling_rate / 2`).

---

### Bug 8 (P1): Timezone & Local Calendar Formatting
- **Target Defect:** `ble_comm.c` hardcodes `+00:00` in `DATETIME` and `TIMEZONE`.
- **Required Fix:** Utilize `g_cfg.time_zone_offset` (in seconds), compute timezone sign (`+` or `-`), hour, and minute components, and apply offset to calendar time for local representation matching `BleComm.cpp:868-886`.

---

## 3. Existing Correct Implementations to Preserve

1. **Audio Acquisition & DSP Pipeline:**
   - ADC1 regular sequence + TIM6 TRGO + DMA circular buffer in SRAM1.
   - 192 kHz 16-bit mono pipeline with 4096-sample DMA blocks.
   - Dual-biquad Butterworth IIR LPF and high-pass DC blocker in `app/src/dsp.c`.
2. **Storage Subsystem:**
   - 3 MB / 12-slot PSRAM staging ring in `app/src/psram_staging.c`.
   - 256 KB SDMMC write batching to FAT/exFAT on SD card.
   - `psram_is66wvs.c` hoisted command structures and Quad SPI driver.
3. **Power Management & RTC:**
   - STM32L476 Stop 2 low-power entry and RTC Wakeup Timer.
   - Safe peripheral shutdown (ADC, DMA, QSPI deep power-down, USART2 TC drain).
   - Scheduler calendar advancement and wake handling.
4. **Flash Configuration Store:**
   - Ping-pong A/B slot alternating flash records with 0x13245768 magic number and additive 32-bit checksum in `app/src/config_store.c`.

---

## 4. Affected Source Files

| File | Changes Required |
|---|---|
| `app/src/ble_comm.c` | Handle `WS+ALIVE` without forwarding to console dispatcher; rebuild `ble_comm_send_dev_info()` and `ble_comm_send_status_info()` to match exact reference schema; enforce single response ownership. |
| `app/src/ble_comm.h` | Expose necessary prototypes and state queries if needed. |
| `app/src/console.c` | Convert handlers from `void` to returning `int`; implement strict argument validation; remove direct `ble_comm_printf` in `handle_rbt`/`handle_arbt`; fix `handle_apply` validation and flash error propagation; format local timezone in status and dev info. |
| `app/src/console.h` | Add fields to `struct wildsonic_config` for preserved parameters; update handler prototypes to return `int`. |
| `app/src/config_store.c` | Rewrite `config_store_import_sd()` for transactional parsing; reject oversized lines; handle EOF without newline; check read errors; preserve original keys; validate 10 time-of-day slots; export accurate `config_status.txt`. |
| `app/src/config_store.h` | Extend persisted configuration record if needed while maintaining 512-byte layout. |
| `tests/host/*` | Add comprehensive unit tests covering all commands, schemas, edge cases, and regression verifications. |

---

## 5. Verification Plan

1. **Host-Side Unit Tests (`pytest tests/host`):**
   - Protocol tests for `WS+ALIVE`, `GET_DEV_INFO`, and `GET_STATUS_INFO`.
   - Strict argument validation tests for all console commands.
   - Configuration importer tests (transactional rollback, EOF without newline, oversized line rejection).
   - Single-ACK guarantee tests for `ARBT` and `RBT`.
   - Timezone parsing and calendar output tests.
2. **Firmware Compilation:**
   - `build/prod`: Production firmware compiles with 0 errors and 0 warnings.
   - `build/test`: Test firmware compiles with 0 errors and 0 warnings.
3. **Documentation:**
   - `docs/IMPLEMENTATION_AUDIT.md`
   - `docs/APP_PROTOCOL_COMPATIBILITY.md`
   - `docs/CONFIGURATION_COMPATIBILITY.md`
   - `docs/BLE_INTEGRATION.md`
   - `docs/COMMAND_REFERENCE.md`
   - `docs/INTEGRATION_TEST_REPORT.md`

