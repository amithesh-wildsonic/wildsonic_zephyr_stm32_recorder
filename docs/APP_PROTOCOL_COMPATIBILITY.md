# WildSonic Mobile Companion Application Protocol Compatibility

**Target Architecture:** STM32L476RG / Zephyr RTOS 4.5.0-rc2  
**Reference Firmware:** MAX78000 (`WildSonic_Firmware-feature-dev`)  
**Companion App:** WildSonic Android / iOS Companion Application  
**Communication Transport:** HM-10 Compatible BLE UART Bridge (USART1 / 9600-8-N-1)

---

## 1. Protocol Architecture & Overview

The WildSonic STM32L476RG firmware integrates complete wire and schema compatibility with the WildSonic mobile companion application without requiring any modifications to the mobile application or its communication protocol.

Communication takes place over a transparent BLE UART bridge:
- **Framing & Termination:** Every incoming command and outgoing response terminates with `\r\n` (CRLF) or standard `\n` (LF).
- **Whitespace Tolerance:** Incoming lines strip leading and trailing whitespace (`\r`, `\n`, spaces, tabs).
- **Case Invariance:** Command verb matching is case-insensitive (`samf 48000`, `SAMF 48000`, `Samf 48000`).
- **Buffer Capacity:** Line buffers support lines up to 128 bytes with strict protection against overflow.

---

## 2. Special Mobile Companion Events

### 2.1 Keep-Alive Event (`WS+ALIVE`)
The mobile companion application periodically sends `WS+ALIVE` as a connection heartbeat while connected.

- **Wire Command:** `WS+ALIVE\r\n`
- **Firmware Handling:**
  1. Intercepted directly inside `ble_comm_process_line()` before dispatcher invocation.
  2. Refreshes the BLE pairing and activity window (`s_pairing_window_active = true`).
  3. Cancels any pending disconnect lock timers (`s_lock_pending_on_disconnect = false`).
  4. Emits **no textual response** over the BLE link (silent absorption matching reference `BleComm.cpp`).

---

## 3. Comprehensive Device Information Report (`GET_DEV_INFO`)

When the mobile app connects, it issues `GET_DEV_INFO` to synchronize device state, hardware capabilities, and schedule settings. The STM32L476RG firmware responds with the exact schema, line order, and field formats of the reference implementation:

| Line # | Format String | Example Value | Description |
|---|---|---|---|
| 1 | `SAMF %u\r\n` | `SAMF 192000` | Audio sampling frequency (Hz) |
| 2 | `BTPS %u\r\n` | `BTPS 16` | Bits per sample (16 or 32) |
| 3 | `NUMCH %u\r\n` | `NUMCH 1` | Number of audio channels (1: Mono) |
| 4 | `GAIN %d\r\n` | `GAIN 0` | Digital mic gain in 0.1 dB units |
| 5 | `LCUTOFF %u\r\n` | `LCUTOFF 0` | High-pass filter cutoff frequency (Hz) |
| 6 | `UCUTOFF %u\r\n` | `UCUTOFF 86400` | Low-pass filter cutoff frequency (Hz) |
| 7 | `RECT %u\r\n` | `RECT 10` | Active recording session duration (s) |
| 8 | `SLEEP %u\r\n` | `SLEEP 10` | Inter-session pause / sleep interval (s) |
| 9 | `VER %s, SHA %s, DT %s %s\r\n` | `VER 1.2.0, SHA d7a4f91, DT Oct 31 2026 19:15:00` | Firmware version, git commit, build date/time |
| 10 | `SDUSED %llu, SDCAP %llu\r\n` | `SDUSED 131072, SDCAP 31256780800` | Used and total SD card capacity in bytes |
| 11 | `FLDR %s\r\n` | `FLDR flat` | Storage folder structure (`flat`, `daily`, `hourly`) |
| 12 | `VBATT N/A, CBATT N/A, REMBATT N/A\r\n` | Literal | Battery voltage, current, and remaining capacity telemetry |
| 13 | `FULLBATT N/A, REMTIME N/A, AVGPWR N/A\r\n` | Literal | Fuel gauge capacity, run-time, and power metrics |
| 14 | `NBAT 1\r\n` | Literal | Number of installed battery cells |
| 15 | `DATETIME YYYY-MM-DD HH:MM:SS.mmm [+-]ZZ:ZZ\r\n` | `DATETIME 2026-10-31 19:15:30.000 +00:00` | Localized RTC calendar with milliseconds and zone offset |
| 16 | `TIMEZONE %c%02d:%02d\r\n` | `TIMEZONE +00:00` | Active timezone formatted with explicit sign |
| 17 | `DATERANGE YYYY/MM/DD-YYYY/MM/DD\r\n` | `DATERANGE 2026/01/01-2099/12/31` | Scheduled recording calendar date range |
| 18 | `SUNMODE Disabled\r\n` | `SUNMODE Disabled` | Solar recording mode state |
| 19-28 | `TIMEOFDAY %d HH:MM:SS-HH:MM:SS\r\n` | `TIMEOFDAY 0 00:00:00-23:59:59` | Exactly 10 slots for daily recording windows |
| 29 | `DAYSOFWEEK %s\r\n` | `DAYSOFWEEK MTWRFSU` | Active days of the week mask string |
| 30 | `MODEL WS-STM32L476\r\n` | Literal | Platform model identifier |
| 31 | `FWVER 1.2.0-BLE\r\n` | Literal | Protocol firmware version tag |
| 32 | `MIC ANALOG\r\n` | Literal | Active microphone hardware type |
| 33 | `BATTCAP 0\r\n` | Literal | Fuel gauge programmed capacity |
| 34 | `TEMP 0.00, HUMID 0.00\r\n` | Literal | Environmental sensors & report termination line |

---

## 4. Status Information Report (`GET_STATUS_INFO`)

When querying live telemetry, the mobile app transmits `GET_STATUS_INFO` (or `STATUS`):

| Line # | Format String | Example Value | Description |
|---|---|---|---|
| 1 | `SDUSED %llu, SDCAP %llu\r\n` | `SDUSED 131072, SDCAP 31256780800` | SD card volume usage and total byte capacity |
| 2 | `VBATT N/A, CBATT N/A, REMBATT N/A\r\n` | Literal | Real-time voltage, current draw, and charge state |
| 3 | `FULLBATT N/A, REMTIME N/A, AVGPWR N/A\r\n` | Literal | Fuel gauge telemetry |
| 4 | `TEMP 0.00, HUMID 0.00\r\n` | Literal | Temperature/humidity sensor status & report terminator |

---

## 5. Command Dispatching and Acknowledgements

### 5.1 Success Acknowledgements
- Configuration setters received via BLE (`SAMF`, `RECT`, `SLEEP`, `GAIN`, `LCUTOFF`, `UCUTOFF`, `FLDR`, `DATERANGE`, `TIMEOFDAY`, `DAYSOFWEEK`, `TIMEZONE`, `DATETIME`, `SN`, `DNAME`, `PW`):
  - When idle: changes stage into RAM and commit immediately. Firmware replies with `OK\r\n`.
  - When active recording: changes stage into RAM for application upon next session. Firmware replies with `OK\r\n`.

### 5.2 Error Acknowledgements
- Strict numeric validation failure (e.g. `SAMF 48000abc`, `RECT -10`, `SLEEP 99999999`):
  - Rejects parameter, retains previous value, and does not return `OK`.
  - Returns `-EINVAL`.

### 5.3 Controlled Device Reboot (`RBT` & `ARBT`)
Reboot commands implement **single response ownership**:
- `RBT`: Quiesces audio pipeline, flushes buffers, closes WAV container, prints `\nRebooting device.\n`, and executes cold reboot.
- `ARBT`: Commits configuration to flash once, quiesces audio pipeline, flushes buffers, closes WAV container, prints `\nRebooting device.\n`, and executes cold reboot.
- No duplicate console prints or conflicting BLE responses are emitted.

