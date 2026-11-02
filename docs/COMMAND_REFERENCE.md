# WildSonic Command Reference Manual

**Target Architecture:** STM32L476RG / Zephyr RTOS 4.5.0-rc2  
**Command Count:** 59 Canonical Commands  
**Interface:** Serial CLI (USART2 / 115200) & Transparent BLE (USART1 / 9600)

---

## 1. Audio Acquisition & Processing Commands

### `SAMF [rate]`
- **Description:** Sets or queries the ADC audio acquisition rate in Hz.
- **Parameters:** `[rate]` = `48000`, `96000`, `192000` (Hz). Strict numeric validation enforced.
- **Query Output:** `Value is set to <rate>`
- **Setter Output:**
  ```text
  Value is set to <rate>
  Send APPLY command and reboot for changes to take effect
  ```
  *(If upper cutoff frequency exceeds Nyquist, adjusts LPF cutoff automatically).*
- **BLE Response:** `OK` on valid setter, `-EINVAL` on invalid rate.

### `BTPS [bits]`
- **Description:** Sets or queries audio bits per sample.
- **Parameters:** `[bits]` = `16` (native) or `32` (experimental container). Strict numeric validation enforced.
- **Query Output:** `Value is set to <bits>`
- **Deferred Hardware Notification:** If `24` is supplied, returns:
  `[DEFERRED] Bits per sample 24 requires future hardware (STM32U585/external I2S codec).`

### `NUMCH [channels]`
- **Description:** Sets or queries active microphone channels.
- **Parameters:** `[channels]` = `1` (native mono) or `2` (stereo). Strict numeric validation enforced.
- **Query Output:** `Value is set to <channels>`

### `AUDIO [numch samf btps]`
- **Description:** Query or set packed audio configuration parameters.
- **Parameters:** 10-digit packed string `%1d%7d%2d` (e.g. `1019200016`).
- **Query Output:**
  ```text
  Number of channels set to <channels>
  Sampling frequency set to <samf>
  Bits per sample set to <btps>
  ```

### `GAIN [dB]`
- **Description:** Sets or queries digital microphone amplification gain in 0.1 dB increments.
- **Parameters:** `[dB]` = `-40.0` to `40.0` dB.
- **Query Output:** `Audio gain set to 0.1*<gain_times_ten>`

### `LCUTOFF [freq]`
- **Description:** Sets or queries high-pass filter (HPF) cutoff frequency in Hz.
- **Parameters:** `[freq]` = `0` (disabled) or `10` to `2000` Hz.
- **Query Output:** `Value is set to <freq>`

### `UCUTOFF [freq]`
- **Description:** Sets or queries low-pass filter (LPF) cutoff frequency in Hz.
- **Parameters:** `[freq]` = `0` (disabled) or `1000` to Nyquist bound (49% of sampling rate).
- **Query Output:** `Value is set to <freq>`

---

## 2. Schedule & Storage Commands

### `RECT [seconds]`
- **Description:** Sets or queries active audio recording duration.
- **Parameters:** `[seconds]` = `1` to `86400` seconds. Strict numeric validation enforced.
- **Query Output:** `Value is set to <seconds>`

### `SLEEP [seconds]`
- **Description:** Sets or queries the inter-session pause/sleep interval.
- **Parameters:** `[seconds]` = `0` to `86400` seconds. Strict numeric validation enforced.
- **Query Output:** `Value is set to <seconds>`
- *Note:* SLEEP configures the scheduler timer interval; it does not trigger an immediate low-power state and does not check `recorder_is_active()`.

### `DATERANGE [start-end]`
- **Description:** Sets or queries scheduled recording calendar date window.
- **Parameters:** Format `YYYY/MM/DD-YYYY/MM/DD` (e.g. `2026/01/01-2099/12/31`).
- **Query Output:** `Begin end date set to <start>-<end>`

### `TIMEOFDAY [index] [start-end]`
- **Description:** Sets or queries daily time-of-day recording windows (up to 10 slots).
- **Parameters:** Slot index `0`..`9` and range `HH:MM:SS-HH:MM:SS`.
- **Query Output:** `Begin end time (Index <i>) set to <start>-<end>`

### `DAYSOFWEEK [mask]`
- **Description:** Sets or queries days of week mask for scheduled recordings.
- **Parameters:** Quotation enclosed string containing `MTWRFSU` or `"X"` (all days).
- **Query Output:** `Days of week set to "<mask_string>"`

### `DATETIME [YYYY-MM-DD HH:MM:SS [+-ZZZZ]]`
- **Description:** Sets or queries onboard hardware RTC calendar.
- **Query Output:**
  ```text
  Boot time UTC set to <utc_str>
  Boot time Local set to <local_str>
  Current time UTC time is <utc_str>
  Current time Local is <local_str>
  ```

### `TIMEZONE [+-ZZZZ]`
- **Description:** Sets or queries time zone offset.
- **Parameters:** Format `[+-]HH:MM` or `[+-]HHMM`.
- **Query Output:** `Time zone is set to [+-]HH:MM`

### `FLDR [flat|daily|hourly]`
- **Description:** Sets or queries directory organization on SD card.
- **Query Output:** `Value is set to <mode>`

### `STOR`
- **Description:** Reports total bytes written and SD card capacity.
- **Output:** `SD card bytes filled : <used> / <total>`

---

## 3. Device Identity & Management Commands

### `SN [serial]`
- **Description:** Sets or queries device serial number.
- **Parameters:** Max 3 alphanumeric characters (e.g. `001`).
- **Query Output:** `Value is set to <sn>`

### `DNAME [name]`
- **Description:** Sets or queries BLE advertised device name.
- **Parameters:** Max 6 alphanumeric characters (e.g. `WILD01`).
- **Query Output:** `Value is set to <name>`

### `PW [password]`
- **Description:** Sets or queries authentication password.
- **Parameters:** Max 8 characters.
- **Query Output:** `Value is set to <pw>`

### `VER`
- **Description:** Displays firmware release version, git commit SHA, and build timestamp.
- **Output:**
  ```text
  Version: v1.2.0
  GIT Branch: production, SHA: d7a4f91
  ```

### `DEFAULT`
- **Description:** Restores all settings to factory defaults.
- **Output:** `Configurations are set to Default.`

---

## 4. Execution, Session Control & Reboot

### `APPLY`
- **Description:** Commits staged RAM configurations to persistent on-chip flash (NVS).
- **Constraints:** If recording is active, returns `BUSY: Cannot commit to flash while recording is active. Send STOP_RECORDING first.`
- **Success Output:** `Device configuration is applied after reboot.`

### `RBT`
- **Description:** Safely stops any active recording, flushes staging buffers, finalizes WAV file, and reboots.
- **Single Response Guarantee:** Outputs `\nRebooting device.\n` exactly once.

### `ARBT`
- **Description:** Commits configuration to flash once, stops any active recording safely, flushes staging buffers, finalizes WAV file, and reboots.
- **Single Response Guarantee:** Outputs `\nRebooting device.\n` exactly once.

### `STOP_RECORDING`
- **Description:** Safely terminates the current active recording session and finalizes file storage.
- **Output:** `OK: Stop recording requested; finalizing WAV container and closing file.`

### `BLEPAIR [seconds|LOCK]`
- **Description:** Controls BLE pairing window. `LOCK` or `0` locks pairing immediately.

---

## 5. Mobile & Telemetry Reporting

### `GET_DEV_INFO`
- **Description:** Emits complete 34-line device telemetry report over BLE UART (see `APP_PROTOCOL_COMPATIBILITY.md`).

### `GET_STATUS_INFO` / `STATUS`
- **Description:** Emits 4-line live status telemetry report over BLE UART.

### `HELP`
- **Description:** Prints list of canonical product commands with descriptions.

---

## 6. Preserved Deferred Hardware Commands
Commands that depend on hardware features not present on the STM32L476RG (such as cellular modem, fuel gauge IC, solar tracking) return informative deferred status:
- `SMSKW`, `SMSKEY`, `A2PPHONE`, `P2APHONE`, `SURL`, `SPORT`, `POSTROUTE`, `POSTCTNT`, `STEST`, `SWS`, `GSM`
- `SMSFREQ`, `BATT`, `SIG`, `LBAT`, `NBAT`, `FULLCAP`, `SENDSTS`
- `ENVSTS`, `LOC`, `SUN`, `SUNMODE`, `SUNRISEBEF`, `SUNRISEAFT`, `SUNSETBEF`, `SUNSETAFT`, `SUNROUND`, `I2C`, `RTCRST`, `TEST_ALARM`

