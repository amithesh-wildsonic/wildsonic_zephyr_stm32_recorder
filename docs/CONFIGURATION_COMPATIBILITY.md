# WildSonic Configuration Compatibility & Storage Specification

**Target Architecture:** STM32L476RG / Zephyr RTOS 4.5.0-rc2  
**Storage Backends:** MicroSD (exFAT / FatFS via SDMMC 4-bit) & On-Chip Flash (NVS)  
**Configuration Files:** `/SD:/config.ini`, `/SD:/config.bak`, `/SD:/config_status.txt`

---

## 1. Overview & Lifecycle

The WildSonic STM32L476RG firmware provides dual-layer configuration management:
1. **Flash Non-Volatile Storage (NVS):** Stores persistent device identity, audio acquisition rates, filter bounds, and schedule configurations across cold boots and Stop 2 sleep cycles.
2. **SD Card Configuration Importer (`config.ini`):** Imports user configurations created by desktop configuration tools or text editors on the SD card root directory during boot.

```
       +--------------------+
       |  /SD:/config.ini   |
       +---------+----------+
                 | (Boot Import)
                 v
       +--------------------+      APPLY / ARBT      +--------------------+
       |  Staged RAM Config | ---------------------> | Flash NVS Config   |
       +---------+----------+                        +--------------------+
                 |                                              |
                 +-------------------+--------------------------+
                                     |
                                     v
                       +---------------------------+
                       | Active Audio & Scheduler  |
                       +---------------------------+
```

---

## 2. SD Card INI Configuration Key Mapping

The SD configuration importer parses key-value pairs formatted as `key = value`. Comments beginning with `#` and leading/trailing whitespace around keys and values are stripped.

### 2.1 Supported Functional Keys

| INI Configuration Key | Translated Canonical Command | Constraints & Range |
|---|---|---|
| `sampling_frequency` / `samf` | `SAMF <val>` | Validated against TIM6/ADC divider table (e.g. 192000, 96000, 48000) |
| `record_length` / `rect` | `RECT <val>` | 1 to 86400 seconds |
| `sleep_interval` / `sleep` | `SLEEP <val>` | 0 to 86400 seconds |
| `bits_per_sample` / `btps` | `BTPS <val>` | 16 (production) or 32 (experimental) |
| `number_of_channels` / `numch`| `NUMCH <val>` | 1 (mono) |
| `mic_gain` / `gain` | `GAIN <val>` | -40.0 dB to +40.0 dB (stored as `0.1*val`) |
| `lower_cutoff_frequency` / `lcutoff` | `LCUTOFF <val>` | 0 (disabled), 10 Hz to 2000 Hz |
| `upper_cutoff_frequency` / `ucutoff` | `UCUTOFF <val>` | 0 (disabled), 1000 Hz to Nyquist bound |
| `date_range` / `daterange` | `DATERANGE <start>-<end>` | Format: `YYYY/MM/DD-YYYY/MM/DD` |
| `time_of_day_0` .. `time_of_day_9` | `TIMEOFDAY <idx> <val>` | Strictly restricted to indices 0..9; format `HH:MM:SS-HH:MM:SS` |
| `time_of_day` / `timeofday` | `TIMEOFDAY <val>` | Ingests single range string |
| `days_of_week` / `daysofweek` | `DAYSOFWEEK <val>` | Mask format: `"MTWRFSU"` or `"X"` (all days) |
| `time_zone` / `timezone` | `TIMEZONE <val>` | Offset format: `[+-]HH:MM` or `[+-]HHMM` |
| `system_time` / `datetime` | `DATETIME <val>` | Calendar format: `YYYY-MM-DD HH:MM:SS` |
| `folder_structure` / `fldr` | `FLDR <val>` | `flat` (0), `daily` (1), `hourly` (2) |

### 2.2 Preserved Cellular & Hardware Setting Keys
Settings relating to legacy cellular modem hardware and solar tracking are classified, preserved in RAM, and maintained without breaking parsing:
- `transmission_mode` / `gsm`
- `location`
- `sms_frequency` / `smsfreq`
- `low_battery_threshold` / `lbat`
- `sun_recording_mode`
- `sunrise_interval_before`
- `sunrise_interval_after`
- `sunset_interval_before`
- `sunset_interval_after`
- `sunrise_sunset_rounding`

### 2.3 Status Metadata Keys
Status headers exported by status reports are recognized and safely ignored as metadata:
- `build_version`
- `build_datetime`
- `build_sha`

---

## 3. Importer Transactional Safety & Error Handling

To protect the file system and hardware configuration integrity during unattended field deployments, `config_store_import_sd()` guarantees transactional safety:

1. **Missing Newline at EOF:**
   Lines at the end of `config.ini` that lack a trailing `\n` or `\r\n` are parsed and applied completely.
2. **Oversized Line Protection:**
   Lines exceeding 127 bytes without a newline trigger a buffer-overflow guard: characters are safely discarded until the next newline, an error is reported, and partial strings are rejected.
3. **Transactional Rollback:**
   Before parsing begins, snapshots of `struct wildsonic_config` and `struct schedule_config` are cached. If any key contains invalid syntax, out-of-range values, or unrecognized options:
   - All staged RAM configurations roll back to the pre-import snapshot.
   - Pending flash writes are cancelled (`config_store_rollback_pending()`).
   - Flash NVS is **not overwritten**.
4. **Archive & Audit Trail:**
   - If import succeeds or fails, `config.ini` is archived to `config.bak` to prevent repeated re-import loops on every boot.
   - A human-readable diagnostic report is saved to `/SD:/config_status.txt` detailing all active parameters and import results.

