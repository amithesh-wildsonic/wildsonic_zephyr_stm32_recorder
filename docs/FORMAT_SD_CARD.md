# One-Shot exFAT Formatter for NUCLEO-L476RG

This utility provides a dedicated, one-shot formatting firmware that creates a clean **exFAT** filesystem over SDMMC on the target microSD card. It is built as a separate standalone image and never changes normal recording logic.

---

## 1. Safety & Data Protection Rules

> [!CAUTION]
> **DESTRUCTIVE OPERATION**: Formatting completely erases the inserted microSD card and creates a fresh exFAT super-floppy volume (`FM_EXFAT | FM_SFD`). All existing data, partition tables, and files will be permanently lost.

1. **Never Automatic at Boot**: The production recorder (`build/zephyr/zephyr.bin`) has `CONFIG_FS_FATFS_MOUNT_MKFS=n` and `CONFIG_FS_FATFS_MKFS=n`. It will **never** format a card automatically if mounting fails.
2. **Explicit Formatter Image Required**: Formatting can **only** be triggered by deliberately compiling with `-DFORMAT_SD_CARD=ON` and flashing the formatter binary (`build_fmt/zephyr/zephyr.bin`).
3. **Card Capacity Safety Verification**:
   The formatter queries the SD controller via `disk_access_ioctl` for the exact sector count and sector size. It verifies this against a pre-configured target card profile before writing any sectors.
   - If the detected capacity does **not** match the configured expected capacity, formatting is **immediately aborted** with a `SAFETY STOP` message.
   - The formatter will **never automatically adopt any detected capacity as trusted**.
   - You must physically check the printed label on your microSD card to confirm its true capacity before selecting the profile.

---

## 2. Card Capacity Profiles

Configure the target profile in `format_sd.conf` or via Kconfig:

| Profile Option | Sector Count | Capacity (Bytes) | Capacity (Decimal GB) | Typical Card |
|---|---|---|---|---|
| **`CONFIG_FORMAT_CARD_32GB=y`** (Default) | `62,333,952` | `31,914,983,424` B | **~31.91 GB** (29.72 GiB) | Standard 32 GB microSDHC |
| **`CONFIG_FORMAT_CARD_64GB=y`** | `124,735,488` | `63,864,569,856` B | **~63.86 GB** (59.48 GiB) | Standard 64 GB microSDXC |
| **`CONFIG_FORMAT_CARD_CUSTOM=y`** | User specified | `CONFIG_FORMAT_CUSTOM_SECTOR_COUNT` $\times$ 512 | Custom | Special cards |

---

## 3. Step-by-Step Formatting Workflow

Assuming PowerShell is in the Zephyr virtual environment (`C:\Users\amith\zephyrproject\.venv`), toolchain on PATH, and `$project = "C:\Users\amith\Downloads\wildsonic_zephyr_stm32_recorder\wildsonic_zephyr"`:

### Step 1: Build the exFAT Formatter Firmware
Build into a separate directory (`build_fmt`) so production build artifacts are preserved:

```powershell
$env:Path = "C:\ncs\toolchains\dcbdc366a1\opt\bin;" + $env:Path
& "C:\Users\amith\zephyrproject\.venv\Scripts\west.exe" build -b nucleo_l476rg "$project" -d "$project\build_fmt" --extra-conf "$project\format_sd.conf" -- -DSD_BUS=sdmmc -DFORMAT_SD_CARD=ON
```

### Step 2: Flash the Formatter Image
```powershell
& "C:\Users\amith\zephyrproject\.venv\Scripts\west.exe" flash -d "$project\build_fmt"
```

### Step 3: Monitor Serial Console
Open your serial monitor at 115200 baud (e.g. PuTTY, miniterm, or VS Code serial monitor):
- **If capacity matches**: The formatter will display:
  ```
  Target Configuration:
    Filesystem Type    : exFAT (FM_EXFAT | FM_SFD)
    Expected Card Model: 62333952 sectors x 512 bytes (~31.91 GB)
  Detected Card:
    Detected Sectors   : 62333952 sectors x 512 bytes (~31.91 GB)
  Capacity verified: 62333952 sectors (~31.91 GB) matches expected profile.
  Formatting SD card as exFAT now... LEAVE POWER CONNECTED.
  FORMAT_SUCCESS: exFAT volume creation completed successfully!
  ```
- **If capacity does not match**:
  ```
  SAFETY STOP: Card capacity mismatch detected! Formatting ABORTED.
    Expected: 124735488 sectors (~63.86 GB)
    Detected: 62333952 sectors (~31.91 GB)
  ```
  The board halts safely without touching the card. Verify the physical card label and adjust `format_sd.conf`.

### Step 4: Immediately Flash Production Recorder Firmware
> [!IMPORTANT]
> Because `format_sd.c` runs on boot, do not press reset with the formatter installed. Immediately restore the production recorder image:

```powershell
& "C:\Users\amith\zephyrproject\.venv\Scripts\west.exe" build -b nucleo_l476rg "$project" -d "$project\build" -- -DSD_BUS=sdmmc
& "C:\Users\amith\zephyrproject\.venv\Scripts\west.exe" flash -d "$project\build"
```

The production recorder will mount the fresh exFAT card, display `SD mounted: exFAT filesystem`, and record 192 kHz WAV files.
