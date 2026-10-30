# WildSonic STM32 Wildlife Audio Recorder (Zephyr RTOS)

Developer Guide and Technical Manual for the **WildSonic STM32 Wildlife Audio Recorder** based on the **STM32 NUCLEO-L476RG** running **Zephyr RTOS v4.5.0-rc2**.

This guide provides step-by-step instructions for hardware assembly, development environment configuration, building production firmware, flashing, operating the recorder via the serial console, configuring recording sessions, and executing automated tests.

---

## 1. Hardware Setup

### 1.1 Development Board Overview
- **Board**: STMicroelectronics NUCLEO-L476RG
- **Microcontroller**: STM32L476RGT6 (ARM Cortex-M4 with FPU @ 72 MHz HCLK, 1 MB Flash, 96 KB SRAM1 + 32 KB SRAM2)
- **Host Interface**: Onboard ST-LINK/V2-1 via Micro-USB (Provides programming, SWD debug, and 115200 8N1 Virtual COM port)

### 1.2 Microphone Connection (MAX4466 Analog Breakout)
The analog front-end connects the MAX4466 electret microphone preamplifier to the internal 12-bit ADC1 on pin PA0.

| MAX4466 Pin | NUCLEO-L476RG Pin | MCU Port & Function | Description |
|---|---|---|---|
| **VCC** | **3V3** (CN6 pin 4) | 3.3 V Power Rail | Regulated 3.3 V analog supply rail |
| **GND** | **GND** (CN5 pin 6 / CN8 pin 7) | AGND / System Ground | Common analog ground |
| **OUT** | **Arduino A0** (CN8 pin 1) | **PA0** / ADC1_IN5 | Analog audio input centered on $+1.65\,\text{V}$ bias |

> [!CAUTION]
> **Electrical Safety Warning**: Connect MAX4466 `VCC` strictly to the **3V3 rail**. Do **NOT** connect to 5 V. Driving the analog pin above the MCU supply voltage ($V_{\text{DDA}} = 3.3\,\text{V}$) risks permanently damaging the STM32 analog input circuitry.

> [!TIP]
> **Analog Potentiometer Tuning**: Adjust the trimmer potentiometer on the back of the MAX4466 board so peak-to-peak audio spans between $200\,\text{mV}$ and $1500\,\text{mV}$. Use the `monitor 10` serial console command to observe live voltage levels in real time.

### 1.3 MicroSD Card Storage Connection (4-bit SDMMC1)
MicroSD storage uses the native 4-bit SDMMC1 controller mounted at `/SD:`, supporting FAT32 and exFAT filesystems:

| SD Card Signal | Morpho CN10 Header | MCU Pin | Alternate Function | Internal Configuration |
|---|---|---|---|---|
| **D0** | Morpho CN10 pin 14 | **PC8** | AF12 (SDMMC1_D0) | Internal pull-up enabled |
| **D1** | Morpho CN10 pin 16 | **PC9** | AF12 (SDMMC1_D1) | Internal pull-up enabled |
| **D2** | Morpho CN10 pin 12 | **PC10** | AF12 (SDMMC1_D2) | Internal pull-up enabled |
| **D3** | Morpho CN10 pin 10 | **PC11** | AF12 (SDMMC1_D3) | Internal pull-up enabled |
| **CLK** | Morpho CN10 pin 2 | **PC12** | AF12 (SDMMC1_CK) | 12 MHz bus clock |
| **CMD** | Morpho CN10 pin 4 | **PD2** | AF12 (SDMMC1_CMD) | Internal pull-up enabled |
| **VCC** | 3V3 (CN6 pin 4) | 3V3 Rail | Power | Regulated 3.3 V rail |
| **GND** | GND (CN6 pin 6) | GND | Ground | Common system ground |

### 1.4 Optional External PSRAM (ISSI IS66WVS4M8 on QUADSPI)
An optional 32 Mbit (4 MB) ISSI IS66WVS4M8 Pseudo-SRAM can be attached over Quad-SPI as an extended 1024 KB staging ring buffer:

| PSRAM Pin (SOIC-8) | Signal Name | NUCLEO Header | MCU Pin | Function |
|---|---|---|---|---|
| **Pin 1** | **CE#** (Chip Select) | Morpho CN10 pin 18 | **PB11** | QUADSPI_BK1_NCS ($t_{\text{CEM}} \le 4\,\mu\text{s}$) |
| **Pin 2** | **SO / SIO1** | Morpho CN7 pin 34 (A3) | **PB0** | QUADSPI_BK1_IO1 |
| **Pin 3** | **SIO2** | Morpho CN10 pin 15 (D11) | **PA7** | QUADSPI_BK1_IO2 |
| **Pin 4** | **VSS** | Morpho CN7 pin 20 (GND) | GND | Ground reference |
| **Pin 5** | **SI / SIO0** | Morpho CN10 pin 24 | **PB1** | QUADSPI_BK1_IO0 (Morpho pin 24 only; PA1 is Arduino A1) |
| **Pin 6** | **SCLK** | Morpho CN10 pin 25 (D6) | **PB10** | QUADSPI_CLK (18 MHz) |
| **Pin 7** | **SIO3** | Morpho CN10 pin 13 (D12) | **PA6** | QUADSPI_BK1_IO3 |
| **Pin 8** | **VDD** | Morpho CN7 pin 16 (3V3) | 3V3 | Regulated 3.3 V power rail |

> [!IMPORTANT]
> **Part Number Requirement**: Must use **IS66WVS4M8BLL** (3.3 V version). Do not use the 1.8 V "ALL" variant directly on 3.3 V NUCLEO rails.

Detailed wiring schematics and connector pinouts are provided in [`docs/HARDWARE_WIRING.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/HARDWARE_WIRING.md).

---

## 2. Development Environment Setup

### 2.1 Prerequisites and Verified Toolchain Versions
This project is built and verified with the following toolchain components:
- **Operating System**: Windows 10 / 11, Linux (Ubuntu 22.04+), or macOS
- **Zephyr RTOS**: v4.5.0-rc2
- **West Meta-Tool**: v1.5.0
- **Python**: 3.12 (with `pytest`, `numpy`, and `scipy`)
- **Cross-Compiler**: Zephyr SDK v1.0.1 (`arm-zephyr-eabi-gcc` 14.3.0) or GNU Arm Embedded Toolchain
- **Build System**: CMake 3.20.0 or newer and Ninja 1.10+

### 2.2 Zephyr Workspace Configuration
If you do not already have a Zephyr workspace, initialize one using West:

```powershell
# 1. Create and activate a Python virtual environment
python -m venv C:\Users\amith\zephyrproject\.venv
C:\Users\amith\zephyrproject\.venv\Scripts\Activate.ps1

# 2. Install West
pip install west

# 3. Initialize the Zephyr workspace
west init C:\Users\amith\zephyrproject
cd C:\Users\amith\zephyrproject
west update
west zephyr-export

# 4. Install Zephyr Python requirements
pip install -r C:\Users\amith\zephyrproject\zephyr\scripts\requirements.txt
```

### 2.3 Environment Variables
Before running builds, ensure the Zephyr SDK tools and toolchain binaries are on your `PATH`:

```powershell
# Windows PowerShell
$env:Path = "C:\ncs\toolchains\dcbdc366a1\opt\bin;" + $env:Path
$env:ZEPHYR_BASE = "C:\Users\amith\zephyrproject\zephyr"
```

Verify your environment:
```powershell
west --version
arm-zephyr-eabi-gcc --version
cmake --version
ninja --version
```

---

## 3. Workbench for Zephyr — VS Code Setup

This project uses **Workbench for Zephyr**, a Visual Studio Code extension, to manage the Zephyr development environment, toolchain, application builds, flashing, and debugging.

### Prerequisites

- Visual Studio Code
- Workbench for Zephyr extension by Ac6
- A Zephyr west workspace
- Zephyr SDK with the ARM toolchain
- STM32 NUCLEO-L476RG board and USB connection

### 3.1 Install Workbench

1. Open Visual Studio Code.
2. Open Extensions using `Ctrl+Shift+X`.
3. Search for **Workbench for Zephyr** by Ac6.
4. Install the extension.
5. Open the Workbench for Zephyr view from the Activity Bar.
6. Select **Install Host Tools** and allow the installation to complete.

Extension: [Workbench for Zephyr on VS Code Marketplace](https://marketplace.visualstudio.com/items?itemName=Ac6.zephyr-workbench)

### 3.2 Register the Zephyr SDK

In the Workbench view, open the toolchain management section.

1. Select the option to add or import a toolchain.
2. Select the existing Zephyr SDK installation if one is already available.
3. Otherwise, install a compatible Zephyr SDK and select its installation directory.
4. Verify that the ARM compiler is available for the selected toolchain.

Use the SDK version compatible with the project's Zephyr workspace and existing build configuration.

### 3.3 Register the west workspace

1. Open the Workbench for Zephyr view.
2. Select **Add West Workspace**.
3. Choose the option to use a local folder.
4. Select the existing west workspace root, which contains the `.west` directory.
5. Confirm that Workbench detects the Zephyr source tree and workspace version.

> [!IMPORTANT]
> Do not initialize another workspace when the existing workspace is intended for this project.

### 3.4 Import the STM32 recorder application

1. Open **Add Application** in Workbench.
2. Select **Import existing application**.
3. Select the registered west workspace.
4. Select the compatible Zephyr SDK toolchain.
5. Select board `nucleo_l476rg`.
6. Browse to the repository's `app/` directory.
7. Create the application entry.

The application directory must contain its Zephyr [`CMakeLists.txt`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/CMakeLists.txt) and [`prj.conf`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/prj.conf).

### 3.5 Configure the production build

Create a production build configuration for board `nucleo_l476rg`.

Set the build output directory to:

`build/prod`

Add the project's required CMake definition:

`-DSD_BUS=sdmmc`

Preserve the production configuration from [`app/prj.conf`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/prj.conf) and the project's existing board/device-tree configuration.

> [!WARNING]
> Use a separate output directory for every additional build configuration. Do not reuse the production build directory for diagnostic, integrity, synthetic-audio, or formatting builds.

### 3.6 Build the firmware

Select the production configuration in Workbench and run **Build**.

Wait for the build to complete. The build output is available in the VS Code integrated terminal.

### 3.7 Flash the Nucleo

1. Connect the NUCLEO-L476RG over USB.
2. Select the production build configuration.
3. Use Workbench's Flash action with the configured, supported flash runner.
4. Reset the board if required by the runner.
5. Open the serial terminal using the firmware's configured UART settings.

### 3.8 Debug the application

Use Workbench's Debug Manager to configure a supported debug server and the generated ELF file.

Select the appropriate ST-LINK or other supported runner, then start the debug session from VS Code.

Refer to the Workbench documentation for runner installation and debug configuration:

https://z-workbench.com/docs/documentation/

---

## 4. Build the Firmware

Build commands use Zephyr's `west` tool. Build artifacts are strictly isolated in independent output directories.

### 4.1 Production Build (Recommended)
Build the clean production recorder firmware (with all benchmark, synthetic, and replay code decoupled):

```powershell
# Execute from your Zephyr workspace directory or with ZEPHYR_BASE set:
west build -b nucleo_l476rg C:\Users\amith\Downloads\wildsonic_zephyr_stm32_recorder\app -d C:\Users\amith\Downloads\wildsonic_zephyr_stm32_recorder\build\prod -p always -- -DSD_BUS=sdmmc
```

**Build Parameters**:
- `-b nucleo_l476rg`: Target board definition for the STM32 NUCLEO-L476RG.
- `app`: Location of the application source directory and `CMakeLists.txt`.
- `-d build/prod`: Isolated output directory for production binaries.
- `-p always`: Pristine clean build.
- `-DSD_BUS=sdmmc`: Selects the high-throughput 4-bit native SDMMC1 storage overlay.

**Production Memory Footprint**:
- **Flash**: ~181.8 KB (17.35% of 1 MB)
- **RAM**: ~80.8 KB (82.23% of 96 KB SRAM1, allocating the 64 KB real-time audio block pool)
- **SRAM1 Aux**: 0 B (Reserved for normal application use)

---

### 4.2 Dedicated Test Builds
Each test configuration uses a dedicated build directory to prevent configuration collisions:

#### 1. Throughput Benchmarks Target
Enables `CONFIG_AUDIO_BENCHMARK=y` and compiles the complete RAM, PSRAM, and SD throughput benchmark suite:
```powershell
west build -b nucleo_l476rg app -d build/bench -p always -- -DSD_BUS=sdmmc "-DEXTRA_CONF_FILE=configs/test_benchmark.conf"
```

#### 2. Signal Stage Diagnostics Target
Enables `CONFIG_AUDIO_STAGE_DIAGNOSTICS=y` for real-time DSP stage tapping (`diagnose`) and Goertzel spectral isolation:
```powershell
west build -b nucleo_l476rg app -d build/diag -p always -- -DSD_BUS=sdmmc "-DEXTRA_CONF_FILE=configs/test_diagnostics.conf"
```

#### 3. Integrity Replay Target
Enables `CONFIG_AUDIO_INTEGRITY_REPLAY=y` for deterministic sample substitution, CRC audits, and WAV readback:
```powershell
west build -b nucleo_l476rg app -d build/integrity -p always -- -DSD_BUS=sdmmc "-DEXTRA_CONF_FILE=configs/test_integrity_replay.conf"
```

#### 4. Synthetic Tone Target
Bypasses physical ADC acquisition to test the SD storage pipeline using a deterministic sine generator:
```powershell
west build -b nucleo_l476rg app -d build/synth -p always -- -DSD_BUS=sdmmc "-DEXTRA_CONF_FILE=configs/test_synth.conf"
```

#### 5. SD Card Formatter Target
One-shot exFAT / FAT32 volume formatting utility:
```powershell
west build -b nucleo_l476rg app -d build/format -p always -- -DSD_BUS=sdmmc -DFORMAT_SD_CARD=ON "-DEXTRA_CONF_FILE=format_sd.conf"
```

---

## 5. Flash and Run

### 5.1 Flashing via West
Connect the NUCLEO-L476RG board to your computer via USB (CN1 connector) and flash:

```powershell
west flash -d build/prod
```

### 5.2 Drag-and-Drop Flashing (Alternative)
When plugged in, the NUCLEO board mounts as a USB mass storage drive named `NODE_L476RG`. Copy the compiled binary:
```powershell
copy build\prod\zephyr\zephyr.bin D:\
```
*(Replace `D:\` with the drive letter of your NUCLEO board).* The onboard ST-LINK LED (LD1) will flash red and green, and the board will automatically reboot upon completion.

### 5.3 Connecting to the Serial Console
Open a serial terminal connected to the ST-LINK Virtual COM port:
- **Baud Rate**: `115200`
- **Data Bits**: `8`
- **Parity**: `None`
- **Stop Bits**: `1`
- **Flow Control**: `None`

You can use PuTTY, Tera Term, minicom, or VS Code's integrated Serial Monitor. Press the black **RESET button (B2)** on the Nucleo to view the boot banner.

---

## 6. Operate the Recorder

Upon boot, the firmware initializes storage, configures the audio hardware, and launches the interactive serial REPL:

```
===================================================================
       WILDSONIC STM32L476RG WILDLIFE AUDIO RECORDER
       Firmware: Zephyr RTOS v4.5.0-rc2 | Arch: ARM Cortex-M4 @ 72 MHz
===================================================================
Interactive console ready. Type 'help' for commands, 'config' for guided setup.

>
```

### 6.1 Production Command Reference

| Command | Arguments | Description / Example |
|---|---|---|
| `help` / `?` | None | Display the interactive command menu. |
| `show` | None | Display the active configuration profile, expected file size, and SD bandwidth. |
| `status` | None | Print real-time system status, DMA callbacks, clipping counts, and register dumps. |
| `config` | None | Launch the guided 7-step interactive configuration wizard. |
| `config psram <on\|off>` | `on`, `off` | Enable or disable the 1024 KB PSRAM circular staging buffer. |
| `start` | None | Start recording session(s) with the current active profile. |
| `stop` / `q` | None | Abort an active recording or sleep phase immediately. |
| `rate <Hz>` | `16000`, `32000`, `44100`, `48000`, `96000`, `192000` | Set sampling rate (e.g. `rate 192000` or shorthand `rate 192`). |
| `record <seconds>`| `1` to `86400` | Set recording session duration in seconds (e.g. `record 60`). |
| `sleep <seconds>` | `0` to `86400` | Set low-power sleep interval between sessions (e.g. `sleep 30`). |
| `cycles <count>` | `0` to `10000` | Set session repetition count (`0` = continuous autonomous recording). |
| `gain <multiplier>`| `0.1` to `16.0` | Set software digital multiplier (e.g. `gain 2.0` for $+6\,\text{dB}$). |
| `filter on` | None | Enable the production 5-stage DSP filter cascade. |
| `filter raw` | None | Bypass all DSP filters (direct verbatim ADC PCM). |
| `filter notch <50\|60\|off>`| `50`, `60`, `off` | Set mains hum notch filter frequency. |
| `filter hpf <Hz>`| `10` to `2000` | Set High-Pass Filter cutoff frequency (e.g. `filter hpf 80`). |
| `filter lpf <Hz>`| `1000` to `40000`| Set Low-Pass Anti-Aliasing cutoff (e.g. `filter lpf 20000`). |
| `mode <normal\|raw>` | `normal`, `raw` | Select production filtered or raw ADC mode. |
| `time show` | None | Display current RTC calendar date and time. |
| `time set <str>` | `YYYY-MM-DD HH:MM:SS` | Initialize hardware RTC calendar (e.g. `time set 2026-10-24 12:00:00`). |
| `psram <on\|off\|status\|burst <B>>` | `on`, `off`, `status`, `burst <B>` | Configure PSRAM circular buffer and transaction burst sizes. |
| `monitor [sec]` | `1` to `3600` | Run live 1 Hz microphone diagnostic meter (min/max/span/RMS/dBFS). |
| `default` | None | Reset all settings to factory defaults. |

### 6.2 Standard Recording Workflow

1. **Check Status**:
   ```
   > status
   ```
   Verifies that `/SD:` is mounted and shows the current sample rate and buffer health.

2. **Configure Parameters**:
   ```
   > rate 192000
   > record 60
   > sleep 60
   > cycles 5
   > gain 1.0
   ```
   Sets 192 kHz recording for 60 seconds, followed by 60 seconds in low-power Stop 2 sleep, repeated for 5 cycles.

3. **Verify Microphone Levels**:
   ```
   > monitor 10
   ```
   Runs a 10-second real-time level check. Speak into the microphone and ensure the reported span is between $200\,\text{mV}$ and $1500\,\text{mV}$ with zero rail clipping.

4. **Initiate Recording**:
   ```
   > start
   ```
   The green user LED (LD2 / PA5) turns on during recording. A real-time session progress banner displays write latency, buffer queue depth, and written block counts.

5. **Stop Early (Optional)**:
   Press `q` or type `stop` at any time to abort the recording session or sleep interval cleanly.

6. **Retrieve Audio**:
   Remove the microSD card and insert it into your computer. WAV files are located in the root directory named sequentially:
   - `REC0001.WAV`
   - `REC0002.WAV`

---

## 7. Recording Configuration

### 7.1 Supported Sampling Frequencies
The STM32L476RG hardware timer (TIM6 TRGO) triggers ADC1 conversions with zero jitter:

| Nominal Rate | Actual Hardware Rate | TIM6 Period Divider (from 80 MHz clock) | Deviation | Acoustic Application |
|---|---|---|---|---|
| **16,000 Hz** | 16,000.0 Hz | 5000 | 0.0 Hz (0.00%) | Speech and low-frequency wildlife calls |
| **32,000 Hz** | 32,000.0 Hz | 2500 | 0.0 Hz (0.00%) | General bioacoustics |
| **44,100 Hz** | 44,101.4 Hz | 1814 | $+1.4\,\text{Hz}$ ($+0.003\%$) | Standard CD audio rate |
| **48,000 Hz** | 48,000.0 Hz | 1666 | $-1.9\,\text{Hz}$ ($-0.004\%$) | Professional acoustic monitoring |
| **96,000 Hz** | 96,000.0 Hz | 833 | $+3.8\,\text{Hz}$ ($+0.004\%$) | Extended frequency bandwidth |
| **192,000 Hz**| 192,000.0 Hz | 416 | $-307\,\text{Hz}$ ($-0.16\%$) | Full-bandwidth ultrasound & throughput testing |

### 7.2 PCM Audio Formats
- **16-bit Signed Integer PCM** (Format Tag 1, Mono): Standard 2 bytes per sample. Baseline production profile producing $384\,\text{kB/s}$ throughput at 192 kHz.
- **32-bit Signed Integer PCM** (Format Tag 1, Mono): 4 bytes per sample container for high-throughput stress testing ($768\,\text{kB/s}$ at 192 kHz).

### 7.3 DSP Filtering Pipeline
The real-time DSP pipeline executes in a high-priority cooperative thread using Direct Form II Transposed biquad filters:
1. **DC Bias Blocker**: 1-pole high-pass filter with corner at 20 Hz, eliminating $+1.65\,\text{V}$ microphone DC bias.
2. **High-Pass Filter**: 2nd-order Butterworth biquad (default 80 Hz, $Q = 0.7071$).
3. **Mains Hum Notch**: 2nd-order notch biquad targeting 50 Hz or 60 Hz ($Q = 10.0$).
4. **Harmonic Notch**: 2nd-order notch biquad targeting 100 Hz or 120 Hz ($Q = 10.0$).
5. **Anti-Aliasing Low-Pass Filter**: 4th-order Butterworth filter cascade with 20 kHz corner frequency ($-24\,\text{dB/octave}$).
6. **Digital Gain**: User-selectable multiplier ($0.1\times$ to $16.0\times$) featuring saturating arithmetic to eliminate integer overflow.

---

## 8. Automated Testing

The repository provides two separate test domains: automated host-side regression tests that run on your computer, and hardware integration tests that execute on physical target hardware.

### 8.1 Host Regression Test Suite (No Hardware Required)
Ten comprehensive regression test suites in `tests/host/` validate digital filter responses, WAV header parsing, PSRAM staging concurrency, and RTC lifecycle logic:

```powershell
# 1. Install test dependencies
pip install -r tests/host/requirements.txt

# 2. Run all regression tests via pytest
pytest tests/host/
```

Or execute individual test suites directly with Python:
```powershell
python tests/host/test_wav_tools.py
python tests/host/test_dsp_pipeline.py
python tests/host/test_dsp_frequency_response.py
python tests/host/test_diagnostic_performance.py
python tests/host/test_integrity_replay.py
python tests/host/test_psram_staging_and_rtc.py
python tests/host/test_psram_staging_concurrency.py
python tests/host/test_psram_quad_patterns_and_isolation.py
python tests/host/test_psram_integrity_and_benchmarks.py
python tests/host/test_rtc_wake_and_start_lifecycle.py
```

### 8.2 Hardware Integration Tests (Requires NUCLEO Board)
Hardware integration tests require a connected NUCLEO-L476RG board and run through dedicated firmware builds:
1. **Benchmark Suite**: Flash `build/bench` and run `bench` commands (`bench ram`, `bench psram`, `bench sd`, `bench pipeline 192000`).
2. **Deterministic Replay Audit**: Flash `build/integrity` and execute `integrity replay 5` to audit DMA/SD write pipeline CRC against expected reference math.
3. **Real-Time Signal Diagnostics**: Flash `build/diag` and execute `diagnose 5` to perform intermediate stage signal tapping and Goertzel spectral isolation.

For test procedures, refer to [`tests/hw_integration/README.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/tests/hw_integration/README.md).

---

## 9. Power Measurement and Low-Power Operation

### 9.1 Autonomous Power-Cycle Operation
Between scheduled recording sessions, the firmware powers down all non-essential peripherals and places the STM32L476 in **Stop 2 Mode**:
- **Clock**: 72 MHz PLL stopped.
- **Peripherals**: TIM6 halted, ADC1 powered down, DMA1 disabled, SDMMC bus clock halted, PSRAM placed in standby.
- **Retention**: Full SRAM1, SRAM2, and RTC calendar registers retained.
- **Wakeup**: Internal RTC Wakeup Timer or blue User Button B1 (PC13).

### 9.2 Current Measurement Procedure
Because the NUCLEO-L476RG does not include an onboard current shunt monitor, measurements must be taken with an external multimeter:

1. **MCU Core Current Only ($I_{\text{MCU}}$)**:
   - Power the board via USB (CN1).
   - Remove jumper **JP6 (IDD)** on the NUCLEO-L476RG.
   - Connect a digital multimeter (microamp/milliamp DC range) across the two pins of **JP6**.
   - *Expected*: Active Recording ~12 to 16 mA @ 72 MHz; Stop 2 Sleep ~1.5 to 3 µA.

2. **Total System Current ($I_{\text{system}}$)**:
   - Supply external regulated 5 V to **E5V** (CN7 pin 6) and **GND** (CN7 pin 8).
   - Move jumper **JP5** to select `E5V` (pins 3-4).
   - **Isolate ST-LINK**: Remove both jumpers on **CN2** and disconnect the USB cable from CN1 to ensure the ST-LINK debugger does not draw power from the target rail.
   - Connect the multimeter in series with the external +5 V supply lead.
   - *Expected*: Active ~25 to 50 mA (dominated by SD flash write current spikes); Standby ~0.4 to 0.8 mA.

---

## 10. Repository and Development Workflow

### 10.1 Directory Layout
```
wildsonic_zephyr_stm32_recorder/
├── app/                         # Core Zephyr application
│   ├── boards/                  # Nucleo L476RG board devicetree overlays
│   ├── configs/                 # Modular test and diagnostic configuration files
│   ├── sd/                      # SDMMC and SPI storage devicetree overlays
│   ├── src/                     # C application source files
│   │   ├── diagnostics/         # Isolated benchmarks and diagnostic suites
│   │   ├── audio_in_stm32l4.c   # Hardware-timed ADC/DMA audio acquisition
│   │   ├── dsp.c                # 5-stage biquad DSP pipeline
│   │   ├── recorder.c           # WAV writer engine and session coordinator
│   │   └── psram_is66wvs.c      # Quad-SPI external PSRAM driver
│   ├── CMakeLists.txt           # Application build definitions
│   ├── Kconfig                  # Application configuration symbol menu
│   └── prj.conf                 # Default production configuration
├── docs/                        # Technical documentation & reference manuals
│   ├── ARCHITECTURE.md          # Real-time pipeline, threading, and DSP math
│   ├── HARDWARE_WIRING.md       # Pin mapping, decoupling, and electrical specs
│   ├── BENCHMARK_VALIDATION.md  # PSRAM and SD throughput characterization
│   └── FORMAT_SD_CARD.md        # exFAT and FAT32 microSD formatting guide
├── tests/
│   ├── host/                    # Automated host regression tests (pytest)
│   └── hw_integration/          # Target-specific hardware integration tests
├── .gitignore                   # Git exclusion rules
└── README.md                    # Primary developer guide (this file)
```


### 10.3 Reference Documentation
- **Architecture & Pipeline**: [`docs/ARCHITECTURE.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/ARCHITECTURE.md)
- **Hardware & Wiring**: [`docs/HARDWARE_WIRING.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/HARDWARE_WIRING.md)
- **SD Card Formatting**: [`docs/FORMAT_SD_CARD.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/FORMAT_SD_CARD.md)
- **Benchmark Analysis**: [`docs/BENCHMARK_VALIDATION.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/BENCHMARK_VALIDATION.md)

2. **Total System Current ($I_{\text{system}}$)**:
   - Supply external regulated 5 V to **E5V** (CN7 pin 6) and **GND** (CN7 pin 8).
   - Move jumper **JP5** to select `E5V` (pins 3-4).
   - **Isolate ST-LINK**: Remove both jumpers on **CN2** and disconnect the USB cable from CN1 to ensure the ST-LINK debugger does not draw power from the target rail.
   - Connect the multimeter in series with the external +5 V supply lead.
   - *Expected*: Active ~25 to 50 mA (dominated by SD flash write current spikes); Standby ~0.4 to 0.8 mA.

---

