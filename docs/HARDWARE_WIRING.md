# Hardware Wiring and Interface Specification

This document provides detailed electrical wiring specifications, pin assignments, signal characteristics, and physical interface considerations for the WildSonic STM32L476RG Audio Recorder.

## BQ34110 Battery Gauge (External Wiring Required)

The firmware configures the NUCLEO-L476RG I2C1 controller for standard-mode
100 kHz on PB8/PB9 (AF4), with the BQ34110 at 7-bit address `0x55`.

| BQ34110 signal | NUCLEO-L476RG connection | Notes |
|---|---|---|
| SDA | D14 / CN5-9 / PB9 | I2C1 SDA, AF4 |
| SCL | D15 / CN5-10 / PB8 | I2C1 SCL, AF4 |
| GND | GND | Common ground required |
| VCC | A compatible regulated supply | Check the actual gauge module and BQ34110 reference design before wiring. Do not infer this from the board's 3V3 header. |
| BAT / battery sense | Battery pack per the BQ34110 reference design | This is a gauge connection; do not connect the pack directly to an STM32 GPIO or ADC. |

PB8/PB9 are not used by the audio, USART1 BLE, SDMMC, or documented optional
PSRAM pin maps. Zephyr's NUCLEO-L476RG DTS already defines the I2C1 PB8/PB9
AF4 pinctrl. The ST board documentation maps PB8/PB9 to the D15/D14 headers.
This firmware uses those digital header pins; the A4/A5 solder-bridge option is
not needed.

The NUCLEO alone does not include the external BQ34110 gauge. The project's
available wiring documents do not establish the gauge module's supply, BAT
input circuit, pull-up resistors, or whether its supply remains on during Stop 2.
Before connecting hardware, inspect the actual gauge module schematic and
measure/confirm existing SDA/SCL pull-ups. If absent, fit pull-ups to the
compatible logic rail according to the BQ34110 electrical requirements and bus
capacitance; do not add parallel pull-ups without checking their combined
resistance. Gauge presence, pack configuration, and Stop 2 power retention have
not been hardware-validated.

---

## 1. Development Platform Overview

- **Host Board**: STMicroelectronics NUCLEO-L476RG
- **Target MCU**: STM32L476RGT6 (ARM Cortex-M4 with FPU @ 72 MHz HCLK)
- **Primary Connectors**:
  - Arduino Uno V3 Compatible Connectors (CN5, CN6, CN8, CN9)
  - ST Morpho Extension Pin Headers (CN7, CN10) for full MCU access
- **Power Delivery**: USB 5 V (CN1), External 5 V (E5V), or External 7–12 V (VIN)

---

## 2. Analog Microphone Wiring (MAX4466 Electret Breakout)

The MAX4466 breakout integrates an electret capsule with an adjustable-gain operational amplifier.

### Signal Connections
| Breakout Pin | NUCLEO Header | MCU Pin | Function / Type | Electrical Characteristics |
|---|---|---|---|---|
| **VCC** | CN6 pin 4 (3V3) | 3V3 Rail | Regulated Power | +3.3 V DC ($\pm 3\%$). Max draw ~24 µA quiescent. |
| **GND** | CN5 pin 6 / CN8 pin 7 | AGND / GND | Analog Ground | Return reference to analog ground plane. |
| **OUT** | CN8 pin 1 (A0) | **PA0** | ADC1_IN5 (Analog In) | AC audio centered on $+1.65\,\text{V}$ DC bias ($V_{\text{CC}}/2$). |

### Electrical Safety and Best Practices
1. **Supply Voltage Warning**:
   > [!CAUTION]
   > Do **NOT** connect the MAX4466 `VCC` pin to the +5 V rail. The STM32L476 analog inputs are rated for $V_{\text{DDA}}$ (+3.3 V). Supplying +5 V to the microphone risks driving the output pin above $V_{\text{DDA}} + 0.3\,\text{V}$, causing permanent damage to the MCU analog input stages.
2. **Analog Noise Decoupling**:
   - Place a **$100\,\mu\text{F}$ electrolytic capacitor** in parallel with a **$0.1\,\mu\text{F}$ ceramic capacitor** directly across `VCC` and `GND` at the MAX4466 breakout terminals.
   - For optimal noise rejection from microSD flash programming surges, insert a low-value series resistor ($22\,\Omega$ to $47\,\Omega$) between NUCLEO 3V3 and MAX4466 `VCC` to form an RC low-pass filter.
   - Connect the microphone ground lead directly to the **AGND** pin of the NUCLEO board to isolate audio return currents from digital ground return paths.
3. **Trimmer Potentiometer Adjustment**:
   - The trimmer potentiometer on the back of the MAX4466 board sets the op-amp analog gain from 25x to 125x.
   - Adjust the trimmer so that normal speech or target ambient sound yields a peak-to-peak span between $200\,\text{mV}$ and $1500\,\text{mV}$ (use the `monitor 10` UART command to inspect live peak-to-peak voltage). Avoid over-amplification that causes clipping at ground (0 V) or 3.3 V.

---

## 3. MicroSD Card Storage Wiring (4-bit SDMMC1)

The storage backend uses native 4-bit SDMMC1 operating with internal pull-ups and DMA data transfers.

### Pin Mapping (Morpho CN10 Header)
| SD Card Pin | Signal Name | NUCLEO Morpho | MCU Pin | Alternate Function | Internal Configuration |
|---|---|---|---|---|---|
| **D0** | SDMMC1_D0 | CN10-14 | **PC8** | AF12 | Pull-up enabled, high speed |
| **D1** | SDMMC1_D1 | CN10-16 | **PC9** | AF12 | Pull-up enabled, high speed |
| **D2** | SDMMC1_D2 | CN10-12 | **PC10** | AF12 | Pull-up enabled, high speed |
| **D3** | SDMMC1_D3 | CN10-10 | **PC11** | AF12 | Pull-up enabled, high speed |
| **CLK** | SDMMC1_CK | CN10-2 | **PC12** | AF12 | 12 MHz bus clock |
| **CMD** | SDMMC1_CMD | CN10-4 | **PD2** | AF12 | Pull-up enabled, high speed |
| **VCC** | 3V3 | CN6-4 / CN7-16| 3V3 | Power | Regulated 3.3 V rail |
| **GND** | GND | CN6-6 / CN7-20| GND | Ground | 0 V Reference |

### Storage Hardware Considerations
- **MicroSD Card Types**: Standard microSD, microSDHC, and microSDXC cards up to 256 GB are supported.
- **Trace Lengths**: Keep SD bus wires under 10 cm (4 inches) to minimize transmission line ringing at the 12 MHz clock rate.
- **Filesystem**: Cards must be formatted with **FAT32** or **exFAT** (see [`docs/FORMAT_SD_CARD.md`](file:///C:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/docs/FORMAT_SD_CARD.md)).

---

## 4. External PSRAM Wiring (ISSI IS66WVS4M8 on QUADSPI)

The firmware supports an optional 32 Mbit (4 MByte) SPI/QPI Pseudo-SRAM as a circular staging ring buffer.

### Device Selection
> [!IMPORTANT]
> Must use **IS66WVS4M8BLL** (3.3 V nominal operating range: 2.7 V to 3.6 V). Do **NOT** use the 1.8 V "ALL" variant directly on 3.3 V NUCLEO rails without level translation.

### Pin Mapping (Morpho and Arduino Connectors)
| PSRAM Pin (SOIC-8) | Signal Name | NUCLEO Header | MCU Pin | Alternate Function | Notes |
|---|---|---|---|---|---|
| **Pin 1** | **CE#** (Chip Enable) | Morpho CN10-18 | **PB11** | AF10 (QUADSPI_BK1_NCS) | Active low; wire time constrained by driver to $t_{\text{CEM}} \le 4\,\mu\text{s}$ |
| **Pin 2** | **SO / SIO1** | Morpho CN7-34 / A3 | **PB0** | AF10 (QUADSPI_BK1_IO1) | Data I/O line 1 |
| **Pin 3** | **SIO2** | Morpho CN10-15 / D11 | **PA7** | AF10 (QUADSPI_BK1_IO2) | Data I/O line 2 |
| **Pin 4** | **VSS** | Morpho CN7-20 / GND | GND | Ground | Ground reference |
| **Pin 5** | **SI / SIO0** | Morpho CN10-24 | **PB1** | AF10 (QUADSPI_BK1_IO0) | Data I/O line 0 (Morpho pin 24 only; Arduino A1 is PA1, NOT PB1) |
| **Pin 6** | **SCLK** | Morpho CN10-25 / D6 | **PB10** | AF10 (QUADSPI_CLK) | 18 MHz bus clock (72 MHz HCLK / 4) |
| **Pin 7** | **SIO3** | Morpho CN10-13 / D12 | **PA6** | AF10 (QUADSPI_BK1_IO3) | Data I/O line 3 |
| **Pin 8** | **VDD** | Morpho CN7-16 / 3V3 | 3V3 | Power | +3.3 V Power Rail |

### Timing and Interface Constraints
- **Maximum CE-Low Duration ($t_{\text{CEM}}$)**: The IS66WVS internal refresh architecture requires chip-enable high periods. The driver strictly limits SPI bursts to 4 bytes and Quad bursts to 16 bytes to guarantee bus tenure remains safely below the $4.0\,\mu\text{s}$ datasheet limit at 18 MHz.
- **Quad Mode Verification**: The firmware performs an automated multi-pattern cross-mode memory self-test on boot. If Quad I/O is not verified, the recorder falls back safely to internal SRAM queueing.

---

## 5. Serial Console (UART)

The primary user interface and configuration console operates via the onboard ST-LINK Virtual COM port:
- **TX Pin**: PA2 (USART2_TX) -> ST-LINK RX (automatic via solder bridges SB13/SB14)
- **RX Pin**: PA3 (USART2_RX) -> ST-LINK TX (automatic via solder bridges SB62/SB63)
- **Baud Rate**: 115200 bps
- **Format**: 8 Data bits, No Parity, 1 Stop bit (8N1), No hardware flow control
- **Connector**: Micro-USB port CN1 on the NUCLEO-L476RG board

