# WildSonic STM32 Architecture Specification

This document details the software architecture, real-time concurrency model, memory partitioning, digital signal processing (DSP) pipeline, and filesystem mechanics of the WildSonic STM32L476RG Audio Recorder.

---

## 1. System Architecture Overview

The firmware is designed around a multi-threaded, zero-copy producer-consumer model running on **Zephyr RTOS v4.5.0-rc2**.

```
  [MAX4466 Analog Mic]
          │
          ▼
   PA0 (ADC1_IN5)
          │
   TIM6 TRGO Trigger ──────────┐
   (16k - 192k Hz)             │
                               ▼
                        [STM32L4 ADC1]
                               │
                        [DMA1 Channel 1]
                     Circular Ping-Pong Buffer
                        (2048 samples/half)
                               │
                ┌──────────────┴──────────────┐
                ▼ Half-Transfer (HT)          ▼ Transfer-Complete (TC)
         k_msgq_put(dma_event_q)       k_msgq_put(dma_event_q)
                               │
                               ▼
                   ┌───────────────────────┐
                   │   Audio DSP Thread    │ (Priority: K_PRIO_COOP(1))
                   │  - 5-stage Biquad DSP │
                   │  - Saturating Gain    │
                   └───────────┬───────────┘
                               │ k_msgq_put
                               ▼
        ┌──────────────────────────────────────────────┐
        │        Buffer Pool / Circular Staging        │
        │  Path A: Internal SRAM1 Pool (16 x 4 KB)     │
        │  Path B: ISSI IS66WVS4M8 PSRAM (1024 KB ring)│
        └──────────────────────┬───────────────────────┘
                               │ k_msgq_get
                               ▼
                   ┌───────────────────────┐
                   │   SD Writer Thread    │ (Priority: K_PRIO_PREEMPT(8))
                   │  - Batch Sector Write │
                   │  - Monotonic 64-bit   │
                   └───────────┬───────────┘
                               │
                               ▼
                      [microSD Card (/SD:)]
                     Native 4-bit SDMMC Bus
```

---

## 2. Real-Time Concurrency and Thread Model

The application isolates time-critical hardware signal capture from storage I/O using cooperative and preemptive threads with distinct priorities:

| Thread | Priority | Stack Size | Function |
|---|---|---|---|
| **DMA ISR** | Hardware IRQ 11 | N/A | Captures Half-Transfer (HT) and Transfer-Complete (TC) interrupts; pushes event timestamps into cooperative queue. |
| **Audio Acquisition Thread** | `K_PRIO_COOP(1)` | 2048 B | Dequeues DMA half-buffers, runs saturating conversions, passes blocks to DSP pipeline. |
| **Audio DSP Thread** | `K_PRIO_COOP(2)` | 4096 B | Executes the 5-stage Direct Form II Transposed biquad filter chain and digital gain multiplier. |
| **SD Writer Thread** | `K_PRIO_PREEMPT(8)`| 8192 B | Handles FatFs filesystem operations, multi-sector SD card writes, and file header finalization. |
| **Main / REPL Thread** | `K_PRIO_PREEMPT(10)`| 4096 B | Drives serial console, parses user commands, and coordinates session lifecycles. |

### Zero-Copy Memory Management
- **Internal RAM Pool**: Defined via `k_mem_slab` (`pool`) in internal SRAM1. Consists of 16 blocks of 2048 samples each (64 KB total pool size).
- At 192 kHz (384 kB/s data rate), 16 blocks provide **170.6 ms** of internal buffer depth, absorbing SD card write latency spikes during flash block erase/re-allocation cycles.
- When optional PSRAM staging is enabled, blocks are staged through a **1024 KB circular ring buffer** in external PSRAM, providing **2.73 seconds** of buffer headroom.

---

## 3. Digital Signal Processing (DSP) Pipeline

The audio engine processes 16-bit or 32-bit PCM audio in-place using double-precision coefficients and Direct Form II Transposed (DF2T) structures for numerical stability:

```
Raw ADC Input ──► [Stage 1] ──► [Stage 2] ──► [Stage 3] ──► [Stage 4] ──► [Stage 5] ──► [Stage 6] ──► Output
                   DC Bias       80 Hz HPF     50/60 Hz      Harmonic      20 kHz LPF    Saturating
                   Blocker      Butterworth     Notch         Notch        Butterworth     Gain
```

### Stage Details
1. **Stage 1: DC Bias Blocker**:
   - Single-pole high-pass IIR with corner at ~20 Hz:
     $$y[n] = x[n] - x[n-1] + R \cdot y[n-1], \quad R = 0.9995$$
   - Removes the $+1.65\,\text{V}$ DC offset injected by the analog microphone preamplifier.
2. **Stage 2: High-Pass Filter (HPF)**:
   - 2nd-order Butterworth biquad filter with configurable cutoff (default 80 Hz, $Q = 0.7071$).
   - Rejects mechanical wind noise and sub-audible handling vibrations.
3. **Stage 3: Mains Hum Notch Filter**:
   - 2nd-order notch biquad at 50 Hz or 60 Hz with configurable $Q$ factor (default $Q = 10.0$).
   - Rejects electromagnetic coupling from power lines.
4. **Stage 4: Harmonic Mains Notch Filter**:
   - 2nd-order notch biquad targeting the 2nd harmonic (100 Hz or 120 Hz, $Q = 10.0$).
5. **Stage 5: Low-Pass Filter (LPF)**:
   - 4th-order Butterworth filter (two cascaded 2nd-order biquad stages) with cutoff at 20 kHz.
   - Attenuates ultrasonic noise and digital switching hash.
6. **Stage 6: Software Digital Gain**:
   - User-configurable multiplier from $0.1\times$ to $16.0\times$ ($-\!20\,\text{dB}$ to $+24\,\text{dB}$).
   - Employs saturating arithmetic (`__SSAT`) to prevent integer wraparound under loud acoustic peaks.

---

## 4. Storage Architecture and WAV File Structure

### Filesystem Integration
- Filesystem mounted at `/SD:` using Zephyr's virtual filesystem (VFS) and FatFs backend.
- Supports both **FAT32** (clusters up to 32 KB) and **exFAT** (clusters up to 128 KB for SDXC cards $>32\,\text{GB}$).
- Write operations are dispatched in aligned multi-sector blocks (minimum 4096 bytes / 8 sectors) to maximize write bandwidth.

### WAV Container Header
Each recording session generates a canonical RIFF/WAVE mono PCM file (`REC%04u.WAV`):

| Offset | Field Name | Size | Value / Description |
|---|---|---|---|
| 0x00 | `ChunkID` | 4 B | `"RIFF"` (0x46464952 little-endian) |
| 0x04 | `ChunkSize` | 4 B | $36 + \text{PCM Data Bytes}$ |
| 0x08 | `Format` | 4 B | `"WAVE"` (0x45564157 little-endian) |
| 0x0C | `Subchunk1ID` | 4 B | `"fmt "` (0x20746D66 little-endian) |
| 0x10 | `Subchunk1Size`| 4 B | 16 (Standard PCM subchunk size) |
| 0x14 | `AudioFormat` | 2 B | 1 (Linear PCM, uncompressed) |
| 0x16 | `NumChannels` | 2 B | 1 (Mono) |
| 0x18 | `SampleRate` | 4 B | 192000 (or configured rate in Hz) |
| 0x1C | `ByteRate` | 4 B | $\text{SampleRate} \times \text{NumChannels} \times \frac{\text{BitsPerSample}}{8}$ |
| 0x20 | `BlockAlign` | 2 B | $\text{NumChannels} \times \frac{\text{BitsPerSample}}{8}$ (2 bytes for 16-bit mono) |
| 0x22 | `BitsPerSample`| 2 B | 16 (or 32 if 32-bit container selected) |
| 0x24 | `Subchunk2ID` | 4 B | `"data"` (0x61746164 little-endian) |
| 0x28 | `Subchunk2Size`| 4 B | Total PCM audio data bytes |
| 0x2C | Audio Payload | $N$ B | Sequential 16-bit signed PCM samples ($[-32768, 32767]$) |

---

## 5. Low-Power Sleep and Wakeup Lifecycle

During inter-session intervals, the recorder enters ultra-low-power mode:
1. **Peripheral Power-Down**:
   - TIM6 hardware timer clock halted.
   - ADC1 powered down into deep power-down mode.
   - DMA1 channels disabled.
   - SDMMC bus clock halted and card placed into standby.
   - External PSRAM placed in Deep Power Down (0xB9) or standby ($CE\#$ held high).
   - Onboard User LED (PA5) turned off.
2. **STM32L476 Power Mode**:
   - Enters **Stop 2 Mode** with full SRAM1 and SRAM2 retention.
3. **Wakeup Sources**:
   - Internal **RTC Wakeup Timer** (fires after programmed sleep duration).
   - External **User Button B1 (PC13)** interrupt for manual wake.
4. **Post-Wakeup Sequence**:
   - Re-initializes system clocks to 72 MHz PLL.
   - Re-enables peripheral clocks and re-arms ADC/DMA buffers.
   - Synchronizes RTC calendar timestamp before starting the next session.

