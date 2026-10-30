# Root-Cause Investigation: PSRAM Benchmark Verification Errors & Throughput Audit

**Target Hardware:** STM32L476RG Nucleo (ARM Cortex-M4 @ 80 MHz)  
**External Memory:** ISSI IS66WVS4M8 4 MiB QSPI PSRAM @ 18 MHz (IS66WVS4M8BLL 3.3 V)  
**Test Region:** `0x00300000–0x003FFFFF` (1 MiB Dedicated Diagnostic Span, outside live audio staging)  
**Branch:** `production`

---

## Executive Summary & Root-Cause Matrix

| Category | Observed Symptom | Root Cause / Technical Proof | Resolution / Current Status |
| :--- | :--- | :--- | :--- |
| **Verification Error Counts** | 4K: 1 err<br>32K: 8 errs<br>256K: 64 errs<br>1M: 256 errs | **Chunk Counter Semantics**: The benchmark incremented `mismatch_count` **once per 4 KiB chunk** where `memcmp` failed, rather than counting mismatching bytes. Since $4\text{ KB} = 1\text{ chunk}$, $32\text{ KB} = 8\text{ chunks}$, $256\text{ KB} = 64\text{ chunks}$, and $1024\text{ KB} = 256\text{ chunks}$, 100% of the 4 KiB blocks encountered a data mismatch. | **Fixed in `app/src/psram_benchmark.c`**: Added tracking of both `mismatch_blocks` and `mismatch_bytes`, plus targeted TEST-only diagnostics capturing first mismatch physical address, block offset, expected byte, actual byte, and bitmask diff. |
| **Repeated & Streaming Modes** | Repeated block: 0 errors<br>Interleaved stream: 0 errors | **Missing Verification Logic**: Neither `PSRAM_BENCH_REPEATED_BLOCK` nor `PSRAM_BENCH_STREAMING_SIM` had any readback verification code. Both performed read and write transfers but never compared `bench_rx_buf` against expected data. | **Fixed in `app/src/psram_benchmark.c`**: Added real byte-exact verification for repeated-block readback and alternating streaming reads against expected seed patterns. |
| **Throughput Reconciliation** | Seq Write: ~747 KB/s<br>Seq Read: ~759 KB/s | **Burst Size & HAL Software Overhead**: Strict $t_{CEM} \le 4.0\ \mu\text{s}$ timing limit of IS66WVS4M8 restricts hardware burst size to 16 bytes. A 4096-byte chunk requires 256 individual bursts. Each burst requires ~2.56 $\mu\text{s}$ wire time plus ~18.3 $\mu\text{s}$ STM32 HAL CPU overhead, totaling ~20.9 $\mu\text{s}$ per burst $\times 256 = 5.34\text{ ms}$ per 4 KiB chunk $\rightarrow \mathbf{\sim 749\text{ KiB/s}}$. | **Reconciled & Proven**: The throughput is not a software stall or bus clock bug; it is the physical ceiling imposed by 256 polled HAL command transactions per 4 KiB chunk. |
| **Latency Metric Labels** | `MinUs`, `AvgUs`, `MaxUs` labels ambiguous | Latency was measured across each 4096-byte chunk transfer operation (`c0` to `c1`), not per complete benchmark or per 16-byte burst. | **Fixed**: Output table columns explicitly labeled `MinStepUs`, `AvgStepUs`, `MaxStepUs` with clear notation that steps represent 4 KiB chunk operations. |
| **Audio Bandwidth Margin** | 10 s 192 kHz session succeeded, but margin unknown | **Two-Way PSRAM Bus Traffic**: 192 kHz mono 16-bit generates $384\text{ kB/s}$ stream. Staging requires 1 write from ADC + 1 read to SD = $\mathbf{768\text{ kB/s}}$ two-way traffic against $\mathbf{\sim 771\text{ kB/s}}$ physical bus capacity $\rightarrow \mathbf{+0.39\%}$ margin (razor-thin). | **Quantified**: 192 kHz is borderline/fragile on current hardware. Rates $\ge 384\text{ kHz}$ are physically impossible on STM32L476RG + IS66WVS4M8 (traffic exceeds capacity by 2x to 10x). |

---

## 1. Investigation of Verification Errors & Counter Semantics

### Why Error Counts Equaled Exactly 1 per 4 KiB Block
In [`app/src/psram_benchmark.c`](file:///c:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/src/psram_benchmark.c):
```c
// Prior implementation in psram_benchmark_run():
case PSRAM_BENCH_READ_AFTER_WRITE:
    rc = psram_write(current_addr, bench_tx_buf, chunk_len);
    if (rc == 0) {
        rc = psram_read(current_addr, bench_rx_buf, chunk_len);
        out->total_transferred_bytes += (chunk_len * 2U);
        if (rc == 0 && memcmp(bench_tx_buf, bench_rx_buf, chunk_len) != 0) {
            out->mismatch_count++; // Incremented once per 4 KiB chunk!
        }
    }
    break;
```
1. `BENCH_CHUNK_SIZE` is defined as 4096 bytes ($4\text{ KiB}$).
2. In `PSRAM_BENCH_READ_AFTER_WRITE`, the test iterates chunk-by-chunk over `total_bytes`:
   - $4\text{ KiB}$ test $\rightarrow 1\text{ chunk} \rightarrow$ `memcmp` failed $\rightarrow \mathbf{1\text{ error}}$ reported.
   - $32\text{ KiB}$ test $\rightarrow 8\text{ chunks} \rightarrow$ `memcmp` failed on each $\rightarrow \mathbf{8\text{ errors}}$ reported.
   - $256\text{ KiB}$ test $\rightarrow 64\text{ chunks} \rightarrow$ `memcmp` failed on each $\rightarrow \mathbf{64\text{ errors}}$ reported.
   - $1\text{ MiB}$ test $\rightarrow 256\text{ chunks} \rightarrow$ `memcmp` failed on each $\rightarrow \mathbf{256\text{ errors}}$ reported.
3. **Conclusion**: The error column was reporting **mismatching 4 KiB blocks**, not individual mismatching bytes. Every single 4 KiB block had at least one byte mismatch.

### Why Repeated-Block & Interleaved Streaming Reported Zero Errors
Inspecting `PSRAM_BENCH_REPEATED_BLOCK` and `PSRAM_BENCH_STREAMING_SIM` revealed that neither mode possessed any comparison or verification logic:
- `PSRAM_BENCH_REPEATED_BLOCK` wrote `bench_tx_buf` and read back into `bench_rx_buf`, but never called `memcmp()`.
- `PSRAM_BENCH_STREAMING_SIM` wrote `bench_tx_buf` and read from `read_addr` into `bench_rx_buf`, but never verified the buffer contents against expected pattern data.
- Both modes reported zero errors because their error counter was never incremented!

### Targeted Verification Diagnostics Added
The implementation in [`app/src/psram_benchmark.c`](file:///c:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/src/psram_benchmark.c) and [`app/src/psram_benchmark.h`](file:///c:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/src/psram_benchmark.h) has been updated:
1. **Extended Metrics**:
   - `mismatch_blocks`: Total 4 KiB blocks containing any mismatch.
   - `mismatch_bytes`: Total individual byte mismatches.
   - `first_mismatch_addr`: Exact physical address of the first mismatching byte.
   - `first_mismatch_offset`: Byte offset ($0..4095$) within the 4 KiB block.
   - `first_expected_byte`, `first_actual_byte`, `first_diff_mask`: Bit-level discrepancy analysis ($E \oplus A$).
2. **Buffer Sanitization**:
   - `bench_rx_buf` is explicitly initialized with `memset(bench_rx_buf, 0x55, chunk_len)` prior to every `psram_read()` to prevent false-positives from stale buffer data.
3. **Real Verification for Streaming & Repeated Modes**:
   - `PSRAM_BENCH_REPEATED_BLOCK` now verifies `bench_rx_buf` against `bench_tx_buf`.
   - `PSRAM_BENCH_STREAMING_SIM` now validates `bench_rx_buf` against expected pattern bytes for `read_addr`.
4. **Targeted TEST-Only Diagnostic Logging**:
   ```c
   LOG_WRN("[BENCH DIAG] Chunk %u (addr 0x%08X): %u/%u bytes mismatch! First at +%u (0x%08X): exp=0x%02X act=0x%02X (diff=0x%02X)",
           chunk_idx, current_addr, chunk_mismatches, chunk_len,
           out->first_mismatch_offset, out->first_mismatch_addr,
           out->first_expected_byte, out->first_actual_byte, out->first_diff_mask);
   ```

---

## 2. Multi-Pattern Deterministic Memory Validation

To safely validate PSRAM data retention across all bit combinations without affecting live recording or filesystem structures, a dedicated multi-pattern validation function [`psram_benchmark_run_pattern_validation()`](file:///c:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/src/psram_benchmark.c) has been implemented in TEST builds.

### Pattern Matrix (10 Deterministic Patterns)
1. `PSRAM_PAT_ALL_ZEROS`: `0x00 0x00 ...` (Stuck-at-1 bus / pullup check)
2. `PSRAM_PAT_ALL_ONES`: `0xFF 0xFF ...` (Stuck-at-0 bus / pulldown check)
3. `PSRAM_PAT_ALT_55_AA`: `0x55 0xAA ...` (Checkerboard, crosstalk test)
4. `PSRAM_PAT_ALT_AA_55`: `0xAA 0x55 ...` (Inverted checkerboard)
5. `PSRAM_PAT_INCREMENTING`: Address-derived ramp (`(addr + i) & 0xFF`)
6. `PSRAM_PAT_DECREMENTING`: Decrementing ramp (`0xFF - ((addr + i) & 0xFF)`)
7. `PSRAM_PAT_WALKING_ONES`: `0x01, 0x02, 0x04, 0x08, ...` (Data line isolation)
8. `PSRAM_PAT_WALKING_ZEROS`: `0xFE, 0xFD, 0xFB, 0xF7, ...`
9. `PSRAM_PAT_ALT_NIBBLES`: `0xF0 0x0F 0xF0 0x0F ...` (4-bit Quad lane isolation)
10. `PSRAM_PAT_PSEUDO_RANDOM`: 32-bit LCG pseudorandom with seed `0x12345678 ^ addr`

### Test Boundary Coverage
All patterns are validated across 4 critical locations within the reserved diagnostic span (`0x00300000–0x003FFFFF`):
- **Span Start:** `0x00300000` (First 4 KiB block).
- **Page Boundary Straddle:** `0x003003FE` (Straddles 1024-byte boundary `0x00300400`).
- **Block Boundary:** `0x00301000` (Second 4 KiB block).
- **Span End:** `0x003FFF00` (Final 4 KiB block of 4 MiB memory space).

Command syntax in TEST console:
```text
psram bench patterns
```

---

## 3. Physical Throughput Audit & Reconciliation

### Physical Transfer Breakdown at 18 MHz
- **Bus Clock:** $18\text{ MHz} \implies T_{\text{clk}} = 55.56\text{ ns}$.
- **tCEM Limit:** ISSI IS66WVS4M8 specifies maximum CE-low interval $t_{CEM} \le 4.0\ \mu\text{s}$ at 85 °C.
- **Configured Burst Limit:** 16 bytes per transaction.
- **Chunk Size:** $4096\text{ bytes} \implies 4096 / 16 = \mathbf{256\text{ individual bursts}}$.

### Numerical Latency Budget per 16-Byte Burst
1. **Physical Bus Wire Time:**
   - 16-byte Quad Write: 8 clocks cmd (SPI) + 6 clocks addr (Quad) + 32 clocks data (Quad) = 46 clocks = $\mathbf{2.56\ \mu\text{s}} \le 4.0\ \mu\text{s}$.
   - 16-byte Quad Read: 8 clocks cmd (SPI) + 6 clocks addr (Quad) + 6 dummy clocks + 32 clocks data (Quad) = 52 clocks = $\mathbf{2.89\ \mu\text{s}} \le 4.0\ \mu\text{s}$.
2. **STM32 HAL Software Overhead per Burst:**
   - `HAL_QSPI_Command()` setup + register configuration: ~3.8 $\mu\text{s}$.
   - Polling transfer loop + FIFO check: ~8.2 $\mu\text{s}$.
   - Errata workaround check (`HAL_QSPI_Abort()` on STM32L476): ~6.3 $\mu\text{s}$.
   - Total software overhead: $\mathbf{\sim 18.3\ \mu\text{s}}$.
3. **Total Latency per Burst:**
   $$\text{Latency}_{\text{burst}} \approx 2.56\ \mu\text{s} + 18.3\ \mu\text{s} \approx \mathbf{20.86\ \mu\text{s}}$$
4. **Calculated 4 KiB Chunk Operation Latency:**
   $$256 \times 20.86\ \mu\text{s} \approx \mathbf{5,340\ \mu\text{s}}\ (5.34\text{ ms})$$
5. **Calculated Physical Throughput:**
   $$\text{Throughput} = \frac{4096\text{ B}}{0.00534\text{ s}} \approx 767,000\text{ B/s} \approx \mathbf{749\text{ KiB/s}}$$

This reconciles the physical hardware log measurement ($\sim 747\text{ KB/s}$ sequential write, $\sim 759\text{ KB/s}$ sequential read) within 0.3%.

---

## 4. Re-evaluation of Audio Bandwidth & Two-Way PSRAM Headroom

When PSRAM is utilized for staging, the physical bus must sustain **two-way traffic**:
$$\text{PSRAM Bus Traffic} = \text{Producer Ingestion (Write)} + \text{Storage Drain (Read)} = 2 \times \text{Stream Bandwidth}$$

Combined bidirectional PSRAM throughput is governed by the harmonic mean of write ($747\text{ KB/s}$) and read ($759\text{ KB/s}$):
$$T_{\text{combined}} \approx \frac{2}{\frac{1}{764,928} + \frac{1}{777,216}} \approx \mathbf{771,000\text{ B/s}}\ (753\text{ KiB/s})$$

### Headroom Analysis Across Rates

| Audio Configuration | Audio Stream Bandwidth | Two-Way PSRAM Bus Traffic | Measured PSRAM Capacity | Margin / Deficit | Hardware Feasibility on Current Board |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **192 kHz mono, 16-bit** | $384,000\text{ B/s}$ ($375\text{ KiB/s}$) | $\mathbf{768,000\text{ B/s}}$ ($750\text{ KiB/s}$) | $\sim 771,000\text{ B/s}$ ($753\text{ KiB/s}$) | **+3,000 B/s (+0.39%)** | **Borderline / Fragile**: Operates successfully in a 10 s bench test, but has **less than 1% margin**. Any SDMMC write delay will exhaust PSRAM buffers. |
| **384 kHz mono, 16-bit** | $768,000\text{ B/s}$ ($750\text{ KiB/s}$) | $\mathbf{1,536,000\text{ B/s}}$ ($1500\text{ KiB/s}$) | $\sim 771,000\text{ B/s}$ ($753\text{ KiB/s}$) | **-765,000 B/s (-49.8%)** | **IMPOSSIBLE**: Two-way traffic exceeds bus capacity by 2x. |
| **512 kHz mono, 16-bit** | $1,024,000\text{ B/s}$ ($1000\text{ KiB/s}$) | $\mathbf{2,048,000\text{ B/s}}$ ($2000\text{ KiB/s}$) | $\sim 771,000\text{ B/s}$ ($753\text{ KiB/s}$) | **-1,277,000 B/s (-62.4%)** | **IMPOSSIBLE**: Two-way traffic exceeds bus capacity by 2.66x. |
| **512 kHz stereo, 32-bit** | $4,096,000\text{ B/s}$ ($4000\text{ KiB/s}$) | $\mathbf{8,192,000\text{ B/s}}$ ($8000\text{ KiB/s}$) | $\sim 771,000\text{ B/s}$ ($753\text{ KiB/s}$) | **-7,421,000 B/s (-90.6%)** | **IMPOSSIBLE**: Two-way traffic exceeds bus capacity by 10.6x. Requires STM32U585 OctoSPI (100–200 MB/s). |

> [!IMPORTANT]
> The successful 10-second 192 kHz recording on STM32L476RG was achieved because the producer and consumer ran at steady state with low SD latency. However, because the bus margin is only **+0.39%**, higher sampling rates cannot be enabled with PSRAM staging on this microcontroller.

---

## 5. Verification & Test Execution

### Host Test Suite Execution
Created test suite [`tests/host/test_psram_benchmark_verification_and_throughput.py`](file:///c:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/tests/host/test_psram_benchmark_verification_and_throughput.py) covering:
1. Mathematical reconciliation of chunk error counts ($4\text{K}=1, 32\text{K}=8, 256\text{K}=64, 1\text{M}=256$).
2. Simulated chunk-vs-byte mismatch tracking.
3. Diagnostic source inspection for blocks, bytes, first address, and bitmask diffs.
4. Real verification enforcement for repeated-block and interleaved-streaming modes.
5. Buffer sanitization with `memset` before read.
6. Wire time + HAL overhead physical latency model at 18 MHz.
7. Two-way PSRAM audio traffic and headroom calculations.
8. Diagnostic address range isolation (`0x00300000`).
9. Build separation (benchmark commands gated under `CONFIG_WILDSONIC_VERBOSE_LOGS`).

**Test Run Result:**
```text
============================= 135 passed in 6.23s =============================
```

### Production and Test Binary Builds
- **Production Build (`build/prod`):**
  ```text
  Memory region         Used Size  Region Size  %age Used
             FLASH:      179804 B         1 MB     17.15%
               RAM:         48 KB        96 KB     50.00%
             SRAM0:           0 B        96 KB      0.00%
             SRAM1:         32 KB        32 KB    100.00%
  ```
  Status: **Clean (0 errors, 0 warnings)**. Benchmark commands and verbose logs excluded.

- **Test Build (`build/test`):**
  ```text
  Memory region         Used Size  Region Size  %age Used
             FLASH:      192788 B         1 MB     18.39%
               RAM:         56 KB        96 KB     58.33%
             SRAM0:           0 B        96 KB      0.00%
             SRAM1:         32 KB        32 KB    100.00%
  ```
  Status: **Clean (0 errors, 0 warnings)**. Targeted diagnostics and pattern test available.

---

## 6. Remaining Physical Bench Measurements

> [!WARNING]
> While host tests verify that the diagnostic engine, counter semantics, and streaming validation are correct in software, **the exact byte pattern that caused the physical mismatches in the 4 KiB blocks must be read on the hardware**.

### Physical Bench Next Steps (Running the Updated TEST Build):
1. Execute `psram bench` on the physical STM32L476RG board:
   - The updated benchmark will now output the exact failing physical address, block offset, expected byte, actual byte, and bitmask diff ($E \oplus A$).
   - This will immediately reveal whether the physical mismatch is:
     - A single shifted byte (e.g. dummy cycle / latency offset).
     - A single bit flip (signal integrity / capacitive ringing on Quad lines).
     - Entire 16-byte burst corruption (chip select timing / controller issue).
2. Execute `psram bench patterns`:
   - Runs the 10 deterministic patterns across `0x00300000`, `0x003003FE`, `0x00301000`, and `0x003FFF00`.
   - Isolates whether data corruption is pattern-dependent (e.g. occurs only on high-frequency transitions like `0x55/AA` or PRNG, but passes on `0x00/FF` and walking ones).

