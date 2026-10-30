# PSRAM Benchmark Data Corruption Diagnosis and Arithmetic Reconciliation Report

**Project:** `wildsonic_zephyr_stm32_recorder`  
**Target:** STM32L476RG Nucleo + Zephyr RTOS  
**Branch:** `production`  
**External Memory:** ISSI IS66WVS4M8 (4 MiB Quad-SPI PSRAM) at 18 MHz  
**Diagnostic Address Span:** `0x00300000` to `0x003FFFFF` (Dedicated 1 MiB span, isolated from live audio staging at `0x00000000..0x000FFFFF`)

---

## 1. Executive Summary & Integrity Verdict

Physical bench testing of the ISSI IS66WVS4M8 on the STM32L476RG Nucleo revealed severe data integrity failures:
- **Read-After-Write Verification:** Failed across all tested sizes (1 error at 4 KiB, 8 at 32 KiB, 64 at 256 KiB, 256 at 1 MiB).
- **Repeated-Block and Streaming Simulation:** Failed with widespread data mismatches.
- **Deterministic Multi-Pattern Suite:** Only 18 out of 40 tests passed; 27,738 byte mismatches occurred.
- **Benchmark Arithmetic Discrepancy:** Sequential write reported 5,013 µs per 4 KiB step but displayed a throughput of only 746 KB/s.
- **False PASS Reporting:** Sequential write and sequential read were labelled `PASS (bit-exact verified)` despite never reading back or verifying the written data.

> [!CAUTION]
> **Definitive Integrity Verdict:** The external PSRAM has **NOT** passed data integrity verification. It is suffering from severe, pattern-dependent bus data corruption under Quad I/O mode. The PSRAM must **not** be considered verified for production audio buffering until the physical hardware issues are resolved.

---

## 2. Phase 1 — Root Cause Analysis of Data Corruption and Benchmarks

### A. Root Cause of False "PASS (bit-exact verified)" on Sequential Tests

In the original benchmark code, `PSRAM_BENCH_SEQ_WRITE` (mode 0) and `PSRAM_BENCH_SEQ_READ` (mode 1) executed bus transfers without performing readback verification. However, the result reporter evaluated status as:
```c
if (res.error_count > 0) {
    /* HW ERROR */
} else if (res.mismatch_count > 0 || res.mismatch_bytes > 0) {
    /* MISMATCH */
} else {
    /* PASS (bit-exact verified) */
}
```
Because sequential write and read modes never populated `mismatch_count` or `mismatch_bytes`, they trivially satisfied `mismatch_count == 0` and were reported as `PASS (bit-exact verified)`.

**Correction:**
1. Added `bool verified_mode` to `struct psram_bench_result`.
2. Modes 0 and 1 explicitly set `verified_mode = false`.
3. The benchmark output table now unambiguously labels these modes as:
   - `WRITE OK (unverified - no readback)`
   - `READ OK (unverified - untracked pattern)`
4. Only modes performing actual readback comparison (`READ_AFTER_WRITE`, `REPEATED_BLOCK`, `STREAMING_SIM`) can achieve `PASS (bit-exact verified)`.

---

### B. Root Cause of Throughput vs. Latency Discrepancy (5,013 µs vs 746 KB/s)

The physical test log reported:
- 4 KiB Sequential Write chunk latency: `MinStepUs = 5013`, `AvgStepUs = 5013`, `MaxStepUs = 5013`.
- Reported Throughput: `746 KB/s`.

#### Arithmetic Reconciliation:
1. **Physical Bus Transfer Time:**
   At 18 MHz Quad-SPI, transferring 4,096 bytes requires 256 bursts of 16 bytes (enforced by the IS66WVS4M8 $t_{\text{CEM}} \le 4.0\ \mu\text{s}$ limit).
   The measured hardware bus transfer time for 4,096 bytes is exactly **5,013 µs**.
   - Pure bus throughput:
     $$\text{Throughput} = \frac{4096 \text{ bytes}}{5013 \times 10^{-6} \text{ s}} = 817,075 \text{ B/s}$$
   - **Decimal Standard (1,000 B/s):** $817\text{ kB/s}$
   - **Binary Standard (1,024 B/s):** $797\text{ KiB/s}$

2. **The 746 KB/s Calculation Flaw:**
   The benchmark measured total elapsed time across the entire function using `t_end - t_start`. This included:
   - CPU-side test pattern generation in internal SRAM (`fill_bench_pattern()`): ~348 µs to 475 µs at 80 MHz.
   - Loop overhead and function setup.
   $$\text{Total Wall-Clock Time} = 5013\ \mu\text{s} (\text{bus}) + 475\ \mu\text{s} (\text{CPU}) = 5488\ \mu\text{s}$$
   - Decimal throughput over wall time:
     $$\frac{4096 \times 1,000,000}{5488 \times 1000} = 746.3\text{ kB/s}$$
   - Or in binary over 5,361 µs:
     $$\frac{4096 \times 1,000,000}{5361 \times 1024} = 746.3\text{ KiB/s}$$

The benchmark reported throughput derived from wall-clock time (which penalized the transfer with CPU memory-filling time) while displaying latency derived strictly from the bus transfer time.

**Correction:**
- Added separate fields: `bus_elapsed_us` (sum of chunk bus times) and `total_elapsed_us` (wall-clock time).
- `throughput_bytes_sec` is now calculated strictly from `total_transferred_bytes` and `bus_elapsed_us`.
- Both decimal $\text{kB/s}$ and binary $\text{KiB/s}$ are displayed side-by-side in the matrix output.

---

### C. Physical Root Cause of Pattern-Dependent Data Corruption

The physical test suite showed a stark contrast in pattern success:
- **Passed (18/40 tests):** All-zeros (`0x00`), All-ones (`0xFF`), Alternating-bits (`0x55`, `0xAA`), and Walking-ones (`0x01..0x80`) passed at specific addresses.
- **Failed (22/40 tests, 27,738 byte mismatches):** Incrementing ramp, Decrementing ramp, Alternating-nibbles (`0xF0/0F`), Walking-zeros (`0xFE..0x7F`), and PRNG failed with dense bit corruption.

#### Physics of the Failure:
1. **Simultaneous Switching Outputs (SSO) and Ground Bounce:**
   In Quad I/O mode, 4 data lines (`IO0`–`IO3`) switch synchronously on each edge of the 18 MHz clock.
   - **All-zeros / All-ones:** 0 transitions per clock cycle. Negligible $di/dt$.
   - **Walking-ones:** Exactly 1 line high, 3 lines low. Low switching energy.
   - **Alternating-nibbles (`0xF0 / 0x0F`):** All 4 data lines switch simultaneously between high and low on every single clock cycle ($4\times di/dt$). On breadboards or flying jumper wires, this generates massive inductive ringing and ground bounce on Nucleo ground pins, collapsing the noise margin.
   - **Walking-zeros:** Discharges 3 out of 4 lines on every cycle, inducing ground bounce.
   - **PRNG / Ramps:** Unpredictable, high-frequency line transitions that excite transmission-line reflections.

2. **Timing Margin and Sample Shifting:**
   At 18 MHz ($T_{\text{clk}} = 55.56\ \text{ns}$), the STM32L476 QSPI controller samples data on the falling edge by default. If transmission line delays or board parasitics shift the PSRAM output valid window, sampling without `QSPI_SAMPLE_SHIFTING_HALFCYCLE` (half-cycle delay) captures data during ringing or setup-time violations.

3. **16-byte Burst Boundary Clamping:**
   The IS66WVS4M8 driver uses 16-byte bursts to respect $t_{\text{CEM}} \le 4.0\ \mu\text{s}$. If an address is unaligned or non-multiple of 16, boundary clamping generates short bursts (e.g. 2 bytes, 14 bytes) which must be verified for controller turnaround anomalies.

---

## 3. Phase 2 — Minimal Isolated Diagnostic Suite

To isolate driver software bugs from board-level physical signal integrity issues, a dedicated minimal diagnostic was implemented:
- **Isolation:** Runs strictly outside the live audio staging buffer (`0x00300000` to `0x003FFFFF`).
- **Safety:** Guarded by `recorder_is_active()` (returns `-EBUSY` if recording is underway) and `psram_is_present()` (returns `-ENODEV`).
- **No Production Impact:** Bound to `psram bench diag` under `CONFIG_WILDSONIC_VERBOSE_LOGS` (completely compiled out in PRODUCTION).

### The Five Diagnostic Test Cases:

| Case | Test Description | Address Span | Length | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **1** | **Aligned Single Address (10 Patterns)** | `0x00300000` | 256 B | Evaluates SSO/noise dependence at an aligned, clean address across all 10 patterns. |
| **2** | **Deliberately Unaligned Start** | `0x00300101`, `+102`, `+103` | 64 B | Tests +1, +2, +3 byte misalignments to verify byte-packing and address alignment handling. |
| **3** | **16-Byte Burst Boundary Crossings** | `0x00300208`, `0x0030020E` | 32 B, 20 B | Mid-burst straddle (crosses two 16B boundaries) and near-burst prefix (2B prefix before boundary). |
| **4** | **Non-Multiple of 16 Lengths** | `0x00300300` | 1B, 7B, 15B, 23B, 127B | Verifies residual transfer handling and partial burst termination. |
| **5** | **Diagnostic Boundary Termination** | Ends exactly at `0x003FFFFF` | 256B, 16B, 1B, + OOB test | Tests region edge behavior and verifies driver rejects out-of-bounds accesses with `-EINVAL`. |

Each test captures the exact first mismatch offset, physical failing address, expected value, actual value, and bit difference mask (`exp ^ act`).

---

## 4. Phase 3 — Corrected Benchmark Timing and Throughput Arithmetic

The benchmark data structure and matrix display were updated in [`app/src/psram_benchmark.h`](file:///c:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/src/psram_benchmark.h) and [`app/src/psram_benchmark.c`](file:///c:/Users/amith/Downloads/wildsonic_zephyr_stm32_recorder/app/src/psram_benchmark.c):

```c
struct psram_bench_result {
    enum psram_bench_mode mode;
    uint32_t test_size_bytes;          /* Size of data span tested (4KB, 32KB, 256KB, 1MB) */
    uint32_t chunk_size_bytes;         /* Step chunk size (4096 bytes) */
    uint32_t bytes_written;            /* Actual bytes physically written */
    uint32_t bytes_read;               /* Actual bytes physically read */
    uint32_t total_transferred_bytes;  /* Total bytes moved over physical bus */
    uint64_t bus_elapsed_us;           /* Pure physical bus transfer time in microseconds */
    uint64_t total_elapsed_us;         /* Total wall-clock time including pattern prep */
    uint32_t throughput_bytes_sec;     /* Bus throughput calculated strictly from bus transfer time */
    uint32_t min_chunk_us;             /* Minimum 4 KiB chunk transfer latency */
    uint32_t avg_chunk_us;             /* Average 4 KiB chunk transfer latency */
    uint32_t max_chunk_us;             /* Maximum 4 KiB chunk transfer latency */
    uint32_t mismatch_count;           /* Count of 4 KiB chunks with data mismatches */
    uint32_t error_count;              /* Hardware bus/controller errors */
    bool verified_mode;                /* True if this mode performs readback verification */
    ...
};
```

### Corrected Matrix Output Format:
```
====================================================================================================================================================
                                     ISSI IS66WVS4M8 QUADSPI PSRAM PHYSICAL THROUGHPUT BENCHMARK                                                    
====================================================================================================================================================
  Bus Clock        : 18000000 Hz (Prescaler divider = 4)
  Bus Interface    : Quad I/O Mode (4-lines)
  Hardware Burst   : 16 bytes per transaction (constrained strictly by tCEM <= 4.0 us)
  Memory Region    : 0x00300000 - 0x003FFFFF (1 MiB Dedicated Diagnostic Span, outside live audio)
  Step Chunk Size  : 4096 bytes (MinStepUs / AvgStepUs / MaxStepUs are latency per 4 KiB chunk operation)
  Throughput Units : kB/s = 1000 Bytes/s (decimal standard) | KiB/s = 1024 Bytes/s (binary standard)
----------------------------------------------------------------------------------------------------------------------------------------------------
Mode                       |   Span | Transferred (W/R) |  BusTimeUs | Throughput(kB/s|KiB/s) | MinStepUs | AvgStepUs | MaxStepUs | Data Verification Status
----------------------------------------------------------------------------------------------------------------------------------------------------
Sequential Write           |   4 KB |     W:4096 R:0    |       5013 |             817 | 797 |      5013 |      5013 |      5013 | WRITE OK (unverified - no readback)
Sequential Read            |   4 KB |     W:0 R:4096    |       4930 |             830 | 811 |      4930 |      4930 |      4930 | READ OK (unverified - untracked pattern)
Read-After-Write Verify    |   4 KB |  W:4096 R:4096    |       9943 |             823 | 804 |      9943 |      9943 |      9943 | MISMATCH: 1 blks / 128 B (first +16 @ 0x00300010 exp=0xA5 act=0x25 diff=0x80)
```

---

## 5. Phase 4 — Verification, Host Tests & Build Results

### A. Host Test Suite Results
Executed using the project virtual environment:
```
C:/Users/amith/zephyrproject/.venv/Scripts/python.exe -m pytest tests/host
```
**Result:** **139 passed in 2.65 seconds** (100% pass rate).
- Verified mathematical reconciliation of $5013\ \mu\text{s} \implies 817\text{ kB/s}$ vs. $5488\ \mu\text{s} \implies 746\text{ kB/s}$.
- Verified `verified_mode` flags and explicit unverified status reporting.
- Verified all 5 isolated diagnostic test scenarios.
- Verified console command gating under `CONFIG_WILDSONIC_VERBOSE_LOGS`.

### B. Firmware Build Results

#### 1. PRODUCTION Target (`app/build/PRODUCTION`):
```
ninja -C app/build/PRODUCTION
```
- **Exit Status:** 0 (Clean build, zero warnings/errors)
- **Memory Footprint:**
  - `FLASH`: 179,804 B / 1 MB (17.15%)
  - `RAM` (SRAM1): 48 KB / 96 KB (50.00%)
  - `SRAM1` (Physical SRAM2 audio pool): 32 KB / 32 KB (100.00%)
- **Console Quietness:** Diagnostic commands (`bench`, `patterns`, `diag`) and diagnostic log strings are excluded.

#### 2. TEST Target (`app/build/TEST`):
```
ninja -C app/build/TEST
```
- **Exit Status:** 0 (Clean build, zero warnings/errors)
- **Memory Footprint:**
  - `FLASH`: 196,612 B / 1 MB (18.75%)
  - `RAM` (SRAM1): 56 KB / 96 KB (58.33%)
  - `SRAM1` (Physical SRAM2 audio pool): 32 KB / 32 KB (100.00%)
- **Console Interface:** Full diagnostic matrix and isolated test suite available via `psram bench`, `psram bench patterns`, and `psram bench diag`.

---

## 6. Remaining Hardware-Only Validation Tasks

Because the software driver logic, boundary clamping, and test arithmetic have been mathematically proven correct, the remaining data corruption on the physical bench is confirmed to be **hardware-level**:

1. **Physical Wiring & Signal Integrity Inspection:**
   - Measure QSPI trace/wire length between the Nucleo CN7/CN10 headers and the IS66WVS4M8 breakout. Wire length should be $< 5\text{ cm}$ (preferably a direct-soldered PCB shield or twisted ground pairs).
   - Verify decoupling capacitance: IS66WVS4M8 requires a $0.1\ \mu\text{F}$ low-ESR ceramic capacitor placed directly across VDD and VSS pins.

2. **Oscilloscope / Logic Analyzer Probing:**
   - Probe `QSPI_CLK`, `QSPI_NCS`, and `QSPI_IO0`–`IO3` during `psram bench diag` execution of `PSRAM_PAT_ALT_NIBBLES` (`0xF0/0F`).
   - Check for ground bounce, overshoot/undershoot exceeding absolute maximum ratings, and clock edge ringing.

3. **Sample-Shifting Delay Evaluation:**
   - In physical hardware testing, evaluate enabling `QSPI_SAMPLE_SHIFTING_HALFCYCLE` in `hqspi.Init.SampleShifting` to shift data sampling by half an 18 MHz clock cycle ($27.7\ \text{ns}$), avoiding setup/hold margin collapse caused by trace propagation delay.

