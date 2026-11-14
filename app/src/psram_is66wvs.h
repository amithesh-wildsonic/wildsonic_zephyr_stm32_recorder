/* SPDX-License-Identifier: Apache-2.0 */
#ifndef PSRAM_IS66WVS_H_
#define PSRAM_IS66WVS_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ISSI IS66WVS4M8ALL / IS66WVS4M8BLL 32Mb (4MB) SPI/QPI Pseudo-SRAM Driver.
 *
 * CRITICAL SPECIFICATIONS & TIMING CONSTRAINTS:
 * 1. Capacity: 32 Mbit = 4,194,304 bytes (4 MBytes).
 * 2. Voltage:
 *    - IS66WVS4M8ALL: 1.65 V to 1.95 V (1.8 V nominal). DO NOT CONNECT DIRECTLY TO 3.3 V!
 *    - IS66WVS4M8BLL: 2.7 V to 3.6 V (3.3 V nominal). Compatible with STM32 NUCLEO 3.3 V.
 * 3. Timing Reference:
 *    - Datasheet specifies maximum CE-low interval (tCEM) of 4.0 us near 85 C
 *      and 1.0 us near 105 C. The driver constrains transaction burst sizes to ensure
 *      wire times strictly respect tCEM.
 *    - tCPH (Chip Enable High Time Minimum): >= 1 tCLK cycle (>= 50 ns).
 *    - Transaction timing at 18 MHz (T_CLK = 55.56 ns, ~72 cycles in 4 us):
 *      * 1-Line SPI Mode (8 cycles cmd + 24 cycles addr = 32 cycles overhead; 8 cycles/byte data):
 *        4 us corresponds to ~72 clocks; max payload is (72 - 32) / 8 = 5 bytes.
 *        Configured safe burst size = 4 bytes (64 cycles = 3.56 us <= 4.0 us).
 *      * 4-Line Quad I/O Mode (8 cycles cmd + 6 cycles addr + 6 dummy = 20 cycles overhead; 2 cycles/byte data):
 *        (72 - 20) / 2 = 26 bytes.
 *        Configured safe burst size = 16 bytes (52 cycles read = 2.89 us; 46 cycles write = 2.56 us <= 4.0 us).
 * 4. NUCLEO-L476RG Pin Mapping for QUADSPI:
 *    - QUADSPI_CLK:    PB10 (AF10) -> Morpho CN10-25 / Arduino D6
 *    - QUADSPI_BK1_NCS:PB11 (AF10) -> Morpho CN10-18 (Morpho only)
 *    - QUADSPI_BK1_IO0:PB1  (AF10) -> Morpho CN10-24 (Morpho only)
 *    - QUADSPI_BK1_IO1:PB0  (AF10) -> Morpho CN7-34  / Arduino A3
 *    - QUADSPI_BK1_IO2:PA7  (AF10) -> Morpho CN10-15 / Arduino D11
 *    - QUADSPI_BK1_IO3:PA6  (AF10) -> Morpho CN10-13 / Arduino D12
 * 5. Production Audio Path:
 *    - PSRAM is an optional secondary test/staging store.
 *    - DO NOT use PSRAM for live production audio unless initialization and Quad I/O memory tests pass.
 *    - Isolated test area resides at 0x00300000–0x003FFFFF (strictly outside 0x000000–0x0FFFFF live staging).
 */

#define PSRAM_IS66WVS_CAPACITY_BYTES   (4 * 1024 * 1024)
#define PSRAM_IS66WVS_PAGE_SIZE        1024

/* Dedicated diagnostic test region: strictly outside 0x000000–0x0FFFFF live audio staging */
#define PSRAM_DIAG_TEST_BASE_ADDR      0x00300000U
#define PSRAM_DIAG_TEST_MAX_ADDR       0x003FFFFFU

/* Safe hardware burst limits: 1024-byte full-page Quad I/O; 4-byte SPI */
#define PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI   4
#define PSRAM_IS66WVS_SAFE_BURST_BYTES_QUAD  1024
#define PSRAM_IS66WVS_SAFE_BURST_BYTES       PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI

#define PSRAM_ISSI_MFID                0x9D
#define PSRAM_ISSI_KGD_PASS            0x5D
#define PSRAM_ISSI_DENSITY_32M         0x4D

/* 10 Deterministic Pattern Classes */
enum psram_pattern_type {
  PSRAM_PAT_ALL_ZEROS = 0,    /* 00 00 00 ... */
  PSRAM_PAT_ALL_ONES,         /* FF FF FF ... */
  PSRAM_PAT_ALT_55_AA,        /* 55 AA 55 AA ... */
  PSRAM_PAT_ALT_AA_55,        /* AA 55 AA 55 ... */
  PSRAM_PAT_INCREMENTING,     /* 00 01 02 ... */
  PSRAM_PAT_DECREMENTING,     /* FF FE FD ... */
  PSRAM_PAT_WALKING_ONES,     /* 01 02 04 08 10 20 40 80 ... */
  PSRAM_PAT_WALKING_ZEROS,    /* FE FD FB F7 EF DF BF 7F ... */
  PSRAM_PAT_ALT_NIBBLES,      /* F0 0F F0 0F ... */
  PSRAM_PAT_PSEUDO_RANDOM,    /* LCG seed 0x12345678 */
  PSRAM_PAT_COUNT
};

struct psram_id {
  uint8_t mfid;
  uint8_t kgd;
  uint8_t density;
};

struct psram_test_report {
  bool detected;
  bool spi_integrity_verified;
  bool quad_mode_verified;
  bool integrity_verified;
  struct psram_id id;
  uint32_t passes_tested;
  uint32_t passes_passed;
  uint32_t patterns_tested;
  uint32_t patterns_passed;
  uint32_t boundary_checks_passed;
  uint32_t error_count;
  uint32_t first_failing_addr;
  uint8_t expected_val;
  uint8_t observed_val;
  uint8_t failing_bitmask;
  uint8_t ctx_expected[8];
  uint8_t ctx_observed[8];
  uint32_t ctx_len;
  char status_msg[160];

  /* Decoupled Cross-Mode Test Error Counts */
  uint32_t mode_a_errors;     /* Test A: SPI Write (0x02) + SPI Read (0x03) */
  uint32_t mode_b_errors;     /* Test B: Quad Write (0x38) + SPI Read (0x03) */
  uint32_t mode_c_errors;     /* Test C: SPI Write (0x02) + Quad Read (0xEB) */
  uint32_t mode_d_errors;     /* Test D: Quad Write (0x38) + Quad Read (0xEB) */
  uint32_t sample_shifting;   /* 0 = NONE, 1 = HALFCYCLE */
  uint32_t sshift_c_errors;   /* Diagnostic re-probe of Test C with HALFCYCLE */
};

/* Initialize QUADSPI peripheral and GPIOs for IS66WVS PSRAM */
int psram_init(void);

/* Read ID register (MFID, KGD, Density) */
int psram_read_id(struct psram_id *id);

/* Chunked writes/reads with strictly enforced burst limits respecting tCEM */
int psram_write(uint32_t address, const uint8_t *data, uint32_t length);
int psram_read(uint32_t address, uint8_t *data, uint32_t length);

/* Run complete standalone hardware bring-up and memory integrity test */
int psram_run_self_test(struct psram_test_report *report);

/* Query whether Quad I/O mode is verified and active */
bool psram_is_quad_mode(void);

/* Query active hardware burst size in bytes */
uint32_t psram_get_burst_size(void);

/* Configure hardware burst size in bytes */
int psram_set_burst_size(uint32_t burst_size);

/* Put external PSRAM into documented low-power state (Deep Power Down 0xB9 / Standby) */
int psram_sleep(void);

/* Wake external PSRAM from low-power state and re-verify communication */
int psram_wake(void);

/* Check if PSRAM hardware was detected and passed identification */
bool psram_is_present(void);

/* Current QUADSPI kernel clock divided by the programmed prescaler. */
uint32_t psram_get_clock_hz(void);

/* Check if PSRAM memory integrity has been verified through self-test */
bool psram_is_integrity_verified(void);

/* Emit standardized single-line boot status via TRACE */
void psram_log_boot_status(void);

/* Pattern helper */
const char *psram_get_pattern_name(enum psram_pattern_type type);
void psram_fill_pattern(enum psram_pattern_type type, uint8_t *buf, uint32_t len, uint32_t base_addr);

#endif /* PSRAM_IS66WVS_H_ */

