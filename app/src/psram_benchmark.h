/* SPDX-License-Identifier: Apache-2.0 */
#ifndef PSRAM_BENCHMARK_H_
#define PSRAM_BENCHMARK_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Benchmark test modes for PSRAM evaluation.
 */
enum psram_bench_mode {
	PSRAM_BENCH_SEQ_WRITE = 0,         /* Sequential write throughput */
	PSRAM_BENCH_SEQ_READ = 1,          /* Sequential read throughput */
	PSRAM_BENCH_READ_AFTER_WRITE = 2,  /* Write then read back with bit-exact verification */
	PSRAM_BENCH_REPEATED_BLOCK = 3,    /* Repeated block transfer (FIFO/controller limit) */
	PSRAM_BENCH_STREAMING_SIM = 4,     /* Mixed producer write + consumer read interleaved streaming */
	PSRAM_BENCH_MODE_COUNT
};

/**
 * @brief Throughput and latency results for a PSRAM benchmark test.
 */
struct psram_bench_result {
	enum psram_bench_mode mode;
	uint32_t test_size_bytes;          /* Size of data span tested (4KB, 32KB, 256KB, 1MB) */
	uint32_t chunk_size_bytes;         /* Step chunk size (e.g. 4096 bytes) */
	uint32_t bytes_written;            /* Actual bytes physically written */
	uint32_t bytes_read;               /* Actual bytes physically read */
	uint32_t total_transferred_bytes;  /* Total bytes moved over physical bus (written + read) */
	uint64_t bus_elapsed_us;           /* Pure physical bus transfer time in microseconds */
	uint64_t total_elapsed_us;         /* Total wall-clock time including pattern prep */
	uint32_t throughput_bytes_sec;     /* Bus throughput calculated strictly from bus transfer time */
	uint32_t min_chunk_us;             /* Minimum 4 KiB chunk transfer latency in microseconds */
	uint32_t avg_chunk_us;             /* Average 4 KiB chunk transfer latency in microseconds */
	uint32_t max_chunk_us;             /* Maximum 4 KiB chunk transfer latency in microseconds */
	uint32_t mismatch_count;           /* Count of 4 KiB chunks with data mismatches */
	uint32_t error_count;              /* Hardware bus/controller errors */
	bool verified_mode;                /* True if this mode performs readback verification */

	/* Targeted verification diagnostics */
	uint32_t mismatch_bytes;           /* Total individual byte mismatches */
	uint32_t mismatch_blocks;          /* Total 4 KiB blocks with at least one mismatch */
	uint32_t first_mismatch_addr;      /* Physical address of first failing byte */
	uint32_t first_mismatch_offset;    /* Byte offset within 4 KiB block of first mismatch */
	uint8_t first_expected_byte;       /* Expected byte value at first mismatch */
	uint8_t first_actual_byte;         /* Observed byte value at first mismatch */
	uint8_t first_diff_mask;           /* Bitmask differences (expected ^ actual) */
	bool has_mismatch_diag;            /* True if mismatch details have been captured */
};

/**
 * @brief Execute a single PSRAM benchmark run.
 *
 * Runs strictly within the dedicated diagnostic address range 0x00300000–0x003FFFFF.
 * Live audio staging buffers (0x00000000–0x000FFFFF) are never accessed.
 *
 * @param mode Benchmark test mode.
 * @param total_bytes Total transfer size in bytes (e.g. 4096, 32768, 262144, 1048576).
 * @param out_result Output metrics structure.
 * @return 0 on success, negative errno on error.
 */
int psram_benchmark_run(enum psram_bench_mode mode,
                        uint32_t total_bytes,
                        struct psram_bench_result *out_result);

/**
 * @brief Execute the full PSRAM benchmark matrix across all modes and sizes.
 *
 * Only available in TEST builds (CONFIG_WILDSONIC_VERBOSE_LOGS=y).
 *
 * @return 0 on success, negative errno on error.
 */
int psram_benchmark_run_matrix(void);

/**
 * @brief Execute deterministic multi-pattern memory validation in the diagnostic span.
 *
 * Runs 10 deterministic test patterns (0x00, 0xFF, 0x55/AA, Walking 1s/0s, ramps, PRNG)
 * at span start, page boundaries, and span end with bit-exact readback verification.
 * Only available in TEST builds (CONFIG_WILDSONIC_VERBOSE_LOGS=y).
 *
 * @return 0 on success (all patterns match), negative errno on error.
 */
int psram_benchmark_run_pattern_validation(void);

/**
 * @brief Execute minimal isolated PSRAM transaction diagnostic (Phase 2).
 *
 * Runs strictly outside live audio staging while recorder is idle:
 *   1. Aligned single address with all 10 patterns.
 *   2. Deliberately unaligned starting addresses (1, 2, 3-byte offsets).
 *   3. 16-byte burst boundary crossings.
 *   4. Non-multiple of 16-byte lengths (1B, 7B, 15B, 23B, 127B).
 *   5. Region boundary check ending exactly at 0x003FFFFF.
 *
 * @return 0 on success (all passed), negative errno on mismatch.
 */
int psram_benchmark_run_isolated_diagnostic(void);

/**
 * @brief Get human-readable name of benchmark mode.
 */
const char *psram_benchmark_mode_name(enum psram_bench_mode mode);

#ifdef __cplusplus
}
#endif

#endif /* PSRAM_BENCHMARK_H_ */

