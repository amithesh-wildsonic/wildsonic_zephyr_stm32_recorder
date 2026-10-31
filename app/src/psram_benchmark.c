/* SPDX-License-Identifier: Apache-2.0 */
#include "psram_benchmark.h"
#include "psram_is66wvs.h"
#include "psram_staging.h"
#include "recorder.h"

#include <errno.h>
#include <string.h>

#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(psram_bench, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(psram_bench, LOG_LEVEL_WRN);
#endif
#else
#include <stdio.h>
#include <time.h>
#define LOG_INF(...) do {} while (0)
#define LOG_ERR(...) do {} while (0)
#define LOG_WRN(...) do {} while (0)
__attribute__((weak)) bool recorder_is_active(void) { return false; }
#endif

#define BENCH_CHUNK_SIZE 4096U
static uint8_t bench_tx_buf[BENCH_CHUNK_SIZE] __aligned(4);
static uint8_t bench_rx_buf[BENCH_CHUNK_SIZE] __aligned(4);

static inline uint64_t bench_get_us(void)
{
#if defined(__ZEPHYR__)
	return k_cyc_to_us_floor64(k_cycle_get_64());
#else
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
#endif
}

const char *psram_benchmark_mode_name(enum psram_bench_mode mode)
{
	switch (mode) {
	case PSRAM_BENCH_SEQ_WRITE:
		return "Sequential Write";
	case PSRAM_BENCH_SEQ_READ:
		return "Sequential Read";
	case PSRAM_BENCH_READ_AFTER_WRITE:
		return "Read-After-Write Verify";
	case PSRAM_BENCH_REPEATED_BLOCK:
		return "Repeated Block Transfer";
	case PSRAM_BENCH_STREAMING_SIM:
		return "Interleaved Streaming (R/W)";
	default:
		return "Unknown";
	}
}

static void fill_bench_pattern(uint8_t *buf, uint32_t len, uint32_t seed)
{
	for (uint32_t i = 0; i < len; i++) {
		buf[i] = (uint8_t)((seed + i) ^ 0xA5U);
	}
}

int psram_benchmark_run(enum psram_bench_mode mode,
                        uint32_t total_bytes,
                        struct psram_bench_result *out)
{
	if (!out || total_bytes == 0) {
		return -EINVAL;
	}

	const uint32_t max_span = PSRAM_DIAG_TEST_MAX_ADDR - PSRAM_DIAG_TEST_BASE_ADDR + 1U;
	if (total_bytes > max_span) {
		return -EINVAL;
	}

	if (!psram_is_present()) {
		return -ENODEV;
	}

	memset(out, 0, sizeof(*out));
	out->mode = mode;
	out->test_size_bytes = total_bytes;
	out->chunk_size_bytes = BENCH_CHUNK_SIZE;
	out->min_chunk_us = UINT32_MAX;

	uint32_t total_chunks = (total_bytes + BENCH_CHUNK_SIZE - 1U) / BENCH_CHUNK_SIZE;
	uint64_t chunk_latency_sum = 0;
	uint32_t chunks_measured = 0;

	uint64_t t_start = bench_get_us();

	for (uint32_t chunk_idx = 0; chunk_idx < total_chunks; chunk_idx++) {
		uint32_t offset = chunk_idx * BENCH_CHUNK_SIZE;
		uint32_t current_addr = PSRAM_DIAG_TEST_BASE_ADDR + offset;
		uint32_t chunk_len = total_bytes - offset;
		if (chunk_len > BENCH_CHUNK_SIZE) {
			chunk_len = BENCH_CHUNK_SIZE;
		}

		fill_bench_pattern(bench_tx_buf, chunk_len, current_addr);

		uint64_t c0 = bench_get_us();
		int rc = 0;

		switch (mode) {
		case PSRAM_BENCH_SEQ_WRITE:
			rc = psram_write(current_addr, bench_tx_buf, chunk_len);
			if (rc == 0) {
				out->bytes_written += chunk_len;
				out->total_transferred_bytes += chunk_len;
			}
			out->verified_mode = false;
			break;

		case PSRAM_BENCH_SEQ_READ:
			memset(bench_rx_buf, 0x55, chunk_len);
			rc = psram_read(current_addr, bench_rx_buf, chunk_len);
			if (rc == 0) {
				out->bytes_read += chunk_len;
				out->total_transferred_bytes += chunk_len;
			}
			out->verified_mode = false;
			break;

		case PSRAM_BENCH_READ_AFTER_WRITE:
			out->verified_mode = true;
			rc = psram_write(current_addr, bench_tx_buf, chunk_len);
			if (rc == 0) {
				out->bytes_written += chunk_len;
				memset(bench_rx_buf, 0x55, chunk_len);
				rc = psram_read(current_addr, bench_rx_buf, chunk_len);
				if (rc == 0) {
					out->bytes_read += chunk_len;
					out->total_transferred_bytes += (chunk_len * 2U);
					uint32_t chunk_mismatches = 0;
					for (uint32_t i = 0; i < chunk_len; i++) {
						if (bench_tx_buf[i] != bench_rx_buf[i]) {
							if (!out->has_mismatch_diag) {
								out->first_mismatch_addr = current_addr + i;
								out->first_mismatch_offset = i;
								out->first_expected_byte = bench_tx_buf[i];
								out->first_actual_byte = bench_rx_buf[i];
								out->first_diff_mask = (uint8_t)(bench_tx_buf[i] ^ bench_rx_buf[i]);
								out->has_mismatch_diag = true;
							}
							chunk_mismatches++;
							out->mismatch_bytes++;
						}
					}
					if (chunk_mismatches > 0) {
						out->mismatch_blocks++;
						out->mismatch_count++;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS) || !defined(__ZEPHYR__)
						LOG_WRN("[BENCH DIAG] Chunk %u (addr 0x%08X): %u/%u bytes mismatch! First at +%u (0x%08X): exp=0x%02X act=0x%02X (diff=0x%02X)",
						        chunk_idx, current_addr, chunk_mismatches, chunk_len,
						        out->first_mismatch_offset, out->first_mismatch_addr,
						        out->first_expected_byte, out->first_actual_byte, out->first_diff_mask);
#endif
					}
				}
			}
			break;

		case PSRAM_BENCH_REPEATED_BLOCK:
			/* Repeatedly transfer the first block to benchmark pure controller turnaround */
			out->verified_mode = true;
			rc = psram_write(PSRAM_DIAG_TEST_BASE_ADDR, bench_tx_buf, chunk_len);
			if (rc == 0) {
				out->bytes_written += chunk_len;
				memset(bench_rx_buf, 0x55, chunk_len);
				rc = psram_read(PSRAM_DIAG_TEST_BASE_ADDR, bench_rx_buf, chunk_len);
				if (rc == 0) {
					out->bytes_read += chunk_len;
					out->total_transferred_bytes += (chunk_len * 2U);
					uint32_t chunk_mismatches = 0;
					for (uint32_t i = 0; i < chunk_len; i++) {
						if (bench_tx_buf[i] != bench_rx_buf[i]) {
							if (!out->has_mismatch_diag) {
								out->first_mismatch_addr = PSRAM_DIAG_TEST_BASE_ADDR + i;
								out->first_mismatch_offset = i;
								out->first_expected_byte = bench_tx_buf[i];
								out->first_actual_byte = bench_rx_buf[i];
								out->first_diff_mask = (uint8_t)(bench_tx_buf[i] ^ bench_rx_buf[i]);
								out->has_mismatch_diag = true;
							}
							chunk_mismatches++;
							out->mismatch_bytes++;
						}
					}
					if (chunk_mismatches > 0) {
						out->mismatch_blocks++;
						out->mismatch_count++;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS) || !defined(__ZEPHYR__)
						LOG_WRN("[BENCH DIAG REPEAT] Iteration %u: %u/%u bytes mismatch! exp=0x%02X act=0x%02X",
						        chunk_idx, chunk_mismatches, chunk_len,
						        out->first_expected_byte, out->first_actual_byte);
#endif
					}
				}
			}
			break;

		case PSRAM_BENCH_STREAMING_SIM:
			/* Interleaved producer push simulation and consumer peek simulation */
			out->verified_mode = true;
			rc = psram_write(current_addr, bench_tx_buf, chunk_len);
			if (rc == 0) {
				out->bytes_written += chunk_len;
				/* Read prior block if available */
				uint32_t read_addr = (chunk_idx > 0) ?
				                     (PSRAM_DIAG_TEST_BASE_ADDR + (chunk_idx - 1U) * BENCH_CHUNK_SIZE) :
				                     current_addr;
				memset(bench_rx_buf, 0x55, chunk_len);
				rc = psram_read(read_addr, bench_rx_buf, chunk_len);
				if (rc == 0) {
					out->bytes_read += chunk_len;
					out->total_transferred_bytes += (chunk_len * 2U);
					uint32_t chunk_mismatches = 0;
					for (uint32_t i = 0; i < chunk_len; i++) {
						uint8_t exp = (uint8_t)((read_addr + i) ^ 0xA5U);
						if (exp != bench_rx_buf[i]) {
							if (!out->has_mismatch_diag) {
								out->first_mismatch_addr = read_addr + i;
								out->first_mismatch_offset = i;
								out->first_expected_byte = exp;
								out->first_actual_byte = bench_rx_buf[i];
								out->first_diff_mask = (uint8_t)(exp ^ bench_rx_buf[i]);
								out->has_mismatch_diag = true;
							}
							chunk_mismatches++;
							out->mismatch_bytes++;
						}
					}
					if (chunk_mismatches > 0) {
						out->mismatch_blocks++;
						out->mismatch_count++;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS) || !defined(__ZEPHYR__)
						LOG_WRN("[BENCH DIAG STREAM] Chunk %u (read 0x%08X): %u/%u bytes mismatch! exp=0x%02X act=0x%02X",
						        chunk_idx, read_addr, chunk_mismatches, chunk_len,
						        out->first_expected_byte, out->first_actual_byte);
#endif
					}
				}
			}
			break;

		default:
			return -EINVAL;
		}

		uint64_t c1 = bench_get_us();
		uint32_t lat_us = (uint32_t)(c1 >= c0 ? (c1 - c0) : 0);

		if (rc != 0) {
			out->error_count++;
			break;
		}

		if (lat_us < out->min_chunk_us) out->min_chunk_us = lat_us;
		if (lat_us > out->max_chunk_us) out->max_chunk_us = lat_us;
		chunk_latency_sum += lat_us;
		chunks_measured++;
	}

	uint64_t t_end = bench_get_us();
	out->bus_elapsed_us = (chunk_latency_sum > 0) ? chunk_latency_sum : 1U;
	out->total_elapsed_us = (t_end >= t_start) ? (t_end - t_start) : out->bus_elapsed_us;

	if (chunks_measured > 0) {
		out->avg_chunk_us = (uint32_t)(chunk_latency_sum / chunks_measured);
	} else {
		out->min_chunk_us = 0;
	}

	/* Throughput in decimal bytes per second, based strictly on pure physical bus transfer time */
	out->throughput_bytes_sec = (uint32_t)(((uint64_t)out->total_transferred_bytes * 1000000ULL) / out->bus_elapsed_us);

	return 0;
}

int psram_benchmark_run_matrix(void)
{
	const uint32_t test_sizes[] = {
		4U * 1024U,      /* 4 KiB */
		32U * 1024U,     /* 32 KiB */
		256U * 1024U,    /* 256 KiB */
		1024U * 1024U    /* 1 MiB */
	};
	const size_t num_sizes = sizeof(test_sizes) / sizeof(test_sizes[0]);

#if defined(__ZEPHYR__)
	printk("\n====================================================================================================================================================\n");
	printk("                                     ISSI IS66WVS4M8 QUADSPI PSRAM PHYSICAL THROUGHPUT BENCHMARK                                                    \n");
	printk("====================================================================================================================================================\n");
	printk("  Bus Clock        : %u Hz (Prescaler divider = %u)\n", psram_get_clock_hz(),
	       HAL_RCC_GetHCLKFreq() / psram_get_clock_hz());
	printk("  Bus Interface    : %s\n", psram_is_quad_mode() ? "Quad I/O Mode (4-lines)" : "SPI Mode (1-line)");
	printk("  Hardware Burst   : %u bytes per transaction (constrained strictly by tCEM <= 4.0 us)\n",
	       psram_get_burst_size());
	printk("  Memory Region    : 0x%08X - 0x%08X (1 MiB Dedicated Diagnostic Span, outside live audio)\n",
	       PSRAM_DIAG_TEST_BASE_ADDR, PSRAM_DIAG_TEST_MAX_ADDR);
	printk("  Step Chunk Size  : %u bytes (MinStepUs / AvgStepUs / MaxStepUs are latency per 4 KiB chunk operation)\n",
	       BENCH_CHUNK_SIZE);
	printk("  Throughput Units : kB/s = 1000 Bytes/s (decimal standard) | KiB/s = 1024 Bytes/s (binary standard)\n");
	printk("----------------------------------------------------------------------------------------------------------------------------------------------------\n");
	printk("%-26s | %8s | %17s | %10s | %21s | %9s | %9s | %9s | %s\n",
	       "Mode", "Span", "Transferred (W/R)", "BusTimeUs", "Throughput(kB/s|KiB/s)", "MinStepUs", "AvgStepUs", "MaxStepUs", "Data Verification Status");
	printk("----------------------------------------------------------------------------------------------------------------------------------------------------\n");
#endif

	for (int m = 0; m < PSRAM_BENCH_MODE_COUNT; m++) {
		for (size_t s = 0; s < num_sizes; s++) {
			struct psram_bench_result res;
			int rc = psram_benchmark_run((enum psram_bench_mode)m, test_sizes[s], &res);
			if (rc != 0) {
#if defined(__ZEPHYR__)
				printk("%-26s | %6u KB | FAILED (code %d)\n",
				       psram_benchmark_mode_name((enum psram_bench_mode)m),
				       test_sizes[s] >> 10, rc);
#endif
				continue;
			}

#if defined(__ZEPHYR__)
			uint32_t dec_kbs = res.throughput_bytes_sec / 1000U;
			uint32_t bin_kibs = res.throughput_bytes_sec / 1024U;
			char xfer_str[24];
			snprintf(xfer_str, sizeof(xfer_str), "W:%u R:%u", res.bytes_written, res.bytes_read);
			char tp_str[24];
			snprintf(tp_str, sizeof(tp_str), "%u | %u", dec_kbs, bin_kibs);

			if (res.error_count > 0) {
				printk("%-26s | %6u KB | %17s | %10llu | %21s | %9u | %9u | %9u | HW ERROR (%u bus errs)\n",
				       psram_benchmark_mode_name((enum psram_bench_mode)m),
				       res.test_size_bytes >> 10,
				       xfer_str,
				       (unsigned long long)res.bus_elapsed_us,
				       tp_str,
				       res.min_chunk_us,
				       res.avg_chunk_us,
				       res.max_chunk_us,
				       res.error_count);
			} else if (res.mismatch_count > 0 || res.mismatch_bytes > 0) {
				printk("%-26s | %6u KB | %17s | %10llu | %21s | %9u | %9u | %9u | MISMATCH: %u blks / %u B (first +%u @ 0x%08X exp=0x%02X act=0x%02X diff=0x%02X)\n",
				       psram_benchmark_mode_name((enum psram_bench_mode)m),
				       res.test_size_bytes >> 10,
				       xfer_str,
				       (unsigned long long)res.bus_elapsed_us,
				       tp_str,
				       res.min_chunk_us,
				       res.avg_chunk_us,
				       res.max_chunk_us,
				       res.mismatch_blocks,
				       res.mismatch_bytes,
				       res.first_mismatch_offset,
				       res.first_mismatch_addr,
				       res.first_expected_byte,
				       res.first_actual_byte,
				       res.first_diff_mask);
			} else if (!res.verified_mode) {
				const char *vmsg = (m == PSRAM_BENCH_SEQ_WRITE) ?
				                   "WRITE OK (unverified - no readback)" :
				                   "READ OK (unverified - untracked pattern)";
				printk("%-26s | %6u KB | %17s | %10llu | %21s | %9u | %9u | %9u | %s\n",
				       psram_benchmark_mode_name((enum psram_bench_mode)m),
				       res.test_size_bytes >> 10,
				       xfer_str,
				       (unsigned long long)res.bus_elapsed_us,
				       tp_str,
				       res.min_chunk_us,
				       res.avg_chunk_us,
				       res.max_chunk_us,
				       vmsg);
			} else {
				printk("%-26s | %6u KB | %17s | %10llu | %21s | %9u | %9u | %9u | PASS (bit-exact verified)\n",
				       psram_benchmark_mode_name((enum psram_bench_mode)m),
				       res.test_size_bytes >> 10,
				       xfer_str,
				       (unsigned long long)res.bus_elapsed_us,
				       tp_str,
				       res.min_chunk_us,
				       res.avg_chunk_us,
				       res.max_chunk_us);
			}
#endif
		}
	}

#if defined(__ZEPHYR__)
	printk("====================================================================================================================================================\n\n");
#endif
	return 0;
}

int psram_benchmark_run_pattern_validation(void)
{
	if (!psram_is_present()) {
		return -ENODEV;
	}

	static const uint32_t test_addrs[] = {
		PSRAM_DIAG_TEST_BASE_ADDR,                        /* Start of 1MB span: 0x00300000 */
		PSRAM_DIAG_TEST_BASE_ADDR + 0x3FEU,               /* Straddles 1024B page boundary: 0x003003FE */
		PSRAM_DIAG_TEST_BASE_ADDR + 0x1000U,              /* 4 KiB boundary: 0x00301000 */
		PSRAM_DIAG_TEST_MAX_ADDR - BENCH_CHUNK_SIZE + 1U  /* End of 1MB span: 0x003FFF00 */
	};
	const size_t num_addrs = sizeof(test_addrs) / sizeof(test_addrs[0]);

#if defined(__ZEPHYR__)
	printk("\n=========================================================================================\n");
	printk("          DETERMINISTIC MULTI-PATTERN 4 KIB PSRAM VERIFICATION TEST (10 PATTERNS)       \n");
	printk("=========================================================================================\n");
#endif

	uint32_t total_tests = 0;
	uint32_t total_passed = 0;
	uint32_t total_byte_mismatches = 0;

	for (int p = 0; p < PSRAM_PAT_COUNT; p++) {
		enum psram_pattern_type pat = (enum psram_pattern_type)p;

		for (size_t a = 0; a < num_addrs; a++) {
			uint32_t addr = test_addrs[a];
			total_tests++;

			psram_fill_pattern(pat, bench_tx_buf, BENCH_CHUNK_SIZE, addr);
			memset(bench_rx_buf, 0x55, BENCH_CHUNK_SIZE);

			int wret = psram_write(addr, bench_tx_buf, BENCH_CHUNK_SIZE);
			int rret = (wret == 0) ? psram_read(addr, bench_rx_buf, BENCH_CHUNK_SIZE) : -EIO;

			if (wret != 0 || rret != 0) {
#if defined(__ZEPHYR__)
				printk("  Pattern %-28s @ 0x%08X: BUS ERROR (wret=%d rret=%d)\n",
				       psram_get_pattern_name(pat), addr, wret, rret);
#endif
				continue;
			}

			uint32_t mismatches = 0;
			uint32_t first_off = 0;
			uint8_t exp = 0, act = 0, diff = 0;

			for (uint32_t i = 0; i < BENCH_CHUNK_SIZE; i++) {
				if (bench_tx_buf[i] != bench_rx_buf[i]) {
					if (mismatches == 0) {
						first_off = i;
						exp = bench_tx_buf[i];
						act = bench_rx_buf[i];
						diff = (uint8_t)(exp ^ act);
					}
					mismatches++;
				}
			}

			if (mismatches == 0) {
				total_passed++;
#if defined(__ZEPHYR__)
				printk("  Pattern %-28s @ 0x%08X: PASS (4096 B verified)\n",
				       psram_get_pattern_name(pat), addr);
#endif
			} else {
				total_byte_mismatches += mismatches;
#if defined(__ZEPHYR__)
				printk("  Pattern %-28s @ 0x%08X: FAIL (%u mismatches, first +%u: exp=0x%02X act=0x%02X diff=0x%02X)\n",
				       psram_get_pattern_name(pat), addr, mismatches, first_off, exp, act, diff);
#endif
			}
		}
	}

#if defined(__ZEPHYR__)
	printk("-----------------------------------------------------------------------------------------\n");
	printk("  Pattern Validation Summary: %u / %u tests passed, %u total byte mismatches\n",
	       total_passed, total_tests, total_byte_mismatches);
	printk("=========================================================================================\n\n");
#endif

	return (total_passed == total_tests) ? 0 : -EBADMSG;
}

struct psram_diag_subtest_result {
	const char *category;
	const char *name;
	uint32_t addr;
	uint32_t len;
	uint32_t mismatch_bytes;
	uint32_t first_mismatch_offset;
	uint32_t first_mismatch_addr;
	uint8_t first_expected;
	uint8_t first_actual;
	uint8_t first_diff_mask;
	int bus_error;
	bool passed;
};

static int run_diag_subtest(const char *category, const char *name,
                            uint32_t addr, const uint8_t *tx, uint32_t len,
                            struct psram_diag_subtest_result *res)
{
	memset(res, 0, sizeof(*res));
	res->category = category;
	res->name = name;
	res->addr = addr;
	res->len = len;

	if (len > BENCH_CHUNK_SIZE) {
		res->bus_error = -EINVAL;
		return -EINVAL;
	}

	memset(bench_rx_buf, 0x5A, len);

	int wret = psram_write(addr, tx, len);
	if (wret != 0) {
		res->bus_error = wret;
		res->passed = false;
		return wret;
	}

	int rret = psram_read(addr, bench_rx_buf, len);
	if (rret != 0) {
		res->bus_error = rret;
		res->passed = false;
		return rret;
	}

	for (uint32_t i = 0; i < len; i++) {
		if (tx[i] != bench_rx_buf[i]) {
			if (res->mismatch_bytes == 0) {
				res->first_mismatch_offset = i;
				res->first_mismatch_addr = addr + i;
				res->first_expected = tx[i];
				res->first_actual = bench_rx_buf[i];
				res->first_diff_mask = (uint8_t)(tx[i] ^ bench_rx_buf[i]);
			}
			res->mismatch_bytes++;
		}
	}

	res->passed = (res->mismatch_bytes == 0);
	return res->passed ? 0 : -EBADMSG;
}

int psram_benchmark_run_isolated_diagnostic(void)
{
	if (!psram_is_present()) {
		return -ENODEV;
	}

#if defined(__ZEPHYR__)
	if (recorder_is_active()) {
		LOG_WRN("Cannot run PSRAM diagnostic while recorder is active");
		return -EBUSY;
	}
#endif

#if defined(__ZEPHYR__)
	printk("\n=========================================================================================================\n");
	printk("             MINIMAL ISOLATED PSRAM TRANSACTION DIAGNOSTIC SUITE (PHASE 2)                              \n");
	printk("=========================================================================================================\n");
	printk("  Target Region    : 0x%08X - 0x%08X (Strictly outside live audio staging)\n",
	       PSRAM_DIAG_TEST_BASE_ADDR, PSRAM_DIAG_TEST_MAX_ADDR);
	printk("  Burst Constraint : %u bytes (tCEM <= 4.0 us)\n", psram_get_burst_size());
	printk("---------------------------------------------------------------------------------------------------------\n");
#endif

	uint32_t total_subtests = 0;
	uint32_t total_passed = 0;
	uint32_t total_failed = 0;
	uint32_t total_byte_mismatches = 0;

	/* ------------------------------------------------------------------
	 * Case 1: Aligned single address with all 10 deterministic patterns
	 * ------------------------------------------------------------------ */
	for (int p = 0; p < PSRAM_PAT_COUNT; p++) {
		enum psram_pattern_type pat = (enum psram_pattern_type)p;
		psram_fill_pattern(pat, bench_tx_buf, 256U, PSRAM_DIAG_TEST_BASE_ADDR);

		struct psram_diag_subtest_result res;
		total_subtests++;
		int rc = run_diag_subtest("Aligned (10 Pat)", psram_get_pattern_name(pat),
		                          PSRAM_DIAG_TEST_BASE_ADDR, bench_tx_buf, 256U, &res);
		if (rc == 0) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len %3u B): OK\n",
			       res.category, res.name, res.addr, res.len);
#endif
		} else {
			total_failed++;
			total_byte_mismatches += res.mismatch_bytes;
#if defined(__ZEPHYR__)
			if (res.bus_error != 0) {
				printk("  [ERR ] %-18s %-24s @ 0x%08X (len %3u B): BUS ERROR %d\n",
				       res.category, res.name, res.addr, res.len, res.bus_error);
			} else {
				printk("  [FAIL] %-18s %-24s @ 0x%08X (len %3u B): %u mismatches! First +%u (@ 0x%08X): exp=0x%02X act=0x%02X diff=0x%02X\n",
				       res.category, res.name, res.addr, res.len,
				       res.mismatch_bytes, res.first_mismatch_offset, res.first_mismatch_addr,
				       res.first_expected, res.first_actual, res.first_diff_mask);
			}
#endif
		}
	}

	/* ------------------------------------------------------------------
	 * Case 2: Deliberately unaligned starting addresses (offsets 1, 2, 3)
	 * ------------------------------------------------------------------ */
	static const uint32_t unaligned_offsets[] = { 1U, 2U, 3U };
	for (size_t i = 0; i < sizeof(unaligned_offsets) / sizeof(unaligned_offsets[0]); i++) {
		uint32_t off = unaligned_offsets[i];
		uint32_t test_addr = PSRAM_DIAG_TEST_BASE_ADDR + 0x100U + off;
		psram_fill_pattern(PSRAM_PAT_INCREMENTING, bench_tx_buf, 64U, test_addr);

		char name_buf[32];
		snprintf(name_buf, sizeof(name_buf), "+%u Byte Offset (64B)", off);

		struct psram_diag_subtest_result res;
		total_subtests++;
		int rc = run_diag_subtest("Unaligned Start", name_buf, test_addr, bench_tx_buf, 64U, &res);
		if (rc == 0) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len %3u B): OK\n",
			       res.category, res.name, res.addr, res.len);
#endif
		} else {
			total_failed++;
			total_byte_mismatches += res.mismatch_bytes;
#if defined(__ZEPHYR__)
			if (res.bus_error != 0) {
				printk("  [ERR ] %-18s %-24s @ 0x%08X (len %3u B): BUS ERROR %d\n",
				       res.category, res.name, res.addr, res.len, res.bus_error);
			} else {
				printk("  [FAIL] %-18s %-24s @ 0x%08X (len %3u B): %u mismatches! First +%u (@ 0x%08X): exp=0x%02X act=0x%02X diff=0x%02X\n",
				       res.category, res.name, res.addr, res.len,
				       res.mismatch_bytes, res.first_mismatch_offset, res.first_mismatch_addr,
				       res.first_expected, res.first_actual, res.first_diff_mask);
			}
#endif
		}
	}

	/* ------------------------------------------------------------------
	 * Case 3: 16-byte burst boundary crossings
	 * ------------------------------------------------------------------ */
	/* 3a: Straddles two 16-byte boundaries (starts at offset +8, len 32) */
	{
		uint32_t test_addr = PSRAM_DIAG_TEST_BASE_ADDR + 0x208U;
		psram_fill_pattern(PSRAM_PAT_PSEUDO_RANDOM, bench_tx_buf, 32U, test_addr);

		struct psram_diag_subtest_result res;
		total_subtests++;
		int rc = run_diag_subtest("Burst Crossing", "Mid-burst 32B (2 bnds)", test_addr, bench_tx_buf, 32U, &res);
		if (rc == 0) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len %3u B): OK\n",
			       res.category, res.name, res.addr, res.len);
#endif
		} else {
			total_failed++;
			total_byte_mismatches += res.mismatch_bytes;
#if defined(__ZEPHYR__)
			if (res.bus_error != 0) {
				printk("  [ERR ] %-18s %-24s @ 0x%08X (len %3u B): BUS ERROR %d\n",
				       res.category, res.name, res.addr, res.len, res.bus_error);
			} else {
				printk("  [FAIL] %-18s %-24s @ 0x%08X (len %3u B): %u mismatches! First +%u (@ 0x%08X): exp=0x%02X act=0x%02X diff=0x%02X\n",
				       res.category, res.name, res.addr, res.len,
				       res.mismatch_bytes, res.first_mismatch_offset, res.first_mismatch_addr,
				       res.first_expected, res.first_actual, res.first_diff_mask);
			}
#endif
		}
	}
	/* 3b: Starts 2 bytes before 16-byte boundary (starts at offset +14, len 20) */
	{
		uint32_t test_addr = PSRAM_DIAG_TEST_BASE_ADDR + 0x20EU;
		psram_fill_pattern(PSRAM_PAT_PSEUDO_RANDOM, bench_tx_buf, 20U, test_addr);

		struct psram_diag_subtest_result res;
		total_subtests++;
		int rc = run_diag_subtest("Burst Crossing", "Near-burst 20B (2B pfx)", test_addr, bench_tx_buf, 20U, &res);
		if (rc == 0) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len %3u B): OK\n",
			       res.category, res.name, res.addr, res.len);
#endif
		} else {
			total_failed++;
			total_byte_mismatches += res.mismatch_bytes;
#if defined(__ZEPHYR__)
			if (res.bus_error != 0) {
				printk("  [ERR ] %-18s %-24s @ 0x%08X (len %3u B): BUS ERROR %d\n",
				       res.category, res.name, res.addr, res.len, res.bus_error);
			} else {
				printk("  [FAIL] %-18s %-24s @ 0x%08X (len %3u B): %u mismatches! First +%u (@ 0x%08X): exp=0x%02X act=0x%02X diff=0x%02X\n",
				       res.category, res.name, res.addr, res.len,
				       res.mismatch_bytes, res.first_mismatch_offset, res.first_mismatch_addr,
				       res.first_expected, res.first_actual, res.first_diff_mask);
			}
#endif
		}
	}

	/* ------------------------------------------------------------------
	 * Case 4: Non-multiple of 16-byte lengths (1, 7, 15, 23, 127 bytes)
	 * ------------------------------------------------------------------ */
	static const uint32_t non_16_lens[] = { 1U, 7U, 15U, 23U, 127U };
	for (size_t i = 0; i < sizeof(non_16_lens) / sizeof(non_16_lens[0]); i++) {
		uint32_t len = non_16_lens[i];
		uint32_t test_addr = PSRAM_DIAG_TEST_BASE_ADDR + 0x300U;
		psram_fill_pattern(PSRAM_PAT_INCREMENTING, bench_tx_buf, len, test_addr);

		char name_buf[32];
		snprintf(name_buf, sizeof(name_buf), "Length %3u Bytes", len);

		struct psram_diag_subtest_result res;
		total_subtests++;
		int rc = run_diag_subtest("Non-16 Multiple", name_buf, test_addr, bench_tx_buf, len, &res);
		if (rc == 0) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len %3u B): OK\n",
			       res.category, res.name, res.addr, res.len);
#endif
		} else {
			total_failed++;
			total_byte_mismatches += res.mismatch_bytes;
#if defined(__ZEPHYR__)
			if (res.bus_error != 0) {
				printk("  [ERR ] %-18s %-24s @ 0x%08X (len %3u B): BUS ERROR %d\n",
				       res.category, res.name, res.addr, res.len, res.bus_error);
			} else {
				printk("  [FAIL] %-18s %-24s @ 0x%08X (len %3u B): %u mismatches! First +%u (@ 0x%08X): exp=0x%02X act=0x%02X diff=0x%02X\n",
				       res.category, res.name, res.addr, res.len,
				       res.mismatch_bytes, res.first_mismatch_offset, res.first_mismatch_addr,
				       res.first_expected, res.first_actual, res.first_diff_mask);
			}
#endif
		}
	}

	/* ------------------------------------------------------------------
	 * Case 5: Diagnostic boundary termination at 0x003FFFFF
	 * ------------------------------------------------------------------ */
	/* 5a: 256 bytes ending exactly at 0x003FFFFF */
	{
		uint32_t test_addr = PSRAM_DIAG_TEST_MAX_ADDR - 256U + 1U;
		psram_fill_pattern(PSRAM_PAT_ALT_AA_55, bench_tx_buf, 256U, test_addr);

		struct psram_diag_subtest_result res;
		total_subtests++;
		int rc = run_diag_subtest("Boundary Term", "256B ending @ 0x3FFFFF", test_addr, bench_tx_buf, 256U, &res);
		if (rc == 0) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len %3u B): OK\n",
			       res.category, res.name, res.addr, res.len);
#endif
		} else {
			total_failed++;
			total_byte_mismatches += res.mismatch_bytes;
#if defined(__ZEPHYR__)
			if (res.bus_error != 0) {
				printk("  [ERR ] %-18s %-24s @ 0x%08X (len %3u B): BUS ERROR %d\n",
				       res.category, res.name, res.addr, res.len, res.bus_error);
			} else {
				printk("  [FAIL] %-18s %-24s @ 0x%08X (len %3u B): %u mismatches! First +%u (@ 0x%08X): exp=0x%02X act=0x%02X diff=0x%02X\n",
				       res.category, res.name, res.addr, res.len,
				       res.mismatch_bytes, res.first_mismatch_offset, res.first_mismatch_addr,
				       res.first_expected, res.first_actual, res.first_diff_mask);
			}
#endif
		}
	}

	/* 5b: 16 bytes ending exactly at 0x003FFFFF */
	{
		uint32_t test_addr = PSRAM_DIAG_TEST_MAX_ADDR - 16U + 1U;
		psram_fill_pattern(PSRAM_PAT_ALT_55_AA, bench_tx_buf, 16U, test_addr);

		struct psram_diag_subtest_result res;
		total_subtests++;
		int rc = run_diag_subtest("Boundary Term", "16B ending @ 0x3FFFFF", test_addr, bench_tx_buf, 16U, &res);
		if (rc == 0) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len %3u B): OK\n",
			       res.category, res.name, res.addr, res.len);
#endif
		} else {
			total_failed++;
			total_byte_mismatches += res.mismatch_bytes;
#if defined(__ZEPHYR__)
			if (res.bus_error != 0) {
				printk("  [ERR ] %-18s %-24s @ 0x%08X (len %3u B): BUS ERROR %d\n",
				       res.category, res.name, res.addr, res.len, res.bus_error);
			} else {
				printk("  [FAIL] %-18s %-24s @ 0x%08X (len %3u B): %u mismatches! First +%u (@ 0x%08X): exp=0x%02X act=0x%02X diff=0x%02X\n",
				       res.category, res.name, res.addr, res.len,
				       res.mismatch_bytes, res.first_mismatch_offset, res.first_mismatch_addr,
				       res.first_expected, res.first_actual, res.first_diff_mask);
			}
#endif
		}
	}

	/* 5c: 1 byte ending exactly at 0x003FFFFF */
	{
		uint32_t test_addr = PSRAM_DIAG_TEST_MAX_ADDR;
		bench_tx_buf[0] = 0x7EU;

		struct psram_diag_subtest_result res;
		total_subtests++;
		int rc = run_diag_subtest("Boundary Term", "1B at exact max (3FFFFF)", test_addr, bench_tx_buf, 1U, &res);
		if (rc == 0) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len %3u B): OK\n",
			       res.category, res.name, res.addr, res.len);
#endif
		} else {
			total_failed++;
			total_byte_mismatches += res.mismatch_bytes;
#if defined(__ZEPHYR__)
			if (res.bus_error != 0) {
				printk("  [ERR ] %-18s %-24s @ 0x%08X (len %3u B): BUS ERROR %d\n",
				       res.category, res.name, res.addr, res.len, res.bus_error);
			} else {
				printk("  [FAIL] %-18s %-24s @ 0x%08X (len %3u B): %u mismatches! First +%u (@ 0x%08X): exp=0x%02X act=0x%02X diff=0x%02X\n",
				       res.category, res.name, res.addr, res.len,
				       res.mismatch_bytes, res.first_mismatch_offset, res.first_mismatch_addr,
				       res.first_expected, res.first_actual, res.first_diff_mask);
			}
#endif
		}
	}

	/* 5d: Out-of-bounds rejection check */
	{
		total_subtests++;
		uint8_t dummy_byte = 0xAAU;
		int w_oob = psram_write(PSRAM_DIAG_TEST_MAX_ADDR, &dummy_byte, 2U);
		int r_oob = psram_read(PSRAM_DIAG_TEST_MAX_ADDR + 1U, &dummy_byte, 1U);
		if (w_oob == -EINVAL && r_oob == -EINVAL) {
			total_passed++;
#if defined(__ZEPHYR__)
			printk("  [PASS] %-18s %-24s @ 0x%08X (len   2 B): OK (rejected with -EINVAL)\n",
			       "Boundary Term", "Out-of-Bounds Rejection", PSRAM_DIAG_TEST_MAX_ADDR);
#endif
		} else {
			total_failed++;
#if defined(__ZEPHYR__)
			printk("  [FAIL] %-18s %-24s @ 0x%08X (len   2 B): FAIL (w_oob=%d r_oob=%d, exp -EINVAL)\n",
			       "Boundary Term", "Out-of-Bounds Rejection", PSRAM_DIAG_TEST_MAX_ADDR, w_oob, r_oob);
#endif
		}
	}

#if defined(__ZEPHYR__)
	printk("---------------------------------------------------------------------------------------------------------\n");
	printk("  Isolated Diagnostic Summary: %u / %u tests passed (%u failed), %u total byte mismatches\n",
	       total_passed, total_subtests, total_failed, total_byte_mismatches);
	printk("=========================================================================================================\n\n");
#endif

	return (total_passed == total_subtests) ? 0 : -EBADMSG;
}

