/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SD_BENCHMARK_H_
#define SD_BENCHMARK_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum sd_bench_type {
	SD_BENCH_FS_DYNAMIC = 0,
	SD_BENCH_FS_PREALLOC,
	SD_BENCH_RAW_BLOCK
};

struct sd_bench_result {
	enum sd_bench_type type;
	uint32_t block_bytes;
	uint32_t total_bytes;
	uint32_t num_operations;
	uint32_t avg_write_us;
	uint32_t max_write_us;
	uint32_t min_write_us;
	uint32_t elapsed_ms;
	uint32_t throughput_kb_s;
	bool passed_audio_rate; /* >= 384 KB/s */
	bool passed_margin;     /* >= 768 KB/s (2x operating margin) */
};

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
/* Execute a single sequential-write benchmark test */
int sd_benchmark_run_test(enum sd_bench_type type,
                          uint32_t block_bytes,
                          uint32_t total_bytes,
                          struct sd_bench_result *out);

/* Execute full benchmark matrix (32 KB, 64 KB, 128 KB, 256 KB, 512 KB) */
int sd_benchmark_run_matrix(void);
#else
static inline int sd_benchmark_run_test(enum sd_bench_type type,
                                        uint32_t block_bytes,
                                        uint32_t total_bytes,
                                        struct sd_bench_result *out)
{
	(void)type; (void)block_bytes; (void)total_bytes; (void)out;
	return -38; /* -ENOSYS */
}
static inline int sd_benchmark_run_matrix(void)
{
	return -38; /* -ENOSYS */
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* SD_BENCHMARK_H_ */

