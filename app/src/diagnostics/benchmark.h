/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct bench_ram_metric {
  uint32_t chunk_size;
  uint32_t iterations;
  uint64_t total_bytes;
  uint32_t write_us;
  uint32_t read_us;
  uint32_t copy_us;
  uint32_t write_mb_s;
  uint32_t read_mb_s;
  uint32_t copy_mb_s;
  uint32_t write_mib_s;
  uint32_t read_mib_s;
  uint32_t copy_mib_s;
  bool integrity_ok;
};

struct bench_psram_metric {
  uint32_t chunk_size;
  uint32_t iterations;
  uint64_t total_bytes;
  uint32_t write_us;
  uint32_t read_us;
  uint32_t copy_us;
  uint32_t write_mb_s;
  uint32_t read_mb_s;
  uint32_t copy_mb_s;
  uint32_t write_mib_s;
  uint32_t read_mib_s;
  uint32_t copy_mib_s;
  bool quad_mode;
  bool integrity_ok;
};

struct bench_sd_metric {
  uint32_t block_size;
  uint32_t total_blocks;
  uint64_t total_bytes;
  uint32_t write_us;
  uint32_t sync_us;
  uint32_t throughput_mb_s_milli;
  uint32_t throughput_mib_s_milli;
  uint32_t lat_min_us;
  uint32_t lat_avg_us;
  uint32_t lat_med_us;
  uint32_t lat_p95_us;
  uint32_t lat_p99_us;
  uint32_t lat_max_us;
  uint32_t write_errors;
  bool integrity_ok;
};

struct bench_pipeline_metric {
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint32_t duration_sec;
  bool use_psram;
  uint64_t bytes_produced;
  uint64_t bytes_written;
  uint32_t blocks_produced;
  uint32_t blocks_written;
  uint32_t blocks_dropped;
  uint32_t queue_peak;
  uint32_t psram_peak_bytes;
  uint32_t avg_write_lat_us;
  uint32_t max_write_lat_us;
  uint32_t effective_kb_s;
  bool integrity_ok;
  bool passed;
};

enum sd_bench_mode {
  SD_BENCH_QUICK = 0, /* 4K, 16K, 64K blocks; 512 KiB; 1 warmup + 3 measured runs */
  SD_BENCH_MEDIUM,    /* 4K, 16K, 64K blocks; 4 MiB; 1 warmup + 3 measured runs */
  SD_BENCH_FULL,      /* 512 B to 64 KiB (8 block sizes); 8 MiB file; 1 warmup + 5 measured runs; opt-in */
};

/* Independent Benchmarks */
int bench_run_ram(void);
int bench_run_cpu(void);
int bench_run_adc(void);
int bench_run_psram(void);
int bench_run_sd_mode(enum sd_bench_mode mode);
int bench_run_sd(void);
int bench_run_pipeline(uint32_t target_rate, bool use_psram);
int bench_run_pipeline_all(void);
void bench_print_status(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCHMARK_H_ */

