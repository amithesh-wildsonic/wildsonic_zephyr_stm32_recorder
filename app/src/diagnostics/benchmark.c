/* SPDX-License-Identifier: Apache-2.0 */
#include "benchmark.h"
#include "audio_in.h"
#include "dsp.h"
#include "psram_is66wvs.h"
#include "psram_staging.h"
#include "recorder.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <stm32_ll_rcc.h>

LOG_MODULE_REGISTER(benchmark, LOG_LEVEL_INF);

/* Track latest benchmark status */
static struct {
  bool ram_tested;
  struct bench_ram_metric ram_last;
  bool psram_tested;
  struct bench_psram_metric psram_last;
  bool sd_tested;
  struct bench_sd_metric sd_last;
  bool pipeline_tested;
  struct bench_pipeline_metric pipe_last;
} s_bench_summary;

/* Helper: simple in-place sort for latency percentiles */
static void sort_u32(uint32_t *arr, uint32_t n) {
  for (uint32_t i = 1; i < n; i++) {
    uint32_t key = arr[i];
    int32_t j = (int32_t)i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

/* Helper: deterministic address-based test pattern */
static inline uint8_t bench_pattern(uint32_t addr, uint32_t idx, uint32_t seed) {
  uint32_t val = (addr + idx) * 37U + ((addr + idx) >> 8) + seed;
  return (uint8_t)(val & 0xFFU);
}

static inline float bench_biquad_step(struct biquad_df2t *s, float x) {
  if (!s->enabled) return x;
  float y = s->b0 * x + s->s1;
  s->s1 = s->b1 * x - s->a1 * y + s->s2;
  s->s2 = s->b2 * x - s->a2 * y;
  return y;
}

static uint32_t stats_median_u32(uint32_t *values, uint32_t count) {
  sort_u32(values, count);
  return count ? values[count / 2] : 0;
}

static uint32_t benchmark_msi_clock_hz(uint32_t range) {
  switch (range) {
  case LL_RCC_MSIRANGE_0: return 100000U;
  case LL_RCC_MSIRANGE_1: return 200000U;
  case LL_RCC_MSIRANGE_2: return 400000U;
  case LL_RCC_MSIRANGE_3: return 800000U;
  case LL_RCC_MSIRANGE_4: return 1000000U;
  case LL_RCC_MSIRANGE_5: return 2000000U;
  case LL_RCC_MSIRANGE_6: return 4000000U;
  case LL_RCC_MSIRANGE_7: return 8000000U;
  case LL_RCC_MSIRANGE_8: return 16000000U;
  case LL_RCC_MSIRANGE_9: return 24000000U;
  case LL_RCC_MSIRANGE_10: return 32000000U;
  case LL_RCC_MSIRANGE_11: return 48000000U;
  default: return 0U;
  }
}

static void *adc_discard_block;
static void *adc_discard_alloc(void *user) {
  ARG_UNUSED(user);
  return adc_discard_block;
}
static int adc_discard_submit(void *block, void *user) {
  ARG_UNUSED(block);
  ARG_UNUSED(user);
  return 0;
}

int bench_run_cpu(void) {
  if (recorder_is_active() || benchmark_is_running()) return -EBUSY;
  benchmark_set_running(true);

  enum { RUNS = 31, SAMPLES = CONFIG_AUDIO_BLOCK_SAMPLES };
  uint32_t idle[RUNS], integer[RUNS], floating[RUNS], dsp[RUNS];
  void *audio_pool = NULL;
  uint32_t audio_pool_size = 0;
  int pool_ret = recorder_borrow_contiguous_pool(&audio_pool, &audio_pool_size);
  if (pool_ret != 0 || audio_pool_size < SAMPLES * sizeof(int32_t)) {
    benchmark_set_running(false);
    return pool_ret ? pool_ret : -ENOMEM;
  }
  uint32_t integer_sink = 0;
  volatile float float_sink = 0.0f;
#if CONFIG_AUDIO_SAMPLE_BITS == 32
  int32_t *block = audio_pool;
  for (uint32_t i = 0; i < SAMPLES; i++) block[i] = (int32_t)(i * 7919U) - 0x3fffffff;
#else
  int16_t *block = audio_pool;
  for (uint32_t i = 0; i < SAMPLES; i++) block[i] = (int16_t)(i * 37U - 32768);
#endif
  uint32_t checksum = 0;
  for (uint32_t i = 0; i < RUNS; i++) {
    uint64_t t0 = k_cycle_get_64();
    __asm__ volatile("" ::: "memory");
    idle[i] = (uint32_t)(k_cycle_get_64() - t0);

    uint32_t acc = i;
    t0 = k_cycle_get_64();
    for (uint32_t n = 0; n < 4096; n++) acc = (acc * 1664525U) + 1013904223U;
    integer[i] = (uint32_t)(k_cycle_get_64() - t0);
    integer_sink ^= acc;

    float f = (float)(i + 1U) * 0.125f;
    t0 = k_cycle_get_64();
    for (uint32_t n = 0; n < 4096; n++) f = f * 1.0001f + 0.0003f;
    floating[i] = (uint32_t)(k_cycle_get_64() - t0);
    float_sink += f;

    t0 = k_cycle_get_64();
#if CONFIG_AUDIO_SAMPLE_BITS == 32
    dsp_process_i32(block, SAMPLES);
#else
    dsp_process_i16(block, SAMPLES);
#endif
    dsp[i] = (uint32_t)(k_cycle_get_64() - t0);
    checksum += (uint32_t)block[i % SAMPLES];
  }

  struct biquad_df2t filter_snapshot[6];
  dsp_get_stage_filters(filter_snapshot);
  static const uint8_t group_first[] = {0, 1, 2, 4};
  static const uint8_t group_count[] = {1, 1, 2, 2};
  static const char *group_names[] = {"DC", "HPF", "Notches", "LPF"};
  uint32_t stage_cycles[4][RUNS];
  volatile uint32_t stage_sink = 0;
  for (uint32_t g = 0; g < 4; g++) {
    for (uint32_t run = 0; run < RUNS; run++) {
      struct biquad_df2t local[2];
      for (uint32_t f = 0; f < group_count[g]; f++) {
        local[f] = filter_snapshot[group_first[g] + f];
        local[f].s1 = 0.0f;
        local[f].s2 = 0.0f;
      }
      uint32_t sum = 0;
      uint64_t t0 = k_cycle_get_64();
      for (uint32_t repeat = 0; repeat < 8; repeat++) {
        for (uint32_t i = 0; i < SAMPLES; i++) {
          float x = (float)block[i];
          for (uint32_t f = 0; f < group_count[g]; f++) x = bench_biquad_step(&local[f], x);
          sum += (uint32_t)(int32_t)x;
        }
      }
      stage_cycles[g][run] = (uint32_t)((k_cycle_get_64() - t0) / 8U);
      stage_sink ^= sum;
    }
    sort_u32(stage_cycles[g], RUNS);
    uint64_t stage_sum = 0;
    for (uint32_t r = 0; r < RUNS; r++) stage_sum += stage_cycles[g][r];
    printk("  DSP stage %-8s cycles/block min/avg/median/p95/max: %u/%llu/%u/%u/%u\n",
           group_names[g], stage_cycles[g][0], stage_sum / RUNS,
           stage_cycles[g][RUNS / 2], stage_cycles[g][(RUNS * 95U) / 100U],
           stage_cycles[g][RUNS - 1]);
  }

  uint32_t idle_med = stats_median_u32(idle, RUNS);
  uint32_t integer_med = stats_median_u32(integer, RUNS);
  uint32_t floating_med = stats_median_u32(floating, RUNS);
  uint32_t dsp_med = stats_median_u32(dsp, RUNS);
  /* Arrays are sorted in place by median helper; min/max and p95 follow. */
  uint32_t dsp_p95 = dsp[(RUNS * 95U) / 100U];
  uint32_t dsp_max = dsp[RUNS - 1];
  uint64_t dsp_cycles_sum = 0;
  for (uint32_t i = 0; i < RUNS; i++) dsp_cycles_sum += dsp[i];
  uint32_t dsp_avg_cycles = (uint32_t)(dsp_cycles_sum / RUNS);
  uint32_t rate = audio_in_rate() ? audio_in_rate() : CONFIG_AUDIO_SAMPLE_RATE;
  uint32_t deadline_us = (uint32_t)(((uint64_t)SAMPLES * 1000000ULL) / rate);
  uint32_t dsp_us = (uint32_t)k_cyc_to_us_floor64(dsp_med);
  uint32_t dsp_avg_us = (uint32_t)k_cyc_to_us_floor64(dsp_avg_cycles);
  uint32_t dsp_overruns = 0;
  uint64_t deadline_cycles = k_us_to_cyc_ceil64(deadline_us);
  for (uint32_t i = 0; i < RUNS; i++) {
    if (dsp[i] > deadline_cycles) dsp_overruns++;
  }
  struct dsp_config active_dsp;
  dsp_get_config(&active_dsp);
  printk("\nCPU/DSP benchmark: %u runs, %u-sample block, sample-rate=%u Hz, filters dc/hpf/notch/harmonic/lpf=%u/%u/%u/%u/%u, checksum=0x%08X sinks=%u/%.3f\n",
         RUNS, SAMPLES, rate, active_dsp.dc_block_enabled, active_dsp.hpf_enabled,
         active_dsp.notch_enabled, active_dsp.harmonic_notch_enabled,
         active_dsp.lpf_enabled, checksum, integer_sink, (double)float_sink);
  printk("  Idle timer overhead median: %u cycles\n", idle_med);
  printk("  Integer 4096-op cycles min/median/p95/max: %u/%u/%u/%u\n",
         integer[0], integer_med, integer[(RUNS * 95U) / 100U], integer[RUNS - 1]);
  printk("  Float 4096-op cycles min/median/p95/max: %u/%u/%u/%u\n",
         floating[0], floating_med, floating[(RUNS * 95U) / 100U], floating[RUNS - 1]);
  printk("  DSP block cycles min/avg/median/p95/max: %u/%u/%u/%u/%u; avg %u us, median %u us, %.2f us/sample, %.1f%% of %u us deadline; equivalent %.0f samples/s; deadline overruns=%u/%u\n",
         dsp[0], dsp_avg_cycles, dsp_med, dsp_p95, dsp_max, dsp_avg_us, dsp_us,
         SAMPLES ? (double)dsp_us / SAMPLES : 0.0,
         deadline_us ? (double)dsp_us * 100.0 / deadline_us : 0.0, deadline_us,
         dsp_avg_us ? (double)SAMPLES * 1000000.0 / dsp_avg_us : 0.0,
         dsp_overruns, RUNS);
  dsp_reset();
  benchmark_set_running(false);
  ARG_UNUSED(checksum);
  ARG_UNUSED(integer_sink);
  ARG_UNUSED(float_sink);
  ARG_UNUSED(stage_sink);
  recorder_return_contiguous_pool();
  return 0;
}

int bench_run_adc(void) {
  if (recorder_is_active() || benchmark_is_running()) return -EBUSY;
#if !defined(CONFIG_AUDIO_SRC_ADC_STM32L4)
  printk("ADC benchmark SKIPPED: firmware is not built with the STM32L4 ADC backend.\n");
  return -ENOTSUP;
#else
  benchmark_set_running(true);
  static const uint32_t rates[] = {16000, 32000, 44100, 48000, 96000, 192000};
  void *discard_pool[1] = {0};
  int pool_ret = recorder_borrow_pool_blocks(discard_pool, 1);
  if (pool_ret != 0) {
    benchmark_set_running(false);
    return pool_ret;
  }
  adc_discard_block = discard_pool[0];
  const struct audio_sink sink = {
      .alloc = adc_discard_alloc,
      .submit = adc_discard_submit,
      .user = NULL,
  };
  int ret = audio_in_init(rates[0], &sink);
  if (ret != 0) {
    printk("ADC benchmark init failed: %d\n", ret);
    recorder_return_pool_blocks(discard_pool, 1);
    adc_discard_block = NULL;
    benchmark_set_running(false);
    return ret;
  }
  const uint32_t duration_s = 20;
  printk("\nADC/DMA acquisition benchmark: discard sink, no DSP or storage, %u seconds per supported rate.\n",
         duration_s);
  printk("Rate target/actual Hz | elapsed ms | expected/acquired samples | HT/TC callbacks | measured Hz | status\n");
  int failures = 0;
  for (uint32_t i = 0; i < ARRAY_SIZE(rates); i++) {
    uint32_t actual = 0, divider = 0;
    int error_hz = 0;
    if (!audio_in_is_rate_supported(rates[i], &actual, &divider, &error_hz)) {
      printk("%6u Hz | SKIPPED: rate unsupported by timer calculation\n", rates[i]);
      continue;
    }
    ret = audio_in_set_rate(rates[i]);
    if (ret != 0) {
      printk("%6u/%6u Hz | SKIPPED: configured conversion budget rejects rate (%d)\n",
             rates[i], actual, ret);
      continue;
    }
    ret = audio_in_start();
    if (ret != 0) {
      printk("%6u/%6u Hz | START FAILED (%d)\n", rates[i], audio_in_rate(), ret);
      failures++;
      continue;
    }
    int64_t t0 = k_uptime_get();
    k_sleep(K_SECONDS(duration_s));
    ret = audio_in_stop();
    int64_t elapsed_ms = k_uptime_get() - t0;
    struct audio_diag diag;
    struct audio_dma_stats dma_stats;
    uint32_t ht = 0, tc = 0;
    audio_in_get_diag(&diag);
    audio_in_get_callback_counts(&ht, &tc);
    audio_in_get_dma_stats(&dma_stats);
    uint64_t expected = ((uint64_t)diag.actual_rate * (uint64_t)elapsed_ms) / 1000U;
    uint32_t measured = elapsed_ms > 0 ? (uint32_t)((uint64_t)diag.total_samples * 1000U / elapsed_ms) : 0;
    uint64_t delta = (diag.total_samples > expected) ? diag.total_samples - expected : expected - diag.total_samples;
    bool ok = (ret == 0) && (delta <= 2U * CONFIG_AUDIO_BLOCK_SAMPLES) &&
              (diag.total_samples == (ht + tc) * CONFIG_AUDIO_BLOCK_SAMPLES) &&
              (dma_stats.callback_errors == 0) && (dma_stats.adc_overruns == 0) &&
              (ht <= tc + 1U) && (tc <= ht + 1U);
    uint32_t interval_target_us = (uint32_t)(((uint64_t)CONFIG_AUDIO_BLOCK_SAMPLES * 1000000U) /
                                             diag.actual_rate);
    uint32_t int_min_us = (uint32_t)k_cyc_to_us_floor64(dma_stats.callback_interval_min_cycles);
    uint32_t int_avg_us = (uint32_t)k_cyc_to_us_floor64(dma_stats.callback_interval_avg_cycles);
    uint32_t int_max_us = (uint32_t)k_cyc_to_us_floor64(dma_stats.callback_interval_max_cycles);
    uint32_t exec_avg_us = (uint32_t)k_cyc_to_us_floor64(dma_stats.callback_exec_avg_cycles);
    uint32_t exec_max_us = (uint32_t)k_cyc_to_us_floor64(dma_stats.callback_exec_max_cycles);
    printk("%6u/%6u Hz | %9lld | %9llu/%9u | %6u/%6u | %9u | %s (div=%u err=%d)\n",
           rates[i], diag.actual_rate, elapsed_ms, expected, diag.total_samples,
           ht, tc, measured, ok ? "PASS" : "FAIL", divider, error_hz);
    printk("    interval min/avg/max=%u/%u/%u us (target block interval %u us); callback CPU avg/max=%u/%u us; ADC OVR=%u DMA callback errors=%u\n",
           int_min_us, int_avg_us, int_max_us, interval_target_us,
           exec_avg_us, exec_max_us, dma_stats.adc_overruns, dma_stats.callback_errors);
    if (!ok) failures++;
  }
  recorder_return_pool_blocks(discard_pool, 1);
  adc_discard_block = NULL;
  benchmark_set_running(false);
  return failures ? -EIO : 0;
#endif
}

/* ========================================================================= */
/* 1. INTERNAL SRAM BENCHMARK                                                */
/* ========================================================================= */
int bench_run_ram(void) {
  if (recorder_is_active()) {
    printk("[ERROR] Cannot run SRAM benchmark while audio recording is active.\n");
    return -EBUSY;
  }
  if (benchmark_is_running()) {
    printk("[ERROR] Another benchmark is already running.\n");
    return -EBUSY;
  }
  benchmark_set_running(true);

  uint32_t hclk_mhz = HAL_RCC_GetHCLKFreq() / 1000000U;
  printk("\nINTERNAL RAM BENCHMARK - MCU %u MHz HCLK\n\n", hclk_mhz);
  printk("Size (B) | Write (MB/s) | Read (MB/s) | Copy (MB/s) | Integrity\n");
  printk("---------------------------------------------------------------\n");

  static const uint32_t sizes[] = {256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
  void *pool = NULL;
  uint32_t pool_size = 0;
  int overall_ret = recorder_borrow_contiguous_pool(&pool, &pool_size);
  if (overall_ret != 0 || !pool || pool_size < 2U * sizes[ARRAY_SIZE(sizes) - 1]) {
    printk("[ERROR] Cannot borrow two bounded SRAM buffers for the RAM benchmark (%d, %u B).\n",
           overall_ret, pool_size);
    benchmark_set_running(false);
    return overall_ret ? overall_ret : -ENOMEM;
  }

  uint8_t *src = pool;
  uint8_t *dst = src + pool_size / 2;
  const uint32_t runs = 5;
  const uint64_t bytes_per_run = 1024U * 1024U;
  uint32_t sink = 0;
  for (uint32_t c = 0; c < ARRAY_SIZE(sizes); c++) {
    const uint32_t size = sizes[c];
    const uint32_t iters = (uint32_t)(bytes_per_run / size);
    uint32_t w_times[5] = {0}, r_times[5] = {0}, cp_times[5] = {0};
    bool integrity = true;
    for (uint32_t run = 0; run <= runs; run++) {
      uint32_t measured = (run == 0) ? UINT32_MAX : run - 1;
      uint64_t t0 = k_cycle_get_64();
      for (uint32_t it = 0; it < iters; it++) {
        volatile uint8_t *p = src;
        for (uint32_t i = 0; i < size; i++) p[i] = (uint8_t)((i + it * 13U + c * 29U) ^ 0xA5U);
      }
      uint32_t w_us = (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64() - t0);
      for (uint32_t i = 0; i < size; i++) {
        uint8_t exp = (uint8_t)((i + (iters - 1U) * 13U + c * 29U) ^ 0xA5U);
        if (src[i] != exp) integrity = false;
      }

      volatile uint32_t accum = 0;
      t0 = k_cycle_get_64();
      for (uint32_t it = 0; it < iters; it++) {
        const volatile uint32_t *p = (const volatile uint32_t *)src;
        for (uint32_t i = 0; i < size / sizeof(uint32_t); i++) accum += p[i];
      }
      uint32_t r_us = (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64() - t0);
      sink ^= accum;

      t0 = k_cycle_get_64();
      for (uint32_t it = 0; it < iters; it++) memcpy(dst, src, size);
      uint32_t cp_us = (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64() - t0);
      if (memcmp(src, dst, size) != 0) integrity = false;
      if (measured != UINT32_MAX) {
        w_times[measured] = w_us;
        r_times[measured] = r_us;
        cp_times[measured] = cp_us;
      }
    }
    uint64_t w_sum = 0, r_sum = 0, cp_sum = 0;
    uint32_t w_min = UINT32_MAX, r_min = UINT32_MAX, cp_min = UINT32_MAX;
    uint32_t w_max = 0, r_max = 0, cp_max = 0;
    uint32_t w_rate[5], r_rate[5], cp_rate[5];
    for (uint32_t i = 0; i < runs; i++) {
      w_sum += w_times[i]; r_sum += r_times[i]; cp_sum += cp_times[i];
      w_rate[i] = w_times[i] ? (uint32_t)(bytes_per_run * 1000U / w_times[i]) : 0;
      r_rate[i] = r_times[i] ? (uint32_t)(bytes_per_run * 1000U / r_times[i]) : 0;
      cp_rate[i] = cp_times[i] ? (uint32_t)(bytes_per_run * 1000U / cp_times[i]) : 0;
      if (w_times[i] < w_min) w_min = w_times[i];
      if (w_times[i] > w_max) w_max = w_times[i];
      if (r_times[i] < r_min) r_min = r_times[i];
      if (r_times[i] > r_max) r_max = r_times[i];
      if (cp_times[i] < cp_min) cp_min = cp_times[i];
      if (cp_times[i] > cp_max) cp_max = cp_times[i];
    }
    sort_u32(w_rate, runs); sort_u32(r_rate, runs); sort_u32(cp_rate, runs);
    uint64_t w_rate_sum = 0, r_rate_sum = 0, cp_rate_sum = 0;
    for (uint32_t i = 0; i < runs; i++) {
      w_rate_sum += w_rate[i]; r_rate_sum += r_rate[i]; cp_rate_sum += cp_rate[i];
    }
    double w_avg = (double)w_rate_sum / (runs * 1000.0);
    double r_avg = (double)r_rate_sum / (runs * 1000.0);
    double cp_avg = (double)cp_rate_sum / (runs * 1000.0);
    /*
     * Display representative throughput in MB/s (10^6 bytes/s), calculated as the
     * arithmetic mean of the 5 timed repetitions for each buffer size.
     */
    if (integrity) {
      printk("%8u | %12.2f | %11.2f | %11.2f |      PASS\n",
             size, w_avg, r_avg, cp_avg);
    } else {
      printk("%8u |          --- |         --- |         --- |      FAIL\n",
             size);
    }
    if (!integrity) overall_ret = -EIO;
    s_bench_summary.ram_tested = true;
    s_bench_summary.ram_last.chunk_size = size;
    s_bench_summary.ram_last.iterations = iters * runs;
    s_bench_summary.ram_last.total_bytes = bytes_per_run * runs;
    s_bench_summary.ram_last.write_us = (uint32_t)(w_sum / runs);
    s_bench_summary.ram_last.read_us = (uint32_t)(r_sum / runs);
    s_bench_summary.ram_last.copy_us = (uint32_t)(cp_sum / runs);
    s_bench_summary.ram_last.write_mb_s = (uint32_t)w_avg;
    s_bench_summary.ram_last.read_mb_s = (uint32_t)r_avg;
    s_bench_summary.ram_last.copy_mb_s = (uint32_t)cp_avg;
    s_bench_summary.ram_last.integrity_ok = integrity;
  }
  ARG_UNUSED(sink);
  recorder_return_contiguous_pool();

  printk("---------------------------------------------------------------\n");
  printk("Overall RAM Benchmark Integrity: %s\n\n",
         (overall_ret == 0) ? "PASS" : "FAIL");
  benchmark_set_running(false);
  return overall_ret;
}

/* ========================================================================= */
/* 2. EXTERNAL PSRAM BENCHMARK                                               */
/* ========================================================================= */
int bench_run_psram(void) {
  if (recorder_is_active()) {
    printk("[ERROR] Cannot run PSRAM benchmark while audio recording is active.\n");
    return -EBUSY;
  }
  if (benchmark_is_running()) {
    printk("[ERROR] Another benchmark is already running.\n");
    return -EBUSY;
  }

  if (!psram_is_present()) {
    printk("\n[ERROR] External ISSI PSRAM is not detected on QUADSPI bus.\n");
    printk("        Verify wiring: PB11=NCS, PB10=CLK, PB1=IO0, PB0=IO1, PA7=IO2, PA6=IO3.\n\n");
    return -ENODEV;
  }

  if (!psram_is_integrity_verified()) {
    printk("\n[ERROR] PSRAM memory integrity has NOT been verified.\n");
    printk("        Run 'psram test' first to verify physical memory before benchmarking.\n\n");
    return -EACCES;
  }

  benchmark_set_running(true);

  bool is_quad = psram_is_quad_mode();
  uint32_t burst_sz = psram_get_burst_size();

  printk("\n================================================================================\n");
  printk("                  EXTERNAL ISSI IS66WVS4M8 PSRAM BENCHMARK                      \n");
  printk("================================================================================\n");
  printk("  Capacity            : 4 MBytes (32 Megabits)\n");
  printk("  Interface Mode      : %s (Burst Size: %u bytes, QSPI: %u Hz)\n",
         is_quad ? "Quad I/O (1-4-4, CMD 0x38 Write / 0xEB Read)" : "Standard 1-Line SPI (1-1-1, CMD 0x02/0x03)",
         burst_sz, psram_get_clock_hz());
  printk("  Benchmarking Range  : 0x00200000 .. 0x00240000 (Base) -> 0x00280000 (Copyback)\n");
  printk("  Staging Ring Safety : Audio staging buffer (0x00000000..0x00100000) completely untouched\n");
  printk("  CE# timing          : tCEM estimated from transfer phases; physical CE# timing is not measured\n");
  printk("--------------------------------------------------------------------------------\n");
  printk("  Transfer Size | Sequential Write    | Sequential Read     | PSRAM Copy (Copyback) | Data Check\n");
  printk("--------------------------------------------------------------------------------\n");

  static const uint32_t bench_sizes[] = {4096, 16384, 65536, 262144};
  const uint32_t base_addr = 0x00200000;
  const uint32_t copy_addr = 0x00280000;

  void *pool_buf = NULL;
  uint32_t pool_max = 0;
  int ret = recorder_borrow_contiguous_pool(&pool_buf, &pool_max);
  if (ret != 0 || !pool_buf) {
    printk("[ERROR] Failed to borrow contiguous SRAM pool for PSRAM transfer: %d\n", ret);
    benchmark_set_running(false);
    return ret;
  }

  uint8_t *sram_buf = (uint8_t *)pool_buf;
  int overall_ret = 0;

  for (int s = 0; s < ARRAY_SIZE(bench_sizes); s++) {
    uint32_t total_size = bench_sizes[s];
    uint32_t chunk_step = (total_size <= pool_max) ? total_size : pool_max;
    uint32_t num_chunks = total_size / chunk_step;
    bool integrity_pass = true;
    uint32_t mismatch_count = 0;

    /* 1. Write Benchmark */
    uint64_t t_w0 = k_cycle_get_64();
    for (uint32_t ch = 0; ch < num_chunks && integrity_pass; ch++) {
      uint32_t cur_addr = base_addr + ch * chunk_step;
      for (uint32_t i = 0; i < chunk_step; i++) {
        sram_buf[i] = bench_pattern(cur_addr, i, 0xA5U + s * 17U);
      }
      ret = psram_write(cur_addr, sram_buf, chunk_step);
      if (ret != 0) {
        printk("\n  [PSRAM WRITE ERROR] Size: %u B, Addr: 0x%08X (Chunk %u/%u): Code %d\n",
               total_size, cur_addr, ch + 1, num_chunks, ret);
        integrity_pass = false;
        overall_ret = ret;
        break;
      }
    }
    uint64_t t_w1 = k_cycle_get_64();
    uint32_t w_us = (uint32_t)k_cyc_to_us_floor64(t_w1 - t_w0);
    double w_mb_s = w_us ? ((double)total_size / (double)w_us) : 0.0;
    double w_mib_s = w_us ? ((double)total_size * 1000000.0 / ((double)w_us * 1048576.0)) : 0.0;

    /* 2. Read Benchmark and Full-Transfer Verification */
    uint64_t t_r0 = k_cycle_get_64();
    for (uint32_t ch = 0; ch < num_chunks && integrity_pass; ch++) {
      uint32_t cur_addr = base_addr + ch * chunk_step;
      /* Zero buffer before reading so old data cannot mask a read error */
      memset(sram_buf, 0x00, chunk_step);
      ret = psram_read(cur_addr, sram_buf, chunk_step);
      if (ret != 0) {
        printk("\n  [PSRAM READ ERROR] Size: %u B, Addr: 0x%08X (Chunk %u/%u): Code %d\n",
               total_size, cur_addr, ch + 1, num_chunks, ret);
        integrity_pass = false;
        overall_ret = ret;
        break;
      }
      /* Verify each byte in chunk */
      for (uint32_t i = 0; i < chunk_step; i++) {
        uint8_t exp = bench_pattern(cur_addr, i, 0xA5U + s * 17U);
        if (sram_buf[i] != exp) {
          if (mismatch_count == 0) {
            uint32_t transfer_offset = ch * chunk_step + i;
            printk("\n  [PSRAM READ MISMATCH] Mode=%s burst=%u transfer=%u B addr=0x%08X (base=0x%08X offset=+%u): expected=0x%02X actual=0x%02X diff=0x%02X\n",
                   is_quad ? "SPI-Quad 1-4-4" : "SPI 1-1-1", burst_sz,
                   total_size, cur_addr + i, base_addr, transfer_offset, exp, sram_buf[i], exp ^ sram_buf[i]);
            printk("  [PSRAM mismatch context] offset +%u..+%u expected:",
                   (i >= 4) ? i - 4 : 0,
                   (i + 4 < chunk_step) ? i + 3 : chunk_step - 1);
            for (uint32_t k = (i >= 4) ? i - 4 : 0;
                 k < chunk_step && k <= i + 3; k++) {
              printk(" %02X", bench_pattern(cur_addr, k, 0xA5U + s * 17U));
            }
            printk(" actual:");
            for (uint32_t k = (i >= 4) ? i - 4 : 0;
                 k < chunk_step && k <= i + 3; k++) {
              printk(" %02X", sram_buf[k]);
            }
            printk("\n");
          }
          mismatch_count++;
          integrity_pass = false;
        }
      }
    }
    uint64_t t_r1 = k_cycle_get_64();
    uint32_t r_us = (uint32_t)k_cyc_to_us_floor64(t_r1 - t_r0);
    double r_mb_s = r_us ? ((double)total_size / (double)r_us) : 0.0;
    double r_mib_s = r_us ? ((double)total_size * 1000000.0 / ((double)r_us * 1048576.0)) : 0.0;

    /* 3. PSRAM Copyback Benchmark (Read base_addr, Write copy_addr) & Verify */
    uint64_t t_c0 = k_cycle_get_64();
    for (uint32_t ch = 0; ch < num_chunks && integrity_pass; ch++) {
      uint32_t src_addr = base_addr + ch * chunk_step;
      uint32_t dst_addr = copy_addr + ch * chunk_step;
      ret = psram_read(src_addr, sram_buf, chunk_step);
      if (ret != 0) {
        integrity_pass = false;
        overall_ret = ret;
        break;
      }
      ret = psram_write(dst_addr, sram_buf, chunk_step);
      if (ret != 0) {
        integrity_pass = false;
        overall_ret = ret;
        break;
      }
    }
    uint64_t t_c1 = k_cycle_get_64();
    uint32_t c_us = (uint32_t)k_cyc_to_us_floor64(t_c1 - t_c0);
    double c_mb_s = c_us ? ((double)total_size / (double)c_us) : 0.0;
    double c_mib_s = c_us ? ((double)total_size * 1000000.0 / ((double)c_us * 1048576.0)) : 0.0;

    /* Verify copyback destination */
    for (uint32_t ch = 0; ch < num_chunks && integrity_pass; ch++) {
      uint32_t dst_addr = copy_addr + ch * chunk_step;
      memset(sram_buf, 0x00, chunk_step);
      ret = psram_read(dst_addr, sram_buf, chunk_step);
      if (ret != 0) {
        integrity_pass = false;
        overall_ret = ret;
        break;
      }
      for (uint32_t i = 0; i < chunk_step; i++) {
        uint8_t exp = bench_pattern(base_addr + ch * chunk_step, i, 0xA5U + s * 17U);
        if (sram_buf[i] != exp) {
          if (mismatch_count == 0) {
            printk("\n  [PSRAM COPY MISMATCH] Size: %u B, Addr: 0x%08X (Offset +%u): Exp 0x%02X, Got 0x%02X\n",
                   total_size, dst_addr, i, exp, sram_buf[i]);
          }
          mismatch_count++;
          integrity_pass = false;
        }
      }
    }

    if (integrity_pass) {
      printk("  %6u B   | %5.2f MB/s (%5.2f MiB/s) | %5.2f MB/s (%5.2f MiB/s) | %5.2f MB/s (%5.2f MiB/s)  | PASS\n",
             total_size, w_mb_s, w_mib_s, r_mb_s, r_mib_s, c_mb_s, c_mib_s);
    } else {
      printk("  %6u B   | INVALID (write/read/copy integrity failed; measured rates discarded) | FAIL\n",
             total_size);
      if (overall_ret == 0) overall_ret = -EIO;
    }

    s_bench_summary.psram_tested = true;
    s_bench_summary.psram_last.chunk_size = total_size;
    s_bench_summary.psram_last.total_bytes = total_size;
    s_bench_summary.psram_last.write_us = w_us;
    s_bench_summary.psram_last.read_us = r_us;
    s_bench_summary.psram_last.copy_us = c_us;
    s_bench_summary.psram_last.write_mb_s = (uint32_t)w_mb_s;
    s_bench_summary.psram_last.read_mb_s = (uint32_t)r_mb_s;
    s_bench_summary.psram_last.copy_mb_s = (uint32_t)c_mb_s;
    s_bench_summary.psram_last.write_mib_s = (uint32_t)w_mib_s;
    s_bench_summary.psram_last.read_mib_s = (uint32_t)r_mib_s;
    s_bench_summary.psram_last.copy_mib_s = (uint32_t)c_mib_s;
    s_bench_summary.psram_last.quad_mode = is_quad;
    s_bench_summary.psram_last.integrity_ok = integrity_pass;
  }

  recorder_return_contiguous_pool();

  printk("--------------------------------------------------------------------------------\n");
  printk("  Real-time 192 kHz Margin: 4096-byte arrival deadline is 10,666 us.\n");
  printk("================================================================================\n\n");

  benchmark_set_running(false);
  return overall_ret;
}

/* ========================================================================= */
/* 3. SD-CARD WRITE & LATENCY BENCHMARK                                      */
/* ========================================================================= */
struct sd_bench_row {
  uint32_t block_sz;
  uint32_t file_bytes;
  uint32_t write_us;
  uint32_t sync_us;
  uint32_t read_us;
  double w_mb_s;
  double w_mib_s;
  double r_mb_s;
  double r_mib_s;
  double effective_mb_s;
  double effective_mib_s;
  uint32_t lat_min;
  uint32_t lat_avg;
  uint32_t lat_med;
  uint32_t lat_p95;
  uint32_t lat_max;
  uint32_t latency_samples;
  uint32_t read_lat_max;
  uint32_t errors;
  bool ok;
};

int bench_run_sd_mode(enum sd_bench_mode mode) {
  if (recorder_is_active()) {
    printk("[ERROR] Cannot run SD benchmark while audio recording is active.\n");
    return -EBUSY;
  }
  if (benchmark_is_running()) {
    printk("[ERROR] Another benchmark is already running.\n");
    return -EBUSY;
  }

  benchmark_set_running(true);
  int64_t t_bench_start = k_uptime_get();

  uint32_t sd_source = LL_RCC_GetSDMMCClockSource(LL_RCC_SDMMC1_CLKSOURCE);
  uint32_t sd_kernel_hz = (sd_source == LL_RCC_SDMMC1_CLKSOURCE_MSI)
                              ? benchmark_msi_clock_hz(LL_RCC_MSI_GetRange())
                              : 0U;
  uint32_t sd_clkcr = SDMMC1->CLKCR;
  uint32_t sd_div = (sd_clkcr & SDMMC_CLKCR_CLKDIV) >> SDMMC_CLKCR_CLKDIV_Pos;
  uint32_t sd_bus_hz = sd_div ? sd_kernel_hz / (2U * sd_div) : sd_kernel_hz;
  uint32_t sd_bus_width = (sd_clkcr & SDMMC_CLKCR_WIDBUS) >> SDMMC_CLKCR_WIDBUS_Pos;
  uint32_t card_sectors = 0, card_sector_bytes = 0;
  int sector_count_ret = disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_COUNT, &card_sectors);
  int sector_size_ret = disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_SIZE, &card_sector_bytes);

  static const uint32_t quick_sizes[] = {4096, 16384, 65536};
  static const uint32_t medium_sizes[] = {4096, 16384, 65536};
  static const uint32_t full_sizes[] = {512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};

  const uint32_t *sd_block_sizes;
  uint32_t num_sizes;
  uint32_t total_bytes;
  uint32_t measured_reps;
  const char *mode_name;

  if (mode == SD_BENCH_MEDIUM) {
    sd_block_sizes = medium_sizes;
    num_sizes = ARRAY_SIZE(medium_sizes);
    total_bytes = 4U * 1024U * 1024U; /* 4 MiB */
    measured_reps = 3;
    mode_name = "MEDIUM (4K, 16K, 64K; 4 MiB file; 1 warmup + 3 measured runs)";
  } else if (mode == SD_BENCH_FULL) {
    sd_block_sizes = full_sizes;
    num_sizes = ARRAY_SIZE(full_sizes);
    total_bytes = 8U * 1024U * 1024U; /* 8 MiB */
    measured_reps = 5;
    mode_name = "FULL OPT-IN (512 B to 64 KiB; 8 MiB file; 5 repetitions)";
  } else { /* SD_BENCH_QUICK */
    sd_block_sizes = quick_sizes;
    num_sizes = ARRAY_SIZE(quick_sizes);
    total_bytes = 512U * 1024U; /* 512 KiB */
    measured_reps = 3;
    mode_name = "QUICK (4K, 16K, 64K; 512 KiB file; 1 warmup + 3 measured runs)";
  }

  printk("\n================================================================================\n");
  printk("              NATIVE SDMMC MICROSD SEQUENTIAL WRITE/READ BENCHMARK              \n");
  printk("================================================================================\n");
  printk("  Target Media        : MicroSD Card via Native SDMMC Interface (4-bit Bus)\n");
  printk("  Runtime SDMMC       : source=%s kernel=%u Hz divider=%u bus=%u Hz width=%s\n",
         (sd_source == LL_RCC_SDMMC1_CLKSOURCE_MSI) ? "MSI" : "other/unknown",
         sd_kernel_hz, sd_div, sd_bus_hz,
         sd_bus_width == 1U ? "4-bit" : (sd_bus_width == 2U ? "8-bit" : "1-bit"));
  printk("  Transfer engine     : Interrupt-driven (HAL IT path, bounded 5000 ms timeouts)\n");
  printk("  Benchmark Mode      : %s\n", mode_name);
  uint64_t estimated_io = (uint64_t)total_bytes * num_sizes * (measured_reps + 1U) * 2U;
  printk("  Estimated total I/O : %llu bytes (write + read/verify; includes warmup)\n", estimated_io);
  if (sector_count_ret == 0 && sector_size_ret == 0) {
    printk("  Card capacity       : %u sectors x %u B = %u MiB\n",
           card_sectors, card_sector_bytes,
           (uint32_t)(((uint64_t)card_sectors * card_sector_bytes) / 1048576U));
  }
  printk("  Dedicated Test File : /SD:/BENCH.DAT (removed upon test completion)\n");
  printk("  Safety Guarantee    : Existing /SD:/RECnnnn.WAV files are strictly protected\n");
  printk("  Memory Safety       : Contiguous 64 KiB buffer strictly bounded in SRAM1\n");
  printk("--------------------------------------------------------------------------------\n");

  const char *test_path = "/SD:/BENCH.DAT";
  int overall_ret = 0;
  struct fs_statvfs vfs;
  int vfs_ret = fs_statvfs("/SD:/", &vfs);
  uint64_t free_bytes = (vfs_ret == 0) ? (uint64_t)vfs.f_bfree * vfs.f_frsize : 0;
  if (vfs_ret == 0) {
    printk("  Filesystem free     : %llu bytes\n", free_bytes);
    if (free_bytes < total_bytes) {
      printk("[ERROR] Need at least %u bytes free for BENCH.DAT; aborting before file changes.\n",
             total_bytes);
      benchmark_set_running(false);
      return -ENOSPC;
    }
  }

  void *pool_buf = NULL;
  uint32_t pool_max = 0;
  int b_ret = recorder_borrow_contiguous_pool(&pool_buf, &pool_max);
  if (b_ret != 0 || !pool_buf) {
    printk("[ERROR] Failed to borrow contiguous SRAM pool for SD benchmark: %d\n", b_ret);
    benchmark_set_running(false);
    return b_ret;
  }

  uint8_t *wbuf = (uint8_t *)pool_buf;
  static struct sd_bench_row summary_rows[8];
  memset(summary_rows, 0, sizeof(summary_rows));
  uint32_t rows_completed = 0;

  for (uint32_t c = 0; c < num_sizes; c++) {
    uint32_t block_sz = sd_block_sizes[c];
    uint32_t num_blks = total_bytes / block_sz;

    if ((uintptr_t)wbuf < 0x20000000 || ((uintptr_t)wbuf + block_sz) > 0x20018000 || block_sz > pool_max) {
      printk("[FATAL ERROR] Buffer pointer bounds check failed: %p + %u > 0x20018000\n",
             wbuf, block_sz);
      overall_ret = -EFAULT;
      break;
    }

    uint32_t write_rates_milli[5] = {0};
    uint32_t read_rates_milli[5] = {0};
    uint32_t valid_reps = 0;

    struct sd_bench_row *row = &summary_rows[c];
    row->block_sz = block_sz;
    row->file_bytes = total_bytes;
    row->ok = true;

    /* 1 warm-up run (rep 0) + measured_reps (rep 1..measured_reps) */
    for (uint32_t rep = 0; rep <= measured_reps; rep++) {
      bool warmup = (rep == 0);

      /* Print concise progress per phase */
      if (!warmup && measured_reps > 1) {
        printk("  [SD %u/%u] block=%5u B (rep %u/%u): write...",
               c + 1, num_sizes, block_sz, rep, measured_reps);
      } else if (!warmup) {
        printk("  [SD %u/%u] block=%5u B (%u KiB file): write...",
               c + 1, num_sizes, block_sz, total_bytes / 1024U);
      }

      /* Deterministic data fill */
      for (uint32_t i = 0; i < block_sz; i++) {
        wbuf[i] = (uint8_t)((i * 37U + c * 13U + 0x55U + rep * 7U) & 0xFFU);
      }

      struct fs_file_t file;
      fs_file_t_init(&file);

      int ret = fs_open(&file, test_path, FS_O_CREATE | FS_O_READ | FS_O_WRITE | FS_O_TRUNC);
      if (ret != 0) {
        printk("\n[ERROR] Failed to open %s: %d\n", test_path, ret);
        overall_ret = ret;
        row->ok = false;
        row->errors++;
        goto sd_storage_abort;
      }

      static uint32_t latencies[128];
      static uint32_t read_latencies[128];
      uint32_t lat_count = (num_blks <= 128) ? num_blks : 128;
      uint32_t write_errors = 0;
      uint32_t max_write_op_us = 0;
      uint32_t max_read_op_us = 0;

      uint64_t t_write_start = k_cycle_get_64();
      for (uint32_t b = 0; b < num_blks; b++) {
        uint64_t t0 = k_cycle_get_64();
        ssize_t w = fs_write(&file, wbuf, block_sz);
        uint64_t t1 = k_cycle_get_64();

        if (w != (ssize_t)block_sz) {
          write_errors++;
          overall_ret = (w < 0) ? (int)w : -EIO;
          printk("\n  [SD WRITE ERROR] block %u/%u returned %d (expected %u); disk status=%d\n",
                 b + 1, num_blks, (int)w, block_sz, disk_access_status("SD"));
          row->ok = false;
          row->errors++;
          goto sd_storage_abort;
        }

        uint32_t lat_us = (uint32_t)k_cyc_to_us_floor64(t1 - t0);
        if (lat_us > max_write_op_us) max_write_op_us = lat_us;
        if (b < lat_count || (num_blks > lat_count && (b % (num_blks / lat_count)) == 0)) {
          uint32_t sample = (num_blks <= lat_count) ? b : (b / (num_blks / lat_count));
          if (sample < lat_count) latencies[sample] = lat_us;
        }
      }
      uint64_t t_write_end = k_cycle_get_64();
      uint32_t write_us = (uint32_t)k_cyc_to_us_floor64(t_write_end - t_write_start);

      if (!warmup) printk(" sync...");

      /* Sync */
      uint64_t t_sync_start = k_cycle_get_64();
      ret = fs_sync(&file);
      uint64_t t_sync_end = k_cycle_get_64();
      uint32_t sync_us = (uint32_t)k_cyc_to_us_floor64(t_sync_end - t_sync_start);
      if (ret != 0) {
        printk("\n  [SD SYNC ERROR] fs_sync failed: %d\n", ret);
        overall_ret = ret;
        row->ok = false;
        row->errors++;
        goto sd_storage_abort;
      }

      /* Close before reading back */
      ret = fs_close(&file);
      if (ret != 0) {
        printk("\n  [SD CLOSE ERROR] fs_close failed: %d\n", ret);
        overall_ret = ret;
        row->ok = false;
        row->errors++;
        goto sd_storage_abort;
      }

      if (!warmup) printk(" read+verify...");

      /* Re-open read-only */
      fs_file_t_init(&file);
      ret = fs_open(&file, test_path, FS_O_READ);
      if (ret != 0) {
        printk("\n  [SD REOPEN ERROR] read open failed: %d\n", ret);
        overall_ret = ret;
        row->ok = false;
        row->errors++;
        goto sd_storage_abort;
      }

      /* Readback verification */
      uint64_t t_read_start = k_cycle_get_64();
      bool data_ok = true;
      for (uint32_t b = 0; b < num_blks; b++) {
        uint64_t t0 = k_cycle_get_64();
        ssize_t r = fs_read(&file, wbuf, block_sz);
        uint64_t t1 = k_cycle_get_64();

        uint32_t read_lat_us = (uint32_t)k_cyc_to_us_floor64(t1 - t0);
        if (read_lat_us > max_read_op_us) max_read_op_us = read_lat_us;
        if (b < lat_count || (num_blks > lat_count && (b % (num_blks / lat_count)) == 0)) {
          uint32_t sample = (num_blks <= lat_count) ? b : (b / (num_blks / lat_count));
          if (sample < lat_count) read_latencies[sample] = read_lat_us;
        }

        if (r != (ssize_t)block_sz) {
          printk("\n  [SD READ ERROR] block %u/%u ret=%d (expected %u)\n",
                 b + 1, num_blks, (int)r, block_sz);
          data_ok = false;
          overall_ret = (r < 0) ? (int)r : -EIO;
          row->ok = false;
          row->errors++;
          goto sd_storage_abort;
        }

        for (uint32_t i = 0; i < block_sz; i++) {
          uint8_t exp = (uint8_t)((i * 37U + c * 13U + 0x55U + rep * 7U) & 0xFFU);
          if (wbuf[i] != exp) {
            printk("\n  [SD DATA MISMATCH] block=%u offset=%u expected=0x%02X actual=0x%02X\n",
                   b, i, exp, wbuf[i]);
            data_ok = false;
            overall_ret = -EIO;
            row->ok = false;
            row->errors++;
            goto sd_storage_abort;
          }
        }
      }
      uint64_t t_read_end = k_cycle_get_64();
      uint32_t read_us = (uint32_t)k_cyc_to_us_floor64(t_read_end - t_read_start);

      struct fs_dirent stat_ent = {0};
      ret = fs_stat(test_path, &stat_ent);
      if (ret != 0 || stat_ent.size != total_bytes) {
        printk("\n  [SD STAT ERROR] size verification failed: ret=%d size=%u expected=%u\n",
               ret, (unsigned int)stat_ent.size, total_bytes);
        overall_ret = (ret != 0) ? ret : -EIO;
        row->ok = false;
        row->errors++;
        goto sd_storage_abort;
      }
      ret = fs_close(&file);
      if (ret != 0) {
        printk("\n  [SD CLOSE ERROR] readback close failed: %d\n", ret);
        overall_ret = ret;
        row->ok = false;
        row->errors++;
        goto sd_storage_abort;
      }
      ret = fs_unlink(test_path);
      if (ret != 0) {
        printk("\n  [SD CLEANUP ERROR] unlink %s failed: %d\n", test_path, ret);
        overall_ret = ret;
        row->ok = false;
        row->errors++;
        goto sd_storage_abort;
      }
      k_sleep(K_MSEC(1)); /* RTOS housekeeping */

      sort_u32(latencies, lat_count);
      sort_u32(read_latencies, lat_count);

      uint64_t sum_lat = 0;
      for (uint32_t i = 0; i < lat_count; i++) sum_lat += latencies[i];

      double w_mb_s = write_us ? ((double)total_bytes / (double)write_us) : 0.0;
      double w_mib_s = write_us ? ((double)total_bytes * 1000000.0 / ((double)write_us * 1048576.0)) : 0.0;
      double r_mb_s = read_us ? ((double)total_bytes / (double)read_us) : 0.0;
      double r_mib_s = read_us ? ((double)total_bytes * 1000000.0 / ((double)read_us * 1048576.0)) : 0.0;
      uint32_t effective_us = write_us + sync_us;
      double effective_mb_s = effective_us ? (double)total_bytes / effective_us : 0.0;
      double effective_mib_s = effective_us ? ((double)total_bytes * 1000000.0 / ((double)effective_us * 1048576.0)) : 0.0;

      if (!warmup) {
        printk(" [PASS] fs_write %.3f MB/s (%.3f MiB/s), sync %u us, effective-write %.3f MB/s (%.3f MiB/s), read+verify %.3f MB/s\n",
               w_mb_s, w_mib_s, sync_us, effective_mb_s, effective_mib_s, r_mb_s);

        /* Save metrics to summary row */
        row->write_us = write_us;
        row->sync_us = sync_us;
        row->read_us = read_us;
        row->w_mb_s = w_mb_s;
        row->w_mib_s = w_mib_s;
        row->r_mb_s = r_mb_s;
        row->r_mib_s = r_mib_s;
        row->effective_mb_s = effective_mb_s;
        row->effective_mib_s = effective_mib_s;
        row->lat_min = lat_count ? latencies[0] : 0;
        row->lat_avg = lat_count ? (uint32_t)(sum_lat / lat_count) : 0;
        row->lat_med = lat_count ? latencies[lat_count / 2] : 0;
        row->lat_p95 = lat_count ? latencies[((lat_count * 95U + 99U) / 100U) - 1U] : 0;
        row->lat_max = max_write_op_us;
        row->latency_samples = lat_count;
        row->read_lat_max = max_read_op_us;

        if (rep <= 5 && write_us && read_us) {
          write_rates_milli[valid_reps] = (uint32_t)(((uint64_t)total_bytes * 1000U) / write_us);
          read_rates_milli[valid_reps] = (uint32_t)(((uint64_t)total_bytes * 1000U) / read_us);
          valid_reps++;
        }

        s_bench_summary.sd_tested = true;
        s_bench_summary.sd_last.block_size = block_sz;
        s_bench_summary.sd_last.total_blocks = num_blks;
        s_bench_summary.sd_last.total_bytes = total_bytes;
        s_bench_summary.sd_last.write_us = write_us;
        s_bench_summary.sd_last.sync_us = sync_us;
        s_bench_summary.sd_last.throughput_mb_s_milli = (uint32_t)(w_mb_s * 1000.0);
        s_bench_summary.sd_last.throughput_mib_s_milli = (uint32_t)(w_mib_s * 1000.0);
        s_bench_summary.sd_last.lat_min_us = row->lat_min;
        s_bench_summary.sd_last.lat_avg_us = row->lat_avg;
        s_bench_summary.sd_last.lat_med_us = row->lat_med;
        s_bench_summary.sd_last.lat_p95_us = row->lat_p95;
        s_bench_summary.sd_last.lat_max_us = row->lat_max;
        s_bench_summary.sd_last.write_errors = write_errors;
        s_bench_summary.sd_last.integrity_ok = data_ok;
      }
    } /* end rep loop */

    if (measured_reps > 1 && valid_reps == measured_reps) {
      sort_u32(write_rates_milli, valid_reps);
      sort_u32(read_rates_milli, valid_reps);
      uint64_t wsum = 0, rsum = 0;
      for (uint32_t i = 0; i < valid_reps; i++) {
        wsum += write_rates_milli[i];
        rsum += read_rates_milli[i];
      }
      printk("    %u-run throughput MB/s min/avg/median/max (range): fs_write %.3f/%.3f/%.3f/%.3f (%.3f); fs_read %.3f/%.3f/%.3f/%.3f (%.3f)\n",
             valid_reps,
             write_rates_milli[0] / 1000.0, (double)wsum / (valid_reps * 1000.0),
             write_rates_milli[valid_reps / 2] / 1000.0, write_rates_milli[valid_reps - 1] / 1000.0,
             (write_rates_milli[valid_reps - 1] - write_rates_milli[0]) / 1000.0,
             read_rates_milli[0] / 1000.0, (double)rsum / (valid_reps * 1000.0),
             read_rates_milli[valid_reps / 2] / 1000.0, read_rates_milli[valid_reps - 1] / 1000.0,
             (read_rates_milli[valid_reps - 1] - read_rates_milli[0]) / 1000.0);
    }
    rows_completed++;
  } /* end sizes loop */

  recorder_return_contiguous_pool();

  int64_t t_bench_end = k_uptime_get();
  double elapsed_sec = (double)(t_bench_end - t_bench_start) / 1000.0;

  /* Print Compact Summary Table */
  printk("\n========================================================================================================================\n");
  printk("                              SDMMC BENCHMARK SUMMARY (DURATION: %.2f s)                                                \n", elapsed_sec);
  printk("========================================================================================================================\n");
  printk("  Block Size | Total Data | fs_write us / MB/s / MiB/s | fs_sync us | fs_read us / MB/s / MiB/s  | Write min/avg/med/p95/max us (n; all-op max), read all-op max us | Status\n");
  printk("------------------------------------------------------------------------------------------------------------------------\n");
  for (uint32_t i = 0; i < rows_completed; i++) {
    struct sd_bench_row *r = &summary_rows[i];
    if (r->ok) {
      printk("  %6u B   |  %4u KiB   | %7u us / %5.2f / %5.2f | %6u us   | %7u us / %5.2f / %5.2f | %5u/%5u/%5u/%5u/%5u (%u), %u | PASS\n",
             r->block_sz, r->file_bytes / 1024U,
             r->write_us, r->w_mb_s, r->w_mib_s,
             r->sync_us,
             r->read_us, r->r_mb_s, r->r_mib_s,
             r->lat_min, r->lat_avg, r->lat_med, r->lat_p95, r->lat_max,
             r->latency_samples, r->read_lat_max);
    } else {
      printk("  %6u B   |  %4u KiB   | INVALID (transfers failed / aborted)                                                          |  FAIL\n",
             r->block_sz, r->file_bytes / 1024U);
    }
  }
  printk("========================================================================================================================\n\n");

  benchmark_set_running(false);
  return overall_ret;

sd_storage_abort:
  recorder_return_contiguous_pool();
  printk("[SD ABORT] Benchmark halted on first storage error (%d). Incomplete file left intact for diagnosis.\n",
         overall_ret);
  printk("[SD RECOVERY] unmounting, reinitializing SDMMC, then remounting exFAT...\n");
  int recovery_ret = recorder_storage_recover();
  printk("[SD RECOVERY] result=%d disk status=%d\n", recovery_ret, disk_access_status("SD"));
  benchmark_set_running(false);
  return overall_ret;
}

int bench_run_sd(void) {
  return bench_run_sd_mode(SD_BENCH_QUICK);
}

/* ========================================================================= */
/* 4. COMBINED PIPELINE BENCHMARK                                            */
/* ========================================================================= */
int bench_run_pipeline(uint32_t target_rate, bool use_psram) {
  if (recorder_is_active()) {
    printk("[ERROR] Cannot run pipeline benchmark while audio recording is active.\n");
    return -EBUSY;
  }
  if (benchmark_is_running()) {
    printk("[ERROR] Another benchmark is already running.\n");
    return -EBUSY;
  }
  if (target_rate != 48000U && target_rate != 96000U && target_rate != 192000U &&
      target_rate != 256000U && target_rate != 384000U && target_rate != 512000U) {
    printk("[ERROR] Synthetic pipeline supports 48/96/192/256/384/512 kHz rates only.\n");
    return -EINVAL;
  }

  if (use_psram && (!psram_is_present() || !psram_is_integrity_verified())) {
    printk("[ERROR] Cannot run PSRAM-staged pipeline benchmark: PSRAM integrity unverified.\n");
    printk("        Run 'psram test' first, or test internal RAM path ('bench pipeline %u ram').\n", target_rate);
    return -EACCES;
  }

  benchmark_set_running(true);

  /* 2048 samples = 4096 bytes per block */
  const uint32_t block_samples = 2048;
  const uint32_t block_bytes = 4096;
  const uint32_t duration_sec = 20;
  uint32_t byte_rate = target_rate * 2;
  uint32_t total_blocks = (target_rate * duration_sec) / block_samples;
  uint64_t target_bytes = (uint64_t)total_blocks * block_bytes;
  uint32_t interval_us = (uint32_t)(((uint64_t)block_samples * 1000000ULL) / target_rate);

  printk("\n================================================================================\n");
  printk("          SYNTHETIC PACED STORAGE-DRAIN SIMULATION (NOT LIVE AUDIO)              \n");
  printk("================================================================================\n");
  printk("  Target Sample Rate  : %u Hz synthetic PCM16 mono (Data Rate: %u B/s, %.3f MB/s, %.3f MiB/s)\n",
         target_rate, byte_rate, (double)byte_rate / 1000000.0, (double)byte_rate / 1048576.0);
  uint64_t total_samples = (uint64_t)total_blocks * block_samples;
  printk("  Session Duration    : requested %u s; generated %llu samples (%.6f s at target rate), %llu bytes\n",
         duration_sec, total_samples, (double)total_samples / target_rate, target_bytes);
  printk("  Block Period (Rate) : %u us per 4096-byte audio block\n", interval_us);
  printk("  Execution Mode      : Single-threaded synthetic pacing; synchronous storage writes; no producer/consumer queue\n");
  printk("  Storage Path        : %s\n",
         use_psram ? "PSRAM circular staging, drained synchronously at 256 KiB thresholds"
                   : "Direct SRAM block to synchronous SD write");
  printk("  Coverage            : Synthetic PCM16 storage workload only; ADC/DMA and DSP are not active.\n");
  printk("--------------------------------------------------------------------------------\n");

  const char *pipe_path = "/SD:/PIPETEST.DAT";
  struct fs_file_t file;
  fs_file_t_init(&file);

  int ret = fs_open(&file, pipe_path, FS_O_CREATE | FS_O_READ | FS_O_WRITE | FS_O_TRUNC);
  if (ret != 0) {
    printk("[ERROR] Failed to open %s: %d\n", pipe_path, ret);
    benchmark_set_running(false);
    return ret;
  }

  void *synth_blk = NULL;
  ret = recorder_borrow_pool_blocks(&synth_blk, 1);
  if (ret != 0 || !synth_blk) {
    printk("[ERROR] Failed to borrow block for pipeline generator: %d\n", ret);
    fs_close(&file);
    fs_unlink(pipe_path);
    benchmark_set_running(false);
    return ret;
  }

  if (use_psram) {
    psram_staging_init(1024 * 1024, 256 * 1024);
  }
  /* Reuse the borrowed 4 KiB block for PSRAM drains. A local 4 KiB array
   * overflows Zephyr's 4 KiB main-thread stack once filesystem call frames
   * are included. The source block is already copied into PSRAM before drain. */
  uint8_t *batch_chunk = (uint8_t *)synth_blk;

  uint32_t blocks_produced = 0;
  uint32_t blocks_written = 0;
  uint32_t blocks_dropped = 0;
  uint32_t write_errors = 0;
  int storage_error = 0;
  uint32_t psram_peak = 0;
  uint64_t total_write_lat_us = 0;
  uint32_t max_write_lat_us = 0;
  uint32_t late_blocks = 0;
  uint32_t max_lateness_us = 0;

  int64_t session_start_us = k_ticks_to_us_floor64(k_uptime_ticks());

  for (uint32_t b = 0; b < total_blocks; b++) {
    int64_t block_deadline_us = session_start_us +
        (int64_t)(((uint64_t)(b + 1U) * block_samples * 1000000ULL) / target_rate);

    /* Generate deterministic ramp pattern in synthetic block */
    uint16_t *samples = (uint16_t *)synth_blk;
    for (uint32_t s = 0; s < block_samples; s++) {
      samples[s] = (uint16_t)((b * block_samples + s) & 0xFFFF);
    }
    blocks_produced++;

    if (use_psram) {
      /* Feed into PSRAM staging buffer */
      int pret = psram_staging_push(synth_blk, block_bytes, block_samples);
      if (pret != 0) {
        blocks_dropped++;
      }
      uint32_t occ = psram_staging_occupied();
      if (occ > psram_peak) psram_peak = occ;

      /* If occupancy reaches 256 KB batch, drain to SD */
      if (occ >= 256 * 1024) {
        uint32_t bytes_peeked = 0;
        uint32_t batch_drained = 0;

        uint64_t t0 = k_cycle_get_64();
        while (batch_drained < 256 * 1024) {
          psram_staging_peek(batch_chunk, block_bytes, &bytes_peeked);
          if (bytes_peeked == 0) break;
          ssize_t w = fs_write(&file, batch_chunk, bytes_peeked);
          if (w != bytes_peeked) {
            write_errors++;
            storage_error = (w < 0) ? (int)w : -EIO;
            printk("[PIPE SD WRITE ERROR] PSRAM batch returned %d/%u bytes; disk status=%d\n",
                   (int)w, bytes_peeked, disk_access_status("SD"));
            break;
          }
          psram_staging_commit(bytes_peeked);
          batch_drained += bytes_peeked;
        }
        uint64_t t1 = k_cycle_get_64();
        uint32_t lat_us = (uint32_t)k_cyc_to_us_floor64(t1 - t0);
        total_write_lat_us += lat_us;
        if (lat_us > max_write_lat_us) max_write_lat_us = lat_us;
        blocks_written += (batch_drained / block_bytes);
      }
    } else {
      /* Direct SRAM write to SD */
      uint64_t t0 = k_cycle_get_64();
      ssize_t w = fs_write(&file, synth_blk, block_bytes);
      uint64_t t1 = k_cycle_get_64();
      uint32_t lat_us = (uint32_t)k_cyc_to_us_floor64(t1 - t0);
      total_write_lat_us += lat_us;
      if (lat_us > max_write_lat_us) max_write_lat_us = lat_us;

      if (w == block_bytes) {
        blocks_written++;
      } else {
        write_errors++;
        storage_error = (w < 0) ? (int)w : -EIO;
        printk("[PIPE SD WRITE ERROR] block %u returned %d/%u bytes; disk status=%d\n",
               b, (int)w, block_bytes, disk_access_status("SD"));
      }
    }

    /* Never continue issuing filesystem calls after a lower disk error. */
    if (write_errors != 0) {
      goto pipeline_storage_abort;
    }

    /* Wait until next block interval */
    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    if (now_us < block_deadline_us) {
      k_busy_wait((uint32_t)(block_deadline_us - now_us));
    } else {
      uint32_t late_us = (uint32_t)(now_us - block_deadline_us);
      late_blocks++;
      if (late_us > max_lateness_us) max_lateness_us = late_us;
    }
  }

  /* Final flush if using PSRAM */
  if (use_psram) {
    uint32_t remaining = psram_staging_occupied();
    uint32_t bytes_peeked = 0;
    while (remaining > 0) {
      psram_staging_peek(batch_chunk, block_bytes, &bytes_peeked);
      if (bytes_peeked == 0) break;
      ssize_t w = fs_write(&file, batch_chunk, bytes_peeked);
      if (w != bytes_peeked) {
        write_errors++;
        storage_error = (w < 0) ? (int)w : -EIO;
        printk("[PIPE SD WRITE ERROR] final PSRAM batch returned %d/%u bytes; disk status=%d\n",
               (int)w, bytes_peeked, disk_access_status("SD"));
        goto pipeline_storage_abort;
      }
      psram_staging_commit(bytes_peeked);
      blocks_written += (bytes_peeked / block_bytes);
      remaining = psram_staging_occupied();
    }
  }

  int64_t session_end_us = k_ticks_to_us_floor64(k_uptime_ticks());
  uint32_t elapsed_session_ms = (uint32_t)((session_end_us - session_start_us) / 1000);

  ret = fs_sync(&file);
  if (ret != 0) {
    write_errors++;
    storage_error = ret;
    printk("[PIPE SD SYNC ERROR] fs_sync=%d disk status=%d\n", ret, disk_access_status("SD"));
    goto pipeline_storage_abort;
  }

  /* Verify file size and pattern on disk */
  bool data_verified = true;
  struct fs_dirent ent;
  ret = fs_stat(pipe_path, &ent);
  if (ret != 0 || (uint64_t)ent.size != target_bytes) {
    data_verified = false;
  }

  /* Verify every stored sample using the deterministic stream pattern. */
  ret = fs_seek(&file, 0, FS_SEEK_SET);
  if (ret != 0) data_verified = false;
  for (uint32_t b = 0; data_verified && b < total_blocks; b++) {
    ssize_t vr = fs_read(&file, synth_blk, block_bytes);
    if (vr != (ssize_t)block_bytes) {
      data_verified = false;
      break;
    }
    const uint16_t *samples = (const uint16_t *)synth_blk;
    for (uint32_t s = 0; s < block_samples; s++) {
      uint16_t expected = (uint16_t)((b * block_samples + s) & 0xFFFFU);
      if (samples[s] != expected) {
        printk("  [PIPE DATA MISMATCH] block=%u sample=%u expected=0x%04X actual=0x%04X\n",
               b, s, expected, samples[s]);
        data_verified = false;
        break;
      }
    }
  }

  ret = fs_close(&file);
  if (ret != 0) write_errors++;
  ret = fs_unlink(pipe_path);
  if (ret != 0) write_errors++;
  recorder_return_pool_blocks(&synth_blk, 1);

  uint32_t avg_lat_us = blocks_written ? (uint32_t)(total_write_lat_us / blocks_written) : 0;
  uint64_t written_bytes = (uint64_t)blocks_written * block_bytes;
  uint32_t eff_kb_s = elapsed_session_ms ? (uint32_t)((written_bytes * 1000ULL) / ((uint64_t)elapsed_session_ms * 1024ULL)) : 0;
  uint32_t target_kb_s = byte_rate / 1024;
  bool integrity_ok = (write_errors == 0) && data_verified;
  bool pacing_ok = (blocks_dropped == 0) && (late_blocks == 0);
  bool throughput_ok = (eff_kb_s >= target_kb_s);
  bool overall_passed = integrity_ok && pacing_ok && throughput_ok;

  printk("  Elapsed Session     : %u ms (Wall Clock)\n", elapsed_session_ms);
  printk("  Blocks Produced     : %u blocks (%llu bytes)\n", blocks_produced, (uint64_t)blocks_produced * block_bytes);
  printk("  Blocks Committed    : %u blocks (%llu bytes)\n", blocks_written, written_bytes);
  printk("  Blocks Dropped      : %u blocks %s\n", blocks_dropped, (blocks_dropped == 0) ? "[PASS: ZERO LOSS]" : "[FAIL: DROPS DETECTED]");
  if (use_psram) {
    printk("  PSRAM Peak Occupancy: %u bytes (%.1f%% of 1024 KB staging buffer)\n",
           psram_peak, (double)psram_peak * 100.0 / (1024.0 * 1024.0));
  }
  printk("  SD Write Latency    : Avg %u us, Max %u us\n", avg_lat_us, max_write_lat_us);
  printk("  Pacing Lateness     : %u late blocks, maximum %u us %s\n",
         late_blocks, max_lateness_us,
         pacing_ok ? "[PASS: ZERO OVERRUNS]" : "[FAIL: REAL-TIME DEADLINES MISSED]");
  printk("  Effective Throughput: %u KB/s (Target was %u KB/s) %s\n",
         eff_kb_s, target_kb_s,
         throughput_ok ? "[PASS: TARGET MET]" : "[FAIL: THROUGHPUT DEFICIT]");
  printk("  Data Integrity      : %s\n",
         integrity_ok ? "PASS (100% Deterministic Match)" : "FAIL (Size/Pattern Mismatch)");
  printk("  Pacing Result       : %s\n",
         pacing_ok ? "PASS (All blocks met real-time intervals)" : "FAIL (Blocks exceeded deadline interval)");
  printk("  Throughput Result   : %s\n",
         throughput_ok ? "PASS (Measured rate meets target)" : "FAIL (Effective rate below target)");
  printk("  Simulation Verdict  : %s\n",
         overall_passed ? "PASS (Real-time timing, throughput, and data verified)"
                        : (integrity_ok ? "FAIL - TIMING/PACING FAILURE (Data intact but real-time deadlines overrun)"
                                        : "FAIL - DATA LOSS OR CORRUPTION"));
  printk("  Notice              : Single-threaded synthetic pacing does NOT establish live concurrent queue reliability.\n");
  printk("================================================================================\n\n");

  s_bench_summary.pipeline_tested = true;
  s_bench_summary.pipe_last.sample_rate = target_rate;
  s_bench_summary.pipe_last.byte_rate = byte_rate;
  s_bench_summary.pipe_last.duration_sec = duration_sec;
  s_bench_summary.pipe_last.use_psram = use_psram;
  s_bench_summary.pipe_last.bytes_produced = (uint64_t)blocks_produced * block_bytes;
  s_bench_summary.pipe_last.bytes_written = written_bytes;
  s_bench_summary.pipe_last.blocks_produced = blocks_produced;
  s_bench_summary.pipe_last.blocks_written = blocks_written;
  s_bench_summary.pipe_last.blocks_dropped = blocks_dropped;
  s_bench_summary.pipe_last.psram_peak_bytes = psram_peak;
  s_bench_summary.pipe_last.avg_write_lat_us = avg_lat_us;
  s_bench_summary.pipe_last.max_write_lat_us = max_write_lat_us;
  s_bench_summary.pipe_last.effective_kb_s = eff_kb_s;
  s_bench_summary.pipe_last.integrity_ok = integrity_ok;
  s_bench_summary.pipe_last.passed = overall_passed;

  benchmark_set_running(false);
  return overall_passed ? 0 : (integrity_ok ? -ETIMEDOUT : -EIO);

pipeline_storage_abort:
  /* fs_unmount invalidates the open FATFS object before disk reinitialization. */
  recorder_return_pool_blocks(&synth_blk, 1);
  printk("[PIPE SD ABORT] first storage failure=%d; skipping stat/read/close/unlink on stale volume.\n",
         storage_error);
  printk("[PIPE SD RECOVERY] unmounting, reinitializing SDMMC, and remounting...\n");
  ret = recorder_storage_recover();
  printk("[PIPE SD RECOVERY] result=%d disk status=%d; PIPETEST.DAT retained if present.\n",
         ret, disk_access_status("SD"));
  benchmark_set_running(false);
  return storage_error ? storage_error : -EIO;
}

int bench_run_pipeline_all(void) {
  static const uint32_t rates[] = {48000, 96000, 192000, 256000, 384000, 512000};
  printk("\n================================================================================\n");
  printk("       RUNNING AUTOMATED SYNTHETIC STORAGE-DRAIN RATE SIMULATIONS              \n");
  printk("================================================================================\n");

  bool psram_ok = psram_is_present() && psram_is_integrity_verified();
  int failed_count = 0;

  for (int r = 0; r < ARRAY_SIZE(rates); r++) {
    uint32_t rate = rates[r];
    /* Test RAM path first */
    printk("\n>>> [TARGET %u Hz] Direct Internal SRAM Queue...\n", rate);
    int res = bench_run_pipeline(rate, false);
    if (res != 0) failed_count++;

    if (psram_ok) {
      printk("\n>>> [TARGET %u Hz] External PSRAM Staging Buffer...\n", rate);
      res = bench_run_pipeline(rate, true);
      if (res != 0) failed_count++;
    }
  }

  printk("\n================================================================================\n");
  printk("Multi-Rate Pipeline Suite Completed: %u rate tests failed.\n", failed_count);
  printk("================================================================================\n\n");
  return failed_count;
}

void bench_print_status(void) {
  printk("\n========================= BENCHMARK SYSTEM STATUS =========================\n");
  printk("  Hardware Platform   : STM32L476RG @ %u MHz HCLK, Native SDMMC 4-bit, IS66WVS4M8 QSPI\n",
         HAL_RCC_GetHCLKFreq() / 1000000U);
  printk("  PSRAM Detected      : %s\n", psram_is_present() ? "YES (JEDEC 0x9D/0x5D/0x4D)" : "NO");
  printk("  PSRAM Integrity     : %s\n", psram_is_integrity_verified() ? "VERIFIED (Passed Multi-Pass Test)" : "UNVERIFIED / PENDING 'psram test'");
  printk("  PSRAM Interface     : %s (Burst %u B)\n",
         psram_is_quad_mode() ? "Quad I/O (1-4-4)" : "Standard 1-Line SPI (1-1-1)",
         psram_get_burst_size());
  printk("---------------------------------------------------------------------------\n");
  if (s_bench_summary.ram_tested) {
    printk("  Internal SRAM       : %5u B: %s [%s]\n",
           s_bench_summary.ram_last.chunk_size,
           s_bench_summary.ram_last.integrity_ok ? "rates available in run log" : "rates invalid",
           s_bench_summary.ram_last.integrity_ok ? "PASS" : "FAIL");
  } else {
    printk("  Internal SRAM       : Not yet benchmarked (type 'bench ram')\n");
  }

  if (s_bench_summary.psram_tested) {
    printk("  External PSRAM      : %5u B: %s [%s]\n",
           s_bench_summary.psram_last.chunk_size,
           s_bench_summary.psram_last.integrity_ok ? "rates available in run log" : "rates invalid",
           s_bench_summary.psram_last.integrity_ok ? "PASS" : "FAIL");
  } else {
    printk("  External PSRAM      : Not yet benchmarked (type 'bench psram')\n");
  }

  if (s_bench_summary.sd_tested) {
    printk("  SDMMC MicroSD       : %5u B block: %s (Avg write latency: %u us) [%s]\n",
           s_bench_summary.sd_last.block_size,
           s_bench_summary.sd_last.integrity_ok ? "rates available in run log" : "rates invalid",
           s_bench_summary.sd_last.lat_avg_us,
           s_bench_summary.sd_last.integrity_ok ? "PASS" : "FAIL");
  } else {
    printk("  SDMMC MicroSD       : Not yet benchmarked (type 'bench sd')\n");
  }

  if (s_bench_summary.pipeline_tested) {
    printk("  Audio Pipeline      : %u Hz (%s): %s (Drops: %u, Effective: %u KB/s)\n",
           s_bench_summary.pipe_last.sample_rate,
           s_bench_summary.pipe_last.use_psram ? "PSRAM" : "RAM",
           s_bench_summary.pipe_last.passed ? "PASS" : "FAIL",
           s_bench_summary.pipe_last.blocks_dropped,
           s_bench_summary.pipe_last.effective_kb_s);
  } else {
    printk("  Audio Pipeline      : Not yet benchmarked (type 'bench pipeline [rate]')\n");
  }
  printk("===========================================================================\n\n");
}
