/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Recorder core: audio backend -> block pool -> queue -> SD writer.
 *
 *   audio_in backend (IRQ/thread)           writer (main thread)
 *   ----------------------------           --------------------
 *   alloc():  k_mem_slab_alloc(NO_WAIT)     k_msgq_get(filled)
 *   fill block                              fs_write(block)   <-- may stall
 * 100+ ms submit(): k_msgq_put(NO_WAIT)     k_mem_slab_free(block)
 *
 * If the writer stalls, blocks pile up in the pool; if the pool runs dry,
 * alloc() returns NULL and the loss is counted in blocks_dropped.
 * Supports configurable 16-bit or 32-bit signed integer PCM.
 */
#include "recorder.h"
#include "audio_in.h"
#include "dsp.h"
#include "psram_is66wvs.h"
#include "psram_staging.h"
#include "rtc_calendar.h"
#include "rtc_sleep.h"
#include "wav.h"
#include "console.h"
#include "scheduler.h"
#include "audio_format.h"
#include "main.h"
#include <ctype.h>
#include <ff.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/time_units.h>
#include "led_indicator.h"
#include "battery_monitor.h"

#if defined(CONFIG_SOC_SERIES_STM32L4X)
#include <stm32_ll_dma.h>
#include <stm32_ll_bus.h>
#endif

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(recorder, LOG_LEVEL_INF);

static void log_sdmmc_stage_diagnostics(const char *stage_name, int err_code,
                                        uint64_t file_offset, uint32_t byte_count) {
  printk("\n[STAGE_DIAG] Stage: %s | Error: %d | Offset: %llu B | Count: %u B\n",
         stage_name, err_code, (unsigned long long)file_offset, byte_count);
#if defined(CONFIG_SOC_SERIES_STM32L4X)
  uint32_t sta = SDMMC1->STA;
  uint32_t dcount = SDMMC1->DCOUNT;
  uint32_t dctrl = SDMMC1->DCTRL;
  uint32_t clkcr = SDMMC1->CLKCR;
  uint32_t clk_div = clkcr & SDMMC_CLKCR_CLKDIV_Msk;
  uint32_t bus_w = ((clkcr & SDMMC_CLKCR_WIDBUS_Msk) == SDMMC_CLKCR_WIDBUS_0) ? 4 :
                   ((clkcr & SDMMC_CLKCR_WIDBUS_Msk) == SDMMC_CLKCR_WIDBUS_1) ? 8 : 1;
  uint32_t approx_sector = (uint32_t)(file_offset / 512);
  uint32_t sector_count = (byte_count + 511) / 512;
  bool dma_en = (LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_DMA2) &&
                 LL_DMA_IsEnabledChannel(DMA2, LL_DMA_CHANNEL_4));
  uint32_t dma_len = dma_en ? LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_4) : 0;

  const char *err_str = "NONE";
  if (sta & SDMMC_STA_DCRCFAIL) err_str = "DATA_CRC_FAIL";
  else if (sta & SDMMC_STA_CCRCFAIL) err_str = "CMD_CRC_FAIL";
  else if (sta & SDMMC_STA_DTIMEOUT) err_str = "DATA_TIMEOUT";
  else if (sta & SDMMC_STA_CTIMEOUT) err_str = "CMD_TIMEOUT";
  else if (sta & SDMMC_STA_TXUNDERR) err_str = "TX_UNDERRUN";
  else if (sta & SDMMC_STA_RXOVERR) err_str = "RX_OVERRUN";

  printk("  [SDMMC_HW] Transfer: %s | STA=0x%08X (%s) | DCOUNT=%u | DCTRL=0x%08X\n",
         (file_offset >= WAV_HEADER_BYTES) ? "DATA_TRANSFER" : "FINALIZATION",
         sta, err_str, dcount, dctrl);
  printk("  [SDMMC_BUS] CLKCR=0x%08X (clk_div=%u, bus=%u-bit) | DMA2_Ch4_En=%d (len=%u) | Sector=%u (count=%u)\n",
         clkcr, clk_div, bus_w, dma_en ? 1 : 0, dma_len, approx_sector, sector_count);
#endif
  while (log_process()) {}
}
#else
LOG_MODULE_REGISTER(recorder, LOG_LEVEL_WRN);
#endif

static void recorder_apply_sdmmc_clock(void) {
#if defined(SDMMC1)
#if defined(CONFIG_WILDSONIC_SDMMC_CLK_DIV)
  uint32_t target_div = CONFIG_WILDSONIC_SDMMC_CLK_DIV;
  if ((SDMMC1->CLKCR & 0xFF) != target_div) {
    SDMMC1->CLKCR = (SDMMC1->CLKCR & ~0xFFU) | (target_div & 0xFFU);
  }
#endif
  uint32_t clkcr = SDMMC1->CLKCR;
  uint32_t clkdiv = clkcr & 0xFF;
  uint32_t clk_hz = 48000000U / (clkdiv + 2U);
  LOG_INF("SDMMC hardware clock: %u Hz (CLKDIV=%u, CLKCR=0x%08X, bus=%s)",
          clk_hz, clkdiv, clkcr, (clkcr & (1U << 11)) ? "4-bit" : "1-bit");
#endif
}

#define SAMPLE_BYTES (CONFIG_AUDIO_SAMPLE_BITS / 8)
#define BLOCK_SAMPLES CONFIG_AUDIO_BLOCK_SAMPLES
#define BLOCK_BYTES (BLOCK_SAMPLES * SAMPLE_BYTES)
#define NUM_BLOCKS CONFIG_AUDIO_NUM_BLOCKS
#define SLOW_WRITE_US 50000U /* log any single write slower than this (50 ms) */

/*
 * Hardware Memory Mapping Note (STM32L476 vs Upstream Zephyr DTS):
 *
 * In ST RM0351 documentation:
 *   - SRAM1: 96 KiB base at 0x20000000 (Main system memory)
 *   - SRAM2: 32 KiB base at 0x10000000 (Zero wait-state retention memory)
 *
 * In Upstream Zephyr Devicetree (dts/arm/st/l4/stm32l4.dtsi & stm32l471.dtsi):
 *   - sram0: memory@20000000 -> zephyr,memory-region = "SRAM0" (maps to linker region RAM / SRAM0)
 *   - sram1: memory@10000000 -> zephyr,memory-region = "SRAM1" (maps to linker region SRAM1)
 *
 * Zephyr numbers DTS nodes 0-indexed. Hence, Zephyr generates the output section
 * name "SRAM1" for the physical 32 KiB SRAM2 at 0x10000000.
 * We define STM32_SRAM2_RETENTION_SECTION to map explicitly to Zephyr's SRAM1 section.
 */
#if defined(__ZEPHYR__)
#define STM32_SRAM2_RETENTION_SECTION __attribute__((section("SRAM1")))
static char __aligned(4) STM32_SRAM2_RETENTION_SECTION
    _k_mem_slab_buf_pool[(NUM_BLOCKS) * ROUND_UP(BLOCK_BYTES, 4)];
static STRUCT_SECTION_ITERABLE(k_mem_slab, pool) =
    Z_MEM_SLAB_INITIALIZER(pool, _k_mem_slab_buf_pool, BLOCK_BYTES, NUM_BLOCKS);
#else
K_MEM_SLAB_DEFINE_STATIC(pool, BLOCK_BYTES, NUM_BLOCKS, 4);
#endif
K_MSGQ_DEFINE(filled_q, sizeof(uint8_t), NUM_BLOCKS, 1);

/* Asynchronous non-PSRAM block message queue to sd_writer_thread */
struct sd_block_msg {
  uint8_t buf_id;
  void *blk;
  uint32_t bytes;
  uint32_t samples;
};
K_MSGQ_DEFINE(sd_block_q, sizeof(struct sd_block_msg), NUM_BLOCKS, 4);

/* Buffer ownership descriptor table */
struct buf_descriptor {
  uint8_t id;
  enum buf_owner_state state;
  void *ptr;
};

static struct buf_descriptor s_buf_table[NUM_BLOCKS];
static struct k_spinlock s_buf_lock;

static inline uint8_t recorder_buf_ptr_to_id(const void *blk) {
  if (blk < (const void *)_k_mem_slab_buf_pool ||
      blk >= (const void *)(_k_mem_slab_buf_pool + (NUM_BLOCKS * ROUND_UP(BLOCK_BYTES, 4)))) {
    return 0xFF;
  }
  return (uint8_t)(((uintptr_t)blk - (uintptr_t)_k_mem_slab_buf_pool) / ROUND_UP(BLOCK_BYTES, 4));
}

static inline void *recorder_buf_id_to_ptr(uint8_t id) {
  if (id < NUM_BLOCKS) {
    return (void *)(_k_mem_slab_buf_pool + (id * ROUND_UP(BLOCK_BYTES, 4)));
  }
  return NULL;
}

static void recorder_buf_table_init(void) {
  k_spinlock_key_t key = k_spin_lock(&s_buf_lock);
  for (uint8_t i = 0; i < NUM_BLOCKS; i++) {
    s_buf_table[i].id = i;
    s_buf_table[i].state = BUF_OWNER_FREE;
    s_buf_table[i].ptr = recorder_buf_id_to_ptr(i);
  }
  k_spin_unlock(&s_buf_lock, key);
}

static inline void recorder_buf_set_state(uint8_t id, enum buf_owner_state state) {
  if (id < NUM_BLOCKS) {
    k_spinlock_key_t key = k_spin_lock(&s_buf_lock);
    s_buf_table[id].state = state;
    k_spin_unlock(&s_buf_lock, key);
  }
}

static inline enum buf_owner_state recorder_buf_get_state(uint8_t id) {
  if (id < NUM_BLOCKS) {
    k_spinlock_key_t key = k_spin_lock(&s_buf_lock);
    enum buf_owner_state st = s_buf_table[id].state;
    k_spin_unlock(&s_buf_lock, key);
    return st;
  }
  return BUF_OWNER_FREE;
}

#define SD_WRITER_STACK_SIZE 4096
/* SD writer runs at preemptive priority 3 (below real-time capture priority 0 and
 * recorder/DSP/PSRAM consumer priority 1, ahead of background tasks) */
#define SD_WRITER_PRIORITY 3
#define PSRAM_READER_PRIORITY 3

K_THREAD_STACK_DEFINE(sd_writer_stack, SD_WRITER_STACK_SIZE);
static struct k_thread sd_writer_thread_data;
static k_tid_t sd_writer_tid;

#define PSRAM_READER_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(psram_reader_stack, PSRAM_READER_STACK_SIZE);
static struct k_thread psram_reader_thread_data;
static k_tid_t psram_reader_tid;

/* Asynchronous Ping-Pong Buffer Architecture (Buffer A + Buffer B) */
enum pingpong_buf_state {
  PINGPONG_BUF_FREE = 0,
  PINGPONG_BUF_READING_PSRAM,
  PINGPONG_BUF_READY_FOR_SD,
  PINGPONG_BUF_WRITING_SD,
  PINGPONG_BUF_COMMITTED,
};

struct pingpong_slot {
  uint8_t *buf;
  uint32_t bytes;
  enum pingpong_buf_state state;
  int err;
  uint64_t peek_start_cyc;
  uint64_t peek_end_cyc;
  uint64_t write_start_cyc;
  uint64_t write_end_cyc;
};

#define SD_WRITER_CHUNK_BYTES (32768U)
#define PINGPONG_SLOT_BYTES (SD_WRITER_CHUNK_BYTES / 2U) /* 16384 bytes */

uint8_t __aligned(4) s_sd_writer_buf[SD_WRITER_CHUNK_BYTES];
/* Static buffer alias for test compatibility: s_pingpong_buf_b[BLOCK_BYTES]; */
static uint8_t *s_pingpong_buf_b = &s_sd_writer_buf[PINGPONG_SLOT_BYTES];
static struct pingpong_slot s_pingpong[2];

static int sd_write_all(struct fs_file_t *fp, const void *data, size_t length) {
  if (!fp || !data) {
    return -EINVAL;
  }
  const uint8_t *ptr = (const uint8_t *)data;
  size_t remaining = length;
  while (remaining > 0) {
    ssize_t w = fs_write(fp, ptr, remaining);
    if (w < 0) {
      return (int)w;
    }
    if (w == 0) {
      return -ENOSPC;
    }
    ptr += (size_t)w;
    remaining -= (size_t)w;
  }
  return 0;
}

static struct k_sem s_reader_start_sem;
static struct k_sem s_reader_done_sem;
static struct k_sem s_slot_ready_sem;
static struct k_sem s_slot_free_sem;
static volatile uint32_t s_reader_to_read = 0;

static volatile uint64_t sd_writer_overlap_us_sum = 0;
static volatile uint32_t sd_writer_swap_count = 0;

static struct k_sem sd_writer_sem;
static struct k_sem sd_writer_done_sem;
static volatile bool sd_writer_exit_req = false;
static volatile bool sd_writer_flush_req = false;
static struct fs_file_t *sd_file_ptr = NULL;
static void *sd_bounce_buf = NULL;
static struct rec_stats *s_active_stats_ptr = NULL;

static volatile uint64_t sd_writer_write_us_sum = 0;
static volatile uint32_t sd_writer_write_count = 0;
static volatile uint32_t sd_writer_worst_write_us = 0;
static volatile uint64_t sd_writer_peek_us_sum = 0;
static volatile uint32_t sd_writer_peek_count = 0;
static volatile uint32_t sd_writer_worst_peek_us = 0;
static volatile uint64_t sd_writer_commit_us_sum = 0;
static volatile uint64_t sd_writer_batch_wall_us_sum = 0;
static volatile uint32_t sd_writer_worst_batch_ms = 0;
static volatile uint64_t sd_writer_bytes_written = 0;
static volatile uint64_t sd_writer_failed_bytes = 0;
static volatile uint32_t sd_writer_batch_count = 0;
static volatile int sd_writer_err = 0;

/* Spinlock-protected 64-bit and 32-bit SD writer performance metrics */
struct rec_writer_shared_stats {
  struct k_spinlock lock;
  uint64_t overlap_us_sum;
  uint32_t swap_count;
  uint64_t write_us_sum;
  uint32_t write_count;
  uint32_t worst_write_us;
  uint64_t peek_us_sum;
  uint32_t peek_count;
  uint32_t worst_peek_us;
  uint64_t commit_us_sum;
  uint64_t batch_wall_us_sum;
  uint32_t worst_batch_ms;
  uint64_t bytes_written;
  uint64_t failed_bytes;
  uint32_t batch_count;
  int err;
};
static struct rec_writer_shared_stats s_writer_stats;

static inline void writer_stats_reset(void) {
  k_spinlock_key_t key = k_spin_lock(&s_writer_stats.lock);
  memset(&s_writer_stats, 0, sizeof(s_writer_stats));
  sd_writer_overlap_us_sum = 0;
  sd_writer_swap_count = 0;
  sd_writer_write_us_sum = 0;
  sd_writer_write_count = 0;
  sd_writer_worst_write_us = 0;
  sd_writer_peek_us_sum = 0;
  sd_writer_peek_count = 0;
  sd_writer_worst_peek_us = 0;
  sd_writer_commit_us_sum = 0;
  sd_writer_batch_wall_us_sum = 0;
  sd_writer_worst_batch_ms = 0;
  sd_writer_bytes_written = 0;
  sd_writer_failed_bytes = 0;
  sd_writer_batch_count = 0;
  sd_writer_err = 0;
  k_spin_unlock(&s_writer_stats.lock, key);
}

static inline void writer_stats_record_write(uint32_t us, uint32_t bytes) {
  k_spinlock_key_t key = k_spin_lock(&s_writer_stats.lock);
  s_writer_stats.write_us_sum += us;
  s_writer_stats.write_count++;
  if (us > s_writer_stats.worst_write_us) {
    s_writer_stats.worst_write_us = us;
  }
  s_writer_stats.bytes_written += bytes;
  sd_writer_write_us_sum = s_writer_stats.write_us_sum;
  sd_writer_write_count = s_writer_stats.write_count;
  sd_writer_worst_write_us = s_writer_stats.worst_write_us;
  sd_writer_bytes_written = s_writer_stats.bytes_written;
  k_spin_unlock(&s_writer_stats.lock, key);
}

static inline void writer_stats_record_peek(uint32_t us) {
  k_spinlock_key_t key = k_spin_lock(&s_writer_stats.lock);
  s_writer_stats.peek_us_sum += us;
  s_writer_stats.peek_count++;
  if (us > s_writer_stats.worst_peek_us) {
    s_writer_stats.worst_peek_us = us;
  }
  sd_writer_peek_us_sum = s_writer_stats.peek_us_sum;
  sd_writer_peek_count = s_writer_stats.peek_count;
  sd_writer_worst_peek_us = s_writer_stats.worst_peek_us;
  k_spin_unlock(&s_writer_stats.lock, key);
}

static inline void writer_stats_record_commit(uint32_t us) {
  k_spinlock_key_t key = k_spin_lock(&s_writer_stats.lock);
  s_writer_stats.commit_us_sum += us;
  sd_writer_commit_us_sum = s_writer_stats.commit_us_sum;
  k_spin_unlock(&s_writer_stats.lock, key);
}

static inline void writer_stats_record_batch(uint64_t wall_us, uint32_t batch_ms) {
  k_spinlock_key_t key = k_spin_lock(&s_writer_stats.lock);
  s_writer_stats.batch_wall_us_sum += wall_us;
  s_writer_stats.batch_count++;
  if (batch_ms > s_writer_stats.worst_batch_ms) {
    s_writer_stats.worst_batch_ms = batch_ms;
  }
  sd_writer_batch_wall_us_sum = s_writer_stats.batch_wall_us_sum;
  sd_writer_batch_count = s_writer_stats.batch_count;
  sd_writer_worst_batch_ms = s_writer_stats.worst_batch_ms;
  k_spin_unlock(&s_writer_stats.lock, key);
}

static inline void writer_stats_record_swap(uint64_t overlap_us) {
  k_spinlock_key_t key = k_spin_lock(&s_writer_stats.lock);
  s_writer_stats.overlap_us_sum += overlap_us;
  s_writer_stats.swap_count++;
  sd_writer_overlap_us_sum = s_writer_stats.overlap_us_sum;
  sd_writer_swap_count = s_writer_stats.swap_count;
  k_spin_unlock(&s_writer_stats.lock, key);
}

static inline void writer_stats_set_err(int err) {
  k_spinlock_key_t key = k_spin_lock(&s_writer_stats.lock);
  s_writer_stats.err = err;
  sd_writer_err = err;
  k_spin_unlock(&s_writer_stats.lock, key);
}

static inline int writer_stats_get_err(void) {
  k_spinlock_key_t key = k_spin_lock(&s_writer_stats.lock);
  int e = s_writer_stats.err;
  k_spin_unlock(&s_writer_stats.lock, key);
  return e;
}
static volatile int s_storage_recovery_result = 0;
static volatile bool s_storage_recovering = false;
enum recorder_storage_state {
  STORAGE_STATE_HEALTHY = 0,
  STORAGE_STATE_FAULT,
};
static volatile enum recorder_storage_state s_storage_state = STORAGE_STATE_HEALTHY;

#define RECORDER_MAX_CONSECUTIVE_DROPS 10U
#define MAX_SESSION_GAPS 32U

struct rec_gap_entry {
  uint64_t sample_index;
  uint32_t sample_count;
};

static struct rec_gap_entry s_session_gaps[MAX_SESSION_GAPS];
static uint32_t s_session_gap_count = 0;
static volatile bool s_backpressure_slowdown = false;

static void recorder_record_gap(uint64_t sample_idx, uint32_t count) {
  if (s_session_gap_count < MAX_SESSION_GAPS) {
    s_session_gaps[s_session_gap_count].sample_index = sample_idx;
    s_session_gaps[s_session_gap_count].sample_count = count;
    s_session_gap_count++;
  }
  LOG_WRN("[RECORDER GAP] Dropped %u samples at index %llu (gap #%u)",
          count, (unsigned long long)sample_idx, s_session_gap_count);
  printk("[RECORDER GAP] index=%llu count=%u\n",
         (unsigned long long)sample_idx, count);
}

static void recorder_write_gap_sidecar(const char *wav_path) {
  if (s_session_gap_count == 0 || wav_path == NULL || wav_path[0] == '\0') {
    return;
  }
  char sidecar_path[96];
  size_t len = strlen(wav_path);
  if (len >= sizeof(sidecar_path) - 6) {
    return;
  }
  memcpy(sidecar_path, wav_path, len);
  sidecar_path[len] = '\0';
  char *dot = strrchr(sidecar_path, '.');
  if (dot) {
    strcpy(dot, ".gaps");
  } else {
    strcat(sidecar_path, ".gaps");
  }

  struct fs_file_t gf;
  fs_file_t_init(&gf);
  if (fs_open(&gf, sidecar_path, FS_O_CREATE | FS_O_WRITE) == 0) {
    char hdr[64];
    int hlen = snprintf(hdr, sizeof(hdr), "SESSION_GAPS: %u\n", s_session_gap_count);
    if (hlen > 0) {
      fs_write(&gf, hdr, hlen);
    }
    for (uint32_t i = 0; i < s_session_gap_count; i++) {
      char line[64];
      int l_len = snprintf(line, sizeof(line), "GAP %u: sample_index=%llu count=%u\n",
                           i + 1U,
                           (unsigned long long)s_session_gaps[i].sample_index,
                           s_session_gaps[i].sample_count);
      if (l_len > 0) {
        fs_write(&gf, line, l_len);
      }
    }
    fs_sync(&gf);
    fs_close(&gf);
    LOG_INF("Wrote %u gap markers to sidecar log: %s", s_session_gap_count, sidecar_path);
  }
}

/* Bound final drain using observed fs_write throughput when it is slower than
 * the 4 KiB/s fallback, with five seconds reserved for filesystem metadata. */
static uint32_t writer_completion_timeout_ms(uint32_t pending_bytes) {
  uint64_t service_bytes_per_sec = 4096ULL;
  if (sd_writer_write_us_sum > 0 && sd_writer_bytes_written > 0) {
    uint64_t observed = (sd_writer_bytes_written * 1000000ULL) /
                        sd_writer_write_us_sum;
    if (observed > 0 && observed < service_bytes_per_sec) {
      service_bytes_per_sec = observed;
    }
  }
  uint64_t timeout = 5000ULL +
                     ((uint64_t)pending_bytes * 1000ULL / service_bytes_per_sec);
  return (timeout > UINT32_MAX) ? UINT32_MAX : (uint32_t)timeout;
}

#ifdef CONFIG_AUDIO_PSRAM_STAGING
static bool g_psram_staging_enabled = true;
#else
static bool g_psram_staging_enabled = false;
#endif
static volatile bool active_use_psram = false;

#if defined(CONFIG_AUDIO_STORAGE_PIPELINE_PSRAM_ASYNC)
static enum storage_pipeline_mode g_storage_pipeline_mode = STORAGE_PIPELINE_PSRAM_ASYNC;
#else
static enum storage_pipeline_mode g_storage_pipeline_mode = STORAGE_PIPELINE_PSRAM_SYNC;
#endif

void recorder_set_pipeline_mode(enum storage_pipeline_mode mode) {
  g_storage_pipeline_mode = mode;
  LOG_INF("Storage pipeline mode set to: %s", recorder_get_pipeline_mode_str(mode));
}

enum storage_pipeline_mode recorder_get_pipeline_mode(void) {
  return g_storage_pipeline_mode;
}

const char *recorder_get_pipeline_mode_str(enum storage_pipeline_mode mode) {
  switch (mode) {
    case STORAGE_PIPELINE_DIRECT_SRAM:
      return "Direct SRAM (Mode A)";
    case STORAGE_PIPELINE_PSRAM_SYNC:
      return "PSRAM Synchronous (Mode B)";
    case STORAGE_PIPELINE_PSRAM_ASYNC:
      return "PSRAM Asynchronous Ping-Pong (Mode C)";
    default:
      return "Unknown";
  }
}

void recorder_set_psram_staging_enabled(bool enabled) {
  if (enabled) {
    if (!psram_is_present() || !psram_is_integrity_verified()) {
      LOG_WRN("Cannot enable PSRAM staging: memory integrity not verified");
      g_psram_staging_enabled = false;
      return;
    }
    if (!psram_is_quad_mode()) {
      LOG_WRN("Cannot enable PSRAM staging: Quad I/O not verified (single-line SPI fallback has insufficient bandwidth for real-time audio)");
      g_psram_staging_enabled = false;
      return;
    }
  }
  g_psram_staging_enabled = enabled;
}

bool recorder_is_psram_staging_enabled(void) { return g_psram_staging_enabled; }

static void psram_reader_thread_fn(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    k_sem_take(&s_reader_start_sem, K_FOREVER);
    if (sd_writer_exit_req) {
      break;
    }

    uint32_t r_idx = 0;
    while (s_reader_to_read > 0 && psram_staging_occupied() > 0 && sd_writer_err == 0) {
      if (k_sem_take(&s_slot_free_sem, K_MSEC(5000)) != 0) {
        if (sd_writer_err == 0) {
          LOG_ERR("PSRAM reader wait slot timeout");
          sd_writer_err = -ETIMEDOUT;
        }
        break;
      }
      if (sd_writer_err != 0 || sd_writer_exit_req) {
        break;
      }

      struct pingpong_slot *slot = &s_pingpong[r_idx];
      slot->state = PINGPONG_BUF_READING_PSRAM;
      uint32_t chunk = (s_reader_to_read > PINGPONG_SLOT_BYTES) ? PINGPONG_SLOT_BYTES : s_reader_to_read;
      uint32_t bytes_read = 0;

      slot->peek_start_cyc = k_cycle_get_64();
      int peek_ret = psram_staging_peek(slot->buf, chunk, &bytes_read);
      slot->peek_end_cyc = k_cycle_get_64();

      uint64_t p_diff = (slot->peek_end_cyc >= slot->peek_start_cyc)
                            ? (slot->peek_end_cyc - slot->peek_start_cyc)
                            : 0;
      uint32_t peek_us = (uint32_t)k_cyc_to_us_floor64(p_diff);
      sd_writer_peek_us_sum += peek_us;
      sd_writer_peek_count++;
      if (peek_us > sd_writer_worst_peek_us) {
        sd_writer_worst_peek_us = peek_us;
      }

      if (peek_ret != 0 || bytes_read == 0) {
        LOG_ERR("PSRAM reader peek failed (%d)", peek_ret);
        slot->err = peek_ret ? peek_ret : -EIO;
        slot->bytes = 0;
        slot->state = PINGPONG_BUF_READY_FOR_SD;
        if (sd_writer_err == 0) {
          sd_writer_err = slot->err;
        }
        k_sem_give(&s_slot_ready_sem);
        break;
      }

      slot->bytes = bytes_read;
      slot->err = 0;
      slot->state = PINGPONG_BUF_READY_FOR_SD;
      s_reader_to_read -= bytes_read;

      k_sem_give(&s_slot_ready_sem);
      r_idx = (r_idx + 1) % 2;
    }

    k_sem_give(&s_reader_done_sem);
  }
}

static void sd_writer_thread_fn(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    k_sem_take(&sd_writer_sem, K_FOREVER);
    if (sd_writer_exit_req) {
      break;
    }

    if (active_use_psram) {
      uint32_t batch_threshold = psram_staging_batch_size();

      while ((psram_staging_occupied() >= batch_threshold) ||
             (sd_writer_flush_req && psram_staging_occupied() > 0)) {
        if (sd_writer_err != 0 || !sd_file_ptr || !sd_bounce_buf) {
          if (sd_writer_err == 0) {
            sd_writer_err = -EBADF;
          }
          sd_writer_failed_bytes = psram_staging_occupied();
          break;
        }

        uint32_t to_write =
            sd_writer_flush_req ? psram_staging_occupied() : batch_threshold;
        if (!sd_writer_flush_req && to_write > batch_threshold) {
          to_write = batch_threshold;
        }

        uint32_t batch_bytes_written = 0;
        uint64_t t_batch_start = k_cycle_get_64();
        uint64_t batch_write_us_sum = 0;
        uint64_t batch_peek_us_sum = 0;
        sd_writer_batch_count++;
        uint32_t cur_batch = sd_writer_batch_count;

        if (!sd_writer_flush_req) {
          LOG_INF("Batch %u: draining %u KB batch to SD (PSRAM occupancy: %u "
                  "KB / %u KB)...",
                  cur_batch, to_write >> 10, psram_staging_occupied() >> 10,
                  psram_staging_capacity() >> 10);
        } else {
          LOG_INF(
              "Final flush: draining remaining %u bytes from PSRAM to SD...",
              to_write);
        }

        if (g_storage_pipeline_mode == STORAGE_PIPELINE_PSRAM_SYNC) {
          /* MODE B: Synchronous Sequential Peek -> Write -> Commit */
          while (to_write > 0 && sd_writer_err == 0) {
            uint32_t chunk = (to_write > SD_WRITER_CHUNK_BYTES) ? SD_WRITER_CHUNK_BYTES : to_write;
            uint32_t bytes_peeked = 0;

            uint64_t t_p0 = k_cycle_get_64();
            int peek_ret = psram_staging_peek(sd_bounce_buf, chunk, &bytes_peeked);
            uint64_t t_p1 = k_cycle_get_64();
            uint32_t peek_us = (uint32_t)k_cyc_to_us_floor64((t_p1 >= t_p0) ? (t_p1 - t_p0) : 0);
            sd_writer_peek_us_sum += peek_us;
            sd_writer_peek_count++;
            if (peek_us > sd_writer_worst_peek_us) {
              sd_writer_worst_peek_us = peek_us;
            }
            batch_peek_us_sum += peek_us;

            if (peek_ret != 0 || bytes_peeked == 0) {
              LOG_ERR("PSRAM sync peek failed (%d)", peek_ret);
              sd_writer_err = peek_ret ? peek_ret : -EIO;
              psram_staging_rewind_peek();
              break;
            }

            uint64_t t_w0 = k_cycle_get_64();
            /* fs_write(sd_file_ptr, sd_bounce_buf, bytes_read); */
            /* fs_write(sd_file_ptr, sd_bounce_buf, bytes_peeked); */
            int w_ret = sd_write_all(sd_file_ptr, sd_bounce_buf, bytes_peeked);
            uint64_t t_w1 = k_cycle_get_64();
            uint32_t us = (uint32_t)k_cyc_to_us_floor64((t_w1 >= t_w0) ? (t_w1 - t_w0) : 0);

            if (w_ret != 0) {
              uint64_t file_offset = sd_writer_bytes_written + WAV_HEADER_BYTES;
              uint32_t sector_approx = (uint32_t)(file_offset / 512);
              LOG_ERR("SD writer fs_write failed: ret=%d, expected=%u, file_offset=%llu, approx_sector=%u, write_latency=%u us",
                      w_ret, bytes_peeked, (unsigned long long)file_offset, sector_approx, us);
#if defined(CONFIG_SOC_SERIES_STM32L4X)
              LOG_ERR("SDMMC hardware registers: STA=0x%08X, DCOUNT=%u, DCTRL=0x%08X, CLKCR=0x%08X",
                      (unsigned int)SDMMC1->STA, (unsigned int)SDMMC1->DCOUNT,
                      (unsigned int)SDMMC1->DCTRL, (unsigned int)SDMMC1->CLKCR);
#endif
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
              log_sdmmc_stage_diagnostics("FILESYSTEM_WRITE_PSRAM", w_ret, file_offset, bytes_peeked);
#endif
              sd_writer_err = w_ret;
              if (s_active_stats_ptr && s_active_stats_ptr->first_error_stage[0] == '\0') {
#if defined(CONFIG_SOC_SERIES_STM32L4X)
                if (SDMMC1->STA & (SDMMC_STA_DCRCFAIL | SDMMC_STA_CCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_CTIMEOUT | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR)) {
                  strncpy(s_active_stats_ptr->first_error_stage, "SDMMC_TRANSFER", sizeof(s_active_stats_ptr->first_error_stage) - 1);
                } else
#endif
                {
                  strncpy(s_active_stats_ptr->first_error_stage, "FS_WRITE", sizeof(s_active_stats_ptr->first_error_stage) - 1);
                }
              }
              psram_staging_rewind_peek();
              break;
            }

            uint64_t t_c0 = k_cycle_get_64();
            int commit_ret = psram_staging_commit(bytes_peeked);
            uint64_t t_c1 = k_cycle_get_64();
            uint32_t commit_us = (uint32_t)k_cyc_to_us_floor64((t_c1 >= t_c0) ? (t_c1 - t_c0) : 0);
            sd_writer_commit_us_sum += commit_us;

            if (commit_ret != 0) {
              LOG_ERR("SD writer staging commit failed after successful write (%d)", commit_ret);
              sd_writer_err = commit_ret;
              break;
            }

            sd_writer_write_us_sum += us;
            batch_write_us_sum += us;
            sd_writer_write_count++;
            if (us > sd_writer_worst_write_us) {
              sd_writer_worst_write_us = us;
            }
            sd_writer_bytes_written += bytes_peeked;
            batch_bytes_written += bytes_peeked;
            to_write -= bytes_peeked;
          }
        } else {
          /* MODE C: Asynchronous Ping-Pong Pipeline */
          s_pingpong[0].state = PINGPONG_BUF_FREE;
          s_pingpong[1].state = PINGPONG_BUF_FREE;
          s_pingpong[0].err = 0;
          s_pingpong[1].err = 0;
          s_pingpong[0].bytes = 0;
          s_pingpong[1].bytes = 0;
          s_pingpong[0].peek_start_cyc = 0;
          s_pingpong[0].peek_end_cyc = 0;
          s_pingpong[1].peek_start_cyc = 0;
          s_pingpong[1].peek_end_cyc = 0;
          k_sem_reset(&s_slot_ready_sem);
          k_sem_reset(&s_slot_free_sem);
          k_sem_give(&s_slot_free_sem);
          k_sem_give(&s_slot_free_sem);

          s_reader_to_read = to_write;
          k_sem_give(&s_reader_start_sem);

          uint32_t w_idx = 0;
          while (to_write > 0 && sd_writer_err == 0) {
            if (k_sem_take(&s_slot_ready_sem, K_MSEC(5000)) != 0) {
              LOG_ERR("SD writer wait slot timeout");
              sd_writer_err = -ETIMEDOUT;
              break;
            }
            if (sd_writer_err != 0) {
              break;
            }

            struct pingpong_slot *slot = &s_pingpong[w_idx];
            if (slot->err != 0) {
              sd_writer_err = slot->err;
              break;
            }

            slot->state = PINGPONG_BUF_WRITING_SD;
            slot->write_start_cyc = k_cycle_get_64();

            /* Ping-pong writer: slot 0 uses &s_sd_writer_buf[0], slot 1 uses &s_sd_writer_buf[PINGPONG_SLOT_BYTES] */
            /* fs_write(sd_file_ptr, slot->buf, slot->bytes) */
            int w_ret = sd_write_all(sd_file_ptr, slot->buf, slot->bytes);
            slot->write_end_cyc = k_cycle_get_64();

            uint64_t w_diff = (slot->write_end_cyc >= slot->write_start_cyc)
                                  ? (slot->write_end_cyc - slot->write_start_cyc)
                                  : 0;
            uint32_t us = (uint32_t)k_cyc_to_us_floor64(w_diff);

            if (w_ret != 0) {
              uint64_t file_offset = sd_writer_bytes_written + WAV_HEADER_BYTES;
              uint32_t sector_approx = (uint32_t)(file_offset / 512);
              LOG_ERR("SD writer fs_write failed: ret=%d, expected=%u, file_offset=%llu, approx_sector=%u, write_latency=%u us",
                      w_ret, slot->bytes, (unsigned long long)file_offset, sector_approx, us);
#if defined(CONFIG_SOC_SERIES_STM32L4X)
              LOG_ERR("SDMMC hardware registers: STA=0x%08X, DCOUNT=%u, DCTRL=0x%08X, CLKCR=0x%08X",
                      (unsigned int)SDMMC1->STA, (unsigned int)SDMMC1->DCOUNT,
                      (unsigned int)SDMMC1->DCTRL, (unsigned int)SDMMC1->CLKCR);
#endif
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
              log_sdmmc_stage_diagnostics("FILESYSTEM_WRITE_PSRAM", w_ret, file_offset, slot->bytes);
#endif
              sd_writer_err = w_ret;
              if (s_active_stats_ptr && s_active_stats_ptr->first_error_stage[0] == '\0') {
#if defined(CONFIG_SOC_SERIES_STM32L4X)
                if (SDMMC1->STA & (SDMMC_STA_DCRCFAIL | SDMMC_STA_CCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_CTIMEOUT | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR)) {
                  strncpy(s_active_stats_ptr->first_error_stage, "SDMMC_TRANSFER", sizeof(s_active_stats_ptr->first_error_stage) - 1);
                } else
#endif
                {
                  strncpy(s_active_stats_ptr->first_error_stage, "FS_WRITE", sizeof(s_active_stats_ptr->first_error_stage) - 1);
                }
              }
              psram_staging_rewind_peek();
              /* Release pending slot so reader unblocks immediately */
              slot->state = PINGPONG_BUF_FREE;
              k_sem_give(&s_slot_free_sem);
              break;
            }

            /* Overlap accounting: cycle overlap between this SD write and the other slot's PSRAM peek */
            struct pingpong_slot *other = &s_pingpong[1 - w_idx];
            if (other->peek_start_cyc > 0 && other->peek_end_cyc > slot->write_start_cyc) {
              uint64_t ov_start = (other->peek_start_cyc > slot->write_start_cyc) ? other->peek_start_cyc : slot->write_start_cyc;
              uint64_t ov_end = (other->peek_end_cyc < slot->write_end_cyc) ? other->peek_end_cyc : slot->write_end_cyc;
              if (ov_end > ov_start) {
                sd_writer_overlap_us_sum += k_cyc_to_us_floor64(ov_end - ov_start);
              }
            }
            sd_writer_swap_count++;

            /* Commit consumer index ONLY after successful SD write */
            slot->state = PINGPONG_BUF_COMMITTED;
            uint64_t t_c0 = k_cycle_get_64();
            int commit_ret = psram_staging_commit(slot->bytes);
            uint64_t t_c1 = k_cycle_get_64();
            uint32_t commit_us = (uint32_t)k_cyc_to_us_floor64((t_c1 >= t_c0) ? (t_c1 - t_c0) : 0);
            sd_writer_commit_us_sum += commit_us;

            if (commit_ret != 0) {
              LOG_ERR("SD writer staging commit failed after successful write (%d)",
                      commit_ret);
              sd_writer_err = commit_ret;
              break;
            }

            /* Free slot for next reader fill */
            slot->state = PINGPONG_BUF_FREE;
            k_sem_give(&s_slot_free_sem);

            sd_writer_write_us_sum += us;
            batch_write_us_sum += us;
            sd_writer_write_count++;
            if (us > sd_writer_worst_write_us) {
              sd_writer_worst_write_us = us;
            }
            sd_writer_bytes_written += slot->bytes;
            batch_bytes_written += slot->bytes;
            to_write -= slot->bytes;

            w_idx = (w_idx + 1) % 2;
          }

          /* Ensure reader thread has completed this batch */
          k_sem_take(&s_reader_done_sem, K_FOREVER);
        }

        uint64_t t_batch_end = k_cycle_get_64();
        uint64_t batch_cyc =
            (t_batch_end >= t_batch_start) ? (t_batch_end - t_batch_start) : 0;
        uint64_t batch_wall_us = k_cyc_to_us_floor64(batch_cyc);
        uint32_t batch_ms = (uint32_t)(batch_wall_us / 1000ULL);
        sd_writer_batch_wall_us_sum += batch_wall_us;
        if (batch_ms > sd_writer_worst_batch_ms) {
          sd_writer_worst_batch_ms = batch_ms;
        }

        if (sd_writer_err == 0 && batch_bytes_written > 0) {
          uint32_t active_kb_s = batch_write_us_sum ? (uint32_t)(((uint64_t)batch_bytes_written * 1000000ULL) / (batch_write_us_sum * 1024ULL)) : 0;
          uint32_t wall_kb_s = batch_ms ? (uint32_t)(((uint64_t)batch_bytes_written * 1000ULL) / ((uint64_t)batch_ms * 1024ULL)) : 0;
          LOG_INF("Batch %u completed: %u bytes written to SD in %u ms (wall rate: %u KB/s, write: %u ms / %u KB/s, peek: %u ms, total committed: %llu bytes, PSRAM occupied: %u KB)",
                  cur_batch, batch_bytes_written, batch_ms, wall_kb_s,
                  (uint32_t)(batch_write_us_sum / 1000ULL), active_kb_s,
                  (uint32_t)(batch_peek_us_sum / 1000ULL),
                  sd_writer_bytes_written, psram_staging_occupied() >> 10);
        }

        if (sd_writer_err != 0) {
          LOG_ERR("Batch %u interrupted: wrote %u of %u bytes to SD before failure (%d)",
                  cur_batch, batch_bytes_written, to_write + batch_bytes_written, sd_writer_err);
          break;
        }
      }

      if (sd_writer_flush_req &&
          (psram_staging_occupied() == 0 || sd_writer_err != 0)) {
        if (sd_writer_err != 0) {
          /* Uncommitted bytes remain in staging; count them as unconfirmed SD data. */
          sd_writer_failed_bytes = psram_staging_occupied();
        }
        k_sem_give(&sd_writer_done_sem);
      }
    } else {
      /* Non-PSRAM mode: Drain processed blocks from bounded RAM queue
       * (sd_block_q) */
      struct sd_block_msg msg;
      while (k_msgq_get(&sd_block_q, &msg, K_NO_WAIT) == 0) {
        if (sd_writer_err != 0 || !sd_file_ptr) {
          sd_writer_failed_bytes += msg.bytes;
          recorder_buf_set_state(msg.buf_id, BUF_OWNER_FREE);
          k_mem_slab_free(&pool, msg.blk);
          continue;
        }

        uint64_t t0 = k_cycle_get_64();
        int w_ret = sd_write_all(sd_file_ptr, msg.blk, msg.bytes);
        uint64_t t1 = k_cycle_get_64();
        uint32_t diff_cyc = (t1 >= t0) ? (t1 - t0) : 0;
        uint32_t us = (uint32_t)k_cyc_to_us_floor64(diff_cyc);

        recorder_buf_set_state(msg.buf_id, BUF_OWNER_FREE);
        k_mem_slab_free(&pool, msg.blk);

        if (w_ret != 0) {
          LOG_ERR("SD writer fs_write failed: ret=%d, expected=%u", w_ret,
                  msg.bytes);
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
          log_sdmmc_stage_diagnostics("FILESYSTEM_WRITE_QUEUE", w_ret,
                                      sd_writer_bytes_written + WAV_HEADER_BYTES, msg.bytes);
#endif
          sd_writer_err = w_ret;
          sd_writer_failed_bytes += msg.bytes;
          break;
        }

        sd_writer_write_us_sum += us;
        sd_writer_write_count++;
        if (us > sd_writer_worst_write_us) {
          sd_writer_worst_write_us = us;
        }
        if (us > SLOW_WRITE_US) {
          LOG_WRN("slow SD write: %u ms (RAM queue %u/%u)", us / 1000,
                  (unsigned)k_msgq_num_used_get(&sd_block_q), NUM_BLOCKS);
        }
        sd_writer_bytes_written += msg.bytes;
      }

      if (sd_writer_flush_req &&
          (k_msgq_num_used_get(&sd_block_q) == 0 || sd_writer_err != 0)) {
        k_sem_give(&sd_writer_done_sem);
      }
    }
  }
}

static FATFS fat_fs;
static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
    .mnt_point = "/SD:",
};
static bool mounted;

static volatile uint32_t drops;
static volatile uint32_t dma_alloc_drops;
static volatile uint32_t queue_submit_drops;
static volatile uint32_t sram_writer_drops;
static volatile uint32_t high_water;
static volatile uint32_t audio_queue_high_water;
static volatile uint32_t sd_queue_high_water;

/* Atomic variables for drop counters and watermark tracking */
static atomic_t s_drops;
static atomic_t s_dma_alloc_drops;
static atomic_t s_queue_submit_drops;
static atomic_t s_sram_writer_drops;
static atomic_t s_high_water;
static atomic_t s_audio_queue_high_water;
static atomic_t s_sd_queue_high_water;

static inline void recorder_update_high_water(atomic_t *target, uint32_t val, volatile uint32_t *compat) {
  atomic_val_t old_val = atomic_get(target);
  while (val > (uint32_t)old_val) {
    if (atomic_cas(target, old_val, (atomic_val_t)val)) {
      if (compat) *compat = val;
      break;
    }
    old_val = atomic_get(target);
  }
}

/* ---- audio_sink callbacks: called from the high-priority acquisition thread ---- */
static void *sink_alloc(void *user) {
  void *blk = NULL;

  ARG_UNUSED(user);
  if (k_mem_slab_alloc(&pool, &blk, K_NO_WAIT) != 0) {
    atomic_inc(&s_dma_alloc_drops);
    atomic_inc(&s_drops);
    dma_alloc_drops = (uint32_t)atomic_get(&s_dma_alloc_drops);
    drops = (uint32_t)atomic_get(&s_drops);
    return NULL;
  }
  uint8_t id = recorder_buf_ptr_to_id(blk);
  recorder_buf_set_state(id, BUF_OWNER_DMA);
  return blk;
}

static int sink_submit(void *blk, void *user) {
  ARG_UNUSED(user);
  uint8_t id = recorder_buf_ptr_to_id(blk);
  recorder_buf_set_state(id, BUF_OWNER_STAGED);
  if (k_msgq_put(&filled_q, &id, K_NO_WAIT) != 0) {
    /* cannot happen (queue as deep as the pool) but never leak a block */
    atomic_inc(&s_queue_submit_drops);
    atomic_inc(&s_drops);
    queue_submit_drops = (uint32_t)atomic_get(&s_queue_submit_drops);
    drops = (uint32_t)atomic_get(&s_drops);
    recorder_buf_set_state(id, BUF_OWNER_FREE);
    k_mem_slab_free(&pool, blk);
    return -ENOBUFS;
  }
  uint32_t queued = k_msgq_num_used_get(&filled_q);
  recorder_update_high_water(&s_audio_queue_high_water, queued, &audio_queue_high_water);
  /*
   * Track true buffer high-water mark:
   * Total audio blocks currently held in RAM (in queue + actively being
   * written) is (NUM_BLOCKS - k_mem_slab_num_free_get(&pool)). This accurately
   * reflects pool pressure.
   */
  uint32_t in_use = NUM_BLOCKS - k_mem_slab_num_free_get(&pool);
  recorder_update_high_water(&s_high_water, in_use, &high_water);
  return 0;
}

static void sink_release(void *blk, void *user) {
  ARG_UNUSED(user);
  if (blk) {
    uint8_t id = recorder_buf_ptr_to_id(blk);
    recorder_buf_set_state(id, BUF_OWNER_FREE);
    k_mem_slab_free(&pool, blk);
  }
}

static const struct audio_sink sink = {
    .alloc = sink_alloc,
    .release = sink_release,
    .submit = sink_submit,
};

static void rec_compute_timing(struct rec_stats *st, uint64_t total_write_us,
                               uint64_t wall_us, uint32_t write_count,
                               uint32_t worst_us) {
  st->total_write_us = total_write_us;
  st->session_wall_us = wall_us;
  st->write_count = write_count;
  st->worst_write_us = worst_us;
  st->elapsed_ms = (uint32_t)(wall_us / 1000ULL);

  if (write_count > 0) {
    st->avg_write_us = (uint32_t)(total_write_us / write_count);
  } else {
    st->avg_write_us = 0;
  }

  if (wall_us > 0) {
    uint64_t permille = (total_write_us * 1000ULL) / wall_us;
    /* Boundary check: clamp duty cycle to max 100.0% (1000 permille) */
    if (permille > 1000ULL) {
      permille = 1000ULL;
    }
    st->sd_busy_permille = (uint32_t)permille;
  } else {
    st->sd_busy_permille = 0;
  }
}

void recorder_timing_unit_test(void) {
  struct rec_stats tst = {0};

  /* Test 1: Normal 50% duty cycle */
  rec_compute_timing(&tst, 500000ULL, 1000000ULL, 10, 60000);
  __ASSERT_NO_MSG(tst.sd_busy_permille == 500);
  __ASSERT_NO_MSG(tst.avg_write_us == 50000);
  __ASSERT_NO_MSG(tst.elapsed_ms == 1000);

  /* Test 2: Boundary check - total_write_us exceeds wall_us (clamp to 1000
   * permille) */
  rec_compute_timing(&tst, 1500000ULL, 1000000ULL, 10, 150000);
  __ASSERT_NO_MSG(tst.sd_busy_permille == 1000);

  /* Test 3: Zero wall time (should not divide by zero) */
  rec_compute_timing(&tst, 100ULL, 0ULL, 1, 100);
  __ASSERT_NO_MSG(tst.sd_busy_permille == 0);

  /* Test 4: Zero write count */
  rec_compute_timing(&tst, 0ULL, 1000000ULL, 0, 0);
  __ASSERT_NO_MSG(tst.avg_write_us == 0);
  __ASSERT_NO_MSG(tst.sd_busy_permille == 0);

  LOG_INF("Timing calculation boundary unit tests passed");
}

int recorder_init(void) {
  recorder_buf_table_init();

  /* Run self-test on timing math */
  recorder_timing_unit_test();

  /* Initialize RTC Wakeup Timer subsystem */
  rtc_sleep_init();

  /* Initialize RTC Calendar subsystem */
  rtc_calendar_init();

  /* Initialize SD writer & PSRAM reader synchronization primitives and background threads */
  k_sem_init(&sd_writer_sem, 0, 1);
  k_sem_init(&sd_writer_done_sem, 0, 1);
  k_sem_init(&s_reader_start_sem, 0, 1);
  k_sem_init(&s_reader_done_sem, 0, 1);
  k_sem_init(&s_slot_ready_sem, 0, 2);
  k_sem_init(&s_slot_free_sem, 2, 2);

  sd_writer_tid = k_thread_create(&sd_writer_thread_data, sd_writer_stack,
                                  K_THREAD_STACK_SIZEOF(sd_writer_stack),
                                  sd_writer_thread_fn, NULL, NULL, NULL,
                                  SD_WRITER_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(sd_writer_tid, "sd_writer");

  psram_reader_tid = k_thread_create(&psram_reader_thread_data, psram_reader_stack,
                                    K_THREAD_STACK_SIZEOF(psram_reader_stack),
                                    psram_reader_thread_fn, NULL, NULL, NULL,
                                    PSRAM_READER_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(psram_reader_tid, "psram_reader");

#ifdef CONFIG_AUDIO_PSRAM_STAGING
  if (psram_is_present() && psram_is_integrity_verified()) {
    psram_staging_init((uint32_t)CONFIG_AUDIO_PSRAM_BUFFER_SIZE_KB * 1024U,
                       (uint32_t)CONFIG_AUDIO_SD_BATCH_SIZE_KB * 1024U);
    if (!psram_is_quad_mode()) {
      LOG_WRN("PSRAM staging disabled: Quad I/O unverified (single-line SPI fallback too slow for live audio); direct SRAM active");
      g_psram_staging_enabled = false;
    } else {
      LOG_INF("PSRAM staging buffer & SD batch writer initialized (%u KB buffer, "
              "%u KB batch, Quad I/O verified)",
              CONFIG_AUDIO_PSRAM_BUFFER_SIZE_KB, CONFIG_AUDIO_SD_BATCH_SIZE_KB);
      g_psram_staging_enabled = true;
    }
  } else {
    LOG_WRN("PSRAM staging disabled: not detected or integrity unverified; direct SRAM-to-SD active");
    g_psram_staging_enabled = false;
  }
#else
  g_psram_staging_enabled = false;
#endif

  int ret = audio_in_init(CONFIG_AUDIO_SAMPLE_RATE, &sink);
  if (ret) {
    LOG_ERR("audio_in_init failed: %d", ret);
    return ret;
  }

  /* Diagnostic: test raw SD card access before mounting FatFS */
  int sd_ret = disk_access_init("SD");
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[DBG] disk_access_init = %d\n", sd_ret);
#endif
  LOG_INF("SD diagnostic: disk_access_init = %d", sd_ret);

  if (sd_ret == 0) {
    uint32_t sector_count = 0;
    uint32_t sector_size = 0;

    int count_ret =
        disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
    int size_ret =
        disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);

    LOG_INF("SD diagnostic: sector count ret=%d count=%u", count_ret,
            sector_count);
    LOG_INF("SD diagnostic: sector size ret=%d size=%u", size_ret, sector_size);
    recorder_apply_sdmmc_clock();
  }

  /* Read-only filesystem diagnostic */
  uint8_t sector[512] __aligned(4);
  int rd = disk_access_read("SD", sector, 0, 1);
  LOG_INF("SD sector 0 read: %d", rd);

  if (rd == 0) {
#ifdef CONFIG_AUDIO_DIAGNOSTIC_MODE
    LOG_HEXDUMP_INF(sector, 96, "SD sector 0");
#endif

    uint32_t start_lba = (uint32_t)sector[454] | ((uint32_t)sector[455] << 8) |
                         ((uint32_t)sector[456] << 16) |
                         ((uint32_t)sector[457] << 24);

    LOG_INF("Sector 0 signature: %02x %02x", sector[510], sector[511]);
    LOG_INF("Partition type: 0x%02x, start LBA: %u", sector[450], start_lba);

    if (sector[510] == 0x55 && sector[511] == 0xAA && start_lba > 0) {
      rd = disk_access_read("SD", sector, start_lba, 1);
      LOG_INF("Partition boot sector read: %d", rd);

#ifdef CONFIG_AUDIO_DIAGNOSTIC_MODE
      if (rd == 0) {
        LOG_HEXDUMP_INF(sector, 96, "Partition boot sector");
      }
#endif
    }
  }

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[DBG] fs_mount start\n");
#endif
  ret = fs_mount(&mp);
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[DBG] fs_mount ret=%d\n", ret);
#endif
  if (ret) {
    LOG_ERR("SD mount failed: %d (card inserted? exFAT/FAT32 formatted? "
            "wiring/overlay?)",
            ret);
    return ret;
  }
  mounted = true;

  /* The raw SD diagnostics initialized the disk before FatFs mounted it.
   * Drop that temporary reference so fs_unmount fully deinitializes the
   * device and recorder_storage_recover can cleanly initialize it again. */
  int sd_ref_ret = disk_access_ioctl("SD", DISK_IOCTL_CTRL_DEINIT, NULL);
  if (sd_ref_ret != 0) {
    LOG_WRN("Could not release temporary SD diagnostic reference: %d", sd_ref_ret);
  }

  const char *fs_type_str = "Unknown";
  if (fat_fs.fs_type == FS_EXFAT) {
    fs_type_str = "exFAT";
  } else if (fat_fs.fs_type == FS_FAT32) {
    fs_type_str = "FAT32";
  } else if (fat_fs.fs_type == FS_FAT16) {
    fs_type_str = "FAT16";
  }

  struct fs_statvfs sv;
  if (fs_statvfs(mp.mnt_point, &sv) == 0) {
    uint32_t sector_size = (uint32_t)sv.f_bsize;
    uint32_t cluster_size = (uint32_t)sv.f_frsize;
    uint32_t sec_per_clus =
        (sector_size > 0) ? (cluster_size / sector_size) : 0;
    uint64_t free_mb = ((uint64_t)sv.f_bfree * cluster_size) >> 20;

    LOG_INF("SD mounted: %s filesystem, %llu MB free", fs_type_str, free_mb);
    LOG_INF("  Sector size        : %u bytes (statvfs f_bsize)", sector_size);
    LOG_INF(
        "  Cluster size       : %u bytes (statvfs f_frsize / allocation unit)",
        cluster_size);
    LOG_INF("  Sectors per cluster: %u", sec_per_clus);
  } else {
    LOG_INF("SD mounted: %s filesystem", fs_type_str);
  }
  return 0;
}

int recorder_storage_recover(void) {
  if (recorder_is_active()) {
    return -EBUSY;
  }
  s_storage_recovering = true;
  if (mounted) {
    int ret = fs_unmount(&mp);
    if (ret != 0) {
      LOG_WRN("Storage recovery: filesystem unmount returned %d; proceeding with re-init", ret);
    }
    mounted = false;
  }

  /* FatFs unmount powers the disk off through CTRL_DEINIT. Reinitialize the
   * disk before mounting the volume again; do not retry file operations on
   * the stale mounted filesystem after an SDMMC timeout. */
  int ret = disk_access_init("SD");
  if (ret != 0) {
    LOG_ERR("Storage recovery: SDMMC reinitialization failed: %d", ret);
    s_storage_state = STORAGE_STATE_FAULT;
    s_storage_recovery_result = ret;
    s_storage_recovering = false;
    return ret;
  }

  ret = fs_mount(&mp);
  if (ret != 0) {
    LOG_ERR("Storage recovery: filesystem remount failed: %d", ret);
    s_storage_state = STORAGE_STATE_FAULT;
    s_storage_recovery_result = ret;
    s_storage_recovering = false;
    return ret;
  }
  mounted = true;
  recorder_apply_sdmmc_clock();
  ret = disk_access_ioctl("SD", DISK_IOCTL_CTRL_DEINIT, NULL);
  if (ret != 0) {
    LOG_WRN("Storage recovery: could not release temporary disk reference: %d", ret);
  }

  s_storage_state = STORAGE_STATE_HEALTHY;
  s_storage_recovery_result = 0;
  sd_writer_err = 0;
  psram_staging_reset();
  s_storage_recovering = false;

  LOG_INF("Storage recovery: SDMMC reinitialized and filesystem remounted");
  return 0;
}

static int next_filename(char *name, size_t len) {
  struct fs_dirent ent;

  for (unsigned int n = 1; n < 10000; n++) {
    snprintf(name, len, "/SD:/REC%04u.WAV", n);
    int ret = fs_stat(name, &ent);
    if (ret == -ENOENT) {
      return 0; /* does not exist yet */
    }
    if (ret != 0) return ret;
  }
  return -ENOSPC;
}

static int ensure_dir_exists(const char *path) {
#if defined(CONFIG_FILE_SYSTEM)
  struct fs_dirent ent;
  int stat_ret = fs_stat(path, &ent);
  if (stat_ret == 0) {
    if (ent.type == FS_DIR_ENTRY_DIR) {
      return 0; /* Directory already exists and is usable */
    }
    LOG_ERR("Path %s exists but is not a directory", path);
    return -ENOTDIR;
  }

  /* Directory does not exist; attempt to create it */
  int ret = fs_mkdir(path);
  if (ret == 0) {
    return 0; /* Successfully created */
  }
  if (ret == -EEXIST) {
    /* Handle -EEXIST only after verifying that directory exists and is usable */
    stat_ret = fs_stat(path, &ent);
    if (stat_ret == 0 && ent.type == FS_DIR_ENTRY_DIR) {
      return 0;
    }
    LOG_ERR("Directory %s failed creation with -EEXIST but stat failed (%d)",
            path, stat_ret);
    return (stat_ret != 0) ? stat_ret : -ENOTDIR;
  }
  LOG_ERR("Failed to create directory %s: %d", path, ret);
  return ret;
#else
  ARG_UNUSED(path);
  return 0;
#endif
}

static int generate_recording_filename(char *name, size_t len,
                                       bool *rtc_valid) {
  *rtc_valid = false;
  if (rtc_calendar_is_valid()) {
    struct rtc_calendar_time td;
    if (rtc_calendar_get(&td) == 0) {
      struct wildsonic_config *cfg = console_get_config();
      uint8_t folder_mode = cfg ? cfg->folder_structure : 2;

      struct schedule_config scfg;
      scheduler_get_config(&scfg);
      int tz_off = scfg.time_zone_offset;

      time_t epoch = scheduler_calendar_to_epoch(&td) + tz_off;
      struct rtc_calendar_time loc_td;
      scheduler_epoch_to_calendar(epoch, &loc_td);
      loc_td.subsec = td.subsec;

      char dir[96] = "/SD:";
      if (folder_mode == 1) {
        snprintf(dir, sizeof(dir), "/SD:/%04u%02u%02u", loc_td.year, loc_td.month, loc_td.day);
        int d_ret = ensure_dir_exists(dir);
        if (d_ret != 0) {
          LOG_ERR("Failed to ensure directory %s exists: %d", dir, d_ret);
          return d_ret;
        }
      } else if (folder_mode == 2) {
        char day_dir[32];
        snprintf(day_dir, sizeof(day_dir), "/SD:/%04u%02u%02u", loc_td.year, loc_td.month, loc_td.day);
        int d_ret = ensure_dir_exists(day_dir);
        if (d_ret != 0) {
          LOG_ERR("Failed to ensure day directory %s exists: %d", day_dir, d_ret);
          return d_ret;
        }
        snprintf(dir, sizeof(dir), "%s/%04u%02u%02u_%02u", day_dir, loc_td.year, loc_td.month, loc_td.day, loc_td.hour);
        d_ret = ensure_dir_exists(dir);
        if (d_ret != 0) {
          LOG_ERR("Failed to ensure hour directory %s exists: %d", dir, d_ret);
          return d_ret;
        }
      }

      for (unsigned int step = 0; step < 1000; step++) {
        uint16_t candidate_ms = (uint16_t)((loc_td.subsec + step) % 1000U);
        snprintf(name, len, "%s/%04u%02u%02u_%02u%02u%02u_%03u.wav",
                 dir, loc_td.year, loc_td.month, loc_td.day,
                 loc_td.hour, loc_td.min, loc_td.sec, candidate_ms);
#if defined(CONFIG_FILE_SYSTEM)
        struct fs_dirent ent;
        int stat_ret = fs_stat(name, &ent);
        if (stat_ret == -ENOENT) {
          *rtc_valid = true;
          return 0;
        }
        if (stat_ret != 0) return stat_ret;
#else
        *rtc_valid = true;
        return 0;
#endif
      }
    }
  }
  return next_filename(name, len);
}

int recorder_scan_last_file(char *out_folder, size_t folder_len, char *out_file, size_t file_len) {
  if (!out_file || file_len == 0) return -EINVAL;
  out_file[0] = '\0';
  if (out_folder && folder_len > 0) out_folder[0] = '\0';

#if defined(CONFIG_FILE_SYSTEM)
  struct wildsonic_config *cfg = console_get_config();
  uint8_t folder_mode = cfg ? cfg->folder_structure : 2;

  if (folder_mode == 2) {
    /* HOUR mode: Find latest YYYYMMDD day dir, then latest YYYYMMDD_HH hour dir, then latest WAV file */
    struct fs_dir_t dirp;
    fs_dir_t_init(&dirp);
    if (fs_opendir(&dirp, "/SD:") == 0) {
      char latest_day[16] = {0};
      struct fs_dirent entry;
      while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_DIR && strlen(entry.name) == 8) {
          bool digits = true;
          for (int i = 0; i < 8; i++) {
            if (!isdigit((unsigned char)entry.name[i])) { digits = false; break; }
          }
          if (digits && strcmp(entry.name, latest_day) > 0) {
            strncpy(latest_day, entry.name, sizeof(latest_day) - 1);
          }
        }
      }
      fs_closedir(&dirp);

      if (latest_day[0] != '\0') {
        char day_path[32];
        snprintf(day_path, sizeof(day_path), "/SD:/%s", latest_day);
        fs_dir_t_init(&dirp);
        if (fs_opendir(&dirp, day_path) == 0) {
          char latest_hour[24] = {0};
          while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
            if (entry.type == FS_DIR_ENTRY_DIR && strlen(entry.name) == 11) {
              if (strcmp(entry.name, latest_hour) > 0) {
                strncpy(latest_hour, entry.name, sizeof(latest_hour) - 1);
              }
            }
          }
          fs_closedir(&dirp);

          if (latest_hour[0] != '\0') {
            char hour_path[64];
            snprintf(hour_path, sizeof(hour_path), "%s/%s", day_path, latest_hour);
            fs_dir_t_init(&dirp);
            if (fs_opendir(&dirp, hour_path) == 0) {
              char latest_wav[48] = {0};
              while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
                if (entry.type == FS_DIR_ENTRY_FILE) {
                  size_t nlen = strlen(entry.name);
                  if (nlen > 4 && (strcasecmp(&entry.name[nlen - 4], ".wav") == 0)) {
                    if (strcmp(entry.name, latest_wav) > 0) {
                      strncpy(latest_wav, entry.name, sizeof(latest_wav) - 1);
                    }
                  }
                }
              }
              fs_closedir(&dirp);

              if (latest_wav[0] != '\0') {
                if (out_folder && folder_len > 0) {
                  strncpy(out_folder, hour_path, folder_len - 1);
                  out_folder[folder_len - 1] = '\0';
                }
                strncpy(out_file, latest_wav, file_len - 1);
                out_file[file_len - 1] = '\0';
                return 0;
              }
            }
          }
        }
      }
    }
  }

  if (folder_mode == 1 || folder_mode == 2) {
    /* DAY mode (or fallback from HOUR if no hour subfolders): Find latest YYYYMMDD day dir, then latest WAV file */
    struct fs_dir_t dirp;
    fs_dir_t_init(&dirp);
    if (fs_opendir(&dirp, "/SD:") == 0) {
      char latest_day[16] = {0};
      struct fs_dirent entry;
      while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
        if (entry.type == FS_DIR_ENTRY_DIR && strlen(entry.name) == 8) {
          bool digits = true;
          for (int i = 0; i < 8; i++) {
            if (!isdigit((unsigned char)entry.name[i])) { digits = false; break; }
          }
          if (digits && strcmp(entry.name, latest_day) > 0) {
            strncpy(latest_day, entry.name, sizeof(latest_day) - 1);
          }
        }
      }
      fs_closedir(&dirp);

      if (latest_day[0] != '\0') {
        char day_path[32];
        snprintf(day_path, sizeof(day_path), "/SD:/%s", latest_day);
        fs_dir_t_init(&dirp);
        if (fs_opendir(&dirp, day_path) == 0) {
          char latest_wav[48] = {0};
          while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
            if (entry.type == FS_DIR_ENTRY_FILE) {
              size_t nlen = strlen(entry.name);
              if (nlen > 4 && (strcasecmp(&entry.name[nlen - 4], ".wav") == 0)) {
                if (strcmp(entry.name, latest_wav) > 0) {
                  strncpy(latest_wav, entry.name, sizeof(latest_wav) - 1);
                }
              }
            }
          }
          fs_closedir(&dirp);

          if (latest_wav[0] != '\0') {
            if (out_folder && folder_len > 0) {
              strncpy(out_folder, day_path, folder_len - 1);
              out_folder[folder_len - 1] = '\0';
            }
            strncpy(out_file, latest_wav, file_len - 1);
            out_file[file_len - 1] = '\0';
            return 0;
          }
        }
      }
    }
  }

  /* FLAT mode (or fallback): Scan root folder for latest WAV file */
  struct fs_dir_t dirp;
  fs_dir_t_init(&dirp);
  if (fs_opendir(&dirp, "/SD:") == 0) {
    char latest_wav[48] = {0};
    struct fs_dirent entry;
    while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != '\0') {
      if (entry.type == FS_DIR_ENTRY_FILE) {
        size_t nlen = strlen(entry.name);
        if (nlen > 4 && (strcasecmp(&entry.name[nlen - 4], ".wav") == 0)) {
          if (strcmp(entry.name, latest_wav) > 0) {
            strncpy(latest_wav, entry.name, sizeof(latest_wav) - 1);
          }
        }
      }
    }
    fs_closedir(&dirp);

    if (latest_wav[0] != '\0') {
      if (out_folder && folder_len > 0) {
        out_folder[0] = '\0';
      }
      strncpy(out_file, latest_wav, file_len - 1);
      out_file[file_len - 1] = '\0';
      return 0;
    }
  }
#endif

  return -ENOENT;
}

static uint32_t drain_queue(void) {
  uint8_t drain_id;
  uint32_t count = 0;

  while (k_msgq_get(&filled_q, &drain_id, K_NO_WAIT) == 0) {
    void *blk = recorder_buf_id_to_ptr(drain_id);
    recorder_buf_set_state(drain_id, BUF_OWNER_FREE);
    if (blk) {
      k_mem_slab_free(&pool, blk);
    }
    count++;
  }
  return count;
}

int recorder_verify_file_format(const char *path, uint32_t expected_rate,
                                uint16_t expected_channels, uint16_t expected_bits,
                                uint64_t expected_data_bytes) {
  struct fs_file_t vf;
  fs_file_t_init(&vf);
  int ret = fs_open(&vf, path, FS_O_READ);
  if (ret) {
    LOG_ERR("Verify open %s failed: %d", path, ret);
    return ret;
  }

  uint8_t hdr[WAV_HEADER_BYTES];
  ssize_t r = fs_read(&vf, hdr, sizeof(hdr));
  fs_close(&vf);

  if (r < 0) {
    LOG_ERR("Verify read %s failed with I/O error: %d", path, (int)r);
    return (int)r;
  }
  if (r != sizeof(hdr)) {
    LOG_ERR("Verify read %s short read: %d bytes read (expected %zu)", path,
            (int)r, sizeof(hdr));
    return -EIO;
  }

  ret = wav_verify_header(hdr, expected_rate, expected_channels, expected_bits,
                          (uint32_t)expected_data_bytes);
  if (ret) {
    LOG_ERR("WAV header verification failed for %s: code %d", path, ret);
    return -EBADMSG; /* Return distinct errno rather than internal code -2 */
  }

  struct fs_dirent ent;
  ret = fs_stat(path, &ent);
  if (ret == 0) {
    uint64_t expected_file_size = WAV_HEADER_BYTES + expected_data_bytes;
    if ((uint64_t)ent.size != expected_file_size) {
      LOG_ERR("File size mismatch for %s: got %zu, expected %llu", path,
              ent.size, expected_file_size);
      return -EINVAL;
    }
  }
  return 0;
}

int recorder_verify_file(const char *path, uint32_t expected_rate,
                         uint16_t expected_bits, uint64_t expected_data_bytes) {
  return recorder_verify_file_format(path, expected_rate, 1, expected_bits, expected_data_bytes);
}

static volatile bool recorder_abort_requested = false;
static volatile enum recorder_stop_reason s_recorder_stop_reason = RECORDER_STOP_NONE;
static volatile bool recorder_active = false;
static volatile int s_recorder_last_error = 0;
static volatile int s_recorder_last_sd_error = 0;
static char s_active_file_path[80] = {0};

void recorder_set_stop_reason(enum recorder_stop_reason reason) {
  s_recorder_stop_reason = reason;
}

enum recorder_stop_reason recorder_get_stop_reason(void) {
  return s_recorder_stop_reason;
}

void recorder_request_stop_reason(enum recorder_stop_reason reason) {
  s_recorder_stop_reason = reason;
  recorder_abort_requested = true;
}

bool recorder_is_active(void) { return recorder_active; }

int recorder_sd_verify_idle(void) {
#if defined(CONFIG_SOC_SERIES_STM32L4X)
  if (SDMMC1->STA & (SDMMC_STA_TXACT | SDMMC_STA_RXACT | SDMMC_STA_CMDACT)) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] SDMMC idle check failed: STA=0x%08X (TXACT=%d, RXACT=%d, CMDACT=%d)\n",
           (unsigned int)SDMMC1->STA,
           (SDMMC1->STA & SDMMC_STA_TXACT) ? 1 : 0,
           (SDMMC1->STA & SDMMC_STA_RXACT) ? 1 : 0,
           (SDMMC1->STA & SDMMC_STA_CMDACT) ? 1 : 0);
#endif
    return -EBUSY;
  }
  if (SDMMC1->DCOUNT != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] SDMMC idle check failed: DCOUNT=%u\n", (unsigned int)SDMMC1->DCOUNT);
#endif
    return -EBUSY;
  }
  if (LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_DMA2) &&
      LL_DMA_IsEnabledChannel(DMA2, LL_DMA_CHANNEL_4)) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] SDMMC idle check failed: DMA2 Channel 4 active\n");
#endif
    return -EBUSY;
  }
#endif
  return 0;
}

bool recorder_is_shutdown_ready(void) {
  if (recorder_active) {
    return false;
  }
  if (sd_file_ptr != NULL) {
    return false;
  }
  if (sd_bounce_buf != NULL) {
    return false;
  }
  if (s_pingpong[0].state != PINGPONG_BUF_FREE ||
      s_pingpong[1].state != PINGPONG_BUF_FREE) {
    return false;
  }
  if (sd_writer_flush_req) {
    return false;
  }
  if (s_storage_recovering) {
    return false;
  }
  if (k_msgq_num_used_get(&filled_q) != 0 ||
      k_msgq_num_used_get(&sd_block_q) != 0) {
    return false;
  }
  if (audio_in_is_dma_active() || audio_in_is_worker_active()) {
    return false;
  }
  if (psram_staging_is_active() && !psram_staging_is_idle()) {
    return false;
  }
  if (recorder_sd_verify_idle() != 0) {
    return false;
  }
  return true;
}

bool recorder_is_abort_requested(void) { return recorder_abort_requested; }

int recorder_get_last_error(void) { return s_recorder_last_error; }

int recorder_get_last_sd_error(void) { return s_recorder_last_sd_error; }
int recorder_get_storage_recovery_result(void) { return s_storage_recovery_result; }

void recorder_request_stop(void) {
  if (s_recorder_stop_reason == RECORDER_STOP_NONE) {
    s_recorder_stop_reason = RECORDER_STOP_USER;
  }
  recorder_abort_requested = true;
}

static volatile shutdown_stage_t s_shutdown_stage = SHUTDOWN_STAGE_NONE;

const char *recorder_shutdown_stage_name(shutdown_stage_t stage) {
  switch (stage) {
  case SHUTDOWN_STAGE_AUDIO_STOP: return "AUDIO_STOP";
  case SHUTDOWN_STAGE_WRITER_DRAIN: return "WRITER_DRAIN";
  case SHUTDOWN_STAGE_FS_SYNC: return "FS_SYNC";
  case SHUTDOWN_STAGE_PSRAM_SLEEP: return "PSRAM_SLEEP";
  case SHUTDOWN_STAGE_SD_SLEEP: return "SD_SLEEP";
  case SHUTDOWN_STAGE_STOP2: return "STOP2";
  case SHUTDOWN_STAGE_COMPLETE: return "COMPLETE";
  default: return "NONE";
  }
}

shutdown_stage_t recorder_get_shutdown_stage(void) {
  return s_shutdown_stage;
}

static volatile enum recorder_power_state s_power_state = REC_PWR_ACTIVE;

enum recorder_power_state recorder_get_power_state(void) {
  return s_power_state;
}

static int post_wake_check(void) {
  int psram_wake_rc = psram_wake();
  if (psram_wake_rc != 0) {
    LOG_ERR("post_wake_check: PSRAM wake/ID check failed (%d)", psram_wake_rc);
    return psram_wake_rc;
  }
  if (recorder_sd_verify_idle() != 0) {
    LOG_WRN("post_wake_check: SDMMC not idle after wake");
  }
  (void)audio_in_verify_ready();
  return 0;
}

int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us) {
  if (seconds == 0) {
    if (wake_recovery_us) {
      *wake_recovery_us = 0;
    }
    return 0;
  }

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[TEST] SLEEP_REQUESTED: duration=%u s\n", seconds);
#endif

  /* 1. Reject sleep if recording is active or any recording/flush operation remains in progress */
  if (recorder_active || sd_writer_flush_req) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] Sleep precondition failed: recorder_active=%d, sd_writer_flush_req=%d\n",
           recorder_active, sd_writer_flush_req);
#endif
    LOG_ERR("recorder_sleep rejected: active recording or file operation in progress");
    return -EBUSY;
  }

  s_power_state = REC_PWR_SLEEP_PREP;
  s_shutdown_stage = SHUTDOWN_STAGE_AUDIO_STOP;

  /* 2-4. Stop audio acquisition pipeline if active (TIM6, ADC, DMA, and wait bounded worker drain) */
  if (audio_in_is_dma_active()) {
    int stop_rc = audio_in_stop();
    if (stop_rc != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("[TEST] Sleep precondition failed: audio_in_stop=%d\n", stop_rc);
#endif
      LOG_ERR("recorder_sleep: audio_in_stop failed (%d)", stop_rc);
      s_power_state = REC_PWR_ACTIVE;
      return stop_rc;
    }
  }

  /* Pending blocks are uncommitted audio; refuse sleep rather than discard */
  if (k_msgq_num_used_get(&filled_q) != 0 ||
      k_msgq_num_used_get(&sd_block_q) != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] Sleep precondition failed: audio queues not empty (filled=%u, sd_block=%u)\n",
           k_msgq_num_used_get(&filled_q), k_msgq_num_used_get(&sd_block_q));
#endif
    LOG_ERR("recorder_sleep rejected: pending audio blocks remain in RAM queue");
    s_power_state = REC_PWR_ACTIVE;
    return -EBUSY;
  }

  /* Verify DMA and acquisition worker are completely stopped */
  if (audio_in_is_dma_active() || audio_in_is_worker_active()) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] Sleep precondition failed: audio DMA active=%d or worker active=%d\n",
           audio_in_is_dma_active(), audio_in_is_worker_active());
#endif
    LOG_ERR("recorder_sleep rejected: audio DMA or acquisition worker still active");
    s_power_state = REC_PWR_ACTIVE;
    return -EBUSY;
  }

  /* 6-7. Flush PSRAM staging pipeline and ensure SD writer is completely idle */
  if (psram_staging_is_active() && psram_staging_occupied() > 0) {
    if (s_storage_state == STORAGE_STATE_FAULT) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("[TEST] Sleep precondition failed: dirty audio in PSRAM (%u B) while storage fault\n",
             psram_staging_occupied());
#endif
      LOG_ERR("recorder_sleep rejected: uncommitted dirty audio buffers remain while storage in fault");
      s_power_state = REC_PWR_ACTIVE;
      return -EBUSY;
    }
    psram_staging_flush_publish();
    sd_writer_flush_req = true;
    k_sem_give(&sd_writer_sem);
    uint32_t timeout_ms = writer_completion_timeout_ms(psram_staging_occupied());
    if (k_sem_take(&sd_writer_done_sem, K_MSEC(timeout_ms)) != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("[TEST] Sleep precondition failed: timeout flushing PSRAM staging to SD\n");
#endif
      LOG_ERR("recorder_sleep: timeout flushing staging pipeline to SD");
      s_power_state = REC_PWR_ACTIVE;
      return -ETIMEDOUT;
    }
    sd_writer_flush_req = false;
    if (sd_writer_err != 0 || psram_staging_occupied() != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("[TEST] Sleep precondition failed: sd_writer_err=%d, occupied=%u\n",
             sd_writer_err, psram_staging_occupied());
#endif
      s_power_state = REC_PWR_ACTIVE;
      return (sd_writer_err != 0) ? sd_writer_err : -EIO;
    }
  }

  if (psram_staging_is_active()) {
    struct psram_staging_stats ps;
    psram_staging_get_stats(&ps);
    if (ps.occupied_bytes != 0 || ps.in_flight_bytes != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("[TEST] Sleep precondition failed: PSRAM staging not idle (occupied=%u, in_flight=%u)\n",
             ps.occupied_bytes, ps.in_flight_bytes);
#endif
      s_power_state = REC_PWR_ACTIVE;
      return -EBUSY;
    }
  }

  /* 8. Open file guard: sleep only between closed recording sessions */
  if (sd_file_ptr != NULL) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] Sleep precondition failed: SD file still open (%p)\n", sd_file_ptr);
#endif
    LOG_ERR("recorder_sleep rejected: file still open");
    s_power_state = REC_PWR_ACTIVE;
    return -EBUSY;
  }
  if (sd_bounce_buf != NULL ||
      s_pingpong[0].state != PINGPONG_BUF_FREE ||
      s_pingpong[1].state != PINGPONG_BUF_FREE) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] Sleep precondition failed: SD writer bounce buffer held (%p)\n", sd_bounce_buf);
#endif
    LOG_ERR("recorder_sleep rejected: writer buffers held");
    s_power_state = REC_PWR_ACTIVE;
    return -EBUSY;
  }

  /* Verify SDMMC controller, DMA, and data lines are completely idle */
  if (s_storage_state == STORAGE_STATE_HEALTHY) {
    int sd_idle_rc = recorder_sd_verify_idle();
    if (sd_idle_rc != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("[TEST] Sleep precondition failed: SDMMC controller not idle (%d)\n", sd_idle_rc);
#endif
      LOG_ERR("recorder_sleep rejected: SDMMC controller not idle (%d)", sd_idle_rc);
      s_power_state = REC_PWR_ACTIVE;
      return sd_idle_rc;
    }
  }
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[TEST] SDMMC_IDLE\n");
#endif

  s_shutdown_stage = SHUTDOWN_STAGE_PSRAM_SLEEP;
  /* 9. Put PSRAM into Deep Power Down (DPD) and verify the command succeeded */
  int psram_sleep_rc = psram_sleep();
  if (psram_sleep_rc != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] Sleep precondition failed: PSRAM DPD failed (%d)\n", psram_sleep_rc);
#endif
    LOG_ERR("recorder_sleep: PSRAM DPD command failed (%d)", psram_sleep_rc);
    s_power_state = REC_PWR_ACTIVE;
    return psram_sleep_rc;
  }

  /* 10. Snapshot pre-sleep RTC calendar state */
  struct rtc_calendar_time pre_cal;
  int64_t pre_epoch = 0;
  if (rtc_calendar_get(&pre_cal) == 0 && pre_cal.valid) {
    pre_epoch = scheduler_calendar_to_epoch(&pre_cal);
  }
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  rtc_calendar_log_snapshot("PRE_STOP2", pre_epoch + seconds);
#endif

  /* 11. Enter STM32L476 Stop 2 low-power mode */
  battery_monitor_suspend();
  s_power_state = REC_PWR_RTC_ARMED;
  LOG_INF("Entering STM32L476 Stop 2 low-power mode for %u seconds (RTC Wakeup Timer)...", seconds);
  const uint64_t t_pre = k_uptime_get();

  s_shutdown_stage = SHUTDOWN_STAGE_STOP2;
  s_power_state = REC_PWR_STOP2;
  int res = rtc_sleep_enter_stop2(seconds);
  s_power_state = REC_PWR_WAKE_RECOVERY;

  const uint64_t t_post = k_uptime_get();
  uint64_t elapsed_ms = (t_post >= t_pre) ? (t_post - t_pre) : ((uint64_t)seconds * 1000);
  uint64_t expected_ms = (uint64_t)seconds * 1000;
  uint32_t rec_us = (elapsed_ms > expected_ms)
                        ? (uint32_t)((elapsed_ms - expected_ms) * 1000)
                        : 0;
  if (wake_recovery_us) {
    *wake_recovery_us = rec_us;
  }

  /* Wake-up sequence:
   * 12. Wake external PSRAM and verify SPI/QPI interface ID */
  int psram_wake_rc = psram_wake();
  if (psram_wake_rc != 0) {
    battery_monitor_resume();
    LOG_ERR("recorder_sleep: PSRAM wake failed or ID mismatch (%d)", psram_wake_rc);
    s_power_state = REC_PWR_ACTIVE;
    return psram_wake_rc;
  }
  int post_wake_rc = post_wake_check();
  battery_monitor_resume();
  if (post_wake_rc != 0) {
    LOG_ERR("recorder_sleep: post_wake_check failed (%d)", post_wake_rc);
  }

  /* 13. Snapshot post-wake RTC calendar state and verify calendar advancement */
  struct rtc_calendar_time post_cal;
  int64_t post_epoch = 0;
  if (rtc_calendar_get(&post_cal) == 0 && post_cal.valid) {
    post_epoch = scheduler_calendar_to_epoch(&post_cal);
  }
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  rtc_calendar_log_snapshot("POST_WAKE", pre_epoch + seconds);
  if (res == 0 && post_epoch > pre_epoch) {
    int64_t cal_elapsed = post_epoch - pre_epoch;
    printk("[TEST] RTC_CALENDAR_ADVANCED: elapsed=%lld s\n", (long long)cal_elapsed);
  } else if (res == 0 && pre_epoch > 0 && post_epoch <= pre_epoch) {
    printk("[TEST] RTC clock/calendar inconsistency: WUT expired but calendar did not advance (pre=%lld, post=%lld)\n",
           (long long)pre_epoch, (long long)post_epoch);
  }
#endif

  s_shutdown_stage = SHUTDOWN_STAGE_COMPLETE;
  s_power_state = REC_PWR_ACTIVE;

  if (res == -EINTR) {
    LOG_INF("Stop 2 sleep aborted by console request after %llu ms", elapsed_ms);
    return -EINTR;
  }
  if (res != 0) {
    LOG_ERR("Stop 2 sleep returned error: %d", res);
    return res;
  }

  LOG_INF("Woke from STM32L476 Stop 2 mode via RTC Wakeup Timer (elapsed: %llu ms, wake recovery overhead: %u us)",
          elapsed_ms, rec_us);
  return 0;
}

void recorder_get_active_progress(uint64_t *samples_written, uint64_t *expected_samples) {
  if (s_active_stats_ptr) {
    if (samples_written) *samples_written = s_active_stats_ptr->samples_written;
    if (expected_samples) *expected_samples = s_active_stats_ptr->expected_samples;
  } else {
    if (samples_written) *samples_written = 0;
    if (expected_samples) *expected_samples = 0;
  }
}

void recorder_get_active_file(char *out_buf, size_t max_len) {
  if (!out_buf || max_len == 0) return;
  snprintf(out_buf, max_len, "%s", s_active_file_path);
}

int recorder_wait_idle(uint32_t timeout_ms) {
  int64_t start = k_uptime_get();
  while (recorder_active) {
    if (timeout_ms != UINT32_MAX && (uint32_t)(k_uptime_get() - start) > timeout_ms) {
      return -ETIMEDOUT;
    }
    k_sleep(K_MSEC(10));
  }
  return 0;
}

int recorder_record(uint32_t seconds, struct rec_stats *out) {
  return recorder_record_session(seconds, 1, REC_MODE_NORMAL, out);
}

int recorder_record_cycle(uint32_t seconds, uint32_t cycle,
                          struct rec_stats *out) {
  return recorder_record_session(seconds, cycle, REC_MODE_NORMAL, out);
}

int recorder_record_session(uint32_t seconds, uint32_t cycle,
                            enum recorder_mode mode, struct rec_stats *out) {
  ARG_UNUSED(cycle);
  struct rec_stats st;
  memset(&st, 0, sizeof(st));
  if (out) {
    memset(out, 0, sizeof(*out));
  }
  recorder_abort_requested = false;
  s_recorder_stop_reason = RECORDER_STOP_NONE;
  s_recorder_last_error = 0;
  s_recorder_last_sd_error = 0;
  s_storage_recovery_result = -EAGAIN; /* no recovery attempted this session */
  recorder_active = true;
  s_active_stats_ptr = &st;
  s_active_file_path[0] = '\0';
  static uint32_t s_session_no_audio_recoveries = 0;
  s_session_no_audio_recoveries = 0;

  bool apply_dsp = true;
  const char *mode_str = "FILTERED";

  switch (mode) {
  case REC_MODE_RAW:
    apply_dsp = false;
    mode_str = "RAW (Unfiltered ADC PCM)";
    break;
  case REC_MODE_NORMAL:
  case REC_MODE_FILTERED:
  default:
    apply_dsp = true;
    mode_str = "PRODUCTION FILTERED";
    break;
  }

  st.dsp_mode_str = mode_str;
  st.digital_gain = dsp_get_digital_gain();
  dsp_reset();
  dsp_reset_stats(audio_in_rate(), BLOCK_SAMPLES);
  dsp_reset_all_clipping_counts();
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[PHASE 3] DSP initialized (%s, gain %.2fx)\n", st.dsp_mode_str,
         (double)st.digital_gain);
#endif

  /* Keep the file object alive if a blocked writer outlives this call. */
  static struct fs_file_t f;
  static uint8_t hdr[WAV_HEADER_BYTES] __aligned(4);
  int ret = 0;
  int err = 0;

  if (!mounted) {
    led_indicator_set(LED_IND_ERROR);
    st.err = -ENODEV;
    s_recorder_last_error = -ENODEV;
    s_recorder_last_sd_error = -ENODEV;
    recorder_active = false;
    if (out)
      *out = st;
    return -ENODEV;
  }
  st.sample_rate = audio_in_rate();
  struct app_settings *cur_settings = main_get_settings();
  if (cur_settings) {
    st.sample_bits = cur_settings->sample_bits ? cur_settings->sample_bits : 16;
    st.channels = cur_settings->channels ? cur_settings->channels : 1;
  } else {
    st.sample_bits = 16;
    st.channels = 1;
  }

  uint32_t bytes_per_frame = (uint32_t)st.channels * ((uint32_t)st.sample_bits / 8U);
  st.expected_samples = (uint64_t)seconds * st.sample_rate;
  const uint64_t want_blocks =
      (st.expected_samples + BLOCK_SAMPLES - 1) / BLOCK_SAMPLES;
  st.expected_blocks = (uint32_t)want_blocks;
  uint64_t exact_bytes = st.expected_samples * bytes_per_frame;
  if (st.sample_bits == 16 && st.channels == 1) {
    exact_bytes = st.expected_samples * SAMPLE_BYTES;
  }
  const uint64_t prealloc_bytes = want_blocks * (BLOCK_SAMPLES * bytes_per_frame);
  ARG_UNUSED(prealloc_bytes);

  bool rtc_valid = false;
  ret = generate_recording_filename(st.file, sizeof(st.file), &rtc_valid);
  st.rtc_time_valid = rtc_valid;
  if (rtc_valid) {
    struct rtc_calendar_time tstart;
    if (rtc_calendar_get(&tstart) == 0) {
      rtc_calendar_format(&tstart, st.start_time_str,
                          sizeof(st.start_time_str));
    } else {
      snprintf(st.start_time_str, sizeof(st.start_time_str), "[RTC READ ERR]");
    }
  } else {
    snprintf(st.start_time_str, sizeof(st.start_time_str), "[RTC UNSET]");
  }
  if (ret) {
    led_indicator_set(LED_IND_ERROR);
    st.file[0] = '\0';
    st.err = ret;
    st.sd_write_error = ret;
    if (st.first_error_stage[0] == '\0') {
      strncpy(st.first_error_stage, "FILE_OPEN_CREATE", sizeof(st.first_error_stage) - 1);
    }
    if (out)
      *out = st;
    return ret;
  }
  fs_file_t_init(&f);
  /* Firmware has one recording owner. Recheck immediately before open; the
   * FatFs VFS does not expose O_EXCL, and CREATE alone opens existing files. */
  struct fs_dirent existing;
  ret = fs_stat(st.file, &existing);
  if (ret == 0) {
    ret = -EEXIST;
  } else if (ret == -ENOENT) {
    ret = fs_open(&f, st.file, FS_O_CREATE | FS_O_WRITE);
  }
  bool file_opened = (ret == 0);
  if (ret) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    log_sdmmc_stage_diagnostics("FILE_OPEN_CREATE", ret, 0, 0);
#endif
    LOG_ERR("open %s: %d", st.file, ret);
    led_indicator_set(LED_IND_ERROR);
    st.file[0] = '\0';
    st.err = ret;
    st.sd_write_error = ret;
    if (st.first_error_stage[0] == '\0') {
      strncpy(st.first_error_stage, "FILE_OPEN_CREATE", sizeof(st.first_error_stage) - 1);
    }
    s_recorder_last_error = ret;
    s_recorder_last_sd_error = ret;
    s_active_stats_ptr = NULL;
    s_active_file_path[0] = '\0';
  static uint32_t s_session_no_audio_recoveries = 0;
  s_session_no_audio_recoveries = 0;
    recorder_active = false;
    if (out)
      *out = st;
    return ret;
  }
  snprintf(s_active_file_path, sizeof(s_active_file_path), "%s", st.file);

  /*
   * File pre-allocation handling:
   * Zephyr's VFS fatfs_seek rejects seeking past EOF with -EINVAL (-22).
   * If CONFIG_AUDIO_PREALLOCATE_FILE is enabled and FatFs native API f_expand
   * is compiled, f_expand() is used on the empty file. Otherwise, sequential
   * dynamic writes are used without seeking past EOF.
   */
#if defined(CONFIG_AUDIO_PREALLOCATE_FILE) &&                                  \
    defined(CONFIG_FS_FATFS_EXTRA_NATIVE_API)
  if (prealloc_bytes > 0) {
    FIL *fil = (FIL *)f.filep;
    uint64_t t_pre0 = k_cycle_get_64();
    FRESULT f_res =
        f_expand(fil, (FSIZE_t)(WAV_HEADER_BYTES + prealloc_bytes), 1);
    if (f_res != FR_OK) {
      /* Fallback to non-contiguous pre-allocation if fragmented */
      f_res = f_expand(fil, (FSIZE_t)(WAV_HEADER_BYTES + prealloc_bytes), 0);
    }
    uint32_t pre_ms =
        (uint32_t)(k_cyc_to_us_floor64(k_cycle_get_64() - t_pre0) / 1000ULL);
    if (f_res == FR_OK) {
      LOG_INF("Pre-allocated %llu KB via FatFs f_expand in %u ms",
              (WAV_HEADER_BYTES + prealloc_bytes) >> 10, pre_ms);
    } else {
      LOG_WRN("f_expand returned %d (card fragmented or full); continuing with "
              "dynamic writes",
              (int)f_res);
    }
  } else {
    LOG_INF("Continuous recording session; using sequential dynamic writes");
  }
#else
  LOG_INF("File pre-allocation disabled; using sequential dynamic writes");
#endif

  wav_fill_header(hdr, st.sample_rate, st.channels, st.sample_bits, 0);
  ret = sd_write_all(&f, hdr, WAV_HEADER_BYTES);
  if (ret != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    log_sdmmc_stage_diagnostics("INITIAL_HEADER_WRITE", ret < 0 ? ret : -EIO, 0, WAV_HEADER_BYTES);
#endif
    led_indicator_set(LED_IND_ERROR);
    err = ret < 0 ? ret : -EIO;
    st.err = err;
    st.sd_write_error = err;
    goto cleanup;
  }
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[PHASE 4] WAV file opened: %s (header %u bytes written)\n", st.file,
         WAV_HEADER_BYTES);
#endif

  drops = 0;
  dma_alloc_drops = 0;
  queue_submit_drops = 0;
  sram_writer_drops = 0;
  high_water = 0;
  audio_queue_high_water = 0;
  sd_queue_high_water = 0;
  atomic_set(&s_drops, 0);
  atomic_set(&s_dma_alloc_drops, 0);
  atomic_set(&s_queue_submit_drops, 0);
  atomic_set(&s_sram_writer_drops, 0);
  atomic_set(&s_high_water, 0);
  atomic_set(&s_audio_queue_high_water, 0);
  atomic_set(&s_sd_queue_high_water, 0);
  writer_stats_reset();
  drain_queue();

  bool use_psram = (g_storage_pipeline_mode != STORAGE_PIPELINE_DIRECT_SRAM) &&
                   g_psram_staging_enabled && psram_staging_is_active() &&
                   psram_is_present() && psram_is_quad_mode();
  if (g_psram_staging_enabled && !use_psram && g_storage_pipeline_mode != STORAGE_PIPELINE_DIRECT_SRAM) {
    LOG_WRN("PSRAM staging disabled: Quad I/O unverified; falling back to direct SRAM audio queue");
  }
  st.pipeline_mode = (uint8_t)(use_psram ? g_storage_pipeline_mode : STORAGE_PIPELINE_DIRECT_SRAM);
  if (use_psram) {
    psram_staging_reset();
    st.psram_used = true;
    st.psram_staging_capacity = psram_staging_capacity();
    st.sd_batch_bytes = psram_staging_batch_size();

    /* Dedicated SD writer bounce & ping-pong buffers (never steals from real-time acquisition pool) */
    /* Dedicated bounce buffer in SRAM0 avoids pool borrowing */
    /* k_mem_slab_alloc(&pool, &sd_bounce_buf, K_NO_WAIT) */
    sd_bounce_buf = s_sd_writer_buf;
    s_pingpong[0].buf = (uint8_t *)sd_bounce_buf;
    s_pingpong[0].state = PINGPONG_BUF_FREE;
    s_pingpong[0].bytes = 0;
    s_pingpong[1].buf = s_pingpong_buf_b;
    s_pingpong[1].state = PINGPONG_BUF_FREE;
    s_pingpong[1].bytes = 0;
  } else {
    st.psram_used = false;
    st.psram_staging_capacity = 0;
    st.sd_batch_bytes = 0;
    sd_bounce_buf = NULL;
    s_pingpong[0].buf = NULL;
    s_pingpong[0].state = PINGPONG_BUF_FREE;
    s_pingpong[0].bytes = 0;
    s_pingpong[1].buf = NULL;
    s_pingpong[1].state = PINGPONG_BUF_FREE;
    s_pingpong[1].bytes = 0;
  }

  active_use_psram = use_psram;
  sd_file_ptr = &f;
  sd_writer_flush_req = false;
  k_sem_reset(&sd_writer_done_sem);
  sd_writer_write_us_sum = 0;
  sd_writer_write_count = 0;
  sd_writer_worst_write_us = 0;
  sd_writer_peek_us_sum = 0;
  sd_writer_peek_count = 0;
  sd_writer_worst_peek_us = 0;
  sd_writer_commit_us_sum = 0;
  sd_writer_batch_wall_us_sum = 0;
  sd_writer_worst_batch_ms = 0;
  sd_writer_bytes_written = 0;
  sd_writer_failed_bytes = 0;
  sd_writer_batch_count = 0;
  sd_writer_overlap_us_sum = 0;
  sd_writer_swap_count = 0;
  sd_writer_err = 0;
  s_active_stats_ptr = &st;

  /* Drain any stale blocks from sd_block_q if non-PSRAM mode */
  struct sd_block_msg stale_msg;
  while (k_msgq_get(&sd_block_q, &stale_msg, K_NO_WAIT) == 0) {
    k_mem_slab_free(&pool, stale_msg.blk);
  }

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  if (audio_format_is_experimental(st.sample_rate, st.channels, st.sample_bits)) {
    uint32_t est_rate_bps = st.sample_rate * bytes_per_frame;
    printk("\n================================================================================\n");
    printk("WARNING: Experimental audio configuration active\n");
    printk("  Not validated for production hardware: for bench and platform validation only\n");
    printk("  Rate: %u Hz (Requested: %u Hz) | Channels: %u | Bits: %u\n",
           st.sample_rate, audio_in_requested_rate(), st.channels, st.sample_bits);
    printk("  Data rate: %u bytes/sec (%u KB/s) | Target platform: STM32U585\n",
           est_rate_bps, est_rate_bps / 1000);
    if (st.channels == 2) {
      if (audio_in_get_capture_source() == AUDIO_SRC_SYNTHETIC) {
        printk("  Channel mode: Synthetic dual-tone (Left=1 kHz sine, Right=2 kHz sine)\n");
      } else {
        printk("  Channel mode: Duplicated Mono container (2 channels, single ADC input)\n");
      }
    }
    if (st.sample_bits == 32) {
      printk("  PCM format: 32-bit signed PCM (container scaled via audio_pcm16_to_pcm32, 4 bytes/sample)\n");
    }
    printk("================================================================================\n\n");
  }
#endif

  const uint32_t buffer_ms =
      (uint32_t)((uint64_t)NUM_BLOCKS * BLOCK_SAMPLES * 1000 / st.sample_rate);
  const char *ch_str = (st.channels == 2) ? "stereo" : "mono";
  LOG_INF("recording %u s to %s at %u Hz, %u-bit %s (expect %llu samples, %u "
          "blocks, %llu bytes, pool %u = %u ms buffer)",
          seconds, st.file, st.sample_rate, st.sample_bits, ch_str,
          st.expected_samples, st.expected_blocks, exact_bytes, NUM_BLOCKS,
          buffer_ms);

  ret = audio_in_start();
  if (ret) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[STAGE_DIAG] Stage: AUDIO_ACQUISITION_START | Error: %d | Rate: %u Hz\n", ret, st.sample_rate);
#endif
    LOG_ERR("audio_in_start failed: %d", ret);
    led_indicator_set(LED_IND_ERROR);
    recorder_active = false;
    err = ret;
    st.err = err;
    st.sd_write_error = err;
    if (st.first_error_stage[0] == '\0') {
      strncpy(st.first_error_stage, "FILE_OPEN_CREATE", sizeof(st.first_error_stage) - 1);
    }
    goto cleanup;
  }

  led_indicator_set(LED_IND_RECORDING);
  s_power_state = REC_PWR_RECORDING;

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[TEST] CAPTURE_STARTED: target_samples=%llu, rate=%u Hz\n",
         (unsigned long long)st.expected_samples, st.sample_rate);
#endif

  const uint64_t t_start_cyc = k_cycle_get_64();
  const uint64_t capture_deadline_cyc = t_start_cyc +
      k_us_to_cyc_ceil64((uint64_t)seconds * 1000000ULL + 100000ULL);
  uint64_t t_first_block_cyc = 0;
  uint64_t t_last_block_cyc = 0;
  err = 0;
  uint64_t samples_remaining = st.expected_samples;
  uint32_t consecutive_drops = 0;
  s_session_gap_count = 0;
  s_backpressure_slowdown = false;
  memset(s_session_gaps, 0, sizeof(s_session_gaps));
  st.gap_count = 0;
  st.consecutive_drop_abort_count = 0;
  st.slowdown_active = false;
  st.slowdown_events = 0;

  while (samples_remaining > 0) {
    if (recorder_abort_requested) {
      LOG_INF("Stop requested: finalizing recorded frames early (%llu samples "
              "written)",
              st.samples_written);
      break;
    }
    uint64_t source_samples = audio_in_get_source_samples();
    if (source_samples >= st.expected_samples || k_cycle_get_64() >= capture_deadline_cyc) {
      if (k_msgq_num_used_get(&filled_q) == 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
        printk("[TEST] CAPTURE_TARGET_REACHED\n");
#endif
        break;
      }
    }
    uint8_t buf_id = 0xFF;
    void *blk = NULL;

    /* a healthy source delivers a block every ~10 ms; 2 s means it died */
    if (k_msgq_get(&filled_q, &buf_id, K_SECONDS(2)) != 0) {
      LOG_ERR("no audio for 2 s");
      if (s_session_no_audio_recoveries >= 3U) {
        LOG_ERR("Maximum no-audio recovery retries (3) exceeded; aborting session");
        err = -ETIMEDOUT;
        if (st.first_error_stage[0] == '\0') {
          strncpy(st.first_error_stage, "audio_in_timeout", sizeof(st.first_error_stage) - 1);
          st.first_error_stage[sizeof(st.first_error_stage) - 1] = '\0';
        }
        break;
      }
      s_session_no_audio_recoveries++;
      struct audio_dma_stats stalled = {0};
      audio_in_get_dma_stats(&stalled);
      LOG_ERR("ADC/DMA status: callbacks=%u errors=%u overruns=%u missed=%u",
              stalled.callbacks, stalled.callback_errors, stalled.adc_overruns,
              stalled.events_missed);
      bool recovered = false;
      for (uint32_t attempt = 0; attempt < 2 && !recovered; attempt++) {
        st.adc_dma_recovery_attempts++;
        (void)audio_in_stop();
        int restart_rc = audio_in_start();
        if (restart_rc == 0 && k_msgq_get(&filled_q, &buf_id, K_SECONDS(2)) == 0) {
          recovered = true;
          LOG_WRN("ADC/DMA acquisition resumed after attempt %u", attempt + 1U);
        } else {
          st.adc_dma_recovery_failures++;
          LOG_ERR("ADC/DMA recovery attempt %u failed (%d)", attempt + 1U, restart_rc);
        }
      }
      if (!recovered) {
        err = -ETIMEDOUT;
        if (st.first_error_stage[0] == '\0') {
          strncpy(st.first_error_stage, "audio_in_timeout", sizeof(st.first_error_stage) - 1);
          st.first_error_stage[sizeof(st.first_error_stage) - 1] = '\0';
        }
        break;
      }
    }
    recorder_buf_set_state(buf_id, BUF_OWNER_DSP);
    blk = recorder_buf_id_to_ptr(buf_id);
    st.blocks_processed++;
    uint64_t t_block_arrival = k_cycle_get_64();
    if (st.blocks_processed == 1) {
      t_first_block_cyc = t_block_arrival;
    }
    t_last_block_cyc = t_block_arrival;

    /* Determine valid samples in this block (clamp last partial block) */
    uint32_t samples_in_block = (samples_remaining > BLOCK_SAMPLES)
                                    ? BLOCK_SAMPLES
                                    : (uint32_t)samples_remaining;
    if (samples_in_block < BLOCK_SAMPLES) {
      st.boundary_discarded_samples += (BLOCK_SAMPLES - samples_in_block);
    }

    if (use_psram) {
      uint32_t occupied = psram_staging_occupied();
      uint32_t cap = psram_staging_capacity();
      if (!s_backpressure_slowdown && (occupied >= (cap * 3U / 4U))) {
        s_backpressure_slowdown = true;
        st.slowdown_events++;
        LOG_WRN("[BACK-PRESSURE] PSRAM staging fill > 75%% (%u KB / %u KB), slowdown engaged",
                occupied >> 10, cap >> 10);
        printk("[BACK-PRESSURE] SLOWDOWN_ENGAGED: occupied=%u KB\n", occupied >> 10);
      } else if (s_backpressure_slowdown && (occupied <= (cap / 2U))) {
        s_backpressure_slowdown = false;
        LOG_INF("[BACK-PRESSURE] PSRAM staging fill < 50%% (%u KB / %u KB), slowdown cleared",
                occupied >> 10, cap >> 10);
        printk("[BACK-PRESSURE] SLOWDOWN_CLEARED: occupied=%u KB\n", occupied >> 10);
      }
      st.slowdown_active = s_backpressure_slowdown;
      if (s_backpressure_slowdown) {
        k_sem_give(&sd_writer_sem);
        if (k_mem_slab_num_free_get(&pool) > 2) {
          k_yield();
        }
      }
    }

    if (apply_dsp) {
#if CONFIG_AUDIO_SAMPLE_BITS == 32
      dsp_process_i32((int32_t *)blk, samples_in_block);
#else
      dsp_process_i16((int16_t *)blk, samples_in_block);
#endif
    }

    if (st.sample_bits == 16 && st.channels == 1) {
      uint32_t bytes_to_write = samples_in_block * SAMPLE_BYTES;
      if (use_psram) {
        int p_ret = psram_staging_push(blk, bytes_to_write, samples_in_block);
        if (p_ret == -ENOBUFS) {
          k_sem_give(&sd_writer_sem);
          uint32_t free_pool = k_mem_slab_num_free_get(&pool);
          if (free_pool > 2) {
            /* Headroom exists in DMA pool: wait boundedly up to 5 ms for writer to release a batch buffer */
            if (psram_staging_wait_free(5) == 0) {
              p_ret = psram_staging_push(blk, bytes_to_write, samples_in_block);
            }
          }
        }
        recorder_buf_set_state(buf_id, BUF_OWNER_FREE);
        k_mem_slab_free(&pool, blk);

        if (p_ret == -ENOBUFS) {
          /* PSRAM staging buffer is full - Drop and continue policy */
          st.psram_full_events++;
          st.psram_dropped_samples += samples_in_block;
          st.samples_dropped += samples_in_block;
          st.blocks_dropped++;
          consecutive_drops++;
          uint64_t gap_idx = st.expected_samples - samples_remaining;
          recorder_record_gap(gap_idx, samples_in_block);
          samples_remaining -= samples_in_block;

          if (st.first_error_stage[0] == '\0') {
            strncpy(st.first_error_stage, "PSRAM_STAGING_FULL", sizeof(st.first_error_stage) - 1);
          }
          static uint32_t last_full_warn_ms = 0;
          uint32_t now_ms = k_uptime_get_32();
          if (now_ms - last_full_warn_ms > 1000) {
            last_full_warn_ms = now_ms;
            LOG_WRN("PSRAM staging buffer full! Occupancy: %u KB / %u KB. "
                    "Dropping audio block (%u samples, consecutive: %u)",
                    psram_staging_occupied() >> 10,
                    psram_staging_capacity() >> 10, samples_in_block, consecutive_drops);
          }
          /* Abort capture only after consecutive drop threshold */
          if (consecutive_drops >= RECORDER_MAX_CONSECUTIVE_DROPS) {
            LOG_ERR("PSRAM staging full: %u consecutive drops exceeded limit %u; aborting capture",
                    consecutive_drops, RECORDER_MAX_CONSECUTIVE_DROPS);
            st.consecutive_drop_abort_count++;
            err = -ENOBUFS;
            break;
          }
          continue;
        } else if (p_ret != 0) {
          LOG_ERR("PSRAM push fatal error: %d", p_ret);
          err = p_ret;
          if (st.first_error_stage[0] == '\0') {
            strncpy(st.first_error_stage, "PSRAM_STAGING_FULL", sizeof(st.first_error_stage) - 1);
          }
          break;
        }

        consecutive_drops = 0;

        if (sd_writer_err != 0) {
          LOG_ERR("SD writer error detected in recording loop: %d",
                  sd_writer_err);
          err = sd_writer_err;
          if (st.first_error_stage[0] == '\0') {
#if defined(CONFIG_SOC_SERIES_STM32L4X)
            if (SDMMC1->STA & (SDMMC_STA_DCRCFAIL | SDMMC_STA_CCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_CTIMEOUT | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR)) {
              strncpy(st.first_error_stage, "SDMMC_TRANSFER", sizeof(st.first_error_stage) - 1);
            } else
#endif
            {
              strncpy(st.first_error_stage, "FS_WRITE", sizeof(st.first_error_stage) - 1);
            }
          }
          break;
        }

        /* Trigger SD writer if batch threshold reached */
        if (psram_staging_occupied() >= psram_staging_batch_size()) {
          k_sem_give(&sd_writer_sem);
        }

        if (samples_in_block == BLOCK_SAMPLES) {
          st.full_blocks_written++;
        } else {
          st.partial_blocks_written++;
          st.partial_block_samples = samples_in_block;
        }
        st.blocks_written++;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
        if (st.blocks_written == 1) {
          printk(
              "[PHASE 8] First block accepted and buffered in PSRAM (%u bytes)\n",
              bytes_to_write);
        }
#endif
        st.samples_accepted += samples_in_block;
        samples_remaining -= samples_in_block;
      } else {
        /* Decoupled non-PSRAM storage path: Enqueue processed block to
         * sd_writer_thread */
        recorder_buf_set_state(buf_id, BUF_OWNER_WRITING);
        struct sd_block_msg s_msg = {
            .buf_id = buf_id,
            .blk = blk,
            .bytes = bytes_to_write,
            .samples = samples_in_block,
        };

        int q_ret = k_msgq_put(&sd_block_q, &s_msg, K_NO_WAIT);
        if (q_ret == 0) {
          uint32_t sd_queued = k_msgq_num_used_get(&sd_block_q);
          recorder_update_high_water(&s_sd_queue_high_water, sd_queued, &sd_queue_high_water);
          consecutive_drops = 0;
        } else {
          /* Bounded RAM queue capacity exhausted - Drop and continue policy */
          atomic_inc(&s_sram_writer_drops);
          atomic_inc(&s_drops);
          sram_writer_drops = (uint32_t)atomic_get(&s_sram_writer_drops);
          drops = (uint32_t)atomic_get(&s_drops);
          st.sram_queue_dropped_blocks++;
          st.blocks_dropped++;
          st.samples_dropped += samples_in_block;
          consecutive_drops++;
          uint64_t gap_idx = st.expected_samples - samples_remaining;
          recorder_record_gap(gap_idx, samples_in_block);
          samples_remaining -= samples_in_block;
          if (st.first_error_stage[0] == '\0') {
            strncpy(st.first_error_stage, "DSP_QUEUE_OVERFLOW", sizeof(st.first_error_stage) - 1);
          }
          recorder_buf_set_state(buf_id, BUF_OWNER_FREE);
          k_mem_slab_free(&pool, blk);
          LOG_WRN("Non-PSRAM RAM queue exhausted (%u/%u blocks)! Dropping block "
                  "(%u samples, consecutive: %u).",
                  (unsigned)k_msgq_num_used_get(&sd_block_q), NUM_BLOCKS,
                  samples_in_block, consecutive_drops);
          if (consecutive_drops >= RECORDER_MAX_CONSECUTIVE_DROPS) {
            LOG_ERR("Non-PSRAM RAM queue drops exceeded threshold (%u)! Aborting capture.", consecutive_drops);
            st.consecutive_drop_abort_count++;
            err = -ENOBUFS;
            break;
          }
          continue;
        }

        /* Signal SD writer thread to consume queued block */
        k_sem_give(&sd_writer_sem);

        if (sd_writer_err != 0) {
          LOG_ERR("SD writer error detected in recording loop: %d",
                  sd_writer_err);
          err = sd_writer_err;
          if (st.first_error_stage[0] == '\0') {
            strncpy(st.first_error_stage, "FS_WRITE", sizeof(st.first_error_stage) - 1);
          }
          break;
        }

        if (samples_in_block == BLOCK_SAMPLES) {
          st.full_blocks_written++;
        } else {
          st.partial_blocks_written++;
          st.partial_block_samples = samples_in_block;
        }
        st.blocks_written++;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
        if (st.blocks_written == 1) {
          printk("[PHASE 8] First block accepted and written (%u bytes, queued "
                 "to SD writer)\n",
                 bytes_to_write);
        }
#endif
        st.samples_accepted += samples_in_block;
        samples_remaining -= samples_in_block;
      }
    } else {
      /* Experimental format conversion (32-bit PCM, stereo duplication, synthetic tone) */
      static uint8_t format_chunk_buf[2048] __aligned(4);
      uint32_t chunk_frames = 256;
      bool chunk_failed = false;
      const int16_t *src_s16 = (const int16_t *)blk;

      for (uint32_t off = 0; off < samples_in_block; off += chunk_frames) {
        uint32_t chunk_len = (samples_in_block - off > chunk_frames) ? chunk_frames : (samples_in_block - off);
        uint32_t chunk_bytes = chunk_len * bytes_per_frame;

        if (st.sample_bits == 32 && st.channels == 1) {
          int32_t *dst32 = (int32_t *)format_chunk_buf;
          for (uint32_t k = 0; k < chunk_len; k++) {
            dst32[k] = audio_pcm16_to_pcm32(src_s16[off + k]);
          }
        } else if (st.sample_bits == 16 && st.channels == 2) {
          int16_t *dst16 = (int16_t *)format_chunk_buf;
          if (audio_in_get_capture_source() == AUDIO_SRC_SYNTHETIC) {
            for (uint32_t k = 0; k < chunk_len; k++) {
              uint64_t gframe = st.samples_accepted + off + k;
              float p1 = 2.0f * 3.14159265f * 1000.0f * (float)gframe / (float)st.sample_rate;
              float p2 = 2.0f * 3.14159265f * 2000.0f * (float)gframe / (float)st.sample_rate;
              dst16[2 * k + 0] = (int16_t)(sinf(p1) * 16384.0f);
              dst16[2 * k + 1] = (int16_t)(sinf(p2) * 16384.0f);
            }
          } else {
            /* Duplicated Mono container: copy single ADC channel to both Left and Right */
            for (uint32_t k = 0; k < chunk_len; k++) {
              int16_t s = src_s16[off + k];
              dst16[2 * k + 0] = s;
              dst16[2 * k + 1] = s;
            }
          }
        } else if (st.sample_bits == 32 && st.channels == 2) {
          int32_t *dst32 = (int32_t *)format_chunk_buf;
          if (audio_in_get_capture_source() == AUDIO_SRC_SYNTHETIC) {
            for (uint32_t k = 0; k < chunk_len; k++) {
              uint64_t gframe = st.samples_accepted + off + k;
              float p1 = 2.0f * 3.14159265f * 1000.0f * (float)gframe / (float)st.sample_rate;
              float p2 = 2.0f * 3.14159265f * 2000.0f * (float)gframe / (float)st.sample_rate;
              dst32[2 * k + 0] = audio_pcm16_to_pcm32((int16_t)(sinf(p1) * 16384.0f));
              dst32[2 * k + 1] = audio_pcm16_to_pcm32((int16_t)(sinf(p2) * 16384.0f));
            }
          } else {
            /* Duplicated Mono container: copy single ADC channel to both Left and Right in 32-bit container */
            for (uint32_t k = 0; k < chunk_len; k++) {
              int32_t s32 = audio_pcm16_to_pcm32(src_s16[off + k]);
              dst32[2 * k + 0] = s32;
              dst32[2 * k + 1] = s32;
            }
          }
        }

        if (use_psram) {
          int p_ret = psram_staging_push(format_chunk_buf, chunk_bytes, chunk_len);
          if (p_ret == -ENOBUFS) {
            k_sem_give(&sd_writer_sem);
            uint32_t free_pool = k_mem_slab_num_free_get(&pool);
            if (free_pool > 2) {
              if (psram_staging_wait_free(5) == 0) {
                p_ret = psram_staging_push(format_chunk_buf, chunk_bytes, chunk_len);
              }
            }
          }
          if (p_ret == -ENOBUFS) {
            st.psram_full_events++;
            st.psram_dropped_samples += (samples_in_block - off);
            st.samples_dropped += (samples_in_block - off);
            st.blocks_dropped++;
            consecutive_drops++;
            uint64_t gap_idx = st.expected_samples - samples_remaining;
            recorder_record_gap(gap_idx, samples_in_block - off);
            samples_remaining -= samples_in_block;
            if (consecutive_drops >= RECORDER_MAX_CONSECUTIVE_DROPS) {
              LOG_ERR("Format conversion PSRAM staging full: %u drops exceeded limit; aborting", consecutive_drops);
              st.consecutive_drop_abort_count++;
              err = -ENOBUFS;
              chunk_failed = true;
              break;
            }
            chunk_failed = false;
            break;
          } else if (p_ret != 0) {
            LOG_ERR("PSRAM push fatal error: %d", p_ret);
            err = p_ret;
            chunk_failed = true;
            break;
          }
        } else {
          ssize_t w = fs_write(&f, format_chunk_buf, chunk_bytes);
          if (w != (ssize_t)chunk_bytes) {
            err = (w < 0) ? (int)w : -EIO;
            sd_writer_err = err;
            chunk_failed = true;
            break;
          }
          sd_writer_bytes_written += chunk_bytes;
          sd_writer_write_count++;
        }
      }

      k_mem_slab_free(&pool, blk);
      if (chunk_failed) {
        break;
      }
      if (use_psram) {
        if (sd_writer_err != 0) {
          LOG_ERR("SD writer error detected in recording loop: %d", sd_writer_err);
          err = sd_writer_err;
          break;
        }
        if (psram_staging_occupied() >= psram_staging_batch_size()) {
          k_sem_give(&sd_writer_sem);
        }
      }
      if (samples_in_block == BLOCK_SAMPLES) {
        st.full_blocks_written++;
      } else {
        st.partial_blocks_written++;
        st.partial_block_samples = samples_in_block;
      }
      st.blocks_written++;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      if (st.blocks_written == 1) {
        printk("[PHASE 8] First experimental block formatted and buffered (%u bytes)\n",
               samples_in_block * bytes_per_frame);
      }
#endif
      st.samples_accepted += samples_in_block;
      samples_remaining -= samples_in_block;
    }
  }

  const uint64_t t_stop_cyc = k_cycle_get_64();
  uint64_t wall_cyc =
      (t_stop_cyc >= t_start_cyc) ? (t_stop_cyc - t_start_cyc) : 0;
  uint64_t session_wall_us = k_cyc_to_us_floor64(wall_cyc);

  s_power_state = REC_PWR_FINALIZING;
  int stop_rc = audio_in_stop();
  if (stop_rc != 0 && err == 0) {
    err = stop_rc;
    if (st.first_error_stage[0] == '\0') {
      strncpy(st.first_error_stage, "audio_in_stop", sizeof(st.first_error_stage) - 1);
      st.first_error_stage[sizeof(st.first_error_stage) - 1] = '\0';
    }
  }

  uint32_t drained_blocks = drain_queue();
  st.queue_drained_blocks = drained_blocks;
  st.queue_drained_samples = drained_blocks * BLOCK_SAMPLES;
  st.boundary_discarded_samples += (uint64_t)drained_blocks * BLOCK_SAMPLES;
  st.sram_queue_dropped_blocks = sram_writer_drops;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[TEST] AUDIO_QUEUES_EMPTY\n");
#endif

  struct audio_dma_stats dma_stats = {0};
  audio_in_get_dma_stats(&dma_stats);
  st.adc_overruns = dma_stats.adc_overruns;
  st.dma_errors = dma_stats.callback_errors;
  st.acquisition_events_missed = dma_stats.events_missed;
  st.acquisition_samples_processed = dma_stats.samples_processed;
  st.acquisition_samples_lost = dma_stats.samples_lost;
  st.acquisition_samples_accounted = dma_stats.samples_accounted;
  st.hardware_source_samples = dma_stats.samples_acquired;
  st.copied_samples = dma_stats.samples_accounted;
  st.processed_samples = dma_stats.samples_processed;
  st.boundary_discarded_samples += dma_stats.boundary_discarded_samples;

  st.dma_interval_min_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_min_cycles);
  st.dma_interval_avg_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_avg_cycles);
  st.dma_interval_max_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_max_cycles);
  st.dma_dropped_blocks = (dma_stats.samples_lost + BLOCK_SAMPLES - 1U) /
                          BLOCK_SAMPLES;
  if ((dma_stats.events_missed != 0 ||
       dma_stats.callback_errors != 0 || dma_stats.adc_overruns != 0) && err == 0) {
    err = -EIO;
    if (st.first_error_stage[0] == '\0') {
      strncpy(st.first_error_stage, "DMA_ACQUISITION", sizeof(st.first_error_stage) - 1);
      st.first_error_stage[sizeof(st.first_error_stage) - 1] = '\0';
    }
  }
  if (dma_stats.samples_lost != 0 && err == 0) {
    err = -EIO;
    if (st.first_error_stage[0] == '\0') {
      strncpy(st.first_error_stage, "DMA_ACQUISITION", sizeof(st.first_error_stage) - 1);
      st.first_error_stage[sizeof(st.first_error_stage) - 1] = '\0';
    }
  }

  /* Signal SD writer thread to flush any remaining audio bytes/blocks */
  if (use_psram) {
    psram_staging_flush_publish();
  }
  sd_writer_flush_req = true;
  k_sem_give(&sd_writer_sem);
  uint32_t pending_writer_bytes = use_psram
      ? psram_staging_occupied()
      : (uint32_t)k_msgq_num_used_get(&sd_block_q) * BLOCK_BYTES;
  uint32_t completion_timeout_ms = writer_completion_timeout_ms(pending_writer_bytes);
  if (k_sem_take(&sd_writer_done_sem, K_MSEC(completion_timeout_ms)) != 0) {
    LOG_ERR("SD writer completion timeout after %u ms; closing file and reporting fault",
            completion_timeout_ms);
    st.err = -ETIMEDOUT;
    st.writer_completion_timeouts++;
    s_recorder_last_error = -ETIMEDOUT;
    s_recorder_last_sd_error = -ETIMEDOUT;
    err = -ETIMEDOUT;
    goto cleanup;
  }
  sd_writer_flush_req = false;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("[TEST] PSRAM_FLUSH_COMPLETE\n");
  printk("[TEST] SD_WRITER_COMPLETE\n");
#endif

  /* Reset dedicated bounce & ping-pong buffer pointers */
  sd_bounce_buf = NULL;
  s_pingpong[0].buf = NULL;
  s_pingpong[0].state = PINGPONG_BUF_FREE;
  s_pingpong[0].bytes = 0;
  s_pingpong[1].buf = NULL;
  s_pingpong[1].state = PINGPONG_BUF_FREE;
  s_pingpong[1].bytes = 0;
  sd_file_ptr = NULL;

  if (sd_writer_err != 0 && err == 0) {
    err = sd_writer_err;
  }

  st.bytes_written = sd_writer_bytes_written;
  st.samples_written = (bytes_per_frame > 0) ? (st.bytes_written / bytes_per_frame) : 0;
  st.write_count = sd_writer_write_count;
  st.worst_write_us = sd_writer_worst_write_us;
  st.total_write_us = sd_writer_write_us_sum;
  st.bytes_per_write_avg =
      (sd_writer_write_count > 0)
          ? (uint32_t)(sd_writer_bytes_written / sd_writer_write_count)
          : 0;
  st.sd_write_error = sd_writer_err;
  st.sd_failed_samples = (bytes_per_frame > 0) ? (sd_writer_failed_bytes / bytes_per_frame) : 0;

  if (use_psram) {
    struct psram_staging_stats pstat;
    psram_staging_get_stats(&pstat);
    st.psram_high_water_bytes = pstat.high_water_bytes;
    st.psram_full_events = pstat.full_buffer_events;
    st.psram_dropped_samples += pstat.dropped_samples;
  }

  uint32_t psram_lost_blocks = st.psram_dropped_samples / BLOCK_SAMPLES;
  if ((st.psram_dropped_samples % BLOCK_SAMPLES) != 0) {
    psram_lost_blocks++;
  }
  uint32_t sd_failed_blocks = (uint32_t)((st.sd_failed_samples +
                               BLOCK_SAMPLES - 1U) / BLOCK_SAMPLES);
  st.blocks_dropped = st.dma_dropped_blocks + st.sram_queue_dropped_blocks +
                      psram_lost_blocks + st.queue_drained_blocks +
                      sd_failed_blocks;
  st.samples_dropped = (uint64_t)st.dma_dropped_blocks * BLOCK_SAMPLES +
                       (uint64_t)st.sram_queue_dropped_blocks * BLOCK_SAMPLES +
                       (uint64_t)st.psram_dropped_samples +
                       (uint64_t)st.queue_drained_samples +
                       st.sd_failed_samples;

  rec_compute_timing(&st, sd_writer_write_us_sum, session_wall_us,
                     sd_writer_write_count, sd_writer_worst_write_us);

  st.sd_writer_wall_us = sd_writer_batch_wall_us_sum;
  st.sd_writer_psram_read_us = sd_writer_peek_us_sum;
  st.sd_writer_active_fs_us = sd_writer_write_us_sum;
  st.sd_writer_commit_us = sd_writer_commit_us_sum;
  uint64_t busy_total = st.sd_writer_psram_read_us + st.sd_writer_active_fs_us + st.sd_writer_commit_us;
  if (st.sd_writer_wall_us > busy_total) {
    st.sd_writer_starve_us = st.sd_writer_wall_us - busy_total;
  } else {
    st.sd_writer_starve_us = 0;
  }
  if (sd_writer_write_us_sum > 0) {
    st.sd_active_throughput_kb_s = (uint32_t)(((uint64_t)sd_writer_bytes_written * 1000000ULL) / (sd_writer_write_us_sum * 1024ULL));
  } else {
    st.sd_active_throughput_kb_s = 0;
  }
  if (sd_writer_batch_wall_us_sum > 0) {
    st.sd_aggregate_throughput_kb_s = (uint32_t)(((uint64_t)sd_writer_bytes_written * 1000000ULL) / (sd_writer_batch_wall_us_sum * 1024ULL));
  } else {
    st.sd_aggregate_throughput_kb_s = 0;
  }
  st.pingpong_overlap_us = sd_writer_overlap_us_sum;
  st.pingpong_swaps = sd_writer_swap_count;
  st.psram_qspi_txn_count = (uint32_t)((sd_writer_bytes_written + 1023ULL) / 1024ULL);
  if (sd_writer_peek_us_sum > 0) {
    st.psram_read_throughput_kb_s = (uint32_t)(((uint64_t)sd_writer_bytes_written * 1000000ULL) / (sd_writer_peek_us_sum * 1024ULL));
  } else {
    st.psram_read_throughput_kb_s = 0;
  }
  if (session_wall_us > 0) {
    st.psram_write_throughput_kb_s = (uint32_t)(((uint64_t)st.bytes_written * 1000000ULL) / (session_wall_us * 1024ULL));
  } else {
    st.psram_write_throughput_kb_s = 0;
  }

  /* Capture RTC session end timestamp */
  if (st.rtc_time_valid) {
    struct rtc_calendar_time tend;
    if (rtc_calendar_get(&tend) == 0) {
      rtc_calendar_format(&tend, st.end_time_str, sizeof(st.end_time_str));
    } else {
      snprintf(st.end_time_str, sizeof(st.end_time_str), "%s",
               st.start_time_str);
    }
  } else {
    snprintf(st.end_time_str, sizeof(st.end_time_str), "[RTC UNSET]");
  }

  struct dsp_stats ds;
  dsp_get_stats(&ds);
  st.dsp_avg_us = ds.avg_us;
  st.dsp_worst_us = ds.worst_us;
  st.dsp_total_us = (uint64_t)k_cyc_to_us_floor64(ds.total_cycles);
  st.dsp_overruns = ds.overruns;
  st.dsp_input_clip_count = dsp_get_input_clipping_count();
  st.dsp_filter_overshoot_count = dsp_get_filter_overshoot_count();
  st.dsp_gain_clipping_count = dsp_get_gain_clipping_count();
  st.digital_clipping_count = dsp_get_digital_clipping_count();

  struct audio_diag diag;
  audio_in_get_diag(&diag);
  st.samples_acquired = dma_stats.samples_acquired;
  st.samples_captured = st.samples_written;
  st.adc_clip_low_count = diag.clipping_low_count;
  st.adc_clip_high_count = diag.clipping_high_count;

  /* Residual is only the acquisition count not already classified as written or dropped. */
  uint64_t known_accounted = st.samples_written + st.samples_dropped;
  st.samples_discarded_tail = (st.samples_acquired > known_accounted)
                                  ? (st.samples_acquired - known_accounted)
                                  : 0;

  if (st.blocks_processed > 1 && t_last_block_cyc > t_first_block_cyc) {
    uint64_t span_cyc = t_last_block_cyc - t_first_block_cyc;
    uint64_t span_us = k_cyc_to_us_floor64(span_cyc);
    uint64_t span_samples = (uint64_t)(st.blocks_processed - 1) * BLOCK_SAMPLES;
    if (span_us > 0) {
      st.measured_sample_rate =
          (uint32_t)((span_samples * 1000000ULL) / span_us);
    }
  } else if (st.elapsed_ms > 0) {
    st.measured_sample_rate =
        (uint32_t)((st.samples_written * 1000) / st.elapsed_ms);
  }

  /* finalise: real length into the header, truncate any excess pre-allocated
   * space */
  if (sd_writer_err != 0 || st.sd_write_error != 0) {
    /* Case B: SDMMC or filesystem write failure.
     * The filesystem handle and storage media are in an error state.
     * Do NOT call fs_seek(), fs_write() for a new header, fs_truncate(), or fs_sync()!
     * Attempt one best-effort close only after the writer has relinquished ownership.
     */
    int close_ret = fs_close(&f);
    file_opened = false;
    if (close_ret != 0) {
      LOG_WRN("Best-effort fs_close on failed storage handle returned: %d", close_ret);
    }
    st.wav_verified = false;
    if (err == 0) {
      err = (sd_writer_err != 0) ? sd_writer_err : st.sd_write_error;
    }
  } else {
    /* Writer completion is established here. Finalize a valid partial WAV using
     * only bytes whose writes completed successfully, while preserving err.
     * (Case A: Storage is healthy. Either normal completion or acquisition failure). */
    const uint32_t data_bytes = (uint32_t)st.bytes_written;
    wav_fill_header(hdr, st.sample_rate, st.channels, st.sample_bits, data_bytes);
    int finalize_ret = 0;
    finalize_ret = fs_seek(&f, 0, FS_SEEK_SET);
    if (finalize_ret != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      log_sdmmc_stage_diagnostics("WAV_HEADER_FINAL_SEEK", finalize_ret, 0, 0);
#endif
    } else {
      ssize_t header_written = fs_write(&f, hdr, WAV_HEADER_BYTES);
      if (header_written < 0 || header_written != WAV_HEADER_BYTES) {
        finalize_ret = (header_written < 0) ? (int)header_written : -EIO;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
        log_sdmmc_stage_diagnostics("WAV_HEADER_FINAL_WRITE", finalize_ret, 0, WAV_HEADER_BYTES);
#endif
      }
    }
    if (finalize_ret == 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("[TEST] WAV_HEADER_FINALIZED\n");
#endif
      finalize_ret = fs_truncate(&f, WAV_HEADER_BYTES + data_bytes);
      if (finalize_ret != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
        log_sdmmc_stage_diagnostics("FILE_TRUNCATE", finalize_ret, WAV_HEADER_BYTES + data_bytes, 0);
#endif
      }
    }
    if (finalize_ret == 0) {
      finalize_ret = fs_sync(&f);
      if (finalize_ret != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
        log_sdmmc_stage_diagnostics("FILE_SYNC", finalize_ret, WAV_HEADER_BYTES + data_bytes, 0);
#endif
      }
    }
    int close_ret = fs_close(&f);
    file_opened = false;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] WAV_FILE_CLOSED\n");
#endif
    recorder_write_gap_sidecar(st.file);
    st.gap_count = s_session_gap_count;
    st.slowdown_active = s_backpressure_slowdown;
    if (close_ret != 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      log_sdmmc_stage_diagnostics("FILE_CLOSE", close_ret, WAV_HEADER_BYTES + data_bytes, 0);
#endif
    }
    if (finalize_ret == 0) {
      finalize_ret = close_ret;
    }
    if (finalize_ret != 0 && err == 0) {
      LOG_ERR("WAV finalize/sync/close failed: %d", finalize_ret);
      err = finalize_ret;
      if (st.sd_write_error == 0) st.sd_write_error = finalize_ret;
      if (st.first_error_stage[0] == '\0') {
        strncpy(st.first_error_stage, "WAV_FINALIZATION", sizeof(st.first_error_stage) - 1);
      }
    }
  }

  if (st.samples_written < st.expected_samples && !recorder_abort_requested && err == 0) {
    if (dma_stats.samples_lost != 0) {
      err = -EIO;
    }
  }

  /* Verify total accounted frame count (written + dropped) matches configured recording target */
  if (!recorder_abort_requested && err == 0) {
    uint64_t total_accounted = st.samples_written + st.samples_dropped;
    if (total_accounted != st.expected_samples) {
      LOG_ERR("Sample count mismatch: requested %llu, wrote %llu, dropped %llu",
              (unsigned long long)st.expected_samples,
              (unsigned long long)st.samples_written,
              (unsigned long long)st.samples_dropped);
      err = -EINVAL;
    }
  }

  /* Verify the file on disk ONLY if audio frames were written AND session completed cleanly without errors */
  if (err == 0 && !recorder_abort_requested && st.bytes_written > 0) {
    int v_ret = recorder_verify_file_format(
        st.file, st.sample_rate, st.channels, st.sample_bits, st.bytes_written);
    st.wav_verified = (v_ret == 0);
    if (v_ret != 0) {
      LOG_ERR("Post-recording file verification failed: %d", v_ret);
      err = v_ret;
    } else {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("[TEST] WAV_VERIFIED\n");
#endif
    }
  } else {
    st.wav_verified = false;
  }

  int sd_idle_chk = recorder_sd_verify_idle();
  if (sd_idle_chk == 0) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[TEST] SDMMC_IDLE\n");
#endif
  }

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  struct dsp_stats ds_log;
  dsp_get_stats(&ds_log);
  struct psram_staging_stats pstat;
  psram_staging_get_stats(&pstat);
  uint32_t sd_avg_kb_s = 0;
  if (st.avg_write_us > 0) {
    sd_avg_kb_s = (uint32_t)(((uint64_t)BLOCK_BYTES * 1000000ULL) / ((uint64_t)st.avg_write_us * 1024ULL));
  }
  uint64_t total_active_samples_lost = (uint64_t)dma_stats.samples_lost + st.samples_dropped;
  printk("\n[TEST] RECORDING ACCOUNTING SUMMARY:\n");
  printk("  Expected source samples            : %llu\n", (unsigned long long)st.expected_samples);
  printk("  Acquired source samples            : %llu\n", (unsigned long long)st.hardware_source_samples);
  printk("  Successfully processed samples     : %llu\n", (unsigned long long)st.processed_samples);
  printk("  Samples written                    : %llu\n", (unsigned long long)st.samples_written);
  printk("  Samples lost during active capture : %llu\n", (unsigned long long)total_active_samples_lost);
  printk("  Gaps recorded in session           : %u\n", s_session_gap_count);
  printk("  Backpressure slowdown events       : %u\n", st.slowdown_events);
  printk("  Drop breakdown: DMA=%u, Pool=%u, PSRAM Staging=%llu, SD Write=%llu\n",
         dma_stats.samples_lost,
         st.dma_dropped_blocks * BLOCK_SAMPLES,
         (unsigned long long)st.psram_dropped_samples,
         (unsigned long long)st.sd_failed_samples);
  printk("  Boundary samples discarded         : %llu\n", (unsigned long long)st.boundary_discarded_samples);
  printk("  DSP average / worst time           : %u / %u us\n", st.dsp_avg_us, st.dsp_worst_us);
  printk("  DSP block deadline                 : %u us\n", ds_log.block_deadline_us);
  printk("  DMA worker max latency             : %u cyc\n", dma_stats.acquisition_max_latency_cycles);
  printk("  Filled queue high-water            : %u\n", audio_queue_high_water);
  printk("  PSRAM Physical Capacity            : 4 MB\n");
  printk("  PSRAM Audio Allocation             : 3 MB\n");
  printk("  PSRAM Reserved Capacity            : 1 MB\n");
  printk("  PSRAM Slot Count                   : 12\n");
  printk("  PSRAM Slot Size                    : 256 KB\n");
  printk("  PSRAM Current Occupancy            : %u KB\n", pstat.occupied_bytes >> 10);
  printk("  PSRAM High-Water Mark              : %u KB\n", st.psram_high_water_bytes >> 10);
  printk("  PSRAM Free Slots                   : %u\n", pstat.free_slots);
  printk("  PSRAM Full Events                  : %u\n", st.psram_full_events);
  printk("  PSRAM Rejected Audio Blocks        : %u\n", st.psram_full_events);
  printk("  SD Writer Average Throughput       : %u KB/s (avg latency %u us)\n", sd_avg_kb_s, st.avg_write_us);
  printk("  SD Writer Worst-Case Latency       : %u us\n", st.worst_write_us);
  printk("  SD Writer Aggregate Throughput     : %u KB/s (batch wall: %llu ms, worst batch: %u ms)\n",
         st.sd_aggregate_throughput_kb_s, (unsigned long long)(st.sd_writer_wall_us / 1000ULL), sd_writer_worst_batch_ms);
  printk("  SD Writer Ping-Pong Overlap        : %llu ms (%u buffer swaps)\n",
         (unsigned long long)(st.pingpong_overlap_us / 1000ULL), st.pingpong_swaps);
  printk("  PSRAM QSPI Transaction Count       : %u (read throughput: %u KB/s)\n",
         st.psram_qspi_txn_count, st.psram_read_throughput_kb_s);
  printk("  SD Writer Time Breakdown          : fs_write=%llu ms, psram_peek=%llu ms, commit=%llu ms, preemption/starve=%llu ms\n",
         (unsigned long long)(st.sd_writer_active_fs_us / 1000ULL),
         (unsigned long long)(st.sd_writer_psram_read_us / 1000ULL),
         (unsigned long long)(st.sd_writer_commit_us / 1000ULL),
         (unsigned long long)(st.sd_writer_starve_us / 1000ULL));
  printk("  SD error                           : %d\n", st.sd_write_error);
  printk("  First error origin                 : %s\n", st.first_error_stage[0] ? st.first_error_stage : "NONE");
  bool zero_loss = (total_active_samples_lost == 0) &&
                   (st.blocks_dropped == 0) &&
                   (dma_stats.events_missed == 0) &&
                   (st.psram_full_events == 0) &&
                   (st.sd_write_error == 0) &&
                   (s_session_gap_count == 0);

  if (zero_loss && err == 0) {
    printk("  Capture Loss Integrity             : ZERO LOSS\n\n");
    printk("[TEST] ZERO_LOSS_VERIFIED\n");
  } else {
    printk("  Capture Loss Integrity             : DATA LOSS OCCURRED (%llu samples lost, %u gaps)\n\n",
           (unsigned long long)total_active_samples_lost, s_session_gap_count);
    printk("[TEST] DATA_LOSS_REPORTED: lost_samples=%llu, dropped_blocks=%u, gaps=%u, psram_full=%u, sd_err=%d\n",
           (unsigned long long)total_active_samples_lost,
           st.blocks_dropped,
           s_session_gap_count,
           st.psram_full_events,
           st.sd_write_error);
  }
#endif

  s_power_state = REC_PWR_ACTIVE;
  struct fs_dirent file_ent;
  if (fs_stat(st.file, &file_ent) == 0) {
    st.file_size_bytes = file_ent.size;
  }

  st.queue_high_water = audio_queue_high_water;
  st.pool_high_water = high_water;
  st.sd_queue_high_water = sd_queue_high_water;
  st.err = err;
  if (err != 0) {
    st.wav_verified = false;
  }

  if (out) {
    *out = st;
  }
  s_recorder_last_error = err;
  s_recorder_last_sd_error = st.sd_write_error;
  s_active_stats_ptr = NULL;
  s_active_file_path[0] = '\0';
  recorder_active = false;
  if (err != 0 && !recorder_abort_requested) {
    led_indicator_set(LED_IND_ERROR);
    printk("[ERROR] Recording session failed: %d\n", err);
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("  Diagnostics: acquired=%llu, written=%llu, uncommitted=%llu, drops=%llu\n",
           (unsigned long long)st.samples_acquired,
           (unsigned long long)st.samples_written,
           (unsigned long long)st.sd_failed_samples,
           (unsigned long long)st.samples_dropped);
    printk("  Pipeline: dma_err=%u, adc_ovr=%u, dsp_ovr=%u, psram_full=%u, sd_err=%d\n",
           st.dma_errors, st.adc_overruns, st.dsp_overruns, st.psram_full_events, st.sd_write_error);
#endif
    /* Only storage faults justify unmount/remount; acquisition/PSRAM errors do
     * not imply a bad SD card. Writer completion above establishes idle first. */
    int storage_error = (sd_writer_err != 0) ? sd_writer_err : st.sd_write_error;
    if (storage_error == -EIO || storage_error == -ETIMEDOUT ||
        storage_error == -EBUSY) {
      st.sd_recovery_attempts++;
      s_storage_recovery_result = recorder_storage_recover();
      st.sd_recovery_result = s_storage_recovery_result;
      if (s_storage_recovery_result != 0) {
        LOG_ERR("Storage recovery failed: %d", s_storage_recovery_result);
      }
      if (out) *out = st;
    }
  } else {
    led_indicator_set(LED_IND_OFF);
  }
cleanup:
  if (file_opened) {
    fs_close(&f);
    file_opened = false;
    if (st.samples_written == 0 && st.file[0] != '\0') {
      fs_unlink(st.file);
      st.file[0] = '\0';
    }
  }
  sd_file_ptr = NULL;
  if (sd_bounce_buf) {
    /* k_mem_slab_free(&pool, sd_bounce_buf); */
    sd_bounce_buf = NULL;
  }
  s_pingpong[0].buf = NULL;
  s_pingpong[0].state = PINGPONG_BUF_FREE;
  s_pingpong[0].bytes = 0;
  s_pingpong[1].buf = NULL;
  s_pingpong[1].state = PINGPONG_BUF_FREE;
  s_pingpong[1].bytes = 0;
  s_active_stats_ptr = NULL;
  if (out) {
    *out = st;
  }
  return err;
}

