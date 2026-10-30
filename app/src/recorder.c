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
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/time_units.h>
#include "led_indicator.h"

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(recorder, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(recorder, LOG_LEVEL_WRN);
#endif

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
K_MSGQ_DEFINE(filled_q, sizeof(void *), NUM_BLOCKS, 4);

/* Asynchronous non-PSRAM block message queue to sd_writer_thread */
struct sd_block_msg {
  void *blk;
  uint32_t bytes;
  uint32_t samples;
};
K_MSGQ_DEFINE(sd_block_q, sizeof(struct sd_block_msg), NUM_BLOCKS, 4);

#define SD_WRITER_STACK_SIZE 4096
/* SD writer runs at preemptive priority 2 (below real-time capture priority 0,
 * above background tasks) */
#define SD_WRITER_PRIORITY 2

K_THREAD_STACK_DEFINE(sd_writer_stack, SD_WRITER_STACK_SIZE);
static struct k_thread sd_writer_thread_data;
static k_tid_t sd_writer_tid;

static struct k_sem sd_writer_sem;
static struct k_sem sd_writer_done_sem;
static volatile bool sd_writer_exit_req = false;
static volatile bool sd_writer_flush_req = false;
static struct fs_file_t *sd_file_ptr = NULL;
static void *sd_bounce_buf = NULL;

static volatile uint64_t sd_writer_write_us_sum = 0;
static volatile uint32_t sd_writer_write_count = 0;
static volatile uint32_t sd_writer_worst_write_us = 0;
static volatile uint64_t sd_writer_bytes_written = 0;
static volatile uint64_t sd_writer_failed_bytes = 0;
static volatile uint32_t sd_writer_batch_count = 0;
static volatile int sd_writer_err = 0;

#ifdef CONFIG_AUDIO_PSRAM_STAGING
static bool g_psram_staging_enabled = true;
#else
static bool g_psram_staging_enabled = false;
#endif
static volatile bool active_use_psram = false;

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
          if (sd_writer_err != 0) {
            sd_writer_failed_bytes = psram_staging_occupied();
          }
          break;
        }

        uint32_t to_write =
            sd_writer_flush_req ? psram_staging_occupied() : batch_threshold;
        if (!sd_writer_flush_req && to_write > batch_threshold) {
          to_write = batch_threshold;
        }

        uint32_t batch_bytes_written = 0;
        uint64_t t_batch_start = k_cycle_get_64();
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

        while (to_write > 0 && psram_staging_occupied() > 0) {
          uint32_t chunk = (to_write > BLOCK_BYTES) ? BLOCK_BYTES : to_write;
          uint32_t bytes_read = 0;
          int peek_ret = psram_staging_peek(sd_bounce_buf, chunk, &bytes_read);
          if (peek_ret != 0 || bytes_read == 0) {
            LOG_ERR("SD writer PSRAM peek failed (%d)", peek_ret);
            sd_writer_err = peek_ret ? peek_ret : -EIO;
            break;
          }

          uint64_t t0 = k_cycle_get_64();
          ssize_t w = fs_write(sd_file_ptr, sd_bounce_buf, bytes_read);
          uint64_t t1 = k_cycle_get_64();
          uint64_t diff_cyc = (t1 >= t0) ? (t1 - t0) : 0;
          uint32_t us = (uint32_t)k_cyc_to_us_floor64(diff_cyc);

          if (w != (ssize_t)bytes_read) {
            LOG_ERR("SD writer fs_write failed: ret=%d, expected=%u", (int)w,
                    bytes_read);
            sd_writer_err = (w < 0) ? (int)w : -EIO;
            break;
          }

          /* Commit consumer index ONLY after successful SD write */
          int commit_ret = psram_staging_commit(bytes_read);
          if (commit_ret != 0) {
            LOG_ERR("SD writer staging commit failed after successful write (%d)",
                    commit_ret);
            sd_writer_err = commit_ret;
            break;
          }

          sd_writer_write_us_sum += us;
          sd_writer_write_count++;
          if (us > sd_writer_worst_write_us) {
            sd_writer_worst_write_us = us;
          }
          sd_writer_bytes_written += bytes_read;
          batch_bytes_written += bytes_read;
          to_write -= bytes_read;
        }

        uint64_t t_batch_end = k_cycle_get_64();
        uint64_t batch_cyc =
            (t_batch_end >= t_batch_start) ? (t_batch_end - t_batch_start) : 0;
        uint32_t batch_ms =
            (uint32_t)(k_cyc_to_us_floor64(batch_cyc) / 1000ULL);

        if (sd_writer_err == 0 && batch_bytes_written > 0) {
          LOG_INF("Batch %u completed: %u bytes written to SD in %u ms (total "
                  "committed: %llu bytes, PSRAM occupied: %u KB)",
                  cur_batch, batch_bytes_written, batch_ms,
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
          k_mem_slab_free(&pool, msg.blk);
          continue;
        }

        uint64_t t0 = k_cycle_get_64();
        ssize_t w = fs_write(sd_file_ptr, msg.blk, msg.bytes);
        uint64_t t1 = k_cycle_get_64();
        uint64_t diff_cyc = (t1 >= t0) ? (t1 - t0) : 0;
        uint32_t us = (uint32_t)k_cyc_to_us_floor64(diff_cyc);

        k_mem_slab_free(&pool, msg.blk);

        if (w != (ssize_t)msg.bytes) {
          LOG_ERR("SD writer fs_write failed: ret=%d, expected=%u", (int)w,
                  msg.bytes);
          sd_writer_err = (w < 0) ? (int)w : -EIO;
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

/* ---- audio_sink callbacks: called from the high-priority acquisition thread ---- */
static void *sink_alloc(void *user) {
  void *blk;

  ARG_UNUSED(user);
  if (k_mem_slab_alloc(&pool, &blk, K_NO_WAIT) != 0) {
    dma_alloc_drops++;
    drops++;
    return NULL;
  }
  return blk;
}

static int sink_submit(void *blk, void *user) {
  ARG_UNUSED(user);
  if (k_msgq_put(&filled_q, &blk, K_NO_WAIT) != 0) {
    /* cannot happen (queue as deep as the pool) but never leak a block */
    queue_submit_drops++;
    drops++;
    k_mem_slab_free(&pool, blk);
    return -ENOBUFS;
  }
  uint32_t queued = k_msgq_num_used_get(&filled_q);
  if (queued > audio_queue_high_water) audio_queue_high_water = queued;
  /*
   * Track true buffer high-water mark:
   * Total audio blocks currently held in RAM (in queue + actively being
   * written) is (NUM_BLOCKS - k_mem_slab_num_free_get(&pool)). This accurately
   * tracks pool buffering even when k_msgq_put hands the block directly to a
   * waiting thread without incrementing msgq->used_msgs.
   */
  uint32_t in_use = NUM_BLOCKS - k_mem_slab_num_free_get(&pool);
  if (in_use > high_water) {
    high_water = in_use;
  }
  return 0;
}

static void sink_release(void *blk, void *user) {
  ARG_UNUSED(user);
  k_mem_slab_free(&pool, blk);
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
  /* Run self-test on timing math */
  recorder_timing_unit_test();

  /* Initialize DSP noise reduction pipeline */
  dsp_init(CONFIG_AUDIO_SAMPLE_RATE);

  /* Initialize RTC Wakeup Timer subsystem */
  rtc_sleep_init();

  /* Initialize RTC Calendar subsystem */
  rtc_calendar_init();

  /* Initialize SD writer synchronization primitives and background thread */
  k_sem_init(&sd_writer_sem, 0, 1);
  k_sem_init(&sd_writer_done_sem, 0, 1);
  sd_writer_tid = k_thread_create(&sd_writer_thread_data, sd_writer_stack,
                                  K_THREAD_STACK_SIZEOF(sd_writer_stack),
                                  sd_writer_thread_fn, NULL, NULL, NULL,
                                  SD_WRITER_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(sd_writer_tid, "sd_writer");

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
    return ret;
  }

  ret = fs_mount(&mp);
  if (ret != 0) {
    LOG_ERR("Storage recovery: filesystem remount failed: %d", ret);
    return ret;
  }
  mounted = true;
  ret = disk_access_ioctl("SD", DISK_IOCTL_CTRL_DEINIT, NULL);
  if (ret != 0) {
    LOG_WRN("Storage recovery: could not release temporary disk reference: %d", ret);
  }
  LOG_INF("Storage recovery: SDMMC reinitialized and filesystem remounted");
  return 0;
}

static int next_filename(char *name, size_t len) {
  struct fs_dirent ent;

  for (unsigned int n = 1; n < 10000; n++) {
    snprintf(name, len, "/SD:/REC%04u.WAV", n);
    if (fs_stat(name, &ent) != 0) {
      return 0; /* does not exist yet */
    }
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

      char dir[64] = "/SD:";
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

      for (unsigned int n = 1; n < 1000; n++) {
        snprintf(name, len, "%s/WS_%04u%02u%02u_%02u%02u%02u_%02u.WAV",
                 dir, loc_td.year, loc_td.month, loc_td.day, loc_td.hour, loc_td.min, loc_td.sec, n);
#if defined(CONFIG_FILE_SYSTEM)
        struct fs_dirent ent;
        if (fs_stat(name, &ent) != 0) {
          *rtc_valid = true;
          return 0;
        }
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
  void *blk;
  uint32_t count = 0;

  while (k_msgq_get(&filled_q, &blk, K_NO_WAIT) == 0) {
    k_mem_slab_free(&pool, blk);
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

int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us) {
  if (seconds == 0) {
    if (wake_recovery_us) {
      *wake_recovery_us = 0;
    }
    return 0;
  }

  /* 1. Put SD card into idle standby and flush any write cache */
  disk_access_ioctl("SD", DISK_IOCTL_CTRL_SYNC, NULL);

  /* 2. Put external IS66WVS PSRAM into documented Deep Power Down (0xB9) state
   */
  psram_sleep();

  /* 3. Confirm audio pipeline is completely stopped and in deep power-down */
  audio_in_stop();
  drain_queue();

  LOG_INF("Entering STM32L476 Stop 2 low-power mode for %u seconds (RTC Wakeup "
          "Timer)...",
          seconds);
  const uint64_t t_pre = k_uptime_get();

  /* 4. Enter STM32L476 Stop 2 mode */
  int res = rtc_sleep_enter_stop2(seconds);

  const uint64_t t_post = k_uptime_get();
  uint64_t elapsed_ms = (t_post >= t_pre) ? (t_post - t_pre) : (seconds * 1000);
  uint64_t expected_ms = (uint64_t)seconds * 1000;
  uint32_t rec_us = (elapsed_ms > expected_ms)
                        ? (uint32_t)((elapsed_ms - expected_ms) * 1000)
                        : 0;
  if (wake_recovery_us) {
    *wake_recovery_us = rec_us;
  }

  /* 5. Wake external PSRAM and verify SPI/QPI interface */
  psram_wake();

  /* 6. Verify SD card is responsive and synchronized */
  disk_access_ioctl("SD", DISK_IOCTL_CTRL_SYNC, NULL);

  enum rtc_wake_source src = rtc_sleep_get_last_wake_source();

  if (res == -EINTR) {
    const char *reason = (src == RTC_WAKE_USER_BUTTON)
                             ? "User Button (B1 / PC13)"
                             : "Console Abort";
    LOG_INF("Stop 2 sleep aborted by %s after %llu ms", reason, elapsed_ms);
    return -EINTR;
  }

  LOG_INF("Woke from STM32L476 Stop 2 mode via RTC Wakeup Timer (elapsed: %llu "
          "ms, wake recovery overhead: %u us)",
          elapsed_ms, rec_us);
  return 0;
}

static volatile bool recorder_abort_requested = false;
static volatile bool recorder_active = false;

bool recorder_is_active(void) { return recorder_active; }

bool recorder_is_abort_requested(void) { return recorder_abort_requested; }

void recorder_request_stop(void) { recorder_abort_requested = true; }

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
  recorder_active = true;

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

  struct fs_file_t f;
  static uint8_t hdr[WAV_HEADER_BYTES] __aligned(4);
  int ret;

  if (!mounted) {
    led_indicator_set(LED_IND_ERROR);
    st.err = -ENODEV;
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
    if (out)
      *out = st;
    return ret;
  }
  fs_file_t_init(&f);
  ret = fs_open(&f, st.file, FS_O_CREATE | FS_O_WRITE);
  if (ret) {
    LOG_ERR("open %s: %d", st.file, ret);
    led_indicator_set(LED_IND_ERROR);
    st.file[0] = '\0';
    st.err = ret;
    if (out)
      *out = st;
    return ret;
  }

  /*
   * File pre-allocation handling:
   * Zephyr's VFS fatfs_seek rejects seeking past EOF with -EINVAL (-22).
   * If CONFIG_AUDIO_PREALLOCATE_FILE is enabled and FatFs native API f_expand
   * is compiled, f_expand() is used on the empty file. Otherwise, sequential
   * dynamic writes are used without seeking past EOF.
   */
#if defined(CONFIG_AUDIO_PREALLOCATE_FILE) &&                                  \
    defined(CONFIG_FS_FATFS_EXTRA_NATIVE_API)
  FIL *fil = (FIL *)f.filep;
  uint64_t t_pre0 = k_cycle_get_64();
  FRESULT f_res =
      f_expand(fil, (FSIZE_t)(WAV_HEADER_BYTES + prealloc_bytes), 1);
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
#else
  LOG_INF("File pre-allocation disabled; using sequential dynamic writes");
#endif

  wav_fill_header(hdr, st.sample_rate, st.channels, st.sample_bits, 0);
  ret = fs_write(&f, hdr, WAV_HEADER_BYTES);
  if (ret != WAV_HEADER_BYTES) {
    led_indicator_set(LED_IND_ERROR);
    fs_close(&f);
    fs_unlink(st.file);
    st.file[0] = '\0';
    st.err = ret < 0 ? ret : -EIO;
    st.sd_write_error = st.err;
    if (out)
      *out = st;
    return st.err;
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
  drain_queue();

  bool use_psram = g_psram_staging_enabled && psram_staging_is_active() &&
                   psram_is_present() && psram_is_quad_mode();
  if (g_psram_staging_enabled && !use_psram) {
    LOG_WRN("PSRAM staging disabled: Quad I/O unverified; falling back to direct SRAM audio queue");
  }
  if (use_psram) {
    psram_staging_reset();
    st.psram_used = true;
    st.psram_staging_capacity = psram_staging_capacity();
    st.sd_batch_bytes = psram_staging_batch_size();

    /* Borrow 1 block from pool as SD writer bounce buffer */
    ret = k_mem_slab_alloc(&pool, &sd_bounce_buf, K_NO_WAIT);
    if (ret != 0) {
      LOG_WRN("Failed to allocate SD bounce buffer from pool (%d); bypassing "
              "PSRAM staging",
              ret);
      use_psram = false;
      st.psram_used = false;
      sd_bounce_buf = NULL;
    }
  } else {
    st.psram_used = false;
    st.psram_staging_capacity = 0;
    st.sd_batch_bytes = 0;
    sd_bounce_buf = NULL;
  }

  active_use_psram = use_psram;
  sd_file_ptr = &f;
  sd_writer_flush_req = false;
  sd_writer_write_us_sum = 0;
  sd_writer_write_count = 0;
  sd_writer_worst_write_us = 0;
  sd_writer_bytes_written = 0;
  sd_writer_failed_bytes = 0;
  sd_writer_batch_count = 0;
  sd_writer_err = 0;

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
    LOG_ERR("audio_in_start failed: %d", ret);
    led_indicator_set(LED_IND_ERROR);
    fs_close(&f);
    fs_unlink(st.file);
    st.file[0] = '\0';
    st.err = ret;
    if (out) {
      *out = st;
    }
    return ret;
  }

  led_indicator_set(LED_IND_RECORDING);

  const uint64_t t_start_cyc = k_cycle_get_64();
  uint64_t t_first_block_cyc = 0;
  uint64_t t_last_block_cyc = 0;
  int err = 0;
  uint64_t samples_remaining = st.expected_samples;

  while (samples_remaining > 0) {
    if (recorder_abort_requested) {
      LOG_INF("Stop requested: finalizing recorded frames early (%llu samples "
              "written)",
              st.samples_written);
      break;
    }
    void *blk;

    /* a healthy source delivers a block every ~10 ms; 2 s means it died */
    if (k_msgq_get(&filled_q, &blk, K_SECONDS(2)) != 0) {
      LOG_ERR("no audio for 2 s");
      err = -ETIMEDOUT;
      break;
    }
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
        k_mem_slab_free(&pool, blk);

        if (p_ret == -ENOBUFS) {
          /* PSRAM staging buffer is full */
          st.psram_full_events++;
          st.psram_dropped_samples += samples_in_block;
          static uint32_t last_full_warn_ms = 0;
          uint32_t now_ms = k_uptime_get_32();
          if (now_ms - last_full_warn_ms > 1000) {
            last_full_warn_ms = now_ms;
            LOG_WRN("PSRAM staging buffer full! Occupancy: %u KB / %u KB. "
                    "Dropping audio block (%u samples)",
                    psram_staging_occupied() >> 10,
                    psram_staging_capacity() >> 10, samples_in_block);
          }
          /* Abort capture cleanly to preserve stream integrity */
          err = -ENOBUFS;
          break;
        } else if (p_ret != 0) {
          LOG_ERR("PSRAM push fatal error: %d", p_ret);
          err = p_ret;
          break;
        }

        if (sd_writer_err != 0) {
          LOG_ERR("SD writer error detected in recording loop: %d",
                  sd_writer_err);
          err = sd_writer_err;
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
        struct sd_block_msg s_msg = {
            .blk = blk,
            .bytes = bytes_to_write,
            .samples = samples_in_block,
        };

        int q_ret = k_msgq_put(&sd_block_q, &s_msg, K_NO_WAIT);
        if (q_ret == 0) {
          uint32_t sd_queued = k_msgq_num_used_get(&sd_block_q);
          if (sd_queued > sd_queue_high_water) sd_queue_high_water = sd_queued;
        }
        if (q_ret != 0) {
          /* Bounded RAM queue capacity exhausted */
          sram_writer_drops++;
          k_mem_slab_free(&pool, blk);
          LOG_ERR("Non-PSRAM RAM queue exhausted (%u/%u blocks)! Dropping block "
                  "(%u samples). Aborting capture.",
                  (unsigned)k_msgq_num_used_get(&sd_block_q), NUM_BLOCKS,
                  samples_in_block);
          err = -ENOBUFS;
          break;
        }

        /* Signal SD writer thread to consume queued block */
        k_sem_give(&sd_writer_sem);

        if (sd_writer_err != 0) {
          LOG_ERR("SD writer error detected in recording loop: %d",
                  sd_writer_err);
          err = sd_writer_err;
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
      static uint8_t format_chunk_buf[8192] __aligned(4);
      uint32_t chunk_frames = 1024;
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
            st.psram_full_events++;
            st.psram_dropped_samples += (samples_in_block - off);
            err = -ENOBUFS;
            chunk_failed = true;
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

  audio_in_stop();
  uint32_t drained_blocks = drain_queue();
  st.queue_drained_blocks = drained_blocks;
  st.queue_drained_samples = drained_blocks * BLOCK_SAMPLES;
  st.sram_queue_dropped_blocks = sram_writer_drops;

  struct audio_dma_stats dma_stats = {0};
  audio_in_get_dma_stats(&dma_stats);
  st.adc_overruns = dma_stats.adc_overruns;
  st.dma_errors = dma_stats.callback_errors;
  st.acquisition_events_missed = dma_stats.events_missed;
  st.acquisition_samples_processed = dma_stats.samples_processed;
  st.acquisition_samples_lost = dma_stats.samples_lost;
  st.acquisition_samples_accounted = dma_stats.samples_accounted;
  st.dma_interval_min_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_min_cycles);
  st.dma_interval_avg_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_avg_cycles);
  st.dma_interval_max_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_max_cycles);
  st.dma_dropped_blocks = (dma_stats.samples_lost + BLOCK_SAMPLES - 1U) /
                          BLOCK_SAMPLES;

  /* Signal SD writer thread to flush any remaining audio bytes/blocks */
  if (use_psram) {
    psram_staging_flush_publish();
  }
  sd_writer_flush_req = true;
  k_sem_give(&sd_writer_sem);
  k_sem_take(&sd_writer_done_sem, K_FOREVER);
  sd_writer_flush_req = false;

  /* Release borrowed bounce buffer back to pool if allocated */
  if (sd_bounce_buf) {
    k_mem_slab_free(&pool, sd_bounce_buf);
    sd_bounce_buf = NULL;
  }
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
    st.psram_dropped_samples = pstat.dropped_samples;
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
  const uint32_t data_bytes = (uint32_t)st.bytes_written;

  wav_fill_header(hdr, st.sample_rate, st.channels, st.sample_bits, data_bytes);
  int finalize_ret = 0;
  if (err == 0 && sd_writer_err == 0) {
    finalize_ret = fs_seek(&f, 0, FS_SEEK_SET);
    if (finalize_ret == 0) {
      ssize_t header_written = fs_write(&f, hdr, WAV_HEADER_BYTES);
      if (header_written != WAV_HEADER_BYTES)
        finalize_ret = (header_written < 0) ? (int)header_written : -EIO;
    }
    if (finalize_ret == 0)
      finalize_ret = fs_truncate(&f, WAV_HEADER_BYTES + data_bytes);
    if (finalize_ret == 0)
      finalize_ret = fs_sync(&f);
  } else {
    LOG_WRN("Skipping WAV header finalization due to previous storage error (%d)",
            err != 0 ? err : sd_writer_err);
  }
  int close_ret = fs_close(&f);
  if (finalize_ret == 0) finalize_ret = close_ret;
  if (finalize_ret != 0 && err == 0) {
    LOG_ERR("WAV finalize/sync/close failed: %d", finalize_ret);
    err = finalize_ret;
    if (st.sd_write_error == 0) st.sd_write_error = finalize_ret;
  }

  /* Verify exact frame count and byte count match configured recording target
   */
  if (!recorder_abort_requested && err == 0) {
    if (st.samples_written != st.expected_samples) {
      LOG_ERR("Sample count mismatch: requested %llu, wrote %llu",
              st.expected_samples, st.samples_written);
      err = -EINVAL;
    }
    if (st.bytes_written != exact_bytes) {
      LOG_ERR("PCM byte count mismatch: expected %llu, wrote %llu", exact_bytes,
              st.bytes_written);
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
    }
  } else {
    st.wav_verified = false;
  }
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
    /* Cleanly unmount and remount storage to recover from SDMMC/FatFs timeout */
    recorder_storage_recover();
  } else {
    led_indicator_set(LED_IND_OFF);
  }
  return err;
}

