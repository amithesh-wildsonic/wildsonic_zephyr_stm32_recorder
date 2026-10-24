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
#include <ff.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/time_units.h>

LOG_MODULE_REGISTER(recorder, LOG_LEVEL_INF);

#define SAMPLE_BYTES (CONFIG_AUDIO_SAMPLE_BITS / 8)
#define BLOCK_SAMPLES CONFIG_AUDIO_BLOCK_SAMPLES
#define BLOCK_BYTES (BLOCK_SAMPLES * SAMPLE_BYTES)
#define NUM_BLOCKS CONFIG_AUDIO_NUM_BLOCKS
#define DIAG_BLOCK_SAMPLES CONFIG_AUDIO_DIAGNOSTIC_BLOCK_SAMPLES
#define DIAG_BLOCK_BYTES (DIAG_BLOCK_SAMPLES * SAMPLE_BYTES)
#define DIAG_POOL_BLOCKS 2
#define SLOW_WRITE_US 50000U /* log any single write slower than this (50 ms)  \
                              */

#if CONFIG_AUDIO_INTEGRITY_REPLAY
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
  while (len--) {
    crc ^= *data++;
    for (unsigned bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
  }
  return crc;
}

static uint32_t integrity_expected_crc(uint64_t samples)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (uint64_t i = 0; i < samples; i++) {
    uint16_t bits = (uint16_t)audio_in_integrity_sample(i);
    uint8_t pcm_le[2] = {(uint8_t)bits, (uint8_t)(bits >> 8)};
    crc = crc32_update(crc, pcm_le, sizeof(pcm_le));
  }
  return ~crc;
}

static uint32_t get_le32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int fs_read_exact(struct fs_file_t *file, void *dst, size_t len)
{
  uint8_t *cursor = dst;
  while (len > 0) {
    ssize_t got = fs_read(file, cursor, len);
    if (got < 0) return (int)got;
    if (got == 0) return -EIO;
    cursor += (size_t)got;
    len -= (size_t)got;
  }
  return 0;
}

/* Parse RIFF chunks and read only the data payload, using a bounded 512 B buffer. */
static int integrity_readback_crc(const char *path, uint32_t *crc_out,
                                  uint64_t *bytes_out)
{
  struct fs_file_t file;
  struct fs_dirent ent;
  uint8_t chunk_hdr[8];
  static uint8_t buf[512] __aligned(4);
  uint64_t data_offset = 0, data_size = 0;
  int ret = fs_stat(path, &ent);
  if (ret != 0) return ret;
  uint64_t file_size = ent.size;

  fs_file_t_init(&file);
  ret = fs_open(&file, path, FS_O_READ);
  if (ret != 0) return ret;
  uint8_t riff[12];
  if (fs_read_exact(&file, riff, sizeof(riff)) != 0 ||
      memcmp(riff, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) {
    fs_close(&file);
    return -EBADMSG;
  }
  uint64_t riff_end = (uint64_t)get_le32(riff + 4) + 8U;
  if (riff_end > file_size || riff_end < sizeof(riff)) {
    fs_close(&file);
    return -EBADMSG;
  }

  uint64_t pos = sizeof(riff);
  while (pos + sizeof(chunk_hdr) <= riff_end) {
    if (fs_seek(&file, (off_t)pos, FS_SEEK_SET) != 0 ||
        fs_read_exact(&file, chunk_hdr, sizeof(chunk_hdr)) != 0) {
      fs_close(&file);
      return -EIO;
    }
    uint32_t chunk_size = get_le32(chunk_hdr + 4);
    uint64_t payload = pos + sizeof(chunk_hdr);
    uint64_t chunk_end = payload + chunk_size + (chunk_size & 1U);
    if (chunk_end > riff_end || chunk_end > file_size) {
      fs_close(&file);
      return -EBADMSG;
    }
    if (memcmp(chunk_hdr, "data", 4) == 0) {
      data_offset = payload;
      data_size = chunk_size;
      break;
    }
    pos = chunk_end;
  }
  if (data_size == 0 || data_offset + data_size > riff_end) {
    fs_close(&file);
    return -EBADMSG;
  }

  ret = fs_seek(&file, (off_t)data_offset, FS_SEEK_SET);
  uint64_t remaining = data_size;
  uint32_t crc = 0xFFFFFFFFU;
  while (ret == 0 && remaining > 0) {
    uint32_t amount = (remaining > sizeof(buf)) ? sizeof(buf) : (uint32_t)remaining;
    ssize_t got = fs_read(&file, buf, amount);
    if (got <= 0) {
      ret = (got < 0) ? (int)got : -EIO;
      break;
    }
    crc = crc32_update(crc, buf, (size_t)got);
    remaining -= (uint64_t)got;
  }
  fs_close(&file);
  if (ret != 0) return ret;
  *crc_out = ~crc;
  *bytes_out = data_size;
  return 0;
}
#endif

K_MEM_SLAB_DEFINE_STATIC(pool, BLOCK_BYTES, NUM_BLOCKS, 4);
K_MSGQ_DEFINE(filled_q, sizeof(void *), NUM_BLOCKS, 4);
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
/* CPU-only diagnostic blocks live in the separate 32 KiB SRAM2 region. */
K_MEM_SLAB_DEFINE_IN_SECT_STATIC(diag_pool, __attribute__((section("SRAM1"))),
				 DIAG_BLOCK_BYTES, DIAG_POOL_BLOCKS, 4);
K_MSGQ_DEFINE(diag_filled_q, sizeof(void *), DIAG_POOL_BLOCKS, 4);
#endif

/* Asynchronous non-PSRAM block message queue to sd_writer_thread */
struct sd_block_msg {
  void *blk;
  uint32_t bytes;
  uint32_t samples;
};
K_MSGQ_DEFINE(sd_block_q, sizeof(struct sd_block_msg), NUM_BLOCKS, 4);

#define SD_WRITER_STACK_SIZE 8192
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
static volatile bool s_benchmark_running = false;

bool benchmark_is_running(void) { return s_benchmark_running; }
void benchmark_set_running(bool running) { s_benchmark_running = running; }

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

int recorder_borrow_pool_blocks(void **blocks, uint32_t count) {
  if (!blocks || count == 0 || count > NUM_BLOCKS) {
    return -EINVAL;
  }
  if (recorder_is_active()) {
    return -EBUSY;
  }
  uint32_t allocated = 0;
  for (uint32_t i = 0; i < count; i++) {
    void *blk = NULL;
    if (k_mem_slab_alloc(&pool, &blk, K_NO_WAIT) != 0) {
      for (uint32_t j = 0; j < allocated; j++) {
        k_mem_slab_free(&pool, blocks[j]);
        blocks[j] = NULL;
      }
      return -ENOMEM;
    }
    blocks[i] = blk;
    allocated++;
  }
  return 0;
}

void recorder_return_pool_blocks(void **blocks, uint32_t count) {
  if (!blocks) return;
  for (uint32_t i = 0; i < count; i++) {
    if (blocks[i]) {
      k_mem_slab_free(&pool, blocks[i]);
      blocks[i] = NULL;
    }
  }
}

int recorder_borrow_contiguous_pool(void **buf, uint32_t *max_bytes) {
  if (!buf || !max_bytes) return -EINVAL;
  if (recorder_is_active()) return -EBUSY;
  if (k_mem_slab_num_free_get(&pool) != NUM_BLOCKS) return -EBUSY;

  /*
   * When recorder is idle and all blocks are free, the underlying static array
   * _k_mem_slab_buf_pool provides a contiguous 64 KiB buffer aligned to 4 bytes.
   */
  *buf = (void *)_k_mem_slab_buf_pool;
  *max_bytes = NUM_BLOCKS * BLOCK_BYTES; /* 16 * 4096 = 65536 */
  return 0;
}

void recorder_return_contiguous_pool(void) {
  /* Restore the slab free-list pointers and clean state for audio recording */
  k_mem_slab_init(&pool, _k_mem_slab_buf_pool, BLOCK_BYTES, NUM_BLOCKS);
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
static volatile uint32_t diag_queue_high_water;
static volatile uint32_t sd_queue_high_water;
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
static volatile bool diagnostic_capture_active;

#define DIAG_TARGET_FREQUENCIES 8
#define DIAG_SCAN_RATE_HZ 8000U
#define DIAG_SCAN_SAMPLES 8192U
#define DIAG_SCAN_LOW_HZ 20U
#define DIAG_SCAN_HIGH_HZ 4000U
#define DIAG_SCAN_SECTION __attribute__((section("SRAM1")))

/* One 1.024 s decimated Stage A window; the CPU analyzes it after capture. */
static int16_t diag_scan_ring[DIAG_SCAN_SAMPLES] DIAG_SCAN_SECTION;

struct diag_scan_capture {
  uint32_t sample_rate_hz;
  uint32_t decimation;
  uint32_t samples;
  uint32_t write_index;
  uint32_t decimation_phase;
  int32_t decimation_sum;
};
struct diag_goertzel {
  float frequency_hz;
  float coeff;
  float s1;
  float s2;
};

struct diag_spectrum {
  struct diag_goertzel bins[DIAG_TARGET_FREQUENCIES];
  uint32_t samples;
};

static const float diag_target_frequencies[DIAG_TARGET_FREQUENCIES] = {
    50.0f, 60.0f, 93.75f, 100.0f, 120.0f, 150.0f, 187.5f, 200.0f};

static void diag_scan_reset(struct diag_scan_capture *s) {
  s->samples = 0;
  s->write_index = 0;
  s->decimation_phase = 0;
  s->decimation_sum = 0;
}

static void diag_scan_init(struct diag_scan_capture *s, uint32_t sample_rate) {
  memset(s, 0, sizeof(*s));
  s->sample_rate_hz = sample_rate;
  s->decimation = (sample_rate % DIAG_SCAN_RATE_HZ == 0)
                      ? sample_rate / DIAG_SCAN_RATE_HZ
                      : 0;
}

static void diag_scan_update(struct diag_scan_capture *s,
                             const int16_t *samples, uint32_t count,
                             bool contiguous) {
  if (!contiguous) diag_scan_reset(s);
  if (s->decimation == 0) return;

  for (uint32_t i = 0; i < count; i++) {
    s->decimation_sum += samples[i];
    s->decimation_phase++;
    if (s->decimation_phase == s->decimation) {
      diag_scan_ring[s->write_index] =
          (int16_t)(s->decimation_sum / (int32_t)s->decimation);
      s->write_index = (s->write_index + 1U) % DIAG_SCAN_SAMPLES;
      if (s->samples < DIAG_SCAN_SAMPLES) s->samples++;
      s->decimation_phase = 0;
      s->decimation_sum = 0;
    }
  }
}

static void diag_scan_report(const struct diag_scan_capture *s) {
  uint32_t fs = s->decimation ? s->sample_rate_hz / s->decimation : 0;
  if (s->samples < fs || fs == 0) {
    printk("  Broad Stage A Scan: unavailable (need >=1 s contiguous input; "
           "have %u samples at %u Hz)\n",
           s->samples, fs);
    return;
  }

  uint32_t start = (s->samples == DIAG_SCAN_SAMPLES) ? s->write_index : 0;
  float strongest_dbfs = -120.0f;
  float strongest_hz = 0.0f;
  uint32_t first_bin = (DIAG_SCAN_LOW_HZ * s->samples + fs - 1U) / fs;
  uint32_t last_bin = (DIAG_SCAN_HIGH_HZ * s->samples) / fs;
  if (last_bin >= s->samples / 2U) last_bin = s->samples / 2U - 1U;

  for (uint32_t bin = first_bin; bin <= last_bin; bin++) {
    float frequency = (float)bin * fs / s->samples;
    float w = 6.283185307179586f * (float)bin / s->samples;
    float coeff = 2.0f * cosf(w);
    float q1 = 0.0f, q2 = 0.0f;
    for (uint32_t i = 0; i < s->samples; i++) {
      uint32_t index = (start + i) % DIAG_SCAN_SAMPLES;
      float q0 = (float)diag_scan_ring[index] + coeff * q1 - q2;
      q2 = q1;
      q1 = q0;
    }
    float power = q1 * q1 + q2 * q2 - coeff * q1 * q2;
    float peak = (power > 0.0f) ? 2.0f * sqrtf(power) / s->samples : 0.0f;
    float half_w = 3.141592653589793f * frequency / s->sample_rate_hz;
    float response = fabsf(sinf(half_w * s->decimation) /
                           (s->decimation * sinf(half_w)));
    if (response > 1e-6f) peak /= response;
    float dbfs = peak > 0.0f ? 20.0f * log10f(peak / 32768.0f) : -120.0f;
    if (dbfs > strongest_dbfs) {
      strongest_dbfs = dbfs;
      strongest_hz = frequency;
    }
  }

  printk("  Broad Stage A Scan: %u decimated centered-PCM samples, %u Hz, "
         "rectangular window, %.3f Hz spacing, %u-%u Hz\n",
         s->samples, fs, (double)fs / s->samples, DIAG_SCAN_LOW_HZ,
         DIAG_SCAN_HIGH_HZ);
  printk("    Strongest scanned peak: %.2f Hz, peak=%7.1f dBFS "
         "(boxcar response corrected)\n",
         (double)strongest_hz, (double)strongest_dbfs);
}

static void diag_spectrum_init(struct diag_spectrum *s, uint32_t sample_rate) {
  memset(s, 0, sizeof(*s));
  for (uint32_t i = 0; i < DIAG_TARGET_FREQUENCIES; i++) {
    float w = 6.283185307179586f * diag_target_frequencies[i] / sample_rate;
    s->bins[i].frequency_hz = diag_target_frequencies[i];
    s->bins[i].coeff = 2.0f * cosf(w);
  }
}

static void diag_spectrum_update(struct diag_spectrum *s, const int16_t *samples,
                                 uint32_t count, bool contiguous) {
  if (!contiguous) {
    s->samples = 0;
    for (uint32_t i = 0; i < DIAG_TARGET_FREQUENCIES; i++) {
      s->bins[i].s1 = 0.0f;
      s->bins[i].s2 = 0.0f;
    }
  }
  for (uint32_t n = 0; n < count; n++) {
    float x = (float)samples[n];
    for (uint32_t i = 0; i < DIAG_TARGET_FREQUENCIES; i++) {
      struct diag_goertzel *b = &s->bins[i];
      float next = x + b->coeff * b->s1 - b->s2;
      b->s2 = b->s1;
      b->s1 = next;
    }
  }
  s->samples += count;
}

static void diag_spectrum_report(const struct diag_spectrum *s,
                                 uint32_t sample_rate) {
  uint32_t resolution_millihz = s->samples
                                    ? (uint32_t)(((uint64_t)sample_rate * 1000ULL) /
                                                 s->samples)
                                    : 0;
  printk("  Long Stage A Spectrum: %u contiguous centered-PCM samples, "
         "rectangular window, resolution %.3f Hz\n",
         s->samples, (double)resolution_millihz / 1000.0);
  if (s->samples == 0) {
    return;
  }
  float strongest_dbfs = -120.0f;
  float strongest_hz = 0.0f;
  for (uint32_t i = 0; i < DIAG_TARGET_FREQUENCIES; i++) {
    const struct diag_goertzel *b = &s->bins[i];
    float power = b->s1 * b->s1 + b->s2 * b->s2 - b->coeff * b->s1 * b->s2;
    float peak = (power > 0.0f) ? (2.0f * sqrtf(power) / s->samples) : 0.0f;
    float dbfs = peak > 0.0f ? 20.0f * log10f(peak / 32768.0f) : -120.0f;
    printk("    %7.2f Hz: peak=%8.2f PCM counts, peak dBFS=%7.1f\n",
           (double)b->frequency_hz, (double)peak, (double)dbfs);
    if (dbfs > strongest_dbfs) {
      strongest_dbfs = dbfs;
      strongest_hz = b->frequency_hz;
    }
  }
  printk("    Strongest requested target: %.2f Hz at %.1f dBFS peak\n",
         (double)strongest_hz, (double)strongest_dbfs);
}
#endif

/* ---- audio_sink callbacks: called from the high-priority acquisition thread ---- */
static void *sink_alloc(void *user) {
  void *blk;

  ARG_UNUSED(user);
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
  if (diagnostic_capture_active) {
    if (k_mem_slab_alloc(&diag_pool, &blk, K_NO_WAIT) != 0) {
      dma_alloc_drops++;
      drops++;
      return NULL;
    }
    return blk;
  }
#endif
  if (k_mem_slab_alloc(&pool, &blk, K_NO_WAIT) != 0) {
    dma_alloc_drops++;
    drops++;
    return NULL;
  }
  return blk;
}

static int sink_submit(void *blk, void *user) {
  ARG_UNUSED(user);
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
  if (diagnostic_capture_active) {
    if (k_msgq_put(&diag_filled_q, &blk, K_NO_WAIT) != 0) {
      queue_submit_drops++;
      drops++;
      k_mem_slab_free(&diag_pool, blk);
      return -ENOBUFS;
    }
    uint32_t queued = k_msgq_num_used_get(&diag_filled_q);
    if (queued > diag_queue_high_water) diag_queue_high_water = queued;
    uint32_t in_use = DIAG_POOL_BLOCKS - k_mem_slab_num_free_get(&diag_pool);
    if (in_use > high_water) {
      high_water = in_use;
    }
    return 0;
  }
#endif
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
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
  if (diagnostic_capture_active) {
    k_mem_slab_free(&diag_pool, blk);
  } else
#endif
  {
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
  printk("[DBG] disk_access_init = %d\n", sd_ret);
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

  printk("[DBG] fs_mount start\n");
  ret = fs_mount(&mp);
  printk("[DBG] fs_mount ret=%d\n", ret);
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
  if (!mounted) {
    return -ENODEV;
  }

  int ret = fs_unmount(&mp);
  if (ret != 0) {
    LOG_ERR("Storage recovery: filesystem unmount failed: %d", ret);
    return ret;
  }
  mounted = false;

  /* FatFs unmount powers the disk off through CTRL_DEINIT. Reinitialize the
   * disk before mounting the volume again; do not retry file operations on
   * the stale mounted filesystem after an SDMMC timeout. */
  ret = disk_access_init("SD");
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

static int generate_recording_filename(char *name, size_t len,
                                       bool *rtc_valid) {
  *rtc_valid = false;
  if (rtc_calendar_is_valid()) {
    struct rtc_calendar_time td;
    if (rtc_calendar_get(&td) == 0) {
      int ret = rtc_calendar_generate_filename(name, len, &td);
      if (ret == 0) {
        *rtc_valid = true;
        return 0;
      }
    }
  }
  return next_filename(name, len);
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

#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
static uint32_t drain_diagnostic_queue(void) {
  void *blk;
  uint32_t count = 0;

  while (k_msgq_get(&diag_filled_q, &blk, K_NO_WAIT) == 0) {
    k_mem_slab_free(&diag_pool, blk);
    count++;
  }
  return count;
}
#endif

int recorder_verify_file(const char *path, uint32_t expected_rate,
                         uint16_t expected_bits, uint64_t expected_data_bytes) {
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

  if (r != sizeof(hdr)) {
    LOG_ERR("Verify read %s failed: %d", path, (int)r);
    return -EIO;
  }

  ret = wav_verify_header(hdr, expected_rate, 1, expected_bits,
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

int recorder_record(uint32_t seconds, struct rec_stats *out) {
  return recorder_record_session(seconds, 1, REC_MODE_NORMAL, out);
}

int recorder_record_cycle(uint32_t seconds, uint32_t cycle,
                          struct rec_stats *out) {
  return recorder_record_session(seconds, cycle, REC_MODE_NORMAL, out);
}

int recorder_record_session(uint32_t seconds, uint32_t cycle,
                            enum recorder_mode mode, struct rec_stats *out) {
  if (benchmark_is_running()) {
    LOG_ERR("Cannot start recording: benchmark is currently running");
    if (out) {
      memset(out, 0, sizeof(*out));
      out->err = -EBUSY;
    }
    return -EBUSY;
  }

  struct rec_stats st;
  memset(&st, 0, sizeof(st));
  if (out) {
    memset(out, 0, sizeof(*out));
  }
  recorder_abort_requested = false;
  recorder_active = true;

  bool apply_dsp = true;
  bool is_test_signal = false;
#if CONFIG_AUDIO_INTEGRITY_REPLAY
  bool is_integrity_replay = (mode == REC_MODE_INTEGRITY_REPLAY);
  bool is_integrity_audit = is_integrity_replay ||
                            (mode == REC_MODE_INTEGRITY_CAPTURE);
#endif
  const char *mode_str = "FILTERED";

  switch (mode) {
  case REC_MODE_RAW:
    apply_dsp = false;
    mode_str = "RAW (Unfiltered ADC PCM)";
    break;
  case REC_MODE_FILTERED:
    apply_dsp = true;
    mode_str = "FILTERED (Active DSP Chain)";
    break;
  case REC_MODE_COMPARE:
    if ((cycle % 2) == 1) {
      apply_dsp = false;
      mode_str = "RAW (Comparison Cycle 1)";
    } else {
      apply_dsp = true;
      mode_str = "FILTERED (Comparison Cycle 2)";
    }
    break;
  case REC_MODE_STRESS:
    apply_dsp = true;
    mode_str = "STRESS TEST";
    break;
  case REC_MODE_TEST_SIGNAL:
    apply_dsp = true;
    is_test_signal = true;
    mode_str = "TEST SIGNAL (Synthetic Calibration Tone)";
    break;
#if CONFIG_AUDIO_INTEGRITY_REPLAY
  case REC_MODE_INTEGRITY_REPLAY:
    apply_dsp = false;
    mode_str = "TEST ONLY PCM REPLAY (ADC values substituted; DSP/gain bypassed)";
    break;
  case REC_MODE_INTEGRITY_CAPTURE:
    apply_dsp = true;
    mode_str = "INTEGRITY ADC CAPTURE (production DSP/gain; PCM readback CRC)";
    break;
#endif
  case REC_MODE_NORMAL:
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
  printk("[PHASE 3] DSP initialized (%s, gain %.2fx)\n", st.dsp_mode_str,
         (double)st.digital_gain);

  struct fs_file_t f;
  static uint8_t hdr[WAV_HEADER_BYTES] __aligned(4);
  int ret;

  if (!mounted) {
    st.err = -ENODEV;
    recorder_active = false;
    if (out)
      *out = st;
    return -ENODEV;
  }
  st.sample_rate = audio_in_rate();
  st.sample_bits = CONFIG_AUDIO_SAMPLE_BITS;
  st.channels = 1;

  st.expected_samples = (uint64_t)seconds * st.sample_rate;
  const uint64_t want_blocks =
      (st.expected_samples + BLOCK_SAMPLES - 1) / BLOCK_SAMPLES;
  st.expected_blocks = (uint32_t)want_blocks;
  const uint64_t exact_bytes = st.expected_samples * SAMPLE_BYTES;
  const uint64_t prealloc_bytes = want_blocks * BLOCK_BYTES;
  ARG_UNUSED(prealloc_bytes);
#if CONFIG_AUDIO_INTEGRITY_REPLAY
  st.integrity_replay = is_integrity_replay;
  st.integrity_audit = is_integrity_audit;
  if (is_integrity_replay) {
    st.integrity_expected_crc32 = integrity_expected_crc(st.expected_samples);
    printk("[INTEGRITY] Expected PCM16 LE payload: %llu samples, %llu bytes, CRC32=%08x\n",
           st.expected_samples, exact_bytes, st.integrity_expected_crc32);
  }
#endif

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

  wav_fill_header(hdr, st.sample_rate, 1, CONFIG_AUDIO_SAMPLE_BITS, 0);
  ret = fs_write(&f, hdr, WAV_HEADER_BYTES);
  if (ret != WAV_HEADER_BYTES) {
    fs_close(&f);
    fs_unlink(st.file);
    st.file[0] = '\0';
    st.err = ret < 0 ? ret : -EIO;
    st.sd_write_error = st.err;
    if (out)
      *out = st;
    return st.err;
  }
  printk("[PHASE 4] WAV file opened: %s (header %u bytes written)\n", st.file,
         WAV_HEADER_BYTES);

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

  const uint32_t buffer_ms =
      (uint32_t)((uint64_t)NUM_BLOCKS * BLOCK_SAMPLES * 1000 / st.sample_rate);
  LOG_INF("recording %u s to %s at %u Hz, %u-bit mono (expect %llu samples, %u "
          "blocks, %llu bytes, pool %u = %u ms buffer)",
          seconds, st.file, st.sample_rate, CONFIG_AUDIO_SAMPLE_BITS,
          st.expected_samples, st.expected_blocks, exact_bytes, NUM_BLOCKS,
          buffer_ms);

#if CONFIG_AUDIO_INTEGRITY_REPLAY
  ret = is_integrity_replay ? audio_in_set_integrity_replay(true) : 0;
#else
  ret = 0;
#endif
  if (ret != 0) {
    fs_close(&f);
    fs_unlink(st.file);
    st.file[0] = '\0';
    st.err = ret;
    if (out) *out = st;
    return ret;
  }
#if CONFIG_AUDIO_INTEGRITY_REPLAY
  uint32_t pipeline_crc_state = 0xFFFFFFFFU;
  uint64_t pipeline_crc_bytes = 0;
#endif
  ret = audio_in_start();
  if (ret) {
#if CONFIG_AUDIO_INTEGRITY_REPLAY
    if (is_integrity_replay) audio_in_set_integrity_replay(false);
#endif
    LOG_ERR("audio_in_start failed: %d", ret);
    fs_close(&f);
    fs_unlink(st.file);
    st.file[0] = '\0';
    st.err = ret;
    if (out) {
      *out = st;
    }
    return ret;
  }

  const uint64_t t_start_cyc = k_cycle_get_64();
  uint64_t t_first_block_cyc = 0;
  uint64_t t_last_block_cyc = 0;
  int err = 0;
  uint64_t samples_remaining = st.expected_samples;

  while (samples_remaining > 0) {
    if (recorder_abort_requested || console_check_abort_requested()) {
      recorder_abort_requested = true;
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
    uint32_t bytes_to_write = samples_in_block * SAMPLE_BYTES;

    if (is_test_signal) {
#if CONFIG_AUDIO_SAMPLE_BITS == 32
      dsp_generate_test_signal_i32((int32_t *)blk, samples_in_block,
                                   st.sample_rate);
#else
      dsp_generate_test_signal_i16((int16_t *)blk, samples_in_block,
                                   st.sample_rate);
#endif
    }

    if (apply_dsp) {
#if CONFIG_AUDIO_SAMPLE_BITS == 32
      dsp_process_i32((int32_t *)blk, samples_in_block);
#else
      dsp_process_i16((int16_t *)blk, samples_in_block);
#endif
    }

#if CONFIG_AUDIO_INTEGRITY_REPLAY
    /* This is the finalized PCM immediately before PSRAM/SD queue handoff. */
    if (is_integrity_audit) {
      pipeline_crc_state = crc32_update(pipeline_crc_state,
                                         (const uint8_t *)blk, bytes_to_write);
      pipeline_crc_bytes += bytes_to_write;
    }
#endif

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
      if (st.blocks_written == 1) {
        printk(
            "[PHASE 8] First block accepted and buffered in PSRAM (%u bytes)\n",
            bytes_to_write);
      }
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
      if (st.blocks_written == 1) {
        printk("[PHASE 8] First block accepted and written (%u bytes, queued "
               "to SD writer)\n",
               bytes_to_write);
      }
      st.samples_accepted += samples_in_block;
      samples_remaining -= samples_in_block;
    }
  }

  const uint64_t t_stop_cyc = k_cycle_get_64();
  uint64_t wall_cyc =
      (t_stop_cyc >= t_start_cyc) ? (t_stop_cyc - t_start_cyc) : 0;
  uint64_t session_wall_us = k_cyc_to_us_floor64(wall_cyc);

  audio_in_stop();
#if CONFIG_AUDIO_INTEGRITY_REPLAY
  if (is_integrity_replay) audio_in_set_integrity_replay(false);
#endif
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
  st.samples_written = st.bytes_written / SAMPLE_BYTES;
  st.write_count = sd_writer_write_count;
  st.worst_write_us = sd_writer_worst_write_us;
  st.total_write_us = sd_writer_write_us_sum;
  st.bytes_per_write_avg =
      (sd_writer_write_count > 0)
          ? (uint32_t)(sd_writer_bytes_written / sd_writer_write_count)
          : 0;
  st.sd_write_error = sd_writer_err;
  st.sd_failed_samples = sd_writer_failed_bytes / SAMPLE_BYTES;
#if CONFIG_AUDIO_INTEGRITY_REPLAY
  st.integrity_pipeline_crc32 = ~pipeline_crc_state;
  if (is_integrity_audit && !is_integrity_replay) {
    /* For live ADC, the finalized pre-writer PCM stream is the reference. */
    st.integrity_expected_crc32 = st.integrity_pipeline_crc32;
  }
#endif

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

  wav_fill_header(hdr, st.sample_rate, 1, CONFIG_AUDIO_SAMPLE_BITS, data_bytes);
  int finalize_ret = fs_seek(&f, 0, FS_SEEK_SET);
  if (finalize_ret == 0) {
    ssize_t header_written = fs_write(&f, hdr, WAV_HEADER_BYTES);
    if (header_written != WAV_HEADER_BYTES)
      finalize_ret = (header_written < 0) ? (int)header_written : -EIO;
  }
  if (finalize_ret == 0)
    finalize_ret = fs_truncate(&f, WAV_HEADER_BYTES + data_bytes);
  if (finalize_ret == 0)
    finalize_ret = fs_sync(&f);
  int close_ret = fs_close(&f);
  if (finalize_ret == 0) finalize_ret = close_ret;
  if (finalize_ret != 0) {
    LOG_ERR("WAV finalize/sync/close failed: %d", finalize_ret);
    if (err == 0) err = finalize_ret;
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

  /* Verify the file on disk if audio frames were written */
  if (st.bytes_written > 0) {
    int v_ret = recorder_verify_file(
        st.file, st.sample_rate, CONFIG_AUDIO_SAMPLE_BITS, st.bytes_written);
    st.wav_verified = (v_ret == 0);
    if (v_ret != 0 && err == 0) {
      err = v_ret;
    }
  }
  struct fs_dirent file_ent;
  if (fs_stat(st.file, &file_ent) == 0) {
    st.file_size_bytes = file_ent.size;
  }

#if CONFIG_AUDIO_INTEGRITY_REPLAY
  if (is_integrity_audit) {
    uint64_t payload_bytes = 0;
    int crc_ret = integrity_readback_crc(st.file,
                                         &st.integrity_readback_crc32,
                                         &payload_bytes);
    if (crc_ret == 0) {
      st.integrity_readback_bytes = payload_bytes;
      st.integrity_readback_samples = payload_bytes / SAMPLE_BYTES;
      st.integrity_readback_blocks = (uint32_t)((st.integrity_readback_samples +
                                     BLOCK_SAMPLES - 1U) / BLOCK_SAMPLES);
      st.integrity_crc_match =
          (st.integrity_expected_crc32 == st.integrity_readback_crc32) &&
          (st.integrity_expected_crc32 == st.integrity_pipeline_crc32) &&
          (pipeline_crc_bytes == exact_bytes) &&
          (payload_bytes == exact_bytes) &&
          st.wav_verified &&
          (st.file_size_bytes == WAV_HEADER_BYTES + exact_bytes) &&
          (st.integrity_readback_samples == st.expected_samples) &&
          (st.integrity_readback_blocks == st.expected_blocks);
    } else if (err == 0) {
      err = crc_ret;
    }
    printk("[INTEGRITY] CRC expected=%08x pipeline=%08x readback=%08x; bytes=%llu/%llu samples=%llu/%llu blocks=%u/%u; %s\n",
           st.integrity_expected_crc32, st.integrity_pipeline_crc32,
           st.integrity_readback_crc32, st.integrity_readback_bytes, exact_bytes,
           st.integrity_readback_samples, st.expected_samples,
           st.integrity_readback_blocks, st.expected_blocks,
           st.integrity_crc_match ? "PASS" : "FAIL");
  }
#endif

  st.queue_high_water = audio_queue_high_water;
  st.pool_high_water = high_water;
  st.sd_queue_high_water = sd_queue_high_water;
  st.err = err;

  if (out) {
    *out = st;
  }
  recorder_active = false;
  return err;
}

#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
int recorder_run_stage_diagnostics(uint32_t seconds) {
  if (seconds == 0)
    seconds = 5;
  if (recorder_active) {
    LOG_ERR("Cannot run stage diagnostic while recording is active");
    return -EBUSY;
  }
  uint32_t rate = audio_in_rate();
  if (rate == 0) return -EINVAL;
  uint64_t requested_samples = (uint64_t)seconds * rate;
  uint64_t total_blocks_64 =
      (requested_samples + DIAG_BLOCK_SAMPLES - 1U) / DIAG_BLOCK_SAMPLES;
  if (total_blocks_64 > UINT32_MAX ||
      total_blocks_64 * DIAG_BLOCK_SAMPLES > UINT32_MAX) {
    return -ERANGE;
  }
  int ret = audio_in_set_block_size(DIAG_BLOCK_SAMPLES);
  if (ret) {
    LOG_ERR("Unable to select diagnostic block size %u: %d",
            DIAG_BLOCK_SAMPLES, ret);
    return ret;
  }
  diagnostic_capture_active = true;
  uint32_t total_blocks = (uint32_t)total_blocks_64;
  float f_block = (float)rate / (float)DIAG_BLOCK_SAMPLES;
  uint32_t deadline_us =
      (uint32_t)((uint64_t)DIAG_BLOCK_SAMPLES * 1000000ULL / rate);

  struct stage_metrics sm[5];
  dsp_stage_metrics_init(&sm[0], "Stage A (Centered PCM)", rate, DIAG_BLOCK_SAMPLES);
  dsp_stage_metrics_init(&sm[1], "Stage B (After DC)", rate, DIAG_BLOCK_SAMPLES);
  dsp_stage_metrics_init(&sm[2], "Stage C (Post-HPF)", rate, DIAG_BLOCK_SAMPLES);
  dsp_stage_metrics_init(&sm[3], "Stage D (Post-LPF)", rate, DIAG_BLOCK_SAMPLES);
  dsp_stage_metrics_init(&sm[4], "Stage E (Final PCM)", rate, DIAG_BLOCK_SAMPLES);

  dsp_reset();
  dsp_reset_all_clipping_counts();
  drops = 0;
  dma_alloc_drops = 0;
  queue_submit_drops = 0;
  high_water = 0;
  diag_queue_high_water = 0;
  drain_queue();
  drain_diagnostic_queue();

  printk("\n==================================================================="
         "======================\n");
  printk("               INTERMEDIATE SIGNAL STAGE ISOLATION DIAGNOSTIC (%u s) "
         "                    \n",
         seconds);
  printk("  MCU / ADC         : STM32L476RG ADC1 + TIM6 TRGO + DMA1 Ch1\n");
  printk("  Sampling Rate     : %u Hz (Block size: %u samples = %.2f ms / %.4f "
         "Hz)\n",
         rate, DIAG_BLOCK_SAMPLES,
         (double)DIAG_BLOCK_SAMPLES * 1000.0 / rate,
         (double)f_block);
  printk("  Block Harmonics   : 1x %.3f Hz, 2x %.3f Hz, 3x %.3f Hz, 4x %.3f Hz\n",
         (double)f_block, (double)f_block * 2.0,
         (double)f_block * 3.0, (double)f_block * 4.0);
  printk(
      "  Block Deadline    : %u us per block (Real-time latency requirement)\n",
      deadline_us);
  printk("  Testing Stages    : A: Raw ADC -> B: DC Block -> C: HPF/Notch -> "
         "D: LPF -> E: Final PCM\n");
  printk("  Measurement Mode  : Real-time In-RAM Bounded Analysis (SDMMC card "
         "quiescent / no writes)\n");
  printk("====================================================================="
         "====================\n");

  int64_t start_ms = k_uptime_get();
  ret = audio_in_start();
  if (ret) {
    LOG_ERR("audio_in_start failed: %d", ret);
    diagnostic_capture_active = false;
    (void)audio_in_set_block_size(BLOCK_SAMPLES);
    return ret;
  }

  uint32_t blocks_read = 0;
  uint32_t last_drops = 0;
  void *ref_blk = NULL;
  bool ref_blk_contiguous = false;
  struct diag_spectrum long_spectrum;
  diag_spectrum_init(&long_spectrum, rate);
  struct diag_scan_capture broad_scan;
  diag_scan_init(&broad_scan, rate);
  uint32_t last_event_misses = 0;
  struct biquad_df2t blk_pre_stages[6];
  struct biquad_df2t ref_pre_stages[6];

  uint32_t min_proc_us = UINT32_MAX;
  uint32_t worst_proc_us = 0;
  uint64_t total_proc_us = 0;

  while (blocks_read < total_blocks) {
    if (console_check_abort_requested()) {
      printk("\nDiagnostic aborted by user.\n");
      break;
    }
    void *blk;
    if (k_msgq_get(&diag_filled_q, &blk, K_SECONDS(2)) != 0) {
      LOG_ERR("Diagnostic timeout waiting for block");
      break;
    }
    blocks_read++;

    uint32_t cur_drops = drops;
    uint32_t cur_event_misses = audio_in_get_missed_event_count();
    bool is_contiguous = (blocks_read == 1)
                             ? (cur_event_misses == 0)
                             : (cur_drops == last_drops &&
                                cur_event_misses == last_event_misses);
    last_drops = cur_drops;
    last_event_misses = cur_event_misses;

#if CONFIG_AUDIO_SAMPLE_BITS == 16
    diag_spectrum_update(&long_spectrum, (const int16_t *)blk,
                         DIAG_BLOCK_SAMPLES, is_contiguous);
    diag_scan_update(&broad_scan, (const int16_t *)blk,
                     DIAG_BLOCK_SAMPLES, is_contiguous);
#endif

    dsp_get_stage_filters(blk_pre_stages);

    uint64_t t0 = k_cycle_get_64();
#if CONFIG_AUDIO_SAMPLE_BITS == 16
    dsp_process_stage_diagnostics_realtime((const int16_t *)blk,
                                           DIAG_BLOCK_SAMPLES, sm,
                                           is_contiguous);
#endif
    uint64_t t1 = k_cycle_get_64();
    uint32_t proc_us =
        (uint32_t)k_cyc_to_us_floor64((t1 >= t0) ? (t1 - t0) : 0);

    if (proc_us < min_proc_us)
      min_proc_us = proc_us;
    if (proc_us > worst_proc_us)
      worst_proc_us = proc_us;
    total_proc_us += proc_us;

    if (ref_blk) {
      k_mem_slab_free(&diag_pool, ref_blk);
    }
    ref_blk = blk;
    ref_blk_contiguous = is_contiguous;
    memcpy(ref_pre_stages, blk_pre_stages, sizeof(ref_pre_stages));
  }

  int64_t elapsed_ms = k_uptime_get() - start_ms;

  audio_in_stop();
  struct audio_dma_stats final_dma_stats = {0};
  audio_in_get_dma_stats(&final_dma_stats);
  uint32_t queued_discarded = drain_diagnostic_queue();

  uint32_t act_drops = drops;
  uint32_t hw = high_water;
  uint32_t prod_blocks = final_dma_stats.callbacks;
  uint32_t avg_proc_us =
      blocks_read ? (uint32_t)(total_proc_us / blocks_read) : 0;
  if (min_proc_us == UINT32_MAX)
    min_proc_us = 0;
  float cpu_load_pct =
      deadline_us ? ((float)avg_proc_us * 100.0f / (float)deadline_us) : 0.0f;

  if (act_drops == 0 && ref_blk && ref_blk_contiguous) {
#if CONFIG_AUDIO_SAMPLE_BITS == 16
    dsp_run_post_capture_spectral_analysis((const int16_t *)ref_blk,
                                           DIAG_BLOCK_SAMPLES, ref_pre_stages,
                                           sm);
#endif
  }

  if (ref_blk) {
    k_mem_slab_free(&diag_pool, ref_blk);
    ref_blk = NULL;
  }

  printk("  Diagnostic Blocks : %u DSP processed, %u producer block drops "
         "(Pool high-water: "
         "%u/%u blocks)\n",
         blocks_read, act_drops, hw, DIAG_POOL_BLOCKS);
  printk("  Producer Blocks   : %u HT/TC callbacks; %u DSP processed; %u "
         "queued after target and discarded\n",
         prod_blocks, blocks_read, queued_discarded);
  printk(
      "  Memory Pool Status: High-water %u/%u blocks (Peak usage: %u bytes)\n",
      hw, DIAG_POOL_BLOCKS, hw * DIAG_BLOCK_BYTES);
  printk("  Queue / Block Drops: diag queue high-water=%u/%u; slab failures=%u; "
         "queue submit drops=%u; total producer drops=%u\n",
         diag_queue_high_water, DIAG_POOL_BLOCKS, dma_alloc_drops,
         queue_submit_drops, act_drops);
  printk("  Real-time Latency : Min: %u us, Avg: %u us, Worst: %u us "
         "(Deadline: %u us, CPU Load: %.1f%%)\n",
         min_proc_us, avg_proc_us, worst_proc_us, deadline_us,
         (double)cpu_load_pct);
  struct audio_dma_stats dma_stats = final_dma_stats;
  struct audio_diag adc_diag = {0};
  audio_in_get_diag(&adc_diag);
  uint32_t dma_ht = 0, dma_tc = 0;
  audio_in_get_callback_counts(&dma_ht, &dma_tc);
  uint32_t dma_interval_min_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_min_cycles);
  uint32_t dma_interval_avg_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_avg_cycles);
  uint32_t dma_interval_max_us = (uint32_t)k_cyc_to_us_floor64(
      dma_stats.callback_interval_max_cycles);
  printk("  DMA / Acquisition : HT=%u TC=%u ADC_OVR=%u DMA_errors=%u missed=%u "
         "samples=%llu accounted=%u processed=%u lost=%u\n",
         dma_ht, dma_tc, dma_stats.adc_overruns,
         dma_stats.callback_errors, dma_stats.events_missed,
         (unsigned long long)dma_stats.samples_acquired,
         dma_stats.samples_accounted, dma_stats.samples_processed,
         dma_stats.samples_lost);
#if CONFIG_AUDIO_SRC_ADC_STM32L4
  printk("  Raw ADC Codes     : N=%u min=%u max=%u mean=%u p-p=%u RMS/DC=%u "
         "AC_RMS=%u codes (%u mV), RMS dBFS=%d.%d rail0=%u railMax=%u\n",
         adc_diag.total_samples, adc_diag.raw_min, adc_diag.raw_max,
         adc_diag.raw_midpoint, adc_diag.peak_to_peak, adc_diag.raw_rms_code,
         adc_diag.raw_ac_rms_code, adc_diag.raw_rms_mv,
         adc_diag.raw_rms_dbfs_x10 / 10,
         abs(adc_diag.raw_rms_dbfs_x10 % 10),
         adc_diag.clipping_low_count, adc_diag.clipping_high_count);
#else
  printk("  Raw ADC Codes     : not applicable for synthetic input\n");
#endif
  diag_spectrum_report(&long_spectrum, rate);
  diag_scan_report(&broad_scan);
  printk("  Callback Interval : min/avg/max=%u/%u/%u us; acquisition "
         "wake latency max=%u cycles, process avg/max=%u/%u cycles; queue "
	 "high-water=%u/%u\n",
         dma_interval_min_us, dma_interval_avg_us, dma_interval_max_us,
         dma_stats.acquisition_max_latency_cycles,
         dma_stats.acquisition_process_avg_cycles,
         dma_stats.acquisition_process_max_cycles,
         dma_stats.event_queue_high_water, AUDIO_DMA_EVENT_QUEUE_DEPTH);
  printk("  Elapsed Time      : %u ms (Target: %u s)\n", (uint32_t)elapsed_ms,
         seconds);
  if (sm[0].spectral_valid) {
    printk("  Final-block Stages: %u contiguous samples for the 1x/2x "
           "block-rate Goertzel comparison\n",
           sm[0].spectral_samples);
  } else {
    printk("  Final-block Stages: REJECTED (spectral_valid = false: %u buffer "
           "drops)\n",
           act_drops);
  }

  for (int s = 0; s < 5; s++) {
    dsp_stage_metrics_finish(&sm[s]);
  }

  printk("\nStage (PCM counts)  | Count  | Min    | Max    | P-P    | Mean   | RMS/DC | AC RMS | Near FS "
         "| Excursions*| Bnd Delta | In Delta | final 1x   | final 2x\n");
  printk("--------------------+--------+--------+--------+--------+--------+--------+--------+---------+--"
         "----------+-----------+----------+------------+------------\n");
  for (int s = 0; s < 5; s++) {
    int32_t mean = sm[s].sample_count
                       ? (int32_t)(sm[s].sum / (int64_t)sm[s].sample_count)
                       : 0;
    uint32_t rms =
        sm[s].sample_count
            ? (uint32_t)sqrtf((float)(sm[s].sum_sq / sm[s].sample_count))
            : 0;
    int64_t mean_sq = sm[s].sample_count
                          ? sm[s].sum / (int64_t)sm[s].sample_count
                          : 0;
    uint64_t mean_square = (uint64_t)(mean_sq * mean_sq);
    uint64_t mean_rms_sq = sm[s].sample_count
                               ? sm[s].sum_sq / sm[s].sample_count
                               : 0;
    uint32_t ac_rms = (uint32_t)sqrtf((float)(mean_rms_sq > mean_square
                                                   ? mean_rms_sq - mean_square
                                                   : 0));
    uint32_t peak_to_peak = sm[s].sample_count
                                ? (uint32_t)((int64_t)sm[s].max - sm[s].min)
                                : 0;
    uint32_t bnd_avg =
        sm[s].boundary_count
            ? (uint32_t)(sm[s].boundary_delta_sum / sm[s].boundary_count)
            : 0;
    uint32_t in_avg =
        sm[s].inside_delta_count
            ? (uint32_t)(sm[s].inside_delta_sum / sm[s].inside_delta_count)
            : 0;

    char nfs_buf[16];
    if (s == 0 || s == 4) {
      snprintk(nfs_buf, sizeof(nfs_buf), "%7u", sm[s].near_full_scale);
    } else {
      snprintk(nfs_buf, sizeof(nfs_buf), "  N/A  ");
    }

    char bnd_buf[16];
    if (sm[s].boundary_count > 0) {
      snprintk(bnd_buf, sizeof(bnd_buf), "%5u/%-3u", bnd_avg,
               sm[s].boundary_delta_max);
    } else {
      snprintk(bnd_buf, sizeof(bnd_buf), "  N/A    ");
    }

    char in_buf[16];
    if (sm[s].inside_delta_count > 0) {
      snprintk(in_buf, sizeof(in_buf), "%4u/%-4u", in_avg,
               sm[s].inside_delta_max);
    } else {
      snprintk(in_buf, sizeof(in_buf), "  N/A   ");
    }

    char spec_buf1[16];
    char spec_buf2[16];
    if (sm[s].spectral_valid) {
      snprintk(spec_buf1, sizeof(spec_buf1), "%6.1f dBFS",
               (double)sm[s].g_mag[0]);
      snprintk(spec_buf2, sizeof(spec_buf2), "%6.1f dBFS",
               (double)sm[s].g_mag[1]);
    } else {
      snprintk(spec_buf1, sizeof(spec_buf1), "  INVALID ");
      snprintk(spec_buf2, sizeof(spec_buf2), "  INVALID ");
    }

    printk("%-19s | %6u | %6d | %6d | %6u | %+6d | %6u | %6u | %-7s | %10u | %-9s | %-8s | %-10s "
           "| %-10s\n",
           sm[s].name, sm[s].sample_count, sm[s].min, sm[s].max, peak_to_peak,
           mean, rms, ac_rms, nfs_buf,
           sm[s].clip_count, bnd_buf, in_buf, spec_buf1, spec_buf2);
  }
  printk("====================================================================="
         "====================\n");
  printk("  * RMS/DC includes DC offset; AC RMS is RMS after subtracting the "
         "stage mean.\n");
  printk("  * Final-block spectra and Long Stage A spectra report peak-amplitude "
         "dBFS (20 log10(peak / 32768)); PCM counts are centered signed 16-bit.\n");
  printk("  * Near FS:    Samples with |v| >= 31128 (within 5%% of PCM full "
         "scale).\n");
  printk("  * Excursions: Stage A = PCM saturation after conversion (hardware "
         "ADC rail counts are printed above);\n");
  printk("                Stages B..D = Intermediate float headroom overshoot "
         "(> +-32767);\n");
  printk("                Stage E = Final digital PCM clamps (clamped to "
         "[-32768, +32767]).\n");
  printk("  Note: Stages B..D use 32-bit floating-point arithmetic with full "
         "headroom (>140 dB dynamic range).\n");
  printk("        Values beyond +-32767 reflect filter resonance/overshoot "
         "without integer wraparound or precision loss.\n");
  printk("---------------------------------------------------------------------"
         "--------------------\n");

  diagnostic_capture_active = false;
  ret = audio_in_set_block_size(BLOCK_SAMPLES);
  if (ret) {
    LOG_ERR("Unable to restore production block size %u: %d",
            BLOCK_SAMPLES, ret);
    return ret;
  }

  /* Diagnostic Automated Verdict */
  printk("DIAGNOSTIC VERDICT & ROOT-CAUSE ANALYSIS:\n");
  bool deadline_violated =
      (worst_proc_us > deadline_us) || (avg_proc_us > deadline_us);

  if (act_drops > 0) {
    printk("  [FAIL] Real-time buffer overflow detected: %u blocks dropped!\n",
           act_drops);
    printk("         Producer generated: %u blocks, Consumer processed: %u "
           "blocks.\n",
           prod_blocks, blocks_read);
    printk("         Spectral analysis REJECTED (spectral_valid = false) to "
           "prevent discontinuous data as noise evidence.\n");
    printk("         Boundary delta calculation skipped dropped-block gaps to "
           "preserve metric integrity.\n");
  } else if (deadline_violated) {
    printk("  [FAIL] Real-time processing deadline violated!\n");
    printk("         Average latency: %u us, Worst latency: %u us (Block "
           "deadline: %u us, CPU Load: %.1f%%).\n",
           avg_proc_us, worst_proc_us, deadline_us, (double)cpu_load_pct);
    printk("         Buffer pool temporarily masked the overrun (high-water: "
           "%u/%u blocks),\n",
           hw, DIAG_POOL_BLOCKS);
    printk("         but longer recording sessions will exhaust available RAM "
           "and drop blocks.\n");
  } else {
    int32_t margin_us = (int32_t)deadline_us - (int32_t)worst_proc_us;
    printk("  [PASS] Real-time processing strictly bounded: 0 blocks dropped "
           "(Pool high-water: %u/%u blocks).\n",
           hw, DIAG_POOL_BLOCKS);
    printk("         Latency: Min=%u us, Avg=%u us, Worst=%u us (Deadline: %u "
           "us, Margin: +%d us, CPU load: %.1f%%).\n",
           min_proc_us, avg_proc_us, worst_proc_us, deadline_us, margin_us,
           (double)cpu_load_pct);

    float a_1x = sm[0].g_mag[0];
    float e_1x = sm[4].g_mag[0];
    float d_diff = fabsf(e_1x - a_1x);

    if (a_1x > -60.0f && d_diff < 3.0f) {
      printk("  [OBSERVATION] Block-frequency components are visible in the "
             "centered PCM Stage A at %.1f Hz and %.1f Hz.\n",
             (double)f_block, (double)(2.0f * f_block));
      printk("           This alone does not establish an electrical or software "
             "cause; compare raw ADC and WAV data.\n");
    } else if (e_1x > (a_1x + 6.0f)) {
      printk("  [RESULT] DSP pipeline amplified block-frequency tone by %.1f "
             "dB.\n",
             (double)(e_1x - a_1x));
    } else {
      printk("  [PASS] Stage A shows clean spectrum (1x f_blk = %.1f dBFS, 2x "
             "f_blk = %.1f dBFS).\n",
             (double)a_1x, (double)sm[0].g_mag[1]);
    }

    uint32_t bnd0 =
        sm[0].boundary_count
            ? (uint32_t)(sm[0].boundary_delta_sum / sm[0].boundary_count)
            : 0;
    uint32_t in0 =
        sm[0].inside_delta_count
            ? (uint32_t)(sm[0].inside_delta_sum / sm[0].inside_delta_count)
            : 0;
    if (bnd0 > in0 * 2 && bnd0 > 500) {
      printk("  [WARN] Block boundary delta (%u) is significantly higher than "
             "inside delta (%u).\n",
             bnd0, in0);
    } else {
      printk("  [PASS] Block boundary delta (%u) is consistent with continuous "
             "intra-block deltas (%u).\n",
             bnd0, in0);
      printk("         Zero evidence of buffer truncation, skipped samples, or "
             "block discontinuities.\n");
    }
  }

  uint32_t rmsA =
      sm[0].sample_count
          ? (uint32_t)sqrtf((float)(sm[0].sum_sq / sm[0].sample_count))
          : 0;
  if (sm[0].clip_count > 0 ||
      sm[0].near_full_scale > (sm[0].sample_count / 10)) {
    printk("  [WARN] PCM NEAR-FULL-SCALE OR SATURATION DETECTED IN STAGE A:\n");
    printk("         PCM saturation samples: %u (ADC rail hits: low=%u high=%u).\n",
           sm[0].clip_count,
           adc_diag.clipping_low_count, adc_diag.clipping_high_count);
    printk("         Near-full-scale samples: %u (%.1f%% of samples within 5%% "
           "of rails).\n",
           sm[0].near_full_scale,
           (double)sm[0].near_full_scale * 100.0 /
               (sm[0].sample_count ? sm[0].sample_count : 1));
    printk(
        "         PCM RMS/DC: %u counts, Peak: [%d .. %d], Mean: %+d counts.\n",
        rmsA, sm[0].min, sm[0].max,
        (int32_t)(sm[0].sum /
                  (int64_t)(sm[0].sample_count ? sm[0].sample_count : 1)));
    printk("         Check the separate raw-code rail-hit counts above before "
           "attributing this to ADC clipping.\n");
  }
  printk("====================================================================="
         "====================\n\n");
  return 0;
}
#endif
