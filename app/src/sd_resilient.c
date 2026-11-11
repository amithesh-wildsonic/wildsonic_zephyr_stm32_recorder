/* SPDX-License-Identifier: Apache-2.0 */
#include "sd_resilient.h"
#include "sd_fault_inject.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#if defined(CONFIG_SOC_SERIES_STM32L4X)
#include <stm32l4xx_ll_bus.h>
#include <stm32l4xx_hal_rcc.h>
#endif

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(sd_resilient, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(sd_resilient, LOG_LEVEL_WRN);
#endif

/* Mutex for serialized single multi-line diagnostic logging */
K_MUTEX_DEFINE(s_resilient_log_mutex);

/* Spinlock protecting statistics and health state machine */
static struct k_spinlock s_stats_lock;
static struct sd_resilient_stats s_stats = {0};
static enum storage_health_state s_health = STORAGE_HEALTH_HEALTHY;

/* Ring buffer for gap tracking */
static struct storage_gap_record s_gap_ring[STORAGE_GAP_RING_CAPACITY];
static uint32_t s_gap_count = 0;
static uint32_t s_gap_head = 0;
static struct k_spinlock s_gap_lock;

/* Forward declarations of disk access operations */
static int resilient_disk_init(struct disk_info *disk);
static int resilient_disk_status(struct disk_info *disk);
static int resilient_disk_read(struct disk_info *disk, uint8_t *data_buf,
                               uint32_t start_sector, uint32_t num_sector);
static int resilient_disk_write(struct disk_info *disk, const uint8_t *data_buf,
                                uint32_t start_sector, uint32_t num_sector);
static int resilient_disk_erase(struct disk_info *disk, uint32_t start_sector,
                                uint32_t num_sector);
static int resilient_disk_ioctl(struct disk_info *disk, uint8_t cmd, void *buff);

static const struct disk_operations s_resilient_ops = {
    .init = resilient_disk_init,
    .status = resilient_disk_status,
    .read = resilient_disk_read,
    .write = resilient_disk_write,
    .erase = resilient_disk_erase,
    .ioctl = resilient_disk_ioctl,
};

static struct disk_info s_resilient_sd_disk = {
    .name = "SD",
    .ops = &s_resilient_ops,
};

static struct disk_info s_resilient_sdr_disk = {
    .name = "SDR",
    .ops = &s_resilient_ops,
};

static bool s_initialized = false;

/*
 * Layer 1 Disk Registration & Interception:
 * Hooks the hardware "SD" disk, renames it "SD_HW", and registers
 * resilient disks "SD" and "SDR" forwarding to "SD_HW".
 */
extern struct disk_info *disk_access_get_di(const char *name);

int sd_resilient_init(void) {
  if (s_initialized) {
    return 0;
  }

  struct disk_info *hw = disk_access_get_di("SD");
  if (hw == NULL) {
    LOG_WRN("sd_resilient: raw disk 'SD' not found; resilient hook deferred");
    return -ENODEV;
  }

  int ret = disk_access_unregister(hw);
  if (ret != 0) {
    LOG_ERR("sd_resilient: unregister raw SD failed: %d", ret);
    return ret;
  }

  hw->name = "SD_HW";
  ret = disk_access_register(hw);
  if (ret != 0) {
    LOG_ERR("sd_resilient: register SD_HW failed: %d", ret);
    return ret;
  }

  s_resilient_sd_disk.name = "SD";
  s_resilient_sd_disk.ops = &s_resilient_ops;
  s_resilient_sd_disk.dev = hw->dev;
  ret = disk_access_register(&s_resilient_sd_disk);
  if (ret != 0) {
    LOG_ERR("sd_resilient: register resilient SD failed: %d", ret);
    return ret;
  }

  s_resilient_sdr_disk.name = "SDR";
  s_resilient_sdr_disk.ops = &s_resilient_ops;
  s_resilient_sdr_disk.dev = hw->dev;
  (void)disk_access_register(&s_resilient_sdr_disk);

  s_initialized = true;
  LOG_INF("sd_resilient: Layer 1 resilient storage wrapper active (SD & SDR)");
  return 0;
}

#if DT_NODE_EXISTS(DT_ALIAS(sd_power))
static const struct gpio_dt_spec s_sd_power_gpio =
    GPIO_DT_SPEC_GET(DT_ALIAS(sd_power), gpios);
#define HAS_SD_POWER_GPIO 1
#else
#define HAS_SD_POWER_GPIO 0
#endif

/*
 * Hardware Card Reset:
 * Bounded reset sequence:
 * 1. disk_access_ioctl(CTRL_DEINIT) to invoke driver's HAL_SD_DeInit and DMA cleanup.
 * 2. Optional VDD power cycle via sd-power GPIO (100 ms off, 10 ms settle on).
 * 3. Fallback RCC peripheral reset only if driver deinit failed.
 * 4. disk_access_init("SD_HW").
 * 5. Verify disk_access_status == OK and log CLKCR.
 * Prints the result of every step in the sequence.
 */
int sd_resilient_hardware_reset(void) {
  int ret = 0;
  int deinit_rc = -1;

  printk("\n[SD_RESET] Initiating hardware card reset sequence...\n");

  /* Step 1: Prefer driver's own deinit path (HAL_SD_DeInit) */
  for (int i = 0; i < 8; i++) {
    deinit_rc = disk_access_ioctl("SD_HW", DISK_IOCTL_CTRL_DEINIT, NULL);
    if (disk_access_status("SD_HW") != DISK_STATUS_OK) {
      break;
    }
  }
  int post_deinit_status = disk_access_status("SD_HW");
  printk("  [SD_RESET] Step 1 (CTRL_DEINIT): rc=%d, status=%d\n", deinit_rc, post_deinit_status);

  /* Step 2: Optional VDD power cycle via GPIO */
#if HAS_SD_POWER_GPIO
  if (device_is_ready(s_sd_power_gpio.port)) {
    printk("  [SD_RESET] Step 2 (VDD power cycle): OFF for 100 ms...\n");
    gpio_pin_configure_dt(&s_sd_power_gpio, GPIO_OUTPUT_INACTIVE);
    k_msleep(100);
    gpio_pin_set_dt(&s_sd_power_gpio, 1);
    k_msleep(10);
    printk("  [SD_RESET] Step 2 (VDD power cycle): ON, settled 10 ms\n");
  } else {
    printk("  [SD_RESET] Step 2 (VDD power cycle): GPIO not ready\n");
  }
#else
  printk("  [SD_RESET] Step 2 (VDD power cycle): None (alias 'sd-power' not configured)\n");
#endif

  /* Step 2b: Fallback RCC reset only if driver deinit failed */
  if (deinit_rc != 0) {
#if defined(CONFIG_SOC_SERIES_STM32L4X)
    printk("  [SD_RESET] Step 2b (RCC force-reset): deinit failed (%d), resetting peripheral\n", deinit_rc);
    __HAL_RCC_SDMMC1_FORCE_RESET();
    __HAL_RCC_SDMMC1_RELEASE_RESET();
    k_msleep(2);
#else
    k_msleep(2);
#endif
  }

  /* Step 3: Driver init */
  ret = disk_access_init("SD_HW");
  printk("  [SD_RESET] Step 3 (disk_access_init): ret=%d\n", ret);
  if (ret != 0) {
    LOG_WRN("sd_resilient: card re-init failed: %d", ret);
    return ret;
  }

  /* Step 4: Driver status */
  int status = disk_access_status("SD_HW");
  printk("  [SD_RESET] Step 4 (disk_access_status): status=%d\n", status);
  if (status != DISK_STATUS_OK) {
    LOG_WRN("sd_resilient: card status not OK after re-init: %d", status);
    return -EIO;
  }

  /* Step 5: Clock and bus state */
  uint32_t clkcr = 0;
#if defined(CONFIG_SOC_SERIES_STM32L4X)
  clkcr = (uint32_t)SDMMC1->CLKCR;
#endif
  printk("  [SD_RESET] Step 5 (CLKCR after init): 0x%08X\n", (unsigned int)clkcr);

  sd_fault_inject_notify_reinit();
  return 0;
}

int sd_resilient_raw_init(void) {
  return disk_access_init("SD_HW");
}

/*
 * Resilient Disk Operations (Layer 1)
 */
static int resilient_disk_init(struct disk_info *disk) {
  ARG_UNUSED(disk);
  return disk_access_init("SD_HW");
}

static int resilient_disk_status(struct disk_info *disk) {
  ARG_UNUSED(disk);
  return disk_access_status("SD_HW");
}

static int resilient_disk_erase(struct disk_info *disk, uint32_t start_sector,
                                uint32_t num_sector) {
  ARG_UNUSED(disk);
  return disk_access_erase("SD_HW", start_sector, num_sector, DISK_ACCESS_ERASE_PHYSICAL);
}

static int resilient_disk_ioctl(struct disk_info *disk, uint8_t cmd, void *buff) {
  ARG_UNUSED(disk);
  return disk_access_ioctl("SD_HW", cmd, buff);
}

static int resilient_disk_read(struct disk_info *disk, uint8_t *data_buf,
                               uint32_t start_sector, uint32_t num_sector) {
  ARG_UNUSED(disk);
  int rc = -EIO;

  for (int attempt = 0; attempt < 2; attempt++) {
    rc = disk_access_read("SD_HW", data_buf, start_sector, num_sector);
    if (rc == 0) {
      return 0;
    }
    LOG_WRN("sd_resilient: read attempt %d failed: %d", attempt + 1, rc);
    (void)sd_resilient_hardware_reset();
  }

  return rc;
}

static int resilient_disk_write(struct disk_info *disk, const uint8_t *data_buf,
                                uint32_t start_sector, uint32_t num_sector) {
  ARG_UNUSED(disk);
  uint32_t attempt = 0;
  uint64_t t_start = k_cycle_get_64();
  int rc = 0;

  while (attempt < 3) {
    uint64_t t_now = k_cycle_get_64();
    uint64_t diff = (t_now >= t_start) ? (t_now - t_start) : 0;
    uint32_t elapsed_us = (uint32_t)k_cyc_to_us_floor64(diff);
    if (elapsed_us >= 1500000U && attempt > 0) {
      LOG_ERR("sd_resilient: Layer 1 write retry budget exhausted (%u us)", elapsed_us);
      break;
    }

    int fault = sd_fault_inject_check_write(start_sector, num_sector);
    if (fault != 0) {
      rc = fault;
    } else {
      rc = disk_access_write("SD_HW", data_buf, start_sector, num_sector);
    }

    if (rc == 0) {
      if (attempt > 0) {
        k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
        s_stats.retry_ok++;
        k_spin_unlock(&s_stats_lock, key);
        sd_resilient_notify_recovery_needed("Layer 1 retry success");
      }
      return 0;
    }

    /* Error detected */
    uint64_t t_fail = k_cycle_get_64();
    uint32_t fail_us = (uint32_t)k_cyc_to_us_floor64((t_fail >= t_start) ? (t_fail - t_start) : 0);

    k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
    s_stats.err_total++;
#if defined(CONFIG_SOC_SERIES_STM32L4X)
    if ((SDMMC1->STA & SDMMC_STA_DCRCFAIL) || rc == -EIO) {
      s_stats.err_crc++;
    } else if ((SDMMC1->STA & SDMMC_STA_DTIMEOUT) || rc == -ETIMEDOUT) {
      s_stats.err_timeout++;
    } else {
      s_stats.err_other++;
    }
#else
    s_stats.err_crc++;
#endif
    k_spin_unlock(&s_stats_lock, key);

    sd_resilient_log_failure_atomic("LAYER1_WRITE_RETRY", rc, start_sector,
                                    num_sector, attempt + 1, fail_us);

    /* Backoff strategy:
     * Attempt 0 failed: 1 ms sleep before Attempt 1 retry.
     * Attempt 1 failed: hardware peripheral reset and 5 ms sleep before Attempt 2 retry.
     * Max 8 re-inits per session.
     */
    if (attempt == 0) {
      k_msleep(1);
    } else if (attempt == 1) {
      int rst_rc = sd_resilient_hardware_reset();
      if (rst_rc != 0) {
        LOG_WRN("sd_resilient: hardware reset failed during retry attempt %u (%d)",
                attempt + 1, rst_rc);
      }
      k_msleep(5);
    }

    attempt++;
  }

  /* Layer 1 exhausted all 3 attempts */
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.retry_fail++;
  k_spin_unlock(&s_stats_lock, key);

  uint64_t t_end = k_cycle_get_64();
  uint32_t rec_us = (uint32_t)k_cyc_to_us_floor64((t_end >= t_start) ? (t_end - t_start) : 0);
  sd_resilient_add_recovery_time(rec_us);

  return -EIO;
}

/*
 * Serialized Atomic Failure Diagnostics Logger:
 * Acquires mutex so log messages from concurrent contexts never interleave.
 */
void sd_resilient_log_failure_atomic(const char *tag, int err, uint32_t sector,
                                     uint32_t count, uint32_t attempt,
                                     uint32_t latency_us) {
  k_mutex_lock(&s_resilient_log_mutex, K_FOREVER);

  uint32_t uptime_ms = k_uptime_get_32();
  uint32_t sta = 0;
  uint32_t dcount = 0;
  uint32_t dctrl = 0;
  uint32_t clkcr = 0;

#if defined(CONFIG_SOC_SERIES_STM32L4X)
  sta = (uint32_t)SDMMC1->STA;
  dcount = (uint32_t)SDMMC1->DCOUNT;
  dctrl = (uint32_t)SDMMC1->DCTRL;
  clkcr = (uint32_t)SDMMC1->CLKCR;
#endif

  printk("\n-------------------------------------------------------------------------\n");
  printk("[STORAGE DIAGNOSTIC] %s (err=%d, attempt=%u)\n", tag, err, attempt);
  printk("  Target Sector : %u (count: %u sectors = %u bytes)\n",
         sector, count, count * 512U);
  printk("  Timing        : Latency=%u us, Uptime=%u ms\n", latency_us, uptime_ms);
  printk("  Hardware Regs : STA=0x%08X, DCOUNT=%u, DCTRL=0x%08X, CLKCR=0x%08X\n",
         (unsigned int)sta, (unsigned int)dcount, (unsigned int)dctrl, (unsigned int)clkcr);
  if (sta & 0x00000002U) {
    printk("  Flag Detail   : [DATA_CRC_FAIL] CRC check failed on data block\n");
  }
  if (sta & 0x00000008U) {
    printk("  Flag Detail   : [DATA_TIMEOUT] Data timeout reached\n");
  }
  if (sta & 0x00000010U) {
    printk("  Flag Detail   : [TX_UNDERRUN] Transmit FIFO underrun\n");
  }
  if (sta & 0x00000020U) {
    printk("  Flag Detail   : [RX_OVERRUN] Receive FIFO overrun\n");
  }
  printk("-------------------------------------------------------------------------\n");

  k_mutex_unlock(&s_resilient_log_mutex);
}

/*
 * Health State Machine & Statistics
 */
enum storage_health_state sd_resilient_get_health(void) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  enum storage_health_state h = s_health;
  k_spin_unlock(&s_stats_lock, key);
  return h;
}

void sd_resilient_set_health(enum storage_health_state state) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  if (s_health != state) {
    LOG_INF("Storage health transition: %s -> %s",
            sd_resilient_health_str(s_health), sd_resilient_health_str(state));
    s_health = state;
  }
  k_spin_unlock(&s_stats_lock, key);
}

const char *sd_resilient_health_str(enum storage_health_state state) {
  switch (state) {
    case STORAGE_HEALTH_HEALTHY:
      return "HEALTHY";
    case STORAGE_HEALTH_DEGRADED:
      return "DEGRADED";
    case STORAGE_HEALTH_FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

void sd_resilient_notify_clean_block(void) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.consecutive_clean_blocks++;
  s_stats.consecutive_discards = 0;
  if (s_health == STORAGE_HEALTH_DEGRADED && s_stats.consecutive_clean_blocks >= 100) {
    s_health = STORAGE_HEALTH_HEALTHY;
    LOG_INF("Storage health recovered: DEGRADED -> HEALTHY (100 clean blocks)");
  }
  k_spin_unlock(&s_stats_lock, key);
}

void sd_resilient_notify_recovery_needed(const char *reason) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.consecutive_clean_blocks = 0;
  if (s_health == STORAGE_HEALTH_HEALTHY) {
    s_health = STORAGE_HEALTH_DEGRADED;
    LOG_WRN("Storage health transition: HEALTHY -> DEGRADED (reason: %s)", reason);
  }
  k_spin_unlock(&s_stats_lock, key);
}

void sd_resilient_get_stats(struct sd_resilient_stats *out_stats) {
  if (out_stats == NULL) {
    return;
  }
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  *out_stats = s_stats;
  k_spin_unlock(&s_stats_lock, key);
}

void sd_resilient_reset_session_stats(void) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.retry_ok = 0;
  s_stats.retry_fail = 0;
  s_stats.reopen_ok = 0;
  s_stats.reopen_fail = 0;
  s_stats.err_total = 0;
  s_stats.err_crc = 0;
  s_stats.err_timeout = 0;
  s_stats.err_other = 0;
  s_stats.blocks_discarded = 0;
  s_stats.samples_discarded = 0;
  s_stats.time_in_recovery_us = 0;
  s_stats.discarded_at_sleep_bytes = 0;
  s_stats.consecutive_discards = 0;
  s_stats.consecutive_clean_blocks = 0;
  s_health = STORAGE_HEALTH_HEALTHY;
  k_spin_unlock(&s_stats_lock, key);

  sd_resilient_clear_gaps();
}

void sd_resilient_inc_reopen_ok(void) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.reopen_ok++;
  k_spin_unlock(&s_stats_lock, key);
  sd_resilient_notify_recovery_needed("Layer 2 reopen success");
}

void sd_resilient_inc_reopen_fail(void) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.reopen_fail++;
  k_spin_unlock(&s_stats_lock, key);
}

void sd_resilient_record_discard(uint32_t block_bytes, uint32_t samples, int err) {
  ARG_UNUSED(block_bytes);
  ARG_UNUSED(err);
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.blocks_discarded++;
  s_stats.samples_discarded += samples;
  s_stats.consecutive_discards++;
  s_stats.consecutive_clean_blocks = 0;
  if (s_stats.consecutive_discards >= 16) {
    s_health = STORAGE_HEALTH_FAULT;
    LOG_ERR("Storage health transition: -> FAULT (consecutive discards >= 16)");
  }
  k_spin_unlock(&s_stats_lock, key);
}

void sd_resilient_add_recovery_time(uint32_t us) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.time_in_recovery_us += us;
  if (s_stats.time_in_recovery_us > 20000000ULL) { /* > 20 s */
    s_health = STORAGE_HEALTH_FAULT;
    LOG_ERR("Storage health transition: -> FAULT (cumulative recovery time > 20 s)");
  }
  k_spin_unlock(&s_stats_lock, key);
}

void sd_resilient_record_sleep_discard(uint64_t bytes) {
  k_spinlock_key_t key = k_spin_lock(&s_stats_lock);
  s_stats.discarded_at_sleep_bytes += bytes;
  k_spin_unlock(&s_stats_lock, key);
}

/*
 * Gap Ring Buffer Management
 */
void sd_resilient_add_gap(uint64_t sample_index, uint32_t sample_count, int32_t err_code) {
  k_spinlock_key_t key = k_spin_lock(&s_gap_lock);
  uint32_t idx = (s_gap_head) % STORAGE_GAP_RING_CAPACITY;
  s_gap_ring[idx].sample_index = sample_index;
  s_gap_ring[idx].sample_count = sample_count;
  s_gap_ring[idx].err_code = (int16_t)err_code;
  s_gap_ring[idx].uptime_s = (uint16_t)(k_uptime_get_32() / 1000U);
  s_gap_head = (s_gap_head + 1) % STORAGE_GAP_RING_CAPACITY;
  if (s_gap_count < STORAGE_GAP_RING_CAPACITY) {
    s_gap_count++;
  }
  k_spin_unlock(&s_gap_lock, key);

  LOG_WRN("[STORAGE GAP] Gap recorded: index=%llu count=%u err=%d",
          (unsigned long long)sample_index, sample_count, err_code);
  printk("[STORAGE GAP] index=%llu count=%u err=%d\n",
         (unsigned long long)sample_index, sample_count, err_code);
}

uint32_t sd_resilient_get_gap_count(void) {
  k_spinlock_key_t key = k_spin_lock(&s_gap_lock);
  uint32_t c = s_gap_count;
  k_spin_unlock(&s_gap_lock, key);
  return c;
}

uint32_t sd_resilient_copy_gaps(struct storage_gap_record *out_buf, uint32_t max_records) {
  if (out_buf == NULL || max_records == 0) {
    return 0;
  }
  k_spinlock_key_t key = k_spin_lock(&s_gap_lock);
  uint32_t to_copy = (s_gap_count < max_records) ? s_gap_count : max_records;
  uint32_t start_idx = (s_gap_head >= to_copy)
                           ? (s_gap_head - to_copy)
                           : (STORAGE_GAP_RING_CAPACITY + s_gap_head - to_copy);
  for (uint32_t i = 0; i < to_copy; i++) {
    uint32_t idx = (start_idx + i) % STORAGE_GAP_RING_CAPACITY;
    out_buf[i] = s_gap_ring[idx];
  }
  k_spin_unlock(&s_gap_lock, key);
  return to_copy;
}

bool sd_resilient_get_gap_at(uint32_t index, struct storage_gap_record *out_rec) {
  if (out_rec == NULL) {
    return false;
  }
  k_spinlock_key_t key = k_spin_lock(&s_gap_lock);
  if (index >= s_gap_count) {
    k_spin_unlock(&s_gap_lock, key);
    return false;
  }
  uint32_t start_idx = (s_gap_head >= s_gap_count)
                           ? (s_gap_head - s_gap_count)
                           : (STORAGE_GAP_RING_CAPACITY + s_gap_head - s_gap_count);
  uint32_t idx = (start_idx + index) % STORAGE_GAP_RING_CAPACITY;
  *out_rec = s_gap_ring[idx];
  k_spin_unlock(&s_gap_lock, key);
  return true;
}

void sd_resilient_clear_gaps(void) {
  k_spinlock_key_t key = k_spin_lock(&s_gap_lock);
  s_gap_count = 0;
  s_gap_head = 0;
  k_spin_unlock(&s_gap_lock, key);
}
