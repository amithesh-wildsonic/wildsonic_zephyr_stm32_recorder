/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SD_RESILIENT_H_
#define SD_RESILIENT_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*
 * Storage Health State Machine:
 * HEALTHY: All operations nominal
 * DEGRADED: A block required Layer 1 retry or Layer 2 reopen recovery
 * FAULT: Card unrecoverable in current session
 */
enum storage_health_state {
  STORAGE_HEALTH_HEALTHY = 0,
  STORAGE_HEALTH_DEGRADED = 1,
  STORAGE_HEALTH_FAULT = 2
};

struct sd_resilient_stats {
  uint32_t retry_ok;
  uint32_t retry_fail;
  uint32_t reopen_ok;
  uint32_t reopen_fail;
  uint32_t err_total;
  uint32_t err_crc;
  uint32_t err_timeout;
  uint32_t err_other;
  uint32_t blocks_discarded;
  uint64_t samples_discarded;
  uint64_t time_in_recovery_us;
  uint64_t discarded_at_sleep_bytes;
  uint32_t consecutive_discards;
  uint32_t consecutive_clean_blocks;
};

/* Gap record structure for RAM ring (>=128 entries) */
struct storage_gap_record {
  uint64_t sample_index;
  uint32_t sample_count;
  int16_t err_code;
  uint16_t uptime_s;
} __attribute__((packed));

#define STORAGE_GAP_RING_CAPACITY 128

/* Layer 1 resilient disk initialization and hardware management */
int sd_resilient_init(void);
int sd_resilient_hardware_reset(void);
int sd_resilient_raw_init(void);

/* Health state management */
enum storage_health_state sd_resilient_get_health(void);
void sd_resilient_set_health(enum storage_health_state state);
const char *sd_resilient_health_str(enum storage_health_state state);
void sd_resilient_notify_clean_block(void);
void sd_resilient_notify_recovery_needed(const char *reason);

/* Statistics */
void sd_resilient_get_stats(struct sd_resilient_stats *out_stats);
void sd_resilient_reset_session_stats(void);
void sd_resilient_inc_reopen_ok(void);
void sd_resilient_inc_reopen_fail(void);
void sd_resilient_record_discard(uint32_t block_bytes, uint32_t samples, int err);
void sd_resilient_add_recovery_time(uint32_t us);
void sd_resilient_record_sleep_discard(uint64_t bytes);

/* Serialized failure diagnostics log (holding mutex, single multi-line message) */
void sd_resilient_log_failure_atomic(const char *tag, int err, uint32_t sector,
                                     uint32_t count, uint32_t attempt,
                                     uint32_t latency_us);

/* Gap ring buffer API */
void sd_resilient_add_gap(uint64_t sample_index, uint32_t sample_count, int32_t err_code);
uint32_t sd_resilient_get_gap_count(void);
uint32_t sd_resilient_copy_gaps(struct storage_gap_record *out_buf, uint32_t max_records);
bool sd_resilient_get_gap_at(uint32_t index, struct storage_gap_record *out_rec);
void sd_resilient_clear_gaps(void);

#endif /* SD_RESILIENT_H_ */

