/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RECORDER_H_
#define RECORDER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*
 * ============================================================================
 * WILDSONIC THREAD PRIORITY & EXECUTION TOPOLOGY TABLE
 * ============================================================================
 *
 * System Priorities (Preemptive, Lower Number = Higher Priority):
 * | Priority         | Thread Name    | Stack Size | Execution Role                         |
 * |------------------|----------------|------------|----------------------------------------|
 * | ISR (Hardware)   | ADC1/TIM6 DMA  | N/A        | Fills DMA half-buffers, signals event  |
 * | K_PRIO_PREEMPT(0)| audio_acquire  | 2048 B     | Drains DMA events, copies to pool blk  |
 * | K_PRIO_PREEMPT(1)| main / console | 4096 B     | Interactive UART console REPL          |
 * | K_PRIO_PREEMPT(1)| recorder       | 6144 B     | DSP filter pipeline & PSRAM staging    |
 * | K_PRIO_PREEMPT(3)| sd_writer      | 4096 B     | Dedicated FatFs SDMMC writer task      |
 * | K_PRIO_PREEMPT(3)| psram_reader   | 2048 B     | Asynchronous PSRAM ping-pong reader    |
 * | K_PRIO_PREEMPT(4)| scheduler      | 4096 B     | Low-power RTC sleep/wake coordination  |
 *
 * Invariants:
 * 1. Real-time acquisition (Priority 0) preempts DSP processing (Priority 1).
 * 2. DSP processing (Priority 1) preempts SD writing and PSRAM reading (Priority 3).
 * 3. SD writing and PSRAM reading (Priority 3) preempt low-power scheduling (Priority 4).
 * ============================================================================
 */

/* WildSonic Buffer Ownership State Machine */
enum buf_owner_state {
  BUF_OWNER_FREE = 0,    /* In pool, available for allocation */
  BUF_OWNER_DMA,         /* Allocated by sink_alloc, being filled by DMA worker */
  BUF_OWNER_DSP,         /* Dequeued from filled_q, undergoing DSP & conversion */
  BUF_OWNER_STAGED,      /* Data pushed to PSRAM staging / SD queue */
  BUF_OWNER_WRITING,     /* Currently being written to SD card (Mode A) */
};

struct rec_stats {
  uint32_t sample_rate;
  uint16_t sample_bits;            /* 16 or 32 */
  uint16_t channels;               /* 1 */
  uint64_t expected_samples;       /* requested samples */
  uint64_t samples_acquired;       /* ADC raw acquisitions */
  uint64_t samples_accepted;       /* valid audio frames accepted */
  uint64_t samples_written;        /* valid audio frames written to disk */
  uint64_t samples_captured;       /* alias for samples_written */
  uint32_t expected_blocks;        /* total blocks required (ceil) */
  uint32_t blocks_processed;       /* total blocks dequeued */
  uint32_t full_blocks_written;    /* complete 2048-sample blocks written */
  uint32_t partial_blocks_written; /* 1 if last block is partial, 0 if exact */
  uint32_t partial_block_samples; /* number of valid samples in partial block */
  uint32_t blocks_written;        /* full + partial blocks written */
  uint32_t blocks_dropped; /* audio lost because the RAM pool/queue was full */
  uint64_t samples_dropped; /* total audio samples lost across all stages */
  uint32_t dma_dropped_blocks; /* blocks dropped in ADC/DMA ISR due to pool starvation */
  uint32_t sram_queue_dropped_blocks; /* blocks dropped due to SRAM writer queue full */
  uint32_t queue_drained_blocks; /* unconsumed queue blocks discarded upon stop/abort */
  uint32_t queue_drained_samples; /* unconsumed queue samples discarded upon stop/abort */
  uint32_t queue_high_water; /* max audio blocks waiting for processing */
  uint32_t pool_high_water; /* max audio blocks held in the SRAM pool */
  uint32_t sd_queue_high_water; /* max blocks waiting for the SD writer */
  int sd_write_error;
  uint64_t sd_failed_samples;
  uint32_t adc_overruns;
  uint32_t adc_dma_recovery_attempts;
  uint32_t adc_dma_recovery_failures;
  uint32_t psram_busy_retries;
  uint32_t psram_full_wait_ms;
  uint32_t sd_recovery_attempts;
  int32_t sd_recovery_result;
  uint32_t writer_completion_timeouts;
  uint32_t dma_errors;
  uint32_t acquisition_events_missed;
  uint32_t acquisition_samples_processed;
  uint32_t acquisition_samples_lost;
  uint32_t acquisition_samples_accounted;
  uint32_t dma_interval_min_us;
  uint32_t dma_interval_avg_us;
  uint32_t dma_interval_max_us;
  uint32_t worst_write_us;   /* slowest single SD write */
  uint32_t avg_write_us;
  uint32_t write_count;      /* total number of SD write operations */
  uint64_t total_write_us;   /* total monotonic time spent in fs_write calls */
  uint64_t session_wall_us;  /* total monotonic session wall time */
  uint32_t sd_busy_permille; /* time spent inside SD writes / wall time * 1000,
                                max 1000 */
  uint64_t bytes_written;    /* exact PCM data bytes written */
  uint32_t elapsed_ms;       /* independent elapsed wall time */
  uint32_t measured_sample_rate;
  bool wav_verified; /* read-back header and file size validation */
  bool clock_untrusted; /* true if RTC source is LSI (+-5-10% drift) */
  int64_t sched_start_utc;
  int64_t first_sample_utc;
  int64_t start_delta_ms;
  uint64_t file_size_bytes;
  int err;           /* first negative errno seen, 0 if none */
  char file[80];     /* expanded to hold /SD:/YYYYMMDD/YYYYMMDD_HH/YYYYMMDD_HHMMSS_NNN.wav */

  /* PSRAM Staging Buffer & SD Batch Write Metrics */
  bool psram_used; /* true if PSRAM circular staging was active */
  uint32_t psram_staging_capacity; /* capacity in bytes (e.g. 1 MiB) */
  uint32_t psram_high_water_bytes; /* maximum bytes queued in PSRAM */
  uint32_t psram_full_events;      /* count of staging overflow events */
  uint32_t psram_dropped_samples;  /* samples lost due to PSRAM overflow */
  uint32_t sd_batch_bytes;      /* target SD write batch size (e.g. 256 KiB) */
  uint32_t bytes_per_write_avg; /* average payload bytes per fs_write call */

  /* RTC Calendar Timestamps */
  char start_time_str[24]; /* "YYYY-MM-DD HH:MM:SS" */
  char end_time_str[24];   /* "YYYY-MM-DD HH:MM:SS" */
  bool rtc_time_valid;     /* true if timestamps originated from valid RTC */

  /* DSP Execution Metrics */
  uint32_t dsp_avg_us;
  uint32_t dsp_worst_us;
  uint64_t dsp_total_us;
  uint32_t dsp_overruns;
  const char *dsp_mode_str;
  float digital_gain;
  uint32_t digital_clipping_count;
  uint32_t adc_clip_low_count;
  uint32_t adc_clip_high_count;
  uint32_t dsp_input_clip_count;
  uint32_t dsp_filter_overshoot_count;
  uint32_t dsp_gain_clipping_count;
  uint64_t samples_discarded_tail;
  char first_error_stage[32];
  uint64_t hardware_source_samples;
  uint64_t copied_samples;
  uint64_t processed_samples;
  uint64_t boundary_discarded_samples;

  /* Enhanced SD Writer & Batch Pipeline Metrics */
  uint64_t sd_writer_wall_us;
  uint64_t sd_writer_psram_read_us;
  uint64_t sd_writer_active_fs_us;
  uint64_t sd_writer_commit_us;
  uint64_t sd_writer_starve_us;
  uint32_t sd_active_throughput_kb_s;
  uint32_t sd_aggregate_throughput_kb_s;
  uint64_t pingpong_overlap_us;
  uint32_t pingpong_swaps;
  uint32_t psram_qspi_txn_count;
  uint32_t psram_read_throughput_kb_s;
  uint32_t psram_write_throughput_kb_s;
  uint8_t pipeline_mode;

  /* Data Loss Policy & Back-pressure Metrics */
  uint32_t gap_count;
  uint32_t consecutive_drop_abort_count;
  bool slowdown_active;
  uint32_t slowdown_events;
};

enum storage_pipeline_mode {
  STORAGE_PIPELINE_DIRECT_SRAM = 0, /* Mode A: Direct SRAM queue to SD (no PSRAM) */
  STORAGE_PIPELINE_PSRAM_SYNC = 1,   /* Mode B: PSRAM Staging with Synchronous SD batch drain */
  STORAGE_PIPELINE_PSRAM_ASYNC = 2,  /* Mode C: PSRAM Staging with Asynchronous Ping-Pong */
};

void recorder_set_pipeline_mode(enum storage_pipeline_mode mode);
enum storage_pipeline_mode recorder_get_pipeline_mode(void);
const char *recorder_get_pipeline_mode_str(enum storage_pipeline_mode mode);

enum recorder_power_state {
  REC_PWR_ACTIVE = 0,
  REC_PWR_RECORDING,
  REC_PWR_FINALIZING,
  REC_PWR_SLEEP_PREP,
  REC_PWR_RTC_ARMED,
  REC_PWR_STOP2,
  REC_PWR_WAKE_RECOVERY,
};

enum recorder_power_state recorder_get_power_state(void);

enum recorder_mode {
  REC_MODE_NORMAL = 0,
  REC_MODE_RAW,
  REC_MODE_FILTERED,
};

/* Split initialization for ordered boot sequence:
 * 1. recorder_init_storage(): threads, queues, SD card mount
 * 2. recorder_init_psram():   PSRAM staging ring setup
 * 3. recorder_init_audio():   single-shot ADC/DMA init at final sample rate
 */
int recorder_init_storage(void);
void recorder_init_psram(void);
int recorder_init_audio(uint32_t sample_rate_hz);
void recorder_log_sd_boot_status(void);
void recorder_log_last_file(void);

/* Remount the SD filesystem after the SDMMC driver has reported an
 * unrecoverable controller/card error. Only valid while recording is idle. */
int recorder_storage_recover(void);

/* Record for `seconds` into a new /SD:/RECnnnn.WAV file. */
int recorder_record(uint32_t seconds, struct rec_stats *out);
int recorder_record_cycle(uint32_t seconds, uint32_t cycle,
                          struct rec_stats *out);
int recorder_record_session(uint32_t seconds, uint32_t cycle,
                            enum recorder_mode mode, struct rec_stats *out);

/* Classification of session termination reason */
enum recorder_stop_reason {
  RECORDER_STOP_NONE = 0,
  RECORDER_STOP_USER,
  RECORDER_STOP_APPLY_REBOOT,
  RECORDER_STOP_SYSTEM_REBOOT,
  RECORDER_STOP_ERROR
};

void recorder_set_stop_reason(enum recorder_stop_reason reason);
enum recorder_stop_reason recorder_get_stop_reason(void);
void recorder_request_stop_reason(enum recorder_stop_reason reason);

/* Request immediate safe stop of ongoing recording (finalizes WAV header
 * cleanly) */
void recorder_request_stop(void);
void recorder_clear_abort_request(void);
bool recorder_is_active(void);
bool recorder_is_shutdown_ready(void);
int recorder_sd_verify_idle(void);
bool recorder_is_abort_requested(void);
int recorder_wait_idle(uint32_t timeout_ms);
int recorder_get_last_error(void);
int recorder_get_last_sd_error(void);
int recorder_get_storage_recovery_result(void);
void recorder_get_active_progress(uint64_t *samples_written, uint64_t *expected_samples);
void recorder_get_active_file(char *out_buf, size_t max_len);

/* Runtime configuration for PSRAM audio staging buffer (ON/OFF) */
void recorder_set_psram_staging_enabled(bool enabled);
bool recorder_is_psram_staging_enabled(void);

/* Scheduled session start time tracking */
void recorder_set_scheduled_start_utc(int64_t start_time_utc);
int64_t recorder_get_scheduled_start_utc(void);

/* Validate WAV file on disk after recording */
int recorder_verify_file(const char *path, uint32_t expected_rate,
                         uint16_t expected_bits, uint64_t expected_data_bytes);
int recorder_verify_file_format(const char *path, uint32_t expected_rate,
                                uint16_t expected_channels, uint16_t expected_bits,
                                uint64_t expected_data_bytes);

/* Scan SD card for the most recently recorded WAV file based on folder structure */
int recorder_scan_last_file(char *out_folder, size_t folder_len, char *out_file, size_t file_len);

/* Low-power sleep transition between sessions, with measured wakeup recovery */
int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us);

/* Per-batch SD timing log mode (TEST/DEV builds print according to mode;
 * measurements are always collected in all builds). */
enum recorder_batchlog_mode {
  BATCHLOG_LIVE = 0,
  BATCHLOG_END,
  BATCHLOG_OFF,
};

void recorder_set_batchlog_mode(enum recorder_batchlog_mode mode);
enum recorder_batchlog_mode recorder_get_batchlog_mode(void);
void recorder_print_last_stats(void);
void recorder_print_stack_free_stats(void);

typedef enum {
  SHUTDOWN_STAGE_NONE = 0,
  SHUTDOWN_STAGE_AUDIO_STOP,
  SHUTDOWN_STAGE_WRITER_DRAIN,
  SHUTDOWN_STAGE_FS_SYNC,
  SHUTDOWN_STAGE_PSRAM_SLEEP,
  SHUTDOWN_STAGE_SD_SLEEP,
  SHUTDOWN_STAGE_STOP2,
  SHUTDOWN_STAGE_COMPLETE
} shutdown_stage_t;

const char *recorder_shutdown_stage_name(shutdown_stage_t stage);
shutdown_stage_t recorder_get_shutdown_stage(void);

#endif
