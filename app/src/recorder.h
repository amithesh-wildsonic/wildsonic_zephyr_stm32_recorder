/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RECORDER_H_
#define RECORDER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

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
};

enum recorder_mode {
  REC_MODE_NORMAL = 0,
  REC_MODE_RAW,
  REC_MODE_FILTERED,
};

/* Initialise audio input + mount the SD card. */
int recorder_init(void);

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
bool recorder_is_active(void);
bool recorder_is_shutdown_ready(void);
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

#endif
