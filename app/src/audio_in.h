/* SPDX-License-Identifier: Apache-2.0 */
#ifndef AUDIO_IN_H_
#define AUDIO_IN_H_

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#define AUDIO_DMA_EVENT_QUEUE_DEPTH 4U

/*
 * Audio-input backend interface (same role as w_hal_i2s_* in the MAX78000
 * firmware). One backend per chip family / input type:
 *   audio_in_synth.c      - software sine, any board
 *   audio_in_stm32l4.c    - L4 internal ADC (TIM trigger + oversampling + DMA)
 *   (later) audio_in_stm32u5.c, audio_in_i2s.c (external ADC via Zephyr I2S API)
 *
 * Output format is signed PCM (16-bit or 32-bit depending on CONFIG_AUDIO_SAMPLE_BITS),
 * CONFIG_AUDIO_BLOCK_SAMPLES per block.
 */

struct audio_sink {
	/* Return an empty block, or NULL if the pool is exhausted (caller counts a drop).
	 * Called from the acquisition thread: must not wait indefinitely. */
	void *(*alloc)(void *user);
	/* Return an allocated block that was not submitted. */
	void (*release)(void *block, void *user);
	/* Hand a filled block to the writer. */
	int (*submit)(void *block, void *user);
	void *user;
};

/* Diagnostics reported by acquisition backend */
struct audio_diag {
	uint32_t total_samples;
	uint32_t total_blocks;
	uint16_t raw_min;
	uint16_t raw_max;
	uint32_t raw_midpoint;        /* Estimated DC midpoint in raw ADC units */
	uint32_t raw_rms_code;        /* RMS including DC in unshifted ADC codes */
	uint32_t raw_ac_rms_code;     /* AC RMS after subtracting mean, in ADC codes */
	uint32_t clipping_low_count;  /* Samples hitting ground / zero code */
	uint32_t clipping_high_count; /* Samples hitting full-scale code */
	uint32_t peak_to_peak;        /* raw_max - raw_min */
	uint32_t actual_rate;         /* Measured hardware sample rate */
	uint32_t raw_rms_mv;          /* AC RMS amplitude in millivolts */
	int32_t raw_rms_dbfs_x10;     /* AC RMS relative to full-scale sine RMS, dBFS * 10 */
};

struct audio_dma_stats {
	uint32_t callback_errors;
	uint32_t adc_overruns;
	uint32_t callbacks;
	uint64_t callback_interval_min_cycles;
	uint64_t callback_interval_avg_cycles;
	uint64_t callback_interval_max_cycles;
	uint64_t callback_exec_min_cycles;
	uint64_t callback_exec_avg_cycles;
	uint64_t callback_exec_max_cycles;
	uint32_t events_missed;
	uint64_t samples_acquired;
	uint32_t samples_accounted;
	uint32_t samples_processed;
	uint32_t samples_lost;
	uint32_t acquisition_max_latency_cycles;
	uint32_t acquisition_process_avg_cycles;
	uint32_t acquisition_process_max_cycles;
	uint32_t event_queue_high_water;
	uint32_t dma_queue_full_drops;
	uint32_t slab_alloc_drops;
	uint32_t half_unstable_drops;
	uint32_t half_overwritten_drops;
	uint32_t queue_submit_drops;
	uint32_t worker_deadline_drops;
	uint32_t boundary_discarded_samples;
};

/* Configure hardware. Returns 0 or a negative errno. */
int audio_in_init(uint32_t requested_rate_hz, const struct audio_sink *sink);

/* Rate the hardware really runs at (integer timer divider), valid after init. */
uint32_t audio_in_rate(void);

/* Runtime sample rate reconfiguration (supported: 16k, 32k, 44.1k, 48k, 96k, 192k) */
int audio_in_set_rate(uint32_t new_rate_hz);
bool audio_in_is_rate_supported(uint32_t rate_hz, uint32_t *act_rate, uint32_t *divider, int *error_hz);

/* Requested rate before hardware timer truncation / rounding */
uint32_t audio_in_requested_rate(void);

/**
 * @brief Capture source mode selection.
 */
enum audio_capture_source {
	AUDIO_SRC_PHYSICAL_MIC = 0,     /* Physical analog microphone input (MAX4466) */
	AUDIO_SRC_STEREO_DUPLICATE = 1, /* Real mono ADC capture duplicated to stereo channels */
	AUDIO_SRC_SYNTHETIC = 2         /* TEST synthetic deterministic sample source */
};

void audio_in_set_capture_source(enum audio_capture_source src);
enum audio_capture_source audio_in_get_capture_source(void);
uint32_t audio_in_get_ovs_ratio(void);

/* Select block size while capture is stopped. */
int audio_in_set_block_size(uint32_t samples);

int audio_in_start(void);
int audio_in_stop(void);
int audio_in_verify_ready(void);

/* Check whether audio DMA is actively running */
bool audio_in_is_dma_active(void);

/* Check whether acquisition worker is processing or has pending events */
bool audio_in_is_worker_active(void);

/* Retrieve diagnostics snapshot */
void audio_in_get_diag(struct audio_diag *diag);

/* Reset diagnostics accumulators */
void audio_in_reset_diag(void);

/* Get callback counts for DMA HT and TC interrupts */
void audio_in_get_callback_counts(uint32_t *ht_count, uint32_t *tc_count);
void audio_in_get_dma_stats(struct audio_dma_stats *stats);
uint32_t audio_in_get_missed_event_count(void);
uint64_t audio_in_get_source_samples(void);

#endif
