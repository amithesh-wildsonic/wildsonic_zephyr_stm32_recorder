/* SPDX-License-Identifier: Apache-2.0 */
#ifndef DSP_H_
#define DSP_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Direct Form II Transposed Biquad Filter:
 *   y[n] = b0 * x[n] + s1
 *   s1   = b1 * x[n] - a1 * y[n] + s2
 *   s2   = b2 * x[n] - a2 * y[n]
 *
 * Numerically stable, canonical in delays, 2 state variables per biquad.
 */
struct biquad_df2t {
	float b0, b1, b2;
	float a1, a2;
	float s1, s2;
	bool enabled;
};

/*
 * DSP Processing Chain Configuration and State
 */
struct dsp_config {
	bool dc_block_enabled;
	bool hpf_enabled;
	uint32_t hpf_cutoff_hz;
	bool notch_enabled;
	uint32_t notch_freq_hz;
	float notch_q;
	bool harmonic_notch_enabled;
	uint32_t harmonic_freq_hz;
	float harmonic_q;
	bool lpf_enabled;
	uint32_t lpf_cutoff_hz;
};

struct dsp_stats {
	uint64_t total_cycles;
	uint32_t blocks_processed;
	uint32_t samples_processed;
	uint32_t worst_cycles;
	uint32_t avg_cycles;
	uint32_t worst_us;
	uint32_t avg_us;
	uint32_t overruns;        /* Blocks that exceeded the real-time deadline */
	uint32_t block_deadline_us;
};

/* Initialize DSP pipeline for given sample rate (Hz) and default config */
int dsp_init(uint32_t sample_rate_hz);

/* Configure DSP pipeline parameters */
int dsp_configure(const struct dsp_config *cfg, uint32_t sample_rate_hz);

/* Reset all filter internal delay states (call before starting a new recording) */
void dsp_reset(void);

/* Process a block of 16-bit signed PCM audio in-place */
void dsp_process_i16(int16_t *samples, uint32_t count);

/* Process a block of 32-bit signed PCM audio in-place */
void dsp_process_i32(int32_t *samples, uint32_t count);

/* Generate a deterministic synthetic test signal for verification */
void dsp_generate_test_signal_i16(int16_t *dst, uint32_t count, uint32_t sample_rate_hz);
void dsp_generate_test_signal_i32(int32_t *dst, uint32_t count, uint32_t sample_rate_hz);

/* Retrieve performance statistics */
void dsp_get_stats(struct dsp_stats *stats);

/* Reset performance statistics */
void dsp_reset_stats(uint32_t sample_rate_hz, uint32_t block_samples);

/* Retrieve active DSP configuration */
void dsp_get_config(struct dsp_config *cfg);

/* Software digital gain control (post-ADC digital multiplier, range 0.1f - 16.0f, default 1.0f) */
int dsp_set_digital_gain(float gain);
float dsp_get_digital_gain(void);
uint32_t dsp_get_digital_clipping_count(void);
uint32_t dsp_get_input_clipping_count(void);
uint32_t dsp_get_filter_overshoot_count(void);
uint32_t dsp_get_gain_clipping_count(void);
void dsp_reset_digital_clipping_count(void);
void dsp_reset_all_clipping_counts(void);

#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
/* Intermediate Stage Diagnostic Metrics */
struct stage_metrics {
	const char *name;
	int32_t min;
	int32_t max;
	int64_t sum;
	uint64_t sum_sq;
	uint32_t near_full_scale;     /* Samples within 5% of full scale (>= 31128 or <= -31129) */
	uint32_t clip_count;          /* Samples hitting full scale or float overshoot */
	uint32_t blocks;
	uint32_t sample_count;
	int32_t last_sample;
	bool has_last_sample;
	uint64_t boundary_delta_sum;
	uint32_t boundary_delta_max;
	uint32_t boundary_count;      /* Number of valid contiguous block boundaries accumulated */
	uint64_t inside_delta_sum;
	uint32_t inside_delta_max;
	uint32_t inside_delta_count;
	/* Post-capture Goertzel filters for 1x and 2x block frequency */
	float g_coeff[2];
	float g_s1[2];
	float g_s2[2];
	float g_mag[2];
	bool spectral_valid;          /* True only if contiguous window analyzed with zero buffer drops */
	uint32_t spectral_samples;    /* Number of contiguous samples in spectral window */
};

void dsp_stage_metrics_init(struct stage_metrics *m, const char *name, uint32_t sample_rate_hz, uint32_t block_samples);
void dsp_stage_metrics_update(struct stage_metrics *m, const float *samples, uint32_t count);
void dsp_stage_metrics_finish(struct stage_metrics *m);

/* Real-time bounded stage processing (zero Goertzel in producer/consumer path) */
void dsp_process_stage_diagnostics_realtime(const int16_t *raw_in, uint32_t count,
                                            struct stage_metrics stages_out[5],
                                            bool is_contiguous);

/* Backward compatibility wrapper */
void dsp_process_stage_diagnostics(const int16_t *raw_in, uint32_t count,
                                   struct stage_metrics stages_out[5]);

/* Post-capture spectral Goertzel analysis on contiguous reference window */
void dsp_run_post_capture_spectral_analysis(const int16_t *ref_in, uint32_t count,
                                            const struct biquad_df2t initial_stages[6],
                                            struct stage_metrics stages_out[5]);
#endif

#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS
/* Get snapshot of active biquad filter states */
void dsp_get_stage_filters(struct biquad_df2t dest[6]);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DSP_H_ */

