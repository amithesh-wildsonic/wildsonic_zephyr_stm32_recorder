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

#ifdef __cplusplus
}
#endif

#endif /* DSP_H_ */

