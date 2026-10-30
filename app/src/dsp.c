/* SPDX-License-Identifier: Apache-2.0 */
#include "dsp.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(dsp, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(dsp, LOG_LEVEL_WRN);
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Five processing stages:
 * 0: DC Blocker (20 Hz)
 * 1: High-Pass Filter (e.g. 80 Hz, Q = 0.7071)
 * 2: Mains Notch Filter (50 Hz or 60 Hz, configurable Q)
 * 3: Mains Harmonic Notch Filter (100 Hz or 120 Hz)
 * 4: Low-Pass Filter Stage 1 (e.g. 20 kHz, Q = 0.54119610)
 * 5: Low-Pass Filter Stage 2 (e.g. 20 kHz, Q = 1.30656296)
 */
#define NUM_STAGES 6
static struct biquad_df2t stages[NUM_STAGES];
static struct dsp_config active_cfg;
static struct dsp_stats stats;
static uint32_t current_fs = 192000;
static float digital_gain = 1.0f;
static volatile uint32_t digital_clipping_count = 0;
static volatile uint32_t dsp_input_clipping_count = 0;
static volatile uint32_t dsp_filter_overshoot_count = 0;
static volatile uint32_t dsp_gain_clipping_count = 0;

static inline float biquad_step(struct biquad_df2t *bq, float in)
{
	if (!bq->enabled) {
		return in;
	}
	float out = bq->b0 * in + bq->s1;
	bq->s1 = bq->b1 * in - bq->a1 * out + bq->s2;
	bq->s2 = bq->b2 * in - bq->a2 * out;
	return out;
}

static void biquad_set_bypass(struct biquad_df2t *bq)
{
	bq->b0 = 1.0f;
	bq->b1 = 0.0f;
	bq->b2 = 0.0f;
	bq->a1 = 0.0f;
	bq->a2 = 0.0f;
	bq->s1 = 0.0f;
	bq->s2 = 0.0f;
	bq->enabled = false;
}

static void biquad_calc_hpf(struct biquad_df2t *bq, float fc, float fs, float Q)
{
	if (fc <= 0.0f || fc >= (fs * 0.49f)) {
		biquad_set_bypass(bq);
		return;
	}
	float w0 = (float)(2.0 * M_PI * (double)fc / (double)fs);
	float cos_w0 = cosf(w0);
	float sin_w0 = sinf(w0);
	float alpha = sin_w0 / (2.0f * Q);

	float b0 = (1.0f + cos_w0) * 0.5f;
	float b1 = -(1.0f + cos_w0);
	float b2 = (1.0f + cos_w0) * 0.5f;
	float a0 = 1.0f + alpha;
	float a1 = -2.0f * cos_w0;
	float a2 = 1.0f - alpha;

	float inv_a0 = 1.0f / a0;
	bq->b0 = b0 * inv_a0;
	bq->b1 = b1 * inv_a0;
	bq->b2 = b2 * inv_a0;
	bq->a1 = a1 * inv_a0;
	bq->a2 = a2 * inv_a0;
	bq->s1 = 0.0f;
	bq->s2 = 0.0f;
	bq->enabled = true;
}

static void biquad_calc_lpf(struct biquad_df2t *bq, float fc, float fs, float Q)
{
	if (fc <= 0.0f || fc >= (fs * 0.49f)) {
		biquad_set_bypass(bq);
		return;
	}
	float w0 = (float)(2.0 * M_PI * (double)fc / (double)fs);
	float cos_w0 = cosf(w0);
	float sin_w0 = sinf(w0);
	float alpha = sin_w0 / (2.0f * Q);

	float b0 = (1.0f - cos_w0) * 0.5f;
	float b1 = 1.0f - cos_w0;
	float b2 = (1.0f - cos_w0) * 0.5f;
	float a0 = 1.0f + alpha;
	float a1 = -2.0f * cos_w0;
	float a2 = 1.0f - alpha;

	float inv_a0 = 1.0f / a0;
	bq->b0 = b0 * inv_a0;
	bq->b1 = b1 * inv_a0;
	bq->b2 = b2 * inv_a0;
	bq->a1 = a1 * inv_a0;
	bq->a2 = a2 * inv_a0;
	bq->s1 = 0.0f;
	bq->s2 = 0.0f;
	bq->enabled = true;
}

static void biquad_calc_notch(struct biquad_df2t *bq, float f0, float fs, float Q)
{
	if (f0 <= 0.0f || f0 >= (fs * 0.49f) || Q <= 0.1f) {
		biquad_set_bypass(bq);
		return;
	}
	float w0 = (float)(2.0 * M_PI * (double)f0 / (double)fs);
	float cos_w0 = cosf(w0);
	float sin_w0 = sinf(w0);
	float alpha = sin_w0 / (2.0f * Q);

	float b0 = 1.0f;
	float b1 = -2.0f * cos_w0;
	float b2 = 1.0f;
	float a0 = 1.0f + alpha;
	float a1 = -2.0f * cos_w0;
	float a2 = 1.0f - alpha;

	float inv_a0 = 1.0f / a0;
	bq->b0 = b0 * inv_a0;
	bq->b1 = b1 * inv_a0;
	bq->b2 = b2 * inv_a0;
	bq->a1 = a1 * inv_a0;
	bq->a2 = a2 * inv_a0;
	bq->s1 = 0.0f;
	bq->s2 = 0.0f;
	bq->enabled = true;
}

static void biquad_calc_dc_block(struct biquad_df2t *bq, float fc, float fs)
{
	/* 1st-order DC blocking filter in biquad form:
	 * H(z) = (1 - z^-1) / (1 - R * z^-1), R = 1 - (2*pi*fc/fs)
	 */
	float R = 1.0f - (float)(2.0 * M_PI * (double)fc / (double)fs);
	if (R > 0.9999f) R = 0.9999f;
	if (R < 0.9000f) R = 0.9000f;

	float scale = (1.0f + R) * 0.5f;
	bq->b0 = scale;
	bq->b1 = -scale;
	bq->b2 = 0.0f;
	bq->a1 = -R;
	bq->a2 = 0.0f;
	bq->s1 = 0.0f;
	bq->s2 = 0.0f;
	bq->enabled = true;
}

int dsp_configure(const struct dsp_config *cfg, uint32_t sample_rate_hz)
{
	if (!cfg || sample_rate_hz == 0) {
		return -EINVAL;
	}
	current_fs = sample_rate_hz;
	active_cfg = *cfg;

	/* Stage 0: DC Blocker (20 Hz) */
	if (cfg->dc_block_enabled) {
		biquad_calc_dc_block(&stages[0], 20.0f, (float)sample_rate_hz);
	} else {
		biquad_set_bypass(&stages[0]);
	}

	/* Stage 1: High-Pass Filter (e.g. 80 Hz, Butterworth Q = 0.7071) */
	if (cfg->hpf_enabled && cfg->hpf_cutoff_hz > 0) {
		biquad_calc_hpf(&stages[1], (float)cfg->hpf_cutoff_hz, (float)sample_rate_hz, 0.70710678f);
	} else {
		biquad_set_bypass(&stages[1]);
	}

	/* Stage 2: Mains Notch Filter (50 Hz or 60 Hz) */
	if (cfg->notch_enabled && cfg->notch_freq_hz > 0) {
		float q = (cfg->notch_q > 0.5f) ? cfg->notch_q : 10.0f;
		biquad_calc_notch(&stages[2], (float)cfg->notch_freq_hz, (float)sample_rate_hz, q);
	} else {
		biquad_set_bypass(&stages[2]);
	}

	/* Stage 3: Mains Harmonic Notch (2x mains freq) */
	if (cfg->harmonic_notch_enabled && cfg->harmonic_freq_hz > 0) {
		float q = (cfg->harmonic_q > 0.5f) ? cfg->harmonic_q : 10.0f;
		biquad_calc_notch(&stages[3], (float)cfg->harmonic_freq_hz, (float)sample_rate_hz, q);
	} else {
		biquad_set_bypass(&stages[3]);
	}

	/* Stages 4 & 5: 4th-Order Butterworth Low-Pass Filter (2 cascaded biquads) */
	if (cfg->lpf_enabled && cfg->lpf_cutoff_hz > 0) {
		/* Normalized Butterworth 4th order: Q1 = 0.54119610, Q2 = 1.30656296 */
		biquad_calc_lpf(&stages[4], (float)cfg->lpf_cutoff_hz, (float)sample_rate_hz, 0.54119610f);
		biquad_calc_lpf(&stages[5], (float)cfg->lpf_cutoff_hz, (float)sample_rate_hz, 1.30656296f);
	} else {
		biquad_set_bypass(&stages[4]);
		biquad_set_bypass(&stages[5]);
	}

	LOG_INF("DSP configured for %u Hz: DC_Block=%d, HPF=%d (%u Hz), Notch=%d (%u Hz, Q=%.1f), LPF=%d (%u Hz, 4th-order)",
	        sample_rate_hz, cfg->dc_block_enabled, cfg->hpf_enabled, cfg->hpf_cutoff_hz,
	        cfg->notch_enabled, cfg->notch_freq_hz, (double)cfg->notch_q,
	        cfg->lpf_enabled, cfg->lpf_cutoff_hz);

	return 0;
}

int dsp_init(uint32_t sample_rate_hz)
{
	struct dsp_config def_cfg = {
#ifdef CONFIG_AUDIO_DSP_DC_BLOCK
		.dc_block_enabled = true,
#else
		.dc_block_enabled = false,
#endif

#ifdef CONFIG_AUDIO_DSP_HPF
		.hpf_enabled = true,
		.hpf_cutoff_hz = CONFIG_AUDIO_DSP_HPF_CUTOFF_HZ,
#else
		.hpf_enabled = false,
		.hpf_cutoff_hz = 80,
#endif

#ifdef CONFIG_AUDIO_DSP_NOTCH
		.notch_enabled = true,
		.notch_freq_hz = CONFIG_AUDIO_DSP_NOTCH_FREQ_HZ,
		.notch_q = (float)CONFIG_AUDIO_DSP_NOTCH_Q_X10 / 10.0f,
#else
		.notch_enabled = false,
		.notch_freq_hz = 50,
		.notch_q = 10.0f,
#endif

#ifdef CONFIG_AUDIO_DSP_HARMONIC_NOTCH
		.harmonic_notch_enabled = true,
		.harmonic_freq_hz = CONFIG_AUDIO_DSP_NOTCH_FREQ_HZ * 2,
		.harmonic_q = 10.0f,
#else
		.harmonic_notch_enabled = false,
		.harmonic_freq_hz = 100,
		.harmonic_q = 10.0f,
#endif

#ifdef CONFIG_AUDIO_DSP_LPF
		.lpf_enabled = true,
		.lpf_cutoff_hz = CONFIG_AUDIO_DSP_LPF_CUTOFF_HZ,
#else
		.lpf_enabled = false,
		.lpf_cutoff_hz = 20000,
#endif
	};

	dsp_reset_stats(sample_rate_hz, CONFIG_AUDIO_BLOCK_SAMPLES);
	return dsp_configure(&def_cfg, sample_rate_hz);
}

void dsp_reset(void)
{
	for (int i = 0; i < NUM_STAGES; i++) {
		stages[i].s1 = 0.0f;
		stages[i].s2 = 0.0f;
	}
}

void dsp_reset_stats(uint32_t sample_rate_hz, uint32_t block_samples)
{
	memset(&stats, 0, sizeof(stats));
	if (sample_rate_hz > 0 && block_samples > 0) {
		stats.block_deadline_us = (uint32_t)(((uint64_t)block_samples * 1000000ULL) / sample_rate_hz);
	} else {
		stats.block_deadline_us = 10666; /* default 2048 @ 192 kHz */
	}
}

void dsp_get_stats(struct dsp_stats *out)
{
	if (!out) return;
	*out = stats;
	if (stats.blocks_processed > 0) {
		out->avg_cycles = (uint32_t)(stats.total_cycles / stats.blocks_processed);
		out->avg_us = (uint32_t)k_cyc_to_us_floor64(stats.total_cycles / stats.blocks_processed);
	}
}

void dsp_process_i16(int16_t *samples, uint32_t count)
{
	if (!samples || count == 0) return;

	uint64_t t0 = k_cycle_get_64();

	for (uint32_t i = 0; i < count; i++) {
		int16_t in_val = samples[i];
		if (in_val >= 32767 || in_val <= -32768) {
			dsp_input_clipping_count++;
		}

		float x = (float)in_val;

		/* Cascade through all active filter stages */
		x = biquad_step(&stages[0], x); /* DC block */
		x = biquad_step(&stages[1], x); /* HPF */
		x = biquad_step(&stages[2], x); /* Notch */
		x = biquad_step(&stages[3], x); /* Harmonic Notch */
		x = biquad_step(&stages[4], x); /* LPF 1 */
		x = biquad_step(&stages[5], x); /* LPF 2 */

		/* Detect filter overshoot before digital gain */
		if ((x > 32767.0f || x < -32768.0f) && !(in_val >= 32767 || in_val <= -32768)) {
			dsp_filter_overshoot_count++;
		}

		/* Apply post-filtering software digital gain */
		if (digital_gain != 1.0f) {
			float x_pre_gain = x;
			x *= digital_gain;
			if ((x > 32767.0f || x < -32768.0f) &&
			    (x_pre_gain <= 32767.0f && x_pre_gain >= -32768.0f)) {
				dsp_gain_clipping_count++;
			}
		}

		/* Saturate to 16-bit signed PCM range without integer wraparound */
		if (x > 32767.0f) {
			x = 32767.0f;
			digital_clipping_count++;
		} else if (x < -32768.0f) {
			x = -32768.0f;
			digital_clipping_count++;
		}
		samples[i] = (int16_t)x;
	}

	uint64_t t1 = k_cycle_get_64();
	uint64_t cyc = (t1 >= t0) ? (t1 - t0) : 0;
	uint32_t us = (uint32_t)k_cyc_to_us_floor64(cyc);

	stats.total_cycles += cyc;
	stats.blocks_processed++;
	stats.samples_processed += count;
	if (cyc > stats.worst_cycles) {
		stats.worst_cycles = (uint32_t)cyc;
		stats.worst_us = us;
	}
	if (us > stats.block_deadline_us) {
		stats.overruns++;
	}
}

void dsp_process_i32(int32_t *samples, uint32_t count)
{
	if (!samples || count == 0) return;

	uint64_t t0 = k_cycle_get_64();

	for (uint32_t i = 0; i < count; i++) {
		int32_t in_val = samples[i];
		if (in_val >= 2147483647 || in_val <= -2147483648) {
			dsp_input_clipping_count++;
		}

		float x = (float)in_val;

		x = biquad_step(&stages[0], x);
		x = biquad_step(&stages[1], x);
		x = biquad_step(&stages[2], x);
		x = biquad_step(&stages[3], x);
		x = biquad_step(&stages[4], x);
		x = biquad_step(&stages[5], x);

		/* Detect filter overshoot before digital gain */
		if ((x > 2147483647.0f || x < -2147483648.0f) &&
		    !(in_val >= 2147483647 || in_val <= -2147483648)) {
			dsp_filter_overshoot_count++;
		}

		/* Apply post-filtering software digital gain */
		if (digital_gain != 1.0f) {
			float x_pre_gain = x;
			x *= digital_gain;
			if ((x > 2147483647.0f || x < -2147483648.0f) &&
			    (x_pre_gain <= 2147483647.0f && x_pre_gain >= -2147483648.0f)) {
				dsp_gain_clipping_count++;
			}
		}

		if (x > 2147483647.0f) {
			x = 2147483647.0f;
			digital_clipping_count++;
		} else if (x < -2147483648.0f) {
			x = -2147483648.0f;
			digital_clipping_count++;
		}
		samples[i] = (int32_t)x;
	}

	uint64_t t1 = k_cycle_get_64();
	uint64_t cyc = (t1 >= t0) ? (t1 - t0) : 0;
	uint32_t us = (uint32_t)k_cyc_to_us_floor64(cyc);

	stats.total_cycles += cyc;
	stats.blocks_processed++;
	stats.samples_processed += count;
	if (cyc > stats.worst_cycles) {
		stats.worst_cycles = (uint32_t)cyc;
		stats.worst_us = us;
	}
	if (us > stats.block_deadline_us) {
		stats.overruns++;
	}
}



void dsp_get_config(struct dsp_config *cfg)
{
	if (cfg) {
		*cfg = active_cfg;
	}
}

int dsp_set_digital_gain(float gain)
{
	if (gain < 0.1f || gain > 16.0f) {
		LOG_ERR("Digital gain %.2f out of safe range [0.1 .. 16.0]", (double)gain);
		return -EINVAL;
	}
	digital_gain = gain;
	LOG_INF("Digital software gain set to %.2fx", (double)gain);
	return 0;
}

float dsp_get_digital_gain(void)
{
	return digital_gain;
}

uint32_t dsp_get_digital_clipping_count(void)
{
	return digital_clipping_count;
}

uint32_t dsp_get_input_clipping_count(void)
{
	return dsp_input_clipping_count;
}

uint32_t dsp_get_filter_overshoot_count(void)
{
	return dsp_filter_overshoot_count;
}

uint32_t dsp_get_gain_clipping_count(void)
{
	return dsp_gain_clipping_count;
}

void dsp_reset_digital_clipping_count(void)
{
	digital_clipping_count = 0;
}

void dsp_reset_all_clipping_counts(void)
{
	digital_clipping_count = 0;
	dsp_input_clipping_count = 0;
	dsp_filter_overshoot_count = 0;
	dsp_gain_clipping_count = 0;
}




