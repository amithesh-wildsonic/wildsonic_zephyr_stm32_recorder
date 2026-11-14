/* SPDX-License-Identifier: Apache-2.0 */
#include "dsp.h"
#include "rate_check.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>
#include "wildsonic_trace.h"

#if defined(CONFIG_CMSIS_DSP)
#include <arm_math.h>
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
#define NUM_STAGES        6
#define DSP_CHUNK_SAMPLES 128U

static struct biquad_df2t stages[NUM_STAGES];
static float cmsis_coeffs[NUM_STAGES * 5];
static float cmsis_state[NUM_STAGES * 2];
static uint32_t cmsis_num_active = 0;
#if defined(CONFIG_CMSIS_DSP)
static arm_biquad_cascade_df2T_instance_f32 cmsis_inst;
#endif
static bool high_rate_dc_only = false;
static int32_t dc_fx16_x_prev = 0;
static int32_t dc_fx16_y_prev = 0;
static int64_t dc_fx32_x_prev = 0;
static int64_t dc_fx32_y_prev = 0;
static struct dsp_config active_cfg;
static struct dsp_stats stats;
static uint32_t current_fs = 192000;
static float digital_gain = 1.0f;
static volatile uint32_t digital_clipping_count = 0;
static volatile uint32_t dsp_input_clipping_count = 0;
static volatile uint32_t dsp_filter_overshoot_count = 0;
static volatile uint32_t dsp_gain_clipping_count = 0;

static void dsp_rebuild_cascade(void)
{
	uint32_t n = 0;
	for (int s = 0; s < NUM_STAGES; s++) {
		if (!stages[s].enabled) {
			continue;
		}
		if (high_rate_dc_only && s > 0) {
			continue;
		}
		/* CMSIS DF2T computes d1 = b1*x + d2 + a1*acc; dsp stages negate a1/a2. */
		cmsis_coeffs[n * 5 + 0] = stages[s].b0;
		cmsis_coeffs[n * 5 + 1] = stages[s].b1;
		cmsis_coeffs[n * 5 + 2] = stages[s].b2;
		cmsis_coeffs[n * 5 + 3] = -stages[s].a1;
		cmsis_coeffs[n * 5 + 4] = -stages[s].a2;
		cmsis_state[n * 2 + 0] = 0.0f;
		cmsis_state[n * 2 + 1] = 0.0f;
		n++;
	}
	cmsis_num_active = n;
#if defined(CONFIG_CMSIS_DSP)
	if (n > 0U) {
		arm_biquad_cascade_df2T_init_f32(&cmsis_inst, (uint8_t)n, cmsis_coeffs, cmsis_state);
	}
#endif
}

static inline void dsp_run_cascade_f32(float *buf, uint32_t count)
{
	if (cmsis_num_active == 0U || count == 0U) {
		return;
	}
#if defined(CONFIG_CMSIS_DSP)
	arm_biquad_cascade_df2T_f32(&cmsis_inst, buf, buf, count);
#else
	for (uint32_t s = 0U; s < cmsis_num_active; s++) {
		const float b0 = cmsis_coeffs[s * 5U + 0U];
		const float b1 = cmsis_coeffs[s * 5U + 1U];
		const float b2 = cmsis_coeffs[s * 5U + 2U];
		const float a1 = cmsis_coeffs[s * 5U + 3U];
		const float a2 = cmsis_coeffs[s * 5U + 4U];
		float d1 = cmsis_state[s * 2U + 0U];
		float d2 = cmsis_state[s * 2U + 1U];
		for (uint32_t i = 0U; i < count; i++) {
			float xn = buf[i];
			float acc = b0 * xn + d1;
			d1 = b1 * xn + d2 + a1 * acc;
			d2 = b2 * xn + a2 * acc;
			buf[i] = acc;
		}
		cmsis_state[s * 2U + 0U] = d1;
		cmsis_state[s * 2U + 1U] = d2;
	}
#endif
}

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
		float nyquist_bound = (float)sample_rate_hz * 0.49f;
		if ((float)cfg->lpf_cutoff_hz >= nyquist_bound) {
			TRACE("LPF cutoff %u Hz >= Nyquist, bypassed", cfg->lpf_cutoff_hz);
			biquad_set_bypass(&stages[4]);
			biquad_set_bypass(&stages[5]);
		} else {
			/* Normalized Butterworth 4th order: Q1 = 0.54119610, Q2 = 1.30656296 */
			biquad_calc_lpf(&stages[4], (float)cfg->lpf_cutoff_hz, (float)sample_rate_hz, 0.54119610f);
			biquad_calc_lpf(&stages[5], (float)cfg->lpf_cutoff_hz, (float)sample_rate_hz, 1.30656296f);
		}
	} else {
		biquad_set_bypass(&stages[4]);
		biquad_set_bypass(&stages[5]);
	}

	dsp_rebuild_cascade();
	return 0;
}

void dsp_set_dc_only_mode(bool dc_only)
{
	if (high_rate_dc_only != dc_only) {
		high_rate_dc_only = dc_only;
		dsp_rebuild_cascade();
	}
}

bool dsp_get_dc_only_mode(void)
{
	return high_rate_dc_only;
}

uint32_t dsp_validate_lpf_cutoff(uint32_t sample_rate_hz, uint32_t requested_cutoff_hz)
{
	if (sample_rate_hz == 0 || requested_cutoff_hz == 0) {
		return 0;
	}

	uint32_t nyquist_bound = (uint32_t)((float)sample_rate_hz * 0.49f);
	if (requested_cutoff_hz < nyquist_bound) {
		return requested_cutoff_hz;
	}

	/* Requested cutoff exceeds or equals the safe 0.49 * fs Nyquist boundary.
	 * Adjust to the highest supported valid cutoff with sufficient filter stability margin. */
	if (sample_rate_hz >= 192000U) {
		return 90000U;
	} else if (sample_rate_hz >= 96000U) {
		return 45000U;
	} else if (sample_rate_hz >= 48000U) {
		return 20000U;
	} else {
		return (sample_rate_hz * 45U) / 100U;
	}
}

bool dsp_is_lpf_effective(void)
{
	return (active_cfg.lpf_enabled && active_cfg.lpf_cutoff_hz > 0 && stages[4].enabled);
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
	memset(cmsis_state, 0, sizeof(cmsis_state));
	dc_fx16_x_prev = 0;
	dc_fx16_y_prev = 0;
	dc_fx32_x_prev = 0;
	dc_fx32_y_prev = 0;
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

static void dsp_run_dc_only_i16(int16_t *samples, uint32_t count)
{
	int32_t x_prev = dc_fx16_x_prev;
	int32_t y_prev = dc_fx16_y_prev;

	for (uint32_t i = 0U; i < count; i++) {
		int16_t in_val = samples[i];
		if (in_val == 32767 || in_val == -32768) {
			dsp_input_clipping_count++;
		}
		int32_t x = (int32_t)in_val * 2048;
		int32_t diff = x - x_prev;
		x_prev = x;
		int32_t decay = (y_prev >= 0) ? ((y_prev + 1024) >> 11) : -((-y_prev + 1024) >> 11);
		int32_t y = diff + y_prev - decay;
		y_prev = y;
		int32_t out = (y >= 0) ? ((y + 1024) >> 11) : -((-y + 1024) >> 11);
		if (out > 32767) {
			samples[i] = 32767;
			digital_clipping_count++;
			dsp_filter_overshoot_count++;
		} else if (out < -32768) {
			samples[i] = -32768;
			digital_clipping_count++;
			dsp_filter_overshoot_count++;
		} else {
			samples[i] = (int16_t)out;
		}
	}

	dc_fx16_x_prev = x_prev;
	dc_fx16_y_prev = y_prev;
}

static void dsp_run_dc_only_i32(int32_t *samples, uint32_t count)
{
	int64_t x_prev = dc_fx32_x_prev;
	int64_t y_prev = dc_fx32_y_prev;

	for (uint32_t i = 0U; i < count; i++) {
		int32_t in_val = samples[i];
		if (in_val == INT32_MAX || in_val == INT32_MIN) {
			dsp_input_clipping_count++;
		}
		int64_t x = (int64_t)in_val;
		int64_t diff = x - x_prev;
		x_prev = x;
		int64_t decay = (y_prev >= 0) ? ((y_prev + 1024LL) >> 11) : -((-y_prev + 1024LL) >> 11);
		int64_t y = diff + y_prev - decay;
		y_prev = y;
		if (y > (int64_t)INT32_MAX) {
			samples[i] = INT32_MAX;
			digital_clipping_count++;
			dsp_filter_overshoot_count++;
		} else if (y < (int64_t)INT32_MIN) {
			samples[i] = INT32_MIN;
			digital_clipping_count++;
			dsp_filter_overshoot_count++;
		} else {
			samples[i] = (int32_t)y;
		}
	}

	dc_fx32_x_prev = x_prev;
	dc_fx32_y_prev = y_prev;
}

void dsp_process_i16(int16_t *samples, uint32_t count)
{
	if (!samples || count == 0U) {
		return;
	}

	uint64_t t0 = k_cycle_get_64();
	const uint32_t num_active = cmsis_num_active;
	const float gain = digital_gain;

	if (high_rate_dc_only && active_cfg.dc_block_enabled && gain == 1.0f) {
		dsp_run_dc_only_i16(samples, count);
	} else if (num_active > 0U || gain != 1.0f) {
		float work[DSP_CHUNK_SAMPLES];
		for (uint32_t off = 0U; off < count; off += DSP_CHUNK_SAMPLES) {
			uint32_t n = (count - off > DSP_CHUNK_SAMPLES) ? DSP_CHUNK_SAMPLES : (count - off);
			int16_t *p = &samples[off];

			for (uint32_t i = 0U; i < n; i++) {
				int16_t in_val = p[i];
				if (in_val == 32767 || in_val == -32768) {
					dsp_input_clipping_count++;
				}
				work[i] = (float)in_val;
			}

			if (num_active > 0U) {
				dsp_run_cascade_f32(work, n);
			}

			for (uint32_t i = 0U; i < n; i++) {
				float x = work[i] * gain;
				if (x >= 32767.0f) {
					p[i] = 32767;
					digital_clipping_count++;
					if (gain > 1.0f && work[i] <= 32767.0f) {
						dsp_gain_clipping_count++;
					} else if (work[i] > 32767.0f) {
						dsp_filter_overshoot_count++;
					}
				} else if (x <= -32768.0f) {
					p[i] = -32768;
					digital_clipping_count++;
					if (gain > 1.0f && work[i] >= -32768.0f) {
						dsp_gain_clipping_count++;
					} else if (work[i] < -32768.0f) {
						dsp_filter_overshoot_count++;
					}
				} else {
					long v = lrintf(x);
					p[i] = (v > 32767L) ? 32767 : ((v < -32768L) ? -32768 : (int16_t)v);
				}
			}
		}
	}

	uint64_t t1 = k_cycle_get_64();
	uint64_t cyc = (t1 >= t0) ? (t1 - t0) : 0ULL;
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
	if (!samples || count == 0U) {
		return;
	}

	uint64_t t0 = k_cycle_get_64();
	const uint32_t num_active = cmsis_num_active;
	const float gain = digital_gain;

	if (high_rate_dc_only && active_cfg.dc_block_enabled && gain == 1.0f) {
		dsp_run_dc_only_i32(samples, count);
	} else if (num_active > 0U || gain != 1.0f) {
		float work[DSP_CHUNK_SAMPLES];
		for (uint32_t off = 0U; off < count; off += DSP_CHUNK_SAMPLES) {
			uint32_t n = (count - off > DSP_CHUNK_SAMPLES) ? DSP_CHUNK_SAMPLES : (count - off);
			int32_t *p = &samples[off];

			for (uint32_t i = 0U; i < n; i++) {
				int32_t in_val = p[i];
				if (in_val == INT32_MAX || in_val == INT32_MIN) {
					dsp_input_clipping_count++;
				}
				work[i] = (float)in_val;
			}

			if (num_active > 0U) {
				dsp_run_cascade_f32(work, n);
			}

			for (uint32_t i = 0U; i < n; i++) {
				float x = work[i] * gain;
				/* 2147483520.0f is max finite float strictly < 2^31 (avoids INT32 UB/wrap). */
				if (x >= 2147483520.0f) {
					p[i] = INT32_MAX;
					digital_clipping_count++;
					if (gain > 1.0f && work[i] < 2147483520.0f) {
						dsp_gain_clipping_count++;
					} else if (work[i] >= 2147483520.0f) {
						dsp_filter_overshoot_count++;
					}
				} else if (x <= -2147483648.0f) {
					p[i] = INT32_MIN;
					digital_clipping_count++;
					if (gain > 1.0f && work[i] > -2147483648.0f) {
						dsp_gain_clipping_count++;
					} else if (work[i] <= -2147483648.0f) {
						dsp_filter_overshoot_count++;
					}
				} else {
					p[i] = (int32_t)lrintf(x);
				}
			}
		}
	}

	uint64_t t1 = k_cycle_get_64();
	uint64_t cyc = (t1 >= t0) ? (t1 - t0) : 0ULL;
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

void dsp_calibrate_cost(void)
{
	/* Measure real ns/sample on target hardware for DC-only and Full active cascade. */
	struct dsp_config saved_cfg = active_cfg;
	uint32_t saved_fs = current_fs;
	bool saved_dc_only = high_rate_dc_only;
	float saved_gain = digital_gain;
	struct dsp_stats saved_stats = stats;

	int16_t dummy[256];
	for (uint32_t i = 0; i < 256U; i++) {
		dummy[i] = (int16_t)(i * 17U - 2048);
	}

	/* 1. Bypass / gain-only path */
	struct dsp_config bypass_cfg = { 0 };
	digital_gain = 1.25f;
	high_rate_dc_only = false;
	dsp_configure(&bypass_cfg, saved_fs ? saved_fs : 48000U);
	uint64_t t0 = k_cycle_get_64();
	dsp_process_i16(dummy, 256U);
	uint64_t cyc_bypass = k_cycle_get_64() - t0;

	/* 2. DC-block-only path (fixed-point DC blocker) */
	struct dsp_config dc_cfg = saved_cfg;
	dc_cfg.dc_block_enabled = true;
	digital_gain = 1.0f;
	high_rate_dc_only = true;
	dsp_configure(&dc_cfg, saved_fs ? saved_fs : 384000U);
	t0 = k_cycle_get_64();
	dsp_process_i16(dummy, 256U);
	uint64_t cyc_dc = k_cycle_get_64() - t0;

	/* 3. Full active filter path */
	digital_gain = 1.0f;
	high_rate_dc_only = false;
	dsp_configure(&saved_cfg, saved_fs ? saved_fs : 48000U);
	t0 = k_cycle_get_64();
	dsp_process_i16(dummy, 256U);
	uint64_t cyc_full = k_cycle_get_64() - t0;

	uint32_t ns_bypass = (uint32_t)((k_cyc_to_ns_ceil64(cyc_bypass) + 128ULL) / 256ULL);
	uint32_t ns_dc     = (uint32_t)((k_cyc_to_ns_ceil64(cyc_dc) + 128ULL) / 256ULL);
	uint32_t ns_full   = (uint32_t)((k_cyc_to_ns_ceil64(cyc_full) + 128ULL) / 256ULL);

	rate_check_set_measured_dsp_ns(ns_bypass ? ns_bypass : 1U,
				       ns_dc ? ns_dc : 1U,
				       ns_full ? ns_full : 1U);

	/* Restore live state cleanly */
	digital_gain = saved_gain;
	high_rate_dc_only = saved_dc_only;
	dsp_configure(&saved_cfg, saved_fs);
	dsp_reset();
	dsp_reset_all_clipping_counts();
	stats = saved_stats;
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
		TRACE("Digital gain %.2f out of range [0.1..16.0]", (double)gain);
		return -EINVAL;
	}
	digital_gain = gain;
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




