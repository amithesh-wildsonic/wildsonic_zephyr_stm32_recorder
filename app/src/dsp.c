/* SPDX-License-Identifier: Apache-2.0 */
#include "dsp.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dsp, LOG_LEVEL_INF);

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
static uint32_t test_phase_acc;
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
	test_phase_acc = 0;
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

/*
 * Test Signal Generator:
 * Generates a calibrated composite waveform:
 *   - 1000 Hz reference tone (amplitude 8192, -12 dBFS)
 *   - 50 Hz mains interference (amplitude 4096, -18 dBFS)
 *   - 40 kHz ultrasonic EMI hash (amplitude 4096, -18 dBFS)
 *   - DC offset of +1500 counts (~120 mV DC shift)
 *
 * When filtered:
 *   - DC is eliminated to ~0.
 *   - 50 Hz is suppressed by HPF / Notch.
 *   - 40 kHz is eliminated by 4th-order LPF.
 *   - 1000 Hz reference tone is preserved within 0.1 dB!
 */
void dsp_generate_test_signal_i16(int16_t *dst, uint32_t count, uint32_t sample_rate_hz)
{
	if (!dst || count == 0 || sample_rate_hz == 0) return;

	double dt = 1.0 / (double)sample_rate_hz;
	for (uint32_t i = 0; i < count; i++) {
		double t = (double)test_phase_acc * dt;
		double sig = 1500.0; /* DC offset */
		sig += 8192.0 * sin(2.0 * M_PI * 1000.0 * t);   /* 1 kHz ref tone */
		sig += 4096.0 * sin(2.0 * M_PI * 50.0 * t);     /* 50 Hz mains hum */
		sig += 4096.0 * sin(2.0 * M_PI * 40000.0 * t);  /* 40 kHz ultrasound */

		if (sig > 32767.0) sig = 32767.0;
		if (sig < -32768.0) sig = -32768.0;
		dst[i] = (int16_t)sig;

		test_phase_acc++;
		if (test_phase_acc >= sample_rate_hz) {
			test_phase_acc = 0;
		}
	}
}

void dsp_generate_test_signal_i32(int32_t *dst, uint32_t count, uint32_t sample_rate_hz)
{
	if (!dst || count == 0 || sample_rate_hz == 0) return;

	double dt = 1.0 / (double)sample_rate_hz;
	for (uint32_t i = 0; i < count; i++) {
		double t = (double)test_phase_acc * dt;
		double sig = 98304000.0; /* DC offset */
		sig += 536870912.0 * sin(2.0 * M_PI * 1000.0 * t);
		sig += 268435456.0 * sin(2.0 * M_PI * 50.0 * t);
		sig += 268435456.0 * sin(2.0 * M_PI * 40000.0 * t);

		if (sig > 2147483647.0) sig = 2147483647.0;
		if (sig < -2147483648.0) sig = -2147483648.0;
		dst[i] = (int32_t)sig;

		test_phase_acc++;
		if (test_phase_acc >= sample_rate_hz) {
			test_phase_acc = 0;
		}
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

#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS
void dsp_get_stage_filters(struct biquad_df2t dest[6])
{
	if (dest) {
		memcpy(dest, stages, sizeof(stages));
	}
}
#endif

#if CONFIG_AUDIO_STAGE_DIAGNOSTICS

void dsp_stage_metrics_init(struct stage_metrics *m, const char *name, uint32_t sample_rate_hz, uint32_t block_samples)
{
	if (!m) return;
	memset(m, 0, sizeof(*m));
	m->name = name;
	m->min = 2147483647;
	m->max = -2147483647;
	m->has_last_sample = false;
	m->boundary_count = 0;
	m->spectral_valid = false;
	m->spectral_samples = 0;
	m->g_mag[0] = -120.0f;
	m->g_mag[1] = -120.0f;

	if (sample_rate_hz > 0 && block_samples > 0) {
		float f_block = (float)sample_rate_hz / (float)block_samples;
		for (int k = 1; k <= 2; k++) {
			float w = (float)(2.0 * M_PI * (double)(k * f_block) / (double)sample_rate_hz);
			m->g_coeff[k - 1] = 2.0f * cosf(w);
		}
	}
}

static inline void dsp_stage_metrics_update_sample(struct stage_metrics *m, float f, bool is_first_in_block)
{
	int32_t v = (int32_t)f;
	if (v < m->min) m->min = v;
	if (v > m->max) m->max = v;
	m->sum += v;
	m->sum_sq += ((int64_t)v * (int64_t)v);

	if (v >= 31128 || v <= -31129) {
		m->near_full_scale++;
	}
	if (v >= 32767 || v <= -32768) {
		m->clip_count++;
	}

	if (is_first_in_block) {
		if (m->has_last_sample) {
			uint32_t delta = (uint32_t)abs(v - m->last_sample);
			m->boundary_delta_sum += delta;
			if (delta > m->boundary_delta_max) {
				m->boundary_delta_max = delta;
			}
			m->boundary_count++;
		}
	} else {
		uint32_t delta = (uint32_t)abs(v - m->last_sample);
		m->inside_delta_sum += delta;
		if (delta > m->inside_delta_max) {
			m->inside_delta_max = delta;
		}
		m->inside_delta_count++;
	}
	m->last_sample = v;
	m->has_last_sample = true;

	/* Goertzel step */
	for (int k = 0; k < 2; k++) {
		float s0 = f + m->g_coeff[k] * m->g_s1[k] - m->g_s2[k];
		m->g_s2[k] = m->g_s1[k];
		m->g_s1[k] = s0;
	}
	m->sample_count++;
}

void dsp_stage_metrics_update(struct stage_metrics *m, const float *samples, uint32_t count)
{
	if (!m || !samples || count == 0) return;
	for (uint32_t i = 0; i < count; i++) {
		dsp_stage_metrics_update_sample(m, samples[i], (i == 0));
	}
	m->blocks++;
}

void dsp_stage_metrics_finish(struct stage_metrics *m)
{
	if (!m) return;
	if (m->spectral_valid) return;

	/* If spectral analysis was not run post-capture, set default -120 dBFS */
	m->g_mag[0] = -120.0f;
	m->g_mag[1] = -120.0f;
}

void dsp_process_stage_diagnostics_realtime(const int16_t *raw_in, uint32_t count,
                                            struct stage_metrics stages_out[5],
                                            bool is_contiguous)
{
	if (!raw_in || count == 0 || !stages_out) return;

	/* ---- Pass 1: Stage A Metrics (Raw ADC) ---- */
	struct stage_metrics *mA = &stages_out[0];
	int32_t minA = mA->min, maxA = mA->max;
	int64_t sumA = mA->sum;
	uint64_t sum_sqA = mA->sum_sq;
	uint32_t clipA = mA->clip_count, near_fsA = mA->near_full_scale;
	uint64_t in_d_sumA = mA->inside_delta_sum;
	uint32_t in_d_maxA = mA->inside_delta_max;
	uint32_t in_d_cntA = mA->inside_delta_count;
	int32_t lastA = mA->last_sample;
	bool has_lastA = mA->has_last_sample;

	/* Boundary step for Stage A (only if contiguous) */
	int32_t v0 = (int32_t)raw_in[0];
	if (is_contiguous && has_lastA) {
		uint32_t b_delta = (uint32_t)abs(v0 - lastA);
		mA->boundary_delta_sum += b_delta;
		if (b_delta > mA->boundary_delta_max) {
			mA->boundary_delta_max = b_delta;
		}
		mA->boundary_count++;
	}

	for (uint32_t i = 0; i < count; i++) {
		int32_t v = (int32_t)raw_in[i];
		if (v < minA) minA = v;
		if (v > maxA) maxA = v;
		sumA += v;
		sum_sqA += (uint64_t)((int64_t)v * v);

		if (v >= 31128 || v <= -31129) near_fsA++;
		/* Rail hit: ground (<= -32768) or supply (>= +32752) */
		if (v >= 32752 || v <= -32768) clipA++;

		if (i > 0) {
			uint32_t d = (uint32_t)abs(v - lastA);
			in_d_sumA += d;
			if (d > in_d_maxA) in_d_maxA = d;
			in_d_cntA++;
		}
		lastA = v;
	}

	mA->min = minA; mA->max = maxA; mA->sum = sumA; mA->sum_sq = sum_sqA;
	mA->near_full_scale = near_fsA; mA->clip_count = clipA;
	mA->inside_delta_sum = in_d_sumA; mA->inside_delta_max = in_d_maxA; mA->inside_delta_count = in_d_cntA;
	mA->last_sample = lastA; mA->has_last_sample = true;
	mA->sample_count += count; mA->blocks++;

	/* ---- Pass 2: Stages B, C, D, E Filters & Metrics ---- */
	struct stage_metrics *mB = &stages_out[1];
	struct stage_metrics *mC = &stages_out[2];
	struct stage_metrics *mD = &stages_out[3];
	struct stage_metrics *mE = &stages_out[4];

	float minB = (float)mB->min, maxB = (float)mB->max;
	float minC = (float)mC->min, maxC = (float)mC->max;
	float minD = (float)mD->min, maxD = (float)mD->max;
	float minE = (float)mE->min, maxE = (float)mE->max;

	uint32_t ovrB = mB->clip_count;
	uint32_t ovrC = mC->clip_count;
	uint32_t ovrD = mD->clip_count;
	uint32_t clipE = mE->clip_count;
	uint32_t nfsE = mE->near_full_scale;

	float sum_sq_norm_B = 0.0f;
	float sum_sq_norm_C = 0.0f;
	float sum_sq_norm_D = 0.0f;
	float sum_sq_norm_E = 0.0f;

	float sum_norm_B = 0.0f;
	float sum_norm_C = 0.0f;
	float sum_norm_D = 0.0f;
	float sum_norm_E = 0.0f;

	const float inv_scale = 1.0f / 32768.0f;
	const double scale_sq = 1073741824.0; /* 32768^2 */

	int32_t lastE = mE->last_sample;
	bool has_lastE = mE->has_last_sample;
	uint64_t in_d_sumE = mE->inside_delta_sum;
	uint32_t in_d_maxE = mE->inside_delta_max;
	uint32_t in_d_cntE = mE->inside_delta_count;

	for (uint32_t i = 0; i < count; i++) {
		float xA = (float)raw_in[i];

		/* Stage B: DC Blocker */
		float xB = biquad_step(&stages[0], xA);
		if (xB < minB) minB = xB;
		if (xB > maxB) maxB = xB;
		if (fabsf(xB) > 32767.0f) ovrB++;
		float uB = xB * inv_scale;
		sum_norm_B += uB;
		sum_sq_norm_B += uB * uB;

		/* Stage C: HPF + Notches */
		float xC = biquad_step(&stages[1], xB);
		xC = biquad_step(&stages[2], xC);
		xC = biquad_step(&stages[3], xC);
		if (xC < minC) minC = xC;
		if (xC > maxC) maxC = xC;
		if (fabsf(xC) > 32767.0f) ovrC++;
		float uC = xC * inv_scale;
		sum_norm_C += uC;
		sum_sq_norm_C += uC * uC;

		/* Stage D: LPF 4th order */
		float xD = biquad_step(&stages[4], xC);
		xD = biquad_step(&stages[5], xD);
		if (xD < minD) minD = xD;
		if (xD > maxD) maxD = xD;
		if (fabsf(xD) > 32767.0f) ovrD++;
		float uD = xD * inv_scale;
		sum_norm_D += uD;
		sum_sq_norm_D += uD * uD;

		/* Stage E: Output PCM with gain and saturating clamp */
		float xE = xD;
		if (digital_gain != 1.0f) xE *= digital_gain;
		if (xE > 32767.0f) { xE = 32767.0f; clipE++; }
		else if (xE < -32768.0f) { xE = -32768.0f; clipE++; }

		if (xE < minE) minE = xE;
		if (xE > maxE) maxE = xE;
		float uE = xE * inv_scale;
		sum_norm_E += uE;
		sum_sq_norm_E += uE * uE;
		if (fabsf(xE) >= 31128.0f) nfsE++;

		int32_t ivE = (int32_t)xE;
		if (i == 0) {
			if (is_contiguous && has_lastE) {
				uint32_t b_deltaE = (uint32_t)abs(ivE - lastE);
				mE->boundary_delta_sum += b_deltaE;
				if (b_deltaE > mE->boundary_delta_max) {
					mE->boundary_delta_max = b_deltaE;
				}
				mE->boundary_count++;
			}
		} else {
			uint32_t dE = (uint32_t)abs(ivE - lastE);
			in_d_sumE += dE;
			if (dE > in_d_maxE) in_d_maxE = dE;
			in_d_cntE++;
		}
		lastE = ivE;
	}

	mB->min = (int32_t)minB; mB->max = (int32_t)maxB;
	mB->sum += (int64_t)(sum_norm_B * 32768.0f);
	mB->sum_sq += (uint64_t)((double)sum_sq_norm_B * scale_sq);
	mB->clip_count = ovrB;
	mB->sample_count += count; mB->blocks++;

	mC->min = (int32_t)minC; mC->max = (int32_t)maxC;
	mC->sum += (int64_t)(sum_norm_C * 32768.0f);
	mC->sum_sq += (uint64_t)((double)sum_sq_norm_C * scale_sq);
	mC->clip_count = ovrC;
	mC->sample_count += count; mC->blocks++;

	mD->min = (int32_t)minD; mD->max = (int32_t)maxD;
	mD->sum += (int64_t)(sum_norm_D * 32768.0f);
	mD->sum_sq += (uint64_t)((double)sum_sq_norm_D * scale_sq);
	mD->clip_count = ovrD;
	mD->sample_count += count; mD->blocks++;

	mE->min = (int32_t)minE; mE->max = (int32_t)maxE;
	mE->sum += (int64_t)(sum_norm_E * 32768.0f);
	mE->sum_sq += (uint64_t)((double)sum_sq_norm_E * scale_sq);
	mE->clip_count = clipE;
	mE->near_full_scale = nfsE;
	mE->inside_delta_sum = in_d_sumE;
	mE->inside_delta_max = in_d_maxE;
	mE->inside_delta_count = in_d_cntE;
	mE->last_sample = lastE;
	mE->has_last_sample = true;
	mE->sample_count += count; mE->blocks++;
}

void dsp_process_stage_diagnostics(const int16_t *raw_in, uint32_t count, struct stage_metrics stages_out[5])
{
	dsp_process_stage_diagnostics_realtime(raw_in, count, stages_out, true);
}

void dsp_run_post_capture_spectral_analysis(const int16_t *ref_in, uint32_t count,
                                            const struct biquad_df2t initial_stages[6],
                                            struct stage_metrics stages_out[5])
{
	if (!ref_in || count == 0 || !stages_out) return;

	struct biquad_df2t local_stages[6];
	if (initial_stages) {
		memcpy(local_stages, initial_stages, sizeof(local_stages));
	} else {
		memcpy(local_stages, stages, sizeof(local_stages));
		for (int i = 0; i < 6; i++) {
			local_stages[i].s1 = 0.0f;
			local_stages[i].s2 = 0.0f;
		}
	}

	for (int s = 0; s < 5; s++) {
		stages_out[s].g_s1[0] = 0.0f;
		stages_out[s].g_s2[0] = 0.0f;
		stages_out[s].g_s1[1] = 0.0f;
		stages_out[s].g_s2[1] = 0.0f;
	}

	float gA_c0 = stages_out[0].g_coeff[0], gA_c1 = stages_out[0].g_coeff[1];
	float gB_c0 = stages_out[1].g_coeff[0], gB_c1 = stages_out[1].g_coeff[1];
	float gC_c0 = stages_out[2].g_coeff[0], gC_c1 = stages_out[2].g_coeff[1];
	float gD_c0 = stages_out[3].g_coeff[0], gD_c1 = stages_out[3].g_coeff[1];
	float gE_c0 = stages_out[4].g_coeff[0], gE_c1 = stages_out[4].g_coeff[1];

	float gA_s1_0 = 0.0f, gA_s2_0 = 0.0f, gA_s1_1 = 0.0f, gA_s2_1 = 0.0f;
	float gB_s1_0 = 0.0f, gB_s2_0 = 0.0f, gB_s1_1 = 0.0f, gB_s2_1 = 0.0f;
	float gC_s1_0 = 0.0f, gC_s2_0 = 0.0f, gC_s1_1 = 0.0f, gC_s2_1 = 0.0f;
	float gD_s1_0 = 0.0f, gD_s2_0 = 0.0f, gD_s1_1 = 0.0f, gD_s2_1 = 0.0f;
	float gE_s1_0 = 0.0f, gE_s2_0 = 0.0f, gE_s1_1 = 0.0f, gE_s2_1 = 0.0f;

	for (uint32_t i = 0; i < count; i++) {
		float xA = (float)ref_in[i];

		/* Stage A Goertzel */
		float s0_a0 = xA + gA_c0 * gA_s1_0 - gA_s2_0; gA_s2_0 = gA_s1_0; gA_s1_0 = s0_a0;
		float s0_a1 = xA + gA_c1 * gA_s1_1 - gA_s2_1; gA_s2_1 = gA_s1_1; gA_s1_1 = s0_a1;

		/* Stage B (DC Blocker) */
		float xB = biquad_step(&local_stages[0], xA);
		float s0_b0 = xB + gB_c0 * gB_s1_0 - gB_s2_0; gB_s2_0 = gB_s1_0; gB_s1_0 = s0_b0;
		float s0_b1 = xB + gB_c1 * gB_s1_1 - gB_s2_1; gB_s2_1 = gB_s1_1; gB_s1_1 = s0_b1;

		/* Stage C (HPF + Notches) */
		float xC = biquad_step(&local_stages[1], xB);
		xC = biquad_step(&local_stages[2], xC);
		xC = biquad_step(&local_stages[3], xC);
		float s0_c0 = xC + gC_c0 * gC_s1_0 - gC_s2_0; gC_s2_0 = gC_s1_0; gC_s1_0 = s0_c0;
		float s0_c1 = xC + gC_c1 * gC_s1_1 - gC_s2_1; gC_s2_1 = gC_s1_1; gC_s1_1 = s0_c1;

		/* Stage D (LPF) */
		float xD = biquad_step(&local_stages[4], xC);
		xD = biquad_step(&local_stages[5], xD);
		float s0_d0 = xD + gD_c0 * gD_s1_0 - gD_s2_0; gD_s2_0 = gD_s1_0; gD_s1_0 = s0_d0;
		float s0_d1 = xD + gD_c1 * gD_s1_1 - gD_s2_1; gD_s2_1 = gD_s1_1; gD_s1_1 = s0_d1;

		/* Stage E (Gain + Clamp) */
		float xE = xD;
		if (digital_gain != 1.0f) xE *= digital_gain;
		if (xE > 32767.0f) xE = 32767.0f;
		else if (xE < -32768.0f) xE = -32768.0f;
		float s0_e0 = xE + gE_c0 * gE_s1_0 - gE_s2_0; gE_s2_0 = gE_s1_0; gE_s1_0 = s0_e0;
		float s0_e1 = xE + gE_c1 * gE_s1_1 - gE_s2_1; gE_s2_1 = gE_s1_1; gE_s1_1 = s0_e1;
	}

	stages_out[0].g_s1[0] = gA_s1_0; stages_out[0].g_s2[0] = gA_s2_0;
	stages_out[0].g_s1[1] = gA_s1_1; stages_out[0].g_s2[1] = gA_s2_1;

	stages_out[1].g_s1[0] = gB_s1_0; stages_out[1].g_s2[0] = gB_s2_0;
	stages_out[1].g_s1[1] = gB_s1_1; stages_out[1].g_s2[1] = gB_s2_1;

	stages_out[2].g_s1[0] = gC_s1_0; stages_out[2].g_s2[0] = gC_s2_0;
	stages_out[2].g_s1[1] = gC_s1_1; stages_out[2].g_s2[1] = gC_s2_1;

	stages_out[3].g_s1[0] = gD_s1_0; stages_out[3].g_s2[0] = gD_s2_0;
	stages_out[3].g_s1[1] = gD_s1_1; stages_out[3].g_s2[1] = gD_s2_1;

	stages_out[4].g_s1[0] = gE_s1_0; stages_out[4].g_s2[0] = gE_s2_0;
	stages_out[4].g_s1[1] = gE_s1_1; stages_out[4].g_s2[1] = gE_s2_1;

	for (int s = 0; s < 5; s++) {
		for (int k = 0; k < 2; k++) {
			float p = stages_out[s].g_s1[k] * stages_out[s].g_s1[k] +
			          stages_out[s].g_s2[k] * stages_out[s].g_s2[k] -
			          stages_out[s].g_coeff[k] * stages_out[s].g_s1[k] * stages_out[s].g_s2[k];
			if (p > 0.0f) {
				float n = (float)count;
				float peak = 2.0f * sqrtf(p) / n;
				stages_out[s].g_mag[k] = 20.0f * log10f(peak / 32768.0f + 1e-6f);
			} else {
				stages_out[s].g_mag[k] = -120.0f;
			}
		}
		stages_out[s].spectral_valid = true;
		stages_out[s].spectral_samples = count;
	}
}
#endif


