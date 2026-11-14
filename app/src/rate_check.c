/* SPDX-License-Identifier: Apache-2.0 */
#include "rate_check.h"
#include <stdio.h>
#include <string.h>

/*
 * Default calibrated STM32L476 values:
 * - DSP costs updated at boot by dsp_init() micro-benchmark once CMSIS-DSP runs.
 * - SD sustained wall rates (PSRAM peek + sequential FatFs write) calibrated per
 *   sample-rate tier (<= 48k, <= 96k, >= 192k), reflecting real bus contention.
 */
#define DEFAULT_DSP_NS_BYPASS  120U
#define DEFAULT_DSP_NS_DC_ONLY 300U
#define DEFAULT_DSP_NS_FULL    480U

#define DEFAULT_SD_KBPS_48K    1189U
#define DEFAULT_SD_KBPS_96K    900U
#define DEFAULT_SD_KBPS_HIGH   598U

static uint32_t s_dsp_ns_bypass  = DEFAULT_DSP_NS_BYPASS;
static uint32_t s_dsp_ns_dc_only = DEFAULT_DSP_NS_DC_ONLY;
static uint32_t s_dsp_ns_full    = DEFAULT_DSP_NS_FULL;

static uint32_t s_sd_kbps_48k  = DEFAULT_SD_KBPS_48K;
static uint32_t s_sd_kbps_96k  = DEFAULT_SD_KBPS_96K;
static uint32_t s_sd_kbps_high = DEFAULT_SD_KBPS_HIGH;

const char *rate_status_to_str(rate_status_t status)
{
	switch (status) {
	case RATE_STATUS_OK:         return "OK";
	case RATE_STATUS_WARNING:    return "WARNING";
	case RATE_STATUS_IMPOSSIBLE: return "IMPOSSIBLE";
	default:                     return "UNKNOWN";
	}
}

const char *rate_dsp_mode_to_str(rate_dsp_mode_t mode)
{
	switch (mode) {
	case RATE_DSP_BYPASS_ALL:    return "BYPASS";
	case RATE_DSP_DC_BLOCK_ONLY: return "DC_ONLY";
	case RATE_DSP_FULL_FILTERS:  return "FULL";
	case RATE_DSP_AUTO:          return "AUTO";
	default:                     return "UNKNOWN";
	}
}

void rate_check_set_measured_dsp_ns(uint32_t ns_bypass, uint32_t ns_dc_only, uint32_t ns_full)
{
	if (ns_bypass > 0U) {
		s_dsp_ns_bypass = ns_bypass;
	}
	if (ns_dc_only > 0U) {
		s_dsp_ns_dc_only = ns_dc_only;
	}
	if (ns_full > 0U) {
		s_dsp_ns_full = ns_full;
	}
}

void rate_check_get_measured_dsp_ns(uint32_t *ns_bypass, uint32_t *ns_dc_only, uint32_t *ns_full)
{
	if (ns_bypass) {
		*ns_bypass = s_dsp_ns_bypass;
	}
	if (ns_dc_only) {
		*ns_dc_only = s_dsp_ns_dc_only;
	}
	if (ns_full) {
		*ns_full = s_dsp_ns_full;
	}
}

void rate_check_set_measured_sd_kbps(uint32_t rate_hz, uint32_t wall_kbps)
{
	if (wall_kbps == 0U) {
		return;
	}
	if (rate_hz <= 48000U) {
		s_sd_kbps_48k = wall_kbps;
	} else if (rate_hz <= 96000U) {
		s_sd_kbps_96k = wall_kbps;
	} else {
		s_sd_kbps_high = wall_kbps;
	}
}

uint32_t rate_check_get_measured_sd_kbps(uint32_t rate_hz)
{
	if (rate_hz <= 48000U) {
		return s_sd_kbps_48k;
	} else if (rate_hz <= 96000U) {
		return s_sd_kbps_96k;
	} else {
		return s_sd_kbps_high;
	}
}

void rate_check_reset_calibration(void)
{
	s_dsp_ns_bypass  = DEFAULT_DSP_NS_BYPASS;
	s_dsp_ns_dc_only = DEFAULT_DSP_NS_DC_ONLY;
	s_dsp_ns_full    = DEFAULT_DSP_NS_FULL;
	s_sd_kbps_48k    = DEFAULT_SD_KBPS_48K;
	s_sd_kbps_96k    = DEFAULT_SD_KBPS_96K;
	s_sd_kbps_high   = DEFAULT_SD_KBPS_HIGH;
}

void rate_check_init_params(rate_check_params_t *p)
{
	if (!p) {
		return;
	}
	memset(p, 0, sizeof(*p));
	p->nominal_rate_hz       = 48000U;
	p->bits                  = 16U;
	p->channels              = 1U;
	p->ovs                   = 0U; /* auto */
	p->sample_cycles_x10     = RATE_CHECK_DEFAULT_SAMPLE_CYCLES_X10;
	p->duration_s            = 0U;
	p->dsp_cfg.mode          = RATE_DSP_AUTO;
	p->f_timer_hz            = RATE_CHECK_DEFAULT_TIMER_HZ;
	p->f_adc_hz              = RATE_CHECK_DEFAULT_ADC_HZ;
	p->dma_max_block_samples = RATE_CHECK_DEFAULT_DMA_MAX_SAMPLES;
	p->slab_block_bytes      = RATE_CHECK_DEFAULT_SLAB_BLOCK_BYTES;
	p->slab_num_blocks       = RATE_CHECK_DEFAULT_SLAB_NUM_BLOCKS;
	p->psram_batch_bytes     = RATE_CHECK_DEFAULT_PSRAM_BATCH_BYTES;
	p->sd_chunk_bytes        = RATE_CHECK_DEFAULT_SD_CHUNK_BYTES;
	p->wav_header_bytes      = RATE_CHECK_DEFAULT_WAV_HEADER_BYTES;
	p->psram_enabled         = true;
	p->psram_bytes           = RATE_CHECK_DEFAULT_PSRAM_BYTES;
	p->measured_sd_kbps      = 0U;
	p->min_pool_ms           = RATE_CHECK_DEFAULT_MIN_POOL_MS;
}

static bool is_valid_ovs_val(uint8_t ovs)
{
	return ovs == 0U || ovs == 1U || ovs == 2U || ovs == 4U || ovs == 8U || ovs == 16U;
}

static uint8_t ovs_enob(uint8_t ovs)
{
	switch (ovs) {
	case 1U:  return 12U;
	case 2U:  return 13U;
	case 4U:  return 14U;
	case 8U:  return 15U;
	case 16U: return 16U;
	default:  return 12U;
	}
}

static uint64_t calc_conv_ns(uint8_t ovs, uint16_t sample_cycles_x10, uint32_t f_adc_hz)
{
	/* Total ADC cycles (x10) = ovs * (sample_cycles + 12.5) + 4.0 sync cycles */
	uint64_t per_conv_x10 = (uint64_t)sample_cycles_x10 + 125ULL;
	uint64_t total_cycles_x10 = (uint64_t)ovs * per_conv_x10 + 40ULL;
	return (total_cycles_x10 * 100000000ULL) / (uint64_t)f_adc_hz;
}

static void impossible_with(rate_check_result_t *out, const char *reason)
{
	out->status = RATE_STATUS_IMPOSSIBLE;
	snprintf(out->reason, sizeof(out->reason), "%s", reason);
}

void rate_config_check_ex(const rate_check_params_t *in, rate_check_result_t *out)
{
	if (!out) {
		return;
	}
	memset(out, 0, sizeof(*out));
	if (!in) {
		impossible_with(out, "Null config parameters");
		return;
	}

	uint32_t f_timer_hz = in->f_timer_hz ? in->f_timer_hz : RATE_CHECK_DEFAULT_TIMER_HZ;
	uint32_t f_adc_hz   = in->f_adc_hz ? in->f_adc_hz : RATE_CHECK_DEFAULT_ADC_HZ;
	uint32_t dma_max    = in->dma_max_block_samples ? in->dma_max_block_samples : RATE_CHECK_DEFAULT_DMA_MAX_SAMPLES;
	uint32_t slab_blk   = in->slab_block_bytes ? in->slab_block_bytes : RATE_CHECK_DEFAULT_SLAB_BLOCK_BYTES;
	uint32_t slab_nblk  = in->slab_num_blocks ? in->slab_num_blocks : RATE_CHECK_DEFAULT_SLAB_NUM_BLOCKS;
	uint32_t psram_bat  = in->psram_batch_bytes ? in->psram_batch_bytes : RATE_CHECK_DEFAULT_PSRAM_BATCH_BYTES;
	uint32_t sd_chunk   = in->sd_chunk_bytes ? in->sd_chunk_bytes : RATE_CHECK_DEFAULT_SD_CHUNK_BYTES;
	uint32_t wav_hdr    = in->wav_header_bytes ? in->wav_header_bytes : RATE_CHECK_DEFAULT_WAV_HEADER_BYTES;
	uint32_t min_pool_ms = in->min_pool_ms ? in->min_pool_ms : RATE_CHECK_DEFAULT_MIN_POOL_MS;
	uint16_t samp_x10   = in->sample_cycles_x10 ? in->sample_cycles_x10 : RATE_CHECK_DEFAULT_SAMPLE_CYCLES_X10;
	uint16_t channels   = in->channels ? in->channels : 1U;
	uint64_t psram_bytes = in->psram_enabled ? in->psram_bytes : 0ULL;

	rate_check_margins_t *m = &out->margins;
	m->experimental = (in->nominal_rate_hz > 192000U) || (in->bits == 32U) || (channels > 1U);

	/* 1. Hard IMPOSSIBLE parameter & code-path bounds */
	if (in->nominal_rate_hz == 0U || in->nominal_rate_hz > f_timer_hz) {
		char buf[192];
		snprintf(buf, sizeof(buf), "Rate %u Hz out of hardware timer range [1..%u Hz]",
			 in->nominal_rate_hz, f_timer_hz);
		impossible_with(out, buf);
		return;
	}
	if (in->bits != 16U && in->bits != 32U) {
		char buf[192];
		snprintf(buf, sizeof(buf), "Bit depth %u-bit unsupported (implemented: 16, 32)",
			 in->bits);
		impossible_with(out, buf);
		return;
	}
	if (channels != 1U && channels != 2U) {
		char buf[192];
		snprintf(buf, sizeof(buf), "Channel count %u unsupported (implemented: 1, 2)", channels);
		impossible_with(out, buf);
		return;
	}
	if (!is_valid_ovs_val(in->ovs)) {
		char buf[192];
		snprintf(buf, sizeof(buf), "Oversampling %ux invalid (supported: auto, 1, 2, 4, 8, 16)",
			 in->ovs);
		impossible_with(out, buf);
		return;
	}
	if (samp_x10 < 25U || samp_x10 > 6405U) {
		impossible_with(out, "ADC sample time out of valid range");
		return;
	}

	/* 2. Timer divider & actual sample rate (64-bit) */
	uint64_t div64 = ((uint64_t)f_timer_hz + ((uint64_t)in->nominal_rate_hz / 2ULL)) /
			 (uint64_t)in->nominal_rate_hz;
	if (div64 == 0ULL || div64 > 65536ULL) {
		char buf[192];
		snprintf(buf, sizeof(buf), "TIM6 divider %llu out of 16-bit range [1..65536]",
			 (unsigned long long)div64);
		impossible_with(out, buf);
		return;
	}
	uint64_t actual64 = (uint64_t)f_timer_hz / div64;
	if (actual64 == 0ULL) {
		impossible_with(out, "Computed actual rate is zero");
		return;
	}
	out->actual_rate = (uint32_t)actual64;
	m->tim_divider   = (uint32_t)div64;
	m->rate_err_hz   = (int32_t)((int64_t)actual64 - (int64_t)in->nominal_rate_hz);
	m->rate_err_ppm  = (int32_t)(((int64_t)m->rate_err_hz * 1000000LL) /
				     (int64_t)in->nominal_rate_hz);
	if (m->rate_err_ppm > 1000 || m->rate_err_ppm < -1000) {
		m->warn_rate_err = true;
	}

	/* 3. Memory & block geometry (strictly derived from bytes_per_frame) */
	m->bytes_per_frame = (uint32_t)channels * ((uint32_t)in->bits / 8U);
	if (m->bytes_per_frame == 0U || slab_blk == 0U || slab_nblk == 0U ||
	    (slab_blk % m->bytes_per_frame) != 0U) {
		impossible_with(out, "Slab block size incompatible with bytes_per_frame");
		return;
	}
	m->block_samples = slab_blk / m->bytes_per_frame;
	if (m->block_samples == 0U || m->block_samples > dma_max) {
		char buf[192];
		snprintf(buf, sizeof(buf), "Block samples %u cannot fit DMA buffer (max %u)",
			 m->block_samples, dma_max);
		impossible_with(out, buf);
		return;
	}
	m->block_bytes = m->block_samples * m->bytes_per_frame;
	if (m->block_bytes > slab_blk ||
	    (psram_bat % m->block_bytes) != 0U ||
	    (sd_chunk % m->block_bytes) != 0U ||
	    (wav_hdr % 512U) != 0U) {
		impossible_with(out, "Buffer/PSRAM/SD/WAV geometry cannot fit alignment");
		return;
	}

	m->bytes_per_sec = actual64 * (uint64_t)m->bytes_per_frame;
	m->stream_kbps   = (uint32_t)((m->bytes_per_sec + 512ULL) / 1024ULL);

	uint64_t sram_bytes = (uint64_t)slab_blk * (uint64_t)slab_nblk;
	uint64_t total_buf_bytes = sram_bytes + psram_bytes;
	uint64_t pool_ms64 = (total_buf_bytes * 1000ULL) / m->bytes_per_sec;
	m->pool_ms = (pool_ms64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)pool_ms64;
	if (m->pool_ms < min_pool_ms) {
		m->warn_small_pool = true;
	}

	/* 4. ADC oversampling budget: <=85% optimal, 85..98% WARNING, >98% IMPOSSIBLE */
	uint64_t period_ns64 = 1000000000ULL / actual64;
	m->sample_period_ns = (uint32_t)period_ns64;

	uint8_t sel_ovs = 0U;
	uint64_t sel_conv_ns = 0ULL;
	if (in->ovs != 0U) {
		uint64_t c_ns = calc_conv_ns(in->ovs, samp_x10, f_adc_hz);
		if (c_ns * 100ULL > period_ns64 * (uint64_t)RATE_CHECK_ADC_HARD_PCT_MAX) {
			uint32_t pct = (uint32_t)((c_ns * 100ULL) / (period_ns64 ? period_ns64 : 1ULL));
			char buf[192];
			snprintf(buf, sizeof(buf),
				 "ADC %ux conv %u ns exceeds %u ns period (%u%%)",
				 in->ovs, (uint32_t)c_ns, (uint32_t)period_ns64, pct);
			impossible_with(out, buf);
			return;
		}
		sel_ovs = in->ovs;
		sel_conv_ns = c_ns;
	} else {
		/* Auto-select highest oversampling ratio in {8, 4, 2, 1} fitting <= 85%.
		 * Capping auto-select at 8x keeps <=192 kHz 16-bit captures bit-identical.
		 */
		static const uint8_t candidates[4] = { 8U, 4U, 2U, 1U };
		for (int i = 0; i < 4; i++) {
			uint8_t c = candidates[i];
			uint64_t c_ns = calc_conv_ns(c, samp_x10, f_adc_hz);
			if (c_ns * 100ULL <= period_ns64 * (uint64_t)RATE_CHECK_ADC_BUDGET_PCT_MAX) {
				sel_ovs = c;
				sel_conv_ns = c_ns;
				break;
			}
		}
		/* Fall back to 1x up to the hard 98% conversion-period limit (WARNING). */
		if (sel_ovs == 0U) {
			uint64_t c_ns = calc_conv_ns(1U, samp_x10, f_adc_hz);
			if (c_ns * 100ULL <= period_ns64 * (uint64_t)RATE_CHECK_ADC_HARD_PCT_MAX) {
				sel_ovs = 1U;
				sel_conv_ns = c_ns;
			} else {
				char buf[192];
				snprintf(buf, sizeof(buf),
					 "1x ADC conv %u ns exceeds %u ns sample period",
					 (uint32_t)c_ns, (uint32_t)period_ns64);
				impossible_with(out, buf);
				return;
			}
		}
	}
	m->ovs_selected       = sel_ovs;
	m->adc_effective_bits = ovs_enob(sel_ovs);
	m->adc_conv_ns        = (uint32_t)sel_conv_ns;
	m->adc_budget_pct     = period_ns64 ? (uint32_t)((sel_conv_ns * 100ULL) / period_ns64) : 100U;
	if (m->adc_budget_pct > RATE_CHECK_ADC_BUDGET_PCT_MAX) {
		m->warn_adc_duty = true;
	}
	if (in->nominal_rate_hz > 192000U && sel_ovs < 4U) {
		m->warn_ovs_reduced = true;
	}

	/* 5. DSP CPU & block deadline (above threshold -> WARNING, never IMPOSSIBLE) */
	rate_dsp_mode_t eff_dsp = in->dsp_cfg.mode;
	if (eff_dsp == RATE_DSP_AUTO) {
		eff_dsp = (in->nominal_rate_hz > 192000U)
			  ? RATE_DSP_DC_BLOCK_ONLY
			  : RATE_DSP_FULL_FILTERS;
	}
	uint32_t dsp_ns = 0U;
	switch (eff_dsp) {
	case RATE_DSP_BYPASS_ALL:
		dsp_ns = in->dsp_cfg.ns_per_sample_bypass
			 ? in->dsp_cfg.ns_per_sample_bypass : s_dsp_ns_bypass;
		break;
	case RATE_DSP_DC_BLOCK_ONLY:
		dsp_ns = in->dsp_cfg.ns_per_sample_dc_only
			 ? in->dsp_cfg.ns_per_sample_dc_only : s_dsp_ns_dc_only;
		break;
	case RATE_DSP_FULL_FILTERS:
	default:
		dsp_ns = in->dsp_cfg.ns_per_sample_full
			 ? in->dsp_cfg.ns_per_sample_full : s_dsp_ns_full;
		break;
	}
	m->effective_dsp_mode = eff_dsp;
	m->dsp_ns_per_sample  = dsp_ns;

	uint64_t dsp_ns_per_s = actual64 * (uint64_t)dsp_ns;
	m->dsp_cpu_pct        = (uint32_t)((dsp_ns_per_s + 5000000ULL) / 10000000ULL);
	m->block_deadline_us  = (uint32_t)(((uint64_t)m->block_samples * 1000000ULL) / actual64);
	m->dsp_block_us       = (uint32_t)((((uint64_t)m->block_samples * (uint64_t)dsp_ns * 12ULL) /
					    10ULL + 999ULL) / 1000ULL);
	if (m->dsp_cpu_pct > RATE_CHECK_DSP_CPU_PCT_MAX ||
	    m->dsp_block_us >= m->block_deadline_us ||
	    m->block_deadline_us < 2000U) {
		m->warn_dsp = true;
	}

	/* 6. 64-bit count/byte overflow & 4 GiB RIFF WAV container limit (IMPOSSIBLE) */
	uint64_t max_wav_pcm = (uint64_t)UINT32_MAX - (uint64_t)wav_hdr;
	uint64_t max_wav_dur64 = max_wav_pcm / m->bytes_per_sec;
	m->max_wav_duration_s = (max_wav_dur64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)max_wav_dur64;

	if (in->duration_s > 0U) {
		if ((uint64_t)in->duration_s > (UINT64_MAX / actual64)) {
			impossible_with(out, "Total sample count overflows uint64");
			return;
		}
		uint64_t tot_samples = actual64 * (uint64_t)in->duration_s;
		if (tot_samples > ((UINT64_MAX - (uint64_t)wav_hdr) / (uint64_t)m->bytes_per_frame)) {
			impossible_with(out, "Total byte count overflows uint64");
			return;
		}
		uint64_t tot_pcm = tot_samples * (uint64_t)m->bytes_per_frame;
		uint64_t tot_wav = (uint64_t)wav_hdr + tot_pcm;
		m->total_samples   = tot_samples;
		m->total_wav_bytes = tot_wav;

		if (tot_wav > (uint64_t)UINT32_MAX) {
			char buf[192];
			snprintf(buf, sizeof(buf),
				 "exceeds 4 GiB WAV limit (max %u s)",
				 m->max_wav_duration_s);
			impossible_with(out, buf);
			return;
		}
	}

	/* 7. Storage sustained bandwidth & PSRAM deficit budgeting (WARNING, never blocks) */
	uint32_t sd_kbps = in->measured_sd_kbps
			   ? in->measured_sd_kbps
			   : rate_check_get_measured_sd_kbps(in->nominal_rate_hz);
	uint64_t sd_bps = (uint64_t)sd_kbps * 1024ULL;
	m->sd_sustained_kbps = sd_kbps;

	if (sd_bps == 0ULL) {
		impossible_with(out, "Measured SD throughput is 0 KB/s");
		return;
	}

	if (m->bytes_per_sec <= sd_bps) {
		m->sd_margin_pct  = (int32_t)(((int64_t)(sd_bps - m->bytes_per_sec) * 100LL) /
					      (int64_t)sd_bps);
		m->max_duration_s = m->max_wav_duration_s;
	} else {
		m->warn_storage = true;
		m->sd_margin_pct = -(int32_t)(((int64_t)(m->bytes_per_sec - sd_bps) * 100LL) /
					      (int64_t)sd_bps);
		if (!in->psram_enabled || psram_bytes == 0ULL) {
			m->max_duration_s = 0U;
		} else {
			uint64_t deficit_bps = m->bytes_per_sec - sd_bps;
			uint64_t psram_max_dur64 = psram_bytes / deficit_bps;
			if (psram_max_dur64 > (uint64_t)m->max_wav_duration_s) {
				psram_max_dur64 = m->max_wav_duration_s;
			}
			m->max_duration_s = (uint32_t)psram_max_dur64;
		}
	}

	/* 8. Final verdict: OK vs WARNING */
	bool any_warn = m->experimental || m->warn_storage || m->warn_dsp ||
			m->warn_adc_duty || m->warn_small_pool || m->warn_rate_err ||
			m->warn_ovs_reduced;
	if (!any_warn) {
		out->status    = RATE_STATUS_OK;
		out->reason[0] = '\0';
		return;
	}

	out->status = RATE_STATUS_WARNING;
	if (m->warn_storage) {
		if (!in->psram_enabled || psram_bytes == 0ULL) {
			snprintf(out->reason, sizeof(out->reason),
				 "will run; SD %u KB/s < stream %u KB/s (PSRAM off), samples may be lost",
				 sd_kbps, m->stream_kbps);
		} else if (m->max_duration_s == 0U) {
			snprintf(out->reason, sizeof(out->reason),
				 "will run; immediate overload (stream %u KB/s > SD %u KB/s), samples may be lost",
				 m->stream_kbps, sd_kbps);
		} else if (in->duration_s == 0U || in->duration_s > m->max_duration_s) {
			snprintf(out->reason, sizeof(out->reason),
				 "will run; expected overload after about %u s (PSRAM full), samples may be lost",
				 m->max_duration_s);
		} else {
			snprintf(out->reason, sizeof(out->reason),
				 "will run; PSRAM buffered (overload after about %u s)",
				 m->max_duration_s);
		}
	} else if (m->warn_dsp) {
		snprintf(out->reason, sizeof(out->reason),
			 "will run; DSP load %u%% CPU (%u/%u us block)",
			 m->dsp_cpu_pct, m->dsp_block_us, m->block_deadline_us);
	} else if (m->warn_adc_duty) {
		snprintf(out->reason, sizeof(out->reason),
			 "will run; ADC duty %u%% > 85%% guard (%ux OVS)",
			 m->adc_budget_pct, m->ovs_selected);
	} else if (m->warn_small_pool) {
		snprintf(out->reason, sizeof(out->reason),
			 "will run; small buffer pool (%u ms)",
			 m->pool_ms);
	} else if (m->warn_rate_err || m->warn_ovs_reduced) {
		snprintf(out->reason, sizeof(out->reason),
			 "will run; actual %u Hz (%d ppm, %ux OVS / %u-bit ENOB)",
			 out->actual_rate, m->rate_err_ppm, m->ovs_selected, m->adc_effective_bits);
	} else {
		snprintf(out->reason, sizeof(out->reason),
			 "will run; experimental mode (%u Hz, %u-bit, %uch)",
			 in->nominal_rate_hz, in->bits, channels);
	}
}

rate_check_result_t rate_config_check(uint32_t rate_hz,
				      uint16_t bits,
				      uint8_t ovs,
				      uint16_t sample_cycles_x10,
				      uint32_t duration_s,
				      const rate_dsp_cfg_t *dsp_cfg)
{
	rate_check_params_t p;
	rate_check_result_t r;

	rate_check_init_params(&p);
	p.nominal_rate_hz   = rate_hz;
	p.bits              = bits;
	p.ovs               = ovs;
	p.sample_cycles_x10 = sample_cycles_x10 ? sample_cycles_x10 : RATE_CHECK_DEFAULT_SAMPLE_CYCLES_X10;
	p.duration_s        = duration_s;
	if (dsp_cfg) {
		p.dsp_cfg = *dsp_cfg;
	}
	rate_config_check_ex(&p, &r);
	return r;
}
