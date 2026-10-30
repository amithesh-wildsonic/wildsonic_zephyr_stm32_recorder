/* SPDX-License-Identifier: Apache-2.0 */
#include "audio_format.h"
#include <errno.h>
#include <string.h>

int audio_format_init(struct audio_config_model *model,
                      uint32_t sample_rate_hz,
                      uint16_t channel_count,
                      uint16_t sample_bits,
                      uint32_t block_samples)
{
	if (!model) {
		return -EINVAL;
	}

	memset(model, 0, sizeof(*model));

	/* Validate discrete sample rates */
	if (sample_rate_hz != 16000 && sample_rate_hz != 32000 && sample_rate_hz != 44100 &&
	    sample_rate_hz != 48000 && sample_rate_hz != 96000 && sample_rate_hz != 192000 &&
	    sample_rate_hz != 384000 && sample_rate_hz != 512000) {
		return -EINVAL;
	}

	/* Validate channel count: 1 (Mono) or 2 (Stereo) */
	if (channel_count != 1 && channel_count != 2) {
		return -EINVAL;
	}

	/* Validate container bit width: 16-bit or 32-bit */
	if (sample_bits != 16 && sample_bits != 32) {
		return -EINVAL;
	}

	/* Default block size if zero */
	if (block_samples == 0) {
		block_samples = 4096U;
	}

	model->sample_rate_hz = sample_rate_hz;
	model->channel_count = channel_count;
	model->sample_bits = sample_bits;
	model->bytes_per_sample = sample_bits / 8U;
	model->bytes_per_frame = channel_count * model->bytes_per_sample;
	model->encoding = (sample_bits == 32) ? AUDIO_ENCODING_PCM_S32_LE : AUDIO_ENCODING_PCM_S16_LE;

	/* Physical resolution baseline */
	model->adc_native_resolution = 12U; /* STM32L4 native SAR 12-bit */
	model->effective_enob_bits = (sample_bits == 32) ? 15U : 12U;

	/* Stream bandwidth and block sizing with 64-bit overflow prevention */
	uint64_t rate_bps = (uint64_t)sample_rate_hz * model->bytes_per_frame;
	if (rate_bps > UINT32_MAX) {
		return -EOVERFLOW;
	}
	model->bytes_per_second = (uint32_t)rate_bps;

	model->block_samples_per_ch = block_samples;
	uint64_t blk_bytes = (uint64_t)block_samples * model->bytes_per_frame;
	if (blk_bytes > UINT32_MAX) {
		return -EOVERFLOW;
	}
	model->block_bytes = (uint32_t)blk_bytes;

	model->block_duration_us = (uint32_t)(((uint64_t)block_samples * 1000000ULL) / sample_rate_hz);

	/* Spectral and filter limits */
	model->nyquist_freq_hz = sample_rate_hz / 2U;
	model->dsp_max_cutoff_hz = (model->nyquist_freq_hz * 90U) / 100U; /* 90% Nyquist margin */
	model->analog_front_end_bw_hz = 20000U; /* MAX4466 microphone capsule limit */

	/* Assess STM32L476RG Hardware Tier */
	if (channel_count > 1) {
		model->tier_l4 = AUDIO_TIER_FUTURE_HARDWARE;
		model->support_notes = "Stereo unsupported on STM32L4 board: single ADC input (PA0/ADC1_IN5) wired to mono MAX4466; requires dual ADC or external codec.";
	} else if (sample_rate_hz > 192000U) {
		model->tier_l4 = AUDIO_TIER_REPRESENTABLE_ONLY;
		model->support_notes = "Rate exceeds STM32L4 oversampling budget (8x oversampling requires >85% of period) and has fractional TIM6 reload divider at 72 MHz.";
	} else {
		model->tier_l4 = AUDIO_TIER_CURRENT_HARDWARE;
		model->support_notes = "Fully supported on current STM32L476RG + internal ADC1 + TIM6 TRGO + DMA.";
	}

	/* Assess STM32U585 Hardware Tier */
	/* U585 has 160 MHz clock, 2.5 Msps 14-bit ADC, dual ADCs for simultaneous stereo, and MDF/ADF digital filters */
	model->tier_u5 = AUDIO_TIER_CURRENT_HARDWARE;

	return 0;
}

bool audio_format_is_representable(const struct audio_config_model *model)
{
	if (!model || model->sample_rate_hz == 0 || model->channel_count == 0 || model->bytes_per_frame == 0) {
		return false;
	}
	return (model->tier_l4 != AUDIO_TIER_INVALID);
}

bool audio_format_is_supported_current_hw(const struct audio_config_model *model)
{
	if (!model) return false;
	return (model->tier_l4 == AUDIO_TIER_CURRENT_HARDWARE);
}

bool audio_format_is_supported_u5(const struct audio_config_model *model)
{
	if (!model) return false;
	return (model->tier_u5 == AUDIO_TIER_CURRENT_HARDWARE);
}

int audio_format_calc_wav_accounting(const struct audio_config_model *model,
                                     uint32_t duration_sec,
                                     uint64_t *out_samples,
                                     uint64_t *out_payload_bytes,
                                     uint64_t *out_total_wav_bytes)
{
	if (!model || duration_sec == 0) {
		return -EINVAL;
	}

	uint64_t total_frames = (uint64_t)duration_sec * model->sample_rate_hz;
	uint64_t total_samples = total_frames * model->channel_count;
	uint64_t payload_bytes = total_frames * model->bytes_per_frame;
	uint64_t total_wav_bytes = 512ULL + payload_bytes; /* 512-byte sector aligned WAV header */

	/* Check 4GB limit for standard 32-bit RIFF/WAV header */
	if (total_wav_bytes > 0xFFFFFFFFULL) {
		return -EFBIG;
	}

	if (out_samples) *out_samples = total_samples;
	if (out_payload_bytes) *out_payload_bytes = payload_bytes;
	if (out_total_wav_bytes) *out_total_wav_bytes = total_wav_bytes;

	return 0;
}

int audio_format_validate_filter_cutoff(const struct audio_config_model *model,
                                        uint32_t cutoff_hz,
                                        bool is_lpf)
{
	if (!model || cutoff_hz == 0) {
		return -EINVAL;
	}

	if (cutoff_hz >= model->nyquist_freq_hz) {
		return -ERANGE; /* Exceeds Nyquist limit */
	}

	if (is_lpf && cutoff_hz > model->dsp_max_cutoff_hz) {
		return -ERANGE; /* Exceeds safe 90% Nyquist margin for biquad filter stability */
	}

	return 0;
}

bool audio_format_is_experimental(uint32_t rate_hz, uint16_t channels, uint16_t bits)
{
	return (rate_hz > 192000U || channels > 1U || bits > 16U);
}


