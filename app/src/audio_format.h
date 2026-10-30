/* SPDX-License-Identifier: Apache-2.0 */
#ifndef AUDIO_FORMAT_H_
#define AUDIO_FORMAT_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Canonical Audio Sample Rates (samples per second per channel).
 */
#define AUDIO_RATE_48K   48000U
#define AUDIO_RATE_96K   96000U
#define AUDIO_RATE_192K 192000U
#define AUDIO_RATE_384K 384000U
#define AUDIO_RATE_512K 512000U

/**
 * @brief Physical hardware and file-format support tiers.
 */
enum audio_support_tier {
	AUDIO_TIER_INVALID = 0,             /* Mathematically impossible or illegal parameters */
	AUDIO_TIER_REPRESENTABLE_ONLY = 1,  /* Valid WAV container, but not feasible on current board hardware */
	AUDIO_TIER_FUTURE_HARDWARE = 2,     /* Planned for future platform (STM32U585, external ADC / I2S) */
	AUDIO_TIER_CURRENT_HARDWARE = 3,    /* Fully supported and executable on current STM32L476RG + internal ADC */
};

/**
 * @brief PCM Stored Format Encoding.
 */
enum audio_pcm_encoding {
	AUDIO_ENCODING_PCM_S16_LE = 1,      /* 16-bit signed integer little-endian (WAV Format Tag 1) */
	AUDIO_ENCODING_PCM_S32_LE = 2,      /* 32-bit signed integer little-endian (WAV Format Tag 1) */
};

/**
 * @brief Convert 16-bit signed PCM sample to 32-bit signed PCM container.
 *
 * Elimination of Undefined Behavior:
 * Shifting a signed negative integer left is undefined behavior in ISO C99/C11.
 * This implementation casts to uint32_t first to perform well-defined unsigned bitwise shift,
 * then converts to int32_t, perfectly placing the 16 audio bits into MSBs [31:16]
 * and zeroing LSBs [15:0].
 *
 * Range mapping:
 *   -32768 (0x8000) -> -2147483648 (0x80000000)
 *        0 (0x0000) ->           0 (0x00000000)
 *   +32767 (0x7FFF) -> +2147418112 (0x7FFF0000)
 *
 * Note: A 32-bit container holding converted 12-bit ADC data provides standard 32-bit
 * container compatibility for DAW/analysis software, but DOES NOT increase physical
 * ADC resolution or SNR beyond the hardware limits.
 */
static inline int32_t audio_pcm16_to_pcm32(int16_t s16)
{
	return (int32_t)((uint32_t)(int32_t)s16 << 16);
}

/**
 * @brief Convert 32-bit signed PCM container back to 16-bit signed PCM.
 */
static inline int16_t audio_pcm32_to_pcm16(int32_t s32)
{
	return (int16_t)((int32_t)s32 >> 16);
}

/**
 * @brief Convert native unsigned ADC code (12-bit native or oversampled) to signed 16-bit PCM.
 *
 * @param raw Unsigned ADC conversion result (0 to max_raw).
 * @param lshift Left-shift required to align ADC bits to 16-bit range (16 - ADC_BITS).
 * @return Signed 16-bit sample clamped to [-32768, 32767].
 */
static inline int16_t audio_adc_raw_to_pcm16(uint32_t raw, uint32_t lshift)
{
	int32_t x = ((int32_t)raw << lshift) - 32768;
	if (x > 32767) {
		x = 32767;
	} else if (x < -32768) {
		x = -32768;
	}
	return (int16_t)x;
}

/**
 * @brief Convert native unsigned ADC code directly to 32-bit signed PCM container.
 */
static inline int32_t audio_adc_raw_to_pcm32(uint32_t raw, uint32_t lshift)
{
	int16_t s16 = audio_adc_raw_to_pcm16(raw, lshift);
	return audio_pcm16_to_pcm32(s16);
}

/**
 * @brief Hardware-independent audio configuration model.
 */
struct audio_config_model {
	/* Core format parameters */
	uint32_t sample_rate_hz;            /* Samples per second per channel (e.g. 48000, 192000, 512000) */
	uint16_t channel_count;             /* 1 = Mono, 2 = Stereo */
	uint16_t sample_bits;               /* Container width in bits: 16 or 32 */
	uint16_t bytes_per_sample;          /* sample_bits / 8 */
	uint16_t bytes_per_frame;           /* channel_count * bytes_per_sample */
	enum audio_pcm_encoding encoding;   /* PCM encoding standard */

	/* Physical acquisition parameters */
	uint16_t adc_native_resolution;     /* Hardware ADC resolution (12-bit on L4, 14-bit on U5) */
	uint16_t effective_enob_bits;       /* Effective bits with oversampling (e.g. 15-bit on L4 with 8x oversampling) */

	/* Streaming and buffer sizing */
	uint32_t bytes_per_second;          /* Decimal B/s: sample_rate_hz * bytes_per_frame */
	uint32_t block_samples_per_ch;      /* Samples per channel per processing block (e.g. 4096) */
	uint32_t block_bytes;               /* Bytes per block across all channels */
	uint32_t block_duration_us;         /* Duration of one block in microseconds */

	/* Spectral and DSP filter constraints */
	uint32_t nyquist_freq_hz;           /* sample_rate_hz / 2 */
	uint32_t dsp_max_cutoff_hz;         /* Maximum valid low-pass cutoff (must be strictly < nyquist) */
	uint32_t analog_front_end_bw_hz;    /* Usable acoustic bandwidth of microphone (e.g. 20000 Hz for MAX4466) */

	/* Platform portability assessment */
	enum audio_support_tier tier_l4;    /* Support status on current STM32L476RG board */
	enum audio_support_tier tier_u5;    /* Support status on target STM32U585 board */
	const char *support_notes;          /* Human-readable feasibility assessment */
};

/**
 * @brief Initialize and validate an audio configuration model.
 *
 * @param model Output configuration model struct.
 * @param sample_rate_hz Desired sampling rate (e.g. 48000, 96000, 192000, 384000, 512000).
 * @param channel_count 1 (Mono) or 2 (Stereo).
 * @param sample_bits 16 or 32.
 * @param block_samples Samples per channel per block (e.g. 4096).
 * @return 0 on success, negative errno on invalid parameter combination.
 */
int audio_format_init(struct audio_config_model *model,
                      uint32_t sample_rate_hz,
                      uint16_t channel_count,
                      uint16_t sample_bits,
                      uint32_t block_samples);

/**
 * @brief Check if a configuration model represents a valid file format.
 */
bool audio_format_is_representable(const struct audio_config_model *model);

/**
 * @brief Check if a configuration model is supported by the current STM32L476RG board.
 */
bool audio_format_is_supported_current_hw(const struct audio_config_model *model);

/**
 * @brief Check if a configuration model is supported by the target STM32U585 board.
 */
bool audio_format_is_supported_u5(const struct audio_config_model *model);

/**
 * @brief Calculate exact WAV accounting (samples, raw PCM payload bytes, and total file size).
 *
 * Safe against 32-bit arithmetic overflow using 64-bit calculations.
 *
 * @param model Configured audio model.
 * @param duration_sec Recording duration in seconds.
 * @param out_samples Total samples across all channels.
 * @param out_payload_bytes Total PCM audio payload bytes.
 * @param out_total_wav_bytes Total WAV file bytes including 512-byte aligned header.
 * @return 0 on success, -EINVAL on invalid parameter or duration.
 */
int audio_format_calc_wav_accounting(const struct audio_config_model *model,
                                     uint32_t duration_sec,
                                     uint64_t *out_samples,
                                     uint64_t *out_payload_bytes,
                                     uint64_t *out_total_wav_bytes);

/**
 * @brief Validate whether a requested DSP filter cutoff is valid for this configuration.
 *
 * Enforces that:
 * 1. Cutoff is strictly below the Nyquist frequency (sample_rate / 2).
 * 2. Cutoff does not exceed physical analog microphone limits without ultrasonic warning.
 *
 * @param model Configured audio model.
 * @param cutoff_hz Requested filter cutoff in Hz.
 * @param is_lpf True if low-pass filter, false if high-pass filter.
 * @return 0 if valid, -ERANGE if exceeds Nyquist, -EINVAL on illegal parameter.
 */
int audio_format_validate_filter_cutoff(const struct audio_config_model *model,
                                        uint32_t cutoff_hz,
                                        bool is_lpf);

/**
 * @brief Determine if an audio configuration is an experimental test-only mode.
 *
 * An experimental mode is defined as any configuration with:
 * - Sampling rate > 192000 Hz (e.g. 384 kHz, 512 kHz)
 * - Channel count > 1 (e.g. Stereo emulation)
 * - Container width > 16 bits (e.g. 32-bit PCM)
 *
 * @param rate_hz Effective or requested sampling rate in Hz.
 * @param channels Number of audio channels.
 * @param bits Stored bits per sample container width.
 * @return true if experimental test-only configuration, false if production-standard.
 */
bool audio_format_is_experimental(uint32_t rate_hz, uint16_t channels, uint16_t bits);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_FORMAT_H_ */

