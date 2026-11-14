/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RATE_CHECK_H_
#define RATE_CHECK_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	RATE_STATUS_OK         = 0,
	RATE_STATUS_WARNING    = 1,
	RATE_STATUS_IMPOSSIBLE = 2,
	/* Backwards-compatible aliases */
	RATE_STATUS_SUPPORTED  = RATE_STATUS_OK,
	RATE_STATUS_LIMITED    = RATE_STATUS_WARNING,
	RATE_STATUS_REJECTED   = RATE_STATUS_IMPOSSIBLE,
} rate_status_t;

typedef enum {
	RATE_DSP_BYPASS_ALL    = 0,
	RATE_DSP_DC_BLOCK_ONLY = 1,
	RATE_DSP_FULL_FILTERS  = 2,
	RATE_DSP_AUTO          = 3, /* Full filters <= 192 kHz, DC block only > 192 kHz */
} rate_dsp_mode_t;

typedef struct {
	rate_dsp_mode_t mode;
	uint32_t hpf_hz;
	uint32_t lpf_hz;
	bool notch_enabled;
	uint32_t ns_per_sample_bypass;  /* 0 = use calibrated platform default */
	uint32_t ns_per_sample_dc_only; /* 0 = use calibrated platform default */
	uint32_t ns_per_sample_full;    /* 0 = use calibrated platform default */
} rate_dsp_cfg_t;

typedef struct {
	uint32_t nominal_rate_hz;
	uint16_t bits;
	uint16_t channels;
	uint8_t ovs;                /* 0 = auto-select highest fitting <= 85%; else 1,2,4,8,16 */
	uint16_t sample_cycles_x10; /* e.g. 245 for 24.5 ADC sample cycles */
	uint32_t duration_s;        /* 0 = continuous */
	rate_dsp_cfg_t dsp_cfg;

	/* Platform hardware & buffer geometry (0 = STM32L476 defaults) */
	uint32_t f_timer_hz;
	uint32_t f_adc_hz;
	uint32_t dma_max_block_samples;
	uint32_t slab_block_bytes;
	uint32_t slab_num_blocks;
	uint32_t psram_batch_bytes;
	uint32_t sd_chunk_bytes;
	uint32_t wav_header_bytes;
	bool psram_enabled;
	uint64_t psram_bytes;
	uint32_t measured_sd_kbps; /* 0 = use per-rate calibrated table */
	uint32_t min_pool_ms;      /* 0 = default 300 ms */
} rate_check_params_t;

typedef struct {
	uint32_t tim_divider;
	int32_t rate_err_hz;
	int32_t rate_err_ppm;
	uint8_t ovs_selected;
	uint8_t adc_effective_bits;
	uint32_t adc_conv_ns;
	uint32_t sample_period_ns;
	uint32_t adc_budget_pct;

	rate_dsp_mode_t effective_dsp_mode;
	uint32_t dsp_ns_per_sample;
	uint32_t dsp_cpu_pct;
	uint32_t dsp_block_us;
	uint32_t block_deadline_us;

	uint32_t bytes_per_frame;
	uint32_t block_samples;
	uint32_t block_bytes;
	uint64_t bytes_per_sec;
	uint32_t stream_kbps;
	uint32_t sd_sustained_kbps;
	int32_t sd_margin_pct;
	uint32_t pool_ms;
	uint32_t max_duration_s;      /* PSRAM-limited duration (0 if sustainable) */
	uint32_t max_wav_duration_s;  /* Max before 4 GiB RIFF WAV limit */
	uint64_t total_samples;
	uint64_t total_wav_bytes;
	bool experimental;
	bool warn_storage;
	bool warn_dsp;
	bool warn_adc_duty;
	bool warn_rate_err;
	bool warn_ovs_reduced;
	bool warn_small_pool;
} rate_check_margins_t;

typedef struct {
	rate_status_t status;
	uint32_t actual_rate;
	char reason[192];
	rate_check_margins_t margins;
} rate_check_result_t;

#define RATE_CHECK_DEFAULT_TIMER_HZ            72000000U
#define RATE_CHECK_DEFAULT_ADC_HZ              72000000U
#define RATE_CHECK_DEFAULT_SAMPLE_CYCLES_X10   245U
#define RATE_CHECK_DEFAULT_DMA_MAX_SAMPLES     4096U
#define RATE_CHECK_DEFAULT_SLAB_BLOCK_BYTES    8192U
#define RATE_CHECK_DEFAULT_SLAB_NUM_BLOCKS     4U
#define RATE_CHECK_DEFAULT_PSRAM_BATCH_BYTES   262144U
#define RATE_CHECK_DEFAULT_SD_CHUNK_BYTES      32768U
#define RATE_CHECK_DEFAULT_WAV_HEADER_BYTES    32768U
#define RATE_CHECK_DEFAULT_PSRAM_BYTES         (3072ULL * 1024ULL)
#define RATE_CHECK_DEFAULT_MIN_POOL_MS         300U

#define RATE_CHECK_ADC_BUDGET_PCT_MAX          85U
#define RATE_CHECK_ADC_HARD_PCT_MAX            98U
#define RATE_CHECK_DSP_CPU_PCT_MAX             70U

const char *rate_status_to_str(rate_status_t status);
const char *rate_dsp_mode_to_str(rate_dsp_mode_t mode);

void rate_check_set_measured_dsp_ns(uint32_t ns_bypass, uint32_t ns_dc_only, uint32_t ns_full);
void rate_check_get_measured_dsp_ns(uint32_t *ns_bypass, uint32_t *ns_dc_only, uint32_t *ns_full);
void rate_check_set_measured_sd_kbps(uint32_t rate_hz, uint32_t wall_kbps);
uint32_t rate_check_get_measured_sd_kbps(uint32_t rate_hz);
void rate_check_reset_calibration(void);

rate_check_result_t rate_config_check(uint32_t rate_hz,
				      uint16_t bits,
				      uint8_t ovs,
				      uint16_t sample_cycles_x10,
				      uint32_t duration_s,
				      const rate_dsp_cfg_t *dsp_cfg);

void rate_config_check_ex(const rate_check_params_t *in, rate_check_result_t *out);

/* Returns default STM32L476 parameters populated with live runtime calibration. */
void rate_check_init_params(rate_check_params_t *p);

#ifdef __cplusplus
}
#endif

#endif /* RATE_CHECK_H_ */
