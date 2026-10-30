/* SPDX-License-Identifier: Apache-2.0 */
/*
 * STM32L4 internal-ADC audio input.
 *
 *   TIM6 update (TRGO) --> ADC1 external trigger        = sample clock, zero jitter
 *   ADC1 hardware oversampler (2^N conversions per trigger, summed)
 *   ADC1 DR --DMA1 circular--> dma_buf[2 halves]
 *   half/full DMA IRQ --> convert to int16, remove DC --> sink block
 *
 * All pins / peripherals / DMA channel come from devicetree
 * (zephyr,user node in boards/<board>.overlay).
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "dsp.h"
#include <stm32_ll_adc.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_dma.h>
#include "audio_in.h"
#include "audio_format.h"

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(audio_adc, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(audio_adc, LOG_LEVEL_WRN);
#endif

#define ZU        DT_PATH(zephyr_user)
#define ADC_NODE  DT_IO_CHANNELS_CTLR(ZU)
#define ADC_CH    DT_IO_CHANNELS_INPUT(ZU)
#define TIM_NODE  DT_NODELABEL(timers6)
#define DMA_NODE  DT_DMAS_CTLR_BY_NAME(ZU, audio_rx)
#define DMA_CH    DT_DMAS_CELL_BY_NAME(ZU, audio_rx, channel)
#define DMA_SLOT  DT_DMAS_CELL_BY_NAME(ZU, audio_rx, slot)
#define DMA_CFG   DT_DMAS_CELL_BY_NAME(ZU, audio_rx, channel_config)

#define BLOCK     CONFIG_AUDIO_BLOCK_SAMPLES
#define MAX_BLOCK BLOCK
#define OVS_LOG2  CONFIG_AUDIO_OVERSAMPLE_LOG2
#define ADC_BITS  (12 + OVS_LOG2)	/* shift = 0, so result keeps all accumulated bits */
#define LSHIFT    (16 - ADC_BITS)
/* Unshifted sum maximum is 4095 * oversampling ratio, not 2^ADC_BITS - 1. */
#define MAX_RAW_CODE (((1U << 12) - 1U) * (1U << OVS_LOG2))
/* sampling 24.5 cycles + 12.5 conversion cycles = 37 cycles (increased settling time for low noise) */
#define CYCLES_PER_CONV 37U

PINCTRL_DT_DEFINE(ADC_NODE);

static const struct device *const rcc = DEVICE_DT_GET(DT_NODELABEL(rcc));
static const struct device *const dma_dev = DEVICE_DT_GET(DMA_NODE);
static const struct stm32_pclken adc_pclken = STM32_CLOCK_INFO(0, DT_PARENT(ADC_NODE));
static const struct stm32_pclken tim_pclken = STM32_CLOCK_INFO(0, TIM_NODE);

static ADC_TypeDef *const adc = (ADC_TypeDef *)DT_REG_ADDR(ADC_NODE);
static TIM_TypeDef *const tim = (TIM_TypeDef *)DT_REG_ADDR(TIM_NODE);

static uint16_t dma_buf[2 * MAX_BLOCK] __aligned(4);
static const struct audio_sink *sink;
static uint32_t actual_rate;
static uint32_t active_requested_rate = CONFIG_AUDIO_SAMPLE_RATE;
static uint32_t active_ovs_log2 = CONFIG_AUDIO_OVERSAMPLE_LOG2;
static enum audio_capture_source current_capture_source = AUDIO_SRC_PHYSICAL_MIC;
static uint32_t tim_period;
static uint32_t active_block_samples = BLOCK;
static bool initialized = false;

struct dma_event {
	uint64_t cycle;
	uint32_t sample_start;
	uint8_t half;
};

K_MSGQ_DEFINE(dma_event_q, sizeof(struct dma_event), AUDIO_DMA_EVENT_QUEUE_DEPTH, 4);
#define ACQUISITION_THREAD_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(acquisition_thread_stack, ACQUISITION_THREAD_STACK_SIZE);
static struct k_thread acquisition_thread_data;
static bool acquisition_thread_started;
static atomic_t acquisition_events_missed;
static atomic_t acquisition_lost_samples;
static atomic_t acquisition_samples_accounted;
static atomic_t acquisition_processed_samples;
static atomic_t acquisition_max_latency_cycles;
static atomic_t acquisition_max_queue_depth;
static atomic_t acquisition_events_pending;
static atomic_t acquisition_worker_active;

/* Diagnostics tracking */
static struct k_spinlock diag_lock;
static volatile uint32_t diag_total_samples;
static volatile uint32_t diag_total_blocks;
static volatile uint16_t diag_raw_min = 0xFFFF;
static volatile uint16_t diag_raw_max = 0;
static volatile uint64_t diag_raw_sum;
static volatile uint64_t diag_raw_sum_sq;
static volatile uint32_t diag_clipping_low;
static volatile uint32_t diag_clipping_high;

void audio_in_reset_diag(void)
{
	k_spinlock_key_t key = k_spin_lock(&diag_lock);
	diag_total_samples = 0;
	diag_total_blocks = 0;
	diag_raw_min = 0xFFFF;
	diag_raw_max = 0;
	diag_raw_sum = 0;
	diag_raw_sum_sq = 0;
	diag_clipping_low = 0;
	diag_clipping_high = 0;
	k_spin_unlock(&diag_lock, key);
}

void audio_in_get_diag(struct audio_diag *diag)
{
	k_spinlock_key_t key = k_spin_lock(&diag_lock);
	diag->total_samples = diag_total_samples;
	diag->total_blocks = diag_total_blocks;
	diag->raw_min = (diag_raw_min == 0xFFFF) ? 0 : diag_raw_min;
	diag->raw_max = diag_raw_max;
	diag->raw_midpoint = diag_total_samples ? (uint32_t)(diag_raw_sum / diag_total_samples) : 0;
	diag->clipping_low_count = diag_clipping_low;
	diag->clipping_high_count = diag_clipping_high;
	diag->peak_to_peak = (diag_raw_max > diag_raw_min && diag_raw_min != 0xFFFF)
				 ? (diag_raw_max - diag_raw_min)
				 : 0;
	diag->actual_rate = actual_rate;

	if (diag_total_samples > 0) {
		uint32_t max_raw = ((1U << 12) - 1U) * (1U << active_ovs_log2);
		uint64_t mean_code = diag_raw_sum / diag_total_samples;
		uint64_t mean_sq = diag_raw_sum_sq / diag_total_samples;
		diag->raw_rms_code = (uint32_t)sqrtf((float)mean_sq);
		uint64_t var = (mean_sq > (mean_code * mean_code)) ? (mean_sq - (mean_code * mean_code)) : 0;
		uint32_t ac_rms_code = (uint32_t)sqrtf((float)var);
		diag->raw_ac_rms_code = ac_rms_code;
		diag->raw_rms_mv = (uint32_t)((uint64_t)ac_rms_code * 3300 / (max_raw ? max_raw : 1));
		if (ac_rms_code > 0 && max_raw > 0) {
			float fs_sine_rms = ((float)max_raw * 0.5f) * 0.70710678f;
			float dbfs = 20.0f * log10f((float)ac_rms_code / fs_sine_rms);
			diag->raw_rms_dbfs_x10 = (int32_t)(dbfs * 10.0f);
		} else {
			diag->raw_rms_dbfs_x10 = -960;
		}
	} else {
		diag->raw_rms_code = 0;
		diag->raw_ac_rms_code = 0;
		diag->raw_rms_mv = 0;
		diag->raw_rms_dbfs_x10 = -960;
	}
	k_spin_unlock(&diag_lock, key);
}

static bool process_half(const uint16_t *src, void *dst_block,
			 uint64_t sample_start)
{
#if CONFIG_AUDIO_SAMPLE_BITS == 32
	int32_t *dst = (int32_t *)dst_block;
#else
	int16_t *dst = (int16_t *)dst_block;
#endif

	uint32_t ovs = active_ovs_log2;
	uint32_t adc_bits = 12U + ovs;
	uint32_t lshift = 16U - adc_bits;
	uint32_t max_raw = ((1U << 12) - 1U) * (1U << ovs);
	bool is_synth = (current_capture_source == AUDIO_SRC_SYNTHETIC);

	/* Always track diagnostics regardless of pool availability */
	uint16_t local_min = 0xFFFF;
	uint16_t local_max = 0;
	uint64_t local_sum = 0;
	uint64_t local_sum_sq = 0;
	uint32_t local_clip_l = 0;
	uint32_t local_clip_h = 0;

	for (uint32_t n = 0; n < active_block_samples; n++) {
#if CONFIG_AUDIO_SAMPLE_BITS == 32
		/* Expand in reverse so writes never overwrite raw samples not yet read. */
		uint32_t i = active_block_samples - 1U - n;
#else
		uint32_t i = n;
#endif
		uint16_t raw;
		if (is_synth) {
			uint64_t global_sample = sample_start + i;
			float phase = 2.0f * 3.14159265f * 1000.0f * (float)global_sample / (float)actual_rate;
			int16_t s16 = (int16_t)(sinf(phase) * 16384.0f);
			raw = (uint16_t)(((int32_t)s16 + 32768) >> lshift);
			if (raw > max_raw) raw = (uint16_t)max_raw;
		} else {
			raw = src[i];
		}

		if (raw < local_min) local_min = raw;
		if (raw > local_max) local_max = raw;
		local_sum += raw;
		local_sum_sq += ((uint64_t)raw * (uint64_t)raw);

		if (raw == 0) {
			local_clip_l++;
		}
		if (raw >= max_raw) {
			local_clip_h++;
		}

#if CONFIG_AUDIO_SAMPLE_BITS == 32
		if (is_synth) {
			uint64_t global_sample = sample_start + i;
			float phase = 2.0f * 3.14159265f * 1000.0f * (float)global_sample / (float)actual_rate;
			int16_t s16 = (int16_t)(sinf(phase) * 16384.0f);
			dst[i] = audio_pcm16_to_pcm32(s16);
		} else {
			/*
			 * Convert native ADC code to signed 16-bit PCM and scale to 32-bit container.
			 * Uses well-defined unsigned conversion without negative shift undefined behavior.
			 */
			int16_t s16 = audio_adc_raw_to_pcm16(raw, lshift);
			dst[i] = audio_pcm16_to_pcm32(s16);
		}
#else
		int16_t s16 = audio_adc_raw_to_pcm16(raw, lshift);
		dst[i] = s16;
#endif
	}

	/* Update global diagnostics */
	k_spinlock_key_t key = k_spin_lock(&diag_lock);
	if (local_min < diag_raw_min) diag_raw_min = local_min;
	if (local_max > diag_raw_max) diag_raw_max = local_max;
	diag_raw_sum += local_sum;
	diag_raw_sum_sq += local_sum_sq;
	diag_clipping_low += local_clip_l;
	diag_clipping_high += local_clip_h;
	diag_total_samples += active_block_samples;
	diag_total_blocks++;
	k_spin_unlock(&diag_lock, key);

	ARG_UNUSED(sample_start);

	if (sink && sink->submit) {
		return sink->submit(dst_block, sink->user) == 0;
	}
	if (sink && sink->release) sink->release(dst_block, sink->user);
	return false;
}

static volatile uint32_t dma_ht_callbacks;
static volatile uint32_t dma_tc_callbacks;
static volatile uint32_t dma_callback_errors;
static volatile uint32_t adc_overruns;
static volatile uint32_t dma_source_sample_index;
static volatile uint64_t dma_last_callback_cycle;
static volatile uint64_t dma_interval_cycles_sum;
static volatile uint64_t dma_interval_cycles_min;
static volatile uint64_t dma_interval_cycles_max;
static volatile uint64_t dma_exec_cycles_sum;
static volatile uint64_t dma_exec_cycles_min;
static volatile uint64_t dma_exec_cycles_max;
static uint64_t acquisition_process_cycles_sum;
static uint32_t acquisition_process_cycles_max;
static uint32_t acquisition_process_blocks;
static bool dma_active;

static bool dma_half_is_stable(uint8_t half, uint32_t block_samples)
{
	uint32_t remaining = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
	if (half == 0) {
		/* HT completed half 0; DMA is stable while it is filling half 1. */
		return remaining > 0 && remaining <= block_samples;
	}
	/* TC completed half 1; DMA is stable while it is filling half 0. */
	return remaining > block_samples && remaining <= 2U * block_samples;
}

static void acquisition_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		struct dma_event event;
		if (k_msgq_get(&dma_event_q, &event, K_FOREVER) != 0) {
			continue;
		}
		atomic_set(&acquisition_worker_active, 1);
		atomic_dec(&acquisition_events_pending);

		uint64_t copy_start = k_cycle_get_64();
		uint64_t latency = copy_start - event.cycle;
		uint32_t block = active_block_samples;
		const uint16_t *src = &dma_buf[event.half ? block : 0];
		void *dst_block = (sink && sink->alloc) ? sink->alloc(sink->user) : NULL;
		if (!dst_block) {
			atomic_add(&acquisition_lost_samples, block);
			atomic_add(&acquisition_samples_accounted, block);
			atomic_set(&acquisition_worker_active, 0);
			continue;
		}
		bool stable_before = dma_half_is_stable(event.half, block);
		if (stable_before) {
			memcpy(dst_block, src, block * sizeof(uint16_t));
		}
		uint64_t copy_end = k_cycle_get_64();
		uint64_t reuse_cycles = k_us_to_cyc_ceil64(
			(uint64_t)block * 1000000ULL / actual_rate);

		/* A late copy may overlap DMA reuse; reject it and account for the loss. */
		bool stable_after = stable_before && dma_half_is_stable(event.half, block);
		if (!stable_after || copy_end - event.cycle >= reuse_cycles) {
			if (sink && sink->release) {
				sink->release(dst_block, sink->user);
			}
			atomic_add(&acquisition_events_missed, 1);
			atomic_add(&acquisition_lost_samples, block);
			atomic_add(&acquisition_samples_accounted, block);
			atomic_set(&acquisition_worker_active, 0);
			continue;
		}

		int old_max = atomic_get(&acquisition_max_latency_cycles);
		while ((uint64_t)old_max < latency &&
		       !atomic_cas(&acquisition_max_latency_cycles, old_max, (atomic_val_t)latency)) {
			old_max = atomic_get(&acquisition_max_latency_cycles);
		}
		uint64_t process_start = k_cycle_get_64();
		bool submitted = process_half((const uint16_t *)dst_block, dst_block,
					      event.sample_start);
		uint64_t process_cycles = k_cycle_get_64() - process_start;
		acquisition_process_cycles_sum += process_cycles;
		acquisition_process_blocks++;
		if (process_cycles > acquisition_process_cycles_max) {
			acquisition_process_cycles_max = (uint32_t)process_cycles;
		}
		if (submitted) {
			atomic_add(&acquisition_processed_samples, block);
		} else {
			atomic_add(&acquisition_lost_samples, block);
		}
		atomic_add(&acquisition_samples_accounted, block);
		atomic_set(&acquisition_worker_active, 0);
	}
}

static void dma_cb(const struct device *dev, void *user, uint32_t ch, int status)
{
	/* [PHASE 7] First DMA callback received: publish the half event only. */
	uint64_t cycle_start = k_cycle_get_64();
	ARG_UNUSED(dev);
	ARG_UNUSED(user);
	ARG_UNUSED(ch);
	if (dma_last_callback_cycle != 0) {
		uint64_t interval = cycle_start - dma_last_callback_cycle;
		dma_interval_cycles_sum += interval;
		if (interval < dma_interval_cycles_min) dma_interval_cycles_min = interval;
		if (interval > dma_interval_cycles_max) dma_interval_cycles_max = interval;
	}
	dma_last_callback_cycle = cycle_start;
	if (LL_ADC_IsActiveFlag_OVR(adc)) {
		adc_overruns++;
		LL_ADC_ClearFlag_OVR(adc);
	}

	struct dma_event event = {
		.cycle = cycle_start,
		.sample_start = dma_source_sample_index,
	};
	if (status == DMA_STATUS_BLOCK) {		/* half transfer: first half is ready */
		dma_ht_callbacks++;
		event.half = 0;
	} else if (status == DMA_STATUS_COMPLETE) {	/* full: second half is ready */
		dma_tc_callbacks++;
		event.half = 1;
	} else {
		dma_callback_errors++;
		goto done;
	}
	dma_source_sample_index += active_block_samples;
	if (k_msgq_put(&dma_event_q, &event, K_NO_WAIT) != 0) {
		atomic_add(&acquisition_events_missed, 1);
		atomic_add(&acquisition_lost_samples, active_block_samples);
		atomic_add(&acquisition_samples_accounted, active_block_samples);
	} else {
		uint32_t used = (uint32_t)atomic_inc(&acquisition_events_pending) + 1U;
		int old_max = atomic_get(&acquisition_max_queue_depth);
		while ((uint32_t)old_max < used &&
		       !atomic_cas(&acquisition_max_queue_depth, old_max, (atomic_val_t)used)) {
			old_max = atomic_get(&acquisition_max_queue_depth);
		}
	}
done:
	uint64_t exec_cycles = k_cycle_get_64() - cycle_start;
	dma_exec_cycles_sum += exec_cycles;
	if (exec_cycles < dma_exec_cycles_min) dma_exec_cycles_min = exec_cycles;
	if (exec_cycles > dma_exec_cycles_max) dma_exec_cycles_max = exec_cycles;
}

void audio_in_get_callback_counts(uint32_t *ht_count, uint32_t *tc_count)
{
	if (ht_count) *ht_count = dma_ht_callbacks;
	if (tc_count) *tc_count = dma_tc_callbacks;
}

void audio_in_get_dma_stats(struct audio_dma_stats *stats)
{
	if (!stats) return;
	uint32_t callbacks = dma_ht_callbacks + dma_tc_callbacks;
	uint32_t intervals = (callbacks > 0) ? callbacks - 1U : 0U;
	stats->callback_errors = dma_callback_errors;
	stats->adc_overruns = adc_overruns;
	stats->callbacks = callbacks;
	stats->callback_interval_min_cycles = intervals ? dma_interval_cycles_min : 0;
	stats->callback_interval_avg_cycles = intervals ? dma_interval_cycles_sum / intervals : 0;
	stats->callback_interval_max_cycles = intervals ? dma_interval_cycles_max : 0;
	stats->callback_exec_min_cycles = callbacks ? dma_exec_cycles_min : 0;
	stats->callback_exec_avg_cycles = callbacks ? dma_exec_cycles_sum / callbacks : 0;
	stats->callback_exec_max_cycles = callbacks ? dma_exec_cycles_max : 0;
	stats->events_missed = (uint32_t)atomic_get(&acquisition_events_missed);
	stats->samples_acquired = (uint64_t)callbacks * active_block_samples;
	stats->samples_accounted = (uint32_t)atomic_get(&acquisition_samples_accounted);
	stats->samples_processed = (uint32_t)atomic_get(&acquisition_processed_samples);
	stats->samples_lost = (uint32_t)atomic_get(&acquisition_lost_samples);
	stats->acquisition_max_latency_cycles = (uint32_t)atomic_get(&acquisition_max_latency_cycles);
	stats->acquisition_process_avg_cycles = acquisition_process_blocks ?
		(uint32_t)(acquisition_process_cycles_sum / acquisition_process_blocks) : 0;
	stats->acquisition_process_max_cycles = acquisition_process_cycles_max;
	stats->event_queue_high_water = (uint32_t)atomic_get(&acquisition_max_queue_depth);
}

uint32_t audio_in_get_missed_event_count(void)
{
	return (uint32_t)atomic_get(&acquisition_events_missed);
}

static uint32_t get_target_ovs_log2(uint32_t rate_hz)
{
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	if (rate_hz >= 500000U) {
		return 0U; /* 1x (off) for 512k: 41 cycles / 80 MHz = 512.5 ns (26.2% of period) */
	} else if (rate_hz >= 350000U) {
		return 1U; /* 2x for 384k: 78 cycles / 80 MHz = 975 ns (37.5% of period) */
	}
#endif
	return CONFIG_AUDIO_OVERSAMPLE_LOG2; /* default 3 (8x): 300 cycles / 80 MHz = 3.75 us */
}

static uint32_t ovs_ratio_const_for_log2(uint32_t ovs_log2)
{
	switch (ovs_log2) {
	case 1: return LL_ADC_OVS_RATIO_2;
	case 2: return LL_ADC_OVS_RATIO_4;
	case 3: return LL_ADC_OVS_RATIO_8;
	default: return LL_ADC_OVS_RATIO_16;
	}
}

int audio_in_init(uint32_t requested_rate_hz, const struct audio_sink *s)
{
	uint32_t ahb_hz, apb_hz;
	int ret;

	sink = s;

	if (!device_is_ready(rcc) || !device_is_ready(dma_dev)) {
		return -ENODEV;
	}
	if (!acquisition_thread_started) {
		k_thread_create(&acquisition_thread_data, acquisition_thread_stack,
				K_THREAD_STACK_SIZEOF(acquisition_thread_stack),
				acquisition_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(-1),
				0, K_NO_WAIT);
		k_thread_name_set(&acquisition_thread_data, "audio_acquire");
		acquisition_thread_started = true;
	}

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(ADC_NODE), PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}
	clock_control_on(rcc, (clock_control_subsys_t)&adc_pclken);
	clock_control_on(rcc, (clock_control_subsys_t)&tim_pclken);
	clock_control_get_rate(rcc, (clock_control_subsys_t)&adc_pclken, &ahb_hz);
	clock_control_get_rate(rcc, (clock_control_subsys_t)&tim_pclken, &apb_hz);

	/* ---- sample clock: TIM6 update rate ---- */
	uint32_t tim_hz = (LL_RCC_GetAPB1Prescaler() == LL_RCC_APB1_DIV_1) ? apb_hz : apb_hz * 2;

	tim_period = (tim_hz + requested_rate_hz / 2) / requested_rate_hz;
	actual_rate = tim_hz / tim_period;
	active_requested_rate = requested_rate_hz;
	active_ovs_log2 = get_target_ovs_log2(requested_rate_hz);

	LOG_INF("timer %u Hz / %u = %u Hz (requested %u)", tim_hz, tim_period, actual_rate,
		requested_rate_hz);
	LOG_INF("DMA block size: record=%u (%u us), max samples=%u",
		BLOCK, (uint32_t)((uint64_t)BLOCK * 1000000ULL / actual_rate),
		MAX_BLOCK);

	/* ---- timing budget: all oversampled conversions must fit in one period ---- */
	uint64_t conv_ns = ((uint64_t)((1U << active_ovs_log2) * CYCLES_PER_CONV + 4U) * 1000000000ULL) /
			   ahb_hz;
	uint64_t period_ns = 1000000000ULL / actual_rate;

	LOG_INF("ADC: %ux oversampling, %u-bit result, conversion %u ns of %u ns period (%u%%)",
		1U << active_ovs_log2, 12U + active_ovs_log2, (uint32_t)conv_ns, (uint32_t)period_ns,
		(uint32_t)(conv_ns * 100 / period_ns));
	if (conv_ns * 100 > period_ns * 85) {
		LOG_ERR("oversampling too high for this sample rate (needs > 85%% of period)");
		return -ERANGE;
	}

	/* ---- ADC Hardware Init ---- */
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc), LL_ADC_CLOCK_SYNC_PCLK_DIV1);
	LL_ADC_DisableDeepPowerDown(adc);
	LL_ADC_EnableInternalRegulator(adc);
	k_busy_wait(30);

	/* Start Calibration with bounded timeout */
	LL_ADC_StartCalibration(adc, LL_ADC_SINGLE_ENDED);
	uint32_t cal_timeout = 10000;
	while (LL_ADC_IsCalibrationOnGoing(adc) && --cal_timeout) {
		k_busy_wait(1);
	}
	if (cal_timeout == 0) {
		LOG_ERR("ADC calibration timed out");
		return -ETIMEDOUT;
	}

	const uint32_t ch = __LL_ADC_DECIMAL_NB_TO_CHANNEL(ADC_CH);

	LL_ADC_SetResolution(adc, LL_ADC_RESOLUTION_12B);
	LL_ADC_SetDataAlignment(adc, LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_DISABLE);
	LL_ADC_REG_SetSequencerRanks(adc, LL_ADC_REG_RANK_1, ch);
	LL_ADC_SetChannelSamplingTime(adc, ch, LL_ADC_SAMPLINGTIME_24CYCLES_5);
	LL_ADC_SetChannelSingleDiff(adc, ch, LL_ADC_SINGLE_ENDED);

	LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM6_TRGO);
	LL_ADC_REG_SetTriggerEdge(adc, LL_ADC_REG_TRIG_EXT_RISING);
	LL_ADC_REG_SetContinuousMode(adc, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetDMATransfer(adc, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);	/* circular */

	if (active_ovs_log2 == 0) {
		LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_DISABLE);
	} else {
		LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
		LL_ADC_ConfigOverSamplingRatioShift(adc, ovs_ratio_const_for_log2(active_ovs_log2), LL_ADC_OVS_SHIFT_NONE);
		/* one trigger = all 2^N conversions back-to-back = one output sample */
		LL_ADC_SetOverSamplingDiscont(adc, LL_ADC_OVS_REG_CONT);
	}

	/* ---- timer: update event drives TRGO ---- */
	LL_TIM_SetPrescaler(tim, 0);
	LL_TIM_SetAutoReload(tim, tim_period - 1);
	LL_TIM_SetCounterMode(tim, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetTriggerOutput(tim, LL_TIM_TRGO_UPDATE);
	LL_TIM_EnableARRPreload(tim);

	/* ---- DMA check (configured per recording session, keeping idle channel free) ---- */
	if (!device_is_ready(dma_dev)) {
		return -ENODEV;
	}
	initialized = true;
	return 0;
}

uint32_t audio_in_rate(void)
{
	return actual_rate;
}

int audio_in_set_block_size(uint32_t samples)
{
	if (dma_active) {
		return -EBUSY;
	}
	if (samples != BLOCK) {
		return -EINVAL;
	}
	active_block_samples = samples;
	return 0;
}

bool audio_in_is_rate_supported(uint32_t rate_hz, uint32_t *act_rate, uint32_t *divider, int *error_hz)
{
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	/* TEST build allows experimental high sample rates 384000 and 512000 */
	if (rate_hz != 16000 && rate_hz != 32000 && rate_hz != 44100 &&
	    rate_hz != 48000 && rate_hz != 96000 && rate_hz != 192000 &&
	    rate_hz != 384000 && rate_hz != 512000) {
		return false;
	}
#else
	/* Production build: strictly validate production hardware-verified rates */
	if (rate_hz != 16000 && rate_hz != 32000 && rate_hz != 44100 &&
	    rate_hz != 48000 && rate_hz != 96000 && rate_hz != 192000) {
		return false;
	}
#endif

	uint32_t apb_hz = 0;
	if (clock_control_get_rate(rcc, (clock_control_subsys_t)&tim_pclken, &apb_hz) != 0 || apb_hz == 0) {
		return false;
	}
	uint32_t tim_hz = (LL_RCC_GetAPB1Prescaler() == LL_RCC_APB1_DIV_1) ? apb_hz : apb_hz * 2;
	uint32_t div = (tim_hz + rate_hz / 2) / rate_hz;
	if (div == 0) return false;
	uint32_t actual = tim_hz / div;

	/* Verify conversion time budget against period with adaptive oversampling */
	uint32_t ahb_hz = 0;
	if (clock_control_get_rate(rcc, (clock_control_subsys_t)&adc_pclken, &ahb_hz) != 0 || ahb_hz == 0) {
		return false;
	}
	uint32_t ovs = get_target_ovs_log2(rate_hz);
	uint64_t conv_ns = ((uint64_t)((1U << ovs) * CYCLES_PER_CONV + 4U) * 1000000000ULL) / ahb_hz;
	uint64_t period_ns = 1000000000ULL / actual;
	if (conv_ns * 100 > period_ns * 85) {
		return false;
	}

	if (act_rate) *act_rate = actual;
	if (divider) *divider = div;
	if (error_hz) *error_hz = (int)actual - (int)rate_hz;
	return true;
}

int audio_in_set_rate(uint32_t new_rate_hz)
{
	if (dma_active) {
		return -EBUSY;
	}

	uint32_t actual = 0, div = 0;
	int err_hz = 0;
	if (!audio_in_is_rate_supported(new_rate_hz, &actual, &div, &err_hz)) {
		LOG_ERR("Unsupported sample rate: %u Hz (clock or oversampling budget exceeded)", new_rate_hz);
		return -ERANGE;
	}

	active_requested_rate = new_rate_hz;
	active_ovs_log2 = get_target_ovs_log2(new_rate_hz);
	tim_period = div;
	actual_rate = actual;

	LL_TIM_SetAutoReload(tim, tim_period - 1);
	LL_TIM_SetCounter(tim, 0);

	/* Reconfigure ADC oversampling scope and ratio */
	if (active_ovs_log2 == 0) {
		LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_DISABLE);
	} else {
		LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
		LL_ADC_ConfigOverSamplingRatioShift(adc, ovs_ratio_const_for_log2(active_ovs_log2), LL_ADC_OVS_SHIFT_NONE);
		LL_ADC_SetOverSamplingDiscont(adc, LL_ADC_OVS_REG_CONT);
	}

	/* Update DSP pipeline sample rate and biquad coefficients */
	dsp_init(actual_rate);

	LOG_INF("Audio rate changed: requested %u Hz, actual %u Hz (TIM6 div=%u, error %d Hz, %ux ovs)",
	        new_rate_hz, actual_rate, tim_period, err_hz, 1U << active_ovs_log2);
	return 0;
}

int audio_in_start(void)
{
	if (!initialized) {
		return -EINVAL;
	}
	if (dma_active) {
		dma_stop(dma_dev, DMA_CH);
		dma_active = false;
	}

	/* Reset callback counters and diagnostics */
	dma_ht_callbacks = 0;
	dma_tc_callbacks = 0;
	dma_callback_errors = 0;
	adc_overruns = 0;
	atomic_set(&acquisition_events_missed, 0);
	atomic_set(&acquisition_lost_samples, 0);
	atomic_set(&acquisition_samples_accounted, 0);
	atomic_set(&acquisition_processed_samples, 0);
	atomic_set(&acquisition_max_latency_cycles, 0);
	atomic_set(&acquisition_max_queue_depth, 0);
	atomic_set(&acquisition_events_pending, 0);
	atomic_set(&acquisition_worker_active, 0);
	acquisition_process_cycles_sum = 0;
	acquisition_process_cycles_max = 0;
	acquisition_process_blocks = 0;
	dma_last_callback_cycle = 0;
	dma_source_sample_index = 0;
	dma_interval_cycles_sum = 0;
	dma_interval_cycles_min = UINT64_MAX;
	dma_interval_cycles_max = 0;
	dma_exec_cycles_sum = 0;
	dma_exec_cycles_min = UINT64_MAX;
	dma_exec_cycles_max = 0;
	audio_in_reset_diag();

	/* Zero circular DMA buffer to ensure deterministic clean startup */
	memset(dma_buf, 0, sizeof(dma_buf));

	/* Clear residual DMA flags */
	LL_DMA_ClearFlag_GI1(DMA1);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_HT1(DMA1);
	LL_DMA_ClearFlag_TE1(DMA1);

	/* Fresh DMA configuration through Zephyr API (driver channel is confirmed free) */
	struct dma_block_config blk = {
		.source_address = (uint32_t)&adc->DR,
		.dest_address = (uint32_t)dma_buf,
		.block_size = 2U * active_block_samples * sizeof(dma_buf[0]),
		.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		/* Circular mode is selected by source/dest reload flags in STM32 driver */
		.source_reload_en = 1,
		.dest_reload_en = 1,
	};
	struct dma_config cfg = {
		.dma_slot = DMA_SLOT,
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.channel_priority = STM32_DMA_CONFIG_PRIORITY(DMA_CFG),
		.source_data_size = 2,
		.dest_data_size = 2,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.block_count = 1,
		.head_block = &blk,
		.dma_callback = dma_cb,
	};

	int ret = dma_config(dma_dev, DMA_CH, &cfg);
	if (ret) {
		LOG_ERR("dma_config failed: %d", ret);
		return ret;
	}

	/* Explicitly ensure TC, HT, and TE interrupts are enabled in hardware */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

	ret = dma_start(dma_dev, DMA_CH);
	if (ret) {
		LOG_ERR("dma_start failed: %d", ret);
		dma_stop(dma_dev, DMA_CH);
		return ret;
	}
	dma_active = true;

	/* Reset TIM6 counter and update flag to prevent premature or misaligned trigger */
	LL_TIM_DisableCounter(tim);
	LL_TIM_SetCounter(tim, 0);
	LL_TIM_ClearFlag_UPDATE(tim);

	/* Exit ADC Deep Power Down and re-enable internal voltage regulator if in low-power state */
	if (LL_ADC_IsDeepPowerDownEnabled(adc)) {
		LL_ADC_DisableDeepPowerDown(adc);
	}
	if (!LL_ADC_IsInternalRegulatorEnabled(adc)) {
		LL_ADC_EnableInternalRegulator(adc);
		k_busy_wait(20); /* wait for regulator startup tADCVREG_STUP */
	}

	if (!LL_ADC_IsEnabled(adc)) {
		LL_ADC_ClearFlag_ADRDY(adc);
		LL_ADC_Enable(adc);

		uint32_t en_timeout = 10000;
		while (!LL_ADC_IsActiveFlag_ADRDY(adc) && --en_timeout) {
			k_busy_wait(1);
		}
		if (en_timeout == 0) {
			LOG_ERR("ADC enable timed out waiting for ADRDY");
			/* Channel is enabled, so dma_stop will disable it and release busy flag */
			dma_stop(dma_dev, DMA_CH);
			dma_active = false;
			return -ETIMEDOUT;
		}
	}
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	printk("[PHASE 5] ADC/DMA configured (DMA1 Ch1 active, ADC ADRDY confirmed)\n");
	printk("[PHASE 6] Timer started (TIM6 active, triggering ADC TRGO @ %u Hz)\n", actual_rate);
#endif

	LL_ADC_REG_StartConversion(adc);	/* waiting for timer trigger */
	LL_TIM_EnableCounter(tim);
	return 0;
}

int audio_in_stop(void)
{
	/* 1. Stop TIM6 triggers first so no new conversions begin */
	LL_TIM_DisableCounter(tim);
	LL_TIM_SetCounter(tim, 0);
	LL_TIM_ClearFlag_UPDATE(tim);

	/* 2. Stop ongoing ADC regular conversion */
	if (LL_ADC_REG_IsConversionOngoing(adc)) {
		LL_ADC_REG_StopConversion(adc);
		uint32_t stop_timeout = 10000;
		while (LL_ADC_REG_IsStopConversionOngoing(adc) && --stop_timeout) {
			k_busy_wait(1);
		}
	}

	/* 3. Stop DMA while the hardware stream is still enabled:
	 *    Zephyr's dma_stm32_stop() checks stm32_dma_is_enabled_stream(),
	 *    clears IRQ, disables stream, and sets stream->busy = false.
	 *    This safely releases driver ownership for the next session!
	 */
	if (dma_active) {
		dma_stop(dma_dev, DMA_CH);
		dma_active = false;
	}
	while (atomic_get(&acquisition_events_pending) != 0 ||
	       atomic_get(&acquisition_worker_active) != 0) {
		k_sleep(K_MSEC(1));
	}

	/* 4. Disable ADC */
	if (LL_ADC_IsEnabled(adc)) {
		LL_ADC_Disable(adc);
		uint32_t dis_timeout = 10000;
		while (LL_ADC_IsEnabled(adc) && --dis_timeout) {
			k_busy_wait(1);
		}
	}

	/* 5. Clear residual flags after DMA is safely stopped */
	LL_DMA_ClearFlag_GI1(DMA1);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_HT1(DMA1);
	LL_DMA_ClearFlag_TE1(DMA1);

	/* 6. Enter ADC Deep Power Down and disable internal regulator for ultra-low sleep current */
	LL_ADC_EnableDeepPowerDown(adc);
	LL_ADC_DisableInternalRegulator(adc);

	return 0;
}

uint32_t audio_in_requested_rate(void)
{
	return active_requested_rate;
}

void audio_in_set_capture_source(enum audio_capture_source src)
{
	current_capture_source = src;
}

enum audio_capture_source audio_in_get_capture_source(void)
{
	return current_capture_source;
}

uint32_t audio_in_get_ovs_ratio(void)
{
	return 1U << active_ovs_log2;
}



