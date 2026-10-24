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

LOG_MODULE_REGISTER(audio_adc, LOG_LEVEL_INF);

#define ZU        DT_PATH(zephyr_user)
#define ADC_NODE  DT_IO_CHANNELS_CTLR(ZU)
#define ADC_CH    DT_IO_CHANNELS_INPUT(ZU)
#define TIM_NODE  DT_NODELABEL(timers6)
#define DMA_NODE  DT_DMAS_CTLR_BY_NAME(ZU, audio_rx)
#define DMA_CH    DT_DMAS_CELL_BY_NAME(ZU, audio_rx, channel)
#define DMA_SLOT  DT_DMAS_CELL_BY_NAME(ZU, audio_rx, slot)
#define DMA_CFG   DT_DMAS_CELL_BY_NAME(ZU, audio_rx, channel_config)

#define BLOCK     CONFIG_AUDIO_BLOCK_SAMPLES
#define DIAG_BLOCK CONFIG_AUDIO_DIAGNOSTIC_BLOCK_SAMPLES
#define MAX_BLOCK ((BLOCK > DIAG_BLOCK) ? BLOCK : DIAG_BLOCK)
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
static volatile bool integrity_replay_enabled;
/* Replay command is bounded to <=30 s, so this fits without 64-bit ISR math. */
static volatile uint32_t dma_source_sample_index;

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
		uint64_t mean_code = diag_raw_sum / diag_total_samples;
		uint64_t mean_sq = diag_raw_sum_sq / diag_total_samples;
		diag->raw_rms_code = (uint32_t)sqrtf((float)mean_sq);
		uint64_t var = (mean_sq > (mean_code * mean_code)) ? (mean_sq - (mean_code * mean_code)) : 0;
		uint32_t ac_rms_code = (uint32_t)sqrtf((float)var);
		diag->raw_ac_rms_code = ac_rms_code;
		diag->raw_rms_mv = (uint32_t)((uint64_t)ac_rms_code * 3300 / MAX_RAW_CODE);
		if (ac_rms_code > 0) {
			float fs_sine_rms = ((float)MAX_RAW_CODE * 0.5f) * 0.70710678f;
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
		uint16_t raw = src[i];
		if (raw < local_min) local_min = raw;
		if (raw > local_max) local_max = raw;
		local_sum += raw;
		local_sum_sq += ((uint64_t)raw * (uint64_t)raw);

		if (raw == 0) {
			local_clip_l++;
		}
		if (raw >= MAX_RAW_CODE) {
			local_clip_h++;
		}

#if CONFIG_AUDIO_SAMPLE_BITS == 32
		/*
		 * STM32L4 ADC delivers ADC_BITS (12 + OVS_LOG2, default 15) bits.
		 * Left-shift by (32 - ADC_BITS) to span 32-bit container, then invert MSB
		 * (equivalent to subtracting midpoint 2^31) to center signed PCM around 0.
		 */
		uint32_t u32 = ((uint32_t)raw << (32 - ADC_BITS));
		int32_t x = (int32_t)(u32 ^ 0x80000000U);
		dst[i] = x;
#else
		/* Convert unsigned raw ADC reading to signed 16-bit PCM centered at 0 with saturating clamp */
		int32_t x = ((int32_t)raw << LSHIFT) - 32768;
		if (x > 32767) x = 32767;
		else if (x < -32768) x = -32768;
		dst[i] = (int16_t)x;
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

#if CONFIG_AUDIO_INTEGRITY_REPLAY
	/* ADC diagnostics above remain real; replace only the downstream PCM block. */
	if (integrity_replay_enabled) {
		int16_t *replay = (int16_t *)dst_block;
		for (uint32_t i = 0; i < active_block_samples; i++) {
			replay[i] = audio_in_integrity_sample(sample_start + i);
		}
	}
#else
	ARG_UNUSED(sample_start);
#endif

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

#if CONFIG_AUDIO_INTEGRITY_REPLAY
int audio_in_set_integrity_replay(bool enabled)
{
	if (dma_active) return -EBUSY;
	integrity_replay_enabled = enabled;
	return 0;
}
#endif

static uint32_t ovs_ratio_const(void)
{
	switch (OVS_LOG2) {
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
	LOG_INF("timer %u Hz / %u = %u Hz (requested %u)", tim_hz, tim_period, actual_rate,
		requested_rate_hz);
	LOG_INF("DMA block sizes: record=%u (%u us), diagnostic=%u (%u us), max samples=%u",
		BLOCK, (uint32_t)((uint64_t)BLOCK * 1000000ULL / actual_rate),
		DIAG_BLOCK, (uint32_t)((uint64_t)DIAG_BLOCK * 1000000ULL / actual_rate),
		MAX_BLOCK);

	/* ---- timing budget: all oversampled conversions must fit in one period ---- */
	uint64_t conv_ns = ((uint64_t)((1U << OVS_LOG2) * CYCLES_PER_CONV + 4U) * 1000000000ULL) /
			   ahb_hz;
	uint64_t period_ns = 1000000000ULL / actual_rate;

	LOG_INF("ADC: %ux oversampling, %u-bit result, conversion %u ns of %u ns period (%u%%)",
		1U << OVS_LOG2, ADC_BITS, (uint32_t)conv_ns, (uint32_t)period_ns,
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

	if (OVS_LOG2 == 0) {
		LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_DISABLE);
	} else {
		LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
		LL_ADC_ConfigOverSamplingRatioShift(adc, ovs_ratio_const(), LL_ADC_OVS_SHIFT_NONE);
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
	if (samples != BLOCK && samples != DIAG_BLOCK) {
		return -EINVAL;
	}
	active_block_samples = samples;
	return 0;
}

bool audio_in_is_rate_supported(uint32_t rate_hz, uint32_t *act_rate, uint32_t *divider, int *error_hz)
{
	/* Supported rates: 16k, 32k, 44.1k, 48k, 96k, 192k */
	if (rate_hz != 16000 && rate_hz != 32000 && rate_hz != 44100 &&
	    rate_hz != 48000 && rate_hz != 96000 && rate_hz != 192000) {
		return false;
	}

	uint32_t apb_hz = 0;
	clock_control_get_rate(rcc, (clock_control_subsys_t)&tim_pclken, &apb_hz);
	uint32_t tim_hz = (LL_RCC_GetAPB1Prescaler() == LL_RCC_APB1_DIV_1) ? apb_hz : apb_hz * 2;
	uint32_t div = (tim_hz + rate_hz / 2) / rate_hz;
	if (div == 0) return false;
	uint32_t actual = tim_hz / div;

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
		LOG_ERR("Unsupported sample rate: %u Hz", new_rate_hz);
		return -EINVAL;
	}

	/* Verify conversion time budget against period */
	uint32_t ahb_hz = 0;
	clock_control_get_rate(rcc, (clock_control_subsys_t)&adc_pclken, &ahb_hz);
	uint64_t conv_ns = ((uint64_t)((1U << OVS_LOG2) * CYCLES_PER_CONV + 4U) * 1000000000ULL) / ahb_hz;
	uint64_t period_ns = 1000000000ULL / actual;
	if (conv_ns * 100 > period_ns * 85) {
		LOG_ERR("Rate %u Hz exceeds ADC oversampling budget", new_rate_hz);
		return -ERANGE;
	}

	tim_period = div;
	actual_rate = actual;

	LL_TIM_SetAutoReload(tim, tim_period - 1);
	LL_TIM_SetCounter(tim, 0);

	/* Update DSP pipeline sample rate and biquad coefficients */
	dsp_init(actual_rate);

	LOG_INF("Audio rate changed: requested %u Hz, actual %u Hz (TIM6 div=%u, error %d Hz)",
	        new_rate_hz, actual_rate, tim_period, err_hz);
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
	printk("[PHASE 5] ADC/DMA configured (DMA1 Ch1 active, ADC ADRDY confirmed)\n");
	printk("[PHASE 6] Timer started (TIM6 active, triggering ADC TRGO @ %u Hz)\n", actual_rate);

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

/* Standalone diagnostic monitor */
#if CONFIG_AUDIO_SAMPLE_BITS == 32
static int32_t diag_dummy_block[MAX_BLOCK];
#else
static int16_t diag_dummy_block[MAX_BLOCK];
#endif
static void *diag_dummy_alloc(void *u) { ARG_UNUSED(u); return diag_dummy_block; }
static int diag_dummy_submit(void *b, void *u) { ARG_UNUSED(b); ARG_UNUSED(u); return 0; }
static const struct audio_sink diag_sink = {
	.alloc = diag_dummy_alloc,
	.submit = diag_dummy_submit,
};

int audio_in_run_diagnostic(uint32_t seconds)
{
	const struct audio_sink *saved_sink = sink;
	int ret = 0;

	if (!initialized) {
		ret = audio_in_init(CONFIG_AUDIO_SAMPLE_RATE, &diag_sink);
		if (ret) {
			LOG_ERR("audio_in_init failed: %d", ret);
			return ret;
		}
	} else {
		sink = &diag_sink;
	}

	audio_in_reset_diag();
	ret = audio_in_start();
	if (ret) {
		LOG_ERR("audio_in_start failed: %d", ret);
		sink = saved_sink;
		return ret;
	}

	printk("\n=========================================================================\n");
	printk("                MAX4466 REAL-TIME ADC DIAGNOSTIC MONITOR                 \n");
	printk("  MCU / Board       : STM32L476RG / NUCLEO-L476RG\n");
	printk("  Input Pin         : PA0 (Arduino A0 / ADC1_IN5)\n");
	printk("  Hardware Config   : TIM6 TRGO + DMA1 Ch1 + 8x HW Oversampling\n");
	printk("  Sampling Rate     : %u Hz (16-bit PCM derived from 12-bit ADC)\n", actual_rate);
	printk("  DMA Block         : %u samples (%.3f Hz block rate)\n",
	       active_block_samples,
	       (double)actual_rate / (double)active_block_samples);
	printk("  Analog Bandwidth  : ~20 Hz - 20 kHz (Audible Range; NOT Ultrasonic)\n");
	printk("  Expected Bias     : VCC/2 ~ 1.65 V (Nominal Midpoint ~ 16384)\n");
	printk("=========================================================================\n");
	printk("Sec | Rate (Hz) | Min (mV) | Max (mV) | Span (mV) | Mid (mV) | AC RMS (mV) | RMS dBFS | Clip L/H | Status\n");
	printk("----+-----------+----------+----------+-----------+----------+----------+--------------+----------+--------------------\n");

	uint32_t prev_samples = 0;
	int64_t t_start = k_uptime_get();

	for (uint32_t s = 1; s <= seconds; s++) {
		k_sleep(K_SECONDS(1));
		struct audio_diag diag;
		audio_in_get_diag(&diag);

		uint32_t sample_delta = diag.total_samples - prev_samples;
		prev_samples = diag.total_samples;

		/* Scale raw counts to millivolts */
		uint32_t min_mv = (uint32_t)((uint64_t)diag.raw_min * 3300 / MAX_RAW_CODE);
		uint32_t max_mv = (uint32_t)((uint64_t)diag.raw_max * 3300 / MAX_RAW_CODE);
		uint32_t span_mv = (max_mv > min_mv) ? (max_mv - min_mv) : 0;
		uint32_t mid_mv = (uint32_t)((uint64_t)diag.raw_midpoint * 3300 / MAX_RAW_CODE);

		const char *status_str;
		if (diag.raw_min == 0 && diag.raw_max == 0) {
			status_str = "ERR: PIN GROUNDED";
		} else if (diag.raw_min >= MAX_RAW_CODE && diag.raw_max >= MAX_RAW_CODE) {
			status_str = "ERR: PIN TIED TO 3.3V";
		} else if (span_mv < 50) {
			status_str = "SILENCE / QUIET";
		} else if (span_mv < 1500) {
			status_str = "NORMAL AUDIO/VOICE";
		} else {
			status_str = "LOUD / TAP / PEAK";
		}

		printk("%3u | %9u | %8u | %8u | %9u | %8u | %8u | %7d.%01d   | %4u/%-4u | %s\n",
		       s, sample_delta, min_mv, max_mv, span_mv, mid_mv,
		       diag.raw_rms_mv, diag.raw_rms_dbfs_x10 / 10, abs(diag.raw_rms_dbfs_x10 % 10),
		       diag.clipping_low_count, diag.clipping_high_count, status_str);
	}

	audio_in_stop();
	sink = saved_sink;

	struct audio_diag final_diag;
	audio_in_get_diag(&final_diag);
	int64_t elapsed_ms = k_uptime_get() - t_start;
	uint32_t measured_hz = (uint32_t)((uint64_t)final_diag.total_samples * 1000 / elapsed_ms);

	printk("-------------------------------------------------------------------------\n");
	printk("Capture summary:\n");
	printk("  Total duration     : %lld ms (%u s)\n", elapsed_ms, seconds);
	printk("  Total samples      : %u samples (%u blocks)\n", final_diag.total_samples, final_diag.total_blocks);
	printk("  Measured rate      : %u Hz (target %u Hz, error %d Hz)\n",
	       measured_hz, actual_rate, (int)measured_hz - (int)actual_rate);
	printk("  DC Midpoint        : %u mV (raw %u / %u)\n",
	       (uint32_t)((uint64_t)final_diag.raw_midpoint * 3300 / MAX_RAW_CODE),
	       final_diag.raw_midpoint, MAX_RAW_CODE);
	printk("  Raw AC RMS         : %u mV (%.1f dBFS RMS)\n",
	       final_diag.raw_rms_mv, (double)final_diag.raw_rms_dbfs_x10 / 10.0);
	printk("  Total clipping     : low = %u, high = %u\n",
	       final_diag.clipping_low_count, final_diag.clipping_high_count);
	printk("=========================================================================\n\n");

	return 0;
}

void audio_in_dump_registers(void)
{
	uint32_t apb_hz = 0;
	clock_control_get_rate(rcc, (clock_control_subsys_t)&tim_pclken, &apb_hz);
	uint32_t tim_hz = (LL_RCC_GetAPB1Prescaler() == LL_RCC_APB1_DIV_1) ? apb_hz : apb_hz * 2;
	uint32_t psc = LL_TIM_GetPrescaler(tim);
	uint32_t arr = LL_TIM_GetAutoReload(tim);
	uint32_t cnt = LL_TIM_GetCounter(tim);
	uint32_t trgo = (uint32_t)(tim->CR2 & TIM_CR2_MMS);
	uint32_t calc_hz = (tim_hz > 0 && (arr + 1) > 0) ? (tim_hz / ((psc + 1) * (arr + 1))) : 0;

	uint32_t adc_cr = adc->CR;
	uint32_t adc_cfgr = adc->CFGR;
	uint32_t adc_cfgr2 = adc->CFGR2;
	uint32_t dma_ccr = LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1);
	uint32_t dma_cndtr = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
	uint32_t dma_cpar = LL_DMA_GetPeriphAddress(DMA1, LL_DMA_CHANNEL_1);
	uint32_t dma_cmar = LL_DMA_GetMemoryAddress(DMA1, LL_DMA_CHANNEL_1);
	int is_tc = LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_1);
	int is_ht = LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_1);
	int is_te = LL_DMA_IsEnabledIT_TE(DMA1, LL_DMA_CHANNEL_1);
	int flag_tc = LL_DMA_IsActiveFlag_TC1(DMA1);
	int flag_ht = LL_DMA_IsActiveFlag_HT1(DMA1);

	printk("  [PERIPHERAL AUDIT] TIM6: clk=%u Hz, psc=%u, arr=%u, cnt=%u, trgo=0x%08X (rate: %u Hz)\n",
	       tim_hz, psc, arr, cnt, trgo, calc_hz);
	printk("  [PERIPHERAL AUDIT] ADC1: CR=0x%08X, CFGR=0x%08X, CFGR2=0x%08X (OVS %ux)\n",
	       adc_cr, adc_cfgr, adc_cfgr2, (1U << OVS_LOG2));
	printk("  [PERIPHERAL AUDIT] DMA1: Ch1 EN=%d, CNDTR=%u, CPAR=0x%08X, CMAR=0x%08X, TCIE=%d, HTIE=%d, TEIE=%d, TCIF=%d, HTIF=%d (HT=%u, TC=%u cb)\n",
	       dma_ccr, dma_cndtr, dma_cpar, dma_cmar, is_tc, is_ht, is_te, flag_tc, flag_ht,
	       dma_ht_callbacks, dma_tc_callbacks);
}
