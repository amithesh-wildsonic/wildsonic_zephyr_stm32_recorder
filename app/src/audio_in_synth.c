/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Synthetic source: paces itself like a real ADC (one block every
 * BLOCK_SAMPLES / rate seconds) so SD throughput and buffer sizing can be
 * tested before any analog hardware is attached.
 */
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "audio_in.h"

LOG_MODULE_REGISTER(audio_synth, LOG_LEVEL_INF);

#define BLOCK CONFIG_AUDIO_BLOCK_SAMPLES
#define DIAG_BLOCK CONFIG_AUDIO_DIAGNOSTIC_BLOCK_SAMPLES
#define TABLE 1024

#if CONFIG_AUDIO_SAMPLE_BITS == 32
static int32_t sine_tab[TABLE];
#else
static int16_t sine_tab[TABLE];
#endif

static const struct audio_sink *sink;
static uint32_t rate;
static uint32_t active_block_samples = BLOCK;
static volatile bool running;
static struct k_thread thr;
static K_THREAD_STACK_DEFINE(thr_stack, 1024);

static void gen_thread(void *a, void *b, void *c)
{
	uint32_t phase = 0;
	const uint32_t step = (uint32_t)(((uint64_t)CONFIG_AUDIO_SYNTH_TONE_HZ << 32) / rate);
	const int64_t t0 = k_uptime_ticks();
	uint64_t n = 0;

	while (running) {
		void *raw_blk = sink->alloc(sink->user);

		if (raw_blk) {
#if CONFIG_AUDIO_SAMPLE_BITS == 32
			int32_t *blk = (int32_t *)raw_blk;
#else
			int16_t *blk = (int16_t *)raw_blk;
#endif
			for (uint32_t i = 0; i < active_block_samples; i++) {
				blk[i] = sine_tab[phase >> 22];	/* top 10 bits */
				phase += step;
			}
			sink->submit(raw_blk, sink->user);
		} else {
			phase += step * active_block_samples;
		}
		n++;
		/* absolute schedule computed from the block count: no drift */
		int64_t due = t0 + k_us_to_ticks_near64((n * active_block_samples * 1000000ULL) / rate);

		k_sleep(K_TIMEOUT_ABS_TICKS(due));
	}
}

int audio_in_init(uint32_t requested_rate_hz, const struct audio_sink *s)
{
	sink = s;
	rate = requested_rate_hz;
	for (int i = 0; i < TABLE; i++) {
#if CONFIG_AUDIO_SAMPLE_BITS == 32
		sine_tab[i] = (int32_t)(1000000000.0f * sinf(2.0f * 3.14159265f * i / TABLE));
#else
		sine_tab[i] = (int16_t)(16000.0f * sinf(2.0f * 3.14159265f * i / TABLE));
#endif
	}
	LOG_INF("synthetic source: %d Hz tone, %u Hz (%u-bit PCM)",
		CONFIG_AUDIO_SYNTH_TONE_HZ, rate, CONFIG_AUDIO_SAMPLE_BITS);
	return 0;
}

uint32_t audio_in_rate(void)
{
	return rate;
}

bool audio_in_is_rate_supported(uint32_t rate_hz, uint32_t *act_rate, uint32_t *divider, int *error_hz)
{
	if (rate_hz == 16000 || rate_hz == 32000 || rate_hz == 44100 ||
	    rate_hz == 48000 || rate_hz == 96000 || rate_hz == 192000) {
		if (act_rate) *act_rate = rate_hz;
		if (divider) *divider = 1;
		if (error_hz) *error_hz = 0;
		return true;
	}
	return false;
}

int audio_in_set_rate(uint32_t new_rate_hz)
{
	if (running) {
		return -EBUSY;
	}
	if (!audio_in_is_rate_supported(new_rate_hz, NULL, NULL, NULL)) {
		return -EINVAL;
	}
	rate = new_rate_hz;
	return 0;
}

int audio_in_set_block_size(uint32_t samples)
{
	if (running) {
		return -EBUSY;
	}
	if (samples != BLOCK && samples != DIAG_BLOCK) {
		return -EINVAL;
	}
	active_block_samples = samples;
	return 0;
}

#if CONFIG_AUDIO_INTEGRITY_REPLAY
int audio_in_set_integrity_replay(bool enabled)
{
	ARG_UNUSED(enabled);
	return -ENOTSUP;
}
#endif

static volatile uint32_t synth_total_samples;
static volatile uint32_t synth_total_blocks;

int audio_in_start(void)
{
	running = true;
	k_thread_create(&thr, thr_stack, K_THREAD_STACK_SIZEOF(thr_stack), gen_thread,
			NULL, NULL, NULL, K_PRIO_COOP(2), 0, K_NO_WAIT);
	return 0;
}

int audio_in_stop(void)
{
	running = false;
	k_thread_join(&thr, K_SECONDS(1));
	return 0;
}

void audio_in_reset_diag(void)
{
	synth_total_samples = 0;
	synth_total_blocks = 0;
}

void audio_in_get_diag(struct audio_diag *diag)
{
	diag->total_samples = synth_total_samples;
	diag->total_blocks = synth_total_blocks;
	diag->raw_min = 16384 - 8000;
	diag->raw_max = 16384 + 8000;
	diag->raw_midpoint = 16384;
	diag->clipping_low_count = 0;
	diag->clipping_high_count = 0;
	diag->peak_to_peak = 16000;
	diag->actual_rate = rate;
	diag->raw_rms_code = 0;
	diag->raw_ac_rms_code = 0;
	diag->raw_rms_mv = 0;
	diag->raw_rms_dbfs_x10 = -960;
}

int audio_in_run_diagnostic(uint32_t seconds)
{
	printk("Synthetic diagnostic mode running for %u seconds at %u Hz...\n", seconds, rate);
	audio_in_start();
	for (uint32_t i = 1; i <= seconds; i++) {
		k_sleep(K_SECONDS(1));
		printk("  [t=%u s] Generating %d Hz sine tone at %u Hz\n",
		       i, CONFIG_AUDIO_SYNTH_TONE_HZ, rate);
	}
	audio_in_stop();
	return 0;
}

void audio_in_dump_registers(void)
{
	printk("  [SYNTHETIC SOURCE] Clock & rate: %u Hz, synthetic generator thread\n", rate);
}

void audio_in_get_callback_counts(uint32_t *ht_count, uint32_t *tc_count)
{
	if (ht_count) *ht_count = 0;
	if (tc_count) *tc_count = 0;
}

void audio_in_get_dma_stats(struct audio_dma_stats *stats)
{
	if (stats) *stats = (struct audio_dma_stats){0};
}

uint32_t audio_in_get_missed_event_count(void)
{
	return 0;
}
