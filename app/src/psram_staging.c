/* SPDX-License-Identifier: Apache-2.0 */
#include "psram_staging.h"
#include "psram_is66wvs.h"
#include <string.h>
#include <errno.h>

#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(psram_staging, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(psram_staging, LOG_LEVEL_WRN);
#endif
#else
#define LOG_INF(...) do {} while (0)
#define LOG_ERR(...) do {} while (0)
#define LOG_WRN(...) do {} while (0)
#endif

static bool staging_active = false;
static uint32_t buf_capacity = PSRAM_STAGING_DEFAULT_CAPACITY_BYTES;
static uint32_t batch_size = PSRAM_STAGING_DEFAULT_BATCH_BYTES;

/* Explicit ping-pong / multi-batch buffer architecture */
struct psram_batch_buf {
	uint32_t base_addr;
	uint32_t capacity_bytes;
	uint32_t filled_bytes;
	uint32_t committed_bytes;
	enum psram_batch_state state;
	uint32_t batch_id;
};

static uint32_t num_batch_buffers = 2U;
static struct psram_batch_buf batch_bufs[PSRAM_MAX_BATCH_BUFFERS];
static uint32_t active_fill_idx = 0;
static uint32_t active_drain_idx = 0;
static uint32_t batch_seq_counter = 0;

static uint32_t head_pos = 0;
static uint32_t tail_pos = 0;
static uint32_t count_occupied = 0;
/* One producer reservation may be writing outside the metadata spinlock. */
static uint32_t reserved_bytes = 0;
static bool write_in_flight = false;
static uint32_t peak_occupancy = 0;

static uint64_t total_buffered = 0;
static uint64_t total_flushed = 0;
static uint64_t total_samples = 0;
static uint32_t full_events = 0;
static uint32_t drops = 0;

#if defined(__ZEPHYR__)
static struct k_spinlock staging_lock;
#define STAGING_LOCK()   k_spinlock_key_t key = k_spin_lock(&staging_lock)
#define STAGING_UNLOCK() k_spin_unlock(&staging_lock, key)
#else
#define STAGING_LOCK()   do {} while (0)
#define STAGING_UNLOCK() do {} while (0)
#endif

int psram_staging_init(uint32_t capacity_bytes, uint32_t batch_size_bytes)
{
	if (!psram_is_present() || !psram_is_integrity_verified()) {
		STAGING_LOCK();
		staging_active = false;
		STAGING_UNLOCK();
		LOG_WRN("PSRAM staging init rejected: memory integrity not verified");
		return -ENODEV;
	}

	if (capacity_bytes == 0 || capacity_bytes > PSRAM_IS66WVS_CAPACITY_BYTES) {
		capacity_bytes = PSRAM_STAGING_DEFAULT_CAPACITY_BYTES;
	}
	if (batch_size_bytes == 0 || batch_size_bytes > capacity_bytes) {
		batch_size_bytes = PSRAM_STAGING_DEFAULT_BATCH_BYTES;
	}

	/* Enforce 512-byte block alignment for SD compatibility */
	if ((batch_size_bytes % 512U) != 0) {
		batch_size_bytes = (batch_size_bytes / 512U) * 512U;
		if (batch_size_bytes == 0) {
			batch_size_bytes = 512U;
		}
	}

	STAGING_LOCK();
	if (write_in_flight) {
		STAGING_UNLOCK();
		return -EBUSY;
	}

	uint32_t n_bufs = capacity_bytes / batch_size_bytes;
	if (n_bufs < 2U) {
		n_bufs = 2U;
	}
	if (n_bufs > PSRAM_MAX_BATCH_BUFFERS) {
		n_bufs = PSRAM_MAX_BATCH_BUFFERS;
	}
	num_batch_buffers = n_bufs;
	batch_size = batch_size_bytes;
	buf_capacity = num_batch_buffers * batch_size;

	for (uint32_t i = 0; i < num_batch_buffers; i++) {
		batch_bufs[i].base_addr = PSRAM_STAGING_BASE_ADDR + i * batch_size;
		batch_bufs[i].capacity_bytes = batch_size;
		batch_bufs[i].filled_bytes = 0;
		batch_bufs[i].committed_bytes = 0;
		batch_bufs[i].state = BATCH_STATE_FREE;
		batch_bufs[i].batch_id = 0;
	}

	active_fill_idx = 0;
	active_drain_idx = 0;
	batch_seq_counter = 0;

	head_pos = 0;
	tail_pos = 0;
	count_occupied = 0;
	reserved_bytes = 0;
	peak_occupancy = 0;
	total_buffered = 0;
	total_flushed = 0;
	total_samples = 0;
	full_events = 0;
	drops = 0;

	batch_bufs[0].state = BATCH_STATE_FILLING;
	batch_bufs[0].batch_id = ++batch_seq_counter;
	staging_active = true;
	STAGING_UNLOCK();

	LOG_INF("PSRAM ping-pong batch buffer initialized: %u buffers x %u KB = %u KB total",
	        num_batch_buffers, batch_size >> 10, buf_capacity >> 10);
	return 0;
}

void psram_staging_reset(void)
{
	STAGING_LOCK();
	/* Lifecycle contract: reset only with producer and consumer stopped. */
	if (write_in_flight) {
		STAGING_UNLOCK();
		return;
	}
	for (uint32_t i = 0; i < num_batch_buffers; i++) {
		batch_bufs[i].state = BATCH_STATE_FREE;
		batch_bufs[i].filled_bytes = 0;
		batch_bufs[i].committed_bytes = 0;
		batch_bufs[i].batch_id = 0;
	}
	active_fill_idx = 0;
	active_drain_idx = 0;
	batch_seq_counter = 0;
	head_pos = 0;
	tail_pos = 0;
	count_occupied = 0;
	reserved_bytes = 0;
	peak_occupancy = 0;
	total_buffered = 0;
	total_flushed = 0;
	total_samples = 0;
	full_events = 0;
	drops = 0;

	batch_bufs[0].state = BATCH_STATE_FILLING;
	batch_bufs[0].batch_id = ++batch_seq_counter;
	STAGING_UNLOCK();
}

bool psram_staging_is_active(void)
{
	STAGING_LOCK();
	bool active = staging_active;
	STAGING_UNLOCK();
	return active;
}

void psram_staging_set_active(bool active)
{
	STAGING_LOCK();
	staging_active = active;
	STAGING_UNLOCK();
}

uint32_t psram_staging_occupied(void)
{
	STAGING_LOCK();
	uint32_t occ = count_occupied;
	STAGING_UNLOCK();
	return occ;
}

uint32_t psram_staging_capacity(void)
{
	return buf_capacity;
}

uint32_t psram_staging_free_space(void)
{
	STAGING_LOCK();
	uint32_t used = count_occupied + reserved_bytes;
	uint32_t free_b = (buf_capacity > used) ? (buf_capacity - used) : 0;
	STAGING_UNLOCK();
	return free_b;
}

uint32_t psram_staging_batch_size(void)
{
	return batch_size;
}

int psram_staging_push(const void *src, uint32_t bytes, uint32_t samples)
{
	if (!src || bytes == 0) {
		return 0;
	}
	uint32_t write_offset;
	{
	STAGING_LOCK();
	if (!staging_active) {
		STAGING_UNLOCK();
		return -ENODEV;
	}
	if (write_in_flight) {
		/* Enforce exactly one producer without blocking while QSPI is active. */
		drops += samples;
		STAGING_UNLOCK();
		return -EBUSY;
	}

	struct psram_batch_buf *cur = &batch_bufs[active_fill_idx];
	if (cur->state == BATCH_STATE_FREE) {
		cur->state = BATCH_STATE_FILLING;
		cur->filled_bytes = 0;
		cur->committed_bytes = 0;
		cur->batch_id = ++batch_seq_counter;
	} else if (cur->state != BATCH_STATE_FILLING) {
		/* Both ping-pong buffers occupied (READY or WRITING); producer cannot overwrite */
		full_events++;
		drops += samples;
		STAGING_UNLOCK();
		LOG_WRN("PSRAM staging buffer full! Dropped %u samples (%u bytes overflow)", samples, bytes);
		return -ENOBUFS;
	}

	uint32_t used = count_occupied + reserved_bytes;
	uint32_t space_in_cur = (cur->capacity_bytes > cur->filled_bytes) ?
	                        (cur->capacity_bytes - cur->filled_bytes) : 0;
	uint32_t available = (buf_capacity > used) ? (buf_capacity - used) : 0;
	if (bytes > space_in_cur || bytes > available) {
		full_events++;
		drops += samples;
		STAGING_UNLOCK();
		LOG_WRN("PSRAM staging buffer full! Dropped %u samples (%u bytes overflow)", samples, bytes);
		return -ENOBUFS;
	}

	write_offset = cur->base_addr + cur->filled_bytes;
	reserved_bytes = bytes;
	write_in_flight = true;
	if (used + bytes > peak_occupancy) {
		peak_occupancy = used + bytes;
	}
	STAGING_UNLOCK();
	}

	/* Perform physical transfer to external PSRAM outside the critical section */
	int ret = psram_write(write_offset, (const uint8_t *)src, bytes);

	if (ret != 0) {
		LOG_ERR("PSRAM physical write failed at addr 0x%06X (code %d)",
		        write_offset, ret);
		STAGING_LOCK();
		reserved_bytes = 0;
		write_in_flight = false;
		drops += samples;
		STAGING_UNLOCK();
		return ret;
	}

	/* Publish only after physical PSRAM write completed successfully */
	STAGING_LOCK();
	struct psram_batch_buf *cur = &batch_bufs[active_fill_idx];
	cur->filled_bytes += bytes;
	head_pos = (write_offset + bytes) % buf_capacity;
	count_occupied += bytes;
	reserved_bytes = 0;
	write_in_flight = false;
	total_buffered += bytes;
	total_samples += samples;

	/* If the current batch buffer has filled its capacity, publish it as READY */
	if (cur->filled_bytes >= cur->capacity_bytes) {
		cur->state = BATCH_STATE_READY;
		uint32_t next_fill = (active_fill_idx + 1U) % num_batch_buffers;
		active_fill_idx = next_fill;
		if (batch_bufs[active_fill_idx].state == BATCH_STATE_FREE) {
			batch_bufs[active_fill_idx].state = BATCH_STATE_FILLING;
			batch_bufs[active_fill_idx].filled_bytes = 0;
			batch_bufs[active_fill_idx].committed_bytes = 0;
			batch_bufs[active_fill_idx].batch_id = ++batch_seq_counter;
		}
	}
	STAGING_UNLOCK();

	return 0;
}

int psram_staging_peek(void *dst, uint32_t bytes_requested, uint32_t *bytes_read)
{
	if (!dst || bytes_requested == 0) {
		if (bytes_read) *bytes_read = 0;
		return 0;
	}
	if (!staging_active) {
		if (bytes_read) *bytes_read = 0;
		return -ENODEV;
	}

	uint32_t read_offset = 0;
	uint32_t to_read = 0;

	STAGING_LOCK();
	if (count_occupied == 0) {
		STAGING_UNLOCK();
		if (bytes_read) *bytes_read = 0;
		return 0;
	}

	struct psram_batch_buf *drain = &batch_bufs[active_drain_idx];
	if (drain->state == BATCH_STATE_READY) {
		drain->state = BATCH_STATE_WRITING;
	} else if (drain->state != BATCH_STATE_WRITING) {
		/* If flush requested or session end, active fill buffer might have data */
		if (drain->state == BATCH_STATE_FILLING && drain->filled_bytes > 0) {
			drain->state = BATCH_STATE_WRITING;
		} else {
			STAGING_UNLOCK();
			if (bytes_read) *bytes_read = 0;
			return 0;
		}
	}

	uint32_t uncommitted = (drain->filled_bytes > drain->committed_bytes) ?
	                       (drain->filled_bytes - drain->committed_bytes) : 0;
	if (uncommitted == 0) {
		STAGING_UNLOCK();
		if (bytes_read) *bytes_read = 0;
		return 0;
	}

	to_read = (bytes_requested < uncommitted) ? bytes_requested : uncommitted;
	read_offset = drain->base_addr + drain->committed_bytes;
	tail_pos = read_offset % buf_capacity;
	STAGING_UNLOCK();

	int ret = psram_read(read_offset, (uint8_t *)dst, to_read);
	if (ret != 0) {
		LOG_ERR("PSRAM physical peek failed at addr 0x%06X (code %d)",
		        read_offset, ret);
		if (bytes_read) *bytes_read = 0;
		return ret;
	}

	if (bytes_read) {
		*bytes_read = to_read;
	}
	return 0;
}

int psram_staging_commit(uint32_t bytes_committed)
{
	if (bytes_committed == 0) {
		return 0;
	}

	STAGING_LOCK();
	struct psram_batch_buf *drain = &batch_bufs[active_drain_idx];
	if (drain->state != BATCH_STATE_WRITING && drain->state != BATCH_STATE_FILLING) {
		STAGING_UNLOCK();
		return -EINVAL;
	}
	if (bytes_committed > count_occupied) {
		STAGING_UNLOCK();
		return -EINVAL;
	}

	drain->committed_bytes += bytes_committed;
	tail_pos = (tail_pos + bytes_committed) % buf_capacity;
	count_occupied -= bytes_committed;
	total_flushed += bytes_committed;

	/* When entire batch buffer has been committed to SD, release to FREE */
	if (drain->committed_bytes >= drain->filled_bytes) {
		if (drain->state == BATCH_STATE_WRITING || active_drain_idx != active_fill_idx) {
			drain->state = BATCH_STATE_FREE;
			drain->filled_bytes = 0;
			drain->committed_bytes = 0;
		}
		active_drain_idx = (active_drain_idx + 1U) % num_batch_buffers;
	}
	STAGING_UNLOCK();
	return 0;
}

int psram_staging_pop(void *dst, uint32_t bytes_requested, uint32_t *bytes_read)
{
	int ret = psram_staging_peek(dst, bytes_requested, bytes_read);
	if (ret == 0 && bytes_read && *bytes_read > 0) {
		psram_staging_commit(*bytes_read);
	}
	return ret;
}

int psram_staging_flush_publish(void)
{
	STAGING_LOCK();
	struct psram_batch_buf *cur = &batch_bufs[active_fill_idx];
	if (cur->state == BATCH_STATE_FILLING && cur->filled_bytes > 0) {
		cur->state = BATCH_STATE_READY;
		STAGING_UNLOCK();
		return 0;
	}
	STAGING_UNLOCK();
	return 1;
}

uint32_t psram_staging_get_buffer_count(void)
{
	STAGING_LOCK();
	uint32_t cnt = num_batch_buffers;
	STAGING_UNLOCK();
	return cnt;
}

int psram_staging_get_batch_info(uint32_t buffer_idx, struct psram_batch_buf_info *info)
{
	if (!info || buffer_idx >= num_batch_buffers) {
		return -EINVAL;
	}
	STAGING_LOCK();
	info->base_addr = batch_bufs[buffer_idx].base_addr;
	info->capacity_bytes = batch_bufs[buffer_idx].capacity_bytes;
	info->filled_bytes = batch_bufs[buffer_idx].filled_bytes;
	info->committed_bytes = batch_bufs[buffer_idx].committed_bytes;
	info->state = batch_bufs[buffer_idx].state;
	info->batch_id = batch_bufs[buffer_idx].batch_id;
	STAGING_UNLOCK();
	return 0;
}

void psram_staging_get_stats(struct psram_staging_stats *stats)
{
	if (!stats) return;

	STAGING_LOCK();
	stats->active = staging_active;
	stats->capacity_bytes = buf_capacity;
	stats->batch_size_bytes = batch_size;
	stats->occupied_bytes = count_occupied;
	stats->in_flight_bytes = reserved_bytes;
	stats->high_water_bytes = peak_occupancy;
	stats->bytes_buffered = total_buffered;
	stats->bytes_flushed = total_flushed;
	stats->samples_accepted = total_samples;
	stats->full_buffer_events = full_events;
	stats->dropped_samples = drops;
	STAGING_UNLOCK();
}
