/* SPDX-License-Identifier: Apache-2.0 */
#include "psram_staging.h"
#include "psram_is66wvs.h"
#include <string.h>
#include <errno.h>

#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(psram_staging, LOG_LEVEL_INF);
#else
#define LOG_INF(...) do {} while (0)
#define LOG_ERR(...) do {} while (0)
#define LOG_WRN(...) do {} while (0)
#endif

static bool staging_active = false;
static uint32_t buf_capacity = PSRAM_STAGING_DEFAULT_CAPACITY_BYTES;
static uint32_t batch_size = PSRAM_STAGING_DEFAULT_BATCH_BYTES;

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
	buf_capacity = capacity_bytes;
	batch_size = batch_size_bytes;
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
	staging_active = true;
	STAGING_UNLOCK();
	LOG_INF("PSRAM audio staging buffer initialized: capacity %u bytes (%.2f MB), batch %u bytes (%.2f KB)",
	        buf_capacity, (double)buf_capacity / (1024.0 * 1024.0),
	        batch_size, (double)batch_size / 1024.0);
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
	uint32_t used = count_occupied + reserved_bytes;
	uint32_t available = (buf_capacity > used) ? (buf_capacity - used) : 0;
	if (bytes > available) {
		full_events++;
		drops += samples;
		STAGING_UNLOCK();
		LOG_WRN("PSRAM staging buffer full! Dropped %u samples (%u bytes overflow)", samples, bytes);
		return -ENOBUFS;
	}

	write_offset = head_pos;
	reserved_bytes = bytes;
	write_in_flight = true;
	if (used + bytes > peak_occupancy) {
		peak_occupancy = used + bytes;
	}
	STAGING_UNLOCK();
	}

	/*
	 * Perform physical transfer to external PSRAM outside the critical section.
	 * Check whether the write wraps around the end of the circular buffer.
	 */
	int ret = 0;
	if (write_offset + bytes <= buf_capacity) {
		ret = psram_write(PSRAM_STAGING_BASE_ADDR + write_offset, (const uint8_t *)src, bytes);
	} else {
		uint32_t part1 = buf_capacity - write_offset;
		uint32_t part2 = bytes - part1;
		ret = psram_write(PSRAM_STAGING_BASE_ADDR + write_offset, (const uint8_t *)src, part1);
		if (ret == 0) {
			ret = psram_write(PSRAM_STAGING_BASE_ADDR, (const uint8_t *)src + part1, part2);
		}
	}

	if (ret != 0) {
		LOG_ERR("PSRAM physical write failed at addr 0x%06X (code %d)",
		        PSRAM_STAGING_BASE_ADDR + write_offset, ret);
		/* Do not publish or consume ring space after an unsuccessful write. */
		{
		STAGING_LOCK();
		reserved_bytes = 0;
		write_in_flight = false;
		drops += samples;
		STAGING_UNLOCK();
		}
		return ret;
	}

	/* Publish only after all wrap segments completed successfully. */
	STAGING_LOCK();
	head_pos = (write_offset + bytes) % buf_capacity;
	count_occupied += bytes;
	reserved_bytes = 0;
	write_in_flight = false;
	total_buffered += bytes;
	total_samples += samples;
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

	to_read = (bytes_requested < count_occupied) ? bytes_requested : count_occupied;
	read_offset = tail_pos;
	STAGING_UNLOCK();

	/*
	 * Perform physical transfer from external PSRAM.
	 * Check whether the read wraps around the end of the circular buffer.
	 * Note: tail_pos is NOT advanced here. It is committed only after successful SD write!
	 */
	int ret = 0;
	if (read_offset + to_read <= buf_capacity) {
		ret = psram_read(PSRAM_STAGING_BASE_ADDR + read_offset, (uint8_t *)dst, to_read);
	} else {
		uint32_t part1 = buf_capacity - read_offset;
		uint32_t part2 = to_read - part1;
		ret = psram_read(PSRAM_STAGING_BASE_ADDR + read_offset, (uint8_t *)dst, part1);
		if (ret == 0) {
			ret = psram_read(PSRAM_STAGING_BASE_ADDR, (uint8_t *)dst + part1, part2);
		}
	}

	if (ret != 0) {
		LOG_ERR("PSRAM physical peek failed at addr 0x%06X (code %d)",
		        PSRAM_STAGING_BASE_ADDR + read_offset, ret);
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
	if (bytes_committed > count_occupied) {
		STAGING_UNLOCK();
		return -EINVAL;
	}
	tail_pos = (tail_pos + bytes_committed) % buf_capacity;
	count_occupied -= bytes_committed;
	total_flushed += bytes_committed;
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
