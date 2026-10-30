/* SPDX-License-Identifier: Apache-2.0 */
#ifndef PSRAM_STAGING_H_
#define PSRAM_STAGING_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PSRAM_STAGING_BASE_ADDR                0x00000000U
#define PSRAM_STAGING_DEFAULT_CAPACITY_BYTES   (1024U * 1024U)  /* 1 MiB staging allocation */
#define PSRAM_STAGING_DEFAULT_BATCH_BYTES      (256U * 1024U)   /* 256 KiB SD batch write */

/**
 * @brief Explicit ownership states for PSRAM ping-pong batch buffers.
 *
 * State transitions:
 *   FREE -> FILLING (producer claims buffer)
 *   FILLING -> READY (buffer is full or flushed; published to writer)
 *   READY -> WRITING (writer consumes buffer)
 *   WRITING -> FREE (writer completes SD write successfully)
 *   WRITING -> [retained on error] (uncommitted data preserved)
 */
enum psram_batch_state {
	BATCH_STATE_FREE = 0,
	BATCH_STATE_FILLING = 1,
	BATCH_STATE_READY = 2,
	BATCH_STATE_WRITING = 3,
};

struct psram_batch_buf_info {
	uint32_t base_addr;
	uint32_t capacity_bytes;
	uint32_t filled_bytes;
	uint32_t committed_bytes;
	enum psram_batch_state state;
	uint32_t batch_id;
};

#define PSRAM_MAX_BATCH_BUFFERS 4U

struct psram_staging_stats {
	bool active;
	uint32_t capacity_bytes;
	uint32_t batch_size_bytes;
	uint32_t occupied_bytes;
	uint32_t in_flight_bytes;
	uint32_t high_water_bytes;
	uint64_t bytes_buffered;
	uint64_t bytes_flushed;
	uint64_t samples_accepted;
	uint32_t full_buffer_events;
	uint32_t dropped_samples;
};

/**
 * @brief Initialize PSRAM circular audio staging buffer.
 *
 * Verifies external PSRAM readiness, sets buffer capacity and batch write threshold.
 *
 * @param capacity_bytes Staging buffer size in bytes (e.g. 1048576 = 1 MiB).
 * @param batch_size_bytes Target batch write size for SD card (e.g. 262144 = 256 KiB).
 * @return 0 on success, negative errno if PSRAM not available or parameters invalid.
 */
int psram_staging_init(uint32_t capacity_bytes, uint32_t batch_size_bytes);

/**
 * @brief Reset buffer indices and session statistics for a new recording.
 */
void psram_staging_reset(void);

/**
 * @brief Check if PSRAM staging is active and enabled for audio recording.
 */
bool psram_staging_is_active(void);

/**
 * @brief Enable or disable PSRAM staging path.
 */
void psram_staging_set_active(bool active);

/**
 * @brief Push a processed audio block from internal SRAM into the PSRAM staging buffer.
 *
 * Single producer only. Reserves capacity before writing, but publishes bytes and
 * advances the head only after every physical PSRAM write succeeds. A concurrent
 * consumer sees committed bytes only. Handles wraparound and counts failed writes as drops.
 *
 * @param src Pointer to processed audio block in internal SRAM.
 * @param bytes Number of PCM bytes in block.
 * @param samples Number of audio samples in block.
 * @return 0 on success, -ENOBUFS if buffer full, negative errno on hardware error.
 */
int psram_staging_push(const void *src, uint32_t bytes, uint32_t samples);

/**
 * @brief Peek audio data from the PSRAM staging buffer without advancing the consumer index.
 *
 * Reads up to bytes_requested starting at the current consumer read position (tail_pos).
 * Does not advance tail_pos or decrement occupied bytes, ensuring uncommitted data
 * is preserved until the corresponding storage write succeeds. Single consumer only.
 *
 * @param dst Destination buffer in internal SRAM.
 * @param bytes_requested Maximum bytes to read.
 * @param bytes_read Actual bytes transferred from PSRAM.
 * @return 0 on success, negative errno on hardware error.
 */
int psram_staging_peek(void *dst, uint32_t bytes_requested, uint32_t *bytes_read);

/**
 * @brief Commit previously peeked audio data after a successful storage write.
 *
 * Advances the consumer index (tail_pos) by bytes_committed and decrements
 * occupied bytes.
 *
 * @param bytes_committed Number of bytes successfully written to storage.
 * @return 0 on success; -EINVAL if asked to commit more than is occupied.
 */
int psram_staging_commit(uint32_t bytes_committed);

/**
 * @brief Pop audio data from the PSRAM staging buffer into a destination SRAM buffer.
 *
 * Atomic peek and commit convenience wrapper.
 *
 * @param dst Destination buffer in internal SRAM.
 * @param bytes_requested Maximum bytes to read.
 * @param bytes_read Actual bytes transferred from PSRAM.
 * @return 0 on success, negative errno on hardware error.
 */
int psram_staging_pop(void *dst, uint32_t bytes_requested, uint32_t *bytes_read);

/**
 * @brief Query current occupied byte count in staging buffer.
 */
uint32_t psram_staging_occupied(void);

/**
 * @brief Query configured total staging buffer capacity in bytes.
 */
uint32_t psram_staging_capacity(void);

/**
 * @brief Query current available free space in bytes.
 */
uint32_t psram_staging_free_space(void);

/**
 * @brief Query configured SD batch write size.
 */
uint32_t psram_staging_batch_size(void);

/**
 * @brief Retrieve snapshot of staging metrics and high-water mark.
 */
void psram_staging_get_stats(struct psram_staging_stats *stats);

/**
 * @brief Get total number of active ping-pong / batch buffers.
 */
uint32_t psram_staging_get_buffer_count(void);

/**
 * @brief Get current ownership state and metrics for a specific batch buffer.
 *
 * @param buffer_idx Buffer index (0 or 1 for ping-pong).
 * @param info Pointer to struct to populate.
 * @return 0 on success, -EINVAL if buffer_idx out of range.
 */
int psram_staging_get_batch_info(uint32_t buffer_idx, struct psram_batch_buf_info *info);

/**
 * @brief Publish any partially filled active buffer as READY for final session draining.
 *
 * Called during recording session shutdown to transition an active FILLING buffer
 * with data into READY so the writer thread can drain it cleanly.
 *
 * @return 0 on success, or 1 if no partially filled buffer needed publishing.
 */
int psram_staging_flush_publish(void);

#ifdef __cplusplus
}
#endif

#endif /* PSRAM_STAGING_H_ */
