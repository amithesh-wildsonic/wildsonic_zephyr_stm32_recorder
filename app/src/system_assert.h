/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SYSTEM_ASSERT_H_
#define SYSTEM_ASSERT_H_

#include <stdint.h>
#include <stdbool.h>
#include "retained_reboot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 32-bit Assert IDs grouped by subsystem:
 * High 8 bits:
 * 0x01: MAIN
 * 0x02: AUDIO_IN / DSP
 * 0x03: PSRAM
 * 0x04: RECORDER / STORAGE
 * 0x05: RTC / SLEEP
 * 0x06: CONFIG_STORE
 * 0x07: SYSTEM / MISC
 */

/* Module 0x01: Main / System Lifecycle */
#define ASSERT_ID_MAIN_INIT_FAILED           0x01000001U
#define ASSERT_ID_MAIN_THREAD_FAIL           0x01000002U
#define ASSERT_ID_MAIN_CLOCK_INVALID         0x01000003U
#define ASSERT_ID_MAIN_NULL_INTERVAL         0x01000004U
#define ASSERT_ID_MAIN_CONSOLE_DEV_NULL      0x01000005U
#define ASSERT_ID_MAIN_BLE_DEV_NULL          0x01000006U

/* Module 0x02: Audio Acquisition & DSP */
#define ASSERT_ID_AUDIO_RATE_UNSUPPORTED     0x02000001U
#define ASSERT_ID_AUDIO_BUF_UNALIGNED        0x02000002U
#define ASSERT_ID_AUDIO_DMA_DEV_NULL         0x02000003U
#define ASSERT_ID_AUDIO_TIM_DEV_NULL         0x02000004U
#define ASSERT_ID_AUDIO_BLOCK_SIZE_ZERO      0x02000005U
#define ASSERT_ID_AUDIO_INVALID_CHANNEL      0x02000006U
#define ASSERT_ID_AUDIO_SAMPLE_BITS_INV      0x02000007U
#define ASSERT_ID_AUDIO_OVERRUN_CORRUPT      0x02000008U
#define ASSERT_ID_DSP_COEFF_INVALID          0x02000009U
#define ASSERT_ID_DSP_STATE_CORRUPT          0x0200000AU

/* Module 0x03: PSRAM IS66WVS & Staging Ring */
#define ASSERT_ID_PSRAM_DEV_NOT_READY        0x03000001U
#define ASSERT_ID_PSRAM_CAPACITY_ZERO        0x03000002U
#define ASSERT_ID_PSRAM_SLOT_OUT_OF_BOUND    0x03000003U
#define ASSERT_ID_PSRAM_RING_CORRUPT         0x03000004U
#define ASSERT_ID_PSRAM_STAGING_OVERFLOW     0x03000005U
#define ASSERT_ID_PSRAM_MODE_INVALID         0x03000006U
#define ASSERT_ID_PSRAM_NULL_READ_BUF        0x03000007U

/* Module 0x04: Storage / SD / FatFs / Writer */
#define ASSERT_ID_SD_MOUNT_FAILED            0x04000001U
#define ASSERT_ID_SD_SECTOR_SIZE_INVALID     0x04000002U
#define ASSERT_ID_SD_CLUSTER_SIZE_INVALID    0x04000003U
#define ASSERT_ID_SD_WAV_HEADER_CORRUPT      0x04000004U
#define ASSERT_ID_SD_BLOCK_ID_INVALID        0x04000005U
#define ASSERT_ID_SD_PINGPONG_SLOT_INV       0x04000006U
#define ASSERT_ID_SD_WRITE_CHUNK_OVERSIZED   0x04000007U
#define ASSERT_ID_SD_FILE_PTR_NULL           0x04000008U
#define ASSERT_ID_SD_ZERO_SAMPLES_WRITE      0x04000009U
#define ASSERT_ID_SD_BOUNCE_BUF_NULL         0x0400000AU
#define ASSERT_ID_SD_RETRY_COUNT_EXCEEDED    0x0400000BU

/* Module 0x05: RTC Calendar & Sleep Standby */
#define ASSERT_ID_RTC_CALENDAR_UNINIT        0x05000001U
#define ASSERT_ID_RTC_INVALID_MONTH          0x05000002U
#define ASSERT_ID_RTC_INVALID_DAY            0x05000003U
#define ASSERT_ID_RTC_INVALID_HOUR           0x05000004U
#define ASSERT_ID_RTC_INVALID_MINUTE         0x05000005U
#define ASSERT_ID_RTC_INVALID_SECOND         0x05000006U
#define ASSERT_ID_RTC_WUT_PERIOD_INV         0x05000007U
#define ASSERT_ID_RTC_SLEEP_NEGATIVE         0x05000008U

/* Module 0x06: Persistent Config Store */
#define ASSERT_ID_CFG_STORAGE_OPEN_FAIL      0x06000001U
#define ASSERT_ID_CFG_MAGIC_MISMATCH         0x06000002U
#define ASSERT_ID_CFG_CRC_MISMATCH           0x06000003U
#define ASSERT_ID_CFG_RATE_UNSUPPORTED       0x06000004U
#define ASSERT_ID_CFG_CHANNELS_INVALID       0x06000005U
#define ASSERT_ID_CFG_BITS_INVALID           0x06000006U

/* Module 0x07: System Telemetry & Console */
#define ASSERT_ID_BATT_VOLTAGE_OUT_RANGE     0x07000001U
#define ASSERT_ID_CONS_RX_RB_NULL            0x07000002U
#define ASSERT_ID_SCHED_INTERVAL_INV         0x07000003U
#define ASSERT_ID_GENERIC_SAFETY_FAULT       0x07000004U
#define ASSERT_ID_WDT_RELOAD_FAILED          0x07000005U

void system_assert_record_id(uint32_t id, const char *file, uint32_t line, const char *expr);

#define SYSTEM_ASSERT(id, cond) do { \
	if (!(cond)) { \
		system_assert_record_id((id), __FILE__, __LINE__, #cond); \
	} \
} while (0)

#define SYSTEM_ASSERT_MSG(id, cond, msg) do { \
	if (!(cond)) { \
		system_assert_record_id((id), __FILE__, __LINE__, msg); \
	} \
} while (0)

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_ASSERT_H_ */
