/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SCHEDULER_RETENTION_H_
#define SCHEDULER_RETENTION_H_

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#define SCHEDULER_RETENTION_MAGIC       0x57534348U /* 'WSCH' */
#define SCHEDULER_RETENTION_VERSION     0x00010001U /* v1.1 */
#define SCHEDULER_RETENTION_XOR_CONST   0xA5A55A5AU

#define BLE_PROV_RETENTION_MAGIC        0x5753424CU /* 'WSBL' */
#define BLE_PROV_RETENTION_VERSION      0x00010001U /* v1.1 */

/**
 * @brief Initialize RTC backup register retention access.
 */
void scheduler_retention_init(void);

/**
 * @brief Save last successful session end UTC timestamp to RTC backup registers.
 *
 * Persists 64-bit UTC timestamp split across BKP registers with magic, version,
 * and XOR checksum.
 *
 * @param end_utc Session end timestamp in UTC epoch seconds.
 * @return 0 on success, negative errno on failure.
 */
int scheduler_retention_save(time_t end_utc);

/**
 * @brief Restore last successful session end UTC timestamp from RTC backup registers.
 *
 * Validates magic, version, and checksum.
 *
 * @param end_utc Output pointer for restored UTC timestamp.
 * @return 0 on success, -EINVAL if no record or invalid, -EBADMSG on checksum failure.
 */
int scheduler_retention_restore(time_t *end_utc);

/**
 * @brief Invalidate retained session timestamp in RTC backup registers.
 */
void scheduler_retention_invalidate(void);

/**
 * @brief Save BLE provisioning fingerprint to RTC backup registers.
 *
 * Persists 32-bit fingerprint across host reboots with magic, version, and checksum.
 *
 * @param fingerprint Computed 32-bit CRC/hash of BLE settings.
 * @return 0 on success, negative errno on failure.
 */
int ble_retention_save_fingerprint(uint32_t fingerprint);

/**
 * @brief Restore BLE provisioning fingerprint from RTC backup registers.
 *
 * Validates magic, version, and checksum.
 *
 * @param fingerprint Output pointer for restored fingerprint.
 * @return 0 on success, -EINVAL if no record or invalid, -EBADMSG on checksum failure.
 */
int ble_retention_restore_fingerprint(uint32_t *fingerprint);

/**
 * @brief Invalidate retained BLE provisioning fingerprint in RTC backup registers.
 */
void ble_retention_invalidate_fingerprint(void);

#if !defined(CONFIG_SOC_SERIES_STM32L4X)
/**
 * @brief Test hook to inspect or inject mock backup registers for host testing.
 */
uint32_t scheduler_retention_mock_get(int reg_idx);
void scheduler_retention_mock_set(int reg_idx, uint32_t val);
#endif

#endif /* SCHEDULER_RETENTION_H_ */

