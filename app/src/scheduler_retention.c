/* SPDX-License-Identifier: Apache-2.0 */
#include "scheduler_retention.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(sched_ret, CONFIG_LOG_DEFAULT_LEVEL);

#if defined(CONFIG_SOC_SERIES_STM32L4X)
#include <stm32l4xx_ll_pwr.h>
#include <stm32l4xx_ll_rtc.h>

static void enable_backup_write(void)
{
	LL_PWR_EnableBkUpAccess();
}

static uint32_t bkp_read(uint32_t bkp_reg)
{
	return LL_RTC_BAK_GetRegister(RTC, bkp_reg);
}

static void bkp_write(uint32_t bkp_reg, uint32_t data)
{
	enable_backup_write();
	LL_RTC_BAK_SetRegister(RTC, bkp_reg, data);
}

#define REG_MAGIC   LL_RTC_BKP_DR0
#define REG_VER     LL_RTC_BKP_DR1
#define REG_TS_LOW  LL_RTC_BKP_DR2
#define REG_TS_HIGH LL_RTC_BKP_DR3
#define REG_CKSUM   LL_RTC_BKP_DR4

#define REG_BLE_MAGIC   LL_RTC_BKP_DR5
#define REG_BLE_VER     LL_RTC_BKP_DR6
#define REG_BLE_FP      LL_RTC_BKP_DR7
#define REG_BLE_CKSUM   LL_RTC_BKP_DR8

#else

/* Host unit-test mock registers */
static uint32_t s_mock_bkp[16] = {0};

uint32_t scheduler_retention_mock_get(int reg_idx)
{
	if (reg_idx >= 0 && reg_idx < 16) {
		return s_mock_bkp[reg_idx];
	}
	return 0;
}

void scheduler_retention_mock_set(int reg_idx, uint32_t val)
{
	if (reg_idx >= 0 && reg_idx < 16) {
		s_mock_bkp[reg_idx] = val;
	}
}

static uint32_t bkp_read(uint32_t bkp_reg)
{
	if (bkp_reg < 16) {
		return s_mock_bkp[bkp_reg];
	}
	return 0;
}

static void bkp_write(uint32_t bkp_reg, uint32_t data)
{
	if (bkp_reg < 16) {
		s_mock_bkp[bkp_reg] = data;
	}
}

#define REG_MAGIC   0
#define REG_VER     1
#define REG_TS_LOW  2
#define REG_TS_HIGH 3
#define REG_CKSUM   4

#define REG_BLE_MAGIC   5
#define REG_BLE_VER     6
#define REG_BLE_FP      7
#define REG_BLE_CKSUM   8

#endif

static inline uint32_t calc_checksum(uint32_t magic, uint32_t ver, uint32_t low, uint32_t high)
{
	return magic ^ ver ^ low ^ high ^ SCHEDULER_RETENTION_XOR_CONST;
}

void scheduler_retention_init(void)
{
#if defined(CONFIG_SOC_SERIES_STM32L4X)
	enable_backup_write();
#endif
}

int scheduler_retention_save(time_t end_utc)
{
	if (end_utc <= 0) {
		return -EINVAL;
	}

	uint32_t magic = SCHEDULER_RETENTION_MAGIC;
	uint32_t ver = SCHEDULER_RETENTION_VERSION;
	uint32_t low = (uint32_t)((uint64_t)end_utc & 0xFFFFFFFFU);
	uint32_t high = (uint32_t)(((uint64_t)end_utc >> 32) & 0xFFFFFFFFU);
	uint32_t cksum = calc_checksum(magic, ver, low, high);

	bkp_write(REG_MAGIC, magic);
	bkp_write(REG_VER, ver);
	bkp_write(REG_TS_LOW, low);
	bkp_write(REG_TS_HIGH, high);
	bkp_write(REG_CKSUM, cksum);

	LOG_INF("Retained session end UTC: %lld (low=0x%08X, high=0x%08X, cksum=0x%08X)",
	        (long long)end_utc, low, high, cksum);
	return 0;
}

int scheduler_retention_restore(time_t *end_utc)
{
	if (!end_utc) {
		return -EINVAL;
	}

	uint32_t magic = bkp_read(REG_MAGIC);
	if (magic != SCHEDULER_RETENTION_MAGIC) {
		return -EINVAL; /* Not programmed or erased */
	}

	uint32_t ver = bkp_read(REG_VER);
	if (ver != SCHEDULER_RETENTION_VERSION) {
		LOG_WRN("Retention version mismatch: got 0x%08X, expected 0x%08X",
		        ver, SCHEDULER_RETENTION_VERSION);
		return -EINVAL;
	}

	uint32_t low = bkp_read(REG_TS_LOW);
	uint32_t high = bkp_read(REG_TS_HIGH);
	uint32_t stored_cksum = bkp_read(REG_CKSUM);

	uint32_t expected_cksum = calc_checksum(magic, ver, low, high);
	if (stored_cksum != expected_cksum) {
		LOG_ERR("Retention checksum mismatch: got 0x%08X, expected 0x%08X",
		        stored_cksum, expected_cksum);
		return -EBADMSG;
	}

	uint64_t raw_ts = ((uint64_t)high << 32) | (uint64_t)low;
	/* Sanity check: valid unix time between ~2024 and 2099 */
	if (raw_ts < 1704067200ULL || raw_ts > 4102444800ULL) {
		LOG_WRN("Retention timestamp out of bounds: %llu", (unsigned long long)raw_ts);
		return -EINVAL;
	}

	*end_utc = (time_t)raw_ts;
	return 0;
}

void scheduler_retention_invalidate(void)
{
	bkp_write(REG_MAGIC, 0);
	bkp_write(REG_VER, 0);
	bkp_write(REG_TS_LOW, 0);
	bkp_write(REG_TS_HIGH, 0);
	bkp_write(REG_CKSUM, 0);
	LOG_INF("Invalidated scheduler retention backup registers");
}

static inline uint32_t calc_ble_checksum(uint32_t magic, uint32_t ver, uint32_t fp)
{
	return magic ^ ver ^ fp ^ SCHEDULER_RETENTION_XOR_CONST;
}

int ble_retention_save_fingerprint(uint32_t fingerprint)
{
	uint32_t magic = BLE_PROV_RETENTION_MAGIC;
	uint32_t ver = BLE_PROV_RETENTION_VERSION;
	uint32_t cksum = calc_ble_checksum(magic, ver, fingerprint);

	bkp_write(REG_BLE_MAGIC, magic);
	bkp_write(REG_BLE_VER, ver);
	bkp_write(REG_BLE_FP, fingerprint);
	bkp_write(REG_BLE_CKSUM, cksum);

	LOG_INF("Retained BLE provisioning fingerprint: 0x%08X (cksum=0x%08X)", fingerprint, cksum);
	return 0;
}

int ble_retention_restore_fingerprint(uint32_t *fingerprint)
{
	if (!fingerprint) {
		return -EINVAL;
	}

	uint32_t magic = bkp_read(REG_BLE_MAGIC);
	if (magic != BLE_PROV_RETENTION_MAGIC) {
		return -EINVAL; /* Not provisioned or retention lost */
	}

	uint32_t ver = bkp_read(REG_BLE_VER);
	if (ver != BLE_PROV_RETENTION_VERSION) {
		LOG_WRN("BLE retention version mismatch: got 0x%08X, expected 0x%08X",
		        ver, BLE_PROV_RETENTION_VERSION);
		return -EINVAL;
	}

	uint32_t fp = bkp_read(REG_BLE_FP);
	uint32_t stored_cksum = bkp_read(REG_BLE_CKSUM);

	uint32_t expected_cksum = calc_ble_checksum(magic, ver, fp);
	if (stored_cksum != expected_cksum) {
		LOG_ERR("BLE retention checksum mismatch: got 0x%08X, expected 0x%08X",
		        stored_cksum, expected_cksum);
		return -EBADMSG;
	}

	*fingerprint = fp;
	return 0;
}

void ble_retention_invalidate_fingerprint(void)
{
	bkp_write(REG_BLE_MAGIC, 0);
	bkp_write(REG_BLE_VER, 0);
	bkp_write(REG_BLE_FP, 0);
	bkp_write(REG_BLE_CKSUM, 0);
	LOG_INF("Invalidated BLE retention backup registers");
}

