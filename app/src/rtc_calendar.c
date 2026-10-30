/* SPDX-License-Identifier: Apache-2.0 */
#include "rtc_calendar.h"
#include "rtc_sleep.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>

#if defined(CONFIG_SOC_SERIES_STM32L4X)

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <soc.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_rtc.h>

LOG_MODULE_REGISTER(rtc_calendar, LOG_LEVEL_INF);

#endif /* CONFIG_SOC_SERIES_STM32L4X */

bool rtc_calendar_is_leap_year(uint16_t year)
{
	return ((year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0)));
}

uint8_t rtc_calendar_days_in_month(uint16_t year, uint8_t month)
{
	if (month < 1 || month > 12) {
		return 0;
	}
	switch (month) {
	case 1:  /* Jan */
	case 3:  /* Mar */
	case 5:  /* May */
	case 7:  /* Jul */
	case 8:  /* Aug */
	case 10: /* Oct */
	case 12: /* Dec */
		return 31;
	case 4:  /* Apr */
	case 6:  /* Jun */
	case 9:  /* Sep */
	case 11: /* Nov */
		return 30;
	case 2:  /* Feb */
		return rtc_calendar_is_leap_year(year) ? 29 : 28;
	default:
		return 0;
	}
}

int rtc_calendar_validate(const struct rtc_calendar_time *t)
{
	if (!t) {
		return -EINVAL;
	}
	if (t->year < 2000 || t->year > 2099) {
		return -EINVAL;
	}
	if (t->month < 1 || t->month > 12) {
		return -EINVAL;
	}
	uint8_t max_days = rtc_calendar_days_in_month(t->year, t->month);
	if (t->day < 1 || t->day > max_days) {
		return -EINVAL;
	}
	if (t->hour > 23 || t->min > 59 || t->sec > 59) {
		return -EINVAL;
	}
	return 0;
}

int rtc_calendar_format(const struct rtc_calendar_time *t, char *buf, size_t len)
{
	if (!t || !buf || len < 20) {
		return -EINVAL;
	}
	if (!t->valid) {
		snprintf(buf, len, "UNSET / INVALID");
		return 0;
	}
	snprintf(buf, len, "%04u-%02u-%02u %02u:%02u:%02u",
	         t->year, t->month, t->day, t->hour, t->min, t->sec);
	return 0;
}

int rtc_calendar_parse(const char *str, struct rtc_calendar_time *t)
{
	if (!str || !t) {
		return -EINVAL;
	}
	memset(t, 0, sizeof(*t));

	unsigned int y = 0, m = 0, d = 0, hr = 0, mn = 0, sc = 0;
	int matched = sscanf(str, "%u-%u-%u %u:%u:%u", &y, &m, &d, &hr, &mn, &sc);
	if (matched != 6) {
		/* Try alternative separator 'T' */
		matched = sscanf(str, "%u-%u-%uT%u:%u:%u", &y, &m, &d, &hr, &mn, &sc);
		if (matched != 6) {
			return -EINVAL;
		}
	}

	t->year = (uint16_t)y;
	t->month = (uint8_t)m;
	t->day = (uint8_t)d;
	t->hour = (uint8_t)hr;
	t->min = (uint8_t)mn;
	t->sec = (uint8_t)sc;

	int ret = rtc_calendar_validate(t);
	if (ret != 0) {
		return ret;
	}
	t->valid = true;
	return 0;
}

#if defined(CONFIG_SOC_SERIES_STM32L4X)

int rtc_calendar_get(struct rtc_calendar_time *t)
{
	if (!t) {
		return -EINVAL;
	}
	memset(t, 0, sizeof(*t));

	/* Ensure RTC peripheral and backup domain access are initialized */
	int init_rc = rtc_sleep_init();
	if (init_rc != 0) {
		t->valid = false;
		return init_rc;
	}

	/*
	 * In STM32L4, RTC_ISR_INITS flag indicates if calendar registers
	 * have been written at least once. If INITS == 0, time has never been configured.
	 */
	if (!LL_RTC_IsActiveFlag_INITS(RTC)) {
		t->valid = false;
		return -ENODATA;
	}

	/*
	 * Per STM32L4 reference manual (RM0351):
	 * To ensure consistent time and date read, read RTC_TR first.
	 * Reading TR locks the content of shadow registers TR and DR until DR is read.
	 */
	uint32_t tr = LL_RTC_TIME_Get(RTC);
	uint32_t dr = LL_RTC_DATE_Get(RTC);

	t->hour  = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_HOUR(tr));
	t->min   = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_MINUTE(tr));
	t->sec   = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_SECOND(tr));

	t->year  = 2000 + __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_YEAR(dr));
	t->month = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_MONTH(dr));
	t->day   = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_DAY(dr));

	if (rtc_calendar_validate(t) != 0) {
		t->valid = false;
		return -EINVAL;
	}

	t->valid = true;
	return 0;
}

int rtc_calendar_set(const struct rtc_calendar_time *t)
{
	int ret = rtc_calendar_validate(t);
	if (ret != 0) {
		return ret;
	}

	/* Ensure RTC peripheral clock and backup domain are enabled */
	int init_rc = rtc_sleep_init();
	if (init_rc != 0) {
		return init_rc;
	}

	LL_PWR_EnableBkUpAccess();
	LL_RTC_DisableWriteProtection(RTC);

	/* Enter RTC Initialization Mode */
	LL_RTC_EnableInitMode(RTC);

	uint32_t timeout = 10000;
	while (!LL_RTC_IsActiveFlag_INIT(RTC) && --timeout) {
		k_busy_wait(10);
	}
	if (timeout == 0) {
		LL_RTC_EnableWriteProtection(RTC);
		LOG_ERR("Timeout waiting for RTC INIT mode");
		return -ETIMEDOUT;
	}

	/* Configure 24-hour time format */
	LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);

	/* Program Time Register (RTC_TR) */
	LL_RTC_TIME_Config(RTC,
	                   LL_RTC_TIME_FORMAT_AM_OR_24,
	                   __LL_RTC_CONVERT_BIN2BCD(t->hour),
	                   __LL_RTC_CONVERT_BIN2BCD(t->min),
	                   __LL_RTC_CONVERT_BIN2BCD(t->sec));

	/* Calculate Day of Week using Sakamoto's algorithm (1 = Monday .. 7 = Sunday) */
	static const int t_table[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
	int y = t->year - (t->month < 3);
	int dow = (y + y / 4 - y / 100 + y / 400 + t_table[t->month - 1] + t->day) % 7;
	uint8_t weekday = (dow == 0) ? 7 : (uint8_t)dow;

	/* Program Date Register (RTC_DR) */
	uint8_t yr_offset = (uint8_t)(t->year - 2000);
	LL_RTC_DATE_Config(RTC,
	                   weekday,
	                   __LL_RTC_CONVERT_BIN2BCD(t->day),
	                   __LL_RTC_CONVERT_BIN2BCD(t->month),
	                   __LL_RTC_CONVERT_BIN2BCD(yr_offset));

	/* Exit RTC Initialization Mode */
	LL_RTC_DisableInitMode(RTC);

	/* Clear and wait for Registers Synchronization Flag (RSF) */
	LL_RTC_ClearFlag_RS(RTC);
	uint32_t rsf_to = 10000;
	while (!LL_RTC_IsActiveFlag_RS(RTC) && --rsf_to) {
		k_busy_wait(10);
	}
	if (rsf_to == 0) {
		LOG_WRN("Timeout waiting for RTC RSF synchronization");
	}

	/* Re-enable write protection */
	LL_RTC_EnableWriteProtection(RTC);

	/* Read back and verify programmed calendar */
	struct rtc_calendar_time readback;
	int read_rc = rtc_calendar_get(&readback);
	if (read_rc != 0 || !readback.valid) {
		LOG_ERR("RTC calendar readback verification failed (%d)", read_rc);
		return (read_rc != 0) ? read_rc : -EIO;
	}
	if (readback.year != t->year || readback.month != t->month || readback.day != t->day ||
	    readback.hour != t->hour || readback.min != t->min ||
	    (readback.sec != t->sec && readback.sec != ((t->sec + 1) % 60))) {
		LOG_ERR("RTC readback mismatch: expected %04u-%02u-%02u %02u:%02u:%02u, got %04u-%02u-%02u %02u:%02u:%02u",
		        t->year, t->month, t->day, t->hour, t->min, t->sec,
		        readback.year, readback.month, readback.day, readback.hour, readback.min, readback.sec);
		return -EIO;
	}

	LOG_INF("RTC calendar updated and verified: %04u-%02u-%02u %02u:%02u:%02u (weekday %u)",
	        t->year, t->month, t->day, t->hour, t->min, t->sec, weekday);
	return 0;
}

int rtc_calendar_init(void)
{
	int rc = rtc_sleep_init();
	if (rc != 0) {
		LOG_ERR("rtc_calendar_init failed: rtc_sleep_init returned %d", rc);
		return rc;
	}
	LL_PWR_EnableBkUpAccess();
	if (LL_RTC_IsActiveFlag_INITS(RTC)) {
		struct rtc_calendar_time cur;
		if (rtc_calendar_get(&cur) == 0) {
			LOG_INF("RTC calendar online: %04u-%02u-%02u %02u:%02u:%02u",
			        cur.year, cur.month, cur.day, cur.hour, cur.min, cur.sec);
		}
	} else {
		LOG_INF("RTC calendar online: uninitialized / unset");
	}
	return 0;
}

bool rtc_calendar_is_valid(void)
{
	if (rtc_sleep_init() != 0) {
		return false;
	}
	return (LL_RTC_IsActiveFlag_INITS(RTC) != 0);
}

#else /* !CONFIG_SOC_SERIES_STM32L4X */

static struct rtc_calendar_time sim_time;
static bool sim_time_valid = false;

int rtc_calendar_init(void)
{
	return 0;
}

bool rtc_calendar_is_valid(void)
{
	return sim_time_valid;
}

int rtc_calendar_get(struct rtc_calendar_time *t)
{
	if (!t) return -EINVAL;
	if (!sim_time_valid) {
		t->valid = false;
		return -ENODATA;
	}
	*t = sim_time;
	return 0;
}

int rtc_calendar_set(const struct rtc_calendar_time *t)
{
	int ret = rtc_calendar_validate(t);
	if (ret != 0) return ret;
	sim_time = *t;
	sim_time.valid = true;
	sim_time_valid = true;
	return 0;
}

#endif /* CONFIG_SOC_SERIES_STM32L4X */

int rtc_calendar_generate_filename(char *name, size_t len, const struct rtc_calendar_time *t)
{
	if (!name || !t || len < 32) {
		return -EINVAL;
	}

	for (unsigned int n = 1; n < 100; n++) {
		snprintf(name, len, "/SD:/WS_%04u%02u%02u_%02u%02u%02u_%02u.WAV",
		         t->year, t->month, t->day, t->hour, t->min, t->sec, n);

#if defined(CONFIG_FILE_SYSTEM)
		struct fs_dirent ent;
		if (fs_stat(name, &ent) != 0) {
			return 0; /* file does not exist yet */
		}
#else
		return 0;
#endif
	}

	return -ENOSPC;
}
