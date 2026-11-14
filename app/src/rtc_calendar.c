/* SPDX-License-Identifier: Apache-2.0 */
#include "rtc_calendar.h"
#include "rtc_sleep.h"
#include "scheduler.h"
#include "wildsonic_trace.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>

#if defined(CONFIG_SOC_SERIES_STM32L4X)

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <soc.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_rtc.h>

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
	if (t->subsec > 999) {
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

static uint16_t rtc_calendar_calc_millis_from_ss(uint32_t ss)
{
	uint32_t pred_s = RTC->PRER & 0x7FFFU;
	if (pred_s == 0U) {
		return 0U;
	}
	if (ss > pred_s) {
		ss = pred_s;
	}
	uint64_t ms = (((uint64_t)pred_s - (uint64_t)ss) * 1000ULL + ((uint64_t)pred_s / 2ULL)) /
	              ((uint64_t)pred_s + 1ULL);
	if (ms > 999ULL) {
		ms = 999ULL;
	}
	return (uint16_t)ms;
}

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

	if (!LL_RCC_IsEnabledRTC() || !LL_RTC_IsActiveFlag_INITS(RTC)) {
		t->valid = false;
		return -ENODATA;
	}

	/*
	 * Per STM32L4 reference manual (RM0351, 32.3.8 Reading the calendar):
	 * When reading shadow registers after low-power modes (Stop 1, Stop 2) or to ensure fresh data:
	 * 1. Clear RSF bit in RTC_ISR (LL_RTC_ClearFlag_RS).
	 * 2. Wait boundedly until RSF is set by hardware (max 2 RTCCLK cycles).
	 * 3. Reading RTC_SSR or RTC_TR locks shadow registers SSR, TR, and DR until DR is read.
	 */
	LL_RTC_ClearFlag_RS(RTC);
	uint32_t rsf_to = 1000;
	while (!LL_RTC_IsActiveFlag_RS(RTC) && --rsf_to) {
		k_busy_wait(10);
	}

	uint32_t ss = RTC->SSR & 0xFFFFU;
	uint32_t tr = LL_RTC_TIME_Get(RTC);
	uint32_t dr = LL_RTC_DATE_Get(RTC);

	t->hour  = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_HOUR(tr));
	t->min   = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_MINUTE(tr));
	t->sec   = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_SECOND(tr));
	t->subsec = rtc_calendar_calc_millis_from_ss(ss);

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

void rtc_calendar_log_snapshot(const char *phase, int64_t target_utc)
{
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	uint32_t tr = RTC->TR;
	uint32_t dr = RTC->DR;
	uint32_t ss = RTC->SSR & 0xFFFFU;
	uint32_t prer = RTC->PRER;
	uint32_t clksrc = LL_RCC_GetRTCClockSource();
	bool lse_rdy = LL_RCC_LSE_IsReady();
	bool lsi_rdy = LL_RCC_LSI_IsReady();
	uint32_t wutr = RTC->WUTR;
	bool wute = LL_RTC_WAKEUP_IsEnabled(RTC);

	struct rtc_calendar_time cur;
	int get_rc = rtc_calendar_get(&cur);
	char time_str[32] = "UNSET";
	int64_t cur_utc = 0;
	if (get_rc == 0 && cur.valid) {
		snprintf(time_str, sizeof(time_str), "%04u-%02u-%02u %02u:%02u:%02u.%03u",
		         cur.year, cur.month, cur.day, cur.hour, cur.min, cur.sec, cur.subsec);
		cur_utc = scheduler_calendar_to_epoch(&cur);
	}
	int64_t remaining = (target_utc > 0 && cur_utc > 0) ? (target_utc - cur_utc) : 0;

	TRACE_RAW_TEST("[TEST] RTC SNAPSHOT [%s]: TR=0x%08X DR=0x%08X SSR=%u PRER=0x%08X CLKSRC=0x%02X LSE=%d LSI=%d WUTR=%u WUTE=%d UTC=%s REMAINING=%lld s\n",
	               phase ? phase : "INFO",
	               (unsigned int)tr, (unsigned int)dr, (unsigned int)ss, (unsigned int)prer,
	               (unsigned int)clksrc, (int)lse_rdy, (int)lsi_rdy,
	               (unsigned int)wutr, (int)wute, time_str, (long long)remaining);
#else
	ARG_UNUSED(phase);
	ARG_UNUSED(target_utc);
#endif
}

bool rtc_calendar_is_untrusted(void)
{
#if defined(CONFIG_SOC_SERIES_STM32L4X)
	return (LL_RCC_GetRTCClockSource() == LL_RCC_RTC_CLKSOURCE_LSI);
#else
	return false;
#endif
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
		TRACE("RTC INIT timeout");
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
		TRACE("RTC RSF sync timeout");
	}

	/* Re-enable write protection */
	LL_RTC_EnableWriteProtection(RTC);

	/* Read back and verify programmed calendar */
	struct rtc_calendar_time readback;
	int read_rc = rtc_calendar_get(&readback);
	if (read_rc != 0 || !readback.valid) {
		TRACE("RTC readback verify failed (%d)", read_rc);
		return (read_rc != 0) ? read_rc : -EIO;
	}
	if (readback.year != t->year || readback.month != t->month || readback.day != t->day ||
	    readback.hour != t->hour || readback.min != t->min ||
	    (readback.sec != t->sec && readback.sec != ((t->sec + 1) % 60))) {
		TRACE("RTC readback mismatch (%04u-%02u-%02u %02u:%02u:%02u)",
		      readback.year, readback.month, readback.day,
		      readback.hour, readback.min, readback.sec);
		return -EIO;
	}

	return 0;
}

static bool s_calendar_initialized = false;

int rtc_calendar_init(void)
{
	if (s_calendar_initialized) {
		return 0;
	}
	int rc = rtc_sleep_init();
	if (rc != 0) {
		TRACE("RTC sleep init failed (%d)", rc);
		return rc;
	}
	s_calendar_initialized = true;
	LL_PWR_EnableBkUpAccess();
	return 0;
}

bool rtc_calendar_is_valid(void)
{
	if (rtc_sleep_init() != 0) {
		return false;
	}
	if (!LL_RCC_IsEnabledRTC() || !LL_RTC_IsActiveFlag_INITS(RTC)) {
		return false;
	}
	struct rtc_calendar_time cal;
	if (rtc_calendar_get(&cal) != 0) {
		return false;
	}
	return (rtc_calendar_validate(&cal) == 0);
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

void rtc_calendar_log_snapshot(const char *phase, int64_t target_utc)
{
	ARG_UNUSED(phase);
	ARG_UNUSED(target_utc);
}

#endif /* CONFIG_SOC_SERIES_STM32L4X */

int rtc_calendar_generate_filename(char *name, size_t len, const struct rtc_calendar_time *t)
{
	if (!name || !t || len < 32) {
		return -EINVAL;
	}
	if (rtc_calendar_validate(t) != 0) {
		return -EINVAL;
	}

	for (unsigned int step = 0; step < 1000; step++) {
		uint16_t candidate_ms = (uint16_t)((t->subsec + step) % 1000U);
		snprintf(name, len, "/SD:/%04u%02u%02u_%02u%02u%02u_%03u.wav",
		         t->year, t->month, t->day, t->hour, t->min, t->sec, candidate_ms);

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

void rtc_calendar_log_boot_times(int32_t tz_offset_sec)
{
	struct rtc_calendar_time cur_cal;
	if (!rtc_calendar_is_valid() || rtc_calendar_get(&cur_cal) != 0) {
		TRACE("RTC Time UTC: UNSET");
		TRACE("RTC Time Local: UNSET");
		return;
	}

	char tz_sign = (tz_offset_sec >= 0) ? '+' : '-';
	int32_t abs_tz = (tz_offset_sec >= 0) ? tz_offset_sec : -tz_offset_sec;
	int tz_h = (int)(abs_tz / 3600);
	int tz_m = (int)((abs_tz % 3600) / 60);

	TRACE("RTC Time UTC: %04u-%02u-%02u %02u:%02u:%02u.%03u",
	      cur_cal.year, cur_cal.month, cur_cal.day,
	      cur_cal.hour, cur_cal.min, cur_cal.sec, cur_cal.subsec);

	time_t epoch = scheduler_calendar_to_epoch(&cur_cal) + (time_t)tz_offset_sec;
	struct rtc_calendar_time loc_cal;
	scheduler_epoch_to_calendar(epoch, &loc_cal);
	TRACE("RTC Time Local: %04u-%02u-%02u %02u:%02u:%02u.%03u %c%02d:%02d",
	      loc_cal.year, loc_cal.month, loc_cal.day,
	      loc_cal.hour, loc_cal.min, loc_cal.sec, cur_cal.subsec,
	      tz_sign, tz_h, tz_m);
}
