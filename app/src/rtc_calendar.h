/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RTC_CALENDAR_H_
#define RTC_CALENDAR_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Representation of calendar date and time.
 */
struct rtc_calendar_time {
	uint16_t year;   /* 2000 .. 2099 */
	uint8_t month;   /* 1 .. 12 */
	uint8_t day;     /* 1 .. 31 (month-dependent) */
	uint8_t hour;    /* 0 .. 23 */
	uint8_t min;     /* 0 .. 59 */
	uint8_t sec;     /* 0 .. 59 */
	uint16_t subsec; /* 0 .. 999; RTC subsecond milliseconds, when available */
	bool valid;      /* true if calendar has been initialized/set */
};

/**
 * @brief Check if a year is a leap year (Gregorian calendar).
 */
bool rtc_calendar_is_leap_year(uint16_t year);

/**
 * @brief Get maximum days in a given month of a given year.
 */
uint8_t rtc_calendar_days_in_month(uint16_t year, uint8_t month);

/**
 * @brief Validate date/time fields.
 *
 * Checks year [2000..2099], month [1..12], day [1..days_in_month],
 * hour [0..23], min [0..59], sec [0..59].
 *
 * @return 0 if valid, -EINVAL on out-of-range value.
 */
int rtc_calendar_validate(const struct rtc_calendar_time *t);

/**
 * @brief Initialize RTC calendar subsystem.
 */
int rtc_calendar_init(void);

/**
 * @brief Check if RTC clock source is untrusted (e.g. LSI fallback, +-5-10% drift).
 */
bool rtc_calendar_is_untrusted(void);

/**
 * @brief Check if hardware RTC calendar currently holds a valid programmed date and time.
 *
 * @return true if RTC calendar is synchronized and running, false if uninitialized/unset.
 */
bool rtc_calendar_is_valid(void);

/**
 * @brief Read current date and time from the STM32 hardware RTC calendar.
 *
 * Reads hardware shadow registers RTC_TR and RTC_DR atomically.
 * Checks hardware INITS flag to determine whether time has been configured.
 *
 * @param t Output date/time structure.
 * @return 0 if valid time retrieved, -ENODATA if calendar is uninitialized/unset.
 */
int rtc_calendar_get(struct rtc_calendar_time *t);

/**
 * @brief Set the STM32 hardware RTC calendar date and time.
 *
 * Validates fields, enters RTC initialization mode, programs RTC_TR and RTC_DR,
 * calculates weekday automatically, and exits init mode.
 *
 * @param t Date/time to program into RTC.
 * @return 0 on success, negative errno on error.
 */
int rtc_calendar_set(const struct rtc_calendar_time *t);

/**
 * @brief Log TEST snapshot of RTC registers, calendar state and remaining sleep interval.
 */
void rtc_calendar_log_snapshot(const char *phase, int64_t target_utc);

/**
 * @brief Format date/time into standard string "YYYY-MM-DD HH:MM:SS".
 *
 * @param t Date/time structure.
 * @param buf Output string buffer.
 * @param len Size of output string buffer (must be >= 20 bytes).
 * @return 0 on success, -EINVAL on error.
 */
int rtc_calendar_format(const struct rtc_calendar_time *t, char *buf, size_t len);

/**
 * @brief Parse standard string "YYYY-MM-DD HH:MM:SS" into date/time structure.
 *
 * @param str Input string.
 * @param t Output date/time structure.
 * @return 0 on success, -EINVAL if string format is invalid or values out of range.
 */
int rtc_calendar_parse(const char *str, struct rtc_calendar_time *t);

/**
 * @brief Generate collision-free timestamped filename: /SD:/YYYYMMDD_HHMMSS_mmm.wav
 *
 * Uses the recorded RTC subsecond field when available and walks through the
 * millisecond bucket to avoid overwriting an existing file.
 *
 * @param name Output string buffer.
 * @param len Size of output buffer (must be >= 32 bytes).
 * @param t Valid date/time structure.
 * @return 0 on success, negative errno on error.
 */
int rtc_calendar_generate_filename(char *name, size_t len, const struct rtc_calendar_time *t);

#ifdef __cplusplus
}
#endif

#endif /* RTC_CALENDAR_H_ */
