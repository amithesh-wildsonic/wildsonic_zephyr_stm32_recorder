/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>
#include "rtc_calendar.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum supported time-of-day recording ranges (matches original Maxim firmware).
 */
#define SCHEDULER_MAX_TIME_OF_DAY_RANGES 10

/**
 * @brief Full week bitmask (bits 0..6 set = Monday through Sunday).
 */
#define SCHEDULER_FULL_WEEK_MASK         0x7F

#define SCHEDULER_SECS_PER_DAY           86400
#define SCHEDULER_SECS_PER_HR            3600
#define SCHEDULER_SECS_PER_MIN           60

/**
 * @brief Day-of-week bit positions.
 */
#define SCHEDULER_DAY_MON                (1U << 0)
#define SCHEDULER_DAY_TUE                (1U << 1)
#define SCHEDULER_DAY_WED                (1U << 2)
#define SCHEDULER_DAY_THU                (1U << 3)
#define SCHEDULER_DAY_FRI                (1U << 4)
#define SCHEDULER_DAY_SAT                (1U << 5)
#define SCHEDULER_DAY_SUN                (1U << 6)

/**
 * @brief Time-of-day range in seconds since midnight (0 .. 86400).
 */
struct schedule_tod_range {
	uint32_t start_second; /* 0 .. 86400 */
	uint32_t end_second;   /* 0 .. 86400 */
};

/**
 * @brief Calendar date representation for start/end bounds.
 */
struct schedule_date {
	uint16_t year;  /* 2000 .. 2099, or 0 if unconstrained */
	uint8_t month;  /* 1 .. 12 */
	uint8_t day;    /* 1 .. 31 */
};

/**
 * @brief Sun recording modes matching canonical WildSonic reference.
 */
enum sun_recording_mode {
	SUN_MODE_DISABLED = 0,
	SUN_MODE_SUNRISE,
	SUN_MODE_SUNSET,
	SUN_MODE_SUNRISE_AND_SUNSET,
	SUN_MODE_SUNRISE_TO_SUNSET,
	SUN_MODE_SUNSET_TO_SUNRISE,
};

#define SCHEDULER_INVALID_LOCATION_COORDINATE ((int32_t)0x80000000)
#define SCHEDULER_LOCATION_COORDINATE_SCALE   10000000

/**
 * @brief Complete scheduler configuration model.
 */
struct schedule_config {
	struct schedule_date start_date; /* Unconstrained if year == 0 */
	struct schedule_date end_date;   /* Unconstrained if year == 0 (inclusive of that day) */
	struct schedule_tod_range tod_ranges[SCHEDULER_MAX_TIME_OF_DAY_RANGES];
	uint8_t day_of_week_mask;        /* Bit 0=Mon .. Bit 6=Sun. 0x7F = all days */
	int32_t time_zone_offset;        /* Signed seconds offset from UTC (-43200 to +50400) */
	uint32_t record_length;          /* Session recording duration in seconds */
	uint32_t sleep_interval;         /* Minimum quiet/pause interval between sessions in seconds */

	/* Solar scheduling & location configuration */
	int32_t latitude_e7;             /* Signed decimal degrees * 1e7; SCHEDULER_INVALID_LOCATION_COORDINATE if unset */
	int32_t longitude_e7;            /* Signed decimal degrees * 1e7; SCHEDULER_INVALID_LOCATION_COORDINATE if unset */
	uint8_t sun_recording_mode;      /* enum sun_recording_mode */
	uint8_t sunrise_sunset_rounding; /* Rounding interval in minutes (0 = none) */
	uint8_t sunrise_interval_before; /* Minutes before sunrise to start recording */
	uint8_t sunrise_interval_after;  /* Minutes after sunrise to end recording */
	uint8_t sunset_interval_before;  /* Minutes before sunset to start recording */
	uint8_t sunset_interval_after;   /* Minutes after sunset to end recording */
};

/**
 * @brief Output result of the next scheduled session evaluation.
 */
struct schedule_interval {
	time_t start_time_utc;      /* Scheduled UTC start time (Unix epoch seconds) */
	uint32_t duration_seconds;  /* Recording duration in seconds */
	bool is_immediate;          /* True if recording window is open right now */
	bool is_valid;              /* True if an active future/current window was found */
	int64_t wait_seconds;       /* Seconds to wait until start_time_utc from reference time */
};

/* --- Epoch and Calendar Conversions --- */

time_t scheduler_calendar_to_epoch(const struct rtc_calendar_time *cal);
void scheduler_epoch_to_calendar(time_t epoch, struct rtc_calendar_time *cal);
uint8_t scheduler_epoch_to_weekday(time_t epoch); /* 0=Mon, 1=Tue, ..., 6=Sun */

/* --- Parsing and Formatting Helpers --- */

int scheduler_parse_date_range(const char *str, struct schedule_date *start, struct schedule_date *end);
int scheduler_format_date_range(const struct schedule_date *start, const struct schedule_date *end, char *buf, size_t len);

int scheduler_parse_time_of_day(const char *str, uint32_t *index_out, struct schedule_tod_range *range);
int scheduler_format_time_of_day(const struct schedule_tod_range *range, char *buf, size_t len);

int scheduler_parse_days_of_week(const char *str, uint8_t *mask);
int scheduler_format_days_of_week(uint8_t mask, char *buf, size_t len);

int scheduler_parse_timezone(const char *str, int32_t *offset_sec);
int scheduler_format_timezone(int32_t offset_sec, char *buf, size_t len);

/* --- Range Verification and Sorting --- */

bool scheduler_is_valid_tod_range(const struct schedule_tod_range *range);
void scheduler_sort_tod_ranges(struct schedule_tod_range *ranges, uint32_t num_ranges);
void scheduler_reset_tod_ranges(struct schedule_tod_range *ranges, uint32_t num_ranges);

/* --- Pure Calculation Engine (Replicating Maxim AudioTask.cpp) --- */

bool scheduler_find_day_interval(
	uint8_t day_of_week_mask,
	time_t start_time_boundary,
	time_t end_time_boundary,
	time_t *record_start_time,
	time_t *record_end_time);

bool scheduler_find_tod_interval(
	const struct schedule_tod_range *ranges,
	uint32_t num_ranges,
	time_t start_time_boundary,
	time_t end_time_boundary,
	time_t *record_start_time,
	time_t *record_end_time);

int scheduler_calculate_next(
	const struct schedule_config *cfg,
	time_t current_time_utc,
	time_t last_end_time_local,
	struct schedule_interval *out);

/* --- Runtime Scheduler Configuration State --- */

void scheduler_init(void);
void scheduler_get_config(struct schedule_config *cfg);
void scheduler_set_config(const struct schedule_config *cfg);
void scheduler_reset_defaults(void);
bool scheduler_has_active_constraints(void);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H_ */

