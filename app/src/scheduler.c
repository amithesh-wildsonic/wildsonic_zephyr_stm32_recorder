/* SPDX-License-Identifier: Apache-2.0 */
#include "scheduler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <errno.h>

/* Global runtime scheduler configuration (staged in RAM) */
static struct schedule_config g_schedule_cfg;
static bool g_scheduler_initialized = false;

/* --- Epoch and Calendar Conversions --- */

time_t scheduler_calendar_to_epoch(const struct rtc_calendar_time *cal)
{
	if (!cal || cal->year < 2000 || cal->year > 2099 ||
	    cal->month < 1 || cal->month > 12 ||
	    cal->day < 1 || cal->day > 31 ||
	    cal->hour > 23 || cal->min > 59 || cal->sec > 59) {
		return 0;
	}

	/* Days from 1970-01-01 to start of year */
	uint32_t y = cal->year - 1970;
	uint32_t days = y * 365 + (cal->year - 1969) / 4 - (cal->year - 1901) / 100 + (cal->year - 1601) / 400;

	/* Cumulative days prior to given month in a normal year */
	static const uint16_t days_before_month[13] = {
		0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
	};
	days += days_before_month[cal->month];

	/* Leap day adjustment if after February in a leap year */
	if (cal->month > 2 && rtc_calendar_is_leap_year(cal->year)) {
		days += 1;
	}

	days += (cal->day - 1);

	return (time_t)days * SCHEDULER_SECS_PER_DAY +
	       (time_t)cal->hour * SCHEDULER_SECS_PER_HR +
	       (time_t)cal->min * SCHEDULER_SECS_PER_MIN +
	       cal->sec;
}

void scheduler_epoch_to_calendar(time_t epoch, struct rtc_calendar_time *cal)
{
	if (!cal) {
		return;
	}
	memset(cal, 0, sizeof(*cal));

	if (epoch <= 0) {
		cal->year = 2000;
		cal->month = 1;
		cal->day = 1;
		cal->valid = false;
		return;
	}

	cal->sec = (uint8_t)(epoch % 60);
	epoch /= 60;
	cal->min = (uint8_t)(epoch % 60);
	epoch /= 60;
	cal->hour = (uint8_t)(epoch % 24);
	epoch /= 24;

	uint32_t days = (uint32_t)epoch;

	/* Days between 1970-01-01 and 2000-01-01 is 10957 */
	uint16_t year = 1970;
	if (days >= 10957) {
		year = 2000;
		days -= 10957;
	}

	while (year <= 2099) {
		uint16_t dim = rtc_calendar_is_leap_year(year) ? 366 : 365;
		if (days < dim) {
			break;
		}
		days -= dim;
		year++;
	}
	cal->year = year;

	uint8_t month = 1;
	while (month <= 12) {
		uint8_t dim = rtc_calendar_days_in_month(year, month);
		if (days < dim) {
			break;
		}
		days -= dim;
		month++;
	}
	cal->month = (month <= 12) ? month : 12;
	cal->day = (uint8_t)(days + 1);
	cal->valid = true;
}

uint8_t scheduler_epoch_to_weekday(time_t epoch)
{
	/* 1970-01-01 was Thursday.
	 * Weekday index: 0=Mon, 1=Tue, 2=Wed, 3=Thu, 4=Fri, 5=Sat, 6=Sun.
	 * (epoch / 86400 + 3) % 7 maps Thu to 3, Fri to 4, Sat to 5, Sun to 6, Mon to 0. */
	int64_t days = epoch / SCHEDULER_SECS_PER_DAY;
	int64_t w = (days + 3) % 7;
	if (w < 0) {
		w += 7;
	}
	return (uint8_t)w;
}

/* --- Range Verification and Sorting --- */

bool scheduler_is_valid_tod_range(const struct schedule_tod_range *range)
{
	if (!range) {
		return false;
	}
	return ((range->start_second > 0) || (range->end_second < SCHEDULER_SECS_PER_DAY)) &&
	       (range->start_second < range->end_second) &&
	       (range->end_second <= SCHEDULER_SECS_PER_DAY);
}

void scheduler_reset_tod_ranges(struct schedule_tod_range *ranges, uint32_t num_ranges)
{
	if (!ranges) {
		return;
	}
	for (uint32_t i = 0; i < num_ranges; i++) {
		ranges[i].start_second = 0;
		ranges[i].end_second = SCHEDULER_SECS_PER_DAY;
	}
}

void scheduler_sort_tod_ranges(struct schedule_tod_range *ranges, uint32_t num_ranges)
{
	if (!ranges || num_ranges <= 1) {
		return;
	}

	for (uint32_t i = 0; i < num_ranges; i++) {
		for (uint32_t j = i + 1; j < num_ranges; j++) {
			bool valid_i = scheduler_is_valid_tod_range(&ranges[i]);
			bool valid_j = scheduler_is_valid_tod_range(&ranges[j]);
			bool do_swap = false;

			if (valid_i && valid_j) {
				/* Overlapping ranges: merge into ranges[i], invalidate j */
				if ((ranges[i].start_second <= ranges[j].end_second) &&
				    (ranges[i].end_second >= ranges[j].start_second)) {
					if (ranges[j].start_second < ranges[i].start_second) {
						ranges[i].start_second = ranges[j].start_second;
					}
					if (ranges[j].end_second > ranges[i].end_second) {
						ranges[i].end_second = ranges[j].end_second;
					}
					ranges[j].start_second = 0;
					ranges[j].end_second = SCHEDULER_SECS_PER_DAY;
				} else if (ranges[i].start_second > ranges[j].start_second) {
					do_swap = true;
				}
			} else if (valid_j) {
				/* Push invalid entries to the back */
				do_swap = true;
			}

			if (do_swap) {
				struct schedule_tod_range tmp = ranges[i];
				ranges[i] = ranges[j];
				ranges[j] = tmp;
			}
		}
	}
}

/* --- Pure Calculation Engine (Replicating Maxim AudioTask.cpp) --- */

bool scheduler_find_day_interval(
	uint8_t day_of_week_mask,
	time_t start_time_boundary,
	time_t end_time_boundary,
	time_t *record_start_time,
	time_t *record_end_time)
{
	if (!record_start_time || !record_end_time) {
		return false;
	}

	if (day_of_week_mask == 0) {
		*record_start_time = end_time_boundary;
		*record_end_time = end_time_boundary;
		return true;
	}

	if (day_of_week_mask == SCHEDULER_FULL_WEEK_MASK) {
		*record_start_time = start_time_boundary;
		*record_end_time = end_time_boundary;
		return true;
	}

	uint8_t current_weekday = scheduler_epoch_to_weekday(start_time_boundary);
	uint32_t first_active_offset = 0;
	uint8_t w = current_weekday;

	for (first_active_offset = 0; first_active_offset < 7; first_active_offset++) {
		if (day_of_week_mask & (1U << w)) {
			break;
		}
		w = (w + 1) % 7;
	}

	if (first_active_offset >= 7) {
		*record_start_time = end_time_boundary;
		*record_end_time = end_time_boundary;
		return false;
	}

	/* Beginning of active day */
	time_t sec_past_midnight = start_time_boundary % SCHEDULER_SECS_PER_DAY;
	time_t day_mask_begin_time = start_time_boundary - sec_past_midnight +
	                             (time_t)first_active_offset * SCHEDULER_SECS_PER_DAY;

	/* Count consecutive active days */
	uint32_t num_consecutive_days = 0;
	for (uint32_t i = first_active_offset; i < 7; i++) {
		if ((day_of_week_mask & (1U << w)) == 0) {
			break;
		}
		num_consecutive_days++;
		w = (w + 1) % 7;
	}

	time_t day_mask_end_time = day_mask_begin_time + (time_t)num_consecutive_days * SCHEDULER_SECS_PER_DAY;

	*record_start_time = (day_mask_begin_time > start_time_boundary) ? day_mask_begin_time : start_time_boundary;
	*record_end_time = (day_mask_end_time < end_time_boundary) ? day_mask_end_time : end_time_boundary;

	return (*record_start_time < *record_end_time);
}

bool scheduler_find_tod_interval(
	const struct schedule_tod_range *ranges,
	uint32_t num_ranges,
	time_t start_time_boundary,
	time_t end_time_boundary,
	time_t *record_start_time,
	time_t *record_end_time)
{
	if (!record_start_time || !record_end_time) {
		return false;
	}

	uint32_t valid_ranges = 0;
	if (ranges) {
		for (uint32_t i = 0; i < num_ranges; i++) {
			if (!scheduler_is_valid_tod_range(&ranges[i])) {
				break;
			}
			valid_ranges++;
		}
	}

	if (valid_ranges == 0) {
		*record_start_time = start_time_boundary;
		*record_end_time = end_time_boundary;
		return true;
	}

	*record_start_time = end_time_boundary;
	*record_end_time = end_time_boundary;

	time_t sec_past_midnight = start_time_boundary % SCHEDULER_SECS_PER_DAY;
	time_t day_begin = start_time_boundary - sec_past_midnight;

	for (uint32_t day_idx = 0; day_idx < 2; day_idx++) {
		for (uint32_t i = 0; i < valid_ranges; i++) {
			time_t interval_start = day_begin + (time_t)day_idx * SCHEDULER_SECS_PER_DAY + ranges[i].start_second;
			time_t interval_end = day_begin + (time_t)day_idx * SCHEDULER_SECS_PER_DAY + ranges[i].end_second;

			if (interval_start < end_time_boundary && interval_end > start_time_boundary) {
				*record_start_time = (interval_start > start_time_boundary) ? interval_start : start_time_boundary;
				*record_end_time = (interval_end < end_time_boundary) ? interval_end : end_time_boundary;
				return true;
			}
		}
	}

	return false;
}

int scheduler_calculate_next(
	const struct schedule_config *cfg,
	time_t current_time_utc,
	time_t last_end_time_local,
	struct schedule_interval *out)
{
	if (!cfg || !out) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	time_t current_time_local = current_time_utc + cfg->time_zone_offset;

	/* Determine start date bound */
	time_t active_start_time_local = 0;
	if (cfg->start_date.year > 0) {
		struct rtc_calendar_time st_cal = {
			.year = cfg->start_date.year,
			.month = cfg->start_date.month,
			.day = cfg->start_date.day,
			.hour = 0,
			.min = 0,
			.sec = 0
		};
		active_start_time_local = scheduler_calendar_to_epoch(&st_cal);
	}

	/* Determine end date bound (inclusive of end day per Maxim AudioTask.cpp:177) */
	time_t active_end_time_local = INT64_MAX;
	if (cfg->end_date.year > 0) {
		struct rtc_calendar_time end_cal = {
			.year = cfg->end_date.year,
			.month = cfg->end_date.month,
			.day = cfg->end_date.day,
			.hour = 0,
			.min = 0,
			.sec = 0
		};
		active_end_time_local = scheduler_calendar_to_epoch(&end_cal) + SCHEDULER_SECS_PER_DAY;
	}

	/* Advance active start time to current time if past start date */
	if (current_time_local > active_start_time_local) {
		active_start_time_local = current_time_local;
	}

	/* Enforce quiet interval after previous session */
	if (cfg->sleep_interval > 0 && last_end_time_local > 0) {
		time_t last_after_sleep = last_end_time_local + cfg->sleep_interval;
		if (last_after_sleep > active_start_time_local) {
			active_start_time_local = last_after_sleep;
		}
	}

	if (active_end_time_local < active_start_time_local) {
		active_end_time_local = active_start_time_local;
	}

	time_t start_time_boundary = active_start_time_local;
	time_t end_time_boundary = active_end_time_local;
	bool found_time_range = false;

	/* Search across up to 2 contiguous day ranges */
	for (int day_range_index = 0; day_range_index < 2; day_range_index++) {
		time_t found_day_range_start = 0;
		time_t found_day_range_end = 0;

		bool found_day_range = scheduler_find_day_interval(
			cfg->day_of_week_mask,
			start_time_boundary,
			end_time_boundary,
			&found_day_range_start,
			&found_day_range_end);

		if (found_day_range) {
			time_t found_time_range_start = 0;
			time_t found_time_range_end = 0;

			found_time_range = scheduler_find_tod_interval(
				cfg->tod_ranges,
				SCHEDULER_MAX_TIME_OF_DAY_RANGES,
				found_day_range_start,
				found_day_range_end,
				&found_time_range_start,
				&found_time_range_end);

			if (found_time_range) {
				active_start_time_local = found_time_range_start;
				active_end_time_local = found_time_range_end;
				break;
			}
		}

		start_time_boundary = found_day_range_end;
	}

	if (!found_time_range) {
		active_end_time_local = active_start_time_local;
	}

	/* Clip active window duration to record_length if specified */
	if (cfg->record_length > 0 && active_end_time_local > active_start_time_local) {
		uint32_t delta = (uint32_t)(active_end_time_local - active_start_time_local);
		if (delta >= cfg->record_length) {
			active_end_time_local = active_start_time_local + cfg->record_length;
		}
	}

	if (active_end_time_local <= active_start_time_local) {
		out->is_valid = false;
		out->start_time_utc = 0;
		out->duration_seconds = 0;
		out->is_immediate = false;
		out->wait_seconds = 0;
		return 0;
	}

	out->is_valid = true;
	out->duration_seconds = (uint32_t)(active_end_time_local - active_start_time_local);
	out->start_time_utc = active_start_time_local - cfg->time_zone_offset;
	out->wait_seconds = (int64_t)(out->start_time_utc - current_time_utc);
	if (out->wait_seconds <= 0) {
		out->is_immediate = true;
		out->wait_seconds = 0;
	} else {
		out->is_immediate = false;
	}

	return 0;
}

/* --- Parsing and Formatting Helpers --- */

int scheduler_parse_date_range(const char *str, struct schedule_date *start, struct schedule_date *end)
{
	if (!str || !start || !end) {
		return -EINVAL;
	}

	while (isspace((unsigned char)*str)) {
		str++;
	}

	if (strcasecmp(str, "clear") == 0 || strcasecmp(str, "off") == 0 ||
	    strcasecmp(str, "none") == 0 || strcasecmp(str, "0") == 0) {
		memset(start, 0, sizeof(*start));
		memset(end, 0, sizeof(*end));
		return 0;
	}

	unsigned int sy = 0, sm = 0, sd = 0;
	unsigned int ey = 0, em = 0, ed = 0;

	/* Try YYYY/MM/DD-YYYY/MM/DD or YYYY-MM-DD-YYYY-MM-DD */
	int matched = sscanf(str, "%u/%u/%u-%u/%u/%u", &sy, &sm, &sd, &ey, &em, &ed);
	if (matched != 6) {
		matched = sscanf(str, "%u-%u-%u-%u-%u-%u", &sy, &sm, &sd, &ey, &em, &ed);
	}
	if (matched != 6) {
		/* Try compact YYYYMMDD-YYYYMMDD */
		matched = sscanf(str, "%4u%2u%2u-%4u%2u%2u", &sy, &sm, &sd, &ey, &em, &ed);
	}
	if (matched != 6) {
		return -EINVAL;
	}

	if (sy < 2000 || sy > 2099 || sm < 1 || sm > 12 || sd < 1 || sd > rtc_calendar_days_in_month(sy, sm) ||
	    ey < 2000 || ey > 2099 || em < 1 || em > 12 || ed < 1 || ed > rtc_calendar_days_in_month(ey, em)) {
		return -EINVAL;
	}

	if (sy > ey || (sy == ey && sm > em) || (sy == ey && sm == em && sd > ed)) {
		return -EINVAL;
	}

	start->year = (uint16_t)sy;
	start->month = (uint8_t)sm;
	start->day = (uint8_t)sd;

	end->year = (uint16_t)ey;
	end->month = (uint8_t)em;
	end->day = (uint8_t)ed;

	return 0;
}

int scheduler_format_date_range(const struct schedule_date *start, const struct schedule_date *end, char *buf, size_t len)
{
	if (!buf || len < 24) {
		return -EINVAL;
	}

	if (!start || !end || start->year == 0 || end->year == 0) {
		snprintf(buf, len, "Unconstrained (All Dates)");
		return 0;
	}

	snprintf(buf, len, "%04u/%02u/%02u-%04u/%02u/%02u",
	         start->year, start->month, start->day,
	         end->year, end->month, end->day);
	return 0;
}

int scheduler_parse_time_of_day(const char *str, uint32_t *index_out, struct schedule_tod_range *range)
{
	if (!str || !index_out || !range) {
		return -EINVAL;
	}

	while (isspace((unsigned char)*str)) {
		str++;
	}

	if (strcasecmp(str, "clear") == 0 || strcasecmp(str, "all") == 0 || strcasecmp(str, "none") == 0) {
		*index_out = 0;
		range->start_second = 0;
		range->end_second = SCHEDULER_SECS_PER_DAY;
		return 0;
	}

	uint32_t idx = 0;
	const char *time_str = str;

	/* Check if leading index is provided: e.g. "0 08:00:00-12:00:00" */
	if (isdigit((unsigned char)*str) && isspace((unsigned char)str[1])) {
		idx = (uint32_t)(str[0] - '0');
		time_str = str + 2;
	} else if (isdigit((unsigned char)*str) && isdigit((unsigned char)str[1]) && isspace((unsigned char)str[2])) {
		idx = (uint32_t)atoi(str);
		time_str = str + 3;
	}

	if (idx >= SCHEDULER_MAX_TIME_OF_DAY_RANGES) {
		return -EINVAL;
	}

	while (isspace((unsigned char)*time_str)) {
		time_str++;
	}

	unsigned int sh = 0, sm = 0, ss = 0;
	unsigned int eh = 0, em = 0, es = 0;

	int matched = sscanf(time_str, "%u:%u:%u-%u:%u:%u", &sh, &sm, &ss, &eh, &em, &es);
	if (matched != 6) {
		matched = sscanf(time_str, "%u:%u-%u:%u", &sh, &sm, &eh, &em);
		if (matched == 4) {
			ss = 0;
			es = 0;
		}
	}
	if (matched != 6 && matched != 4) {
		/* Try compact HHMMSS-HHMMSS */
		matched = sscanf(time_str, "%2u%2u%2u-%2u%2u%2u", &sh, &sm, &ss, &eh, &em, &es);
	}
	if (matched != 6 && matched != 4) {
		return -EINVAL;
	}

	if (sh > 24 || sm > 59 || ss > 59 || eh > 24 || em > 59 || es > 59) {
		return -EINVAL;
	}

	uint32_t s_sec = sh * SCHEDULER_SECS_PER_HR + sm * SCHEDULER_SECS_PER_MIN + ss;
	uint32_t e_sec = eh * SCHEDULER_SECS_PER_HR + em * SCHEDULER_SECS_PER_MIN + es;

	if (s_sec > SCHEDULER_SECS_PER_DAY) s_sec = SCHEDULER_SECS_PER_DAY;
	if (e_sec > SCHEDULER_SECS_PER_DAY) e_sec = SCHEDULER_SECS_PER_DAY;

	if (s_sec > e_sec) {
		e_sec = s_sec;
	}

	*index_out = idx;
	range->start_second = s_sec;
	range->end_second = e_sec;

	return 0;
}

int scheduler_format_time_of_day(const struct schedule_tod_range *range, char *buf, size_t len)
{
	if (!range || !buf || len < 20) {
		return -EINVAL;
	}

	if (!scheduler_is_valid_tod_range(range)) {
		snprintf(buf, len, "00:00:00-24:00:00 (All Day)");
		return 0;
	}

	uint32_t sh = range->start_second / SCHEDULER_SECS_PER_HR;
	uint32_t sm = (range->start_second % SCHEDULER_SECS_PER_HR) / SCHEDULER_SECS_PER_MIN;
	uint32_t ss = range->start_second % SCHEDULER_SECS_PER_MIN;

	uint32_t eh = range->end_second / SCHEDULER_SECS_PER_HR;
	uint32_t em = (range->end_second % SCHEDULER_SECS_PER_HR) / SCHEDULER_SECS_PER_MIN;
	uint32_t es = range->end_second % SCHEDULER_SECS_PER_MIN;

	snprintf(buf, len, "%02u:%02u:%02u-%02u:%02u:%02u", sh, sm, ss, eh, em, es);
	return 0;
}

int scheduler_parse_days_of_week(const char *str, uint8_t *mask)
{
	if (!str || !mask) {
		return -EINVAL;
	}

	while (isspace((unsigned char)*str) || *str == '"') {
		str++;
	}

	if (*str == '\0' || strcasecmp(str, "X") == 0 || strcasecmp(str, "all") == 0 ||
	    strcasecmp(str, "\"X\"") == 0 || strcasecmp(str, "\"\"") == 0) {
		*mask = SCHEDULER_FULL_WEEK_MASK;
		return 0;
	}

	uint8_t m = 0;
	while (*str && *str != '"') {
		char c = (char)toupper((unsigned char)*str);
		switch (c) {
		case 'M': m |= SCHEDULER_DAY_MON; break;
		case 'T': m |= SCHEDULER_DAY_TUE; break;
		case 'W': m |= SCHEDULER_DAY_WED; break;
		case 'R': m |= SCHEDULER_DAY_THU; break;
		case 'F': m |= SCHEDULER_DAY_FRI; break;
		case 'S': m |= SCHEDULER_DAY_SAT; break;
		case 'U': m |= SCHEDULER_DAY_SUN; break;
		case ' ':
		case ',':
			break;
		default:
			return -EINVAL;
		}
		str++;
	}

	*mask = (m != 0) ? m : SCHEDULER_FULL_WEEK_MASK;
	return 0;
}

int scheduler_format_days_of_week(uint8_t mask, char *buf, size_t len)
{
	if (!buf || len < 10) {
		return -EINVAL;
	}

	if (mask == SCHEDULER_FULL_WEEK_MASK || mask == 0) {
		snprintf(buf, len, "MTWRFSU (All Days)");
		return 0;
	}

	char out[8] = {0};
	int idx = 0;
	if (mask & SCHEDULER_DAY_MON) out[idx++] = 'M';
	if (mask & SCHEDULER_DAY_TUE) out[idx++] = 'T';
	if (mask & SCHEDULER_DAY_WED) out[idx++] = 'W';
	if (mask & SCHEDULER_DAY_THU) out[idx++] = 'R';
	if (mask & SCHEDULER_DAY_FRI) out[idx++] = 'F';
	if (mask & SCHEDULER_DAY_SAT) out[idx++] = 'S';
	if (mask & SCHEDULER_DAY_SUN) out[idx++] = 'U';
	out[idx] = '\0';

	snprintf(buf, len, "%s", out);
	return 0;
}

int scheduler_parse_timezone(const char *str, int32_t *offset_sec)
{
	if (!str || !offset_sec) {
		return -EINVAL;
	}

	while (isspace((unsigned char)*str)) {
		str++;
	}

	if (strcasecmp(str, "Z") == 0 || strcasecmp(str, "UTC") == 0 || strcmp(str, "0") == 0) {
		*offset_sec = 0;
		return 0;
	}

	int sign = 1;
	if (*str == '-') {
		sign = -1;
		str++;
	} else if (*str == '+') {
		sign = 1;
		str++;
	}

	unsigned int hr = 0, mn = 0;
	int matched = sscanf(str, "%u:%u", &hr, &mn);
	if (matched != 2) {
		matched = sscanf(str, "%2u%2u", &hr, &mn);
	}
	if (matched != 2) {
		matched = sscanf(str, "%u", &hr);
		if (matched == 1) {
			mn = 0;
		} else {
			return -EINVAL;
		}
	}

	if (hr > 14 || mn > 59) {
		return -EINVAL;
	}

	int32_t sec = (int32_t)(hr * SCHEDULER_SECS_PER_HR + mn * SCHEDULER_SECS_PER_MIN) * sign;
	if (sec < -43200 || sec > 50400) {
		return -EINVAL;
	}

	*offset_sec = sec;
	return 0;
}

int scheduler_format_timezone(int32_t offset_sec, char *buf, size_t len)
{
	if (!buf || len < 10) {
		return -EINVAL;
	}

	char sign = '+';
	if (offset_sec < 0) {
		sign = '-';
		offset_sec = -offset_sec;
	}

	uint32_t hr = (uint32_t)(offset_sec / SCHEDULER_SECS_PER_HR);
	uint32_t mn = (uint32_t)((offset_sec % SCHEDULER_SECS_PER_HR) / SCHEDULER_SECS_PER_MIN);

	snprintf(buf, len, "%c%02u:%02u", sign, hr, mn);
	return 0;
}

/* --- Runtime Scheduler Configuration State --- */

void scheduler_reset_defaults(void)
{
	memset(&g_schedule_cfg, 0, sizeof(g_schedule_cfg));
	g_schedule_cfg.day_of_week_mask = SCHEDULER_FULL_WEEK_MASK;
	g_schedule_cfg.time_zone_offset = 0;
	g_schedule_cfg.record_length = 60;
	g_schedule_cfg.sleep_interval = 60;

	scheduler_reset_tod_ranges(g_schedule_cfg.tod_ranges, SCHEDULER_MAX_TIME_OF_DAY_RANGES);
	g_scheduler_initialized = true;
}

void scheduler_init(void)
{
	if (!g_scheduler_initialized) {
		scheduler_reset_defaults();
	}
}

void scheduler_get_config(struct schedule_config *cfg)
{
	if (!g_scheduler_initialized) {
		scheduler_init();
	}
	if (cfg) {
		*cfg = g_schedule_cfg;
	}
}

void scheduler_set_config(const struct schedule_config *cfg)
{
	if (cfg) {
		g_schedule_cfg = *cfg;
		g_scheduler_initialized = true;
	}
}

bool scheduler_has_active_constraints(void)
{
	if (!g_scheduler_initialized) {
		return false;
	}

	if (g_schedule_cfg.start_date.year > 0 || g_schedule_cfg.end_date.year > 0) {
		return true;
	}
	if (g_schedule_cfg.day_of_week_mask != SCHEDULER_FULL_WEEK_MASK) {
		return true;
	}
	for (uint32_t i = 0; i < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
		if (scheduler_is_valid_tod_range(&g_schedule_cfg.tod_ranges[i])) {
			return true;
		}
	}
	return false;
}
