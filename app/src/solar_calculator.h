/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file solar_calculator.h
 * @brief NOAA Solar Calculator and Sunrise/Sunset Window Generator for WildSonic STM32.
 *
 * Direct port of canonical WildSonic NOAA solar calculation algorithm
 * (app/solar_calculator.cpp & app/AudioTask.cpp).
 */

#ifndef SOLAR_CALCULATOR_H_
#define SOLAR_CALCULATOR_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SOLAR_SECONDS_PER_DAY        86400U
#define SOLAR_SECONDS_PER_HR         3600U
#define SOLAR_SECONDS_PER_MIN        60U
#define SOLAR_MINS_PER_HR            60U
#define SOLAR_HOURS_PER_DAY          24.0
#define SOLAR_MINUTES_PER_DAY        1440.0
#define SOLAR_MINUTES_PER_HALF_DAY   720.0
#define SOLAR_MINUTES_PER_DEGREE_ROT 4.0
#define SOLAR_DEGREES_PER_CIRCLE     360.0
#define SOLAR_DEGREES_PER_RADIAN     180.0
#define SOLAR_SUN_ZENITH_DEGREES     90.833

#define SOLAR_INVALID_LOCATION_COORDINATE ((int32_t)0x80000000)
#define SOLAR_LOCATION_COORDINATE_SCALE   10000000

#define SOLAR_MAX_RANGES_PER_EVENT 2

/**
 * @brief Compute the Julian date from calendar year, month, day, and UT fractional hours.
 *
 * Core Julian Date equation (Fliegel & Van Flandern algorithm).
 *
 * @param y Year (e.g. 2026)
 * @param m Month (1 .. 12)
 * @param d Day of month (1 .. 31)
 * @param ut_hours Fractional hours in UT (0.0 for midnight)
 * @return Julian day number
 */
double solar_compute_julian_date(int y, int m, int d, double ut_hours);

/**
 * @brief Computes sunrise and sunset time in seconds from 00:00:00 local midnight
 *        using the NOAA Solar Calculator algorithm.
 *
 * @param julian_date Julian day number
 * @param latitude Latitude in decimal degrees [-90, +90]
 * @param longitude Longitude in decimal degrees [-180, +180]
 * @param time_zone_offset Local timezone offset from UTC in seconds
 * @param sunrise_second Output: seconds from local midnight (0 .. 86399)
 * @param sunset_second Output: seconds from local midnight (0 .. 86399)
 * @return true if sun rises and sets on this day, false if polar day or night.
 */
bool solar_compute_sunrise_sunset_seconds(double julian_date,
                                          double latitude,
                                          double longitude,
                                          int time_zone_offset,
                                          uint32_t *sunrise_second,
                                          uint32_t *sunset_second);

/**
 * @brief Compute time-of-day ranges around a solar event (sunrise or sunset)
 *        with before/after intervals, handling midnight-crossing split windows.
 *
 * @param event_second Event time in seconds from local midnight
 * @param before_sec Seconds before event
 * @param after_sec Seconds after event
 * @param out_ranges Output array to hold 1 or 2 schedule_tod_range entries
 * @return Number of populated ranges (1 or 2)
 */
uint32_t solar_compute_sun_ranges(uint32_t event_second,
                                  uint32_t before_sec,
                                  uint32_t after_sec,
                                  struct schedule_tod_range *out_ranges);

/**
 * @brief Compute active recording time-of-day ranges for a given date according to SUNMODE.
 *
 * Populates out_ranges with sorted, non-overlapping ranges clamped to [0, 86400].
 *
 * @param cfg Active schedule configuration with solar parameters
 * @param year Target calendar year (e.g. 2026)
 * @param month Target calendar month (1 .. 12)
 * @param day Target calendar day (1 .. 31)
 * @param out_ranges Output buffer for schedule_tod_range (must hold at least SCHEDULER_MAX_TIME_OF_DAY_RANGES)
 * @param num_ranges_out Output number of valid populated ranges
 * @return true if solar calculation succeeded, false if location unset or polar day/night.
 */
bool solar_compute_tod_ranges_for_date(const struct schedule_config *cfg,
                                       int year,
                                       int month,
                                       int day,
                                       struct schedule_tod_range *out_ranges,
                                       uint32_t *num_ranges_out);

#ifdef __cplusplus
}
#endif

#endif /* SOLAR_CALCULATOR_H_ */

