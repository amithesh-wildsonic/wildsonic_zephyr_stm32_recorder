/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file solar_calculator.c
 * @brief NOAA Solar Calculator and Sunrise/Sunset Window Generator for WildSonic STM32.
 *
 * Implements the NOAA Solar Calculator algorithm and solar window splitting
 * with full parity to original WildSonic reference firmware (app/solar_calculator.cpp
 * and app/AudioTask.cpp).
 */

#include "solar_calculator.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double degrees_to_radians(double degrees)
{
	return (degrees * M_PI) / SOLAR_DEGREES_PER_RADIAN;
}

static double radians_to_degrees(double radians)
{
	return (radians * SOLAR_DEGREES_PER_RADIAN) / M_PI;
}

static double normalize_day_fraction(double fraction)
{
	return fraction - floor(fraction);
}

static uint32_t day_fraction_to_seconds(double fraction)
{
	uint32_t seconds = (uint32_t)floor(
		(normalize_day_fraction(fraction) * (double)SOLAR_SECONDS_PER_DAY) + 0.5);

	if (seconds >= SOLAR_SECONDS_PER_DAY) {
		seconds %= SOLAR_SECONDS_PER_DAY;
	}
	return seconds;
}

double solar_compute_julian_date(int y, int m, int d, double ut_hours)
{
	const int JD_GREGORIAN_CENTURY    = 100;
	const int JD_GREGORIAN_LEAP_CYCLE = 4;
	const int JD_GREGORIAN_OFFSET     = 2;
	const double JD_DAYS_PER_YEAR     = 365.25;
	const double JD_DAYS_PER_MONTH    = 30.6001;
	const int JD_YEAR_SHIFT           = 4716;
	const double JD_EPOCH_OFFSET      = 1524.5;
	const double HOURS_IN_DAY         = 24.0;

	/* Jan and Feb are treated as months 13 and 14 of the previous year */
	if (m <= 2) {
		y -= 1;
		m += 12;
	}

	int a = y / JD_GREGORIAN_CENTURY;
	int b = JD_GREGORIAN_OFFSET - a + (a / JD_GREGORIAN_LEAP_CYCLE);

	double jd = floor(JD_DAYS_PER_YEAR * (y + JD_YEAR_SHIFT)) +
	            floor(JD_DAYS_PER_MONTH * (m + 1)) + (double)d + (double)b -
	            JD_EPOCH_OFFSET + (ut_hours / HOURS_IN_DAY);
	return jd;
}

bool solar_compute_sunrise_sunset_seconds(double julian_date,
                                          double latitude,
                                          double longitude,
                                          int time_zone_offset,
                                          uint32_t *sunrise_second,
                                          uint32_t *sunset_second)
{
	if (!sunrise_second || !sunset_second) {
		return false;
	}

	const double JD_J2000_EPOCH          = 2451545.0;
	const double DAYS_PER_JULIAN_CENTURY = 36525.0;

	const double MINUTES_PER_HALF_DAY    = 720.0;
	const double MINUTES_PER_DEGREE_ROT  = 4.0;

	const double SUN_MEAN_LONG_C0        = 280.46646;
	const double SUN_MEAN_LONG_C1        = 36000.76983;
	const double SUN_MEAN_LONG_C2        = 0.0003032;

	const double SUN_MEAN_ANOM_C0        = 357.52911;
	const double SUN_MEAN_ANOM_C1        = 35999.05029;
	const double SUN_MEAN_ANOM_C2        = 0.0001537;

	const double EARTH_ECCENTRICITY_C0   = 0.016708634;
	const double EARTH_ECCENTRICITY_C1   = 0.000042037;
	const double EARTH_ECCENTRICITY_C2   = 0.0000001267;

	const double SUN_EQ_CENTER_C0        = 1.914602;
	const double SUN_EQ_CENTER_C1        = 0.004817;
	const double SUN_EQ_CENTER_C2        = 0.000014;
	const double SUN_EQ_CENTER_C3        = 0.019993;
	const double SUN_EQ_CENTER_C4        = 0.000101;
	const double SUN_EQ_CENTER_C5        = 0.000289;

	const double SUN_OMEGA_BASE          = 125.04;
	const double SUN_OMEGA_SCALE         = 1934.136;
	const double APPARENT_LONG_CORR1     = 0.00569;
	const double APPARENT_LONG_CORR2     = 0.00478;

	const double OBLIQUITY_DEG           = 23.0;
	const double OBLIQUITY_MIN           = 26.0;
	const double OBLIQUITY_SEC_BASE      = 21.448;
	const double OBLIQUITY_SEC_SCALE1    = 46.815;
	const double OBLIQUITY_SEC_SCALE2    = 0.00059;
	const double OBLIQUITY_SEC_SCALE3    = 0.001813;
	const double OBLIQUITY_CORR          = 0.00256;

	const double EOT_MULT_Y2             = 0.5;
	const double EOT_MULT_E2             = 1.25;

	const double HARMONIC_2              = 2.0;
	const double HARMONIC_3              = 3.0;
	const double HARMONIC_4              = 4.0;
	const double HALF_ANGLE_DIV          = 2.0;

	const double ARC_SECS_PER_ARC_MIN    = 60.0;
	const double ARC_MINS_PER_DEGREE     = 60.0;
	const double MINS_PER_HOUR           = 60.0;
	const double SECS_PER_HOUR           = 3600.0;

	const double VALID_COSINE_MIN        = -1.0;
	const double VALID_COSINE_MAX        = 1.0;

	const double julian_century = (julian_date - JD_J2000_EPOCH) / DAYS_PER_JULIAN_CENTURY;

	double geom_mean_longitude = fmod(
		SUN_MEAN_LONG_C0 +
		(julian_century * (SUN_MEAN_LONG_C1 + (julian_century * SUN_MEAN_LONG_C2))),
		SOLAR_DEGREES_PER_CIRCLE);
	if (geom_mean_longitude < 0.0) {
		geom_mean_longitude += SOLAR_DEGREES_PER_CIRCLE;
	}

	const double geom_mean_anomaly =
		SUN_MEAN_ANOM_C0 +
		(julian_century * (SUN_MEAN_ANOM_C1 - (julian_century * SUN_MEAN_ANOM_C2)));
	const double mean_anomaly_rad = degrees_to_radians(geom_mean_anomaly);

	const double earth_orbit_eccentricity =
		EARTH_ECCENTRICITY_C0 -
		(julian_century * (EARTH_ECCENTRICITY_C1 + (julian_century * EARTH_ECCENTRICITY_C2)));

	const double sun_eq_center =
		(sin(mean_anomaly_rad) *
		 (SUN_EQ_CENTER_C0 -
		  (julian_century * (SUN_EQ_CENTER_C1 + (SUN_EQ_CENTER_C2 * julian_century))))) +
		(sin(HARMONIC_2 * mean_anomaly_rad) *
		 (SUN_EQ_CENTER_C3 - (SUN_EQ_CENTER_C4 * julian_century))) +
		(sin(HARMONIC_3 * mean_anomaly_rad) * SUN_EQ_CENTER_C5);

	const double sun_true_longitude = geom_mean_longitude + sun_eq_center;
	const double omega = SUN_OMEGA_BASE - (SUN_OMEGA_SCALE * julian_century);
	const double sun_apparent_longitude =
		sun_true_longitude - APPARENT_LONG_CORR1 -
		(APPARENT_LONG_CORR2 * sin(degrees_to_radians(omega)));

	const double mean_obliquity =
		OBLIQUITY_DEG +
		((OBLIQUITY_MIN +
		  ((OBLIQUITY_SEC_BASE -
		    (julian_century *
		     (OBLIQUITY_SEC_SCALE1 +
		      (julian_century * (OBLIQUITY_SEC_SCALE2 -
		                         (julian_century * OBLIQUITY_SEC_SCALE3)))))) /
		   ARC_SECS_PER_ARC_MIN)) /
		 ARC_MINS_PER_DEGREE);

	const double corrected_obliquity =
		mean_obliquity + (OBLIQUITY_CORR * cos(degrees_to_radians(omega)));
	const double corrected_obliquity_rad = degrees_to_radians(corrected_obliquity);
	const double sun_declination =
		asin(sin(corrected_obliquity_rad) *
		     sin(degrees_to_radians(sun_apparent_longitude)));

	const double y = tan(corrected_obliquity_rad / HALF_ANGLE_DIV) *
	                 tan(corrected_obliquity_rad / HALF_ANGLE_DIV);
	const double geom_mean_longitude_rad = degrees_to_radians(geom_mean_longitude);
	const double equation_of_time =
		MINUTES_PER_DEGREE_ROT *
		radians_to_degrees(
			(y * sin(HARMONIC_2 * geom_mean_longitude_rad)) -
			(HARMONIC_2 * earth_orbit_eccentricity * sin(mean_anomaly_rad)) +
			(HARMONIC_4 * earth_orbit_eccentricity * y * sin(mean_anomaly_rad) *
			 cos(HARMONIC_2 * geom_mean_longitude_rad)) -
			(EOT_MULT_Y2 * y * y * sin(HARMONIC_4 * geom_mean_longitude_rad)) -
			(EOT_MULT_E2 * earth_orbit_eccentricity * earth_orbit_eccentricity *
			 sin(HARMONIC_2 * mean_anomaly_rad)));

	const double time_zone_hours = (double)time_zone_offset / SECS_PER_HOUR;
	const double solar_noon_fraction =
		(MINUTES_PER_HALF_DAY - (MINUTES_PER_DEGREE_ROT * longitude) -
		 equation_of_time + (time_zone_hours * MINS_PER_HOUR)) /
		SOLAR_MINUTES_PER_DAY;

	const double latitude_rad = degrees_to_radians(latitude);
	const double hour_angle_arg =
		(cos(degrees_to_radians(SOLAR_SUN_ZENITH_DEGREES)) /
		 (cos(latitude_rad) * cos(sun_declination))) -
		(tan(latitude_rad) * tan(sun_declination));

	if ((hour_angle_arg < VALID_COSINE_MIN) || (hour_angle_arg > VALID_COSINE_MAX)) {
		return false;
	}

	const double hour_angle_degrees = radians_to_degrees(acos(hour_angle_arg));
	const double sunrise_fraction =
		solar_noon_fraction - (hour_angle_degrees / SOLAR_DEGREES_PER_CIRCLE);
	const double sunset_fraction =
		solar_noon_fraction + (hour_angle_degrees / SOLAR_DEGREES_PER_CIRCLE);

	*sunrise_second = day_fraction_to_seconds(sunrise_fraction);
	*sunset_second  = day_fraction_to_seconds(sunset_fraction);

	return *sunrise_second < *sunset_second;
}

uint32_t solar_compute_sun_ranges(uint32_t event_second,
                                  uint32_t before_sec,
                                  uint32_t after_sec,
                                  struct schedule_tod_range *out_ranges)
{
	int64_t raw_start = (int64_t)event_second - (int64_t)before_sec;
	int64_t raw_end   = (int64_t)event_second + (int64_t)after_sec;
	uint32_t count = 0;

	if (raw_start < 0 && raw_end <= SOLAR_SECONDS_PER_DAY) {
		/* End spills into previous day — split into two ranges:
		 * Range 0: spillover from previous day (end of today)
		 * Range 1: normal portion from 00:00:00 */
		out_ranges[count].start_second = (uint32_t)(SOLAR_SECONDS_PER_DAY + raw_start);
		out_ranges[count].end_second   = SOLAR_SECONDS_PER_DAY;
		count++;
		out_ranges[count].start_second = 0;
		out_ranges[count].end_second   = (uint32_t)raw_end;
		count++;
	} else if (raw_start >= 0 && raw_end > SOLAR_SECONDS_PER_DAY) {
		/* End spills into next day — split into two ranges:
		 * Range 0: normal portion until midnight
		 * Range 1: spillover into next day from 00:00:00 */
		out_ranges[count].start_second = (uint32_t)raw_start;
		out_ranges[count].end_second   = SOLAR_SECONDS_PER_DAY;
		count++;
		out_ranges[count].start_second = 0;
		out_ranges[count].end_second   = (uint32_t)(raw_end - SOLAR_SECONDS_PER_DAY);
		count++;
	} else {
		/* No split — clamp to [0, 86400] */
		out_ranges[count].start_second = (uint32_t)(raw_start < 0 ? 0 : raw_start);
		out_ranges[count].end_second   = (uint32_t)(raw_end > SOLAR_SECONDS_PER_DAY ? SOLAR_SECONDS_PER_DAY : raw_end);
		count++;
	}

	return count;
}

bool solar_compute_tod_ranges_for_date(const struct schedule_config *cfg,
                                       int year,
                                       int month,
                                       int day,
                                       struct schedule_tod_range *out_ranges,
                                       uint32_t *num_ranges_out)
{
	if (!cfg || !out_ranges || !num_ranges_out) {
		return false;
	}
	*num_ranges_out = 0;

	if (cfg->latitude_e7 == SOLAR_INVALID_LOCATION_COORDINATE ||
	    cfg->longitude_e7 == SOLAR_INVALID_LOCATION_COORDINATE) {
		return false;
	}

	double lat = (double)cfg->latitude_e7 / (double)SOLAR_LOCATION_COORDINATE_SCALE;
	double lon = (double)cfg->longitude_e7 / (double)SOLAR_LOCATION_COORDINATE_SCALE;

	if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
		return false;
	}

	double jd = solar_compute_julian_date(year, month, day, 0.0);
	uint32_t sunrise_sec = 0;
	uint32_t sunset_sec = 0;

	if (!solar_compute_sunrise_sunset_seconds(jd, lat, lon, cfg->time_zone_offset,
	                                         &sunrise_sec, &sunset_sec)) {
		return false;
	}

	/* Apply rounding if configured */
	if (cfg->sunrise_sunset_rounding > 0) {
		uint32_t round_sec = (uint32_t)cfg->sunrise_sunset_rounding * SOLAR_SECONDS_PER_MIN;
		if (round_sec > 0) {
			sunrise_sec = ((sunrise_sec + (round_sec / 2U)) / round_sec) * round_sec;
			sunset_sec  = ((sunset_sec  + (round_sec / 2U)) / round_sec) * round_sec;
			if (sunrise_sec > SOLAR_SECONDS_PER_DAY) sunrise_sec = SOLAR_SECONDS_PER_DAY;
			if (sunset_sec > SOLAR_SECONDS_PER_DAY) sunset_sec = SOLAR_SECONDS_PER_DAY;
		}
	}

	uint32_t slot = 0;
	struct schedule_tod_range tmp[SOLAR_MAX_RANGES_PER_EVENT];
	memset(tmp, 0, sizeof(tmp));

	uint32_t s_bef = (uint32_t)cfg->sunrise_interval_before * SOLAR_SECONDS_PER_MIN;
	uint32_t s_aft = (uint32_t)cfg->sunrise_interval_after * SOLAR_SECONDS_PER_MIN;
	uint32_t ss_bef = (uint32_t)cfg->sunset_interval_before * SOLAR_SECONDS_PER_MIN;
	uint32_t ss_aft = (uint32_t)cfg->sunset_interval_after * SOLAR_SECONDS_PER_MIN;

	switch (cfg->sun_recording_mode) {
	case SUN_MODE_SUNRISE: {
		uint32_t n = solar_compute_sun_ranges(sunrise_sec, s_bef, s_aft, tmp);
		for (uint32_t i = 0; i < n && slot < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
			out_ranges[slot++] = tmp[i];
		}
		break;
	}
	case SUN_MODE_SUNSET: {
		uint32_t n = solar_compute_sun_ranges(sunset_sec, ss_bef, ss_aft, tmp);
		for (uint32_t i = 0; i < n && slot < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
			out_ranges[slot++] = tmp[i];
		}
		break;
	}
	case SUN_MODE_SUNRISE_AND_SUNSET: {
		uint32_t n1 = solar_compute_sun_ranges(sunrise_sec, s_bef, s_aft, tmp);
		for (uint32_t i = 0; i < n1 && slot < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
			out_ranges[slot++] = tmp[i];
		}
		uint32_t n2 = solar_compute_sun_ranges(sunset_sec, ss_bef, ss_aft, tmp);
		for (uint32_t i = 0; i < n2 && slot < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
			out_ranges[slot++] = tmp[i];
		}
		break;
	}
	case SUN_MODE_SUNRISE_TO_SUNSET: {
		int64_t r_start = (int64_t)sunrise_sec - (int64_t)s_bef;
		int64_t r_end   = (int64_t)sunset_sec  + (int64_t)ss_aft;
		out_ranges[0].start_second = (uint32_t)(r_start < 0 ? 0 : r_start);
		out_ranges[0].end_second   = (uint32_t)(r_end > SOLAR_SECONDS_PER_DAY ? SOLAR_SECONDS_PER_DAY : r_end);
		slot = 1;
		break;
	}
	case SUN_MODE_SUNSET_TO_SUNRISE: {
		/* Range 0: 00:00:00 -> sunrise + after */
		int64_t r_end = (int64_t)sunrise_sec + (int64_t)s_aft;
		out_ranges[0].start_second = 0;
		out_ranges[0].end_second   = (uint32_t)(r_end > SOLAR_SECONDS_PER_DAY ? SOLAR_SECONDS_PER_DAY : r_end);

		/* Range 1: sunset - before -> 86400 */
		int64_t r_start = (int64_t)sunset_sec - (int64_t)ss_bef;
		out_ranges[1].start_second = (uint32_t)(r_start < 0 ? 0 : r_start);
		out_ranges[1].end_second   = SOLAR_SECONDS_PER_DAY;
		slot = 2;
		break;
	}
	default:
		return false;
	}

	scheduler_sort_tod_ranges(out_ranges, slot);
	*num_ranges_out = slot;
	return true;
}

