#if (__CORE__ == AUDIO_CORE) || (__CORE__ == CONSOLE_CORE)

#include "solar_calculator.hpp"

// Sun calculation shared constants
static constexpr double DEGREES_PER_CIRCLE = 360.0;
static constexpr double MINUTES_PER_DAY    = 1440.0;
static constexpr double SUN_ZENITH_DEGREES = 90.833;
// Math
static constexpr double DEGREES_PER_RADIAN = 180.0;
static constexpr double ROUND_OFFSET       = 0.5;

/**
 * @brief function to compute the julian date from the date structure
 *
 * @param y year
 * @param m month
 * @param d day
 * @param ut_hours time in hours (can be fractional) in UT (0.0 for midnight)
 * @return double julian date corresponding to the input date and time
 */
double SunCalculator::compute_julian_date(int y, int m, int d, double ut_hours)
{
    // --- Julian Date Constants ---
    constexpr int    JD_GREGORIAN_CENTURY    = 100;
    constexpr int    JD_GREGORIAN_LEAP_CYCLE = 4;
    constexpr int    JD_GREGORIAN_OFFSET     = 2;
    constexpr double JD_DAYS_PER_YEAR        = 365.25;
    constexpr double JD_DAYS_PER_MONTH       = 30.6001;
    constexpr int    JD_YEAR_SHIFT           = 4716;
    constexpr double JD_EPOCH_OFFSET         = 1524.5;
    constexpr double HOURS_IN_DAY            = 24.0;

    // Jan and Feb are treated as months 13 and 14 of the previous year
    if (m <= 2)
    {
        y -= 1;
        m += 12;
    }

    int    A  = y / JD_GREGORIAN_CENTURY;
    int    B  = JD_GREGORIAN_OFFSET - A + (A / JD_GREGORIAN_LEAP_CYCLE);
    // Core Julian Date equation (Fliegel & Van Flandern algorithm).
    // The floor(30.6001 * (m + 1)) term is a math trick that perfectly
    // reproduces the 30/31-day lengths of the months without needing a lookup
    // array.
    double jd = floor(JD_DAYS_PER_YEAR * (y + JD_YEAR_SHIFT)) +
                floor(JD_DAYS_PER_MONTH * (m + 1)) + d + B - JD_EPOCH_OFFSET +
                (ut_hours / HOURS_IN_DAY);
    return jd;
}

/**
 * @brief Converts degrees to radians
 *
 * @param degrees angle in degrees
 * @return double angle in radians
 */
static double degrees_to_radians(double degrees)
{
    return degrees * static_cast<double>(PI) / DEGREES_PER_RADIAN;
}

/**
 * @brief Converts radians to degrees
 *
 * @param radians angle in radians
 * @return double angle in degrees
 */
static double radians_to_degrees(double radians)
{
    return radians * DEGREES_PER_RADIAN / static_cast<double>(PI);
}

/**
 * @brief Normalize degrees to the range [0, 360)
 *
 * @param degrees angle in degrees
 * @return double normalized angle in degrees
 */
static double normalize_degrees(double degrees)
{
    // Normalization Constant
    constexpr double NORMALIZED_MIN = 0.0;
    degrees                         = fmod(degrees, DEGREES_PER_CIRCLE);
    if (degrees < NORMALIZED_MIN)
    {
        degrees += DEGREES_PER_CIRCLE;
    }
    return degrees;
}

/**
 * @brief Normalize a fractional day to the range [0, 1)
 *
 * @param fraction fractional day
 * @return double normalized fractional day
 */
static double normalize_day_fraction(double fraction)
{
    return fraction - floor(fraction);
}

/**
 * @brief Convert a fractional day to seconds, with proper handling of edge
 * cases.
 *
 * @param fraction fractional day
 * @return uint32_t seconds corresponding to the fractional day, normalized to
 * the range [0, SECONDS_PER_DAY)
 */
static uint32_t day_fraction_to_seconds(double fraction)
{
    uint32_t seconds = static_cast<uint32_t>(floor(
        (normalize_day_fraction(fraction) * SECONDS_PER_DAY) + ROUND_OFFSET));

    if (seconds >= SECONDS_PER_DAY)
    {
        seconds %= SECONDS_PER_DAY;
    }

    return seconds;
}

/**
 * @brief Computes sunrise and sunset time in seconds from 00:00:00 local time
 *        using the NOAA Solar Calculator algorithm.
 *
 * @param julian_date      Julian day of the target date
 * @param latitude         Location latitude in degrees
 * @param longitude        Location longitude in degrees
 * @param time_zone_offset Offset from UTC in seconds
 * @param sunrise_second[out] Calculated sunrise time in seconds from
 * midnight
 * @param sunset_second[out] Calculated sunset time in seconds from midnight
 * @return true if the sun rises and sets on this day, false if polar day/night.
 */
bool SunCalculator::compute_sunrise_sunset_seconds(double    julian_date,
                                                   double    latitude,
                                                   double    longitude,
                                                   int       time_zone_offset,
                                                   uint32_t &sunrise_second,
                                                   uint32_t &sunset_second)
{
    // Julian calendar constants
    constexpr double JD_J2000_EPOCH          = 2451545.0;
    constexpr double DAYS_PER_JULIAN_CENTURY = 36525.0;

    // Solar noon calculation
    constexpr double MINUTES_PER_HALF_DAY    = 720.0;
    constexpr double MINUTES_PER_DEGREE_ROT  = 4.0;

    // Sun Mean Longitude
    constexpr double SUN_MEAN_LONG_C0        = 280.46646;
    constexpr double SUN_MEAN_LONG_C1        = 36000.76983;
    constexpr double SUN_MEAN_LONG_C2        = 0.0003032;

    // Sun Mean Anomaly
    constexpr double SUN_MEAN_ANOM_C0        = 357.52911;
    constexpr double SUN_MEAN_ANOM_C1        = 35999.05029;
    constexpr double SUN_MEAN_ANOM_C2        = 0.0001537;

    // Earth Orbit Eccentricity
    constexpr double EARTH_ECCENTRICITY_C0   = 0.016708634;
    constexpr double EARTH_ECCENTRICITY_C1   = 0.000042037;
    constexpr double EARTH_ECCENTRICITY_C2   = 0.0000001267;

    // Sun Equation of Center
    constexpr double SUN_EQ_CENTER_C0        = 1.914602;
    constexpr double SUN_EQ_CENTER_C1        = 0.004817;
    constexpr double SUN_EQ_CENTER_C2        = 0.000014;
    constexpr double SUN_EQ_CENTER_C3        = 0.019993;
    constexpr double SUN_EQ_CENTER_C4        = 0.000101;
    constexpr double SUN_EQ_CENTER_C5        = 0.000289;

    // Sun Apparent Longitude Corrections
    constexpr double SUN_OMEGA_BASE          = 125.04;
    constexpr double SUN_OMEGA_SCALE         = 1934.136;
    constexpr double APPARENT_LONG_CORR1     = 0.00569;
    constexpr double APPARENT_LONG_CORR2     = 0.00478;

    // Obliquity of Ecliptic
    constexpr double OBLIQUITY_DEG           = 23.0;
    constexpr double OBLIQUITY_MIN           = 26.0;
    constexpr double OBLIQUITY_SEC_BASE      = 21.448;
    constexpr double OBLIQUITY_SEC_SCALE1    = 46.815;
    constexpr double OBLIQUITY_SEC_SCALE2    = 0.00059;
    constexpr double OBLIQUITY_SEC_SCALE3    = 0.001813;
    constexpr double OBLIQUITY_CORR          = 0.00256;

    // Equation of Time multipliers
    constexpr double EOT_MULT_Y2             = 0.5;
    constexpr double EOT_MULT_E2             = 1.25;

    // --- Math & Conversion Constants ---
    constexpr double HARMONIC_2              = 2.0;
    constexpr double HARMONIC_3              = 3.0;
    constexpr double HARMONIC_4              = 4.0;
    constexpr double HALF_ANGLE_DIV          = 2.0;

    constexpr double ARC_SECS_PER_ARC_MIN    = 60.0;
    constexpr double ARC_MINS_PER_DEGREE     = 60.0;
    constexpr double MINS_PER_HOUR           = 60.0;
    constexpr double SECS_PER_HOUR           = 3600.0;

    constexpr double VALID_COSINE_MIN        = -1.0;
    constexpr double VALID_COSINE_MAX        = 1.0;

    // Time Scale: Convert Julian date to "Julian Centuries" since the J2000
    // epoch. This is the standard time variable (T) used in almost all
    // astronomical formulas.
    const double     julian_century =
        (julian_date - JD_J2000_EPOCH) / DAYS_PER_JULIAN_CENTURY;

    // Base Orbital Elements: Calculate Earth's idealized position and orbit
    // shape
    const double geom_mean_longitude = normalize_degrees(
        SUN_MEAN_LONG_C0 +
        (julian_century *
         (SUN_MEAN_LONG_C1 + (julian_century * SUN_MEAN_LONG_C2))));
    const double geom_mean_anomaly =
        SUN_MEAN_ANOM_C0 +
        (julian_century *
         (SUN_MEAN_ANOM_C1 - (SUN_MEAN_ANOM_C2 * julian_century)));
    const double earth_orbit_eccentricity =
        EARTH_ECCENTRICITY_C0 -
        (julian_century *
         (EARTH_ECCENTRICITY_C1 + (EARTH_ECCENTRICITY_C2 * julian_century)));

    // Equation of Center: Adjusts the mean position to account for the
    // Earth's elliptical (non-circular) orbit.
    const double mean_anomaly_rad = degrees_to_radians(geom_mean_anomaly);
    const double sun_eq_center =
        (sin(mean_anomaly_rad) *
         (SUN_EQ_CENTER_C0 -
          (julian_century *
           (SUN_EQ_CENTER_C1 + (SUN_EQ_CENTER_C2 * julian_century))))) +
        (sin(HARMONIC_2 * mean_anomaly_rad) *
         (SUN_EQ_CENTER_C3 - (SUN_EQ_CENTER_C4 * julian_century))) +
        (sin(HARMONIC_3 * mean_anomaly_rad) * SUN_EQ_CENTER_C5);

    // Apparent Longitude: The actual position of the Sun, corrected for
    // Earth's nutation (axis wobble) and the travel time of light.
    const double sun_true_longitude = geom_mean_longitude + sun_eq_center;
    const double omega = SUN_OMEGA_BASE - (SUN_OMEGA_SCALE * julian_century);
    const double sun_apparent_longitude =
        sun_true_longitude - APPARENT_LONG_CORR1 -
        (APPARENT_LONG_CORR2 * sin(degrees_to_radians(omega)));

    // Earth's Axial Tilt (Obliquity) and Sun's Declination.
    // Declination is essentially the latitude on Earth where the sun is
    // directly overhead.
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
    const double corrected_obliquity_rad =
        degrees_to_radians(corrected_obliquity);
    const double sun_declination =
        asin(sin(corrected_obliquity_rad) *
             sin(degrees_to_radians(sun_apparent_longitude)));

    // Equation of Time (EoT): The difference in minutes between "clock time"
    // and "sundial time" due to orbit eccentricity and axial tilt.
    const double y = tan(corrected_obliquity_rad / HALF_ANGLE_DIV) *
                     tan(corrected_obliquity_rad / HALF_ANGLE_DIV);
    const double geom_mean_longitude_rad =
        degrees_to_radians(geom_mean_longitude);
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

    // Solar Noon: Calculate the exact fractional day (0.0 to 1.0) when the
    // sun crosses the local meridian (highest point in the sky).
    const double time_zone_hours =
        static_cast<double>(time_zone_offset) / SECS_PER_HOUR;
    const double solar_noon_fraction =
        (MINUTES_PER_HALF_DAY - (MINUTES_PER_DEGREE_ROT * longitude) -
         equation_of_time + (time_zone_hours * MINS_PER_HOUR)) /
        MINUTES_PER_DAY;

    // Hour Angle: The angular distance between the sun at noon and at the
    // horizon.
    const double latitude_rad   = degrees_to_radians(latitude);
    const double hour_angle_arg = (cos(degrees_to_radians(SUN_ZENITH_DEGREES)) /
                                   (cos(latitude_rad) * cos(sun_declination))) -
                                  (tan(latitude_rad) * tan(sun_declination));

    // Check if the sun never rises or never sets (e.g., Polar regions in
    // summer/winter)
    if ((hour_angle_arg < VALID_COSINE_MIN) ||
        (hour_angle_arg > VALID_COSINE_MAX))
    {
        return false;
    }

    // Final Calculation: Apply the hour angle offset to solar noon to get
    // the sunrise and sunset day-fractions, then convert to raw seconds.
    const double hour_angle_degrees = radians_to_degrees(acos(hour_angle_arg));
    const double sunrise_fraction =
        solar_noon_fraction - (hour_angle_degrees / DEGREES_PER_CIRCLE);
    const double sunset_fraction =
        solar_noon_fraction + (hour_angle_degrees / DEGREES_PER_CIRCLE);

    sunrise_second = day_fraction_to_seconds(sunrise_fraction);
    sunset_second  = day_fraction_to_seconds(sunset_fraction);

    return sunrise_second < sunset_second;
}

#endif // (__CORE__ == AUDIO_CORE) || (__CORE__ == CONSOLE_CORE)