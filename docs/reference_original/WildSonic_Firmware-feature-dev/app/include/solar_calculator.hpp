#ifndef APP_INCLUDE_SOLAR_CALCULATOR_HPP
#define APP_INCLUDE_SOLAR_CALCULATOR_HPP

#include "constants.hpp"
#include <cmath>
#include <stdint.h>

class SunCalculator
{
  public:
    // function to compute the julian date from the date structure
    static double compute_julian_date(int y, int m, int d, double ut_hours);

    // Function to compute sunrise and sunset time in seconds from 00:00:00
    // local midnight using the NOAA Solar Calculator algorithm. Returns true if
    // the sun rises and sets on this day, false if polar day/night.
    static bool   compute_sunrise_sunset_seconds(double    julian_date,
                                                 double    latitude,
                                                 double    longitude,
                                                 int       time_zone_offset,
                                                 uint32_t &sunrise_second,
                                                 uint32_t &sunset_second);
};
#endif // APP_INCLUDE_SOLAR_CALCULATOR_HPP