#ifndef LIB_HAL_INCLUDE_W_HAL_RTC_HPP
#define LIB_HAL_INCLUDE_W_HAL_RTC_HPP

#include "w_hal_error_code.hpp"

#include <time.h>

#define NSEC_PER_MSEC (1000000ULL)
#define NSEC_PER_SEC  (1000000000ULL)
#define USEC_PER_SEC  (1000000)
#define MSEC_PER_SEC  (1000)
#define SECS_PER_MIN  (60)
#define MINS_PER_HR   (60)
#define HOURS_PER_DAY (24)
#define SECS_PER_HR   (MINS_PER_HR * SECS_PER_MIN)
#define SECS_PER_DAY  (HOURS_PER_DAY * SECS_PER_HR)

/**
 * @brief POD Structure containing time difference between two time structures
 */
class w_hal_rtc_time_delta_st
{
  public:
    // Default constructor
    w_hal_rtc_time_delta_st()                                     = default;

    // Default copy constructor
    w_hal_rtc_time_delta_st(const w_hal_rtc_time_delta_st &delta) = default;

    // Constructor from seconds and nanoseconds
    w_hal_rtc_time_delta_st(const time_t &time, const uint32_t ns)
    {
        seconds     = time;
        nanoseconds = ns;
    }

    bool operator>(const w_hal_rtc_time_delta_st &delta)
    {
        return (seconds > delta.seconds) || ((seconds == delta.seconds) &&
                                             (nanoseconds > delta.nanoseconds));
    }

    bool operator>=(const w_hal_rtc_time_delta_st &delta)
    {
        return (seconds > delta.seconds) ||
               ((seconds == delta.seconds) &&
                (nanoseconds >= delta.nanoseconds));
    }

    bool operator<(const w_hal_rtc_time_delta_st &delta)
    {
        return (seconds < delta.seconds) || ((seconds == delta.seconds) &&
                                             (nanoseconds < delta.nanoseconds));
    }

    bool operator<=(const w_hal_rtc_time_delta_st &delta)
    {
        return (seconds < delta.seconds) ||
               ((seconds == delta.seconds) &&
                (nanoseconds <= delta.nanoseconds));
    }

    w_hal_rtc_time_delta_st &operator=(const w_hal_rtc_time_delta_st &delta)
    {
        seconds     = delta.seconds;
        nanoseconds = delta.nanoseconds;
        return *this;
    }

    w_hal_rtc_time_delta_st &operator+=(const w_hal_rtc_time_delta_st &delta)
    {
        seconds += delta.seconds;
        nanoseconds += delta.nanoseconds;

        // check if we need to carry to seconds
        if (nanoseconds >= NSEC_PER_SEC)
        {
            seconds++;
            nanoseconds -= NSEC_PER_SEC;
        }

        return *this;
    }

    int64_t  seconds;
    uint32_t nanoseconds;
};

/**
 * @brief POD Structure containing time information
 */
class w_hal_rtc_time_st
{
  public:
    // Use default constructor
    w_hal_rtc_time_st()                              = default;

    // Use default copy constructor
    w_hal_rtc_time_st(const w_hal_rtc_time_st &time) = default;

    operator time_t() const
    {
        return time_seconds;
    }

    void clear()
    {
        time_seconds     = 0;
        time_nanoseconds = 0;
    }

    void invalidate()
    {
        time_seconds     = INVALID_SECONDS;
        time_nanoseconds = INVALID_NANOSECONDS;
    }

    bool is_valid() const
    {
        return (time_seconds != INVALID_SECONDS) &&
               (time_nanoseconds != INVALID_NANOSECONDS);
    }

    w_hal_rtc_time_st &operator+=(const w_hal_rtc_time_delta_st &delta)
    {
        if (is_valid())
        {
            time_seconds += delta.seconds;
            time_nanoseconds += delta.nanoseconds;

            // check if we need to carry to seconds
            if (time_nanoseconds >= NSEC_PER_SEC)
            {
                time_seconds++;
                time_nanoseconds -= NSEC_PER_SEC;
            }
        }

        return *this;
    }

    w_hal_rtc_time_st &operator-=(const w_hal_rtc_time_delta_st &delta)
    {
        if (is_valid())
        {
            // check if we need to borrow from seconds
            if (time_nanoseconds < delta.nanoseconds)
            {
                time_seconds--;
                time_nanoseconds += NSEC_PER_SEC;
            }

            time_seconds -= delta.seconds;
            time_nanoseconds -= delta.nanoseconds;

            // check if we need to carry to seconds
            if (time_nanoseconds >= NSEC_PER_SEC)
            {
                time_seconds++;
                time_nanoseconds -= NSEC_PER_SEC;
            }
        }

        return *this;
    }

    w_hal_rtc_time_st &operator=(const time_t &time)
    {
        time_seconds     = time;
        time_nanoseconds = 0;
        return *this;
    }

    w_hal_rtc_time_st &operator=(const w_hal_rtc_time_st &time)
    {
        time_seconds     = time.time_seconds;
        time_nanoseconds = time.time_nanoseconds;
        return *this;
    }

    bool operator==(const w_hal_rtc_time_st &time)
    {
        return (time_seconds == time.time_seconds) &&
               (time_nanoseconds == time.time_nanoseconds);
    }

    bool operator!=(const w_hal_rtc_time_st &time)
    {
        return (time_seconds != time.time_seconds) ||
               (time_nanoseconds != time.time_nanoseconds);
    }

    bool operator<(const w_hal_rtc_time_st &time)
    {
        return (time_seconds < time.time_seconds) ||
               ((time_seconds == time.time_seconds) &&
                (time_nanoseconds < time.time_nanoseconds));
    }

    bool operator<=(const w_hal_rtc_time_st &time)
    {
        return (time_seconds < time.time_seconds) ||
               ((time_seconds == time.time_seconds) &&
                (time_nanoseconds <= time.time_nanoseconds));
    }

    bool operator>(const w_hal_rtc_time_st &time)
    {
        return (time_seconds > time.time_seconds) ||
               ((time_seconds == time.time_seconds) &&
                (time_nanoseconds > time.time_nanoseconds));
    }

    bool operator>=(const w_hal_rtc_time_st &time)
    {
        return (time_seconds > time.time_seconds) ||
               ((time_seconds == time.time_seconds) &&
                (time_nanoseconds >= time.time_nanoseconds));
    }

    /**
     * @brief Subtract two time structures
     *
     * @param time time structure to subtract
     * @param delta [out] : time difference
     */
    void subtract(const w_hal_rtc_time_st &time,
                  w_hal_rtc_time_delta_st &delta) const
    {
        if (!is_valid() || !time.is_valid())
        {
            delta.seconds     = 0;
            delta.nanoseconds = 0;
            return;
        }

        if ((time_seconds < time.time_seconds) ||
            ((time_seconds == time.time_seconds) &&
             (time_nanoseconds < time.time_nanoseconds)))
        {
            delta.seconds     = 0;
            delta.nanoseconds = 0;
            return;
        }

        if (time_nanoseconds < time.time_nanoseconds)
        {
            delta.seconds = time_seconds - time.time_seconds - 1;
            delta.nanoseconds =
                time_nanoseconds + (NSEC_PER_SEC - time.time_nanoseconds);
        }
        else
        {
            delta.seconds     = time_seconds - time.time_seconds;
            delta.nanoseconds = time_nanoseconds - time.time_nanoseconds;
        }
    }

    /**
     * @brief Subtract delta to time structure. Stores result in result time
     * structure
     *
     * @param delta : time difference
     * @param result [out]: time structure to subtract
     */
    void subtract(const w_hal_rtc_time_delta_st &delta,
                  w_hal_rtc_time_st &            result) const
    {
        result = *this;
        result -= delta;
    }

    /**
     * @brief Add delta to time structure. Stores result in result time
     * structure
     *
     * @param delta : time difference
     * @param result [out]: time structure to subtract
     */
    void add(const w_hal_rtc_time_delta_st &delta, w_hal_rtc_time_st &result)
    {
        result = *this;
        result += delta;
    }

    static const time_t   INVALID_SECONDS     = static_cast<time_t>(-1LL);
    static const uint32_t INVALID_NANOSECONDS = static_cast<uint32_t>(-1);

    time_t                time_seconds;     // seconds since 1970
    uint32_t              time_nanoseconds; // nanoseconds since last second
};

/**
 * @brief
 * RTC initialization
 *
 * @param p_time time variable to update (always in UTC)
 * @param p_time_update flag to indicate time updated
 * @param time_zone_offset time zone offset (for translation to local time)
 * @return wHalErrorCode error code if failure, NO_ERROR if successful
 */
wHalErrorCode w_hal_rtc_init(w_hal_rtc_time_st *p_time,
                             uint32_t *         p_time_update,
                             int                time_zone_offset);

/**
 * @brief RTC post initialization
 *
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_rtc_post_init();

/**
 * @brief
 * Sets base time. This will reset timer count as well
 *
 * @param time base time to set
 * @return wHalErrorCode BUSY if could not set up time, NO_ERROR otherwise
 */
wHalErrorCode w_hal_rtc_set_time(w_hal_rtc_time_st &time);

/**
 * @brief Get the time from the system
 */
time_t        w_hal_rtc_get_time();

/**
 * @brief Get the time in nanoseconds from the system
 */
void          w_hal_rtc_get_time(w_hal_rtc_time_st &time);

/**
 * @brief
 * Obtain second timer from time structure
 * @param __time_pointer time structure
 * @return time_t second timer value
 */
inline time_t w_hal_rtc_timegm(tm *__time_pointer)
{
    tm     stage_tm;
    time_t t = mktime(__time_pointer);
    return t + (mktime(localtime_r(&t, &stage_tm)) -
                mktime(gmtime_r(&t, &stage_tm)));
}

/**
 * @brief Deconstruct time zone into components
 *
 * @param time_zone_offset time zone offset
 * @param time_zone_sign[out] sign of the time zone (either '+' or '-')
 * @param tz_hour[out] hour component of the time zone (always positive)
 * @param tz_min[out] minute component of the time zone (always positive)
 */
inline void w_hal_rtc_get_time_zone_components(int       time_zone_offset,
                                               char &    time_zone_sign,
                                               uint16_t &tz_hour,
                                               uint16_t &tz_min)
{
    time_zone_sign = '+';

    if (time_zone_offset < 0)
    {
        time_zone_offset = -time_zone_offset;
        time_zone_sign   = '-';
    }

    tz_hour = static_cast<uint16_t>(time_zone_offset / SECS_PER_HR);
    tz_min =
        static_cast<uint16_t>((time_zone_offset % SECS_PER_HR) / SECS_PER_MIN);
}

/**
 * @brief Teardown of the RTC
 *
 * @return wHalErrorCode RTC_TERMINATION_FAILED if teardown cannot be completed,
 * BUSY if busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_rtc_teardown();

#endif // LIB_HAL_INCLUDE_W_HAL_RTC_HPP
