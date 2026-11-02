#include "w_hal_rtc.hpp"

// pointer to time variable to update
static w_hal_rtc_time_st *p_current_time        = nullptr;
static uint32_t *         p_current_time_update = nullptr;

/**
 * @brief
 * RTC initialization
 *
 * @param p_time time variable to update (always in UTC)
 * @param p_time_update flag to indicate time updated
 * @param time_zone_offset time zone offset (for translation to local time)
 * @return wHalErrorCode error code if failure, NO_ERROR if successful
 */
wHalErrorCode             w_hal_rtc_init(w_hal_rtc_time_st *p_time,
                                         uint32_t *         p_time_update,
                                         int                time_zone_offset)
{
    // Save pointer
    p_current_time        = p_time;
    p_current_time_update = p_time_update;
    p_time->clear();
    p_time->time_seconds = time(0);

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief RTC post initialization
 *
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_rtc_post_init()
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief
 * Sets base time. This will reset timer count as well
 *
 * @param time base time to set
 * @return wHalErrorCode BUSY if could not set up time, NO_ERROR otherwise
 */
wHalErrorCode w_hal_rtc_set_time(w_hal_rtc_time_st &time)
{
    *p_current_time = time;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Get the time from the system
 *
 * @return time_t time in seconds since 1970
 */
time_t w_hal_rtc_get_time()
{
    w_hal_rtc_time_st time;

    w_hal_rtc_get_time(time);

    return time.time_seconds;
}

/**
 * @brief Get the time in nanoseconds from the system
 *
 * @param time [out]: time structure to populate
 */
void w_hal_rtc_get_time(w_hal_rtc_time_st &time)
{
    // TODO: Replace with better implementation
    // For now, just increment second counter every call to this function
    if (p_current_time == nullptr)
    {
        time.time_seconds     = static_cast<time_t>(0);
        time.time_nanoseconds = 0;

        return;
    }

    time = *p_current_time;
}

/**
 * @brief Teardown of the RTC
 *
 * @return wHalErrorCode RTC_TERMINATION_FAILED if teardown cannot be completed,
 * BUSY if busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_rtc_teardown()
{
    return wHalErrorCode::NO_ERROR;
}