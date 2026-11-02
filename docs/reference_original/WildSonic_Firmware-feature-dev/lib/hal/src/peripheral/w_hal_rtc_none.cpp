#if __CORE__ == RTC_CORE

#include "w_hal_error_code.hpp"
#include "w_hal_rtc.hpp"
#include "w_hal_rtc_internal.hpp"

/**
 * @brief Initialize the RTC peripheral
 *
 * @param time[in/out] : reference to the time structure. The RTC time will
 * be updated to the time in the structure or the time in the structure will
 * be updated to the RTC time.
 * @return wHalErrorCode NO_ERROR if successful, RTC_TIME_UPDATED if the
 * argument time was updated with the peripheral, error code otherwise
 */
wHalErrorCode w_hal_rtc_init_peripheral(w_hal_rtc_time_st &time)
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Set the time in the RTC peripheral
 *
 * @param time [in]: time structure to set
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_rtc_peripheral_set_time(w_hal_rtc_time_st &time)
{
    return wHalErrorCode::NO_ERROR;
}

#endif // __CORE__ == RTC_CORE