#ifndef LIB_HAL_INCLUDE_W_HAL_RTC_INTERNAL_HPP
#define LIB_HAL_INCLUDE_W_HAL_RTC_INTERNAL_HPP

#include "w_hal_error_code.hpp"
#include <stdint.h>

/**
 * @brief Initialize the RTC peripheral
 *
 * @param time[in/out] : reference to the time structure. The RTC time will
 * be updated to the time in the structure or the time in the structure will
 * be updated to the RTC time.
 * @return wHalErrorCode NO_ERROR if successful, RTC_TIME_UPDATED if the
 * argument time was updated with the peripheral, error code otherwise
 */
wHalErrorCode w_hal_rtc_init_peripheral(w_hal_rtc_time_st &time);

/**
 * @brief Set the time in the RTC peripheral
 *
 * @param time [in]: time structure to set
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_rtc_peripheral_set_time(w_hal_rtc_time_st &time);

#endif // LIB_HAL_INCLUDE_W_HAL_RTC_INTERNAL_HPP