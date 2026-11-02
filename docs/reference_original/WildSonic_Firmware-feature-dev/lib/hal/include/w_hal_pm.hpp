#ifndef LIB_HAL_INCLUDE_W_HAL_PM_HPP
#define LIB_HAL_INCLUDE_W_HAL_PM_HPP

#include "w_hal_error_code.hpp"

/**
 * @brief Initialization for Power Management
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_pm_init();

/**
 * @brief Background handling for all low-level power management
 *
 * @return wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_pm_handler();

/**
 * @brief To get Fuel gauge information
 *
 * @param voltage[out]: in mV
 * @param current[out]: in mA
 * @param remaining_capacity[out]: in mAh
 * @param full_charge_capacity[out]:in mAh
 * @param average_current[out]:in mA
 * @param time_to_empty[out]:in seconds
 * @param average_power[out]:in mW
 * @param temperature[out]: in Centigrade
 * @return wHalErrorCode PM_INVALID_FUEL_GAUGE_DATA if no valid fuel gauge data
 * is available, otherwise NO_ERROR
 */
wHalErrorCode w_hal_pm_get_fuel_gauge_info(int32_t &voltage,
                                           int32_t &current,
                                           int32_t &remaining_capacity,
                                           int32_t &full_charge_capacity,
                                           int32_t &average_current,
                                           int32_t &time_to_empty,
                                           int32_t &average_power,
                                           int32_t &temperature);

/**
 * @brief Program the fuel gauge full charge capacity in Data Flash.
 *
 * @param full_charge_capacity_mah capacity to program in mAh
 * @return wHalErrorCode NO_ERROR on success, otherwise an error code
 */
wHalErrorCode
w_hal_pm_set_fuel_gauge_full_charge_capacity(uint16_t full_charge_capacity_mah);

/**
 * @brief This will print the power management parameters on boot up
 *
 */
void          w_hal_pm_print_info();

/**
 * @brief Teardown the PM
 *
 * @return wHalErrorCode I2C_SHUTDOWN_FAILED if teardown cannot be
 * complete, NO_ERROR otherwise
 */
wHalErrorCode w_hal_pm_teardown();

#endif // LIB_HAL_INCLUDE_W_HAL_PM_HPP
