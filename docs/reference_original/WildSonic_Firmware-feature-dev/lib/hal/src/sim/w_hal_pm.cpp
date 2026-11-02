#if __CORE__ == PM_CORE

#include "w_hal_pm.hpp"

/**
 * @brief Initialization for Power Management
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_pm_init()
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Background handling for all low-level power management
 *
 * @return wHalErrorCode BUSY if hal is busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_pm_handler()
{
    return wHalErrorCode::NO_ERROR;
}

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
                                           int32_t &temperature)
{
    voltage              = 0;
    current              = 0;
    remaining_capacity   = 0;
    full_charge_capacity = 0;
    average_current      = 0;
    time_to_empty        = 0;
    average_power        = 0;
    temperature          = 0;

    return wHalErrorCode::NO_ERROR;
}

#endif // __CORE__ == PM_CORE

/**
 * @brief Teardown the PM
 *
 * @return wHalErrorCode I2C_SHUTDOWN_FAILED if teardown cannot be
 * complete, NO_ERROR otherwise
 */
wHalErrorCode w_hal_pm_teardown()
{
    return wHalErrorCode::NO_ERROR;
}