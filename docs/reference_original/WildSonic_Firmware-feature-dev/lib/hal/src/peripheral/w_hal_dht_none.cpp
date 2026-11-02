#include "w_hal_dht_internal.hpp"

#include "w_hal_error_code.hpp"

/**
 * @brief Get temperature in Celsius
 *
 * @return float temperature in Celsius
 */
float get_temperature_celsius()
{
    return 0;
}

/**
 * @brief Get relative humidity in percentage
 *
 * @return float relative humidity in percentage
 */
float get_relative_humidity()
{
    return 0;
}

/**
 * @brief Initialize the DHT peripheral
 *
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_dht_init_peripheral(void)
{
    return wHalErrorCode::NO_ERROR;
}