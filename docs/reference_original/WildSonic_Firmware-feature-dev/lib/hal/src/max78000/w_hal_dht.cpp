#include "w_hal_dht.hpp"
#include "w_hal_dht_internal.hpp"
#include "w_hal_error_code.hpp"

/**
 * @brief Initialize DHT peripheral
 *
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_dht_init(void)
{
    wHalErrorCode error = w_hal_dht_init_peripheral();
    return error;
}

/**
 * @brief Get the temperature and humidity from DHT peripheral
 *
 * @param temperature [out] temperature in Celsius
 * @param humidity [out] relative humidity in percentage
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_dht_get_env_info(float &temperature, float &humidity)
{
    temperature = get_temperature_celsius();
    humidity    = get_relative_humidity();

    return wHalErrorCode::NO_ERROR;
}