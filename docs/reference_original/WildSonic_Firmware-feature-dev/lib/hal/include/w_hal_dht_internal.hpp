#ifndef LIB_HAL_W_HAL_DHT_INTERNAL_HPP
#define LIB_HAL_W_HAL_DHT_INTERNAL_HPP

#include "w_hal_error_code.hpp"
#include <stdint.h>

/**
 * @brief Get temperature in Celsius
 *
 * @return float temperature in Celsius
 */
float         get_temperature_celsius();

/**
 * @brief Get relative humidity in percentage
 *
 * @return float relative humidity in percentage
 */
float         get_relative_humidity();

/**
 * @brief Initialize the DHT peripheral
 *
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_dht_init_peripheral(void);

#endif // LIB_HAL_W_HAL_DHT_INTERNAL_HPP