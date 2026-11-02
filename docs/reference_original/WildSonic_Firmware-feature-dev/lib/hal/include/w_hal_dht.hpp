#ifndef LIB_HAL_W_HAL_DHT_HPP
#define LIB_HAL_W_HAL_DHT_HPP

#include "w_hal_error_code.hpp"
#include <stdint.h>

/**
 * @brief Initialize DHT peripheral
 *
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_dht_init(void);

/**
 * @brief Get the temperature and humidity from DHT peripheral
 *
 * @param temperature [out] temperature in Celsius
 * @param humidity [out] relative humidity in percentage
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_dht_get_env_info(float &temperature, float &humidity);

#endif // LIB_HAL_W_HAL_DHT_HPP