#ifndef LIB_HAL_INCLUDE_W_HAL_I2S_HPP
#define LIB_HAL_INCLUDE_W_HAL_I2S_HPP

#include "device_config.hpp"

#include "w_hal_error_code.hpp"
#include <stdint.h>

/**
 * @brief : Validate the I2S Configuration
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
w_hal_i2s_validate_config(DeviceConfig::config_page_st &device_config,
                          bool                          fix);

/**
 * @brief : This function initialises the I2S of the MCU
 *
 * @param device_config device configuration
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_init(DeviceConfig::config_page_st &device_config);

/**
 * @brief Check if sample drops are happening or are about to happen
 *
 * @return wHalErrorCode I2S_NEAR_SAMPLE_DROP if sample drops are about to
 * happen, I2S_SAMPLES_DROPPED if samples actually were dropped NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_i2s_check_drop_alert();

/**
 * @brief Add empty buffer to receive audio data
 *
 * @param p_buffer : Pointer to the buffer which stores the audio sample
 * @param size : size of the buffer
 * @return wHalErrorCode BUSY if cannot add buffer, NO_ERROR otherwise
 */
wHalErrorCode w_hal_i2s_add_rx_buffer(void *p_buffer, uint32_t size);

/**
 * @brief Get filled audio buffer with data
 *
 * @param p_buffer [out] : Pointer to the buffer which stores the audio sample
 * @param size [out] : size of the buffer
 * @return wHalErrorCode I2S_NO_BUFFER_AVAILABLE if no buffer in system, BUSY if
 * working on something, NO_ERROR otherwise
 */
wHalErrorCode w_hal_i2s_get_filled_rx_buffer(void *&p_buffer, uint32_t &size);

/**
 * @brief Get the number of samples dropped internally by the I2S driver
 *
 * @return uint32_t : number of samples dropped
 */
uint32_t      w_hal_i2s_samples_dropped();

/**
 * @brief This function is used to process the I2S data
 *
 * @return wHalErrorCode BUSY if in process, NO_ERROR if otherwise idle
 */
wHalErrorCode w_hal_i2s_handler();

/**
 * @brief : Shutdown the I2S
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_teardown();

#endif // LIB_HAL_INCLUDE_W_HAL_I2S_HPP
