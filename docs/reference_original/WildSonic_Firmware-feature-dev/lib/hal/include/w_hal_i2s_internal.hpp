#ifndef LIB_HAL_INCLUDE_W_HAL_I2S_INTERNAL_HPP
#define LIB_HAL_INCLUDE_W_HAL_I2S_INTERNAL_HPP

#include "w_hal_error_code.hpp"
#include <stdint.h>

// Reference m-clock from crystal
#if (defined(TLV320AIC3204))
constexpr uint32_t CODEC_MCLOCK = 24000000; //(24000000 for TLV320AIC)
#else
constexpr uint32_t CODEC_MCLOCK = 12288000; //(12288000 for sgtl5000)
#endif

/**
 * @brief I2S peripheral input source
 */
enum class i2s_peripheral_input_source_t : uint8_t
{
    LINE_IN = 0, // Line-in pins
    MIC     = 1, // Microphone input pins
    LINE1   = 2, // LINE1 (IN1_L / IN1_R)
    LINE2   = 3, // LINE2 (IN2_L / IN2_R)
    LINE3   = 4, // LINE3 (IN3_L / IN3_R)
};

/**
 * @brief Correct cutoff frequency to be within the range of sampling rate
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_validate_cutoff_frequencies(
    DeviceConfig::config_page_st &device_config,
    bool                          fix);

/**
 * @brief : Validate the I2S Configuration
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_peripheral_validate_config(
    DeviceConfig::config_page_st &device_config,
    bool                          fix);

/**
 * @brief Power up/down I2S peripheral
 *
 * @param on whether to power up or down
 * @return wHalErrorCode error code
 */
wHalErrorCode w_hal_i2s_power_peripheral(bool on);

/**
 * @brief Initialization sequence for I2S Peripheral
 *
 * @param device_config [in]: device configuration
 * @param input_source [in]: input source for I2S peripheral
 *
 * @return wHalErrorCode error code
 */
wHalErrorCode
     w_hal_i2s_init_peripheral(DeviceConfig::config_page_st &device_config,
                               i2s_peripheral_input_source_t input_source);

/**
 * @brief Get I2S Control Bit lengths
 *
 * @param sample_length [out]: bits per sample over interface. Not to be
 * confused with what software shifts down
 * @param bits_per_ws [out]: number of cycles per channel when ws/lrclk
 * toggles
 */
void w_hal_i2s_get_channel_lengths(uint32_t &sample_length,
                                   uint32_t &bits_per_ws);

/**
 * @brief Check whether SCLK and LRCLK is coming from peripheral
 *
 * @return true if SCLK and LRCLK is from peripheral
 * @return false if SCLK and LRCLK is from controller
 */
bool w_hal_i2s_is_sck_from_peripheral();

/**
 * @brief Teardown sequence for I2S Peripheral
 *
 * @return wHalErrorCode error code
 */
wHalErrorCode w_hal_i2s_teardown_peripheral();

#endif // LIB_HAL_INCLUDE_W_HAL_I2S_INTERNAL_HPP
