#include "w_hal_i2s.hpp"

/**
 * @brief I2S Buffer information
 */
struct i2s_buffer_info_st
{
    void *   p_buffer; // Buffer pointer
    uint32_t size;     // Size of buffer in bytes
};

static i2s_buffer_info_st i2s_buffer_info;

/**
 * @brief : Validate the I2S Configuration
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
w_hal_i2s_validate_config(DeviceConfig::config_page_st &device_config, bool fix)
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief : This function initialises the I2S of the MCU
 *
 * @param device_config device configuration
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_init(DeviceConfig::config_page_st &device_config)
{
    // TODO: Replace with real SIM implementation

    i2s_buffer_info.p_buffer = nullptr;
    i2s_buffer_info.size     = 0;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Check if sample drops are happening or are about to happen
 *
 * @return wHalErrorCode I2S_NEAR_SAMPLE_DROP if sample drops are about to
 * happen, I2S_SAMPLES_DROPPED if samples actually were dropped NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_i2s_check_drop_alert()
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Add empty buffer to receive audio data
 *
 * @param p_buffer : Pointer to the buffer which stores the audio sample
 * @param size : size of the buffer
 * @return wHalErrorCode BUSY if cannot add buffer, NO_ERROR otherwise
 */
wHalErrorCode w_hal_i2s_add_rx_buffer(void *p_buffer, uint32_t size)
{
    if (i2s_buffer_info.p_buffer == nullptr)
    {
        i2s_buffer_info.p_buffer = p_buffer;
        i2s_buffer_info.size     = size;

        return wHalErrorCode::NO_ERROR;
    }
    else
    {
        return wHalErrorCode::BUSY;
    }
}

/**
 * @brief Get filled audio buffer with data
 *
 * @param p_buffer [out] : Pointer to the buffer which stores the audio sample
 * @param size [out] : size of the buffer
 * @return wHalErrorCode I2S_NO_BUFFER_AVAILABLE if no buffer in system, BUSY if
 * working on something, NO_ERROR otherwise
 */
wHalErrorCode w_hal_i2s_get_filled_rx_buffer(void *&p_buffer, uint32_t &size)
{
    // TODO: Replace with real SIM implementation
    static uint16_t count = 0;
    static uint16_t calls = 0;

    if (i2s_buffer_info.p_buffer == nullptr)
    {
        return wHalErrorCode::I2S_NO_BUFFER_AVAILABLE;
    }
    else
    {
        p_buffer = i2s_buffer_info.p_buffer;
        size     = i2s_buffer_info.size;

        if (++calls != (size / sizeof(uint16_t)))
        {
            return wHalErrorCode::I2S_NO_BUFFER_AVAILABLE;
        }

        calls = 0;

        for (uint32_t index = 0; index < (size / sizeof(uint16_t)); index++)
        {
            static_cast<uint16_t *>(p_buffer)[index] = count++;
        }

        i2s_buffer_info.p_buffer = nullptr;

        return wHalErrorCode::NO_ERROR;
    }
}

/**
 * @brief Get the number of samples dropped internally by the I2S driver
 *
 * @return uint32_t : number of samples dropped
 */
uint32_t w_hal_i2s_samples_dropped()
{
    return 0;
}

/**
 * @brief This function is used to process the I2S data
 *
 * @return wHalErrorCode BUSY if in process, NO_ERROR if otherwise idle
 */
wHalErrorCode w_hal_i2s_handler()
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief : Shutdown the I2S
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_teardown()
{
    // TODO: Replace with real SIM implementation
    i2s_buffer_info.p_buffer = nullptr;
    i2s_buffer_info.size     = '\0';
    return wHalErrorCode::NO_ERROR;
}