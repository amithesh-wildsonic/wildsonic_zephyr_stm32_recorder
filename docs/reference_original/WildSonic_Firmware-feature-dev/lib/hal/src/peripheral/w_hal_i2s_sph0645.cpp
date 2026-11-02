#if AUDIO_CORE != INVALID_CORE_NUM

#include "w_hal_error_code.hpp"
#include "w_hal_i2s.hpp" //I2S header file
#include "w_hal_i2s_internal.hpp"

#include "assert.hpp"
#include "constants.hpp"
#include "device_config.hpp"
#include "trace.hpp"

#if __CORE__ == AUDIO_CORE

#include "w_hal_i2c.hpp"
#include "w_hal_system.hpp"

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
                          i2s_peripheral_input_source_t input_source)
{
    return w_hal_i2s_power_peripheral(true);
}

/**
 * @brief Get I2S Control Bit lengths
 *
 * @param sample_length [out]: bits per sample over interface. Not to be
 * confused with what software shifts down
 * @param bits_per_ws [out]: number of cycles per channel when ws/lrclk toggles
 */
void w_hal_i2s_get_channel_lengths(uint32_t &sample_length,
                                   uint32_t &bits_per_ws)
{
    sample_length = 18;
    bits_per_ws   = 32;
}

/**
 * @brief Teardown sequence for I2S Peripheral
 *
 * @return wHalErrorCode error code
 */
wHalErrorCode w_hal_i2s_teardown_peripheral()
{
    return w_hal_i2s_power_peripheral(false);
}

#endif // __CORE__ == AUDIO_CORE

/**
 * @brief Check whether SCLK and LRCLK is coming from peripheral
 *
 * @return true if SCLK and LRCLK is from peripheral
 * @return false if SCLK and LRCLK is from controller
 */
bool w_hal_i2s_is_sck_from_peripheral()
{
    return false;
}

/**
 * @brief : Validate the I2S Configuration
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_peripheral_validate_config(
    DeviceConfig::config_page_st &device_config,
    bool                          fix)
{
    if (device_config.sampling_rate > AUDIO_SAMPLE_RATE_64KHz)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_SPH_CPP,
                "Sampling rate %d is not supported. Setting to 64kHz\n",
                device_config.sampling_rate);

            device_config.sampling_rate = AUDIO_SAMPLE_RATE_64KHz;
            wHalErrorCode error =
                w_hal_i2s_validate_cutoff_frequencies(device_config, fix);
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                              "Sampling rate %d is not supported.\n",
                              device_config.sampling_rate);

            return wHalErrorCode::I2S_INVALID_SAMPLING_RATE;
        }
    }

    if (device_config.num_audio_channels != 1)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_SPH_CPP,
                "Number of channels %d is not supported. Setting to 1\n",
                device_config.num_audio_channels);

            device_config.num_audio_channels = 1;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                              "Number of channels %d is not supported.\n",
                              device_config.num_audio_channels);

            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }

    if (device_config.audio_sampling_width != BITS_PER_AUDIO_SAMPLE_16)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_SPH_CPP,
                "Sampling width %d is not supported. Setting to 16 bits\n",
                device_config.audio_sampling_width);

            device_config.audio_sampling_width = BITS_PER_AUDIO_SAMPLE_16;
        }
        else
        {

            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                              "Sampling width %d is not supported.\n",
                              device_config.audio_sampling_width);

            return wHalErrorCode::I2S_INVALID_SAMPLING_WIDTH;
        }
    }

    if (device_config.mic_gain_times_ten != 0)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_SPH_CPP,
                "Mic gain 0.1*%d is not supported. Setting to 0\n",
                device_config.mic_gain_times_ten);

            device_config.mic_gain_times_ten = 0;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                              "Mic gain 0.1*%d is not supported.\n",
                              device_config.mic_gain_times_ten);

            return wHalErrorCode::I2S_INVALID_GAIN;
        }
    }

    // Check lower and upper cutoff frequencies should be min and max values
    // respectively
    const uint32_t max_cutoff_frequency =
        (device_config.sampling_rate / 2) + (device_config.sampling_rate % 2);
    if (device_config.upper_cutoff_frequency != max_cutoff_frequency)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                              "Upper cutoff frequency %d is not supported. "
                              "Setting to %d\n",
                              device_config.upper_cutoff_frequency,
                              max_cutoff_frequency);

            device_config.upper_cutoff_frequency = max_cutoff_frequency;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                              "Upper cutoff frequency %d is not supported.\n",
                              device_config.upper_cutoff_frequency);

            return wHalErrorCode::I2S_INVALID_UPPER_CUTOFF_FREQUENCY;
        }
    }

    if (device_config.lower_cutoff_frequency != 0)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                              "Lower cutoff frequency %d is not supported. "
                              "Setting to %d\n",
                              device_config.lower_cutoff_frequency,
                              0);

            device_config.lower_cutoff_frequency = 0;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                              "Lower cutoff frequency %d is not supported.\n",
                              device_config.lower_cutoff_frequency);

            return wHalErrorCode::I2S_INVALID_LOWER_CUTOFF_FREQUENCY;
        }
    }

    return wHalErrorCode::NO_ERROR;
}

#endif // AUDIO_CORE != INVALID_CORE_NUM
