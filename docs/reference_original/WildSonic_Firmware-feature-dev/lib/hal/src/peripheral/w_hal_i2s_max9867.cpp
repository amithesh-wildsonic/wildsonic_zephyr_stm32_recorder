#if AUDIO_CORE != INVALID_CORE_NUM

#include "w_hal_i2s.hpp" //I2S header file
#include "w_hal_i2s_internal.hpp"

#include "assert.hpp"
#include "device_config.hpp"
#include "trace.hpp"

static const int MIN_GAIN_TIMES_TEN = -180; // Min total gain offered by a codec
static const int MAX_GAIN_TIMES_TEN = 270;  // Max total gain offered by a codec
static const int FIRST_CYCLE_ADC_GAIN_MSB      = 120; // 12Db gain at 0xF
                                                      // ADC_Gain value and
                                                      // 0x0 LineIn_Gain value

static const int SECOND_CYCLE_ADC_GAIN_MSB     = -40; //-4Db gain at 0xF
                                                      // ADC_Gain value and
                                                      // 0x8 LineIn_Gain value

static const int THIRD_CYCLE_ADC_GAIN_LSB      = -50; //-5Db gain at 0x2
                                                      // ADC_Gain value and
                                                      // 0xF LineIn_Gain value

static const int DIFFERENCE_BETWEEN_GAINS      = 10;
static const int HALF_DIFFERENCE_BETWEEN_GAINS = DIFFERENCE_BETWEEN_GAINS / 2;

static int       get_gain_reg_field_values(int      desired_gain_times_ten,
                                           uint8_t &ADC_Gain,
                                           uint8_t &LineIn_Gain);

#if __CORE__ == AUDIO_CORE

#include "w_hal_i2c.hpp"
#include "w_hal_system.hpp"

#include "constants.hpp"

static const uint32_t I2C_FREQUENCY         = 100000; // Standard frequency

static const uint8_t  MAX9867_I2C_CHANNEL   = 0x01;
static const uint16_t MAX9867_SLAVE_ADDRESS = 0x18;
static const uint8_t  MAX9867_REG_ADDRESS_SIZE_BYTES = 1;
static const uint8_t  MAX9867_REG_DATA_SIZE_BYTES    = 1;

/**
 * @brief Register addresses for Maxim MAX9867 Codec
 */
enum class MAX9867_CMD : uint8_t
{
    STATUS                          = 0x00,
    JACK_SENSE                      = 0x01,
    AUX_HIGH                        = 0x02,
    AUX_LOW                         = 0x03,
    INTERRUPT_ENABLE                = 0x04,
    SYSTEM_CLOCK                    = 0x05,
    STEREO_AUDIO_CLOCK_CONTROL_HIGH = 0x06,
    STEREO_AUDIO_CLOCK_CONTROL_LOW  = 0x07,
    INTERFACE_MODE_LOW              = 0x08,
    INTERFACE_MODE_HIGH             = 0x09,
    CODEC_FILTERS                   = 0x0A,
    SIDETONE                        = 0x0B,
    DAC_LEVEL                       = 0x0C,
    ADC_LEVEL                       = 0x0D,
    LEFT_LINE_INPUT_LEVEL           = 0x0E,
    RIGHT_LINE_INPUT_LEVEL          = 0x0F,
    LEFT_VOLUME_CONTROL             = 0x10,
    RIGHT_VOLUME_CONTROL            = 0x11,
    LEFT_MIC_GAIN                   = 0x12,
    RIGHT_MIC_GAIN                  = 0x13,
    ADC_INPUT                       = 0x14,
    MICROPHONE                      = 0x15,
    MODE                            = 0x16,
    SYSTEM_SHUTDOWN                 = 0x17,
    REVISION                        = 0xFF,
};

/**
 * @brief Register address 0x05
 */
union system_clock_control_st
{
    uint8_t dw;
    struct
    {
        uint8_t freq  : 4;
        uint8_t psclk : 2;
        uint8_t rsvd  : 2;
    } bits;
};

/**
 * @brief Register address 0x06
 */
union stereo_audio_clock_control_high_st
{
    uint8_t dw;
    struct
    {
        uint8_t ni_high : 7;
        uint8_t pll     : 1;
    } bits;
};

/**
 * @brief Register address 0x08
 */
union interface_mode_low_st
{
    uint8_t dw;
    struct
    {
        uint8_t rsvd   : 2;
        uint8_t tdm    : 1;
        uint8_t hizoff : 1;
        uint8_t dly    : 1;
        uint8_t bci    : 1;
        uint8_t wci    : 1;
        uint8_t mas    : 1;
    } bits;
};

/**
 * @brief Register address 0x09
 */
union interface_mode_high_st
{
    uint8_t dw;
    struct
    {
        uint8_t bsel    : 3;
        uint8_t dmono   : 1;
        uint8_t lvolfix : 1;
        uint8_t rsvd    : 3;
    } bits;
};

/**
 * @brief Register address 0x0D
 */
union adc_level_st
{
    uint8_t dw;
    struct
    {
        uint8_t avr : 4;
        uint8_t avl : 4;
    } bits;
};

/**
 * @brief Register address 0x0E
 *
 */
union left_line_input_level_st
{
    uint8_t dw;
    struct
    {
        uint8_t ligl  : 4;
        uint8_t rsvd0 : 2;
        uint8_t lilm  : 1;
        uint8_t rsvd1 : 1;
    } bits;
};

/**
 * @brief Register address 0x0F
 *
 */
union right_line_input_level_st
{
    uint8_t dw;
    struct
    {
        uint8_t ligr  : 4;
        uint8_t rsvd0 : 2;
        uint8_t lirm  : 1;
        uint8_t rsvd1 : 1;
    } bits;
};

/**
 * @brief Register address 0x14
 */
union adc_input_st
{
    uint8_t dw;
    struct
    {
        uint8_t auxen   : 1;
        uint8_t auxcal  : 1;
        uint8_t auxgain : 1;
        uint8_t auxcap  : 1;
        uint8_t mxinr   : 2;
        uint8_t mxinl   : 2;
    } bits;
};

/**
 * @brief Register address 0x17
 */
union system_shutdown_st
{
    uint8_t dw;
    struct
    {
        uint8_t adren : 1;
        uint8_t adlen : 1;
        uint8_t daren : 1;
        uint8_t dalen : 1;
        uint8_t rsvd  : 1;
        uint8_t lnren : 1;
        uint8_t lnlen : 1;
        uint8_t shdn_ : 1;
    } bits;
};

/**
 * @brief Write to register on codec
 *
 * @param address register address
 * @param word data to write to register
 */
static void i2c_write(MAX9867_CMD address, uint8_t word)
{
    wHalErrorCode error = w_hal_i2c_write_word<MAX9867_REG_ADDRESS_SIZE_BYTES,
                                               MAX9867_REG_DATA_SIZE_BYTES>(
        MAX9867_I2C_CHANNEL,
        static_cast<uint8_t>(address),
        word,
        false,
        MAX9867_SLAVE_ADDRESS,
        true);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_COMPLETE);
}

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
    wHalErrorCode error = w_hal_i2c_init(MAX9867_I2C_CHANNEL,
                                         whal_i2c_mode::MASTER,
                                         I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Validation should have already been done to fix the sampling rate
    SYSTEM_ASSERT_EQUAL(device_config.audio_sampling_width,
                        BITS_PER_AUDIO_SAMPLE_16);

    // Shutdown codec before configuration
    system_shutdown_st system_shutdown;
    system_shutdown.dw = 0;
    i2c_write(MAX9867_CMD::SYSTEM_SHUTDOWN, system_shutdown.dw);

    // Reset to POR state
    i2c_write(MAX9867_CMD::INTERRUPT_ENABLE, 0x0);
    i2c_write(MAX9867_CMD::SYSTEM_CLOCK, 0x0);
    i2c_write(MAX9867_CMD::STEREO_AUDIO_CLOCK_CONTROL_HIGH, 0x0);
    i2c_write(MAX9867_CMD::STEREO_AUDIO_CLOCK_CONTROL_LOW, 0x0);
    i2c_write(MAX9867_CMD::INTERFACE_MODE_LOW, 0x0);
    i2c_write(MAX9867_CMD::INTERFACE_MODE_HIGH, 0x0);
    i2c_write(MAX9867_CMD::CODEC_FILTERS, 0x0);
    i2c_write(MAX9867_CMD::SIDETONE, 0x0);
    i2c_write(MAX9867_CMD::DAC_LEVEL, 0x0);
    i2c_write(MAX9867_CMD::ADC_LEVEL, 0x0);
    i2c_write(MAX9867_CMD::LEFT_LINE_INPUT_LEVEL, 0x0);
    i2c_write(MAX9867_CMD::RIGHT_LINE_INPUT_LEVEL, 0x0);
    i2c_write(MAX9867_CMD::LEFT_VOLUME_CONTROL, 0x0);
    i2c_write(MAX9867_CMD::RIGHT_VOLUME_CONTROL, 0x0);
    i2c_write(MAX9867_CMD::LEFT_MIC_GAIN, 0x0);
    i2c_write(MAX9867_CMD::RIGHT_MIC_GAIN, 0x0);
    i2c_write(MAX9867_CMD::ADC_INPUT, 0x0);
    i2c_write(MAX9867_CMD::MICROPHONE, 0x0);
    i2c_write(MAX9867_CMD::MODE, 0x0);

    static_assert((CODEC_MCLOCK >= 10000000) && (CODEC_MCLOCK < 60000000),
                  "CODEC M-CLOCK Frequency not supported");

    system_clock_control_st system_clock_control;
    system_clock_control.dw         = 0;
    system_clock_control.bits.psclk = 1 + (CODEC_MCLOCK / 20000000);
    i2c_write(MAX9867_CMD::SYSTEM_CLOCK, system_clock_control.dw);

    // Digital PLL locks on to any externally supplied LRCLK signal
    stereo_audio_clock_control_high_st stereo_audio_clock_control_high;
    stereo_audio_clock_control_high.dw       = 0;
    stereo_audio_clock_control_high.bits.pll = 1;
    i2c_write(MAX9867_CMD::STEREO_AUDIO_CLOCK_CONTROL_HIGH,
              stereo_audio_clock_control_high.dw);

    // I2S-compatible mode, Hi-Z mode is off
    interface_mode_low_st interface_mode_low;
    interface_mode_low.dw          = 0;
    interface_mode_low.bits.hizoff = 1;
    interface_mode_low.bits.dly    = 1;
    i2c_write(MAX9867_CMD::INTERFACE_MODE_LOW, interface_mode_low.dw);

    // Analog mic configuration is not implemented
    SYSTEM_ASSERT_EQUAL(input_source, i2s_peripheral_input_source_t::LINE_IN);

    // Enable line-in only to audio input mixer, Disable analog mic
    adc_input_st adc_input;
    adc_input.dw         = 0;
    adc_input.bits.mxinl = 2;
    if (device_config.num_audio_channels > 1)
    {
        adc_input.bits.mxinr = 2;
    }
    i2c_write(MAX9867_CMD::ADC_INPUT, adc_input.dw);

    uint8_t ADC_Gain = 0, LineIn_Gain = 0;
    // Set ADC gain for both left and right line-in
    int     new_gain_times_ten =
        get_gain_reg_field_values(device_config.mic_gain_times_ten,
                                  ADC_Gain,
                                  LineIn_Gain);

    if (new_gain_times_ten != device_config.mic_gain_times_ten)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_MXC_CPP,
                          "Mic gain 0.1*%d is not supported. "
                          "Setting to 0.1*%d\n",
                          device_config.mic_gain_times_ten,
                          new_gain_times_ten);
    }

    adc_level_st adc_level;
    adc_level.dw       = 0;
    adc_level.bits.avl = ADC_Gain;
    if (device_config.num_audio_channels > 1)
    {
        adc_level.bits.avr = ADC_Gain;
    }
    i2c_write(MAX9867_CMD::ADC_LEVEL, adc_level.dw);

    // Set line-input left gain
    left_line_input_level_st left_line_input_level;
    left_line_input_level.dw        = 0;
    left_line_input_level.bits.lilm = 1;
    left_line_input_level.bits.ligl = LineIn_Gain;
    i2c_write(MAX9867_CMD::LEFT_LINE_INPUT_LEVEL, left_line_input_level.dw);

    // Set line-input right gain
    if (device_config.num_audio_channels > 1)
    {
        right_line_input_level_st right_line_input_level;
        right_line_input_level.dw        = 0;
        right_line_input_level.bits.lirm = 1;
        right_line_input_level.bits.ligr = LineIn_Gain;
        i2c_write(MAX9867_CMD::RIGHT_LINE_INPUT_LEVEL,
                  right_line_input_level.dw);
    }

    // Enable left ADC, left line-in, right ADC, right line-in and power up
    system_shutdown.bits.shdn_ = 1;
    system_shutdown.bits.adlen = 1;
    system_shutdown.bits.lnlen = 1;
    if (device_config.num_audio_channels > 1)
    {
        system_shutdown.bits.adren = 1;
        system_shutdown.bits.lnren = 1;
    }
    i2c_write(MAX9867_CMD::SYSTEM_SHUTDOWN, system_shutdown.dw);

    error = w_hal_i2c_teardown(MAX9867_I2C_CHANNEL);

    return error;
}

/**
 * @brief Get I2S Control Bit lengths
 *
 * @param sample_length [out]: bits per sample over interface. Not to be
 * confused with what software shifts down
 * @param bits_per_ws [out]: number of cycles per channel when ws/lrclk
 * toggles
 */
void w_hal_i2s_get_channel_lengths(uint32_t &sample_length,
                                   uint32_t &bits_per_ws)
{
    sample_length = 16;
    bits_per_ws   = 16;
}

/**
 * @brief Teardown sequence for I2S Peripheral
 *
 * @return wHalErrorCode error code
 */
wHalErrorCode w_hal_i2s_teardown_peripheral()
{
    wHalErrorCode error = w_hal_i2c_init(MAX9867_I2C_CHANNEL,
                                         whal_i2c_mode::MASTER,
                                         I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Turnoff the clock
    system_clock_control_st system_clock_control;
    system_clock_control.dw = 0;
    i2c_write(MAX9867_CMD::SYSTEM_CLOCK, system_clock_control.dw);

    // Shutdown codec
    system_shutdown_st system_shutdown;
    system_shutdown.dw = 0;
    i2c_write(MAX9867_CMD::SYSTEM_SHUTDOWN, system_shutdown.dw);

    return w_hal_i2c_teardown(MAX9867_I2C_CHANNEL);
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
 * @brief Get the gain reg field values object
 *
 * @param desired_gain_times_ten[in]: the gain value in 0.1*Db
 * @param ADC_Gain[out]: a register value to set for specific ADC_Gain
 * @param LineIn_Gain[out]: a register value to set for specific LineIN_Gain
 */
static int get_gain_reg_field_values(int      desired_gain_times_ten,
                                     uint8_t &ADC_Gain,
                                     uint8_t &LineIn_Gain)
{
    int quotient, remainder, reg_val;
    quotient               = desired_gain_times_ten / AUDIO_GAIN_DIVIDER;
    remainder              = abs(desired_gain_times_ten % AUDIO_GAIN_DIVIDER);

    int new_gain_times_ten = desired_gain_times_ten;

    if (desired_gain_times_ten >= MAX_GAIN_TIMES_TEN)
    {
        LineIn_Gain        = 0x0;
        ADC_Gain           = 0x0;

        new_gain_times_ten = MAX_GAIN_TIMES_TEN;
    }
    else if (desired_gain_times_ten <= MIN_GAIN_TIMES_TEN)
    {
        LineIn_Gain        = 0xF;
        ADC_Gain           = 0xF;
        new_gain_times_ten = MIN_GAIN_TIMES_TEN;
    }

    // Here the gain is divided into 3 bands. LineIn_Gain will be constants in
    // different bands,by varying ADC_Gain we will get full range of values
    else
    {
        // In first cycle the LineIn_Gain is set to 0x00 and by varying ADC_Gain
        // from 0x00 to 0x0F,we can get the total gain value in the range of
        // 12Db to 27Db
        if (desired_gain_times_ten >= FIRST_CYCLE_ADC_GAIN_MSB)
        {
            LineIn_Gain = 0x0;
            reg_val     = (MAX_GAIN_TIMES_TEN / AUDIO_GAIN_DIVIDER) - quotient;
        }

        // In Second cycle the LineIn_Gain is set to 0x08 and by varying
        // ADC_Gain from 0x00 to 0x0F, we can get the total gain value in the
        // range of -4Db to 11Db.
        else if (desired_gain_times_ten >= SECOND_CYCLE_ADC_GAIN_MSB)
        {
            LineIn_Gain = 0x8;
            reg_val =
                (FIRST_CYCLE_ADC_GAIN_MSB / AUDIO_GAIN_DIVIDER) - quotient - 1;
        }

        // In third cycle the LineIn_Gain is set to 0x0F and by varying ADC_Gain
        // from 0x00 to 0x0F, we can get the total gain value in the range of
        // -18Db to -5Db
        else
        {
            LineIn_Gain = 0xF;
            reg_val =
                2 + (THIRD_CYCLE_ADC_GAIN_LSB / AUDIO_GAIN_DIVIDER) - quotient;
        }

        if (remainder == 0)
        {
            ADC_Gain = reg_val;
        }
        else
        {
            if (remainder < HALF_DIFFERENCE_BETWEEN_GAINS)
            {
                new_gain_times_ten = quotient * AUDIO_GAIN_DIVIDER;
                ADC_Gain           = reg_val;
            }
            else
            {
                new_gain_times_ten = (quotient + 1) * AUDIO_GAIN_DIVIDER;
                ADC_Gain           = reg_val + 1;
            }
        }
    }

    return new_gain_times_ten;
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
    // Can support sampling rates from 8-48kHz
    if (device_config.sampling_rate > AUDIO_SAMPLE_RATE_48KHz)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_MXC_CPP,
                "Sampling rate %d is not supported. Setting to 48kHz\n",
                device_config.sampling_rate);

            device_config.sampling_rate = AUDIO_SAMPLE_RATE_48KHz;
            wHalErrorCode error =
                w_hal_i2s_validate_cutoff_frequencies(device_config, fix);
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_MXC_CPP,
                              "Sampling rate %d is not supported.\n",
                              device_config.sampling_rate);

            return wHalErrorCode::I2S_INVALID_SAMPLING_RATE;
        }
    }
    else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_8KHz)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_MXC_CPP,
                "Sampling rate %d is not supported. Setting to 8kHz\n",
                device_config.sampling_rate);

            device_config.sampling_rate = AUDIO_SAMPLE_RATE_8KHz;
            wHalErrorCode error =
                w_hal_i2s_validate_cutoff_frequencies(device_config, fix);
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_MXC_CPP,
                              "Sampling rate %d is not supported.\n",
                              device_config.sampling_rate);

            return wHalErrorCode::I2S_INVALID_SAMPLING_RATE;
        }
    }

    // Can support 1-2 channel
    if (device_config.num_audio_channels == 0)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_MXC_CPP,
                              "Number of audio channels %d cannot be 0. "
                              "Setting to 1\n",
                              device_config.num_audio_channels);

            device_config.num_audio_channels = 1;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_MXC_CPP,
                              "Number of audio channels %d cannot be 0\n",
                              device_config.num_audio_channels);

            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }
    else if (device_config.num_audio_channels > 2)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_MXC_CPP,
                "Number of audio channels %d cannot be greater than 2. "
                "Setting to 2\n",
                device_config.num_audio_channels);

            device_config.num_audio_channels = 2;
        }
        else
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_MXC_CPP,
                "Number of audio channels %d cannot be greater than 2.\n",
                device_config.num_audio_channels);

            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }

    // Can only support 16-bit
    if (device_config.audio_sampling_width != BITS_PER_AUDIO_SAMPLE_16)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_MXC_CPP,
                "Sampling width %d is not supported. Setting to 16 bits\n",
                device_config.audio_sampling_width);

            device_config.audio_sampling_width = BITS_PER_AUDIO_SAMPLE_16;
        }
        else
        {

            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_MXC_CPP,
                              "Sampling width %d is not supported.\n",
                              device_config.audio_sampling_width);

            return wHalErrorCode::I2S_INVALID_SAMPLING_WIDTH;
        }
    }

    uint8_t ADC_Gain = 0, LineIn_Gain = 0;
    int     new_gain_times_ten =
        get_gain_reg_field_values(device_config.mic_gain_times_ten,
                                  ADC_Gain,
                                  LineIn_Gain);

    if (new_gain_times_ten != device_config.mic_gain_times_ten)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_MXC_CPP,
                              "Mic gain 0.1*%d is not supported. "
                              "Setting to 0.1*%d\n",
                              device_config.mic_gain_times_ten,
                              new_gain_times_ten);

            device_config.mic_gain_times_ten = new_gain_times_ten;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_MXC_CPP,
                              "Mic gain 0.1*%d is not supported.\n",
                              device_config.mic_gain_times_ten);

            return wHalErrorCode::I2S_INVALID_GAIN;
        }
    }

    return w_hal_i2s_validate_cutoff_frequencies(device_config, fix);
}

#endif // AUDIO_CORE != INVALID_CORE_NUM
