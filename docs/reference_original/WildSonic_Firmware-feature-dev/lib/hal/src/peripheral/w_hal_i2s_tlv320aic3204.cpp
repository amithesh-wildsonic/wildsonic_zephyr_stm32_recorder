#if AUDIO_CORE != INVALID_CORE_NUM

#include "w_hal_i2s.hpp" //I2S header file
#include "w_hal_i2s_internal.hpp"

#include "device_config.hpp"
#include "trace.hpp"

#include <math.h>
#include <string.h>

#define TLV320AIC3204

static const bool I2S_PERIPHERAL_MASTER = true;

// Configure whether MICBIAS is enabled: true/false
constexpr bool    TLV320_ENABLE_MICBIAS = true;

// Configure MICBIAS voltage: 0x40 = 1.04V, 0x50 = 1.425V, 0x60 = 2.075V (AVDD)
// || 0x48 = 1.25V, 0x58 = 1.7V, 0x68 = 2.5V (LDOIN)
constexpr uint8_t TLV320_MICBIAS_VAL    = 0x60;

// Loopback modes: 0 = None (default), 1 = Analog Bypass, 2 = Digital I2S
// Loopback
constexpr int     TLV320_LOOPBACK_MODE  = 0;

#include "assert.hpp"
#include "constants.hpp"

#if __CORE__ == AUDIO_CORE

#include "w_hal_i2c.hpp"
#include "w_hal_system.hpp"

static const uint32_t I2C_FREQUENCY         = 100000; // Standard frequency
static const uint8_t  AIC3204_I2C_CHANNEL   = 0x01;
static const uint8_t  AIC3204_SLAVE_ADDRESS = 0x18;
static const uint8_t  AIC3204_REG_ADDR_SIZE = 1; // 1-byte register address
static const uint8_t  AIC3204_REG_DATA_SIZE = 1; // 1-byte register data

static int8_t         g_current_page        = -1;
static uint32_t       i2s_sample_length     = BITS_PER_AUDIO_SAMPLE_16;
static uint8_t        g_aosr                = 128;
static uint8_t        g_madc                = 16;

/**
 * @brief Select the active register page of TLV320AIC3204
 *
 * @param page page number to select
 * @return wHalErrorCode error code
 */
static wHalErrorCode  select_page(uint8_t page)
{
    if (g_current_page == static_cast<int8_t>(page))
    {
        return wHalErrorCode::NO_ERROR;
    }

    wHalErrorCode error =
        w_hal_i2c_write_word<AIC3204_REG_ADDR_SIZE, AIC3204_REG_DATA_SIZE>(
            AIC3204_I2C_CHANNEL,
            0x00, // Register 0 is Page Select
            page,
            false,
            AIC3204_SLAVE_ADDRESS,
            true); // blocking write

    if (error == wHalErrorCode::I2C_WRITE_COMPLETE)
    {
        g_current_page = static_cast<int8_t>(page);
        return wHalErrorCode::NO_ERROR;
    }

    return error;
}

/**
 * @brief Write a value to a register on a specific page
 *
 * @param page page number
 * @param reg register address
 * @param value value to write
 * @return wHalErrorCode error code
 */
static wHalErrorCode tlvread(uint8_t page, uint8_t reg, uint8_t &value);

static void          tlvwrite(uint8_t page, uint8_t reg, uint8_t value)
{
    // LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_TLV_CPP, "tlvwrite: page %d, reg
    // 0x%x, value 0x%x \n\r", page, reg, value);

    uint8_t dummy = 0;
    tlvread(page, reg, dummy);

    wHalErrorCode error = select_page(page);

    error = w_hal_i2c_write_word<AIC3204_REG_ADDR_SIZE, AIC3204_REG_DATA_SIZE>(
        AIC3204_I2C_CHANNEL,
        reg,
        value,
        false,
        AIC3204_SLAVE_ADDRESS,
        true); // blocking write

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_COMPLETE);
}

/**
 * @brief Read a value from a register on a specific page
 *
 * @param page page number
 * @param reg register address
 * @param value [out] variable to store the read value
 * @return wHalErrorCode error code
 */
static wHalErrorCode tlvread(uint8_t page, uint8_t reg, uint8_t &value)
{
    wHalErrorCode error = select_page(page);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    uint8_t address_bytes[AIC3204_REG_ADDR_SIZE] = {reg};
    uint8_t read_bytes[AIC3204_REG_DATA_SIZE]    = {0xFF};

    error = w_hal_i2c_write_read_data(AIC3204_I2C_CHANNEL,
                                      address_bytes,
                                      AIC3204_REG_ADDR_SIZE,
                                      read_bytes,
                                      AIC3204_REG_DATA_SIZE,
                                      false,
                                      AIC3204_SLAVE_ADDRESS,
                                      true); // blocking write-read

    if (error == wHalErrorCode::I2C_READ_COMPLETE)
    {
        value = read_bytes[0];
        wHalErrorCode clear_error =
            w_hal_i2c_clear_completion_status(AIC3204_I2C_CHANNEL);
        if (clear_error != wHalErrorCode::NO_ERROR)
        {
            return clear_error;
        }
        return wHalErrorCode::NO_ERROR;
    }

    return error;
}

/**
 * @brief Debug helper function to dump all registers of page 0 and page 1
 *
 * @return wHalErrorCode error code
 */
wHalErrorCode w_hal_i2s_peripheral_dump_registers()
{
    LOG_MSG(TraceFile::W_HAL_I2S_TLV_CPP,
            "=== DUMPING TLV320AIC3204 REGISTERS ===\n");
    for (uint8_t page : {0, 1})
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_TLV_CPP,
                          "--- Page %d ---\n",
                          page);
        for (uint16_t reg = 0; reg <= 127; reg++)
        {
            uint8_t       val   = 0;
            wHalErrorCode error = tlvread(page, reg, val);
            if (error == wHalErrorCode::NO_ERROR)
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_TLV_CPP,
                                  "Reg 0x%02X (Dec %d): 0x%02X\n",
                                  reg,
                                  reg,
                                  val);
            }
            else
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_TLV_CPP,
                                  "Reg 0x%02X (Dec %d): READ ERROR %d\n",
                                  reg,
                                  reg,
                                  static_cast<int>(error));
            }
        }
    }
    LOG_MSG(TraceFile::W_HAL_I2S_TLV_CPP,
            "=======================================\n");
    return wHalErrorCode::NO_ERROR;
}

#endif // __CORE__ == AUDIO_CORE

/**
 * @brief Page numbers for TI TLV320AIC3204 register map
 */
enum class TLV320AIC3204_PAGE : uint8_t
{
    PAGE_0  = 0x00,
    PAGE_1  = 0x01,
    PAGE_8  = 0x08,
    PAGE_9  = 0x09,
    PAGE_16 = 0x10,
    PAGE_26 = 0x1A,
    PAGE_34 = 0x22,
    PAGE_44 = 0x2C,
    PAGE_45 = 0x2D,
    PAGE_52 = 0x34,
    PAGE_62 = 0x3E,
    PAGE_70 = 0x46,
};

/**
 * @brief Page 0 register offsets for TI TLV320AIC3204 Codec
 */
enum class TLV320AIC3204_PAGE_0_REG : uint8_t
{
    PAGE_SELECT        = 0x00,
    SW_RESET           = 0x01,
    CLK_SET_1_MUX      = 0x04,
    CLK_SET_2_PLLPR    = 0x05,
    CLK_SET_3_PLLJ     = 0x06,
    CLK_SET_4_PLLDMSB  = 0x07,
    CLK_SET_5_PLLDLSB  = 0x08,
    CLK_SET_6_NDAC     = 0x0B,
    CLK_SET_7_MDAC     = 0x0C,
    DOSR_SET_1_MSB     = 0x0D,
    DOSR_SET_2_LSB     = 0x0E,
    CLK_SET_8_NADC     = 0x12,
    CLK_SET_9_MADC     = 0x13,

    AOSR               = 0x14,
    CLK_SET_10_MUX2    = 0x19,
    CLK_SET_11_CLKOUTM = 0x1A,
    AUD_IFACE_SET_1    = 0x1B,
    AUD_IFACE_SET_2    = 0x1C,
    AUD_IFACE_SET_3    = 0x1D,
    CLK_SET_12_BCLKDIV = 0x1E,
    AUD_IFACE_SET_4    = 0x1F,
    AUD_IFACE_SET_5    = 0x20,
    AUD_IFACE_SET_6    = 0x21,
    DIG_IFACE_SET_1    = 0x22,
    ADC_FLAG           = 0x24,
    DAC_FLAG_1         = 0x25,
    DAC_FLAG_2         = 0x26,
    STICKY_FLAG_1      = 0x2A,
    INT_FLAG_1         = 0x2B,
    STICKY_FLAG_2      = 0x2C,
    STICKY_FLAG_3      = 0x2D,
    INT_FLAG_2         = 0x2E,
    INT_FLAG_3         = 0x2F,
    INT1_CNTRL         = 0x30,
    INT2_CNTRL         = 0x31,
    GPIO_CNTRL         = 0x34,
    DOUT_FN_CNTRL      = 0x35,
    DIN_FN_CNTRL       = 0x36,
    MISO_FN_CNTRL      = 0x37,
    SCLK_FN_CNTRL      = 0x38,
    DAC_SPB_CNTRL      = 0x3C,
    ADC_SPB_CNTRL      = 0x3D,
    DAC_SETUP_1        = 0x3F,
    DAC_SETUP_2        = 0x40,
    LDAC_DIG_VOL_CNTRL = 0x41,
    RDAC_DIG_VOL_CNTRL = 0x42,
    HDST_DETECT_CNTRL  = 0x43,
    DRC_CNTRL_1        = 0x44,
    DRC_CNTRL_2        = 0x45,
    DRC_CNTRL_3        = 0x46,

    BP_GNRT_1          = 0x47,
    BP_GNRT_2          = 0x48,
    BP_GNRT_3          = 0x49,
    BP_GNRT_4          = 0x4A,
    BP_GNRT_5          = 0x4B,
    BP_GNRT_6          = 0x4C,
    BP_GNRT_7          = 0x4D,
    BP_GNRT_8          = 0x4E,
    BP_GNRT_9          = 0x4F,
    ADC_CHNL_SET       = 0x51,
    ADC_FINE_GAIN_ADJ  = 0x52,
    LADC_VOL_CNTRL     = 0x53,
    RADC_VOL_CNTRL     = 0x54,
    ADC_PHASE_ADJ      = 0x55,
    LAGC_CNTRL_1       = 0x56,
    LAGC_CNTRL_2       = 0x57,
    LAGC_CNTRL_3       = 0x58,
    LAGC_CNTRL_4       = 0x59,
    LAGC_CNTRL_5       = 0x5A,
    LAGC_CNTRL_6       = 0x5B,
    LAGC_CNTRL_7       = 0x5C,
    LAGC_CNTRL_8       = 0x5D,
    RAGC_CNTRL_1       = 0x5E,
    RAGC_CNTRL_2       = 0x5F,
    RAGC_CNTRL_3       = 0x60,
    RAGC_CNTRL_4       = 0x61,
    RAGC_CNTRL_5       = 0x62,
    RAGC_CNTRL_6       = 0x63,
    RAGC_CNTRL_7       = 0x64,
    RAGC_CNTRL_8       = 0x65,
    DC_MEAS_1          = 0x66,
    DC_MEAS_2          = 0x67,
    L_DC_MEAS_OUT_1    = 0x68,
    L_DC_MEAS_OUT_2    = 0x69,
    L_DC_MEAS_OUT_3    = 0x6A,
    R_DC_MEAS_OUT_1    = 0x6B,
    R_DC_MEAS_OUT_2    = 0x6C,
    R_DC_MEAS_OUT_3    = 0x6D,
};

/**
 * @brief Page 1 register offsets for TI TLV320AIC3204 Codec
 */
enum class TLV320AIC3204_PAGE_1_REG : uint8_t
{
    PAGE_SELECT                 = 0x00,
    PWR_CONFIG                  = 0x01,
    LDO_CNTRL                   = 0x02,
    PLBCK_CONFIG_1              = 0x03,
    PLBCK_CONFIG_2              = 0x04,
    OUT_DR_PWR_CNTRL            = 0x09,
    COMMON_MODE_CNTRL           = 0x0A,
    OVER_CURR_PROT_CONFIG       = 0x0B,
    HPL_ROUTE_SELECT            = 0x0C,
    HPR_ROUTE_SELECT            = 0x0D,
    LOL_ROUTE_SELECT            = 0x0E,
    LOR_ROUTE_SELECT            = 0x0F,
    HPL_DR_GAIN_SET             = 0x10,
    HPR_DR_GAIN_SET             = 0x11,
    LOL_DR_GAIN_SET             = 0x12,
    LOR_DR_GAIN_SET             = 0x13,
    HDPHN_DR_STRUP_CNTRL        = 0x14,
    IN1_L_TO_HPL_VOL_CNTRL      = 0x16,
    IN1_L_TO_HPR_VOL_CNTRL      = 0x17,
    MXR_AMP_LVOL_CNTRL          = 0x18,
    MXR_AMP_RVOL_CNTRL          = 0x19,
    MICBIAS_CONFIG              = 0x33,
    LMICPGA_POS_IN_ROUTE_CONFIG = 0x34,
    LMICPGA_NEG_IN_ROUTE_CONFIG = 0x36,
    RMICPGA_POS_IN_ROUTE_CONFIG = 0x37,
    RMICPGA_NEG_IN_ROUTE_CONFIG = 0x39,
    FLOAT_IN_CONFIG             = 0x3A,
    LMICPGA_VOL_CNTRL           = 0x3B,
    RMICPGA_VOL_CNTRL           = 0x3C,
    ADC_PWR_TUNE_CONFIG         = 0x3D,
    ADC_ANALOG_VOL_CNTRL_FLAG   = 0x3E,
    DAC_ANALOG_GAIN_CNTRL_FLAG  = 0x3F,
    ANALOG_IN_QUICK_CHG_CONFIG  = 0x47,
    REF_POWER_UP_CONFIG         = 0x7B,
};

/**
 * @brief Page 8 register offsets for TI TLV320AIC3204 Codec
 */
enum class TLV320AIC3204_PAGE_8_REG : uint8_t
{
    PAGE_SELECT                = 0x00,
    ADC_ADAPTIVE_FILTER_CONFIG = 0x01,
    ADC_COEFF_BUFF_A_START     = 0x08,
};

/**
 * @brief Page 9-16 register offsets for TI TLV320AIC3204 Codec
 */
enum class TLV320AIC3204_PAGE_9_TO_16_REG : uint8_t
{
    PAGE_SELECT                 = 0x00,
    ADC_COEFF_BUFF_A_CONT_START = 0x08,
};

/**
 * @brief Page 26-34 register offsets for TI TLV320AIC3204 Codec
 */
enum class TLV320AIC3204_PAGE_26_TO_34_REG : uint8_t
{
    PAGE_SELECT            = 0x00,
    ADC_COEFF_BUFF_B_START = 0x08,
};

/**
 * @brief Page 44 register offsets for TI TLV320AIC3204 Codec
 */
enum class TLV320AIC3204_PAGE_44_REG : uint8_t
{
    PAGE_SELECT                = 0x00,
    DAC_ADAPTIVE_FILTER_CONFIG = 0x01,
    DAC_COEFF_BUFF_A_START     = 0x08,
};

/**
 * @brief Page 45-52 register offsets for TI TLV320AIC3204 Codec
 */
enum class TLV320AIC3204_PAGE_45_TO_52_REG : uint8_t
{
    PAGE_SELECT                 = 0x00,
    DAC_COEFF_BUFF_A_CONT_START = 0x08,
};

/**
 * @brief Page 62-70 register offsets for TI TLV320AIC3204 Codec
 */
enum class TLV320AIC3204_PAGE_62_TO_70_REG : uint8_t
{
    PAGE_SELECT            = 0x00,
    DAC_COEFF_BUFF_B_START = 0x08,
};

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
    const uint32_t supported_rates[] = {AUDIO_SAMPLE_RATE_8KHz,
                                        AUDIO_SAMPLE_RATE_16KHz,
                                        AUDIO_SAMPLE_RATE_22_05KHz,
                                        AUDIO_SAMPLE_RATE_24KHz,
                                        AUDIO_SAMPLE_RATE_32KHz,
                                        AUDIO_SAMPLE_RATE_44_1KHz,
                                        AUDIO_SAMPLE_RATE_48KHz,
                                        AUDIO_SAMPLE_RATE_96KHz,
                                        AUDIO_SAMPLE_RATE_192KHz};

    uint32_t       new_rate          = supported_rates[0];
    for (uint32_t rate : supported_rates)
    {
        new_rate = rate;
        if (device_config.sampling_rate <= rate)
        {
            break;
        }
    }

    if (new_rate != device_config.sampling_rate)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_TLV_CPP,
                "Sampling rate %d is not supported. Setting to %dHz\n",
                device_config.sampling_rate,
                new_rate);
            device_config.sampling_rate = new_rate;
            wHalErrorCode error =
                w_hal_i2s_validate_cutoff_frequencies(device_config, fix);
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_TLV_CPP,
                              "Sampling rate %d is not supported.\n",
                              device_config.sampling_rate);
            return wHalErrorCode::I2S_INVALID_SAMPLING_RATE;
        }
    }

    if (device_config.num_audio_channels == 0)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_TLV_CPP,
                "Number of channels %d is not supported. Setting to 1\n",
                device_config.num_audio_channels);
            device_config.num_audio_channels = 1;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_TLV_CPP,
                              "Number of channels %d is not supported.\n",
                              device_config.num_audio_channels);
            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }
    else if (device_config.num_audio_channels > 2)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_TLV_CPP,
                "Number of channels %d is not supported. Setting to 2\n",
                device_config.num_audio_channels);
            device_config.num_audio_channels = 2;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_TLV_CPP,
                              "Number of channels %d is not supported.\n",
                              device_config.num_audio_channels);
            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }

    if (device_config.audio_sampling_width != BITS_PER_AUDIO_SAMPLE_16 &&
        // device_config.audio_sampling_width != BITS_PER_AUDIO_SAMPLE_20 && //
        // TLV320AIC3204 does not support 20-bit audio
        device_config.audio_sampling_width != BITS_PER_AUDIO_SAMPLE_24 &&
        device_config.audio_sampling_width != BITS_PER_AUDIO_SAMPLE_32)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2S_TLV_CPP,
                "Sampling width %d is not supported. Setting to 16 bits\n",
                device_config.audio_sampling_width);
            device_config.audio_sampling_width = BITS_PER_AUDIO_SAMPLE_16;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_TLV_CPP,
                              "Sampling width %d is not supported.\n",
                              device_config.audio_sampling_width);
            return wHalErrorCode::I2S_INVALID_SAMPLING_WIDTH;
        }
    }

    return wHalErrorCode::NO_ERROR;
}

// === PAGE 0 UNIONS ===

/**
 * @brief Register address 0x00, Page 0
 *        Name:
 */
union page_select_st
{
    uint16_t dw;
    struct
    {
        uint16_t page : 8;
    } bits;
};

/**
 * @brief Register address 0x01, Page 0
 *        Name:
 */
union sw_reset_st
{
    uint16_t dw;
    struct
    {
        uint16_t self_clearing_software_reset : 1;
        uint16_t rsvd0                        : 7;
    } bits;
};

/**
 * @brief Register address 0x04, Page 0
 *        Name:
 */
union clk_set_1_mux_st
{
    uint16_t dw;
    struct
    {
        uint16_t select_codec_clkin : 2;
        uint16_t pll_input_clock    : 2;
        uint16_t rsvd0              : 2;
        uint16_t pll_range          : 1;
        uint16_t rsvd1              : 1;
    } bits;
};

/**
 * @brief Register address 0x05, Page 0
 *        Name:
 */
union clk_set_2_pllpr_st
{
    uint16_t dw;
    struct
    {
        uint16_t pll_r        : 4;
        uint16_t pll_p        : 3;
        uint16_t pll_power_up : 1;
    } bits;
};

/**
 * @brief Register address 0x06, Page 0
 *        Name:
 */
union clk_set_3_pllj_st
{
    uint16_t dw;
    struct
    {
        uint16_t pll_j : 6;
        uint16_t rsvd0 : 2;
    } bits;
};

/**
 * @brief Register address 0x07, Page 0
 *        Name:
 */
union clk_set_4_plldmsb_st
{
    uint16_t dw;
    struct
    {
        uint16_t pll_d_msb : 6;
        uint16_t rsvd0     : 2;
    } bits;
};

/**
 * @brief Register address 0x08, Page 0
 *        Name:
 */
union clk_set_5_plldlsb_st
{
    uint16_t dw;
    struct
    {
        uint16_t pll_d_lsb : 8;
    } bits;
};

/**
 * @brief Register address 0x0B, Page 0
 *        Name:
 */
union clk_set_6_ndac_st
{
    uint16_t dw;
    struct
    {
        uint16_t ndac_value : 7;
        uint16_t ndac_power : 1;
    } bits;
};

/**
 * @brief Register address 0x0C, Page 0
 *        Name:
 */
union clk_set_7_mdac_st
{
    uint16_t dw;
    struct
    {
        uint16_t mdac_value : 7;
        uint16_t mdac_power : 1;
    } bits;
};

/**
 * @brief Register address 0x0D, Page 0
 *        Name:
 */
union dosr_set_1_msb_st
{
    uint16_t dw;
    struct
    {
        uint16_t dac_osr_dosr_setting : 2;
        uint16_t rsvd0                : 6;
    } bits;
};

/**
 * @brief Register address 0x0E, Page 0
 *        Name:
 */
union dosr_set_2_lsb_st
{
    uint16_t dw;
    struct
    {
        uint16_t dac_osr_dosr_setting : 8;
    } bits;
};

/**
 * @brief Register address 0x12, Page 0
 *        Name:
 */
union clk_set_8_nadc_st
{
    uint16_t dw;
    struct
    {
        uint16_t nadc_value       : 7;
        uint16_t nadc_clock_power : 1;
    } bits;
};

/**
 * @brief Register address 0x13, Page 0
 *        Name:
 */
union clk_set_9_madc_st
{
    uint16_t dw;
    struct
    {
        uint16_t madc_value       : 7;
        uint16_t madc_clock_power : 1;
    } bits;
};

/**
 * @brief Register address 0x14, Page 0
 *        Name:
 */
union aosr_st
{
    uint16_t dw;
    struct
    {
        uint16_t adc_oversampling : 8;
    } bits;
};

/**
 * @brief Register address 0x19, Page 0
 *        Name:
 */
union clk_set_10_mux2_st
{
    uint16_t dw;
    struct
    {
        uint16_t cdiv_clkin_clock_selection : 3;
        uint16_t rsvd0                      : 5;
    } bits;
};

/**
 * @brief Register address 0x1A, Page 0
 *        Name:
 */
union clk_set_11_clkoutm_st
{
    uint16_t dw;
    struct
    {
        uint16_t clkout_m       : 7;
        uint16_t clkout_m_power : 1;
    } bits;
};

/**
 * @brief Register address 0x1B, Page 0
 *        Name:
 */
union aud_iface_set_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t dout_high_impendance_output : 1;
        uint16_t rsvd0                       : 1;
        uint16_t wclk_direction              : 1;
        uint16_t bclk_direction              : 1;
        uint16_t audio_data_word_length      : 2;
        uint16_t audio_interface_selection   : 2;
    } bits;
};

/**
 * @brief Register address 0x1C, Page 0
 *        Name:
 */
union aud_iface_set_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t data_offset : 8;
    } bits;
};

/**
 * @brief Register address 0x1D, Page 0
 *        Name:
 */
union aud_iface_set_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t bdiv_clkin_multiplexer   : 2;
        uint16_t primary_bclk_and_primary : 1;
        uint16_t audio_bit_clock_polarity : 1;
        uint16_t loopback_control         : 1;
        uint16_t loopback_control_5       : 1;
        uint16_t rsvd0                    : 2;
    } bits;
};

/**
 * @brief Register address 0x1E, Page 0
 *        Name:
 */
union clk_set_12_bclkdiv_st
{
    uint16_t dw;
    struct
    {
        uint16_t bclk_n       : 7;
        uint16_t bclk_n_power : 1;
    } bits;
};

/**
 * @brief Register address 0x1F, Page 0
 *        Name:
 */
union aud_iface_set_4_st
{
    uint16_t dw;
    struct
    {
        uint16_t secondary_data_input_multiplexer : 1;
        uint16_t adc_word_clock_multiplexer       : 2;
        uint16_t secondary_word_clock_multiplexer : 2;
        uint16_t secondary_bit_clock_multiplexer  : 2;
        uint16_t rsvd0                            : 1;
    } bits;
};

/**
 * @brief Register address 0x20, Page 0
 *        Name:
 */
union aud_iface_set_5_st
{
    uint16_t dw;
    struct
    {
        uint16_t audio_data_in            : 1;
        uint16_t adc_word_clock           : 1;
        uint16_t primary___secondary_word : 1;
        uint16_t primary___secondary_bit  : 1;
        uint16_t rsvd0                    : 4;
    } bits;
};

/**
 * @brief Register address 0x21, Page 0
 *        Name:
 */
union aud_iface_set_6_st
{
    uint16_t dw;
    struct
    {
        uint16_t secondary_data_out_output   : 1;
        uint16_t primary_data_out_output     : 1;
        uint16_t secondary_word_clock_output : 2;
        uint16_t wclk_output                 : 2;
        uint16_t secondary_bit_clock_output  : 1;
        uint16_t bclk_output                 : 1;
    } bits;
};

/**
 * @brief Register address 0x22, Page 0
 *        Name:
 */
union dig_iface_set_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                    : 5;
        uint16_t i2c_general_call_address : 1;
        uint16_t rsvd1                    : 1;
        uint16_t rsvd2                    : 1;
    } bits;
};

/**
 * @brief Register address 0x24, Page 0
 *        Name:
 */
union adc_flag_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                  : 1;
        uint16_t right_agc_gain_status  : 1;
        uint16_t right_adc_power_status : 1;
        uint16_t right_adc_pga_status   : 1;
        uint16_t rsvd1                  : 1;
        uint16_t left_agc_gain_status   : 1;
        uint16_t left_adc_power_status  : 1;
        uint16_t left_adc_pga_status    : 1;
    } bits;
};

/**
 * @brief Register address 0x25, Page 0
 *        Name:
 */
union dac_flag_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                       : 1;
        uint16_t right_headphone_driver_hpr  : 1;
        uint16_t right_line_output_driverlor : 1;
        uint16_t right_dac_power_status      : 1;
        uint16_t rsvd1                       : 1;
        uint16_t left_headphone_driver_hpl   : 1;
        uint16_t left_line_output_driverlol  : 1;
        uint16_t left_dac_power_status       : 1;
    } bits;
};

/**
 * @brief Register address 0x26, Page 0
 *        Name:
 */
union dac_flag_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_dac_pga_status : 1;
        uint16_t rsvd0                : 3;
        uint16_t left_dac_pga_status  : 1;
        uint16_t rsvd1                : 3;
    } bits;
};

/**
 * @brief Register address 0x2A, Page 0
 *        Name:
 */
union sticky_flag_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                     : 2;
        uint16_t right_adc_overflow_status : 1;
        uint16_t left_adc_overflow_status  : 1;
        uint16_t rsvd1                     : 2;
        uint16_t right_dac_overflow_status : 1;
        uint16_t left_dac_overflow_status  : 1;
    } bits;
};

/**
 * @brief Register address 0x2B, Page 0
 *        Name:
 */
union int_flag_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                     : 2;
        uint16_t right_adc_overflow_status : 1;
        uint16_t left_adc_overflow_status  : 1;
        uint16_t rsvd1                     : 2;
        uint16_t right_dac_overflow_status : 1;
        uint16_t left_dac_overflow_status  : 1;
    } bits;
};

/**
 * @brief Register address 0x2C, Page 0
 *        Name:
 */
union sticky_flag_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                                 : 2;
        uint16_t right_channel_drc_signal              : 1;
        uint16_t left_channel_drc_signal               : 1;
        uint16_t headset_insertion_removal_detect_flag : 1;
        uint16_t headset_button_press                  : 1;
        uint16_t hpr_over_current_detect               : 1;
        uint16_t hpl_over_current_detect               : 1;
    } bits;
};

/**
 * @brief Register address 0x2D, Page 0
 *        Name:
 */
union sticky_flag_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                     : 1;
        uint16_t right_adc_dc_measurement  : 1;
        uint16_t left_adc_dc_measurement   : 1;
        uint16_t rsvd1                     : 2;
        uint16_t right_agc_noise_threshold : 1;
        uint16_t left_agc_noise_threshold  : 1;
        uint16_t rsvd2                     : 1;
    } bits;
};

/**
 * @brief Register address 0x2E, Page 0
 *        Name:
 */
union int_flag_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                                 : 2;
        uint16_t right_channel_drc_signal              : 1;
        uint16_t left_channel_drc_signal               : 1;
        uint16_t headset_insertion_removal_detect_flag : 1;
        uint16_t headset_button_press                  : 1;
        uint16_t hpr_over_current_detect               : 1;
        uint16_t hpl_over_current_detect               : 1;
    } bits;
};

/**
 * @brief Register address 0x2F, Page
 *        Name:
 */
union int_flag_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                     : 1;
        uint16_t right_adc_dc_measurement  : 1;
        uint16_t left_adc_dc_measurement   : 1;
        uint16_t rsvd1                     : 2;
        uint16_t right_agc_noise_threshold : 1;
        uint16_t left_agc_noise_threshold  : 1;
        uint16_t rsvd2                     : 1;
    } bits;
};

/**
 * @brief Register address 0x30, Page 0
 *        Name:
 */
union int1_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t int1_pulse                  : 1;
        uint16_t int1_interrupt_for_dc       : 1;
        uint16_t int1_interrupt_for_overflow : 1;
        uint16_t int1_interrupt_for_over     : 1;
        uint16_t int1_interrupt_for_agc      : 1;
        uint16_t int1_interrupt_for_dac      : 1;
        uint16_t int1_interrupt_for_button   : 1;
        uint16_t int1_interrupt_for_headset  : 1;
    } bits;
};

/**
 * @brief Register address 0x31, Page 0
 *        Name:
 */
union int2_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t int2_pulse                  : 1;
        uint16_t int2_interrupt_for_dc       : 1;
        uint16_t int2_interrupt_for_overflow : 1;
        uint16_t int2_interrupt_for_over     : 1;
        uint16_t int2_interrupt_for_agc      : 1;
        uint16_t int2_interrupt_for_dac      : 1;
        uint16_t int2_interrupt_for_button   : 1;
        uint16_t int2_interrupt_for_headset  : 1;
    } bits;
};

/**
 * @brief Register address 0x34, Page 0
 *        Name:
 */
union gpio_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t gpio_as_general_purpose : 1;
        uint16_t gpio_input_pin_state    : 1;
        uint16_t gpio_control            : 4;
        uint16_t rsvd0                   : 2;
    } bits;
};

/**
 * @brief Register address 0x35, Page 0
 *        Name:
 */
union dout_fn_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t dout_as_general_purpose : 1;
        uint16_t dout_mux                : 3;
        uint16_t dout_bus_keeper         : 1;
        uint16_t rsvd0                   : 3;
    } bits;
};

/**
 * @brief Register address 0x36, Page 0
 *        Name:
 */
union din_fn_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t of_din_input_pin : 1;
        uint16_t din_function     : 2;
        uint16_t rsvd0            : 5;
    } bits;
};

/**
 * @brief Register address 0x37, Page 0
 *        Name:
 */
union miso_fn_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t to_be_driven_on : 1;
        uint16_t miso_function   : 4;
        uint16_t rsvd0           : 3;
    } bits;
};

/**
 * @brief Register address 0x38, Page 0
 *        Name:
 */
union sclk_fn_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t of_sclk_input_pin : 1;
        uint16_t sclk_function     : 2;
        uint16_t rsvd0             : 5;
    } bits;
};

/**
 * @brief Register address 0x3C, Page 0
 *        Name:
 */
union dac_spb_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t selects_the_dac_playback : 5;
        uint16_t rsvd0                    : 3;
    } bits;
};

/**
 * @brief Register address 0x3D, Page 0
 *        Name:
 */
union adc_spb_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t selects_the_adc_recording : 5;
        uint16_t rsvd0                     : 3;
    } bits;
};

/**
 * @brief Register address 0x3F, Page 0
 *        Name:
 */
union dac_setup_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t dac_channel_volume_controls : 2;
        uint16_t right_dac_data_path         : 2;
        uint16_t left_dac_data_path          : 2;
        uint16_t right_dac_channel_power     : 1;
        uint16_t left_dac_channel_power      : 1;
    } bits;
};

/**
 * @brief Register address 0x40, Page 0
 *        Name:
 */
union dac_setup_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t dac_master_volume      : 2;
        uint16_t right_dac_channel_mute : 1;
        uint16_t left_dac_channel_mute  : 1;
        uint16_t dac_auto_mute          : 3;
        uint16_t right_modulator_output : 1;
    } bits;
};

/**
 * @brief Register address 0x41, Page 0
 *        Name:
 */
union ldac_dig_vol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_dac_channel_digital : 8;
    } bits;
};

/**
 * @brief Register address 0x42, Page 0
 *        Name:
 */
union rdac_dig_vol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_dac_channel_digital : 8;
    } bits;
};

/**
 * @brief Register address 0x43, Page 0
 *        Name:
 */
union hdst_detect_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t headset_button_press_debounce              : 2;
        uint16_t headset_detection_debounce_programmability : 3;
        uint16_t headset_type_flag                          : 2;
        uint16_t headset_detection_disabled                 : 1;
    } bits;
};

/**
 * @brief Register address 0x44, Page 0
 *        Name:
 */
union drc_cntrl_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t drc_hysteresis : 2;
        uint16_t drc_threshold  : 3;
        uint16_t drc            : 1;
        uint16_t drc_6          : 1;
        uint16_t rsvd0          : 1;
    } bits;
};

/**
 * @brief Register address 0x45, Page 0
 *        Name:
 */
union drc_cntrl_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                    : 3;
        uint16_t drc_hold_programmability : 4;
        uint16_t rsvd1                    : 1;
    } bits;
};

/**
 * @brief Register address 0x46, Page 0
 *        Name:
 */
union drc_cntrl_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t drc_decay_rate  : 4;
        uint16_t drc_attack_rate : 4;
    } bits;
};

/**
 * @brief Register address 0x47, Page 0
 *        Name:
 */
union bp_gnrt_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_beep_volume : 6;
        uint16_t rsvd0                    : 1;
        uint16_t beep_generator_disabled  : 1;
    } bits;
};

/**
 * @brief Register address 0x48, Page 0
 *        Name:
 */
union bp_gnrt_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_beep_volume    : 6;
        uint16_t beep_generator_master_volume : 2;
    } bits;
};

/**
 * @brief Register address 0x49, Page 0
 *        Name:
 */
union bp_gnrt_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t programmed_is_beep_sample : 8;
    } bits;
};

/**
 * @brief Register address 0x4A, Page 0
 *        Name:
 */
union bp_gnrt_4_st
{
    uint16_t dw;
    struct
    {
        uint16_t programmed_is_beep_sample : 8;
    } bits;
};

/**
 * @brief Register address 0x4B, Page 0
 *        Name:
 */
union bp_gnrt_5_st
{
    uint16_t dw;
    struct
    {
        uint16_t programmed_is_beep_sample : 8;
    } bits;
};

/**
 * @brief Register address 0x4C, Page 0
 *        Name:
 */
union bp_gnrt_6_st
{
    uint16_t dw;
    struct
    {
        uint16_t programmed_is_beep_sinx158 : 8;
    } bits;
};

/**
 * @brief Register address 0x4D, Page 0
 *        Name:
 */
union bp_gnrt_7_st
{
    uint16_t dw;
    struct
    {
        uint16_t programmed_is_beep_sinx70 : 8;
    } bits;
};

/**
 * @brief Register address 0x4E, Page 0
 *        Name:
 */
union bp_gnrt_8_st
{
    uint16_t dw;
    struct
    {
        uint16_t programmed_is_beep_cosx158 : 8;
    } bits;
};

/**
 * @brief Register address 0x4F, Page 0
 *        Name:
 */
union bp_gnrt_9_st
{
    uint16_t dw;
    struct
    {
        uint16_t programmed_is_beep_cosx70 : 8;
    } bits;
};

/**
 * @brief Register address 0x51, Page 0
 *        Name:
 */
union adc_chnl_set_st
{
    uint16_t dw;
    struct
    {
        uint16_t adc_volume_soft_stepping               : 2;
        uint16_t right_channel_digital_microphone       : 1;
        uint16_t left_channel_digital_microphone        : 1;
        uint16_t digital_microphone_input_configuration : 2;
        uint16_t right_channel_adc_power                : 1;
        uint16_t left_channel_adc_power                 : 1;
    } bits;
};

/**
 * @brief Register address 0x52, Page 0
 *        Name:
 */
union adc_fine_gain_adj_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_adc_channel_fine : 3;
        uint16_t right_adc_channel_mute : 1;
        uint16_t left_adc_channel_fine  : 3;
        uint16_t left_adc_channel_mute  : 1;
    } bits;
};
/**
 * @brief Register address 0x53, Page 0
 *        Name:
 */
union ladc_vol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_adc_channel_volume : 7;
        uint16_t rsvd0                   : 1;
    } bits;
};

/**
 * @brief Register address 0x54, Page 0
 *        Name:
 */
union radc_vol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_adc_channel_volume : 7;
        uint16_t rsvd0                    : 1;
    } bits;
};

/**
 * @brief Register address 0x55, Page 0
 *        Name:
 */
union adc_phase_adj_st
{
    uint16_t dw;
    struct
    {
        uint16_t adc_phase_compensation : 8;
    } bits;
};

/**
 * @brief Register address 0x56, Page 0
 *        Name:
 */
union lagc_cntrl_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_agc_gain     : 2;
        uint16_t rsvd0                     : 2;
        uint16_t left_channel_agc_target   : 3;
        uint16_t left_channel_agc_disabled : 1;
    } bits;
};

/**
 * @brief Register address 0x57, Page 0
 *        Name:
 */
union lagc_cntrl_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                       : 1;
        uint16_t left_channel_agc_noise      : 5;
        uint16_t left_channel_agc_hysteresis : 2;
    } bits;
};

/**
 * @brief Register address 0x58, Page 0
 *        Name:
 */
union lagc_cntrl_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_agc_maximum : 7;
        uint16_t rsvd0                    : 1;
    } bits;
};

/**
 * @brief Register address 0x59, Page 0
 *        Name:
 */
union lagc_cntrl_4_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_agc_attack   : 3;
        uint16_t left_channel_agc_attack_3 : 5;
    } bits;
};

/**
 * @brief Register address 0x5A, Page 0
 *        Name:
 */
union lagc_cntrl_5_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_agc_decay   : 3;
        uint16_t left_channel_agc_decay_3 : 5;
    } bits;
};

/**
 * @brief Register address 0x5B, Page 0
 *        Name:
 */
union lagc_cntrl_6_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_agc_noise : 5;
        uint16_t rsvd0                  : 3;
    } bits;
};

/**
 * @brief Register address 0x5C, Page 0
 *        Name:
 */
union lagc_cntrl_7_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_agc_signal : 4;
        uint16_t rsvd0                   : 4;
    } bits;
};

/**
 * @brief Register address 0x5D, Page 0
 *        Name:
 */
union lagc_cntrl_8_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_agc_gain : 8;
    } bits;
};

/**
 * @brief Register address 0x5E, Page 0
 *        Name:
 */
union ragc_cntrl_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_agc_gain     : 2;
        uint16_t rsvd0                      : 2;
        uint16_t right_channel_agc_target   : 3;
        uint16_t right_channel_agc_disabled : 1;
    } bits;
};

/**
 * @brief Register address 0x5F, Page 0
 *        Name:
 */
union ragc_cntrl_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                        : 1;
        uint16_t right_channel_agc_noise      : 5;
        uint16_t right_channel_agc_hysteresis : 2;
    } bits;
};

/**
 * @brief Register address 0x60, Page 0
 *        Name:
 */
union ragc_cntrl_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_agc_maximum : 7;
        uint16_t rsvd0                     : 1;
    } bits;
};

/**
 * @brief Register address 0x61, Page 0
 *        Name:
 */
union ragc_cntrl_4_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_agc_attack   : 3;
        uint16_t right_channel_agc_attack_3 : 5;
    } bits;
};

/**
 * @brief Register address 0x62, Page 0
 *        Name:
 */
union ragc_cntrl_5_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_agc_decay   : 3;
        uint16_t right_channel_agc_decay_3 : 5;
    } bits;
};

/**
 * @brief Register address 0x63, Page 0
 *        Name:
 */
union ragc_cntrl_6_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_agc_noise : 5;
        uint16_t rsvd0                   : 3;
    } bits;
};

/**
 * @brief Register address 0x64, Page 0
 *        Name:
 */
union ragc_cntrl_7_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_agc_signal : 4;
        uint16_t rsvd0                    : 4;
    } bits;
};

/**
 * @brief Register address 0x65, Page 0
 *        Name:
 */
union ragc_cntrl_8_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_agc_gain : 8;
    } bits;
};

/**
 * @brief Register address 0x66, Page 0
 *        Name:
 */
union dc_meas_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t dc_measurement_d_setting       : 5;
        uint16_t dc_measurement_is_done         : 1;
        uint16_t dc_measurement_mode_disabled   : 1;
        uint16_t dc_measurement_mode_disabled_7 : 1;
    } bits;
};

/**
 * @brief Register address 0x67, Page 0
 *        Name:
 */
union dc_meas_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t iir_based_dc_measurement : 5;
        uint16_t for_iir_based_dc         : 1;
        uint16_t left_and_right_channel   : 1;
        uint16_t rsvd0                    : 1;
    } bits;
};

/**
 * @brief Register address 0x68, Page 0
 *        Name:
 */
union l_dc_meas_out_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_dc_measurement : 8;
    } bits;
};

/**
 * @brief Register address 0x69, Page 0
 *        Name:
 */
union l_dc_meas_out_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_dc_measurement : 8;
    } bits;
};

/**
 * @brief Register address 0x6A, Page 0
 *        Name:
 */
union l_dc_meas_out_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_channel_dc_measurement : 8;
    } bits;
};

/**
 * @brief Register address 0x6B, Page 0
 *        Name:
 */
union r_dc_meas_out_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_dc_measurement : 8;
    } bits;
};

/**
 * @brief Register address 0x6C, Page 0
 *        Name:
 */
union r_dc_meas_out_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_dc_measurement : 8;
    } bits;
};

/**
 * @brief Register address 0x6D, Page 0
 *        Name:
 */
union r_dc_meas_out_3_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_dc_measurement : 8;
    } bits;
};

// === PAGE 1 UNIONS ===

/**
 * @brief Register address 0x01, Page 1
 *        Name:
 */
union pwr_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0               : 3;
        uint16_t avdd_will_be_weakly : 1;
        uint16_t rsvd1               : 4;
    } bits;
};

/**
 * @brief Register address 0x02, Page 1
 *        Name:
 */
union ldo_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t avdd_ldo_power        : 1;
        uint16_t avdd_ldo_over_current : 1;
        uint16_t dvdd_ldo_over_current : 1;
        uint16_t analog_block_power    : 1;
        uint16_t avdd_ldo              : 2;
        uint16_t dvdd_ldo              : 2;
    } bits;
};

/**
 * @brief Register address 0x03, Page 1
 *        Name:
 */
union plbck_config_1_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0               : 2;
        uint16_t left_dac_ptm        : 3;
        uint16_t rsvd1               : 1;
        uint16_t left_dac_routing_to : 2;
    } bits;
};

/**
 * @brief Register address 0x04, Page 1
 *        Name:
 */
union plbck_config_2_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                : 2;
        uint16_t right_dac_ptm        : 3;
        uint16_t rsvd1                : 1;
        uint16_t right_dac_routing_to : 2;
    } bits;
};

/**
 * @brief Register address 0x09, Page 1
 *        Name:
 */
union out_dr_pwr_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_mixer_amplifier_mar : 1;
        uint16_t left_mixer_amplifier_mal  : 1;
        uint16_t lor_is_powered_down       : 1;
        uint16_t lol_is_powered_down       : 1;
        uint16_t hpr_is_powered_down       : 1;
        uint16_t hpl_is_powered_down       : 1;
        uint16_t rsvd0                     : 2;
    } bits;
};

/**
 * @brief Register address 0x0A, Page 1
 *        Name:
 */
union common_mode_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t when_page_1_reg_10_d11   : 1;
        uint16_t output_of_hpl_and        : 1;
        uint16_t rsvd0                    : 1;
        uint16_t output_common_mode_for   : 1;
        uint16_t output_common_mode_for_4 : 2;
        uint16_t full_chip_common_mode    : 1;
        uint16_t rsvd1                    : 1;
    } bits;
};

/**
 * @brief Register address 0x0B, Page 1
 *        Name:
 */
union over_curr_prot_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t output_current_will_be    : 1;
        uint16_t no_debounce_is_used       : 3;
        uint16_t over_current_detection_is : 1;
        uint16_t rsvd0                     : 3;
    } bits;
};

/**
 * @brief Register address 0x0C, Page 1
 *        Name:
 */
union hpl_route_select_st
{
    uint16_t dw;
    struct
    {
        uint16_t mar_output_is_not               : 1;
        uint16_t mal_output_is_not               : 1;
        uint16_t in1l_is_not_routed              : 1;
        uint16_t left_channel_dac_reconstruction : 1;
        uint16_t rsvd0                           : 4;
    } bits;
};

/**
 * @brief Register address 0x0D, Page 1
 *        Name:
 */
union hpr_route_select_st
{
    uint16_t dw;
    struct
    {
        uint16_t hpl_output_is_not                : 1;
        uint16_t mar_output_is_not                : 1;
        uint16_t in1r_is_not_routed               : 1;
        uint16_t right_channel_dac_reconstruction : 1;
        uint16_t left_channel_dac_reconstruction  : 1;
        uint16_t rsvd0                            : 3;
    } bits;
};

/**
 * @brief Register address 0x0E, Page 1
 *        Name:
 */
union lol_route_select_st
{
    uint16_t dw;
    struct
    {
        uint16_t lor_output_is_not                : 1;
        uint16_t mal_output_is_not                : 1;
        uint16_t rsvd0                            : 1;
        uint16_t left_channel_dac_reconstruction  : 1;
        uint16_t right_channel_dac_reconstruction : 1;
        uint16_t rsvd1                            : 3;
    } bits;
};

/**
 * @brief Register address 0x0F, Page 1
 *        Name:
 */
union lor_route_select_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                            : 1;
        uint16_t mar_output_is_not                : 1;
        uint16_t rsvd1                            : 1;
        uint16_t right_channel_dac_reconstruction : 1;
        uint16_t rsvd2                            : 4;
    } bits;
};

/**
 * @brief Register address 0x10, Page 1
 *        Name:
 */
union hpl_dr_gain_set_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0             : 6;
        uint16_t hpl_driver_is_not : 1;
        uint16_t rsvd1             : 1;
    } bits;
};

/**
 * @brief Register address 0x11, Page 1
 *        Name:
 */
union hpr_dr_gain_set_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0             : 6;
        uint16_t hpr_driver_is_not : 1;
        uint16_t rsvd1             : 1;
    } bits;
};

/**
 * @brief Register address 0x12, Page 1
 *        Name:
 */
union lol_dr_gain_set_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0             : 6;
        uint16_t lol_driver_is_not : 1;
        uint16_t rsvd1             : 1;
    } bits;
};

/**
 * @brief Register address 0x13, Page 1
 *        Name:
 */
union lor_dr_gain_set_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0             : 6;
        uint16_t lor_driver_is_not : 1;
        uint16_t rsvd1             : 1;
    } bits;
};

/**
 * @brief Register address 0x14, Page 1
 *        Name:
 */
union hdphn_dr_strup_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t headphone_ramps_power_up : 2;
        uint16_t slow_power_up_of         : 4;
        uint16_t soft_routing_step_time   : 2;
    } bits;
};

/**
 * @brief Register address 0x16, Page 1
 *        Name:
 */
union in1_l_to_hpl_vol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t in1l_to_hpl_volume : 7;
        uint16_t rsvd0              : 1;
    } bits;
};

/**
 * @brief Register address 0x17, Page 1
 *        Name:
 */
union in1_l_to_hpr_vol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t in1r_to_hpr_volume : 7;
        uint16_t rsvd0              : 1;
    } bits;
};

/**
 * @brief Register address 0x18, Page 1
 *        Name:
 */
union mxr_amp_lvol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t mixer_amplifier_left_volume : 6;
        uint16_t rsvd0                       : 2;
    } bits;
};

/**
 * @brief Register address 0x19, Page 1
 *        Name:
 */
union mxr_amp_rvol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t mixer_amplifier_right_volume : 6;
        uint16_t rsvd0                        : 2;
    } bits;
};

/**
 * @brief Register address 0x33, Page 1
 *        Name:
 */
union micbias_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0                                : 3;
        uint16_t micbias_voltage_is_generated         : 1;
        uint16_t micbias_output_voltage_configuration : 2;
        uint16_t micbias_powered_down                 : 1;
        uint16_t rsvd1                                : 1;
    } bits;
};

/**
 * @brief Register address 0x34, Page
 *        Name:
 */
union lmicpga_pos_in_route_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t in1r_to_left_micpga : 2;
        uint16_t in3l_to_left_micpga : 2;
        uint16_t in2l_to_left_micpga : 2;
        uint16_t in1l_to_left_micpga : 2;
    } bits;
};

/**
 * @brief Register address 0x36, Page 1
 *        Name:
 */
union lmicpga_neg_in_route_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t cm_to_left_micpga   : 2;
        uint16_t in3r_to_left_micpga : 2;
        uint16_t in2r_to_left_micpga : 2;
        uint16_t cm_to_left_micpga_6 : 2;
    } bits;
};

/**
 * @brief Register address 0x37, Page 1
 *        Name:
 */
union rmicpga_pos_in_route_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t in2l_to_right_micpga : 2;
        uint16_t in3r_to_right_micpga : 2;
        uint16_t in2r_to_right_micpga : 2;
        uint16_t in1r_to_right_micpga : 2;
    } bits;
};

/**
 * @brief Register address 0x39, Page 1
 *        Name:
 */
union rmicpga_neg_in_route_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t cm_to_right_micpga   : 2;
        uint16_t in3l_to_right_micpga : 2;
        uint16_t in1l_to_right_micpga : 2;
        uint16_t cm_to_right_micpga_6 : 2;
    } bits;
};

/**
 * @brief Register address 0x3A, Page 1
 *        Name:
 */
union float_in_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0             : 2;
        uint16_t in3r_input_is_not : 1;
        uint16_t in3l_input_is_not : 1;
        uint16_t in2r_input_is_not : 1;
        uint16_t in2l_input_is_not : 1;
        uint16_t in1r_input_is_not : 1;
        uint16_t in1l_input_is_not : 1;
    } bits;
};

/**
 * @brief Register address 0x3B, Page 1
 *        Name:
 */
union lmicpga_vol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t left_micpga_volume  : 7;
        uint16_t left_micpga_gain_is : 1;
    } bits;
};

/*
 * @brief Register address 0x3C, Page 1
 *        Name:
 */
union rmicpga_vol_cntrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_micpga_volume  : 7;
        uint16_t right_micpga_gain_is : 1;
    } bits;
};

/**
 * @brief Register address 0x3D, Page 1
 *        Name:
 */
union adc_pwr_tune_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t ptm_r4_default : 8;
    } bits;
};

/**
 * @brief Register address 0x3E, Page 1
 *        Name:
 */
union adc_analog_vol_cntrl_flag_st
{
    uint16_t dw;
    struct
    {
        uint16_t right_channel_analog_volume : 1;
        uint16_t left_channel_analog_volume  : 1;
        uint16_t rsvd0                       : 6;
    } bits;
};

/**
 * @brief Register address 0x3F, Page 1
 *        Name:
 */
union dac_analog_gain_cntrl_flag_st
{
    uint16_t dw;
    struct
    {
        uint16_t mar_volume_flag    : 1;
        uint16_t mal_volume_flag    : 1;
        uint16_t in1r_to_hpr_bypass : 1;
        uint16_t in1l_to_hpl_bypass : 1;
        uint16_t lor_gain_flag      : 1;
        uint16_t lol_gain_flag      : 1;
        uint16_t hpr_gain_flag      : 1;
        uint16_t hpl_gain_flag      : 1;
    } bits;
};

/**
 * @brief Register address 0x47, Page 1
 *        Name:
 */
union analog_in_quick_chg_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t analog_inputs_power_up : 6;
        uint16_t rsvd0                  : 2;
    } bits;
};

/**
 * @brief Register address 0x7B, Page 1
 *        Name:
 */
union ref_power_up_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t reference_power_up_configuration : 3;
        uint16_t rsvd0                            : 5;
    } bits;
};

// === PAGE 8 UNIONS ===

/**
 * @brief Register address 0x01, Page 8
 *        Name:
 */
union adc_adaptive_filter_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t adc_adaptive_filter_buffer   : 1;
        uint16_t adc_adaptive_filter_buffer_1 : 1;
        uint16_t adc_adaptive_filtering       : 1;
        uint16_t rsvd0                        : 5;
    } bits;
};

/**
 * @brief Register address 0x08, Page 8
 *        Name: Coefficient Buffer Register
 */
union adc_coeff_buff_a_start_st
{
    uint16_t dw;
    struct
    {
        uint16_t coefficient_byte : 8;
    } bits;
};

// === PAGE 9-16 UNIONS ===

/**
 * @brief Register address 0x08, Page 9-16
 *        Name: Coefficient Buffer Register
 */
union adc_coeff_buff_a_cont_start_st
{
    uint16_t dw;
    struct
    {
        uint16_t coefficient_byte : 8;
    } bits;
};

// === PAGE 26-34 UNIONS ===

/**
 * @brief Register address 0x08, Page 26-34
 *        Name: Coefficient Buffer Register
 */
union adc_coeff_buff_b_start_st
{
    uint16_t dw;
    struct
    {
        uint16_t coefficient_byte : 8;
    } bits;
};

// === PAGE 44 UNIONS ===

/**
 * @brief Register address 0x01, Page 44
 *        Name:
 */
union dac_adaptive_filter_config_st
{
    uint16_t dw;
    struct
    {
        uint16_t dac_adaptive_filter_buffer   : 1;
        uint16_t dac_adaptive_filter_buffer_1 : 1;
        uint16_t dac_adaptive_filtering       : 1;
        uint16_t rsvd0                        : 5;
    } bits;
};

/**
 * @brief Register address 0x08, Page 44
 *        Name: Coefficient Buffer Register
 */
union dac_coeff_buff_a_start_st
{
    uint16_t dw;
    struct
    {
        uint16_t coefficient_byte : 8;
    } bits;
};

// === PAGE 45-52 UNIONS ===

/**
 * @brief Register address 0x08, Page 45-52
 *        Name: Coefficient Buffer Register
 */
union dac_coeff_buff_a_cont_start_st
{
    uint16_t dw;
    struct
    {
        uint16_t coefficient_byte : 8;
    } bits;
};

// === PAGE 62-70 UNIONS ===

/**
 * @brief Register address 0x08, Page 62-70
 *        Name: Coefficient Buffer Register
 */
union dac_coeff_buff_b_start_st
{
    uint16_t dw;
    struct
    {
        uint16_t coefficient_byte : 8;
    } bits;
};

#if __CORE__ == AUDIO_CORE

/**
 * @brief Initialization sequence for I2S Peripheral
 *
 * @param device_config [in]: device configuration
 * @param input_source [in]: input source for I2S peripheral
 *
 * @return wHalErrorCode error code
 */
struct biquad_filter_st
{
    int32_t N0;
    int32_t N1;
    int32_t N2;
    int32_t D1;
    int32_t D2;
};

enum class filter_type_t : uint8_t
{
    LOW_PASS_FILTER  = 0,
    HIGH_PASS_FILTER = 1,
    BAND_PASS_FILTER = 2,
};

static float get_quantized_value(float coeff)
{
    // TLV320AIC3204 uses 24-bit 2's complement, Q1.23 format
    // Multiplier is 2^23 = 8388608.0
    if (coeff >= 0)
    {
        coeff = (floor(coeff * 8388608.0f + 0.5f));
        if (coeff > 8388607.0f)
            coeff = 8388607.0f;
    }
    else
    {
        coeff = (ceil(coeff * 8388608.0f - 0.5f));
        if (coeff < -8388608.0f)
            coeff = -8388608.0f;
    }
    return coeff;
}

static float BIQUAD_QUALITY_FACTOR = 1.0f;

static void  calc_biquad(filter_type_t     filter_type,
                         float             fc,
                         uint64_t          fs,
                         biquad_filter_st &biquad_filter)
{
    float W0    = 2.0f * PI * fc / fs;
    float alpha = sinf(W0) * sinh((logf(2.0f) / 2.0f) * BIQUAD_QUALITY_FACTOR *
                                  W0 / sinf(W0));
    float a0 = 0, a1 = 0, a2 = 0, b0 = 0, b1 = 0, b2 = 0;

    switch (filter_type)
    {
        case filter_type_t::LOW_PASS_FILTER:
            a0 = 1.0f + alpha;
            a1 = -2.0f * cosf(W0);
            a2 = 1.0f - alpha;
            b0 = (1.0f - cosf(W0)) / 2.0f;
            b1 = 1.0f - cosf(W0);
            b2 = (1.0f - cosf(W0)) / 2.0f;
            break;
        case filter_type_t::HIGH_PASS_FILTER:
            a0 = 1.0f + alpha;
            a1 = -2.0f * cosf(W0);
            a2 = 1.0f - alpha;
            b0 = (1.0f + cosf(W0)) / 2.0f;
            b1 = -(1.0f + cosf(W0));
            b2 = (1.0f + cosf(W0)) / 2.0f;
            break;
        case filter_type_t::BAND_PASS_FILTER:
            a0 = 1.0f + alpha;
            a1 = -2.0f * cosf(W0);
            a2 = 1.0f - alpha;
            b0 = alpha;
            b1 = 0.0f;
            b2 = -alpha;
            break;
        default:
            break;
    }

    biquad_filter.N0 = static_cast<int32_t>(get_quantized_value(b0 / a0));
    biquad_filter.N1 = static_cast<int32_t>(get_quantized_value(b1 / a0));
    biquad_filter.N2 = static_cast<int32_t>(get_quantized_value(b2 / a0));
    biquad_filter.D1 = static_cast<int32_t>(
        get_quantized_value(-a1 / a0)); // Note: negative for TI format H(z)
    biquad_filter.D2 = static_cast<int32_t>(
        get_quantized_value(-a2 / a0)); // Note: negative for TI format H(z)
}

static void set_biquad_filter_coef(uint8_t                 page,
                                   uint8_t                 start_reg,
                                   const biquad_filter_st &biquad_filter)
{
    int32_t coeffs[5]    = {biquad_filter.N0,
                         biquad_filter.N1,
                         biquad_filter.N2,
                         biquad_filter.D1,
                         biquad_filter.D2};
    uint8_t current_page = page;
    uint8_t reg          = start_reg;

    for (int i = 0; i < 5; i++)
    {
        uint32_t val      = static_cast<uint32_t>(coeffs[i]);
        uint8_t  bytes[4] = {
            static_cast<uint8_t>((val >> 16) & 0xFF),
            static_cast<uint8_t>((val >> 8) & 0xFF),
            static_cast<uint8_t>(val & 0xFF),
            0x00 // 4th byte is filler/ignored by the hardware
        };

        for (int b = 0; b < 4; b++)
        {
            if (reg > 127)
            {
                current_page++;
                reg = 8; // Coefficient RAM on page 9 starts at register 8
            }
            tlvwrite(current_page, reg++, bytes[b]);
        }
    }
}

static void tlvwrite(uint8_t page, TLV320AIC3204_PAGE_0_REG reg, uint8_t value)
{
    tlvwrite(page, static_cast<uint8_t>(reg), value);
}
static void tlvwrite(uint8_t page, TLV320AIC3204_PAGE_1_REG reg, uint8_t value)
{
    tlvwrite(page, static_cast<uint8_t>(reg), value);
}
static void tlvwrite(uint8_t page, TLV320AIC3204_PAGE_8_REG reg, uint8_t value)
{
    tlvwrite(page, static_cast<uint8_t>(reg), value);
}

static wHalErrorCode codec_stage1_init()
{
    // Software Reset Codec
    sw_reset_st sw_reset;
    sw_reset.bits.self_clearing_software_reset = 1;
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::SW_RESET, sw_reset.dw);

    w_hal_system_delay_ms(40);

    // Disable internal crude AVDD generation
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::PWR_CONFIG, 0x08);

    // Enable internal AVDD LDO
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::LDO_CNTRL, 0x01);

    // Set REF charging time to 40ms
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::REF_POWER_UP_CONFIG, 0x05);

    // Configure Headphone Depop Settings (Rpop = 6k, N = 6, 20us soft stepping)
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::HDPHN_DR_STRUP_CNTRL, 0x25);

    // Set Common mode voltage to 0.9V
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::COMMON_MODE_CNTRL, 0x00);

    w_hal_system_delay_ms(45);

    return wHalErrorCode::NO_ERROR;
}

static wHalErrorCode
codec_stage2_init(DeviceConfig::config_page_st &device_config)
{
    uint8_t  P = 2, R = 1, J = 8;
    uint16_t D    = 1920;
    uint8_t  NDAC = 1, MDAC = 16, NADC = 1, MADC = 16;
    uint16_t DOSR = 128;
    uint8_t  AOSR = 128;

    switch (device_config.sampling_rate)
    {
        case AUDIO_SAMPLE_RATE_8KHz:
            NDAC = 6;
            MDAC = 16;
            DOSR = 128;
            NADC = 6;
            MADC = 16;
            AOSR = 128;
            break;
        case AUDIO_SAMPLE_RATE_16KHz:
            NDAC = 3;
            MDAC = 16;
            DOSR = 128;
            NADC = 3;
            MADC = 16;
            AOSR = 128;
            break;
        case AUDIO_SAMPLE_RATE_22_05KHz:
            J    = 7;
            D    = 5264;
            NDAC = 4;
            MDAC = 8;
            DOSR = 128;
            NADC = 4;
            MADC = 8;
            AOSR = 128;
            break;
        case AUDIO_SAMPLE_RATE_24KHz:
            NDAC = 2;
            MDAC = 16;
            DOSR = 128;
            NADC = 2;
            MADC = 16;
            AOSR = 128;
            break;
        case AUDIO_SAMPLE_RATE_32KHz:
            NDAC = 3;
            MDAC = 16;
            DOSR = 64;
            NADC = 3;
            MADC = 16;
            AOSR = 64;
            break;
        case AUDIO_SAMPLE_RATE_44_1KHz:
            J    = 7;
            D    = 5264;
            NDAC = 2;
            MDAC = 8;
            DOSR = 128;
            NADC = 2;
            MADC = 8;
            AOSR = 128;
            break;
        case AUDIO_SAMPLE_RATE_48KHz:
            NDAC = 1;
            MDAC = 16;
            DOSR = 128;
            NADC = 1;
            MADC = 16;
            AOSR = 128;
            break;
        case AUDIO_SAMPLE_RATE_96KHz:
            NDAC = 1;
            MDAC = 16;
            DOSR = 64;
            NADC = 1;
            MADC = 16;
            AOSR = 64;
            break;
        case AUDIO_SAMPLE_RATE_192KHz:
            NDAC = 2;
            MDAC = 8;
            DOSR = 32;
            NADC = 2;
            MADC = 8;
            AOSR = 32;
            break;
        default:
            // Default to 48kHz
            NDAC = 1;
            MDAC = 16;
            DOSR = 128;
            NADC = 1;
            MADC = 16;
            AOSR = 128;
            break;
    }

    g_aosr = AOSR;
    g_madc = MADC;

    // Select PLL for codec clock, MCLK for PLL input
    tlvwrite(0,
             TLV320AIC3204_PAGE_0_REG::CLK_SET_1_MUX,
             0x03); // CODEC_CLKIN = MCLK

    // Set J
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_3_PLLJ, J);

    // Set D (MSB and LSB)
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_4_PLLDMSB, (D >> 8) & 0x3F);
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_5_PLLDLSB, D & 0xFF);

    // Power up PLL, set P and R
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_2_PLLPR, 0x80 | (P << 4) | R);

    // Wait for PLL to lock
    w_hal_system_delay_ms(15);

    // Set NDAC, MDAC (power up by setting bit 7)
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_6_NDAC, 0x80 | (NDAC & 0x7F));
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_7_MDAC, 0x80 | (MDAC & 0x7F));

    // Set DOSR
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::DOSR_SET_1_MSB, (DOSR >> 8) & 0x03);
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::DOSR_SET_2_LSB, DOSR & 0xFF);

    // Set NADC, MADC (power up by setting bit 7)
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_8_NADC, 0x80 | (NADC & 0x7F));
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_9_MADC, 0x80 | (MADC & 0x7F));

    // Set AOSR
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::AOSR, AOSR);

    return wHalErrorCode::NO_ERROR;
}

static wHalErrorCode
codec_stage3_init(DeviceConfig::config_page_st &device_config,
                  i2s_peripheral_input_source_t input_source)
{

    // Audio Interface Formatting (Page 0, Reg 27)
    uint8_t  word_length_val  = 0; // 16-bit default
    uint32_t bits_per_channel = 16;
    switch (device_config.audio_sampling_width)
    {
        case BITS_PER_AUDIO_SAMPLE_16:
            word_length_val  = 0x00;
            bits_per_channel = 16;
            break;
        // case BITS_PER_AUDIO_SAMPLE_20: word_length_val = 0x10;
        // bits_per_channel = 20; break; // TLV320AIC3204 does not support
        // 20-bit audio
        case BITS_PER_AUDIO_SAMPLE_24:
            word_length_val  = 0x20;
            bits_per_channel = 24;
            break;
        case BITS_PER_AUDIO_SAMPLE_32:
            word_length_val  = 0x30;
            bits_per_channel = 32;
            break;
        default:
            word_length_val  = 0x00;
            bits_per_channel = 16;
            break; // keep default to 16 bit
    }

    // Calculate BCLK divider dynamically based on AOSR and bits per channel to
    // support different sample widths
    uint8_t bclk_n;
    uint8_t aud_iface_3_val = 0x07; // Default to ADC_MOD_CLK as source for
                                    // BDIV_CLKIN (0x07 has D1-D0 = 11)

    if (device_config.sampling_rate == AUDIO_SAMPLE_RATE_192KHz)
    {
        // For 192kHz, source BCLK divider from ADC_CLK (0x06 has D1-D0 = 10) to
        // support 12.288MHz BCLK
        aud_iface_3_val = 0x06;
        bclk_n          = (g_madc * g_aosr) / (2 * bits_per_channel);
    }
    else
    {
        // BCLK = 2 * bits_per_channel * LRCLK. BCLK_N = ADC_MOD_CLK / BCLK =
        // AOSR / (2 * bits_per_channel)
        bclk_n = g_aosr / (2 * bits_per_channel);
    }
    if (bclk_n < 1)
        bclk_n = 1;

    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::CLK_SET_12_BCLKDIV, 0x80 | bclk_n);

    uint8_t dac_prb_val =
        (device_config.sampling_rate == AUDIO_SAMPLE_RATE_192KHz) ?
            0x11 :
            0x01; // PRB_P17 (Filter C) or PRB_P1 (Filter A)
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::DAC_SPB_CNTRL, dac_prb_val);

    bool use_filters = (device_config.upper_cutoff_frequency <
                        device_config.sampling_rate / 2) ||
                       (device_config.lower_cutoff_frequency > 0);
    uint8_t adc_prb_val;
    if (device_config.sampling_rate == AUDIO_SAMPLE_RATE_192KHz)
    {
        adc_prb_val =
            use_filters ? 0x0E : 0x0D; // PRB_R14 or PRB_R13 (Filter Type C)
    }
    else
    {
        adc_prb_val =
            use_filters ? 0x02 : 0x01; // PRB_R2 or PRB_R1 (Filter Type A/B)
    }
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::ADC_SPB_CNTRL, adc_prb_val);

    // I2S mode (0x00 for I2S), Word length
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::AUD_IFACE_SET_1, word_length_val);

    // Master mode (BCLK and WCLK are outputs) if I2S_PERIPHERAL_MASTER is true
    if (I2S_PERIPHERAL_MASTER)
    {
        tlvwrite(0,
                 TLV320AIC3204_PAGE_0_REG::AUD_IFACE_SET_1,
                 word_length_val | 0x0C); // BCLK, WCLK as outputs, I2S mode
    }

    if (TLV320_LOOPBACK_MODE == 2)
    {
        aud_iface_3_val |=
            0x10; // Enable ADC-to-DAC Loopback (Digital Loopback)
    }
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::AUD_IFACE_SET_3, aud_iface_3_val);

    // 2. ADC/DAC Routing & Power

    // Enable or disable MICBIAS based on configuration
    if (TLV320_ENABLE_MICBIAS)
    {
        tlvwrite(1,
                 TLV320AIC3204_PAGE_1_REG::MICBIAS_CONFIG,
                 TLV320_MICBIAS_VAL);
    }
    else
    {
        tlvwrite(1,
                 TLV320AIC3204_PAGE_1_REG::MICBIAS_CONFIG,
                 0x00); // Powered down
    }

    // Route inputs dynamically based on selected input source (LINE1, LINE2,
    // LINE3)
    uint8_t route_val = 0x10; // Default to IN2 (LINE2 / MIC)
    switch (input_source)
    {
        case i2s_peripheral_input_source_t::LINE_IN:
        case i2s_peripheral_input_source_t::MIC:
            route_val = 0x10; // IN1_L / IN1_R
            break;
        case i2s_peripheral_input_source_t::LINE1:
            route_val = 0x40; // IN1_L / IN1_R
            break;
        case i2s_peripheral_input_source_t::LINE2:
            route_val = 0x10; // IN2_L / IN2_R
            break;
        case i2s_peripheral_input_source_t::LINE3:
            route_val = 0x04; // IN3_L / IN3_R
            break;
        default:
            route_val = 0x10; // IN2_L / IN2_R
            break;
    }

    // Route IN_L to Left MicPGA positive terminal
    tlvwrite(1,
             TLV320AIC3204_PAGE_1_REG::LMICPGA_POS_IN_ROUTE_CONFIG,
             route_val);

    // CM to Left PGA ngative terminal
    tlvwrite(1,
             TLV320AIC3204_PAGE_1_REG::LMICPGA_NEG_IN_ROUTE_CONFIG,
             0x40); // CM to Left PGA negative terminal
    // Route IN_R to Right MicPGA positive terminal
    tlvwrite(1,
             TLV320AIC3204_PAGE_1_REG::RMICPGA_POS_IN_ROUTE_CONFIG,
             route_val);

    // CM to Right PGA negative terminal
    tlvwrite(1,
             TLV320AIC3204_PAGE_1_REG::RMICPGA_NEG_IN_ROUTE_CONFIG,
             0x40); // CM to Right PGA negative terminal

    // float active inputs, pull unused inputs to CM
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::FLOAT_IN_CONFIG, 0x3C);

    // Unmute Left/Right MicPGA and set gain to 0dB (PGA gain of 0x28 is +20dB)
    tlvwrite(1,
             TLV320AIC3204_PAGE_1_REG::LMICPGA_VOL_CNTRL,
             0x28); // Unmute Left MicPGA
    tlvwrite(1,
             TLV320AIC3204_PAGE_1_REG::RMICPGA_VOL_CNTRL,
             0x28); // Unmute Right MicPGA

    // set ADC powertune to lowest power (PTM_R4)
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::ADC_PWR_TUNE_CONFIG, 0x00);

    // Route Left DAC to HPL, Right DAC to HPR (with Mixer bypass routing if
    // loopback mode is Analog)
    uint8_t hpl_route = 0x08; // Left DAC to HPL
    uint8_t hpr_route = 0x08; // Right DAC to HPR
    if (TLV320_LOOPBACK_MODE == 1)
    {
        hpl_route |= 0x02; // Route Left Mixer to HPL
        hpr_route |= 0x02; // Route Right Mixer to HPR

        // PGA to Mixer bypass set to 0dB gain
        tlvwrite(1, TLV320AIC3204_PAGE_1_REG::MXR_AMP_LVOL_CNTRL, 0x00);
        tlvwrite(1, TLV320AIC3204_PAGE_1_REG::MXR_AMP_RVOL_CNTRL, 0x00);
    }

    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::HPL_ROUTE_SELECT, hpl_route);
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::HPR_ROUTE_SELECT, hpr_route);

    // Unmute HPL/HPR driver, set gain to 0.0 dB
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::HPL_DR_GAIN_SET, 0x00);
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::HPR_DR_GAIN_SET, 0x00);

    // Power up HPL and HPR (only if loopback diagnostics is enabled)
    uint8_t out_dr_pwr = (TLV320_LOOPBACK_MODE != 0) ? 0x30 : 0x00;
    tlvwrite(1,
             TLV320AIC3204_PAGE_1_REG::OUT_DR_PWR_CNTRL,
             out_dr_pwr); // HPL/HPR power control

    w_hal_system_delay_ms(10);

    // 3. Filtering
    if (use_filters)
    {
        biquad_filter_st bypass = {
            0x7FFFFF, // N0 (1.0 in Q1.23 format)
            0,        // N1
            0,        // N2
            0,        // D1
            0         // D2
        };

        // Biquad A (LPF): Left starts at Page 8, Reg 36; Right at Reg 100
        if (device_config.upper_cutoff_frequency <
            device_config.sampling_rate / 2)
        {
            biquad_filter_st biquad_lpf;
            calc_biquad(filter_type_t::LOW_PASS_FILTER,
                        device_config.upper_cutoff_frequency,
                        device_config.sampling_rate,
                        biquad_lpf);
            set_biquad_filter_coef(8, 36, biquad_lpf);
            set_biquad_filter_coef(8, 100, biquad_lpf);
        }
        else
        {
            set_biquad_filter_coef(8, 36, bypass);
            set_biquad_filter_coef(8, 100, bypass);
        }

        // Biquad B (HPF): Left starts at Page 8, Reg 56; Right at Reg 120
        // (spans Page 8 & 9)
        if (device_config.lower_cutoff_frequency > 0)
        {
            biquad_filter_st biquad_hpf;
            calc_biquad(filter_type_t::HIGH_PASS_FILTER,
                        device_config.lower_cutoff_frequency,
                        device_config.sampling_rate,
                        biquad_hpf);
            set_biquad_filter_coef(8, 56, biquad_hpf);
            set_biquad_filter_coef(8, 120, biquad_hpf);
        }
        else
        {
            set_biquad_filter_coef(8, 56, bypass);
            set_biquad_filter_coef(8, 120, bypass);
        }

        // Enable ADC Adaptive Filter (Page 8, Reg 1) to apply the programmed
        // coefficients
        tlvwrite(8, TLV320AIC3204_PAGE_8_REG::ADC_ADAPTIVE_FILTER_CONFIG, 0x04);
    }

    // Power up Left/Right ADC
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::ADC_CHNL_SET, 0xC0);
    // Unmute Left/Right ADC
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::ADC_FINE_GAIN_ADJ, 0x00);
    // Power up Left/Right DAC (only if loopback diagnostics is enabled)
    if (TLV320_LOOPBACK_MODE != 0)
    {
        tlvwrite(0,
                 TLV320AIC3204_PAGE_0_REG::DAC_SETUP_1,
                 0xD6); // Power up Left/Right DAC, route Left data to Left DAC,
                        // Right data to Right DAC
        tlvwrite(0, TLV320AIC3204_PAGE_0_REG::DAC_SETUP_2, 0x00); // Unmute DACs
    }
    else
    {
        tlvwrite(0,
                 TLV320AIC3204_PAGE_0_REG::DAC_SETUP_1,
                 0x00); // Power down Left/Right DAC
        tlvwrite(0,
                 TLV320AIC3204_PAGE_0_REG::DAC_SETUP_2,
                 0x0C); // Mute Left/Right DAC
    }

    w_hal_system_delay_ms(10);

    return wHalErrorCode::NO_ERROR;
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
    i2s_sample_length   = device_config.audio_sampling_width;

    wHalErrorCode error = w_hal_i2s_power_peripheral(true);

    error               = w_hal_i2c_init(AIC3204_I2C_CHANNEL,
                           whal_i2c_mode::MASTER,
                           I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    g_current_page = -1;
    error          = codec_stage1_init();
    error          = w_hal_i2s_peripheral_validate_config(device_config, true);
    error          = codec_stage2_init(device_config);
    error          = codec_stage3_init(device_config, input_source);
    error          = w_hal_i2c_teardown(AIC3204_I2C_CHANNEL);

    return error;
}

/**
 * @brief Teardown sequence for I2S Peripheral
 *
 * @return wHalErrorCode error code
 */
wHalErrorCode w_hal_i2s_teardown_peripheral()
{
    wHalErrorCode error = w_hal_i2c_init(AIC3204_I2C_CHANNEL,
                                         whal_i2c_mode::MASTER,
                                         I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
        return error;

    g_current_page = -1;

    // Mute DACs and ADCs
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::DAC_SETUP_2, 0x0F);
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::ADC_FINE_GAIN_ADJ, 0x88);

    // Power down DACs and ADCs
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::DAC_SETUP_1, 0x00);
    tlvwrite(0, TLV320AIC3204_PAGE_0_REG::ADC_CHNL_SET, 0x00);

    // Power down HPL/HPR
    tlvwrite(1, TLV320AIC3204_PAGE_1_REG::OUT_DR_PWR_CNTRL, 0x00);

    error = w_hal_i2c_teardown(AIC3204_I2C_CHANNEL);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Power down peripheral
    return w_hal_i2s_power_peripheral(false);
}

/**
 * @brief Get I2S Control Bit lengths
 *
 * @param sample_length [out]: bits per sample over interface
 * @param bits_per_ws [out]: number of cycles per channel when ws/lrclk toggles
 */
void w_hal_i2s_get_channel_lengths(uint32_t &sample_length,
                                   uint32_t &bits_per_ws)
{
    sample_length = bits_per_ws = i2s_sample_length;
}

#endif // __CORE__ == AUDIO_CORE

/**
 * @brief Check whether SCLK and LRCLK is coming from peripheral
 *
 * @return true if SCLK and LRCLK are driven by the codec (master mode)
 * @return false if SCLK and LRCLK are driven by the controller
 */
bool w_hal_i2s_is_sck_from_peripheral()
{
    return I2S_PERIPHERAL_MASTER;
}

#endif // AUDIO_CORE != INVALID_CORE_NUM