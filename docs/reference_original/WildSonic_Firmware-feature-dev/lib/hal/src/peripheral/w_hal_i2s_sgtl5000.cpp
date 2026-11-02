#if AUDIO_CORE != INVALID_CORE_NUM

#include "w_hal_i2s.hpp" //I2S header file
#include "w_hal_i2s_internal.hpp"

#include "device_config.hpp"
#include "trace.hpp"

#include <math.h>

static const bool     I2S_PERIPHERAL_MASTER = true;

static const uint32_t BIQUAD_QUALITY_FACTOR = 1; // Quality factor of an filter

static const int      MAX_GAIN_TIMES_TEN =
    225; // Max Gain in multiple of ten in normal operation
static const int MIN_GAIN_TIMES_TEN =
    0; // Min Gain in multiple of ten in normal operation
static const int M6DB_MAX_GAIN_TIMES_TEN =
    165; // Max Gain in M6DB operation, i.e reduction of gain value by 6Db than
         // normal operation
static const int M6DB_MIN_GAIN_TIMES_TEN =
    -60; // Min Gain in M6DB operation, i.e reduction of gain value by 6Db than
         // normal operation
static const int DIFFERENCE_BETWEEN_GAINS = 15;
static const int HALF_DIFFERENCE_BETWEEN_GAINS =
    (DIFFERENCE_BETWEEN_GAINS - 1) / 2 + 1;

static int get_gain_reg_field_values(int      desired_gain_times_ten,
                                     uint8_t &ADC_Gain,
                                     uint8_t &adc_vol_m6db);

#if __CORE__ == AUDIO_CORE

#include "w_hal_i2c.hpp"
#include "w_hal_system.hpp"

#include "assert.hpp"
#include "constants.hpp"

static const uint32_t I2C_FREQUENCY          = 100000; // Standard frequency

static const uint8_t  SGTL5000_I2C_CHANNEL   = 0x01;
static const uint16_t SGTL5000_SLAVE_ADDRESS = 0x0A;
static const uint8_t  SGTL5000_REG_ADDRESS_SIZE_BYTES = 2;
static const uint8_t  SGTL5000_REG_DATA_SIZE_BYTES    = 2;

/**
 * @brief Register addresses for TI SGTL5000 Codec
 */
enum class SGTL5000_CMD : uint16_t
{
    CHIP_ID                   = 0x0000,
    CHIP_DIG_POWER            = 0x0002,
    CHIP_CLK_CTRL             = 0x0004,
    CHIP_I2S_CTRL             = 0x0006,
    CHIP_SSS_CTRL             = 0x000A,
    CHIP_ADCDAC_CTRL          = 0x000E,
    CHIP_DAC_VOL              = 0x0010,
    CHIP_PAD_STRENGTH         = 0x0014,
    CHIP_ANA_ADC_CTRL         = 0x0020,
    CHIP_ANA_HP_CTRL          = 0x0022,
    CHIP_ANA_CTRL             = 0x0024,
    CHIP_LINREG_CTRL          = 0x0026,
    CHIP_REF_CTRL             = 0x0028,
    CHIP_MIC_CTRL             = 0x002A,
    CHIP_LINE_OUT_CTRL        = 0x002C,
    CHIP_LINE_OUT_VOL         = 0x002E,
    CHIP_ANA_POWER            = 0x0030,
    CHIP_PLL_CTRL             = 0x0032,
    CHIP_CLK_TOP_CTRL         = 0x0034,
    CHIP_ANA_STATUS           = 0x0036,
    CHIP_ANA_TEST1            = 0x0038,
    CHIP_ANA_TEST2            = 0x003A,
    CHIP_SHORT_CTRL           = 0x003C,
    DAP_CONTROL               = 0x0100,
    DAP_PEQ                   = 0x0102,
    DAP_BASS_ENHANCE          = 0x0104,
    DAP_BASS_ENHANCE_CTRL     = 0x0106,
    DAP_AUDIO_EQ              = 0x0108,
    DAP_SGTL_SURROUND         = 0x010A,
    DAP_FILTER_COEF_ACCESS    = 0x010C,
    DAP_COEF_WR_B0_MSB        = 0x010E,
    DAP_COEF_WR_B0_LSB        = 0x0110,
    DAP_AUDIO_EQ_BASS_BAND0   = 0x0116,
    DAP_AUDIO_EQ_BAND1        = 0x0118,
    DAP_AUDIO_EQ_BAND2        = 0x011A,
    DAP_AUDIO_EQ_BAND3        = 0x011C,
    DAP_AUDIO_EQ_TREBLE_BAND4 = 0x011E,
    DAP_MAIN_CHAN             = 0x0120,
    DAP_MIX_CHAN              = 0x0122,
    DAP_AVC_CTRL              = 0x0124,
    DAP_AVC_THRESHOLD         = 0x0126,
    DAP_AVC_ATTACK            = 0x0128,
    DAP_AVC_DECAY             = 0x012A,
    DAP_COEF_WR_B1_MSB        = 0x012C,
    DAP_COEF_WR_B1_LSB        = 0x012E,
    DAP_COEF_WR_B2_MSB        = 0x0130,
    DAP_COEF_WR_B2_LSB        = 0x0132,
    DAP_COEF_WR_A1_MSB        = 0x0134,
    DAP_COEF_WR_A1_LSB        = 0x0136,
    DAP_COEF_WR_A2_MSB        = 0x0138,
    DAP_COEF_WR_A2_LSB        = 0x013A,
};

enum class filter_type_t : uint32_t
{
    NONE,
    LOW_PASS_FILTER,
    HIGH_PASS_FILTER,
    BAND_PASS_FILTER,
};

/**
 * @brief Register address 0x0002
 */
union chip_dig_power_st
{
    uint16_t dw;
    struct
    {
        uint16_t i2s_in_powerup  : 1;
        uint16_t i2s_out_powerup : 1;
        uint16_t rsvd0           : 2;
        uint16_t dap_powerup     : 1;
        uint16_t dac_powerup     : 1;
        uint16_t adc_powerup     : 1;
        uint16_t rsvd1           : 9;
    } bits;
};

/**
 * @brief Register address 0x0004
 */
union chip_clk_ctrl_reg_st
{
    uint16_t dw;
    struct
    {
        uint16_t mclk_freq : 2;
        uint16_t sys_fs    : 2;
        uint16_t rate_mode : 2;
        uint16_t rsvd      : 10;
    } bits;
};

/**
 * @brief Register address 0x0006
 */
union chip_i2s_ctrl_reg_st
{
    uint16_t dw;
    struct
    {
        uint16_t lr_pol   : 1;
        uint16_t lr_align : 1;
        uint16_t i2s_mode : 2;
        uint16_t dlen     : 2;
        uint16_t sclk_inv : 1;
        uint16_t ms       : 1;
        uint16_t sclkfreq : 1;
        uint16_t rsvd     : 7;
    } bits;
};

/**
 * @brief Register address 0x000A
 */
union chip_sss_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t i2s_select     : 2;
        uint16_t rsvd0          : 2;
        uint16_t dac_select     : 2;
        uint16_t dap_select     : 2;
        uint16_t dap_mix_select : 2;
        uint16_t i2s_lrswap     : 1;
        uint16_t rsvd1          : 1;
        uint16_t dac_lrswap     : 1;
        uint16_t dap_lrswap     : 1;
        uint16_t dap_mix_lrswap : 1;
        uint16_t rsvd2          : 1;
    } bits;
};

/**
 * @brief Register address 0x000E
 */
union chip_adcdac_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t adc_hpf_bypass     : 1;
        uint16_t adc_hpf_freeze     : 1;
        uint16_t dac_mute_left      : 1;
        uint16_t dac_mute_right     : 1;
        uint16_t rsvd0              : 4;
        uint16_t vol_expo_ramp      : 1;
        uint16_t vol_ramp_en        : 1;
        uint16_t rsvd1              : 2;
        uint16_t vol_busy_dac_left  : 1;
        uint16_t vol_busy_dac_right : 1;
        uint16_t rsvd2              : 2;
    } bits;
};

/**
 * @brief Register address 0x0010
 */
union chip_dac_vol_st
{
    uint16_t dw;
    struct
    {
        uint16_t dac_vol_left  : 8;
        uint16_t dac_vol_right : 8;
    } bits;
};

/**
 * @brief Register address 0x0014
 */
union chip_pad_strength_st
{
    uint16_t dw;
    struct
    {
        uint16_t ctrl_clk  : 2;
        uint16_t ctrl_data : 2;
        uint16_t i2s_dout  : 2;
        uint16_t i2s_sclk  : 2;
        uint16_t i2s_lrclk : 2;
        uint16_t rsvd      : 6;
    } bits;
};

/**
 * @brief Register address 0x0020
 */
union chip_ana_adc_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t adc_vol_left  : 4;
        uint16_t adc_vol_right : 4;
        uint16_t adc_vol_m6db  : 1;
        uint16_t rsvd          : 7;
    } bits;
};

/**
 * @brief Register address 0x0022
 */
union chip_ana_hp_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t hp_vol_left  : 7;
        uint16_t rsvd0        : 1;
        uint16_t hp_vol_right : 7;
        uint16_t rsvd1        : 1;
    } bits;
};

/**
 * @brief Register address 0x0024
 */
union chip_ana_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t mute_adc   : 1;
        uint16_t en_zcd_adc : 1;
        uint16_t select_adc : 1;
        uint16_t rsvd0      : 1;
        uint16_t mute_hp    : 1;
        uint16_t en_zcd_hp  : 1;
        uint16_t select_hp  : 1;
        uint16_t rsvd1      : 1;
        uint16_t mute_lo    : 1;
        uint16_t rsvd2      : 7;
    } bits;
};

/**
 * @brief Register address 0x0026
 */
union chip_linreg_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t d_programming  : 4;
        uint16_t rsvd0          : 1;
        uint16_t vddc_assn_ovrd : 1;
        uint16_t vddc_man_assn  : 1;
        uint16_t rsvd1          : 9;
    } bits;
};

/**
 * @brief Register address 0x0028
 */
union chip_ref_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t small_pop : 1;
        uint16_t bias_ctrl : 3;
        uint16_t vag_val   : 5;
        uint16_t rsvd      : 7;
    } bits;
};

/**
 * @brief Register address 0x002A
 */
union chip_mic_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t gain          : 2;
        uint16_t rsvd0         : 2;
        uint16_t bias_volt     : 3;
        uint16_t rsvd1         : 1;
        uint16_t bias_resistor : 2;
        uint16_t rsvd2         : 6;
    } bits;
};

/**
 * @brief Register address 0x002C
 *
 */
union chip_line_out_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t lo_vagcntrl : 6;
        uint16_t rsvd0       : 2;
        uint16_t out_current : 4;
        uint16_t rsvd1       : 4;
    } bits;
};

/**
 * @brief Register address 0x002E
 *
 */
union chip_line_out_vol_st
{
    uint16_t dw;
    struct
    {
        uint16_t lo_vol_left  : 5;
        uint16_t rsvd0        : 3;
        uint16_t lo_vol_right : 5;
        uint16_t rsvd1        : 3;
    } bits;
};

/**
 * @brief Register address 0x0030
 */
union chip_ana_power_st
{
    uint16_t dw;
    struct
    {
        uint16_t lineout_powerup           : 1;
        uint16_t adc_powerup               : 1;
        uint16_t capless_headphone_powerup : 1;
        uint16_t dac_powerup               : 1;
        uint16_t headphone_powerup         : 1;
        uint16_t reftop_powerup            : 1;
        uint16_t adc_mono                  : 1;
        uint16_t vag_powerup               : 1;
        uint16_t vcoamp_powerup            : 1;
        uint16_t linreg_d_powerup          : 1;
        uint16_t pll_powerup               : 1;
        uint16_t vddc_chrgpmp_powerup      : 1;
        uint16_t startup_powerup           : 1;
        uint16_t linreg_simple_powerup     : 1;
        uint16_t dac_mono                  : 1;
        uint16_t rsvd                      : 1;
    } bits;
};

/**
 * @brief Register address 0x0032
 */
union chip_pll_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t frac_divisor : 11;
        uint16_t int_divisor  : 5;
    } bits;
};

/**
 * @brief Register address 0x0034
 */
union chip_clk_top_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t rsvd0           : 3;
        uint16_t input_freq_div2 : 1;
        uint16_t rsvd1           : 7;
        uint16_t enable_int_osc  : 1;
        uint16_t rsvd2           : 4;
    } bits;
};

/**
 * @brief Register address 0x003C
 */
union chip_short_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t mode_cm : 2;
        uint16_t mode_lr : 2;
        uint16_t lvladjc : 3;
        uint16_t rsvd0   : 1;
        uint16_t lvladjl : 3;
        uint16_t rsvd1   : 1;
        uint16_t lvladjr : 3;
        uint16_t rsvd2   : 1;
    } bits;
};

/**
 * @brief Register address 0x0100
 */
union dap_control_st
{
    uint16_t dw;
    struct
    {
        uint16_t dap_en : 1;
        uint16_t rsvd0  : 3;
        uint16_t mix_en : 1;
        uint16_t rsvd1  : 11;
    } bits;
};
/**
 * @brief Register address 0x0102
 *
 */
union dap_peq_st
{
    uint16_t dw;
    struct
    {
        uint16_t en    : 3;
        uint16_t rsvd0 : 13;
    } bits;
};

/**
 * @brief Register address 0x0104
 */
union dap_bass_enhance_st
{
    uint16_t dw;
    struct
    {
        uint16_t en         : 1;
        uint16_t rsvd0      : 3;
        uint16_t cutoff     : 3;
        uint16_t rsvd1      : 1;
        uint16_t bypass_hpf : 1;
        uint16_t rsvd3      : 7;
    } bits;
};

/**
 * @brief Register address 0x0106
 */
union dap_bass_enhance_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t bass_level : 7;
        uint16_t rsvd0      : 1;
        uint16_t lr_level   : 6;
        uint16_t rsvd1      : 2;
    } bits;
};

/**
 * @brief Register address 0x0108
 */
union dap_audio_eq_st
{
    uint16_t dw;
    struct
    {
        uint16_t en    : 2;
        uint16_t rsvd0 : 14;
    } bits;
};

/**
 * @brief Register address 0x010A
 */
union dap_sgtl_surround_st
{
    uint16_t dw;
    struct
    {
        uint16_t select     : 2;
        uint16_t rsvd0      : 2;
        uint16_t width_ctrl : 3;
        uint16_t rsvd1      : 9;
    } bits;
};

/**
 * @brief Register address 0x010C
 */
union dap_filter_coef_access_st
{
    uint16_t dw;
    struct
    {
        uint16_t index : 8;
        uint16_t wr    : 1;
        uint16_t rsvd0 : 7;
    } bits;
};

/**
 * @brief Register address 0x010E
 */
union dap_coef_wr_b0_msb_st
{
    uint16_t dw;
    struct
    {
        uint16_t msb : 16;
    } bits;
};

/**
 * @brief Register address 0x0110
 */
union dap_coef_wr_b0_lsb_st
{
    uint16_t dw;
    struct
    {
        uint16_t lsb   : 4;
        uint16_t rsvd0 : 12;
    } bits;
};

/**
 * @brief Register address 0x0116
 */
union dap_audio_eq_bass_band0_st
{
    uint16_t dw;
    struct
    {
        uint16_t volume : 7;
        uint16_t rsvd0  : 9;
    } bits;
};

/**
 * @brief Register address 0x0118
 */
union dap_audio_eq_band1_st
{
    uint16_t dw;
    struct
    {
        uint16_t volume : 7;
        uint16_t rsvd0  : 9;
    } bits;
};

/**
 * @brief Register address 0x011A
 */
union dap_audio_eq_band2_st
{
    uint16_t dw;
    struct
    {
        uint16_t volume : 7;
        uint16_t rsvd0  : 9;
    } bits;
};

/**
 * @brief Register address 0x011C
 */
union dap_audio_eq_band3_st
{
    uint16_t dw;
    struct
    {
        uint16_t volume : 7;
        uint16_t rsvd0  : 9;
    } bits;
};

/**
 * @brief Register address 0x011E
 */
union dap_audio_eq_treble_band4_st
{
    uint16_t dw;
    struct
    {
        uint16_t volume : 7;
        uint16_t rsvd0  : 9;
    } bits;
};

/**
 * @brief Register address 0x0120
 *
 */
union dap_main_chan_st
{
    uint16_t dw;
    struct
    {
        uint16_t vol : 16;
    } bits;
};

/**
 * @brief Register address 0x0122
 *
 */
union dap_mix_chan_st
{
    uint16_t dw;
    struct
    {
        uint16_t vol : 16;
    } bits;
};

/**
 * @brief Register address 0x0124
 *
 */
union dap_avc_ctrl_st
{
    uint16_t dw;
    struct
    {
        uint16_t en            : 1;
        uint16_t rsvd0         : 4;
        uint16_t hard_limit_en : 1;
        uint16_t rsvd1         : 2;
        uint16_t lbi_response  : 2;
        uint16_t rsvd2         : 2;
        uint16_t max_gain      : 2;
        uint16_t rsvd3         : 1;
        uint16_t rsvd4         : 1;

    } bits;
};

/**
 * @brief Register address 0x0126
 *
 */
union dap_avc_threshold_st
{
    uint16_t dw;
    struct
    {
        uint16_t thresh : 16;
    } bits;
};

/**
 * @brief Register address 0x0128
 *
 */
union dap_avc_attack_st
{
    uint16_t dw;
    struct
    {
        uint16_t rate  : 12;
        uint16_t rsvd0 : 4;
    } bits;
};

/**
 * @brief Register address 0x012A
 *
 */
union dap_avc_decay_st
{
    uint16_t dw;
    struct
    {
        uint16_t rate  : 12;
        uint16_t rsvd0 : 4;
    } bits;
};

/**
 * @brief Register address 0x012C
 */
union dap_coef_wr_b1_msb_st
{
    uint16_t dw;
    struct
    {
        uint16_t msb : 16;
    } bits;
};

/**
 * @brief Register address 0x012E
 */
union dap_coef_wr_b1_lsb_st
{
    uint16_t dw;
    struct
    {
        uint16_t lsb   : 4;
        uint16_t rsvd0 : 12;
    } bits;
};

/**
 * @brief Register address 0x0130
 */
union dap_coef_wr_b2_msb_st
{
    uint16_t dw;
    struct
    {
        uint16_t msb : 16;
    } bits;
};

/**
 * @brief Register address 0x0132
 */
union dap_coef_wr_b2_lsb_st
{
    uint16_t dw;
    struct
    {
        uint16_t lsb   : 4;
        uint16_t rsvd0 : 12;
    } bits;
};

/**
 * @brief Register address 0x0134
 */
union dap_coef_wr_a1_msb_st
{
    uint16_t dw;
    struct
    {
        uint16_t msb : 16;
    } bits;
};

/**
 * @brief Register address 0x0136
 */
union dap_coef_wr_a1_lsb_st
{
    uint16_t dw;
    struct
    {
        uint16_t lsb   : 4;
        uint16_t rsvd0 : 12;
    } bits;
};

/**
 * @brief Register address 0x0138
 */
union dap_coef_wr_a2_msb_st
{
    uint16_t dw;
    struct
    {
        uint16_t msb : 16;
    } bits;
};

/**
 * @brief Register address 0x0140
 */
union dap_coef_wr_a2_lsb_st
{
    uint16_t dw;
    struct
    {
        uint16_t lsb   : 4;
        uint16_t rsvd0 : 12;
    } bits;
};

/**
 * @brief biquad filter coefficients
 *
 */
struct biquad_filter_st
{
    int32_t B0;
    int32_t B1;
    int32_t B2;
    int32_t A1;
    int32_t A2;
};

static uint32_t i2s_sample_length;

/**
 * @brief Read from register on codec
 *
 * @param address register address
 * @return uint16_t data to read from register
 */
static uint16_t i2c_read(SGTL5000_CMD address)
{
    uint16_t read_word = 0;
    uint8_t  read_bytes[4];

    for (uint32_t index = 0; index < SGTL5000_REG_ADDRESS_SIZE_BYTES; index++)
    {
        read_bytes[index] =
            (uint8_t)(static_cast<uint16_t>(address) >>
                      (CHAR_BIT *
                       (SGTL5000_REG_ADDRESS_SIZE_BYTES - 1 - index)));
    }

    for (uint32_t index = 0; index < SGTL5000_REG_DATA_SIZE_BYTES; index++)
    {
        read_bytes[index + SGTL5000_REG_ADDRESS_SIZE_BYTES] = 0xFF;
    }

    wHalErrorCode error =
        w_hal_i2c_write_read_data(SGTL5000_I2C_CHANNEL,
                                  read_bytes,
                                  SGTL5000_REG_ADDRESS_SIZE_BYTES,
                                  &read_bytes[SGTL5000_REG_ADDRESS_SIZE_BYTES],
                                  SGTL5000_REG_DATA_SIZE_BYTES,
                                  false,
                                  SGTL5000_SLAVE_ADDRESS,
                                  true);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_READ_COMPLETE);

    error = w_hal_i2c_clear_completion_status(SGTL5000_I2C_CHANNEL);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    for (uint32_t index = 0; index < SGTL5000_REG_DATA_SIZE_BYTES; index++)
    {
        read_word |= read_bytes[index + SGTL5000_REG_ADDRESS_SIZE_BYTES]
                     << (CHAR_BIT * (SGTL5000_REG_DATA_SIZE_BYTES - 1 - index));
    }

    return read_word;
}

/**
 * @brief Write to register on codec
 *
 * @param address register address
 * @param word data to write to register
 */
static void i2c_write(SGTL5000_CMD address, uint16_t word)
{
    // Workaround for register writes not going through
    // Before writing register, read it first.
    i2c_read(address);

    wHalErrorCode error = w_hal_i2c_write_word<SGTL5000_REG_ADDRESS_SIZE_BYTES,
                                               SGTL5000_REG_DATA_SIZE_BYTES>(
        SGTL5000_I2C_CHANNEL,
        static_cast<uint16_t>(address),
        word,
        false,
        SGTL5000_SLAVE_ADDRESS,
        true);

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_COMPLETE);
}

/**
 * @brief Get the quantized value of the coefficient
 *
 * @param coeff : Coefficient which need to be quantized
 * @return float : The quantized value of the coefficient
 */
static float get_quantized_value(float coeff)
{
    if (coeff >= 0)
    {
        coeff = (floor(coeff * pow(2, 19) + 0.5));
    }
    else
    {
        coeff = (ceil(coeff * pow(2, 19) + 0.5));
    }
    return coeff;
}

/**
 * @brief To calculate the biquad filter coefficients
 *
 * @param filter_type : The type of the filter i.e LPF/HPF/BPF
 * @param fc : Cut off frequency for the filtering
 * @param fs : Sampling rate
 * @param biquad_filter[out] : biquad filter coefficients
 */
static void calc_biquad(filter_type_t     filter_type,
                        float             fc,
                        uint64_t          fs,
                        biquad_filter_st &biquad_filter)
{
    float W0 = 2 * PI * fc / fs;
    float alpha =
        sinf(W0) * sinh((log(2) / 2) * BIQUAD_QUALITY_FACTOR * W0 / sinf(W0));
    float a0 = 0, a1 = 0, a2 = 0, b0 = 0, b1 = 0, b2 = 0;

    switch (filter_type)
    {
        case filter_type_t::LOW_PASS_FILTER: // LOW_PASS_FILTER
        {
            a0 = 1.0F + alpha;
            a1 = get_quantized_value((((2.0F * cosf(W0)) / a0) / 2));
            a2 = get_quantized_value((((alpha - 1.0F) / a0) / 2));
            b0 = get_quantized_value(((((1.0F - cosf(W0)) / 2.0F) / a0) / 2));
            b1 = get_quantized_value((((1.0F - cosf(W0)) / a0) / 2));
            b2 = b0;
            break;
        }
        case filter_type_t::HIGH_PASS_FILTER: // HIGH_PASS_FILTER
        {
            a0 = 1.0F + alpha;
            a1 = get_quantized_value((((2.0F * cosf(W0)) / a0) / 2));
            a2 = get_quantized_value((((alpha - 1.0F) / a0) / 2));
            b0 = get_quantized_value(((((1.0F + cosf(W0)) / 2.0F) / a0) / 2));
            b1 = get_quantized_value(-(((1.0F + cosf(W0)) / a0) / 2));
            b2 = b0;
            break;
        }
        case filter_type_t::BAND_PASS_FILTER: // BAND_PASS_FILTER
        {
            a0 = 1.0F + alpha;
            a1 = get_quantized_value((((2.0F * cosf(W0)) / a0) / 2));
            a2 = get_quantized_value((((alpha - 1.0F) / a0) / 2));
            b0 = get_quantized_value(((alpha / a0) / 2));
            b1 = 0;
            b2 = -b0;
            break;
        }
        default:
        {
            break;
        }
    }

    biquad_filter.A1 = static_cast<int32_t>(a1);
    biquad_filter.A2 = static_cast<int32_t>(a2);
    biquad_filter.B0 = static_cast<int32_t>(b0);
    biquad_filter.B1 = static_cast<int32_t>(b1);
    biquad_filter.B2 = static_cast<int32_t>(b2);
}

/**
 * @brief Set the biquad filter coefficient object
 *
 * @param biquad_filter[out] : biquad filter coefficients
 */
void static set_biquad_filter_coef(biquad_filter_st &biquad_filter)
{
    dap_coef_wr_b0_msb_st dap_coef_wr_b0_msb;
    dap_coef_wr_b0_msb.dw       = 0;
    dap_coef_wr_b0_msb.bits.msb = biquad_filter.B0 >> 4;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_B0_MSB, dap_coef_wr_b0_msb.dw);

    dap_coef_wr_b0_lsb_st dap_coef_wr_b0_lsb;
    dap_coef_wr_b0_lsb.dw       = 0;
    dap_coef_wr_b0_lsb.bits.lsb = biquad_filter.B0;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_B0_LSB, dap_coef_wr_b0_lsb.dw);

    dap_coef_wr_b1_msb_st dap_coef_wr_b1_msb;
    dap_coef_wr_b1_msb.dw       = 0;
    dap_coef_wr_b1_msb.bits.msb = biquad_filter.B1 >> 4;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_B1_MSB, dap_coef_wr_b1_msb.dw);

    dap_coef_wr_b1_lsb_st dap_coef_wr_b1_lsb;
    dap_coef_wr_b1_lsb.dw       = 0;
    dap_coef_wr_b1_lsb.bits.lsb = biquad_filter.B1;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_B1_LSB, dap_coef_wr_b1_lsb.dw);

    dap_coef_wr_b2_msb_st dap_coef_wr_b2_msb;
    dap_coef_wr_b2_msb.dw       = 0;
    dap_coef_wr_b2_msb.bits.msb = biquad_filter.B2 >> 4;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_B2_MSB, dap_coef_wr_b2_msb.dw);

    dap_coef_wr_b2_lsb_st dap_coef_wr_b2_lsb;
    dap_coef_wr_b2_lsb.dw       = 0;
    dap_coef_wr_b2_lsb.bits.lsb = biquad_filter.B2;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_B2_LSB, dap_coef_wr_b2_lsb.dw);

    dap_coef_wr_a1_msb_st dap_coef_wr_a1_msb;
    dap_coef_wr_a1_msb.dw       = 0;
    dap_coef_wr_a1_msb.bits.msb = biquad_filter.A1 >> 4;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_A1_MSB, dap_coef_wr_a1_msb.dw);

    dap_coef_wr_a1_lsb_st dap_coef_wr_a1_lsb;
    dap_coef_wr_a1_lsb.dw       = 0;
    dap_coef_wr_a1_lsb.bits.lsb = biquad_filter.A1;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_A1_LSB, dap_coef_wr_a1_lsb.dw);

    dap_coef_wr_a2_msb_st dap_coef_wr_a2_msb;
    dap_coef_wr_a2_msb.dw       = 0;
    dap_coef_wr_a2_msb.bits.msb = biquad_filter.A2 >> 4;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_A2_MSB, dap_coef_wr_a2_msb.dw);

    dap_coef_wr_a2_lsb_st dap_coef_wr_a2_lsb;
    dap_coef_wr_a2_lsb.dw       = 0;
    dap_coef_wr_a2_lsb.bits.lsb = biquad_filter.A2;
    i2c_write(SGTL5000_CMD::DAP_COEF_WR_A2_LSB, dap_coef_wr_a2_lsb.dw);
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
    wHalErrorCode error = w_hal_i2s_power_peripheral(true);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Calculate the i2s_sample_length
    if (device_config.audio_sampling_width <= BITS_PER_AUDIO_SAMPLE_16)
    {
        i2s_sample_length = BITS_PER_AUDIO_SAMPLE_16;
    }
    else if (device_config.audio_sampling_width <= BITS_PER_AUDIO_SAMPLE_24)
    {
        i2s_sample_length = BITS_PER_AUDIO_SAMPLE_24;
    }
    else
    {
        i2s_sample_length = BITS_PER_AUDIO_SAMPLE_32;
    }

    error = w_hal_i2c_init(SGTL5000_I2C_CHANNEL,
                           whal_i2c_mode::MASTER,
                           I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    w_hal_system_delay(1);

    chip_i2s_ctrl_reg_st i2s_ctrl_reg;
    i2s_ctrl_reg.dw = i2c_read(SGTL5000_CMD::CHIP_I2S_CTRL);
    if (I2S_PERIPHERAL_MASTER && i2s_ctrl_reg.bits.ms)
    {
        // Already initialized
        error = w_hal_i2c_teardown(SGTL5000_I2C_CHANNEL);
        return error;
    }

    // VDDD is externally driven with 1.8V
    chip_ana_power_st chip_ana_power;
    chip_ana_power.dw                  = 0;
    chip_ana_power.bits.reftop_powerup = 1;
    i2c_write(SGTL5000_CMD::CHIP_ANA_POWER, chip_ana_power.dw);

    // VDDA & VDDIO are tied together at 3.3V
    chip_linreg_ctrl_st chip_linreg_ctrl;
    chip_linreg_ctrl.dw                  = 0;
    chip_linreg_ctrl.bits.vddc_assn_ovrd = 1;
    chip_linreg_ctrl.bits.vddc_man_assn  = 1;
    i2c_write(SGTL5000_CMD::CHIP_LINREG_CTRL, chip_linreg_ctrl.dw);

    // VAG=1.575, normal ramp, +12.5% bias current
    chip_ref_ctrl_st chip_ref_ctrl;
    chip_ref_ctrl.dw             = 0;
    chip_ref_ctrl.bits.vag_val   = 0x1f;
    chip_ref_ctrl.bits.bias_ctrl = 1;
    i2c_write(SGTL5000_CMD::CHIP_REF_CTRL, chip_ref_ctrl.dw);

    // enable zero cross detectors
    chip_ana_ctrl_st chip_ana_ctrl;
    chip_ana_ctrl.dw              = 0;
    chip_ana_ctrl.bits.mute_adc   = 1;
    chip_ana_ctrl.bits.en_zcd_adc = 1;
    if (input_source == i2s_peripheral_input_source_t::MIC)
    {
        chip_ana_ctrl.bits.select_adc = 0; // Select mic in as input
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(input_source,
                            i2s_peripheral_input_source_t::LINE_IN);
        chip_ana_ctrl.bits.select_adc = 1; // Select line in as input
    }
    chip_ana_ctrl.bits.mute_hp = 1;
    chip_ana_ctrl.bits.mute_lo = 1;
    i2c_write(SGTL5000_CMD::CHIP_ANA_CTRL, chip_ana_ctrl.dw);

    // Enable mic bias
    chip_mic_ctrl_st chip_mic_ctrl;
    chip_mic_ctrl.dw                 = 0;
    chip_mic_ctrl.bits.bias_resistor = 0x02;
    chip_mic_ctrl.bits.bias_volt     = 0x07;
    if (input_source == i2s_peripheral_input_source_t::MIC)
    {
        // Mic preamp gain:
        // 0x0 = 0 dB, 0x1 = +20 dB, 0x2 = +30 dB, 0x3 = +40 dB
        chip_mic_ctrl.bits.gain = 0x02; // 30dB
    }
    i2c_write(SGTL5000_CMD::CHIP_MIC_CTRL, chip_mic_ctrl.dw);

    if (I2S_PERIPHERAL_MASTER)
    {
        chip_clk_top_ctrl_st chip_clk_top_ctrl;
        chip_clk_top_ctrl.dw = 0;
        if (CODEC_MCLOCK > 17000000)
        {
            chip_clk_top_ctrl.bits.input_freq_div2 = 1;
        }
        else
        {
            chip_clk_top_ctrl.bits.input_freq_div2 = 0;
        }
        i2c_write(SGTL5000_CMD::CHIP_CLK_TOP_CTRL, chip_clk_top_ctrl.dw);

        chip_pll_ctrl_st chip_pll_ctrl;
        chip_pll_ctrl.dw = 0;
        if ((device_config.sampling_rate == AUDIO_SAMPLE_RATE_22_05KHz) ||
            (device_config.sampling_rate == AUDIO_SAMPLE_RATE_44_1KHz))
        {
            // PLL_OUTPUT_FREQ = 180.6336 MHz
            const uint32_t pll_output_freq = 180633600;
            chip_pll_ctrl.bits.int_divisor = pll_output_freq / CODEC_MCLOCK;

            uint64_t remainder =
                pll_output_freq -
                (chip_pll_ctrl.bits.int_divisor * CODEC_MCLOCK);
            chip_pll_ctrl.bits.frac_divisor = remainder * 2048 / CODEC_MCLOCK;
        }
        else
        {
            // PLL_OUTPUT_FREQ = 196.608 MHz
            const uint32_t pll_output_freq = 196608000;
            chip_pll_ctrl.bits.int_divisor = pll_output_freq / CODEC_MCLOCK;

            uint64_t remainder =
                pll_output_freq -
                (chip_pll_ctrl.bits.int_divisor * CODEC_MCLOCK);
            chip_pll_ctrl.bits.frac_divisor = remainder * 2048 / CODEC_MCLOCK;
        }
        i2c_write(SGTL5000_CMD::CHIP_PLL_CTRL, chip_pll_ctrl.dw);

        // power up adc
        chip_ana_power.bits.adc_powerup    = 1;
        chip_ana_power.bits.vag_powerup    = 1;
        chip_ana_power.bits.vcoamp_powerup = 1;
        chip_ana_power.bits.pll_powerup    = 1;
        if (device_config.num_audio_channels > 1)
        {
            chip_ana_power.bits.adc_mono = 1;
        }
        i2c_write(SGTL5000_CMD::CHIP_ANA_POWER, chip_ana_power.dw);
    }
    else
    {
        // SGTL is I2S Slave
        // power up adc
        chip_ana_power.bits.adc_powerup = 1;
        chip_ana_power.bits.vag_powerup = 1;
        i2c_write(SGTL5000_CMD::CHIP_ANA_POWER, chip_ana_power.dw);
    }

    uint8_t          biquad_filter_en, num_filters, bandpass_filter;
    biquad_filter_st biquad_filter;

    memset(&biquad_filter, 0, sizeof(biquad_filter));

    // Checking whether to enable the filter or not
    if ((device_config.upper_cutoff_frequency ==
         device_config.sampling_rate / 2) &&
        (device_config.lower_cutoff_frequency == 0))
    {
        biquad_filter_en = 0;
    }
    else
    {
        biquad_filter_en = 1;
    }

    // power up all digital stuff
    chip_dig_power_st chip_dig_power;
    chip_dig_power.dw                   = 0;
    chip_dig_power.bits.adc_powerup     = 1;
    chip_dig_power.bits.i2s_out_powerup = 1;
    chip_dig_power.bits.dap_powerup     = biquad_filter_en;
    i2c_write(SGTL5000_CMD::CHIP_DIG_POWER, chip_dig_power.dw);

    w_hal_system_delay_ms(400);

    chip_clk_ctrl_reg_st clk_ctrl_reg;
    clk_ctrl_reg.dw = 0;
    switch (device_config.sampling_rate)
    {
        case AUDIO_SAMPLE_RATE_8KHz:
        {
            // Rate is 1/4 of the SYS_FS rate
            clk_ctrl_reg.bits.rate_mode = 2;
            // Internal system sample rate is 32 kHz
            clk_ctrl_reg.bits.sys_fs    = 0;
            break;
        }
        case AUDIO_SAMPLE_RATE_16KHz:
        {
            // Rate is 1/2 of the SYS_FS rate
            clk_ctrl_reg.bits.rate_mode = 1;
        }
        // FALL-THROUGH
        case AUDIO_SAMPLE_RATE_32KHz:
        {
            // SYS_MCLK = 384 * SYS_FS
            clk_ctrl_reg.bits.mclk_freq = 1;
            // Internal system sample rate is 32 kHz
            clk_ctrl_reg.bits.sys_fs    = 0;
            break;
        }
        case AUDIO_SAMPLE_RATE_22_05KHz:
        {
            // Rate is 1/2 of the SYS_FS rate
            clk_ctrl_reg.bits.rate_mode = 1;
        }
        // FALL-THROUGH
        case AUDIO_SAMPLE_RATE_44_1KHz:
        {
            // SYS_MCLK is not a standard multiple of Fs. Use PLL
            clk_ctrl_reg.bits.mclk_freq = 3;
            // Internal system sample rate is 44.1 kHz
            clk_ctrl_reg.bits.sys_fs    = 1;
            break;
        }
        case AUDIO_SAMPLE_RATE_24KHz:
        {
            // Rate is 1/2 of the SYS_FS rate
            clk_ctrl_reg.bits.rate_mode = 1;
        }
        // FALL-THROUGH
        case AUDIO_SAMPLE_RATE_48KHz:
        {
            // SYS_MCLK = 256 * SYS_FS
            clk_ctrl_reg.bits.mclk_freq = 0;
            // Internal system sample rate is 48 kHz
            clk_ctrl_reg.bits.sys_fs    = 2;
            break;
        }
        case AUDIO_SAMPLE_RATE_96KHz:
        {
            // SYS_MCLK is not a standard multiple of Fs. Use PLL
            clk_ctrl_reg.bits.mclk_freq = 3;
            // Internal system sample rate is 96 kHz
            clk_ctrl_reg.bits.sys_fs    = 3;
            break;
        }
        default:
        {
            // Unsupported sample rate
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Unsupported sample rate: %d\n",
                              device_config.sampling_rate);
            SYSTEM_ASSERT_TRUE(false);
        }
    }
    i2c_write(SGTL5000_CMD::CHIP_CLK_CTRL, clk_ctrl_reg.dw);

    i2s_ctrl_reg.bits.lr_pol   = 0; // left-first LRCLK
    i2s_ctrl_reg.bits.lr_align = 0; // 1 I2S_SCLK delay alignment
    i2s_ctrl_reg.bits.i2s_mode = 0; // I2S Mode
    i2s_ctrl_reg.bits.sclk_inv = 0; // data is valid on rising edge of I2S_SCLK

    if (I2S_PERIPHERAL_MASTER)
    {
        i2s_ctrl_reg.bits.ms = 1; // I2S_LRCLK and I2S_SCLK are outputs
    }
    else
    {
        i2s_ctrl_reg.bits.ms = 0; // I2S_LRCLK and I2S_SCLK are inputs
    }

    switch (device_config.audio_sampling_width)
    {
        case BITS_PER_AUDIO_SAMPLE_8:
        case BITS_PER_AUDIO_SAMPLE_16:
        {
            // 16-bit data coming from SGTL
            // For 8-bit setting, will convert to 8-bit in FW
            // SCLK=32*Fs
            i2s_ctrl_reg.bits.sclkfreq = 1;
            i2s_ctrl_reg.bits.dlen     = 3;
            break;
        }
        case BITS_PER_AUDIO_SAMPLE_24:
        {
            // 24-bit, SCLK=64*Fs
            i2s_ctrl_reg.bits.sclkfreq = 0;
            i2s_ctrl_reg.bits.dlen     = 1;
            break;
        }
        case BITS_PER_AUDIO_SAMPLE_32:
        {
            // 32-bit, SCLK=64*Fs
            i2s_ctrl_reg.bits.sclkfreq = 0;
            i2s_ctrl_reg.bits.dlen     = 0;
            break;
        }
        default:
        {
            // Unsupported bit per sample
            SYSTEM_ASSERT_TRUE(false);
        }
    }
    i2c_write(SGTL5000_CMD::CHIP_I2S_CTRL, i2s_ctrl_reg.dw);

    chip_sss_ctrl_st chip_sss_ctrl;
    chip_sss_ctrl.dw = 0;
    if (biquad_filter_en)
    {
        chip_sss_ctrl.bits.i2s_select = 3; // DAP -> I2S_DOUT
    }
    else
    {
        chip_sss_ctrl.bits.i2s_select = 0; // ADC -> I2S_DOUT
    }
    chip_sss_ctrl.bits.dap_select = 0; // ADC->DAP
    chip_sss_ctrl.bits.i2s_lrswap = 0; // Left and Right Channels
                                       // for I2S_DOUT not swapped
    i2c_write(SGTL5000_CMD::CHIP_SSS_CTRL, chip_sss_ctrl.dw);

    // Enable or disable Digital Audio Processing
    dap_control_st dap_control;
    dap_control.dw          = 0;
    dap_control.bits.dap_en = biquad_filter_en;
    dap_control.bits.mix_en = 0;
    i2c_write(SGTL5000_CMD::DAP_CONTROL, dap_control.dw);

    // Enable or disable filter
    dap_audio_eq_st dap_audio_eq;
    dap_audio_eq.dw      = 0;
    dap_audio_eq.bits.en = biquad_filter_en;
    i2c_write(SGTL5000_CMD::DAP_AUDIO_EQ, dap_audio_eq.dw);

    // Checking number of filters needed
    if ((device_config.upper_cutoff_frequency <
         device_config.sampling_rate / 2) &&
        (device_config.lower_cutoff_frequency > 0))
    {
        num_filters     = 7;    // Enable 7 filters as 3 for
                                // LPF and HPF, 1 for BPF
        bandpass_filter = true; // Band pass filter is enabled
    }
    else
    {
        num_filters     = 3;
        bandpass_filter = false;
    }

    // enable 7 band PEQ
    dap_peq_st dap_peq;
    dap_peq.dw      = 0;
    dap_peq.bits.en = num_filters; // it enables total number of filters needed
    i2c_write(SGTL5000_CMD::DAP_PEQ, dap_peq.dw);

    if (bandpass_filter)
    {
        filter_type_t filter_type;
        uint32_t      cutoff_freq;
        for (uint8_t filter_index = 0; filter_index < num_filters;
             filter_index++)
        {
            if (filter_index < 3)
            {
                filter_type = filter_type_t::LOW_PASS_FILTER;
                cutoff_freq = device_config.upper_cutoff_frequency;
            }
            else if (filter_index < 6)
            {
                filter_type = filter_type_t::HIGH_PASS_FILTER;
                cutoff_freq = device_config.lower_cutoff_frequency;
            }
            else
            {
                filter_type = filter_type_t::BAND_PASS_FILTER;
                cutoff_freq = (device_config.upper_cutoff_frequency +
                               device_config.lower_cutoff_frequency) /
                              2;
            }

            calc_biquad(filter_type,
                        cutoff_freq,
                        device_config.sampling_rate,
                        biquad_filter);

            set_biquad_filter_coef(biquad_filter);

            // Loading filter values for num_filters
            dap_filter_coef_access_st dap_filter_coef_access;
            dap_filter_coef_access.dw         = 0;
            dap_filter_coef_access.bits.index = filter_index;
            dap_filter_coef_access.bits.wr    = 0x1;
            i2c_write(SGTL5000_CMD::DAP_FILTER_COEF_ACCESS,
                      dap_filter_coef_access.dw);
        }
    }
    else
    {
        if (device_config.upper_cutoff_frequency <
            device_config.sampling_rate / 2)
        {
            // Loading filter values for num_filters
            for (uint8_t filter_index = 0; filter_index < num_filters;
                 filter_index++)
            {
                calc_biquad(filter_type_t::LOW_PASS_FILTER,
                            device_config.upper_cutoff_frequency,
                            device_config.sampling_rate,
                            biquad_filter);

                set_biquad_filter_coef(biquad_filter);

                dap_filter_coef_access_st dap_filter_coef_access;
                dap_filter_coef_access.dw         = 0;
                dap_filter_coef_access.bits.index = filter_index;
                dap_filter_coef_access.bits.wr    = 0x1;
                i2c_write(SGTL5000_CMD::DAP_FILTER_COEF_ACCESS,
                          dap_filter_coef_access.dw);
            }
        }

        if (device_config.lower_cutoff_frequency > 0)
        {
            // Loading filter values for num_filters
            for (uint8_t filter_index = 0; filter_index < num_filters;
                 filter_index++)
            {
                calc_biquad(filter_type_t::HIGH_PASS_FILTER,
                            device_config.lower_cutoff_frequency,
                            device_config.sampling_rate,
                            biquad_filter);

                set_biquad_filter_coef(biquad_filter);

                dap_filter_coef_access_st dap_filter_coef_access;
                dap_filter_coef_access.dw         = 0;
                dap_filter_coef_access.bits.index = filter_index;
                dap_filter_coef_access.bits.wr    = 0x1;
                i2c_write(SGTL5000_CMD::DAP_FILTER_COEF_ACCESS,
                          dap_filter_coef_access.dw);
            }
        }
    }

    // Set ADC_Gain
    uint8_t              ADC_Gain = 0, adc_vol_m6db = 0;
    chip_ana_adc_ctrl_st chip_ana_adc_ctrl;
    chip_ana_adc_ctrl.dw = 0;
    int new_gain_times_ten =
        get_gain_reg_field_values(device_config.mic_gain_times_ten,
                                  ADC_Gain,
                                  adc_vol_m6db);

    if (new_gain_times_ten != device_config.mic_gain_times_ten)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                          "Mic gain 0.1*%d is not supported. "
                          "Setting to 0.1*%d\n",
                          device_config.mic_gain_times_ten,
                          new_gain_times_ten);
    }

    chip_ana_adc_ctrl.bits.adc_vol_m6db = adc_vol_m6db; // ADC volume range
                                                        // reduction by 6.0 Db
    // Separate gain for two channels in stereo mode
    chip_ana_adc_ctrl.bits.adc_vol_left = ADC_Gain;
    if (device_config.num_audio_channels > 1)
    {
        chip_ana_adc_ctrl.bits.adc_vol_right = ADC_Gain;
    }
    i2c_write(SGTL5000_CMD::CHIP_ANA_ADC_CTRL, chip_ana_adc_ctrl.dw);

    // Enable ADC High-Pass Filter, Mute both channels of DAC
    chip_adcdac_ctrl_st chip_adcdac_ctrl;
    chip_adcdac_ctrl.dw                  = 0;
    chip_adcdac_ctrl.bits.adc_hpf_bypass = 0;
    chip_adcdac_ctrl.bits.adc_hpf_freeze = 0;
    chip_adcdac_ctrl.bits.dac_mute_left  = 1;
    chip_adcdac_ctrl.bits.dac_mute_right = 1;
    i2c_write(SGTL5000_CMD::CHIP_ADCDAC_CTRL, chip_adcdac_ctrl.dw);

    // Mute DAC Volume
    chip_dac_vol_st chip_dac_vol;
    chip_dac_vol.dw                 = 0;
    chip_dac_vol.bits.dac_vol_left  = 0xFC;
    chip_dac_vol.bits.dac_vol_right = 0xFC;
    i2c_write(SGTL5000_CMD::CHIP_DAC_VOL, chip_dac_vol.dw);

    chip_pad_strength_st chip_pad_strength;
    chip_pad_strength.dw             = 0;
    chip_pad_strength.bits.ctrl_clk  = 3;
    chip_pad_strength.bits.ctrl_data = 3;
    chip_pad_strength.bits.i2s_dout  = 1;
    chip_pad_strength.bits.i2s_sclk  = 1;
    chip_pad_strength.bits.i2s_lrclk = 1;
    i2c_write(SGTL5000_CMD::CHIP_PAD_STRENGTH, chip_pad_strength.dw);

    // set volume (lowest level)
    chip_ana_hp_ctrl_st chip_ana_hp_ctrl;
    chip_ana_hp_ctrl.dw                = 0;
    chip_ana_hp_ctrl.bits.hp_vol_left  = 0x7f;
    chip_ana_hp_ctrl.bits.hp_vol_right = 0x7f;
    i2c_write(SGTL5000_CMD::CHIP_ANA_HP_CTRL, chip_ana_hp_ctrl.dw);

    // Enable ADC
    chip_ana_ctrl.bits.mute_adc = 0;
    i2c_write(SGTL5000_CMD::CHIP_ANA_CTRL, chip_ana_ctrl.dw);

    error = w_hal_i2c_teardown(SGTL5000_I2C_CHANNEL);

    return error;
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
    sample_length = bits_per_ws = i2s_sample_length;
}

/**
 * @brief Teardown sequence for I2S Peripheral
 *
 * @return wHalErrorCode error code
 */
wHalErrorCode w_hal_i2s_teardown_peripheral()
{
    wHalErrorCode error = w_hal_i2c_init(SGTL5000_I2C_CHANNEL,
                                         whal_i2c_mode::MASTER,
                                         I2C_FREQUENCY);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Disable ADC
    chip_ana_ctrl_st chip_ana_ctrl;
    chip_ana_ctrl.dw            = 0;
    chip_ana_ctrl.bits.mute_adc = 1;
    chip_ana_ctrl.bits.mute_hp  = 1;
    chip_ana_ctrl.bits.mute_lo  = 1;
    i2c_write(SGTL5000_CMD::CHIP_ANA_CTRL, chip_ana_ctrl.dw);

    w_hal_system_delay_ms(10);

    // Disable I2S
    chip_i2s_ctrl_reg_st chip_i2s_ctrl_reg;
    chip_i2s_ctrl_reg.dw = 0;
    i2c_write(SGTL5000_CMD::CHIP_I2S_CTRL, chip_i2s_ctrl_reg.dw);

    w_hal_system_delay_ms(10);

    // Switch MCLK to mode 0
    chip_clk_ctrl_reg_st clk_ctrl_reg;
    clk_ctrl_reg.dw = 0;
    i2c_write(SGTL5000_CMD::CHIP_CLK_CTRL, clk_ctrl_reg.dw);

    w_hal_system_delay_ms(10);

    // Disable output and PLL
    chip_ana_power_st chip_ana_power;
    chip_ana_power.dw = i2c_read(SGTL5000_CMD::CHIP_ANA_POWER);
    chip_ana_power.bits.lineout_powerup   = 0;
    chip_ana_power.bits.headphone_powerup = 0;
    chip_ana_power.bits.vcoamp_powerup    = 0;
    chip_ana_power.bits.pll_powerup       = 0;
    i2c_write(SGTL5000_CMD::CHIP_ANA_POWER, chip_ana_power.dw);

    w_hal_system_delay_ms(10);

    // power down all digital stuff
    chip_dig_power_st chip_dig_power;
    chip_dig_power.dw = 0;
    i2c_write(SGTL5000_CMD::CHIP_DIG_POWER, chip_dig_power.dw);

    w_hal_system_delay_ms(10);

    error = w_hal_i2c_teardown(SGTL5000_I2C_CHANNEL);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

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
    return I2S_PERIPHERAL_MASTER;
}

/**
 * @brief Get the gain reg field values object
 *
 * @param desired_gain_times_ten[in]: Gain value in multiple of ten
 * @param ADC_Gain[out]: Register value to set for specific ADC_Gain value
 * @param adc_vol_m6db[out]: set if desired_gain_times_ten is less than zero
 *
 * @param int new gain times ten if correction needed, otherwise same as input
 */
static int get_gain_reg_field_values(int      desired_gain_times_ten,
                                     uint8_t &ADC_Gain,
                                     uint8_t &adc_vol_m6db)
{
    int min_gain_times_ten;
    int max_gain_times_ten;
    if (desired_gain_times_ten < 0)
    {
        adc_vol_m6db       = 1;
        min_gain_times_ten = M6DB_MIN_GAIN_TIMES_TEN;
        max_gain_times_ten = M6DB_MAX_GAIN_TIMES_TEN;
    }
    else
    {
        adc_vol_m6db       = 0;
        min_gain_times_ten = MIN_GAIN_TIMES_TEN;
        max_gain_times_ten = MAX_GAIN_TIMES_TEN;
    }

    uint32_t new_gain_times_ten = desired_gain_times_ten;
    if (desired_gain_times_ten < min_gain_times_ten)
    {
        new_gain_times_ten = min_gain_times_ten;
        ADC_Gain           = 0x0;
    }
    else if (desired_gain_times_ten > max_gain_times_ten)
    {
        new_gain_times_ten = max_gain_times_ten;
        ADC_Gain           = 0xF;
    }
    else
    {
        int16_t quotient;
        int16_t remainder;
        quotient  = desired_gain_times_ten / DIFFERENCE_BETWEEN_GAINS;
        remainder = desired_gain_times_ten % DIFFERENCE_BETWEEN_GAINS;

        if (remainder == 0)
        {
            ADC_Gain = quotient;
        }
        else
        {
            if ((remainder < -HALF_DIFFERENCE_BETWEEN_GAINS) && (quotient < 0))
            {
                new_gain_times_ten = (quotient - 1) * DIFFERENCE_BETWEEN_GAINS;
                ADC_Gain           = (quotient - 1);
            }
            else if (remainder < HALF_DIFFERENCE_BETWEEN_GAINS)
            {
                new_gain_times_ten = quotient * DIFFERENCE_BETWEEN_GAINS;
                ADC_Gain           = quotient;
            }
            else
            {
                new_gain_times_ten = (quotient + 1) * DIFFERENCE_BETWEEN_GAINS;
                ADC_Gain           = quotient + 1;
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
    uint32_t new_rate;
    if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_16KHz)
    {
        new_rate = AUDIO_SAMPLE_RATE_8KHz;
    }
    else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_22_05KHz)
    {
        new_rate = AUDIO_SAMPLE_RATE_16KHz;
    }
    else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_24KHz)
    {
        new_rate = AUDIO_SAMPLE_RATE_22_05KHz;
    }
    else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_32KHz)
    {
        new_rate = AUDIO_SAMPLE_RATE_24KHz;
    }
    else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_44_1KHz)
    {
        new_rate = AUDIO_SAMPLE_RATE_32KHz;
    }
    else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_48KHz)
    {
        new_rate = AUDIO_SAMPLE_RATE_44_1KHz;
    }
    else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_96KHz)
    {
        new_rate = AUDIO_SAMPLE_RATE_48KHz;
    }
    else
    {
        new_rate = AUDIO_SAMPLE_RATE_96KHz;
    }

    if (new_rate != device_config.sampling_rate)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Sampling rate %d is not supported. "
                              "Setting to %d\n",
                              device_config.sampling_rate,
                              new_rate);

            device_config.sampling_rate = new_rate;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Sampling rate %d is not supported\n",
                              device_config.sampling_rate);

            return wHalErrorCode::I2S_INVALID_SAMPLING_RATE;
        }
    }

    // Can support 1-2 channel
    if (device_config.num_audio_channels == 0)
    {
        if (fix)
        {
            LOG_MSG(TraceFile::W_HAL_I2S_SGTL_CPP,
                    "Number of audio channels cannot be 0. "
                    "Setting to 1\n");

            device_config.num_audio_channels = 1;
        }
        else
        {
            LOG_MSG(TraceFile::W_HAL_I2S_SGTL_CPP,
                    "Number of audio channels cannot be 0\n");

            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }
    else if (device_config.num_audio_channels > 2)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Number of audio channels %d cannot be greater "
                              "than 2. Setting to 2\n",
                              device_config.num_audio_channels);

            device_config.num_audio_channels = 2;
        }
        else
        {

            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Number of audio channels %d cannot be greater "
                              "than 2\n",
                              device_config.num_audio_channels);

            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }

    uint32_t new_width;
    if (device_config.audio_sampling_width < BITS_PER_AUDIO_SAMPLE_24)
    {
        new_width = BITS_PER_AUDIO_SAMPLE_16;
    }
    else if (device_config.audio_sampling_width < BITS_PER_AUDIO_SAMPLE_32)
    {
        new_width = BITS_PER_AUDIO_SAMPLE_24;
    }
    else
    {
        new_width = BITS_PER_AUDIO_SAMPLE_32;
    }

    if (new_width != device_config.audio_sampling_width)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Sampling width %d is not supported. "
                              "Setting to %d\n",
                              device_config.audio_sampling_width,
                              new_width);

            device_config.audio_sampling_width = new_width;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Sampling width %d is not supported.\n",
                              device_config.audio_sampling_width);

            return wHalErrorCode::I2S_INVALID_SAMPLING_WIDTH;
        }
    }

    uint8_t ADC_Gain = 0, adc_vol_m6db = 0;
    int     new_gain_times_ten =
        get_gain_reg_field_values(device_config.mic_gain_times_ten,
                                  ADC_Gain,
                                  adc_vol_m6db);

    if (new_gain_times_ten != device_config.mic_gain_times_ten)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Mic gain 0.1*%d is not supported. "
                              "Setting to 0.1*%d\n",
                              device_config.mic_gain_times_ten,
                              new_gain_times_ten);

            device_config.mic_gain_times_ten = new_gain_times_ten;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Mic gain 0.1*%d is not supported.\n",
                              device_config.mic_gain_times_ten);

            return wHalErrorCode::I2S_INVALID_GAIN;
        }
    }

    return w_hal_i2s_validate_cutoff_frequencies(device_config, fix);
}

#endif // AUDIO_CORE != INVALID_CORE_NUM
