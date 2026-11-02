// This is the HAL implementation for I2S on the Teensy 3.5/3.6, TBD: create
// submodules for libraries imported

#include "w_hal_i2s.hpp"        // I2S header file
#include "w_hal_error_code.hpp" // HAL error codes

#include "assert.hpp"

//*********** Teensy specific I2S libraries **********************//
#include "filter_biquad.h" //signal filters library
#include "input_i2s.h"     //I2S configuration library
#include "mixer.h"         //Signal amplification library
#include "record_queue.h"  //Audio queue library

//*****************Teensy specific structures ******************//
static AudioInputI2S     i2s1;
static AudioAmplifier    amp1;
static AudioFilterBiquad biquad1;
static AudioRecordQueue  queue1;
static AudioConnection   patchCord1(i2s1, 0, amp1, 0);
static AudioConnection   patchCord2(amp1, biquad1);
static AudioConnection   patchCord3(biquad1, queue1);

//*****************Teensy specific defines ******************//
const uint32_t           AUDIO_QUEUE_SIZE = 200; // 200 slots of 256 KB each
const uint32_t           MIC_GAIN         = 15;  // On a scale of 1-30 ?
const uint32_t           HP_CUTOFF_FREQ   = 200; // Cut off freq for the high
                                                 // pass filter
// Bytes per Queue slot
const uint32_t BYTES_PER_QUEUE_SLOT  = AUDIO_BLOCK_SAMPLES * sizeof(int16_t);
const uint32_t NUMBER_OF_FREQUENCIES = 14;

struct tmclk
{
    uint8_t  mult;
    uint16_t div;
};

/**
 * @brief I2S Buffer information
 */
struct i2s_buffer_info_st
{
    void *   p_buffer; // Buffer pointer
    uint32_t size;     // Size of buffer in bytes
    uint32_t filled;   // Number of bytes filled in buffer
};

static i2s_buffer_info_st i2s_buffer_info;

static const int          samplefreqs[NUMBER_OF_FREQUENCIES] = {8000,
                                                       11025,
                                                       16000,
                                                       22050,
                                                       32000,
                                                       44100,
                                                       (int)44117.64706,
                                                       48000,
                                                       88200,
                                                       (int)44117.64706 * 2,
                                                       96000,
                                                       176400,
                                                       (int)44117.64706 * 4,
                                                       192000};

/**
 * @brief : This function sets the I2S sampling frequency
 *
 */
static wHalErrorCode      w_hal_i2s_set_sample_rate(int freq)
{
    for (int f = 0; f < NUMBER_OF_FREQUENCIES; f++)
    {
        if (freq == samplefreqs[f])
        {
            while (I2S0_MCR & I2S_MCR_DUF)
                ;
            I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) |
                       I2S_MDR_DIVIDE((clkArr[f].div - 1));
            return (wHalErrorCode::NO_ERROR);
        }
    }

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : This function initialises the I2S PLLs - move to system if needed
 *
 */
static wHalErrorCode w_hal_i2s_set_PLLs()
{

#if (F_PLL == 16000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{16, 125},
                                                 {148, 839},
                                                 {32, 125},
                                                 {145, 411},
                                                 {64, 125},
                                                 {151, 214},
                                                 {12, 17},
                                                 {96, 125},
                                                 {151, 107},
                                                 {24, 17},
                                                 {192, 125},
                                                 {127, 45},
                                                 {48, 17},
                                                 {255, 83}};
#elif (F_PLL == 72000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{32, 1125},
                                                 {49, 1250},
                                                 {64, 1125},
                                                 {49, 625},
                                                 {128, 1125},
                                                 {98, 625},
                                                 {8, 51},
                                                 {64, 375},
                                                 {196, 625},
                                                 {16, 51},
                                                 {128, 375},
                                                 {249, 397},
                                                 {32, 51},
                                                 {185, 271}};
#elif (F_PLL == 96000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{8, 375},
                                                 {73, 2483},
                                                 {16, 375},
                                                 {147, 2500},
                                                 {32, 375},
                                                 {147, 1250},
                                                 {2, 17},
                                                 {16, 125},
                                                 {147, 625},
                                                 {4, 17},
                                                 {32, 125},
                                                 {151, 321},
                                                 {8, 17},
                                                 {64, 125}};
#elif (F_PLL == 120000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{32, 1875},
                                                 {89, 3784},
                                                 {64, 1875},
                                                 {147, 3125},
                                                 {128, 1875},
                                                 {205, 2179},
                                                 {8, 85},
                                                 {64, 625},
                                                 {89, 473},
                                                 {16, 85},
                                                 {128, 625},
                                                 {178, 473},
                                                 {32, 85},
                                                 {145, 354}};
#elif (F_PLL == 144000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{16, 1125},
                                                 {49, 2500},
                                                 {32, 1125},
                                                 {49, 1250},
                                                 {64, 1125},
                                                 {49, 625},
                                                 {4, 51},
                                                 {32, 375},
                                                 {98, 625},
                                                 {8, 51},
                                                 {64, 375},
                                                 {196, 625},
                                                 {16, 51},
                                                 {128, 375}};
#elif (F_PLL == 168000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{32, 2625},
                                                 {21, 1250},
                                                 {64, 2625},
                                                 {21, 625},
                                                 {128, 2625},
                                                 {42, 625},
                                                 {8, 119},
                                                 {64, 875},
                                                 {84, 625},
                                                 {16, 119},
                                                 {128, 875},
                                                 {168, 625},
                                                 {32, 119},
                                                 {189, 646}};
#elif (F_PLL == 180000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{46, 4043},
                                                 {49, 3125},
                                                 {73, 3208},
                                                 {98, 3125},
                                                 {183, 4021},
                                                 {196, 3125},
                                                 {16, 255},
                                                 {128, 1875},
                                                 {107, 853},
                                                 {32, 255},
                                                 {219, 1604},
                                                 {214, 853},
                                                 {64, 255},
                                                 {219, 802}};
#elif (F_PLL == 192000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{4, 375},
                                                 {37, 2517},
                                                 {8, 375},
                                                 {73, 2483},
                                                 {16, 375},
                                                 {147, 2500},
                                                 {1, 17},
                                                 {8, 125},
                                                 {147, 1250},
                                                 {2, 17},
                                                 {16, 125},
                                                 {147, 625},
                                                 {4, 17},
                                                 {32, 125}};
#elif (F_PLL == 216000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{32, 3375},
                                                 {49, 3750},
                                                 {64, 3375},
                                                 {49, 1875},
                                                 {128, 3375},
                                                 {98, 1875},
                                                 {8, 153},
                                                 {64, 1125},
                                                 {196, 1875},
                                                 {16, 153},
                                                 {128, 1125},
                                                 {226, 1081},
                                                 {32, 153},
                                                 {147, 646}};
#elif (F_PLL == 240000000)
    const tmclk clkArr[NUMBER_OF_FREQUENCIES] = {{16, 1875},
                                                 {29, 2466},
                                                 {32, 1875},
                                                 {89, 3784},
                                                 {64, 1875},
                                                 {147, 3125},
                                                 {4, 85},
                                                 {32, 625},
                                                 {205, 2179},
                                                 {8, 85},
                                                 {64, 625},
                                                 {89, 473},
                                                 {16, 85},
                                                 {128, 625}};
#endif

    return (wHalErrorCode::NO_ERROR);
}

/**
 * @brief : This function initialises the I2S of the MCU
 *
 * @param device_config device configuration
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_init(DeviceConfig::config_page_st &device_config)
{

    i2s_buffer_info.p_buffer = nullptr;
    i2s_buffer_info.size     = 0;
    i2s_buffer_info.filled   = 0;

    AudioMemory(
        AUDIO_QUEUE_SIZE); // Set the maximum queue size to allocate memory

    amp1.gain(MIC_GAIN); // Set mic gain

    biquad1.setHighpass(
        0,
        HP_CUTOFF_FREQ,
        0.707); // 0.707 is the highest Q factor for selectiveness

    w_hal_i2s_set_PLLs();

    w_hal_i2s_set_sample_rate(freq);

    queue1.begin(); // Initialize queue

    return (wHalErrorCode::NO_ERROR);
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
        i2s_buffer_info.filled   = 0;

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
    if (i2s_buffer_info.p_buffer == nullptr)
    {
        return wHalErrorCode::I2S_NO_BUFFER_AVAILABLE;
    }
    else
    {
        uint32_t filled      = i2s_buffer_info.filled;
        uint32_t buffer_size = i2s_buffer_info.size;

        while (queue1.available() && (filled < buffer_size))
        {
            memcpy(static_cast<uint8_t>(i2s_buffer_info.p_buffer) + filled,
                   queue1.readBuffer(),
                   BYTES_PER_QUEUE_SLOT);

            // Handle alignment
            SYSTEM_ASSERT_LT(filled + AUDIO_BLOCK_SAMPLES, buffer_size);
            filled += BYTES_PER_QUEUE_SLOT;

            queue1.freeBuffer();
        }

        i2s_buffer_info.filled = filled;

        if (i2s_buffer_info.filled == i2s_buffer_info.size)
        {
            p_buffer                 = i2s_buffer_info.p_buffer;
            size                     = i2s_buffer_info.size;

            i2s_buffer_info.p_buffer = nullptr;
            i2s_buffer_info.size     = 0;
            i2s_buffer_info.filled   = 0;

            return wHalErrorCode::NO_ERROR;
        }
        else
        {
            return wHalErrorCode::BUSY;
        }
    }
}

/**
 * @brief : Shutdown the I2S
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_teardown()
{
    queue1.end();
    return (wHalErrorCode::NO_ERROR);
}
