#if AUDIO_CORE != INVALID_CORE_NUM

#include "w_hal_i2s.hpp"        //I2S header file
#include "w_hal_error_code.hpp" //Error code header foe HAL
#include "w_hal_i2s_internal.hpp"

#include "device_config.hpp"
#include "trace.hpp"

#if __CORE__ == AUDIO_CORE

#include "w_hal_rtc.hpp"

#include "FifoQueue.hpp"
#include "assert.hpp"
#include "constants.hpp"

#include "board.h"    //To get the board configurations
#include "i2s.h"      //I2S header for MAX78000
#include "i2s_regs.h" //I2S register header
#include "icc.h"      //I2C header for MAX78000
#include "mxc_device.h"
#include "mxc_sys.h"

#include <climits>
#include <cstring>

#define USE_DMA

// MAX I2S Buffers in HAL
static const uint32_t MAX_I2S_BUFFERS            = 5;
static const uint32_t INVALID_BUFFER_STATE_INDEX = MAX_I2S_BUFFERS;

#ifdef USE_DMA
static const int INVALID_DMA_CHANNEL = -1;
#else
static const uint32_t RESERVE_BUFFER_SIZE_BYTES = 200;
#endif

/**
 * @brief I2S buffer state
 */
enum class i2s_buffer_state_t : uint8_t
{
    FREE,   // Buffer is free
    ACTIVE, // Buffer is queued or is being filled with samples
    FILLED, // Buffer is filled with samples
};

/**
 * @brief I2S Buffer information
 */
struct i2s_buffer_info_st
{
    i2s_buffer_state_t state;    // Buffer state
    void *             p_buffer; // Buffer pointer
    uint32_t           size;     // Size of buffer in bytes
};

#ifndef USE_DMA

// Internal reserve buffer in case no buffer is available
// The purpose of this buffer is to avoid dropping samples.
// When the reserve buffer gets filled past a certain threshold,
// it should flag the application layer to take evasive action
// against losing samples.
static uint8_t  reserve_buffer[RESERVE_BUFFER_SIZE_BYTES];
static uint32_t reserve_buffer_head; // Head of ring buffer.
                                     // Samples are written
                                     // here.
static uint32_t reserve_buffer_tail; // Tail of ring buffer.
                                     // Samples are read
                                     // from here
static bool     reserve_buffer_full; // Flag to indicate if
                                     // reserve buffer is full
#endif

static i2s_buffer_info_st                  i2s_buffer_info[MAX_I2S_BUFFERS];

static FifoQueue<uint8_t, MAX_I2S_BUFFERS> free_index_queue; // Free indices
                                                             // to buffer state
static FifoQueue<uint8_t, MAX_I2S_BUFFERS> active_index_queue; // Active
                                                               // indices to
                                                               // buffer state
static FifoQueue<uint8_t, MAX_I2S_BUFFERS> filled_index_queue; // Finished
                                                               // indices to
                                                               // buffer state

static uint8_t current_buffer_state_index; // working buffer state index
#ifdef USE_DMA
static uint8_t next_buffer_state_index; // next buffer state index

static int     current_dma_channel; // current DMA channel
#else
static uint32_t       current_buffer_offset; // working buffer byte offset
#endif

// Number of bits per audio sample
static uint32_t       bits_per_audio_sample;

// Number of bits to downshift after receiving sample
static uint32_t       num_discard_lower_bits = 0;

// Number of samples to discard due to microphone charging cap effect and also
// this gives time to other modules to start up.
static uint32_t       num_discard_samples    = 0;

static uint32_t       discard_samples;
static uint32_t       dropped_samples;
static uint32_t       dropped_samples_for_print;
static time_t         last_dropped_print_time;

// Size of receiving buffer and the buffer declaration
static const uint32_t I2S_RX_BUFFER_SIZE = 1;
static int32_t        i2s_rx_buffer[I2S_RX_BUFFER_SIZE];

// Struct which stores the configuration of I2S
static mxc_i2s_req_t  i2s_instance;

#ifdef USE_DMA

/**
 * @brief Stop DMA for current buffers
 */
static void stop_current_dma()
{
    if (current_dma_channel != INVALID_DMA_CHANNEL)
    {
        MXC_DMA_Stop(current_dma_channel);

        // Wait for transfer paused
        int status;
        while ((status = MXC_DMA_ChannelGetFlags(current_dma_channel)) &
               MXC_F_DMA_STATUS_IPEND)
        {
            // Wait for pending clear
        }

        // Clear CTZ flag
        MXC_DMA_ChannelClearFlags(current_dma_channel, MXC_F_DMA_STATUS_CTZ_IF);
    }
}

/**
 * @brief Set up reload values for DMA
 */
static void service_dma_reload_buffer()
{
    // If dma is active with only one buffer, chain the next buffer
    if ((current_buffer_state_index != INVALID_BUFFER_STATE_INDEX) &&
        (next_buffer_state_index == INVALID_BUFFER_STATE_INDEX))
    {
        SYSTEM_ASSERT_NOT_EQUAL(current_dma_channel, INVALID_DMA_CHANNEL);

        // Add buffer to DMA list
        bool result = active_index_queue.pop(next_buffer_state_index);

        if (result)
        {
            SYSTEM_ASSERT_NOT_EQUAL(next_buffer_state_index,
                                    INVALID_BUFFER_STATE_INDEX);
            SYSTEM_ASSERT_EQUAL(i2s_buffer_info[next_buffer_state_index].state,
                                i2s_buffer_state_t::ACTIVE);

            const mxc_dma_ch_regs_t *p_dma_ch_regs =
                MXC_DMA_GetCHRegs(current_dma_channel);

            // Pause DMA to set up next buffer in DMA
            stop_current_dma();

            // Check if buffer was completed
            if (p_dma_ch_regs->cnt == 0)
            {
                // Buffer was completed. Add buffer to filled queue
                i2s_buffer_info[current_buffer_state_index].state =
                    i2s_buffer_state_t::FILLED;
                result = filled_index_queue.push(current_buffer_state_index);
                SYSTEM_ASSERT_TRUE(result);

                current_buffer_state_index = INVALID_BUFFER_STATE_INDEX;

                MXC_DMA_ReleaseChannel(current_dma_channel);
                current_dma_channel = INVALID_DMA_CHANNEL;

                // Push buffer back to active queue.
                // There should be no issue pushing to the front of the
                // queue since this is some in the same ISR invocation where
                // the buffer was popped from the queue
                result = active_index_queue.push_front(next_buffer_state_index);
                SYSTEM_ASSERT_TRUE(result);

                next_buffer_state_index = INVALID_BUFFER_STATE_INDEX;
            }
            else
            {
                // Configure reload values
                mxc_dma_srcdst_t dma_srcdst;
                dma_srcdst.ch = current_dma_channel;
                dma_srcdst.dest =
                    i2s_buffer_info[next_buffer_state_index].p_buffer;
                dma_srcdst.source =
                    i2s_buffer_info[next_buffer_state_index].p_buffer;
                dma_srcdst.len = i2s_buffer_info[next_buffer_state_index].size;
                MXC_DMA_SetSrcReload(dma_srcdst);

                // Resume DMA transfer
                MXC_DMA_EnableInt(current_dma_channel);
                MXC_DMA_ChannelEnableInt(current_dma_channel, -1);
                MXC_DMA_Start(current_dma_channel);
            }
        }
    }
}

/**
 * @brief Set up first buffer for DMA
 */
static void service_dma_first_buffer()
{
    // Check if buffer can be added to DMA list
    if (current_buffer_state_index == INVALID_BUFFER_STATE_INDEX)
    {
        SYSTEM_ASSERT_EQUAL(next_buffer_state_index,
                            INVALID_BUFFER_STATE_INDEX);
        SYSTEM_ASSERT_EQUAL(current_dma_channel, INVALID_DMA_CHANNEL);

        bool result = active_index_queue.pop(current_buffer_state_index);

        if (result)
        {
            SYSTEM_ASSERT_EQUAL(
                i2s_buffer_info[current_buffer_state_index].state,
                i2s_buffer_state_t::ACTIVE);
            SYSTEM_ASSERT_NOT_EQUAL(current_buffer_state_index,
                                    INVALID_BUFFER_STATE_INDEX);
            void *p_buffer =
                i2s_buffer_info[current_buffer_state_index].p_buffer;
            int length = i2s_buffer_info[current_buffer_state_index].size;

            current_dma_channel = MXC_I2S_RXDMAConfig(p_buffer, length);

            if (current_dma_channel < 0)
            {
                // current_dma_channel < 0 indicates an error
                int error = current_dma_channel;
                if ((error != E_BUSY) && (error != E_NONE_AVAIL))
                {
                    SYSTEM_ASSERT_GTEQ(error, 0);
                }

                MXC_DMA_ReleaseChannel(current_dma_channel);
                current_dma_channel = INVALID_DMA_CHANNEL;

                // Push buffer back to active queue.
                // There should be no issue pushing to the front of the queue
                // since this is some in the same ISR invocation where the
                // buffer was popped from the queue
                result =
                    active_index_queue.push_front(current_buffer_state_index);
                SYSTEM_ASSERT_TRUE(result);

                current_buffer_state_index = INVALID_BUFFER_STATE_INDEX;
            }
        }
    }
}

/**
 * @brief Service DMA
 */
static void service_dma()
{
    // Check if first buffer can be added to DMA list.
    service_dma_first_buffer();

    // Check if second buffer can be added to DMA list.
    service_dma_reload_buffer();

    // Check if first buffer can be added to DMA list again.
    // The reason for this is that the first buffer may have
    // completed before the second buffer could be added to
    // the DMA list.
    service_dma_first_buffer();
}

/**
 * @brief I2S DMA callback
 *
 * @param dmaCh DMA channel
 * @param error error code of dma operation
 */
static void i2s_dma_callback(int dmaCh, int error)
{
    SYSTEM_ASSERT_NOT_EQUAL(current_buffer_state_index,
                            INVALID_BUFFER_STATE_INDEX);
    SYSTEM_ASSERT_EQUAL(i2s_buffer_info[current_buffer_state_index].state,
                        i2s_buffer_state_t::ACTIVE);

    // Add current buffer to filled queue
    i2s_buffer_info[current_buffer_state_index].state =
        i2s_buffer_state_t::FILLED;
    bool result = filled_index_queue.push(current_buffer_state_index);
    SYSTEM_ASSERT_TRUE(result);

    // Move next buffer to current buffer and set next buffer to invalid
    current_buffer_state_index = next_buffer_state_index;
    next_buffer_state_index    = INVALID_BUFFER_STATE_INDEX;
    if (current_buffer_state_index == INVALID_BUFFER_STATE_INDEX)
    {
        MXC_DMA_ReleaseChannel(current_dma_channel);
        current_dma_channel = INVALID_DMA_CHANNEL;
    }

    // Service DMA
    service_dma();
}

#else // ifndef USE_DMA

static uint32_t       w_hal_i2s_get_audio_sample(void *    p_buffer,
                                                 uint32_t  size,
                                                 uint32_t &samples_read);

/**
 * @brief Returns the largest writeable space in the reserve buffer
 *
 * @return uint32_t the largest writeable space in the reserve buffer
 */
static uint32_t       largest_writeable_reserve_space()
{
    // The space for writing can overwrite the oldest data in the reserve
    // buffer. Hence, this code has no regard for tail pointer.
    return RESERVE_BUFFER_SIZE_BYTES - reserve_buffer_head;
}

/**
 * @brief Returns the unused space in the reserve buffer
 *
 * @return uint32_t the unused space in the reserve buffer in bytes
 */
static uint32_t reserve_buffer_unused_space()
{
    uint32_t unused_space;
    if (reserve_buffer_full)
    {
        unused_space = 0;
    }
    else if (reserve_buffer_head >= reserve_buffer_tail)
    {
        unused_space = RESERVE_BUFFER_SIZE_BYTES -
                       (reserve_buffer_head - reserve_buffer_tail);
    }
    else
    {
        unused_space = reserve_buffer_tail - reserve_buffer_head;
    }

    return unused_space;
}

/**
 * @brief Returns the used space in the reserve buffer
 *
 * @return uint32_t the used space in the reserve buffer in bytes
 */
static uint32_t reserve_buffer_used_space()
{
    return RESERVE_BUFFER_SIZE_BYTES - reserve_buffer_unused_space();
}

/**
 * @brief Increment the head pointer of the reserve buffer
 *
 * @param size number of bytes to increment the head pointer
 * @param dropped_bytes [out] number of bytes dropped due to head pointer
 * incrementing past tail pointer
 */
static void increment_reserve_head(uint32_t size, uint32_t &dropped_bytes)
{
    uint32_t &head         = reserve_buffer_head;

    uint32_t  unused_space = reserve_buffer_unused_space();

    head += size;
    if (head >= RESERVE_BUFFER_SIZE_BYTES)
    {
        head -= RESERVE_BUFFER_SIZE_BYTES;
    }

    // Head can move past tail pointer, overwriting oldest samples.
    // If it does, move tail pointer with head pointer.
    if (unused_space <= size)
    {
        reserve_buffer_tail = head;
        dropped_bytes       = size - unused_space;

        reserve_buffer_full = true;
    }
    else
    {
        dropped_bytes = 0;
    }
}

/**
 * @brief Increment the tail pointer of the reserve buffer
 *
 * @param size number of bytes to increment the tail pointer
 */
static void increment_reserve_tail(uint32_t size)
{
    uint32_t &tail = reserve_buffer_tail;

    tail += size;
    if (tail >= RESERVE_BUFFER_SIZE_BYTES)
    {
        tail -= RESERVE_BUFFER_SIZE_BYTES;
    }

    reserve_buffer_full = false;
}

/**
 * @brief Get buffer to fill with audio samples
 *
 * @param p_buffer [out]: Pointer to the buffer which stores the audio sample
 * @param size [out] : size of the buffer
 */
static void w_hal_i2s_get_buffer_to_fill(uint8_t *&p_buffer, uint32_t &size)
{
    if (current_buffer_state_index != INVALID_BUFFER_STATE_INDEX)
    {
        SYSTEM_ASSERT_EQUAL(i2s_buffer_info[current_buffer_state_index].state,
                            i2s_buffer_state_t::ACTIVE);

        p_buffer = static_cast<uint8_t *>(
            i2s_buffer_info[current_buffer_state_index].p_buffer);
        size = i2s_buffer_info[current_buffer_state_index].size;
        return;
    }

    p_buffer                      = nullptr;
    size                          = bits_per_audio_sample / CHAR_BIT;

    // Check if buffer is available. Otherwise I2S fifo gets overrun and
    // samples are dropped
    uint32_t reserve_buffer_bytes = reserve_buffer_used_space();
    while (current_buffer_state_index == INVALID_BUFFER_STATE_INDEX)
    {
        uint8_t index;
        bool    result = active_index_queue.pop(index);
        if (result)
        {
            SYSTEM_ASSERT_EQUAL(i2s_buffer_info[index].state,
                                i2s_buffer_state_t::ACTIVE);

            current_buffer_state_index = index;
            current_buffer_offset      = 0;
        }
        else
        {
            // No buffer available. Use reserve buffer
            break;
        }

        p_buffer = static_cast<uint8_t *>(
            i2s_buffer_info[current_buffer_state_index].p_buffer);
        size = i2s_buffer_info[current_buffer_state_index].size;

        // Copy any samples from reserve buffer to the current buffer
        if (reserve_buffer_bytes > 0)
        {
            uint32_t copy_bytes =
                std::min(reserve_buffer_bytes, size - current_buffer_offset);

            memcpy(p_buffer + current_buffer_offset,
                   &reserve_buffer[reserve_buffer_tail],
                   copy_bytes);

            // Increment the tail pointer of the reserve buffer
            increment_reserve_tail(copy_bytes);
            reserve_buffer_bytes -= copy_bytes;

            // Increment the current buffer offset
            current_buffer_offset += copy_bytes;
            if (current_buffer_offset == size)
            {
                // Move buffer to buffer filled queue
                i2s_buffer_info[current_buffer_state_index].state =
                    i2s_buffer_state_t::FILLED;
                bool result =
                    filled_index_queue.push(current_buffer_state_index);
                SYSTEM_ASSERT_TRUE(result);

                current_buffer_state_index = INVALID_BUFFER_STATE_INDEX;

                continue;
            }

            // Still some more to copy after tail wrapped around?
            if (reserve_buffer_bytes > 0)
            {
                copy_bytes = std::min(reserve_buffer_bytes,
                                      size - current_buffer_offset);

                memcpy(p_buffer + current_buffer_offset,
                       &reserve_buffer[reserve_buffer_tail],
                       copy_bytes);

                // Increment the tail pointer of the reserve buffer
                increment_reserve_tail(copy_bytes);
                reserve_buffer_bytes -= copy_bytes;

                // Increment the current buffer offset
                current_buffer_offset += copy_bytes;
                if (current_buffer_offset == size)
                {
                    // Move buffer to buffer filled queue
                    i2s_buffer_info[current_buffer_state_index].state =
                        i2s_buffer_state_t::FILLED;
                    bool result =
                        filled_index_queue.push(current_buffer_state_index);
                    SYSTEM_ASSERT_TRUE(result);

                    current_buffer_state_index = INVALID_BUFFER_STATE_INDEX;
                }
            }
        }
    }

    if (current_buffer_state_index == INVALID_BUFFER_STATE_INDEX)
    {
        p_buffer = reserve_buffer + reserve_buffer_head;
        size     = largest_writeable_reserve_space();
    }
}

#endif // USE_DMA

extern "C" void __attribute__((interrupt("machine"))) I2S_IRQHandler(void)
{
    // Get flags and clear them
    uint32_t flags = MXC_I2S->intfl;
    MXC_I2S_ClearFlags(flags);

    NVIC_ClearPendingIRQ(I2S_IRQn);
#ifdef __riscv
    NVIC_ClearPendingEVENT(I2S_IRQn);
#endif

    // Check if there are any samples to discard
    if (flags & MXC_F_I2S_INTFL_RX_OV_CH0)
    {
        // Flush the FIFO
        MXC_I2S->ctrl0ch0 |= MXC_F_I2S_CTRL0CH0_FLUSH;

        const int samples_per_fifo_entry = 4 >> i2s_instance.wordSize;

        // Number of samples dropped is at least the number of samples in the
        // FIFO plus one
        const int samples_dropped =
            ((MXC_I2S->dmach0 >> MXC_F_I2S_DMACH0_RX_LVL_POS) + 1) *
            samples_per_fifo_entry;

        dropped_samples += samples_dropped;
        dropped_samples_for_print += samples_dropped;
    }

#ifndef USE_DMA
    // Check if there are any samples to fill in the current buffer
    if (flags & MXC_F_I2S_INTFL_RX_THD_CH0)
    {
        bool buffer_filled;

        do
        {
            buffer_filled = false;

            uint8_t *p_buffer;
            uint32_t size;

            w_hal_i2s_get_buffer_to_fill(p_buffer, size);
            SYSTEM_ASSERT_LT(current_buffer_offset, size);

            // read samples from buffer
            uint32_t samples_read;
            uint32_t bytes_read =
                w_hal_i2s_get_audio_sample(p_buffer + current_buffer_offset,
                                           size - current_buffer_offset,
                                           samples_read);

            if (current_buffer_state_index != INVALID_BUFFER_STATE_INDEX)
            {
                current_buffer_offset += bytes_read;

                if (current_buffer_offset == size)
                {
                    // Move buffer to buffer filled queue
                    i2s_buffer_info[current_buffer_state_index].state =
                        i2s_buffer_state_t::FILLED;
                    bool result =
                        filled_index_queue.push(current_buffer_state_index);
                    SYSTEM_ASSERT_TRUE(result);

                    current_buffer_state_index = INVALID_BUFFER_STATE_INDEX;
                    current_buffer_offset      = 0;

                    // Repeat again to see if there any more samples to fill in
                    // the next buffer
                    buffer_filled              = true;
                }
            }
            else
            {
                if ((bytes_read > 0) &&
                    ((p_buffer >= reserve_buffer) &&
                     (p_buffer < (reserve_buffer + RESERVE_BUFFER_SIZE_BYTES))))
                {
                    // Increment the head pointer of the reserve buffer
                    uint32_t reserve_dropped_bytes;
                    increment_reserve_head(bytes_read, reserve_dropped_bytes);

                    if (reserve_dropped_bytes > 0)
                    {
                        uint32_t reserve_dropped_samples = 0;

                        switch (bits_per_audio_sample)
                        {
                            case BITS_PER_AUDIO_SAMPLE_8:
                            {
                                reserve_dropped_samples = reserve_dropped_bytes;
                                break;
                            }
                            case BITS_PER_AUDIO_SAMPLE_16:
                            {
                                reserve_dropped_samples =
                                    reserve_dropped_bytes / 2;
                                break;
                            }
                            case BITS_PER_AUDIO_SAMPLE_24:
                            {
                                reserve_dropped_samples =
                                    reserve_dropped_bytes / 3;
                                break;
                            }
                            case BITS_PER_AUDIO_SAMPLE_32:
                            {
                                reserve_dropped_samples =
                                    reserve_dropped_bytes / 4;
                                break;
                            }
                            default:
                            {
                                SYSTEM_ASSERT_TRUE(false);
                                break;
                            }
                        }

                        dropped_samples += reserve_dropped_samples;
                        dropped_samples_for_print += reserve_dropped_samples;
                    }

                    // Repeat again to see if there any more samples to fill in
                    // the next buffer
                    buffer_filled = true;
                }
            }

        } while (buffer_filled); // Repeat again to see if there any more
                                 // samples to fill in the next buffer
    }
#endif // ifndef USE_DMA
}

/**
 * @brief Power up/down I2S peripheral
 *
 * @param on whether to power up or down
 * @return wHalErrorCode error code
 */
wHalErrorCode w_hal_i2s_power_peripheral(bool on)
{
    int power = on ? POWER_ON : POWER_OFF;

    int err;
    // Enable microphone power on Feather board
    if ((err = Microphone_Power(power)) != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SPH_CPP,
                          "Unable to power on microphone. "
                          "Got error 0x%x\n",
                          err);
        return (wHalErrorCode::I2S_INITIALISATION_FAILED);
    }

    w_hal_system_delay_ms(200);

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
    w_hal_i2s_validate_config(device_config, true);

    bits_per_audio_sample = device_config.audio_sampling_width;

    memset(i2s_rx_buffer, 0, sizeof(i2s_rx_buffer));
    free_index_queue.initialize();
    active_index_queue.initialize();
    filled_index_queue.initialize();

    for (uint32_t index = 0; index < MAX_I2S_BUFFERS; index++)
    {
        free_index_queue.push(index);

        i2s_buffer_info[index].p_buffer = nullptr;
        i2s_buffer_info[index].size     = 0;
        i2s_buffer_info[index].state    = i2s_buffer_state_t::FREE;
    }

#ifndef USE_DMA
    // Ring buffer for reserve I2S samples is empty
    reserve_buffer_head = 0;
    reserve_buffer_tail = 0;
#endif

    current_buffer_state_index = INVALID_BUFFER_STATE_INDEX;
#ifdef USE_DMA
    next_buffer_state_index = INVALID_BUFFER_STATE_INDEX;
    current_dma_channel     = INVALID_DMA_CHANNEL;
#else
    current_buffer_offset = 0;
#endif

    discard_samples           = 0;
    dropped_samples           = 0;
    dropped_samples_for_print = 0;
    last_dropped_print_time   = 0;

    i2s_peripheral_input_source_t input_source;
#if defined(BOARD_WS_Rev1) && (BOARD_REVISION >= 5)
    input_source =
        i2s_peripheral_input_source_t::MIC; // only for TLV320AIC: Change with
                                            // respect to the input
                                            // options(LINE1, Line2, Line3)
#else
    input_source          = i2s_peripheral_input_source_t::LINE_IN;
#endif

    wHalErrorCode error =
        w_hal_i2s_init_peripheral(device_config, input_source);

    if (error != wHalErrorCode::NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                          "Unable to initialize the codec/microphone. "
                          "Got error 0x%x\n",
                          static_cast<int>(error));
        return wHalErrorCode::I2S_INITIALISATION_FAILED;
    }

    LOG_MSG(TraceFile::W_HAL_I2S_CPP, "Codec/Microphone enabled!\n");

    // Configure I2S interface parameters
    uint32_t sample_length;
    uint32_t bits_per_ws;
    w_hal_i2s_get_channel_lengths(sample_length, bits_per_ws);
    SYSTEM_ASSERT_LTEQ(sample_length, bits_per_ws);

    SYSTEM_ASSERT_LTEQ(sample_length, 32);
    if (sample_length <= 8)
    {
        i2s_instance.sampleSize = MXC_I2S_SAMPLESIZE_EIGHT;
        num_discard_lower_bits  = 8 - sample_length;
    }
    else if (sample_length <= 16)
    {
        i2s_instance.sampleSize = MXC_I2S_SAMPLESIZE_SIXTEEN;
        num_discard_lower_bits  = 16 - sample_length;
    }
    else if (sample_length <= 20)
    {
        i2s_instance.sampleSize = MXC_I2S_SAMPLESIZE_TWENTY;
        num_discard_lower_bits  = 20 - sample_length;
    }
    else if (sample_length <= 24)
    {
        i2s_instance.sampleSize = MXC_I2S_SAMPLESIZE_TWENTYFOUR;
        num_discard_lower_bits  = 24 - sample_length;
    }
    else
    {
        i2s_instance.sampleSize = MXC_I2S_SAMPLESIZE_THIRTYTWO;
        num_discard_lower_bits  = 32 - sample_length;
    }
    // Discard samples to allow microphone to charge up.
    num_discard_samples =
        2 * device_config.sampling_rate * device_config.num_audio_channels;

    uint32_t bits_per_ws_log2;
    SYSTEM_ASSERT_LTEQ(bits_per_ws, 32);
    switch (bits_per_ws)
    {
        case 8:
        {
            bits_per_ws_log2      = 3;
            i2s_instance.wordSize = MXC_I2S_DATASIZE_BYTE;
            break;
        }
        case 16:
        {
            bits_per_ws_log2      = 4;
            i2s_instance.wordSize = MXC_I2S_DATASIZE_HALFWORD;
            break;
        }
        case 32:
        {
            bits_per_ws_log2      = 5;
            i2s_instance.wordSize = MXC_I2S_DATASIZE_WORD;
            break;
        }
        default:
        {
            // Invalid bits per ws option
            bits_per_ws_log2 = 0;
            SYSTEM_ASSERT_TRUE(false);
        }
    }

    if (w_hal_i2s_is_sck_from_peripheral())
    {
        i2s_instance.clkdiv      = 0;
        i2s_instance.channelMode = MXC_I2S_EXTERNAL_SCK_EXTERNAL_WS;
    }
    else
    {
        // I2S sample rate = 12.288MHz / (2*(clkdiv + 1)) / (2 * bits_per_ws)

        uint32_t clock_dividend;
        switch (device_config.sampling_rate)
        {
            case AUDIO_SAMPLE_RATE_16KHz:
            {
                clock_dividend = CODEC_MCLOCK / AUDIO_SAMPLE_RATE_16KHz;
                break;
            }
            case AUDIO_SAMPLE_RATE_24KHz:
            {
                clock_dividend = CODEC_MCLOCK / AUDIO_SAMPLE_RATE_24KHz;
                break;
            }
            case AUDIO_SAMPLE_RATE_32KHz:
            {
                clock_dividend = CODEC_MCLOCK / AUDIO_SAMPLE_RATE_32KHz;
                break;
            }
            case AUDIO_SAMPLE_RATE_48KHz:
            {
                clock_dividend = CODEC_MCLOCK / AUDIO_SAMPLE_RATE_48KHz;
                break;
            }
            case AUDIO_SAMPLE_RATE_64KHz:
            {
                clock_dividend = CODEC_MCLOCK / AUDIO_SAMPLE_RATE_64KHz;
                break;
            }
            case AUDIO_SAMPLE_RATE_96KHz:
            {
                clock_dividend = CODEC_MCLOCK / AUDIO_SAMPLE_RATE_96KHz;
                break;
            }
            case AUDIO_SAMPLE_RATE_192KHz:
            {
                clock_dividend = CODEC_MCLOCK / AUDIO_SAMPLE_RATE_192KHz;
                break;
            }
            case AUDIO_SAMPLE_RATE_22_05KHz:
            case AUDIO_SAMPLE_RATE_44_1KHz:
            default:
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                                  "Reference clock cannot be divided to "
                                  "desired sample rate: %d\n",
                                  device_config.sampling_rate);
                SYSTEM_ASSERT_TRUE(false);
                break;
            }
        }

        // I2S sample rate = 12.288MHz / (2*(clkdiv + 1)) / (2 * bits_per_ws)
        i2s_instance.clkdiv = ((clock_dividend >> 2) >> bits_per_ws_log2) - 1;
        i2s_instance.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    }

    i2s_instance.justify    = MXC_I2S_LSB_JUSTIFY;
    i2s_instance.wsPolarity = MXC_I2S_POL_NORMAL;
    // Get only left and Right channel data from microphone based on
    // number of channel allocated
    i2s_instance.stereoMode = (device_config.num_audio_channels > 1) ?
                                  MXC_I2S_STEREO :
                                  MXC_I2S_MONO_LEFT_CH;
    i2s_instance.bitOrder   = MXC_I2S_MSB_FIRST;

    i2s_instance.rawData    = NULL;
    i2s_instance.txData     = NULL;
    i2s_instance.rxData     = i2s_rx_buffer;
    i2s_instance.length     = I2S_RX_BUFFER_SIZE;

    int err;
    if ((err = MXC_I2S_Init(&i2s_instance)) != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                          "Error in I2S_Init: %d\n",
                          err);

        return (wHalErrorCode::I2S_INITIALISATION_FAILED);
    }

    // Set I2S RX FIFO threshold to generate interrupt
    MXC_I2S_SetRXThreshold(4);

    // enable I2S interrupt
    NVIC_EnableIRQ(I2S_IRQn);
#ifdef __riscv
    NVIC_EnableEVENT(I2S_IRQn);
#endif

#ifdef USE_DMA
    MXC_I2S_RegisterDMACallback(i2s_dma_callback);

    // Enable RX FIFO Overrun
    MXC_I2S_EnableInt(MXC_F_I2S_INTEN_RX_OV_CH0);
#else
    // Enable RX FIFO Overrun and Threshold Interrupts
    MXC_I2S_EnableInt(MXC_F_I2S_INTEN_RX_OV_CH0 | MXC_F_I2S_INTEN_RX_THD_CH0);
#endif

    // Enable Receive mode
    MXC_I2S_RXEnable();
    __enable_irq();

    LOG_MSG(TraceFile::W_HAL_I2S_CPP, "I2S Init Done\n");

    return (wHalErrorCode::NO_ERROR);
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
    if (dropped_samples > 0)
    {
        return wHalErrorCode::I2S_SAMPLES_DROPPED;
    }

#ifndef USE_DMA
    // Check if more than 1/2 of the reserve buffer is filled
    NVIC_DisableIRQ(I2S_IRQn);

    uint32_t used_bytes = reserve_buffer_used_space();

    NVIC_EnableIRQ(I2S_IRQn);

    if (used_bytes > (RESERVE_BUFFER_SIZE_BYTES / 2))
    {
        return wHalErrorCode::I2S_NEAR_SAMPLE_DROP;
    }
#endif

    // TODO: Handle USE_DMA case

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
    // If buffer queue limit reached, cannot add buffer to list
    if (free_index_queue.isEmpty())
    {
        return wHalErrorCode::BUSY;
    }

    // Add buffer to list
    uint8_t index;
    bool    result = free_index_queue.pop(index);
    SYSTEM_ASSERT_TRUE(result);

    SYSTEM_ASSERT_EQUAL(i2s_buffer_info[index].state, i2s_buffer_state_t::FREE);
    i2s_buffer_info[index].p_buffer = p_buffer;
    i2s_buffer_info[index].size     = size;
    i2s_buffer_info[index].state    = i2s_buffer_state_t::ACTIVE;

    result                          = active_index_queue.push(index);
    SYSTEM_ASSERT_TRUE(result);

    w_hal_i2s_handler();

    return wHalErrorCode::NO_ERROR;
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
    // Log samples that have been dropped. Avoid performing
    // logging in the interrupt handler.
    if (dropped_samples_for_print > 0)
    {
        // To avoid excessive printing, print every
        // 10 seconds whether samples have been dropped.
        time_t current_time = w_hal_rtc_get_time();
        if ((current_time - last_dropped_print_time) > 10)
        {
            // Disable I2S interrupt to capture samples
            // dropped and clear the count
#ifndef USE_DMA
            NVIC_DisableIRQ(I2S_IRQn);
#endif
            uint32_t captured_drops   = dropped_samples_for_print;
            dropped_samples_for_print = 0;
#ifndef USE_DMA
            NVIC_EnableIRQ(I2S_IRQn);
#endif
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Dropped %d samples\n",
                              captured_drops);

            last_dropped_print_time = current_time;
        }
    }

    // Return error code if no completed buffers
    if (filled_index_queue.isEmpty())
    {
        if (free_index_queue.isFull())
        {
            return wHalErrorCode::I2S_NO_BUFFER_AVAILABLE;
        }
        else
        {
            return wHalErrorCode::BUSY;
        }
    }

    uint8_t index;
    bool    result = filled_index_queue.pop(index);
    SYSTEM_ASSERT_TRUE(result);

    SYSTEM_ASSERT_EQUAL(i2s_buffer_info[index].state,
                        i2s_buffer_state_t::FILLED);
    p_buffer                     = i2s_buffer_info[index].p_buffer;
    size                         = i2s_buffer_info[index].size;

    i2s_buffer_info[index].state = i2s_buffer_state_t::FREE;
    result                       = free_index_queue.push(index);
    SYSTEM_ASSERT_TRUE(result);

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Get the number of samples dropped internally by the I2S driver
 *
 * @return uint32_t : number of samples dropped
 */
uint32_t w_hal_i2s_samples_dropped()
{
    uint32_t captured_drops = 0;
    // Log samples that have been dropped. Avoid performing
    // logging in the interrupt handler.
    if (dropped_samples > 0)
    {
// Disable I2S interrupt to capture samples
// dropped and clear the count
#ifndef USE_DMA
        NVIC_DisableIRQ(I2S_IRQn);
#endif
        captured_drops  = dropped_samples;
        dropped_samples = 0;
#ifndef USE_DMA
        NVIC_EnableIRQ(I2S_IRQn);
#endif
    }

    return captured_drops;
}

#ifndef USE_DMA
/**
 * @brief This function stores the audio which is in the queue of the audio
 * device to the buffer and returns the number of audio samples stored into
 * the buffer
 *
 * @param p_buffer : Pointer to the buffer which stores the audio sample
 * @param size : size of the buffer
 * @param samples_read [out] : number of samples stored into the buffer
 *
 * @return uint32_t : returns the number of bytes stored into the buffer
 */
static uint32_t w_hal_i2s_get_audio_sample(void *    p_buffer,
                                           uint32_t  size,
                                           uint32_t &samples_read)

{
    uint16_t num_byte_samples = 0; // Total number of bytes of samples
    uint32_t rx_size;              // Number of available samples in FIFO
    samples_read                          = 0;
    const uint32_t word_size_bytes        = 1 << i2s_instance.wordSize;
    const uint32_t samples_per_fifo_entry = 4 >> i2s_instance.wordSize;
    const bool     all_fifo_bits_usable =
        ((samples_per_fifo_entry * bits_per_audio_sample) ==
         (sizeof(uint32_t) * CHAR_BIT));

    // Check how many number of samples are available
    rx_size = MXC_I2S->dmach0 >> MXC_F_I2S_DMACH0_RX_LVL_POS;

    while ((rx_size) && (size))
    {
        uint32_t fifo_entry = MXC_I2S->fifoch0;

        if (discard_samples < num_discard_samples)
        {
            discard_samples += samples_per_fifo_entry;
        }
        else if (p_buffer == nullptr)
        {
            // If there is no buffer available, the i2s fifo
            // needs to be flushed out, hence dropping samples
            dropped_samples += samples_per_fifo_entry;
            dropped_samples_for_print += samples_per_fifo_entry;
        }
        else
        {
            // Remaining size in buffer should be aligned in size with the
            // samples returned from the fifo
            SYSTEM_ASSERT_GTEQ(size,
                               (bits_per_audio_sample / CHAR_BIT) *
                                   samples_per_fifo_entry);

            // Optimization here to avoid multiple access to memory in a loop if
            // the number of usable bits in the fifo entry matches the max
            // possible bits per fifo entry
            if (all_fifo_bits_usable)
            {
                switch (bits_per_audio_sample)
                {
                    case BITS_PER_AUDIO_SAMPLE_8:
                    case BITS_PER_AUDIO_SAMPLE_16:
                    case BITS_PER_AUDIO_SAMPLE_32:
                    {
                        int32_t *inst_p_buffer =
                            static_cast<int32_t *>(p_buffer);
                        *inst_p_buffer++ = fifo_entry;
                        p_buffer         = inst_p_buffer;
                        break;
                    }
                    default:
                    {
                        SYSTEM_ASSERT_TRUE(false);
                        break;
                    }
                }
            }
            else
            {
                for (uint32_t sample_in_fifo_entry_index = 0;
                     sample_in_fifo_entry_index < samples_per_fifo_entry;
                     sample_in_fifo_entry_index++)
                {
                    int32_t sample =
                        fifo_entry >> (word_size_bytes *
                                       sample_in_fifo_entry_index * CHAR_BIT);

                    sample = sample >> num_discard_lower_bits;

                    switch (bits_per_audio_sample)
                    {
                        case BITS_PER_AUDIO_SAMPLE_8:
                        {
                            int8_t *inst_p_buffer =
                                static_cast<int8_t *>(p_buffer);
                            *inst_p_buffer++ = sample;
                            p_buffer         = inst_p_buffer;
                            break;
                        }
                        case BITS_PER_AUDIO_SAMPLE_16:
                        {
                            int16_t *inst_p_buffer =
                                static_cast<int16_t *>(p_buffer);
                            *inst_p_buffer++ = sample;
                            p_buffer         = inst_p_buffer;
                            break;
                        }
                        case BITS_PER_AUDIO_SAMPLE_32:
                        {
                            int32_t *inst_p_buffer =
                                static_cast<int32_t *>(p_buffer);
                            *inst_p_buffer++ = sample;
                            p_buffer         = inst_p_buffer;
                            break;
                        }
                        default:
                        {
                            SYSTEM_ASSERT_TRUE(false);
                            break;
                        }
                    }
                }
            }

            samples_read += samples_per_fifo_entry;

            const uint32_t bytes_per_fifo_entry =
                (bits_per_audio_sample / CHAR_BIT) * samples_per_fifo_entry;
            size -= bytes_per_fifo_entry;
            num_byte_samples += bytes_per_fifo_entry;
        }

        rx_size--;
    }

    return (num_byte_samples);
}
#endif

/**
 * @brief This function is used to process the I2S data
 *
 * @return wHalErrorCode BUSY if in process, NO_ERROR if otherwise idle
 */
wHalErrorCode w_hal_i2s_handler()
{
    wHalErrorCode error = wHalErrorCode::NO_ERROR;

#ifdef USE_DMA
    // Check if buffer can be added to DMA list
    if ((current_buffer_state_index == INVALID_BUFFER_STATE_INDEX) ||
        (next_buffer_state_index == INVALID_BUFFER_STATE_INDEX))
    {
        // Disable dma interrupt. Since we don't know which dma channel
        // is being used before disabling the interrupt, disable all
        __disable_irq();

        // Since active_index_queue is popped in the ISR, it is possible
        // that the buffer is not yet added to the DMA list. In this case,
        // add the buffer to the DMA list here.
        service_dma();

        // enable dma interrupt
        __enable_irq();

        error = wHalErrorCode::BUSY;
    }
#endif

    return error;
}

/**
 * @brief : Shutdown the I2S
 *
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_teardown()
{
#ifdef USE_DMA
    // Stop DMA transfer
    stop_current_dma();

    if (current_dma_channel != INVALID_DMA_CHANNEL)
    {
        // Release DMA channel
        MXC_DMA_ReleaseChannel(current_dma_channel);
        current_dma_channel = INVALID_DMA_CHANNEL;
    }

    current_buffer_state_index = INVALID_BUFFER_STATE_INDEX;
    next_buffer_state_index    = INVALID_BUFFER_STATE_INDEX;
#endif

    // Disable I2S interrupt
    NVIC_DisableIRQ(I2S_IRQn);
#ifdef __riscv
    NVIC_DisableEVENT(I2S_IRQn);
#endif

    // Disable all I2S Interrupt
    MXC_I2S_DisableInt(0xFFFFFFFF);

    // Shutdown the I2S function
    if (MXC_I2S_Shutdown() != E_NO_ERROR)
    {
        return (wHalErrorCode::I2S_SHUTDOWN_FAILED);
    }

    // Now turn off the peripheral
    wHalErrorCode error = w_hal_i2s_teardown_peripheral();

    return error;
}

#endif // __CORE__ == AUDIO_CORE

/**
 * @brief Correct cutoff frequency to be within the range of sampling rate
 *
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode w_hal_i2s_validate_cutoff_frequencies(
    DeviceConfig::config_page_st &device_config,
    bool                          fix)
{
    uint32_t max_cutoff_frequency =
        (device_config.sampling_rate / 2) + (device_config.sampling_rate % 2);

    if (device_config.upper_cutoff_frequency > max_cutoff_frequency)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Upper cutoff frequency %d is not supported. "
                              "Setting to %d\n",
                              device_config.upper_cutoff_frequency,
                              max_cutoff_frequency);
            device_config.upper_cutoff_frequency = max_cutoff_frequency;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Upper cutoff frequency %d is not supported.\n",
                              device_config.upper_cutoff_frequency);
            return wHalErrorCode::I2S_INVALID_UPPER_CUTOFF_FREQUENCY;
        }
    }

    if (device_config.lower_cutoff_frequency >= max_cutoff_frequency)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Lower cutoff frequency %d is not supported. "
                              "Setting to 0\n",
                              device_config.lower_cutoff_frequency);
            device_config.lower_cutoff_frequency = 0;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Lower cutoff frequency %d is not supported.\n",
                              device_config.lower_cutoff_frequency);
            return wHalErrorCode::I2S_INVALID_LOWER_CUTOFF_FREQUENCY;
        }
    }

    if (device_config.lower_cutoff_frequency >=
        device_config.upper_cutoff_frequency)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Lower cutoff frequency %d is greater than or "
                              "equal to upper cutoff frequency %d. "
                              "Setting to 0 and %d\n",
                              device_config.lower_cutoff_frequency,
                              device_config.upper_cutoff_frequency,
                              max_cutoff_frequency);

            device_config.lower_cutoff_frequency = 0;
            device_config.upper_cutoff_frequency = max_cutoff_frequency;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Lower cutoff frequency %d is greater than or "
                              "equal to upper cutoff frequency %d.\n",
                              device_config.lower_cutoff_frequency,
                              device_config.upper_cutoff_frequency);

            return wHalErrorCode::I2S_INVALID_CUTOFF_FREQUENCY;
        }
    }

    return wHalErrorCode::NO_ERROR;
}

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
    wHalErrorCode error =
        w_hal_i2s_peripheral_validate_config(device_config, fix);

    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    error = w_hal_i2s_validate_cutoff_frequencies(device_config, fix);

    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // If I2S is using internal clock, then validate the frequency
    // selections here
    if (!w_hal_i2s_is_sck_from_peripheral())
    {
        uint32_t new_rate;
        if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_24KHz)
        {
            new_rate = AUDIO_SAMPLE_RATE_16KHz;
        }
        else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_32KHz)
        {
            new_rate = AUDIO_SAMPLE_RATE_24KHz;
        }
        else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_48KHz)
        {
            new_rate = AUDIO_SAMPLE_RATE_32KHz;
        }
        else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_64KHz)
        {
            new_rate = AUDIO_SAMPLE_RATE_48KHz;
        }
        else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_96KHz)
        {
            new_rate = AUDIO_SAMPLE_RATE_64KHz;
        }
        else if (device_config.sampling_rate < AUDIO_SAMPLE_RATE_192KHz)
        {
            new_rate = AUDIO_SAMPLE_RATE_96KHz;
        }
        else
        {
            new_rate = AUDIO_SAMPLE_RATE_192KHz;
        }

        if (new_rate != device_config.sampling_rate)
        {
            if (fix)
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                                  "Sampling rate %d is not supported. "
                                  "Setting to %d\n",
                                  device_config.sampling_rate,
                                  new_rate);

                device_config.sampling_rate = new_rate;
            }
            else
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                                  "Sampling rate %d is not supported.\n",
                                  device_config.sampling_rate);

                return wHalErrorCode::I2S_INVALID_SAMPLING_RATE;
            }
        }
    }

#if defined(BOARD_WS_Rev1) && (BOARD_REVISION >= 5)
    // Only mono audio is supported
    if (device_config.num_audio_channels != 1)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Number of audio channels %d must be 1. "
                              "Setting to 1\n",
                              device_config.num_audio_channels);

            device_config.num_audio_channels = 1;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_SGTL_CPP,
                              "Number of audio channels %d must be 1\n",
                              device_config.num_audio_channels);

            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }
#endif

    if (device_config.num_audio_channels == 0)
    {
        if (fix)
        {
            LOG_MSG(TraceFile::W_HAL_I2S_CPP,
                    "Number of audio channels cannot be 0. Setting number of "
                    "audio channels to 1.\n");

            device_config.num_audio_channels = 1;
        }
        else
        {
            LOG_MSG(TraceFile::W_HAL_I2S_CPP,
                    "Number of audio channels cannot be 0\n");

            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }
    else if (device_config.num_audio_channels > 2)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Number of channels %d is not supported. "
                              "Setting to 2\n",
                              device_config.num_audio_channels);

            device_config.num_audio_channels = 2;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Number of channels %d is not supported.\n",
                              device_config.num_audio_channels);

            return wHalErrorCode::I2S_INVALID_NUM_CHANNELS;
        }
    }

    uint32_t new_width;
    if (device_config.audio_sampling_width < BITS_PER_AUDIO_SAMPLE_32)
    {
        new_width = BITS_PER_AUDIO_SAMPLE_16;
    }
    else
    {
        new_width = BITS_PER_AUDIO_SAMPLE_32;
    }

    if (new_width != device_config.audio_sampling_width)
    {
        if (fix)
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Sampling width %d is not supported. "
                              "Setting to 16\n",
                              device_config.audio_sampling_width);

            device_config.audio_sampling_width = new_width;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Sampling width %d is not supported.\n",
                              device_config.audio_sampling_width);

            return wHalErrorCode::I2S_INVALID_SAMPLING_WIDTH;
        }
    }

#ifdef USE_DMA
    const uint32_t MAX_DATA_BIT_RATE = 6000000;
#else
    // This is a firmware limitation. Getting data through the I2S ISR
    // is not possible at higher data rates.
    const uint32_t MAX_DATA_BIT_RATE = 3000000;
#endif
    uint32_t data_bit_rate = DeviceConfig::get_data_bit_rate(device_config);
    if (data_bit_rate > MAX_DATA_BIT_RATE)
    {
        if (fix)
        {
            do
            {
                // First try lowering the sampling width, then the number of
                // channels, then the sampling rate
                if (device_config.audio_sampling_width >
                    BITS_PER_AUDIO_SAMPLE_16)
                {
                    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                                      "Data bit rate "
                                      "%d cannot be "
                                      "supported at "
                                      "%d bits per sample. "
                                      "Lowering to %d "
                                      "bits per sample\n",
                                      data_bit_rate,
                                      device_config.audio_sampling_width,
                                      BITS_PER_AUDIO_SAMPLE_16);

                    device_config.audio_sampling_width =
                        BITS_PER_AUDIO_SAMPLE_16;

                    data_bit_rate =
                        DeviceConfig::get_data_bit_rate(device_config);

                    if (data_bit_rate < MAX_DATA_BIT_RATE)
                    {
                        break;
                    }
                }

                if (device_config.num_audio_channels > 1)
                {
                    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                                      "Data bit rate "
                                      "%d cannot be "
                                      "supported at "
                                      "%d channels. "
                                      "Lowering to 1 "
                                      "channel\n",
                                      data_bit_rate,
                                      device_config.num_audio_channels);

                    device_config.num_audio_channels = 1;

                    data_bit_rate =
                        DeviceConfig::get_data_bit_rate(device_config);

                    if (data_bit_rate < MAX_DATA_BIT_RATE)
                    {
                        break;
                    }
                }

                if (device_config.sampling_rate > AUDIO_SAMPLE_RATE_16KHz)
                {
                    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                                      "Data bit rate "
                                      "%d cannot be "
                                      "supported at "
                                      "%d Hz. "
                                      "Lowering to "
                                      "%d Hz\n",
                                      data_bit_rate,
                                      device_config.sampling_rate,
                                      AUDIO_SAMPLE_RATE_16KHz);

                    device_config.sampling_rate = AUDIO_SAMPLE_RATE_16KHz;

                    data_bit_rate =
                        DeviceConfig::get_data_bit_rate(device_config);

                    if (data_bit_rate < MAX_DATA_BIT_RATE)
                    {
                        break;
                    }
                }

                return wHalErrorCode::I2S_INVALID_CONFIG;

            } while (false);
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2S_CPP,
                              "Data bit rate %d "
                              "cannot be supported at "
                              "%d bits per sample, %d "
                              "channels, and %d Hz\n",
                              data_bit_rate,
                              device_config.audio_sampling_width,
                              device_config.num_audio_channels,
                              device_config.sampling_rate);

            return wHalErrorCode::I2S_INVALID_CONFIG;
        }
    }

    return wHalErrorCode::NO_ERROR;
}

#endif // AUDIO_CORE != INVALID_CORE_NUM
