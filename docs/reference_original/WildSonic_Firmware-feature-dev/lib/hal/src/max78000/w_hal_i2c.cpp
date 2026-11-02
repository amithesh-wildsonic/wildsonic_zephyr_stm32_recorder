#include "w_hal_i2c.hpp"        //I2C header file
#include "w_hal_error_code.hpp" //Error code header foe HAL
#include "w_hal_system.hpp"

#include "assert.hpp"
#include "trace.hpp"

#include "i2c.h"
#include "mxc.h"
#include "mxc_device.h"

#include <stdio.h>

#define USE_ASYNC

static constexpr uint8_t  I2C1_SHARED_CHANNEL        = 1;
static constexpr uint16_t I2C1_SEMAPHORE_INDEX       = 1;
static constexpr uint32_t I2C1_LOCK_POLL_DELAY_US    = 50;
static constexpr uint32_t I2C1_LOCK_MAX_RETRIES      = 100000;
static constexpr uint32_t I2C1_INIT_MAX_ATTEMPTS     = 3;
static constexpr uint32_t I2C1_INIT_RETRY_DELAY_MS   = 1;
static constexpr uint32_t I2C_TEARDOWN_POLL_DELAY_US = 10;
static constexpr uint32_t I2C_TEARDOWN_MAX_RETRIES   = 10000;

/**
 * @brief Operational state of i2c channel
 */
enum class i2c_state_t : uint8_t
{
    NONE,
    READ_IN_PROGRESS,
    READ_DONE,
    WRITE_IN_PROGRESS,
    WRITE_DONE,
    WRITE_READ_IN_PROGRESS,
};

/**
 * @brief Tracking for I2C channel
 */
struct i2c_channel_tracker_t
{
    mxc_i2c_req_t req;   // request structure to MXC HAL
    i2c_state_t   state; // state of operation
    whal_i2c_mode mode;  // I2C mode
    volatile bool owned; // true if this core currently owns the channel
};

static i2c_channel_tracker_t i2c_channel[MXC_I2C_INSTANCES];

/**
 * @brief Check if an asynchronous transaction is currently active on a channel
 *
 * @param channel I2C channel
 * @return true if an asynchronous read/write transaction is in progress
 */
static bool                  w_hal_i2c_is_transaction_active(uint8_t channel)
{
    i2c_state_t state = i2c_channel[channel].state;
    return (state == i2c_state_t::READ_IN_PROGRESS) ||
           (state == i2c_state_t::WRITE_IN_PROGRESS) ||
           (state == i2c_state_t::WRITE_READ_IN_PROGRESS);
}

/**
 * @brief Acquire cross-core ownership for shared I2C1 channel
 *
 * @param channel I2C channel
 * @return wHalErrorCode NO_ERROR if ownership acquired, BUSY otherwise
 */
static wHalErrorCode w_hal_i2c_acquire_ownership(uint8_t channel)
{
    if (channel != I2C1_SHARED_CHANNEL)
    {
        i2c_channel[channel].owned = true;
        return wHalErrorCode::NO_ERROR;
    }

    if (i2c_channel[channel].owned)
    {
        return wHalErrorCode::BUSY;
    }

    wHalErrorCode sema_error = wHalErrorCode::BUSY;
    for (uint32_t attempt = 0; attempt < I2C1_LOCK_MAX_RETRIES; attempt++)
    {
        sema_error = w_hal_system_get_semaphore(I2C1_SEMAPHORE_INDEX);
        if (sema_error == wHalErrorCode::NO_ERROR)
        {
            break;
        }
        w_hal_system_delay_us(I2C1_LOCK_POLL_DELAY_US);
    }

    if (sema_error != wHalErrorCode::NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2C_CPP,
                          "Failed to acquire semaphore for I2C channel %d\n",
                          channel);
        return wHalErrorCode::BUSY;
    }

    i2c_channel[channel].owned = true;
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Release cross-core ownership for shared I2C1 channel
 *
 * @param channel I2C channel
 */
static void w_hal_i2c_release_ownership(uint8_t channel)
{
    if (channel != I2C1_SHARED_CHANNEL)
    {
        i2c_channel[channel].owned = false;
        return;
    }

    if (i2c_channel[channel].owned)
    {
        i2c_channel[channel].owned = false;
        w_hal_system_free_semaphore(I2C1_SEMAPHORE_INDEX);
    }
}

extern "C" void __attribute__((interrupt("machine"))) I2C0_IRQHandler(void)
{
    MXC_I2C_AsyncHandler(MXC_I2C0);
}

extern "C" void __attribute__((interrupt("machine"))) I2C1_IRQHandler(void)
{
    if (i2c_channel[I2C1_SHARED_CHANNEL].owned)
    {
        MXC_I2C_AsyncHandler(MXC_I2C1);
    }
}

extern "C" void __attribute__((interrupt("machine"))) I2C2_IRQHandler(void)
{
    MXC_I2C_AsyncHandler(MXC_I2C2);
}

/**
 * @brief Initializes a I2C interface
 *
 * @param channel I2C channel, must be less than max channels
 * @param mode I2C mode
 * @param frequency interface frequency
 * @param slave_address address of this device (only applicable if slave
 * mode)
 * @return wHalErrorCode I2C_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_i2c_init(uint8_t       channel,
                             whal_i2c_mode mode,
                             uint32_t      frequency,
                             uint16_t      slave_address)
{
    if (mode == whal_i2c_mode::SLAVE)
    {
        return wHalErrorCode::NOT_IMPLEMENTED;
    }

    wHalErrorCode lock_error = w_hal_i2c_acquire_ownership(channel);
    if (lock_error != wHalErrorCode::NO_ERROR)
    {
        return lock_error;
    }

    mxc_i2c_regs_t *p_i2c_regs = MXC_I2C_GET_I2C(channel);
    const uint32_t  max_attempts =
        (channel == I2C1_SHARED_CHANNEL) ? I2C1_INIT_MAX_ATTEMPTS : 1;

    int error = E_NO_ERROR;
    for (uint32_t attempt = 0; attempt < max_attempts; attempt++)
    {
        error = MXC_I2C_Init(p_i2c_regs,
                             mode == whal_i2c_mode::MASTER,
                             slave_address);
        if (error == E_NO_ERROR)
        {
            break;
        }
        if ((attempt + 1) < max_attempts)
        {
            w_hal_system_delay_ms(I2C1_INIT_RETRY_DELAY_MS);
        }
    }

    if (error != E_NO_ERROR)
    {
        w_hal_i2c_release_ownership(channel);
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2C_CPP,
                          "Failed I2C init on channel %d with error 0x%x\n",
                          channel,
                          error);
        return wHalErrorCode::I2C_INITIALISATION_FAILED;
    }

    switch (channel)
    {
        case 0:
        {
            NVIC_ClearPendingIRQ(I2C0_IRQn);
            NVIC_EnableIRQ(I2C0_IRQn);
            break;
        }
        case 1:
        {
            NVIC_ClearPendingIRQ(I2C1_IRQn);
            NVIC_EnableIRQ(I2C1_IRQn);
            break;
        }
        case 2:
        {
            NVIC_ClearPendingIRQ(I2C2_IRQn);
            NVIC_EnableIRQ(I2C2_IRQn);
            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }

    MXC_I2C_SetFrequency(p_i2c_regs, frequency);
    MXC_I2C_SetTXThreshold(p_i2c_regs, 1);
    MXC_I2C_SetRXThreshold(p_i2c_regs, 1);

    i2c_channel[channel].state = i2c_state_t::NONE;
    i2c_channel[channel].mode  = mode;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Teardown a I2C interface
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode I2C_TEARDOWN_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_i2c_teardown(uint8_t channel)
{
    if ((channel == I2C1_SHARED_CHANNEL) && !i2c_channel[channel].owned)
    {
        return wHalErrorCode::I2C_TEARDOWN_FAILED;
    }

    mxc_i2c_regs_t *p_i2c_regs = MXC_I2C_GET_I2C(channel);

    // Wait for any active asynchronous transaction to finish before shutdown
    for (uint32_t retry = 0; w_hal_i2c_is_transaction_active(channel) &&
                             (retry < I2C_TEARDOWN_MAX_RETRIES);
         retry++)
    {
        w_hal_system_delay_us(I2C_TEARDOWN_POLL_DELAY_US);
    }

    if (w_hal_i2c_is_transaction_active(channel))
    {
        p_i2c_regs->inten0 = 0;
        p_i2c_regs->inten1 = 0;
    }

    // Ensure any pending START/RESTART/STOP or bus activity has completed
    for (uint32_t retry = 0; retry < I2C_TEARDOWN_MAX_RETRIES; retry++)
    {
        const bool mstctrl_busy =
            (p_i2c_regs->mstctrl &
             (MXC_F_I2C_MSTCTRL_START | MXC_F_I2C_MSTCTRL_RESTART |
              MXC_F_I2C_MSTCTRL_STOP)) != 0;
        const bool bus_busy = (p_i2c_regs->status & MXC_F_I2C_STATUS_BUSY) != 0;
        if (!mstctrl_busy && !bus_busy)
        {
            break;
        }
        w_hal_system_delay_us(I2C_TEARDOWN_POLL_DELAY_US);
    }

    switch (channel)
    {
        case 0:
        {
            NVIC_DisableIRQ(I2C0_IRQn);
            NVIC_ClearPendingIRQ(I2C0_IRQn);
            break;
        }
        case 1:
        {
            NVIC_DisableIRQ(I2C1_IRQn);
            NVIC_ClearPendingIRQ(I2C1_IRQn);
            break;
        }
        case 2:
        {
            NVIC_DisableIRQ(I2C2_IRQn);
            NVIC_ClearPendingIRQ(I2C2_IRQn);
            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }

    i2c_channel[channel].state = i2c_state_t::NONE;

    int error                  = MXC_I2C_Shutdown(p_i2c_regs);

    w_hal_i2c_release_ownership(channel);

    if (error != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_I2C_CPP,
                          "Failed I2C teardown on channel %d with error 0x%x\n",
                          channel,
                          error);
        return wHalErrorCode::I2C_TEARDOWN_FAILED;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Call back function for I2C completion
 *
 * @param req I2C request information
 * @param error E_ABORT, E_COMM_ERR, or E_NO_ERROR
 */
static void i2c_callback_function(mxc_i2c_req_t *req, int error)
{
    uint8_t channel = MXC_I2C_GET_IDX(req->i2c);

    if (i2c_channel[channel].state == i2c_state_t::WRITE_IN_PROGRESS)
    {
        i2c_channel[channel].state = i2c_state_t::WRITE_DONE;
    }
    else
    {
        const i2c_state_t valid_states[] = {
            i2c_state_t::READ_IN_PROGRESS,
            i2c_state_t::WRITE_READ_IN_PROGRESS};
        SYSTEM_ASSERT_IN_SET(i2c_channel[channel].state,
                             valid_states,
                             sizeof(valid_states) / sizeof(valid_states[0]));

        i2c_channel[channel].state = i2c_state_t::READ_DONE;
    }
}

/**
 * @brief Transfer data buffer, then receive data from I2C
 *
 * @param channel I2C channel, must be less than max channels
 * @param p_write_data data buffer to write
 * @param write_length length of write data in bytes
 * @param p_read_data data buffer to read into
 * @param read_length length of read data in bytes
 * @param restart restart start condition after transfer
 * @param slave_address address of the the peripheral device (if this is master
 * mode)
 * @param blocking should this function block until operation completes?
 *
 * @return wHalErrorCode I2C_WRITE_FAILED/I2C_READ_FAILED if failed to write or
 * read, BUSY if another operation is occurring,
 * I2C_WRITE_COMPLETE/I2C_READ_COMPLETE if write/read completed,
 * I2C_WRITE_IN_PROGRESS/I2C_READ_IN_PROGRESS otherwise
 */
wHalErrorCode w_hal_i2c_write_read_data(uint8_t        channel,
                                        const uint8_t *p_write_data,
                                        uint32_t       write_length,
                                        uint8_t *      p_read_data,
                                        uint32_t       read_length,
                                        bool           restart,
                                        uint16_t       slave_address,
                                        bool           blocking)
{
    wHalErrorCode fail_error;

    if ((channel == I2C1_SHARED_CHANNEL) && !i2c_channel[channel].owned)
    {
        return (read_length > 0) ? wHalErrorCode::I2C_READ_FAILED :
                                   wHalErrorCode::I2C_WRITE_FAILED;
    }

    if (i2c_channel[channel].state != i2c_state_t::NONE)
    {
        return wHalErrorCode::BUSY;
    }

    mxc_i2c_req_t *p_req = &i2c_channel[channel].req;
    p_req->i2c           = MXC_I2C_GET_I2C(channel);
    p_req->addr          = slave_address;
    p_req->tx_buf        = const_cast<unsigned char *>(p_write_data);
    p_req->tx_len        = write_length;
    p_req->rx_buf        = p_read_data;
    p_req->rx_len        = read_length;
    p_req->restart       = restart;
    p_req->callback      = i2c_callback_function;

    if ((p_req->rx_len > 0) && (p_req->tx_len > 0))
    {
        i2c_channel[channel].state = i2c_state_t::WRITE_READ_IN_PROGRESS;
        fail_error                 = wHalErrorCode::I2C_READ_FAILED;
    }
    else if (p_req->rx_len > 0)
    {
        i2c_channel[channel].state = i2c_state_t::READ_IN_PROGRESS;
        fail_error                 = wHalErrorCode::I2C_READ_FAILED;
    }
    else
    {
        i2c_channel[channel].state = i2c_state_t::WRITE_IN_PROGRESS;
        fail_error                 = wHalErrorCode::I2C_WRITE_FAILED;
    }

    int error;
    if (i2c_channel[channel].mode == whal_i2c_mode::MASTER)
    {
#ifdef USE_ASYNC
        if (!blocking)
        {
            error = MXC_I2C_MasterTransactionAsync(p_req);
        }
        else
#endif
        {
            error = MXC_I2C_MasterTransaction(p_req);

            if (read_length > 0)
            {
                i2c_channel[channel].state = i2c_state_t::READ_DONE;
            }
            else
            {
                i2c_channel[channel].state = i2c_state_t::WRITE_DONE;
            }
        }
    }
    else
    {
        i2c_channel[channel].state = i2c_state_t::NONE;
        return wHalErrorCode::NOT_IMPLEMENTED;
    }

    if (error != 0)
    {
        // TARGETED I2C DIAGNOSTIC: capture intfl0, intfl1, status BEFORE
        // clearing or resetting anything
        const uint32_t intfl0 = p_req->i2c->intfl0;
        const uint32_t intfl1 = p_req->i2c->intfl1;
        const uint32_t status = p_req->i2c->status;
        (void)intfl0;
        (void)intfl1;
        (void)status;

        i2c_channel[channel].state = i2c_state_t::NONE;
        if ((error == E_BUSY) || (error == E_NONE_AVAIL))
        {
            fail_error = wHalErrorCode::BUSY;
        }
        else
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_I2C_CPP,
                "Error transferring: %d [ch%u addr:0x%02X] intfl0:0x%08lX "
                "intfl1:0x%08lX status:0x%08lX%s%s%s%s%s%s%s%s\n",
                error,
                static_cast<unsigned int>(channel),
                static_cast<unsigned int>(slave_address),
                static_cast<unsigned long>(intfl0),
                static_cast<unsigned long>(intfl1),
                static_cast<unsigned long>(status),
                (intfl0 & MXC_F_I2C_INTFL0_ADDR_NACK_ERR) ? " [ADDR_NACK]" : "",
                (intfl0 & MXC_F_I2C_INTFL0_DATA_ERR) ? " [DATA_ERR]" : "",
                (intfl0 & MXC_F_I2C_INTFL0_TO_ERR) ? " [TIMEOUT]" : "",
                (intfl0 & MXC_F_I2C_INTFL0_ARB_ERR) ? " [ARB_ERR]" : "",
                (intfl0 & MXC_F_I2C_INTFL0_START_ERR) ? " [START_ERR]" : "",
                (intfl0 & MXC_F_I2C_INTFL0_STOP_ERR) ? " [STOP_ERR]" : "",
                (intfl0 & MXC_F_I2C_INTFL0_TX_LOCKOUT) ? " [TX_LOCKOUT]" : "",
                (intfl0 & MXC_F_I2C_INTFL0_DNR_ERR) ? " [DNR_ERR]" : "");
        }

        return fail_error;
    }
    else
    {
        return w_hal_i2c_check_status(channel);
    }
}

/**
 * @brief Transfer data from data buffer to I2C
 *
 * @param channel I2C channel, must be less than max channels
 * @param p_data data buffer to write
 * @param length length of data in bytes
 * @param restart restart start condition after transfer
 * @param slave_address address of the the peripheral device (if this is master
 * mode)
 * @param blocking should this function block until operation completes?
 *
 * @return wHalErrorCode I2C_WRITE_FAILED if failed to write, BUSY if another
 * operation is occurring, I2C_WRITE_COMPLETE if write completed,
 * I2C_WRITE_IN_PROGRESS otherwise
 */
wHalErrorCode w_hal_i2c_write_data(uint8_t        channel,
                                   const uint8_t *p_data,
                                   uint32_t       length,
                                   bool           restart,
                                   uint16_t       slave_address,
                                   bool           blocking)
{
    return w_hal_i2c_write_read_data(channel,
                                     p_data,
                                     length,
                                     nullptr,
                                     0,
                                     restart,
                                     slave_address,
                                     blocking);
}

/**
 * @brief Receive data from I2C to data buffer
 *
 * @param channel I2C channel, must be less than max channels
 * @param p_data data buffer to read into
 * @param length length of data in bytes
 * @param restart restart start condition after transfer
 * @param slave_address address of the the peripheral device (if this is master
 * mode)
 * @param blocking should this function block until operation completes?
 *
 * @return wHalErrorCode I2C_READ_FAILED if failed to issue read, BUSY if
 * another operation is occurring, I2C_READ_COMPLETE if read completed,
 * I2C_READ_IN_PROGRESS otherwise
 */
wHalErrorCode w_hal_i2c_read_data(uint8_t  channel,
                                  uint8_t *p_data,
                                  uint32_t length,
                                  bool     restart,
                                  uint16_t slave_address,
                                  bool     blocking)
{
    return w_hal_i2c_write_read_data(channel,
                                     nullptr,
                                     0,
                                     p_data,
                                     length,
                                     restart,
                                     slave_address,
                                     blocking);
}

/**
 * @brief Check if current outstanding operation is done
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode I2C_WRITE_COMPLETE if write is done,
 * I2C_WRITE_IN_PROGRESS if write is in progress, I2C_READ_COMPLETE if read is
 * done, I2C_READ_IN_PROGRESS if read in progress, I2C_NO_ACTIVE_OPERATION
 * otherwise
 */
wHalErrorCode w_hal_i2c_check_status(uint8_t channel)
{
    wHalErrorCode error;

    switch (i2c_channel[channel].state)
    {
        case i2c_state_t::WRITE_DONE:
        {
            error = wHalErrorCode::I2C_WRITE_COMPLETE;
            break;
        }
        case i2c_state_t::WRITE_IN_PROGRESS:
        {
            error = wHalErrorCode::I2C_WRITE_IN_PROGRESS;
            break;
        }
        case i2c_state_t::READ_DONE:
        {
            error = wHalErrorCode::I2C_READ_COMPLETE;
            break;
        }
        case i2c_state_t::WRITE_READ_IN_PROGRESS:
        case i2c_state_t::READ_IN_PROGRESS:
        {
            error = wHalErrorCode::I2C_READ_IN_PROGRESS;
            break;
        }
        default:
        {
            SYSTEM_ASSERT_EQUAL(i2c_channel[channel].state, i2c_state_t::NONE);
            error = wHalErrorCode::I2C_NO_ACTIVE_OPERATION;
            break;
        }
    }

    return error;
}

/**
 * @brief Clear completion status for channel
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_i2c_clear_completion_status(uint8_t channel)
{
    const i2c_state_t valid_states[] = {i2c_state_t::READ_DONE,
                                        i2c_state_t::WRITE_DONE};

    SYSTEM_ASSERT_IN_SET(i2c_channel[channel].state,
                         valid_states,
                         sizeof(valid_states) / sizeof(valid_states[0]));

    i2c_channel[channel].state = i2c_state_t::NONE;

    return wHalErrorCode::NO_ERROR;
}
