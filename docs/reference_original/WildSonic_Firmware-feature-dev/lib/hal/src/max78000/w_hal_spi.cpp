#if (__CORE__ == OFF_CHIP_MEMORY_CORE) || (__CORE__ == STORAGE_CORE) ||        \
    (__CORE__ == CORE_0)

#include "w_hal_spi.hpp"

#include "assert.hpp"
#include "trace.hpp"

#include "board.h"
#include "spi.h"

#define USE_ASYNC

/**
 * @brief Operational state of spi channel
 */
enum class spi_state_t : uint8_t
{
    NONE,
    READ_IN_PROGRESS,
    READ_DONE,
    WRITE_IN_PROGRESS,
    WRITE_DONE,
};

/**
 * @brief Tracking for SPI channel
 */
struct spi_channel_tracker_t
{
    mxc_spi_req_t            req;   // request structure to MXC HAL
    spi_state_t              state; // state of operation
    whal_master_slave_select master_slave_select;
};

static spi_channel_tracker_t spi_channel[MXC_SPI_INSTANCES];

/**
 * @brief Initializes a SPI interface
 *
 * @param channel SPI channel, must be less than max channels
 * @param spi_mode interface mode
 * @param master_slave_select indicates whether SOC is master or slave
 * @param voltage_select select voltage on control and data lines
 * @param frequency interface frequency
 * @param num_slaves number of slave select lines
 * @return wHalErrorCode SPI_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode                w_hal_spi_init(uint8_t                  channel,
                                            whal_spi_mode            spi_mode,
                                            whal_master_slave_select master_slave_select,
                                            whal_voltage_select      voltage_select,
                                            uint32_t                 frequency,
                                            uint8_t                  num_slaves)
{
    SYSTEM_ASSERT_LT(channel, MXC_SPI_INSTANCES);
    const uint32_t ssPol = 0;
    mxc_spi_pins_t spi_pins;

    if (master_slave_select == whal_master_slave_select::SLAVE)
    {
        return wHalErrorCode::NOT_IMPLEMENTED;
    }

    spi_pins.clock = true;
    spi_pins.ss0   = false;
    spi_pins.ss1   = false;
    spi_pins.ss2   = false;
    spi_pins.miso  = true; ///< miso pin
    spi_pins.mosi  = true; ///< mosi pin
    if (spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX)
    {
        spi_pins.sdio2 = true; ///< SDIO2 pin
        spi_pins.sdio3 = true; ///< SDIO3 pin
    }
    else
    {
        spi_pins.sdio2 = false; ///< SDIO2 pin
        spi_pins.sdio3 = false; ///< SDIO3 pin
    }

    if (voltage_select == whal_voltage_select::VOLTAGE_3V3)
    {
        spi_pins.vddioh = true;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(voltage_select, whal_voltage_select::VOLTAGE_1V8);
        spi_pins.vddioh = false;
    }

    mxc_spi_regs_t *p_spi_regs = MXC_SPI_GET_SPI(channel);

    int             error =
        MXC_SPI_Init(p_spi_regs,
                     master_slave_select == whal_master_slave_select::MASTER,
                     spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX,
                     num_slaves,
                     ssPol,
                     frequency,
                     spi_pins);

    if (error != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SPI_CPP,
                          "Failed SPI init on channel %d with error 0x%x\n",
                          channel,
                          error);
        return wHalErrorCode::SPI_INITIALISATION_FAILED;
    }

    error = MXC_SPI_SetDataSize(p_spi_regs, 8);

    if (error != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(
            TraceFile::W_HAL_SPI_CPP,
            "Failed to set datasize on channel %d with error 0x%x\n",
            channel,
            error);
        return wHalErrorCode::SPI_INITIALISATION_FAILED;
    }

    wHalErrorCode error_code = w_hal_spi_set_mode(channel, spi_mode);
    if (error_code != wHalErrorCode::NO_ERROR)
    {
        return error_code;
    }

    spi_channel[channel].state               = spi_state_t::NONE;
    spi_channel[channel].master_slave_select = master_slave_select;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Call back function for SPI completion
 *
 * @param req SPI request information
 * @param error E_ABORT, E_COMM_ERR, or E_NO_ERROR
 */
static void spi_callback_function(void *req, int error)
{
    mxc_spi_req_t *p_request = reinterpret_cast<mxc_spi_req_t *>(req);
    uint8_t        channel   = MXC_SPI_GET_IDX(p_request->spi);

    if (spi_channel[channel].state == spi_state_t::WRITE_IN_PROGRESS)
    {
        spi_channel[channel].state = spi_state_t::WRITE_DONE;
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(spi_channel[channel].state,
                            spi_state_t::READ_IN_PROGRESS);

        spi_channel[channel].state = spi_state_t::READ_DONE;
    }
}

/**
 * @brief Switch to different spi width
 *
 * @param channel SPI channel, must be less than max channels
 * @param spi_mode mode to switch to
 * @return wHalErrorCode SPI_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_set_mode(int channel, whal_spi_mode spi_mode)
{
    // Switch to new mode on maxim side
    mxc_spi_width_t spi_width;
    switch (spi_mode)
    {
        case whal_spi_mode::DUAL_HALF_DUPLEX:
        {
            spi_width = mxc_spi_width_t::SPI_WIDTH_DUAL;
            break;
        }
        case whal_spi_mode::QUAD_HALF_DUPLEX:
        {
            spi_width = mxc_spi_width_t::SPI_WIDTH_QUAD;
            break;
        }
        default:
        {
            spi_width = mxc_spi_width_t::SPI_WIDTH_STANDARD;
            break;
        }
    }

    int error = MXC_SPI_SetWidth(MXC_SPI_GET_SPI(channel), spi_width);
    if (error != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(
            TraceFile::W_HAL_SPI_CPP,
            "Failed to set standard width on channel %d with error 0x%x\n",
            channel,
            error);
        return wHalErrorCode::SPI_INITIALISATION_FAILED;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Perform SPI transfer data request
 *
 * @param channel SPI channel, must be less than max channels
 * @param p_data data buffer to read/write
 * @param length length of data in bytes
 * @param read_write true if write, false if read
 * @return wHalErrorCode SPI_WRITE_FAILED/SPI_READ_FAILED if failed to write or
 * read, BUSY if another operation is occurring,
 * SPI_WRITE_COMPLETE/SPI_READ_COMPLETE if write/read completed,
 * SPI_WRITE_IN_PROGRESS/SPI_READ_IN_PROGRESS otherwise
 */
static wHalErrorCode w_hal_spi_transfer_data(uint8_t  channel,
                                             uint8_t *p_data,
                                             uint32_t length,
                                             bool     read_write)
{
    wHalErrorCode fail_error;

    if (spi_channel[channel].state != spi_state_t::NONE)
    {
        return wHalErrorCode::BUSY;
    }

    // RX and TX FIFO should be empty
    mxc_spi_regs_t *p_spi = MXC_SPI_GET_SPI(channel);
    SYSTEM_ASSERT_EQUAL(MXC_SPI_GetRXFIFOAvailable(p_spi), 0);
    SYSTEM_ASSERT_EQUAL(MXC_SPI_GetTXFIFOAvailable(p_spi), MXC_SPI_FIFO_DEPTH);

    mxc_spi_req_t *p_req = &spi_channel[channel].req;
    p_req->spi           = MXC_SPI_GET_SPI(channel);
    p_req->ssIdx         = 1;
    p_req->ssDeassert    = 0;
    if (read_write)
    {
        p_req->txData = p_data;
        p_req->rxData = 0;
        p_req->txLen  = length;
        p_req->rxLen  = 0;
    }
    else
    {
        p_req->txData = 0;
        p_req->rxData = p_data;
        p_req->txLen  = 0;
        p_req->rxLen  = length;
    }
    p_req->txCnt      = 0;
    p_req->rxCnt      = 0;
    p_req->completeCB = spi_callback_function;
    p_req->ssDeassert = 0;

    if (read_write)
    {
        spi_channel[channel].state = spi_state_t::WRITE_IN_PROGRESS;
        fail_error                 = wHalErrorCode::SPI_WRITE_FAILED;
    }
    else
    {
        spi_channel[channel].state = spi_state_t::READ_IN_PROGRESS;
        fail_error                 = wHalErrorCode::SPI_READ_FAILED;
    }

    int error;
    if (spi_channel[channel].master_slave_select ==
        whal_master_slave_select::MASTER)
    {
#ifdef USE_ASYNC
        error = MXC_SPI_MasterTransactionDMA(p_req);
#else
        error = MXC_SPI_MasterTransaction(p_req);

        spi_channel[channel].state =
            read_write ? spi_state_t::WRITE_DONE : spi_state_t::READ_DONE;
#endif
    }
    else
    {
        return wHalErrorCode::NOT_IMPLEMENTED;
    }

    if (error != 0)
    {
        if ((error == E_BUSY) || (error == E_NONE_AVAIL))
        {
            fail_error = wHalErrorCode::BUSY;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SPI_CPP,
                              "Error transferring: %d\n",
                              error);
        }

        return fail_error;
    }
    else
    {
        return w_hal_spi_check_status(channel);
    }
}

/**
 * @brief Transfer data from data buffer to SPI
 *
 * @param channel SPI channel, must be less than max channels
 * @param p_data data buffer to write
 * @param length length of data in bytes
 * @return wHalErrorCode SPI_WRITE_FAILED if failed to write, BUSY if
 * another operation is occurring, SPI_WRITE_COMPLETE if write completed,
 * SPI_WRITE_IN_PROGRESS otherwise
 */
wHalErrorCode
w_hal_spi_write_data(uint8_t channel, const uint8_t *p_data, uint32_t length)
{
    return w_hal_spi_transfer_data(channel,
                                   const_cast<uint8_t *>(p_data),
                                   length,
                                   true);
}

/**
 * @brief Receive data from SPI to data buffer
 *
 * @param channel SPI channel, must be less than max channels
 * @param p_data data buffer to read into
 * @param length length of data in bytes
 * @return wHalErrorCode SPI_READ_FAILED if failed to issue read, BUSY if
 * another operation is occurring, SPI_READ_COMPLETE if read completed,
 * SPI_READ_IN_PROGRESS otherwise
 */
wHalErrorCode
w_hal_spi_read_data(uint8_t channel, uint8_t *p_data, uint32_t length)
{
    return w_hal_spi_transfer_data(channel, p_data, length, false);
}

/**
 * @brief Check if current outstanding operation is done
 *
 * @param channel SPI channel, must be less than max channels
 * @return wHalErrorCode SPI_WRITE_COMPLETE if write is done,
 * SPI_WRITE_IN_PROGRESS if write is in progress, SPI_READ_COMPLETE if read
 * is done, SPI_READ_IN_PROGRESS if read in progress,
 * SPI_NO_ACTIVE_OPERATION otherwise
 */
wHalErrorCode w_hal_spi_check_status(uint8_t channel)
{
    wHalErrorCode error;

    switch (spi_channel[channel].state)
    {
        case spi_state_t::WRITE_DONE:
        {
            if (MXC_SPI_GetTXFIFOAvailable(MXC_SPI_GET_SPI(channel)) ==
                MXC_SPI_FIFO_DEPTH)
            {
                error = wHalErrorCode::SPI_WRITE_COMPLETE;
                break;
            }
        }
        // FALL-THROUGH
        case spi_state_t::WRITE_IN_PROGRESS:
        {
            error = wHalErrorCode::SPI_WRITE_IN_PROGRESS;
            break;
        }
        case spi_state_t::READ_DONE:
        {
            if (MXC_SPI_GetRXFIFOAvailable(MXC_SPI_GET_SPI(channel)) == 0)
            {
                error = wHalErrorCode::SPI_READ_COMPLETE;
                break;
            }
        }
        // FALL-THROUGH
        case spi_state_t::READ_IN_PROGRESS:
        {
            error = wHalErrorCode::SPI_READ_IN_PROGRESS;
            break;
        }
        default:
        {
            SYSTEM_ASSERT_EQUAL(spi_channel[channel].state, spi_state_t::NONE);
            error = wHalErrorCode::SPI_NO_ACTIVE_OPERATION;
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
wHalErrorCode w_hal_spi_clear_completion_status(uint8_t channel)
{
    const spi_state_t valid_states[] = {spi_state_t::READ_DONE,
                                        spi_state_t::WRITE_DONE};

    SYSTEM_ASSERT_IN_SET(spi_channel[channel].state,
                         valid_states,
                         sizeof(valid_states) / sizeof(valid_states[0]));

    spi_channel[channel].state = spi_state_t::NONE;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief SPI Teardown function
 *
 * @return wHalErrorCode SPI_SHUTDOWN_FAILED if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_teardown(uint8_t channel)
{
    int retval;
    retval = MXC_SPI_Shutdown(MXC_SPI_GET_SPI(channel));

    if (retval != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SPI_CPP,
                          "SPI SHUTDOWN ERROR : %d\n",
                          retval);
        return wHalErrorCode::SPI_SHUTDOWN_FAILED;
    }
    return wHalErrorCode::NO_ERROR;
}

#endif // (__CORE__ == OFF_CHIP_MEMORY_CORE) || (__CORE__ == STORAGE_CORE) ||
       // (__CORE__ == CORE_0)
