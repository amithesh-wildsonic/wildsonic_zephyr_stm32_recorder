#if (__CORE__ == COMM_CORE) || (__CORE__ == CONSOLE_CORE) ||                   \
    (__CORE__ == BLE_CORE)

// This code is included in the COMM_CORE, BLE_CORE and the core that
// CONSOLE UART is used for.

#include "w_hal_uart.hpp"       //UART header file
#include "w_hal_error_code.hpp" //Error code header foe HAL

#include "assert.hpp"

#include "trace.hpp"

#include "mxc.h"
#include "mxc_device.h"
#include "uart.h"

#include <climits>

/**
 * @brief Operational read state of uart channel
 */
enum class uart_read_state_t : uint8_t
{
    NONE,
    READ_IN_PROGRESS,
    READ_DONE,
};

/**
 * @brief Operational write state of uart channel
 */
enum class uart_write_state_t : uint8_t
{
    NONE,
    WRITE_IN_PROGRESS,
    WRITE_DONE,
};

#if TARGET_NUM == 78000
// Use P2_6, P2_7 for RTS/CTS
const uint32_t       UART2_CTS_PIN               = MXC_GPIO_PIN_7;
const uint32_t       UART2_RTS_PIN               = MXC_GPIO_PIN_6;

const mxc_gpio_cfg_t gpio_cfg_uart2_flow_disable = {MXC_GPIO2,
                                                    UART2_CTS_PIN |
                                                        UART2_RTS_PIN,
                                                    MXC_GPIO_FUNC_IN,
                                                    MXC_GPIO_PAD_NONE,
                                                    MXC_GPIO_VSSEL_VDDIO};

const mxc_gpio_cfg_t gpio_cfg_uart2_flow         = gpio_cfg_uart2_flow_disable;

#endif

#define BLE_UART_CHANNEL 0

/**
 * @brief Tracking for UART channel
 */
struct uart_channel_tracker_t
{
    mxc_uart_req_t      read_req;    // read request structure to MXC HAL
    mxc_uart_req_t      write_req;   // write request structure to MXC HAL
    uart_read_state_t   read_state;  // state of read operation
    uart_write_state_t  write_state; // state of write operation
    uint16_t            read_buffer_length;  // read buffer size
    whal_voltage_select voltage_select;      // Voltage selected
    bool                flow_control_enable; // Flow Control enabled?
};

static uart_channel_tracker_t     uart_channel[MXC_UART_INSTANCES];
static whal_uart_new_character_cb uart_console_cb_function = nullptr;
static whal_uart_new_character_cb uart_ble_cb_function     = nullptr;

/**
 * @brief Handler for console input
 */
static void                       console_handler()
{
    mxc_uart_regs_t *p_uart     = MXC_UART_GET_UART(CONSOLE_UART);
    unsigned int     uart_flags = MXC_UART_GetFlags(p_uart);
    int              retval     = 0;

    MXC_UART_ClearFlags(p_uart, uart_flags);

    if (uart_flags & MXC_F_UART_INT_FL_RX_THD)
    {
        // RX FIFO getting full, drain into buffer
        while (true)
        {
            // Read out any available chars
            retval = MXC_UART_ReadCharacterRaw(p_uart);

            if (retval >= 0)
            {

                if (uart_console_cb_function != nullptr)
                {
                    uart_console_cb_function(static_cast<char>(retval));
                }
                else
                {
                    LOG_MSG(TraceFile::W_HAL_UART_CPP, "Null CB Func\n");
                }
            }
            else
            {
                // Otherwise nothing available
                break;
            }
        }
    }
}

/**
 * @brief Handler for BLE input
 */
static void ble_handler()
{
    mxc_uart_regs_t *p_uart     = MXC_UART_GET_UART(BLE_UART_CHANNEL);
    unsigned int     uart_flags = MXC_UART_GetFlags(p_uart);
    int              retval     = 0;

    MXC_UART_ClearFlags(p_uart, uart_flags);

    if (uart_flags & MXC_F_UART_INT_FL_RX_THD)
    {
        // RX FIFO getting full, drain into buffer
        while (true)
        {
            // Read out any available chars
            retval = MXC_UART_ReadCharacterRaw(p_uart);

            if (retval >= 0)
            {
                if (uart_ble_cb_function != nullptr)
                {
                    uart_ble_cb_function(static_cast<char>(retval));
                }
                else
                {
                    LOG_MSG(TraceFile::W_HAL_UART_CPP, "Null CB Func\n");
                }
            }
            else
            {
                // Otherwise nothing available
                break;
            }
        }
    }
}

extern "C" void __attribute__((interrupt("machine"))) UART0_IRQHandler(void)
{
#if BLE_UART_CHANNEL == 0
    ble_handler();
#else
    MXC_UART_AsyncHandler(MXC_UART_GET_UART(0));
#endif
}

extern "C" void __attribute__((interrupt("machine"))) UART1_IRQHandler(void)
{
#if CONSOLE_UART == 1
    console_handler();
#else
    MXC_UART_AsyncHandler(MXC_UART_GET_UART(1));
#endif
}

extern "C" void __attribute__((interrupt("machine"))) UART2_IRQHandler(void)
{
#if CONSOLE_UART == 2
    console_handler();
#else
    MXC_UART_AsyncHandler(MXC_UART_GET_UART(2));
#endif
}

extern "C" void __attribute__((interrupt("machine"))) UART3_IRQHandler(void)
{
#if CONSOLE_UART == 3
    console_handler();
#else
    MXC_UART_AsyncHandler(MXC_UART_GET_UART(3));
#endif
}

/**
 * @brief Initializes a UART interface
 *
 * @param channel UART channel, must be less than max channels
 * @param baud_rate desired baud rate
 * @return wHalErrorCode UART_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_uart_init(uint8_t channel, uint32_t baud_rate)
{
    int error;

    for (uint32_t index = 0; index < MXC_UART_INSTANCES; index++)
    {
        uart_channel[index].read_state  = uart_read_state_t::NONE;
        uart_channel[index].write_state = uart_write_state_t::NONE;
    }
    uart_channel[channel].voltage_select = whal_voltage_select::VOLTAGE_1V8;
    uart_channel[channel].flow_control_enable = false;
#ifdef USE_ASYNC
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(channel));
#ifdef __riscv
    NVIC_ClearPendingEVENT(MXC_UART_GET_IRQ(channel));
#endif
    NVIC_DisableIRQ(MXC_UART_GET_IRQ(channel));
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(channel));
#endif

    mxc_uart_clock_t uart_clock = MXC_UART_APB_CLK;
    if ((IBRO_FREQ % baud_rate) == 0)
    {
        uart_clock = MXC_UART_IBRO_CLK;
    }

    // Initialize the UART
    if ((error = MXC_UART_Init(MXC_UART_GET_UART(channel),
                               baud_rate,
                               uart_clock)) != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                          "Error initializing UART. Got MXC Error %d\n",
                          error);
        return wHalErrorCode::UART_INITIALISATION_FAILED;
    }

    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                      "UART channel %d Initialized\n",
                      channel);

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Initialize UART and register a new input character callback function
 * to pass along to app layer
 *
 * @param callback callback function in app layer
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_uart_console_init(whal_uart_new_character_cb callback)
{
    uart_console_cb_function = callback;

    NVIC_DisableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
#ifdef __riscv
    NVIC_ClearPendingEVENT(MXC_UART_GET_IRQ(CONSOLE_UART));
#endif
    MXC_UART_EnableInt(MXC_UART_GET_UART(CONSOLE_UART),
                       MXC_F_UART_INT_EN_RX_THD);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));

    // No need to call uart initialization function here since it has already
    // been called earlier
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Initialize UART and register a new input character callback function
 * to pass along to app layer
 *
 * @param callback callback function in app layer
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_uart_ble_init(whal_uart_new_character_cb callback)
{
    uart_ble_cb_function = callback;
    wHalErrorCode error;

    // Setup for the  channel
    error = w_hal_uart_init(BLE_UART_CHANNEL, DEFAULT_UART_BAUD_RATE);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    error = w_hal_uart_set_data_bits(BLE_UART_CHANNEL, 8);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    error = w_hal_uart_set_stop_bits(BLE_UART_CHANNEL,
                                     whal_uart_stop_bits::ONE_BIT);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    error =
        w_hal_uart_set_parity_bits(BLE_UART_CHANNEL, whal_uart_parity::NONE);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    error = w_hal_uart_set_flow_control(BLE_UART_CHANNEL, false);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    error = w_hal_uart_set_voltage(BLE_UART_CHANNEL,
                                   whal_voltage_select::VOLTAGE_3V3);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    error = w_hal_uart_flush(BLE_UART_CHANNEL);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    NVIC_DisableIRQ(MXC_UART_GET_IRQ(BLE_UART_CHANNEL));
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(BLE_UART_CHANNEL));
#ifdef __riscv
    NVIC_ClearPendingEVENT(MXC_UART_GET_IRQ(BLE_UART_CHANNEL));
#endif
    MXC_UART_EnableInt(MXC_UART_GET_UART(BLE_UART_CHANNEL),
                       MXC_F_UART_INT_EN_RX_THD);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(BLE_UART_CHANNEL));

    // No need to call uart initialization function here since it has already
    // been called earlier
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Blocking write to BLE UART channel
 *
 * @param p_data pointer to data buffer to write
 * @param length length of data buffer
 * @return wHalErrorCode UART_WRITE_FAILED if write fails, NO_ERROR otherwise
 */
wHalErrorCode w_hal_uart_ble_write_blocking(const uint8_t *p_data,
                                            uint32_t       length)
{
    mxc_uart_regs_t *p_uart = MXC_UART_GET_UART(BLE_UART_CHANNEL);
    for (uint32_t i = 0; i < length; i++)
    {
        uint32_t timeout = 1000000; // Safety timeout
        while ((p_uart->status & MXC_F_UART_STATUS_TX_FULL) && (timeout > 0))
        {
            timeout--;
        }

        if (timeout == 0)
        {
            return wHalErrorCode::UART_WRITE_FAILED;
        }

        p_uart->fifo = p_data[i];
    }
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Drain the BLE UART RX FIFO by calling the registered callback for
 *        each byte currently in the hardware FIFO.
 *
 * During a long blocking operation inside the ISR (e.g. get_device_info),
 * new bytes from HM-10 (like OK+LOST\r\n) accumulate in the hardware RX FIFO
 * but cannot be processed because the ISR is occupied. Calling this function
 * immediately after the blocking operation drains those bytes through the same
 * callback path as the normal ISR, so OK+LOST is handled right away.
 */
void w_hal_uart_ble_drain_fifo()
{
    mxc_uart_regs_t *p_uart = MXC_UART_GET_UART(BLE_UART_CHANNEL);
    int              retval = 0;

    while (true)
    {
        retval = MXC_UART_ReadCharacterRaw(p_uart);
        if (retval >= 0)
        {
            if (uart_ble_cb_function != nullptr)
            {
                uart_ble_cb_function(static_cast<char>(retval));
            }
        }
        else
        {
            break;
        }
    }
}

/**
 * @brief Set the voltage to be used for the UART channel
 *
 * @param channel UART channel, must be less than max channels
 * @param voltage_select selected voltage
 * @return wHalErrorCode error code if failed or invalid input, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_voltage(uint32_t            channel,
                                     whal_voltage_select voltage_select)
{
    int                   error;
    const mxc_gpio_cfg_t *p_mxc_gpio_cfg;
    const mxc_gpio_cfg_t *p_mxc_gpio_cfg_uart_flow         = nullptr;
    const mxc_gpio_cfg_t *p_mxc_gpio_cfg_uart_flow_disable = nullptr;
    mxc_gpio_vssel_t      vssel;

    switch (channel)
    {
        case 0:
            p_mxc_gpio_cfg = &gpio_cfg_uart0;
            if (uart_channel[channel].flow_control_enable)
            {
                p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart0_flow;
            }
            else
            {
                p_mxc_gpio_cfg_uart_flow_disable = &gpio_cfg_uart0_flow_disable;
            }
            break;
        case 1:
            p_mxc_gpio_cfg = &gpio_cfg_uart1;
#if TARGET_NUM == 78000
            // Only Channel 0 supports hardware based flow control
            p_mxc_gpio_cfg_uart_flow_disable = &gpio_cfg_uart1_flow_disable;
#else
            p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart1_flow;
#endif // TARGET_NUM != 78000
            break;
        case 2:
            p_mxc_gpio_cfg = &gpio_cfg_uart2;
#if defined(BOARD_FTHR_REVA)
#if TARGET_NUM == 78000
            // Only Channel 0 supports hardware based flow control
            p_mxc_gpio_cfg_uart_flow_disable = &gpio_cfg_uart2_flow_disable;
#else
            p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart2_flow;
#endif // TARGET_NUM != 78000
#endif // defined(BOARD_FTHR_REVA)
            break;
        default:
            return wHalErrorCode::NOT_IMPLEMENTED;
            break;
    }

    uart_channel[channel].voltage_select = voltage_select;

    switch (voltage_select)
    {
        case whal_voltage_select::VOLTAGE_1V8:
            vssel = MXC_GPIO_VSSEL_VDDIO;
            break;
        case whal_voltage_select::VOLTAGE_3V3:
            vssel = MXC_GPIO_VSSEL_VDDIOH;
            break;
        default:
            return wHalErrorCode::UART_VOLTAGE_NOT_SUPPORTED;
            break;
    }

    if ((error = MXC_GPIO_SetVSSEL(p_mxc_gpio_cfg->port,
                                   vssel,
                                   p_mxc_gpio_cfg->mask)) != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                          "Error initializing UART VSSEL. Got MXC Error %d\n",
                          error);
        return wHalErrorCode::UART_VOLTAGE_SELECT_FAILED;
    }

    if (p_mxc_gpio_cfg_uart_flow != nullptr)
    {
        if ((error = MXC_GPIO_SetVSSEL(p_mxc_gpio_cfg_uart_flow->port,
                                       vssel,
                                       p_mxc_gpio_cfg_uart_flow->mask)) !=
            E_NO_ERROR)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_UART_CPP,
                "Error initializing UART VSSEL. Got MXC Error %d\n",
                error);
            return wHalErrorCode::UART_VOLTAGE_SELECT_FAILED;
        }
    }

    if (p_mxc_gpio_cfg_uart_flow_disable != nullptr)
    {
        if ((error = MXC_GPIO_SetVSSEL(
                 p_mxc_gpio_cfg_uart_flow_disable->port,
                 vssel,
                 p_mxc_gpio_cfg_uart_flow_disable->mask)) != E_NO_ERROR)
        {
            LOG_MSG_WITH_ARGS(
                TraceFile::W_HAL_UART_CPP,
                "Error initializing UART VSSEL. Got MXC Error %d\n",
                error);
            return wHalErrorCode::UART_VOLTAGE_SELECT_FAILED;
        }
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Check CTS pin value from channel
 *
 * @param channel UART channel, must be less than max channels
 * @param value [out] : observed CTS value
 * @return wHalErrorCode NOT_IMPLEMENTED if channel not supported, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_check_cts(uint8_t channel, int &value)
{
    const mxc_gpio_cfg_t *p_mxc_gpio_cfg_uart_flow;

    switch (channel)
    {
        case 0:
            p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart0_flow;
            break;
        case 1:
            p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart1_flow;
            break;
        case 2:
            p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart2_flow;
            break;
        default:
            return wHalErrorCode::NOT_IMPLEMENTED;
            break;
    }

    uint32_t mask;
#ifdef BOARD_FTHR_REVA
    if (channel == 2)
    {
        mask = UART2_CTS_PIN;
    }
    else
#endif
    {
        // Remove highest position bit (RTS) to get bit mask for CTS
        mask = p_mxc_gpio_cfg_uart_flow->mask ^
               (p_mxc_gpio_cfg_uart_flow->mask &
                (p_mxc_gpio_cfg_uart_flow->mask - 1));
    }

    value = (MXC_GPIO_InGet(p_mxc_gpio_cfg_uart_flow->port, mask) != 0);

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Assert RTS pin value for channel
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode NOT_IMPLEMENTED if channel not supported, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_assert_rts(uint8_t channel)
{
    const mxc_gpio_cfg_t *p_mxc_gpio_cfg_uart_flow;

    switch (channel)
    {
        case 0:
            p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart0_flow;
            break;
        case 1:
            p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart1_flow;
            break;
        case 2:
            p_mxc_gpio_cfg_uart_flow = &gpio_cfg_uart2_flow;
            break;
        default:
            return wHalErrorCode::NOT_IMPLEMENTED;
            break;
    }

    uint32_t mask;
#ifdef BOARD_FTHR_REVA
    if (channel == 2)
    {
        mask = UART2_RTS_PIN;
    }
    else
#endif
    {
        // Remove lowest position bit (CTS) to get bit position for RTS
        mask = p_mxc_gpio_cfg_uart_flow->mask &
               (p_mxc_gpio_cfg_uart_flow->mask - 1);
    }

    if (p_mxc_gpio_cfg_uart_flow->func == MXC_GPIO_FUNC_IN)
    {
        mxc_gpio_cfg_t gpio_cfg = *p_mxc_gpio_cfg_uart_flow;
        gpio_cfg.func           = MXC_GPIO_FUNC_OUT;
        gpio_cfg.mask           = mask;

        switch (uart_channel[channel].voltage_select)
        {
            case whal_voltage_select::VOLTAGE_1V8:
                gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIO;
                break;
            case whal_voltage_select::VOLTAGE_3V3:
                gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
                break;
            default:
                return wHalErrorCode::UART_VOLTAGE_NOT_SUPPORTED;
                break;
        }

        MXC_GPIO_Config(&gpio_cfg);
    }

    MXC_GPIO_OutClr(p_mxc_gpio_cfg_uart_flow->port, mask);

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Flushes a UART interface
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode UART_FLUSH_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_uart_flush(uint8_t channel)
{
    mxc_uart_regs_t *p_uart      = MXC_UART_GET_UART(channel);

    const uint32_t   clear_tries = 10;
    for (uint32_t index = 0; index < clear_tries; index++)
    {
        MXC_UART_ClearRXFIFO(p_uart);
        MXC_UART_ClearTXFIFO(p_uart);

        MXC_Delay(MSEC(10));

        if ((MXC_UART_GetRXFIFOAvailable(p_uart) == 0) &&
            (MXC_UART_GetTXFIFOAvailable(p_uart) != 0))
        {
            break;
        }
    }

    if ((MXC_UART_GetRXFIFOAvailable(p_uart) > 0) ||
        (MXC_UART_GetTXFIFOAvailable(p_uart) == 0))
    {
        return (wHalErrorCode::UART_FLUSH_FAILED);
    }
    else
    {
        return wHalErrorCode::NO_ERROR;
    }
}

/**
 * @brief Set UART baud rate
 *
 * @param channel UART channel, must be less than max channels
 * @param baud_rate desired baud rate
 * @return wHalErrorCode NO_ERROR if success, error code otherwise
 */
wHalErrorCode w_hal_uart_set_baud_rate(uint8_t channel, uint32_t baud_rate)
{
    mxc_uart_clock_t uart_clock = MXC_UART_APB_CLK;
    if ((IBRO_FREQ % baud_rate) == 0)
    {
        uart_clock = MXC_UART_IBRO_CLK;
    }

    int err = MXC_UART_SetFrequency(MXC_UART_GET_UART(channel),
                                    baud_rate,
                                    uart_clock);
    if (err < 0)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                          "Failed to set UART Baud Rate. Got MXC Error %d\n",
                          err);
        return wHalErrorCode::UART_INITIALISATION_FAILED;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Set bit width for UART data frame
 *
 * @param channel UART channel, must be less than max channels
 * @param data_size bit width for UART data frame
 * @return wHalErrorCode UART_INITIALISATION_FAILED if failure, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_data_bits(uint8_t channel, uint32_t data_size)
{
    int err = MXC_UART_SetDataSize(MXC_UART_GET_UART(channel), data_size);

    if (err)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                          "Failed to set UART Data Bits. Got MXC Error %d",
                          err);
        return wHalErrorCode::UART_INITIALISATION_FAILED;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Set number of stop bits in UART data frame
 *
 * @param channel UART channel, must be less than max channels
 * @param stop_bits number of stop bits in UART data frame
 * @return wHalErrorCode UART_INITIALISATION_FAILED if failure, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_stop_bits(uint8_t             channel,
                                       whal_uart_stop_bits stop_bits)
{
    mxc_uart_stop_t mxc_stop_bits = mxc_uart_stop_t::MXC_UART_STOP_1;

    switch (stop_bits)
    {
        case whal_uart_stop_bits::ONE_BIT:
            mxc_stop_bits = mxc_uart_stop_t::MXC_UART_STOP_1;
            break;
        case whal_uart_stop_bits::ONE_AND_HALF_BITS:
        case whal_uart_stop_bits::TWO_BITS:
            mxc_stop_bits = mxc_uart_stop_t::MXC_UART_STOP_2;
            break;
        default:
            SYSTEM_ASSERT_TRUE(false);
            break;
    }

    int err = MXC_UART_SetStopBits(MXC_UART_GET_UART(channel), mxc_stop_bits);

    if (err)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                          "Failed to set UART Stop Bits. Got MXC Error %d",
                          err);
        return wHalErrorCode::UART_INITIALISATION_FAILED;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Set parity type for UART data frame
 *
 * @param channel UART channel, must be less than max channels
 * @param parity parity type for UART data frame
 * @return wHalErrorCode UART_INITIALISATION_FAILED if failure, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_parity_bits(uint8_t          channel,
                                         whal_uart_parity parity)
{
    mxc_uart_parity_t mxc_parity = MXC_UART_PARITY_DISABLE;

    switch (parity)
    {
        case whal_uart_parity::NONE:
            mxc_parity = mxc_uart_parity_t::MXC_UART_PARITY_DISABLE;
            break;
        case whal_uart_parity::EVEN_0:
            mxc_parity = mxc_uart_parity_t::MXC_UART_PARITY_EVEN_0;
            break;
        case whal_uart_parity::EVEN_1:
            mxc_parity = mxc_uart_parity_t::MXC_UART_PARITY_EVEN_1;
            break;
        case whal_uart_parity::ODD_0:
            mxc_parity = mxc_uart_parity_t::MXC_UART_PARITY_ODD_0;
            break;
        case whal_uart_parity::ODD_1:
            mxc_parity = mxc_uart_parity_t::MXC_UART_PARITY_ODD_1;
            break;
        default:
            // Unsupported option
            SYSTEM_ASSERT_TRUE(false);
            break;
    }

    int err = MXC_UART_SetParity(MXC_UART_GET_UART(channel), mxc_parity);

    if (err)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                          "Failed to set UART Parity Bits. Got MXC Error %d",
                          err);
        return wHalErrorCode::UART_INITIALISATION_FAILED;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Enable/Disable flow control for UART data frame
 *
 * @param channel UART channel, must be less than max channels
 * @param enable whether to enable flow control
 * @return wHalErrorCode UART_INITIALISATION_FAILED if failure, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_flow_control(uint8_t channel, bool enable)
{
    const int       rtsThreshold = 1;

    mxc_uart_flow_t flowCtrl = enable ? MXC_UART_FLOW_EN : MXC_UART_FLOW_DIS;

#if TARGET_NUM == 78000
    // Only Channel 0 supports hardware based flow control
    if (channel != 0)
    {
        flowCtrl = MXC_UART_FLOW_DIS;
    }
#endif

    uart_channel[channel].flow_control_enable = enable;

    int err = MXC_UART_SetFlowCtrl(MXC_UART_GET_UART(channel),
                                   flowCtrl,
                                   rtsThreshold);

    if (err)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                          "Failed to Set UART Flow Control. Got MXC Error %d",
                          err);
        return wHalErrorCode::UART_INITIALISATION_FAILED;
    }

    return w_hal_uart_set_voltage(channel,
                                  uart_channel[channel].voltage_select);
}

/**
 * @brief Call back function for UART write completion
 *
 * @param req UART request information
 * @param error E_ABORT, E_COMM_ERR, or E_NO_ERROR
 */
static void write_callback_function(mxc_uart_req_t *req, int error)
{
    uint8_t channel = MXC_UART_GET_IDX(req->uart);

    SYSTEM_ASSERT_EQUAL(uart_channel[channel].write_state,
                        uart_write_state_t::WRITE_IN_PROGRESS);
    uart_channel[channel].write_state = uart_write_state_t::WRITE_DONE;
}

/**
 * @brief Transfer data from data buffer to UART
 *
 * @param channel UART channel, must be less than max channels
 * @param p_data data buffer to write
 * @param length length of data buffer in bytes
 * @return wHalErrorCode UART_WRITE_FAILED if failed to write, BUSY if another
 * write is occurring, UART_WRITE_COMPLETE if write completed, NO_ERROR
 * otherwise
 */
wHalErrorCode
w_hal_uart_write_data(uint8_t channel, const uint8_t *p_data, uint32_t length)
{
    if (uart_channel[channel].write_state ==
        uart_write_state_t::WRITE_IN_PROGRESS)
    {
        return wHalErrorCode::BUSY;
    }

    SYSTEM_ASSERT_EQUAL(uart_channel[channel].write_state,
                        uart_write_state_t::NONE);

    mxc_uart_req_t *p_req             = &uart_channel[channel].write_req;
    p_req->uart                       = MXC_UART_GET_UART(channel);
    p_req->txData                     = p_data;
    p_req->txLen                      = length;
    p_req->rxLen                      = 0;
    p_req->callback                   = write_callback_function;

    uart_channel[channel].write_state = uart_write_state_t::WRITE_IN_PROGRESS;

#ifdef USE_DMA
    int error = MXC_UART_TransactionDMA(p_req);
#elif defined(USE_ASYNC)
    int error = MXC_UART_TransactionAsync(p_req);
#else
    int error = MXC_UART_Transaction(p_req);
#endif

    if (error)
    {
        if ((error == E_BUSY) || (error == E_NONE_AVAIL))
        {
            return wHalErrorCode::BUSY;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                              "UART Write Transaction failed with error %d\n",
                              error);
            return wHalErrorCode::UART_WRITE_FAILED;
        }
    }
    else
    {
#if defined(USE_DMA) || defined(USE_ASYNC)
        return wHalErrorCode::UART_WRITE_IN_PROGRESS;
#else
        return wHalErrorCode::UART_WRITE_COMPLETE;
#endif
    }
}

/**
 * @brief Check if current outstanding write is done
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode UART_WRITE_COMPLETE if write is done,
 * UART_WRITE_IN_PROGRESS if write is in progress, UART_NO_ACTIVE_WRITE
 * otherwise
 */
wHalErrorCode w_hal_uart_check_write_status(uint8_t channel)
{
    wHalErrorCode error;

    switch (uart_channel[channel].write_state)
    {
        case uart_write_state_t::WRITE_DONE:
        {
            error = wHalErrorCode::UART_WRITE_COMPLETE;
            break;
        }
        case uart_write_state_t::WRITE_IN_PROGRESS:
        {
            error = wHalErrorCode::UART_WRITE_IN_PROGRESS;
            break;
        }
        default:
        {
            SYSTEM_ASSERT_EQUAL(uart_channel[channel].write_state,
                                uart_write_state_t::NONE);
            error = wHalErrorCode::UART_NO_ACTIVE_WRITE;
            break;
        }
    }

    return error;
}

/**
 * @brief Clear write completion status for channel
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_uart_clear_write_completion_status(uint8_t channel)
{
    SYSTEM_ASSERT_EQUAL(uart_channel[channel].write_state,
                        uart_write_state_t::WRITE_DONE);
    uart_channel[channel].write_state = uart_write_state_t::NONE;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Call back function for UART read completion
 *
 * @param req UART request information
 * @param error E_ABORT, E_COMM_ERR, or E_NO_ERROR
 */
static void read_callback_function(mxc_uart_req_t *p_req, int error)
{
    uint8_t channel = MXC_UART_GET_IDX(p_req->uart);

    SYSTEM_ASSERT_EQUAL(uart_channel[channel].read_state,
                        uart_read_state_t::READ_IN_PROGRESS);
    uart_channel[channel].read_state = uart_read_state_t::READ_DONE;
}

/**
 * @brief Receive data from UART to data buffer
 *
 * @param channel UART channel, must be less than max channels
 * @param p_data data buffer to read into
 * @param length length of data to read in bytes
 * @return wHalErrorCode UART_READ_FAILED if failed to issue read, BUSY if
 * another read is occurring, UART_READ_COMPLETE if read complete, NO_ERROR
 * otherwise
 */
wHalErrorCode
w_hal_uart_read_data(uint8_t channel, uint8_t *p_data, uint32_t length)
{
    if (uart_channel[channel].read_state != uart_read_state_t::NONE)
    {
        return wHalErrorCode::BUSY;
    }

    SYSTEM_ASSERT_EQUAL(uart_channel[channel].read_state,
                        uart_read_state_t::NONE);

    mxc_uart_req_t *p_req            = &uart_channel[channel].read_req;
    p_req->uart                      = MXC_UART_GET_UART(channel);
    p_req->rxData                    = p_data;
    p_req->txLen                     = 0;
    p_req->rxLen                     = length;
    p_req->callback                  = read_callback_function;

    uart_channel[channel].read_state = uart_read_state_t::READ_IN_PROGRESS;
    uart_channel[channel].read_buffer_length = length;

    int error;
#ifdef USE_DMA
    error = MXC_UART_TransactionDMA(p_req);
#else
    error = MXC_UART_TransactionAsync(p_req);
#endif

    if (error)
    {
        if ((error == E_BUSY) || (error == E_NONE_AVAIL))
        {
            return wHalErrorCode::BUSY;
        }
        else
        {
            LOG_MSG_WITH_ARGS(TraceFile::W_HAL_UART_CPP,
                              "UART Read Transaction failed with error %d\n",
                              error);
            return wHalErrorCode::UART_READ_FAILED;
        }
    }
    else
    {
        return wHalErrorCode::UART_READ_IN_PROGRESS;
    }
}

/**
 * @brief Check if current outstanding read is done
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode UART_READ_COMPLETE if read is done,
 * UART_READ_IN_PROGRESS if read in progress, UART_NO_ACTIVE_READ otherwise
 */
wHalErrorCode w_hal_uart_check_read_status(uint8_t channel)
{
    wHalErrorCode error;

    switch (uart_channel[channel].read_state)
    {
        case uart_read_state_t::READ_DONE:
        {
            error = wHalErrorCode::UART_READ_COMPLETE;
            break;
        }
        case uart_read_state_t::READ_IN_PROGRESS:
        {
            error = wHalErrorCode::UART_READ_IN_PROGRESS;
            break;
        }
        default:
        {
            error = wHalErrorCode::UART_NO_ACTIVE_READ;
            break;
        }
    }

    return error;
}

/**
 * @brief Clear read completion status for channel
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_uart_clear_read_completion_status(uint8_t channel)
{
    SYSTEM_ASSERT_EQUAL(uart_channel[channel].read_state,
                        uart_read_state_t::READ_DONE);
    uart_channel[channel].read_state = uart_read_state_t::NONE;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Stop current outstanding read
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode UART_READ_COMPLETE if read was already done,
 * UART_READ_STOPPED if read was stopped, UART_NO_ACTIVE_READ otherwise
 */
wHalErrorCode w_hal_uart_stop_read(uint8_t channel)
{
    wHalErrorCode error_code;

    __disable_irq();
    switch (uart_channel[channel].read_state)
    {
        case uart_read_state_t::READ_DONE:
        {
            uart_channel[channel].read_state = uart_read_state_t::NONE;
            error_code = wHalErrorCode::UART_READ_COMPLETE;
            break;
        }
        case uart_read_state_t::READ_IN_PROGRESS:
        {
            int error;
#ifdef USE_ASYNC
            error = MXC_UART_AbortAsync(MXC_UART_GET_UART(channel));
#else
            error = MXC_UART_AbortRXDMA(MXC_UART_GET_UART(channel));
#endif
            SYSTEM_ASSERT_EQUAL(error, E_NO_ERROR);

            uart_channel[channel].read_state = uart_read_state_t::NONE;
            error_code                       = wHalErrorCode::UART_READ_STOPPED;
            break;
        }
        default:
        {
            error_code = wHalErrorCode::UART_NO_ACTIVE_READ;
            break;
        }
    }
    __enable_irq();

    return error_code;
}

/**
 * @brief UART teardown after completing all outstanding function will shutdown
 * the UART
 *
 * @param channel UART channel, must be less than max channels
 */
void w_hal_uart_teardown(uint8_t channel)
{
    // Flushes out the buffer
    w_hal_uart_flush(channel);

#ifdef USE_ASYNC
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(channel));
#ifdef __riscv
    NVIC_ClearPendingEVENT(MXC_UART_GET_IRQ(channel));
#endif
#endif

    NVIC_DisableIRQ(MXC_UART_GET_IRQ(channel));

    MXC_UART_Shutdown(MXC_UART_GET_UART(channel));
}

#endif // (__CORE__ == COMM_CORE) || (__CORE__ == CONSOLE_CORE)