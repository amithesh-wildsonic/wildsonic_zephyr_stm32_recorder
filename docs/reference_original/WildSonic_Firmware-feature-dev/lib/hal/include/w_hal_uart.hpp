#ifndef LIB_HAL_INCLUDE_W_HAL_UART_HPP
#define LIB_HAL_INCLUDE_W_HAL_UART_HPP

#include "w_hal_common.hpp"
#include "w_hal_error_code.hpp"

/**
 * @brief   The list of UART stop bit lengths supported
 */
enum class whal_uart_stop_bits
{
    ONE_BIT,           // 1 clock cycle
    ONE_AND_HALF_BITS, // 1.5 clock cycles
    TWO_BITS,          // 2 clock cycles
};

/**
 * @brief   The list of UART Parity options supported
 */
enum class whal_uart_parity
{
    NONE,   // Parity Disabled
    EVEN_0, // Parity Even, 0 based
    EVEN_1, // Parity Even, 1 based
    ODD_0,  // Parity Odd, 0 based
    ODD_1,  // Parity Odd, 1 based
    MARK,   // 1 Parity
    SPACE,  // 0 Parity
};

using whal_uart_new_character_cb          = void (*)(char c);

constexpr uint32_t DEFAULT_UART_BAUD_RATE = 115200;

/**
 * @brief Initializes a UART interface
 *
 * @param channel UART channel, must be less than max channels
 * @param baud_rate desired baud rate
 * @return wHalErrorCode UART_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode      w_hal_uart_init(uint8_t  channel,
                                   uint32_t baud_rate = DEFAULT_UART_BAUD_RATE);

/**
 * @brief Initialize UART and register a new input character callback function
 * to pass along to app layer
 *
 * @param callback callback function in app layer
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode      w_hal_uart_console_init(whal_uart_new_character_cb callback);

#if __CORE__ == BLE_CORE
/**
 * @brief Initialize UART and register a new input character callback function
 * to pass along to app layer
 *
 * @param callback callback function in app layer
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_uart_ble_init(whal_uart_new_character_cb callback);

/**
 * @brief Blocking write to BLE UART channel
 *
 * @param p_data pointer to data buffer to write
 * @param length length of data buffer
 * @return wHalErrorCode UART_WRITE_FAILED if write fails, NO_ERROR otherwise
 */
wHalErrorCode w_hal_uart_ble_write_blocking(const uint8_t *p_data,
                                            uint32_t       length);

/**
 * @brief Drain the BLE UART RX FIFO by calling the registered callback for
 *        each byte currently in the hardware FIFO.
 *
 * Call this after a long blocking operation performed inside the ISR context
 * (e.g. get_device_info) so that bytes that arrived in the FIFO during the
 * blocking period (e.g. OK+LOST) are processed immediately without waiting
 * for the next ISR invocation.
 */
void          w_hal_uart_ble_drain_fifo();
#endif // __CORE__==BLE_CORE

/**
 * @brief Set the voltage to be used for the UART channel
 *
 * @param channel UART channel, must be less than max channels
 * @param voltage_select selected voltage
 * @return wHalErrorCode error code if failed or invalid input, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_voltage(uint32_t            channel,
                                     whal_voltage_select voltage_select);

/**
 * @brief Check CTS pin value from channel
 *
 * @param channel UART channel, must be less than max channels
 * @param value [out] : observed CTS value
 * @return wHalErrorCode NOT_IMPLEMENTED if channel not supported, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_check_cts(uint8_t channel, int &value);

/**
 * @brief Assert RTS pin value for channel
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode NOT_IMPLEMENTED if channel not supported, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_assert_rts(uint8_t channel);

/**
 * @brief Flushes a UART interface
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode UART_FLUSH_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_uart_flush(uint8_t channel);

/**
 * @brief Set UART baud rate
 *
 * @param channel UART channel, must be less than max channels
 * @param baud_rate desired baud rate
 * @return wHalErrorCode NO_ERROR if success, error code otherwise
 */
wHalErrorCode w_hal_uart_set_baud_rate(uint8_t channel, uint32_t baud_rate);

/**
 * @brief Set bit width for UART data frame
 *
 * @param channel UART channel, must be less than max channels
 * @param data_size bit width for UART data frame
 * @return wHalErrorCode UART_INITIALISATION_FAILED if failure, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_data_bits(uint8_t channel, uint32_t data_size);

/**
 * @brief Set number of stop bits in UART data frame
 *
 * @param channel UART channel, must be less than max channels
 * @param stop_bits number of stop bits in UART data frame
 * @return wHalErrorCode UART_INITIALISATION_FAILED if failure, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_stop_bits(uint8_t             channel,
                                       whal_uart_stop_bits stop_bits);

/**
 * @brief Set parity type for UART data frame
 *
 * @param channel UART channel, must be less than max channels
 * @param parity parity type for UART data frame
 * @return wHalErrorCode UART_INITIALISATION_FAILED if failure, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_parity_bits(uint8_t          channel,
                                         whal_uart_parity parity);

/**
 * @brief Enable/Disable flow control for UART data frame
 *
 * @param channel UART channel, must be less than max channels
 * @param enable whether to enable flow control
 * @return wHalErrorCode UART_INITIALISATION_FAILED if failure, NO_ERROR
 * otherwise
 */
wHalErrorCode w_hal_uart_set_flow_control(uint8_t channel, bool enable);

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
w_hal_uart_write_data(uint8_t channel, const uint8_t *p_data, uint32_t length);

/**
 * @brief Check if current outstanding write is done
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode UART_WRITE_COMPLETE if write is done,
 * UART_WRITE_IN_PROGRESS if write is in progress, UART_NO_ACTIVE_WRITE
 * otherwise
 */
wHalErrorCode w_hal_uart_check_write_status(uint8_t channel);

/**
 * @brief Clear write completion status for channel
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_uart_clear_write_completion_status(uint8_t channel);

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
w_hal_uart_read_data(uint8_t channel, uint8_t *p_data, uint32_t length);

/**
 * @brief Check if current outstanding read is done
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode UART_READ_COMPLETE if read is done,
 * UART_READ_IN_PROGRESS if read in progress, UART_NO_ACTIVE_READ otherwise
 */
wHalErrorCode w_hal_uart_check_read_status(uint8_t channel);

/**
 * @brief Clear read completion status for channel
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_uart_clear_read_completion_status(uint8_t channel);

/**
 * @brief Stop current outstanding read
 *
 * @param channel UART channel, must be less than max channels
 * @return wHalErrorCode UART_READ_COMPLETE if read was already done,
 * UART_READ_STOPPED if read was stopped, UART_NO_ACTIVE_READ otherwise
 */
wHalErrorCode w_hal_uart_stop_read(uint8_t channel);

/**
 * @brief UART teardown after completing all outstanding function will shutdown
 * the UART
 *
 * @param channel UART channel, must be less than max channels
 */
void          w_hal_uart_teardown(uint8_t channel);

#endif // LIB_HAL_INCLUDE_W_HAL_UART_HPP