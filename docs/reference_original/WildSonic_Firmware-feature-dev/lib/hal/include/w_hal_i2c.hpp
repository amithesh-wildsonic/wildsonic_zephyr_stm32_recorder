#ifndef LIB_HAL_INCLUDE_W_HAL_I2C_HPP
#define LIB_HAL_INCLUDE_W_HAL_I2C_HPP

#include "w_hal_error_code.hpp"

#include "assert.hpp"

#include <climits>

/**
 * @brief The list of I2C Parity options supported
 */
enum class whal_i2c_mode
{
    MASTER,
    SLAVE,
};

/**
 * @brief Initializes a I2C interface
 *
 * @param channel I2C channel, must be less than max channels
 * @param mode I2C mode
 * @param frequency interface frequency
 * @param slave_address address of this device (only applicable if slave mode)
 * @return wHalErrorCode I2C_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_i2c_init(uint8_t       channel,
                             whal_i2c_mode mode,
                             uint32_t      frequency,
                             uint16_t      slave_address = 0);

/**
 * @brief Teardown a I2C interface
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode I2C_TEARDOWN_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_i2c_teardown(uint8_t channel);

/**
 * @brief Transfer data from data buffer to I2C
 *
 * @param channel I2C channel, must be less than max channels
 * @param p_data data buffer to write
 * @param length length of data in bytes
 * @param restart restart start condition after transfer
 * @param slave_address address of the the peripheral device (if this is master
 * mode)
 * @return wHalErrorCode I2C_WRITE_FAILED if failed to write, BUSY if another
 * operation is occurring, I2C_WRITE_COMPLETE if write completed,
 * I2C_WRITE_IN_PROGRESS otherwise
 */
wHalErrorCode w_hal_i2c_write_data(uint8_t        channel,
                                   const uint8_t *p_data,
                                   uint32_t       length,
                                   bool           restart,
                                   uint16_t       slave_address = 0,
                                   bool           blocking      = false);

/**
 * @brief Receive data from I2C to data buffer
 *
 * @param channel I2C channel, must be less than max channels
 * @param p_data data buffer to read into
 * @param length length of data in bytes
 * @param restart restart start condition after transfer
 * @param slave_address address of the the peripheral device (if this is master
 * mode)
 * @return wHalErrorCode I2C_READ_FAILED if failed to issue read, BUSY if
 * another operation is occurring, I2C_READ_COMPLETE if read completed,
 * I2C_READ_IN_PROGRESS otherwise
 */
wHalErrorCode w_hal_i2c_read_data(uint8_t  channel,
                                  uint8_t *p_data,
                                  uint32_t length,
                                  bool     restart,
                                  uint16_t slave_address = 0,
                                  bool     blocking      = false);

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
                                        uint16_t       slave_address = 0,
                                        bool           blocking      = false);

/**
 * @brief Check if current outstanding operation is done
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode I2C_WRITE_COMPLETE if write is done,
 * I2C_WRITE_IN_PROGRESS if write is in progress, I2C_READ_COMPLETE if read is
 * done, I2C_READ_IN_PROGRESS if read in progress, I2C_NO_ACTIVE_OPERATION
 * otherwise
 */
wHalErrorCode w_hal_i2c_check_status(uint8_t channel);

/**
 * @brief Clear completion status for channel
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_i2c_clear_completion_status(uint8_t channel);

/**
 * @brief Write word to register of peripheral device on I2C
 *
 * @tparam ADDRESS_SIZE_BYTES register address size in bytes
 * @tparam DATA_SIZE_BYTES register value size in bytes
 * @param channel I2C channel, must be less than max channels
 * @param address register address
 * @param word register value
 * @param restart restart start condition after transfer
 * @param slave_address address of the the peripheral device (if this is master
 * mode)
 * @param blocking block until operation completes
 * @return wHalErrorCode I2C_WRITE_FAILED if failed to write, BUSY if another
 * operation is occurring, I2C_WRITE_COMPLETE if write completed,
 * I2C_WRITE_IN_PROGRESS otherwise
 */
template <int ADDRESS_SIZE_BYTES, int DATA_SIZE_BYTES>
wHalErrorCode w_hal_i2c_write_word(uint16_t channel,
                                   uint32_t address,
                                   uint32_t word,
                                   bool     restart,
                                   uint16_t slave_address,
                                   bool     blocking = false)
{
    uint8_t data[ADDRESS_SIZE_BYTES + DATA_SIZE_BYTES];

    for (uint32_t index = 0; index < ADDRESS_SIZE_BYTES; index++)
    {
        data[index] =
            (uint8_t)(address >> (CHAR_BIT * (ADDRESS_SIZE_BYTES - 1 - index)));
    }

    for (uint32_t index = 0; index < DATA_SIZE_BYTES; index++)
    {
        data[index + ADDRESS_SIZE_BYTES] =
            (uint8_t)(word >> (CHAR_BIT * (DATA_SIZE_BYTES - 1 - index)));
    }

    wHalErrorCode error;
    do
    {
        error = w_hal_i2c_write_data(channel,
                                     data,
                                     ADDRESS_SIZE_BYTES + DATA_SIZE_BYTES,
                                     restart,
                                     slave_address,
                                     blocking);
    } while (blocking && (error == wHalErrorCode::BUSY));

    if (blocking)
    {
        while (error == wHalErrorCode::I2C_WRITE_IN_PROGRESS)
        {
            error = w_hal_i2c_check_status(channel);
        }

        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::I2C_WRITE_COMPLETE);
        w_hal_i2c_clear_completion_status(channel);
    }

    return error;
}

#endif // LIB_HAL_INCLUDE_W_HAL_I2C_HPP