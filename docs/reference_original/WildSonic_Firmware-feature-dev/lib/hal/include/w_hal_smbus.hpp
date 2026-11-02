#ifndef LIB_HAL_INCLUDE_W_HAL_SMBUS_HPP
#define LIB_HAL_INCLUDE_W_HAL_SMBUS_HPP

#include "w_hal_error_code.hpp"
#include "w_hal_i2c.hpp"

/**
 * @brief Initializes a SMBUS interface
 *
 * @param channel I2C channel, must be less than max channels
 * @param frequency interface frequency
 * @return wHalErrorCode I2C_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_smbus_init(uint8_t channel, uint32_t frequency);

/**
 * @brief Teardown a SMBUS interface
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode I2C_TEARDOWN_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_smbus_teardown(uint8_t channel);

/**
 * @brief Write word data to register on peripheral
 *
 * @param channel I2C channel, must be less than max channels
 * @param slave_address address of the the peripheral device
 * @param reg_address register address
 * @param word word data
 * @return wHalErrorCode I2C_WRITE_FAILED if failed to write, BUSY if another
 * operation is occurring, I2C_WRITE_COMPLETE if write completed,
 * I2C_WRITE_IN_PROGRESS otherwise
 */
wHalErrorCode w_hal_smbus_write_word_data(uint8_t  channel,
                                          uint16_t slave_address,
                                          uint16_t reg_address,
                                          uint32_t word);

/**
 * @brief Issue read word data from register on peripheral
 *
 * @param channel I2C channel, must be less than max channels
 * @param slave_address address of the the peripheral device
 * @param reg_address register address
 * @return wHalErrorCode I2C_READ_FAILED if failed to issue read, BUSY if
 * another operation is occurring, I2C_READ_COMPLETE if read completed,
 * I2C_READ_IN_PROGRESS otherwise
 */
wHalErrorCode w_hal_smbus_read_word_data(uint8_t  channel,
                                         uint16_t slave_address,
                                         uint16_t reg_address);

/**
 * @brief Check if current outstanding operation is done
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode I2C_WRITE_COMPLETE if write is done,
 * I2C_WRITE_IN_PROGRESS if write is in progress, I2C_READ_COMPLETE if read
 * is done, I2C_READ_IN_PROGRESS if read in progress,
 * I2C_NO_ACTIVE_OPERATION otherwise
 */
wHalErrorCode w_hal_smbus_check_status(uint8_t channel);

/**
 * @brief Get read data result after getting completion for read word data from
 * register on peripheral
 *
 * @param channel I2C channel, must be less than max channels
 * @return uint32_t read value
 */
uint32_t      w_hal_smbus_get_read_result(uint8_t channel);

/**
 * @brief Clear completion status for channel
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_smbus_clear_completion_status(uint8_t channel);

#endif // LIB_HAL_INCLUDE_W_HAL_SMBUS_HPP