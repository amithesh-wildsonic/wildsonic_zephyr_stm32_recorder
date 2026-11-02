#include "w_hal_smbus.hpp"
#include "w_hal_i2c.hpp"

#include "mxc.h"

#include <climits>

const uint32_t SMBUS_WORD_SIZE        = 2;
const uint32_t SMBUS_REG_ADDRESS_SIZE = 1;

/**
 * @brief Tracking for I2C channel
 */
struct smbus_channel_tracker_t
{
    uint8_t data[SMBUS_REG_ADDRESS_SIZE + SMBUS_WORD_SIZE];
};

static smbus_channel_tracker_t smbus_channel[MXC_I2C_INSTANCES];

/**
 * @brief Initializes a SMBUS interface
 *
 * @param channel I2C channel, must be less than max channels
 * @param frequency interface frequency
 * @return wHalErrorCode I2C_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_smbus_init(uint8_t channel, uint32_t frequency)
{
    return w_hal_i2c_init(channel, whal_i2c_mode::MASTER, frequency);
}

/**
 * @brief Teardown a SMBUS interface
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode I2C_TEARDOWN_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_smbus_teardown(uint8_t channel)
{
    return w_hal_i2c_teardown(channel);
}

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
                                          uint32_t word)
{
    smbus_channel[channel].data[0] = reg_address;
    for (uint32_t index = 0; index < SMBUS_WORD_SIZE; index++)
    {
        smbus_channel[channel].data[index + 1] =
            (uint8_t)(word >> (CHAR_BIT * index));
    }

    return w_hal_i2c_write_data(channel,
                                smbus_channel[channel].data,
                                SMBUS_REG_ADDRESS_SIZE + SMBUS_WORD_SIZE,
                                false,
                                slave_address);
}

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
                                         uint16_t reg_address)
{
    smbus_channel[channel].data[0] = reg_address;
    return w_hal_i2c_write_read_data(channel,
                                     smbus_channel[channel].data,
                                     SMBUS_REG_ADDRESS_SIZE,
                                     &smbus_channel[channel].data[1],
                                     SMBUS_WORD_SIZE,
                                     false,
                                     slave_address);
}

/**
 * @brief Check if current outstanding operation is done
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode I2C_WRITE_COMPLETE if write is done,
 * I2C_WRITE_IN_PROGRESS if write is in progress, I2C_READ_COMPLETE if read
 * is done, I2C_READ_IN_PROGRESS if read in progress,
 * I2C_NO_ACTIVE_OPERATION otherwise
 */
wHalErrorCode w_hal_smbus_check_status(uint8_t channel)
{
    return w_hal_i2c_check_status(channel);
}

/**
 * @brief Get read data result after getting completion for read word data from
 * register on peripheral
 *
 * @param channel I2C channel, must be less than max channels
 * @return uint32_t read value
 */
uint32_t w_hal_smbus_get_read_result(uint8_t channel)
{

    uint32_t value = 0;

    for (uint32_t index = 0; index < SMBUS_WORD_SIZE; index++)
    {
        value |= smbus_channel[channel].data[index + 1] << (CHAR_BIT * index);
    }

    return value;
}

/**
 * @brief Clear completion status for channel
 *
 * @param channel I2C channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_smbus_clear_completion_status(uint8_t channel)
{
    return w_hal_i2c_clear_completion_status(channel);
}