#ifndef LIB_HAL_INCLUDE_W_HAL_SPI_HPP
#define LIB_HAL_INCLUDE_W_HAL_SPI_HPP

#include "w_hal_common.hpp"
#include "w_hal_error_code.hpp"

/**
 * @brief The list of SPI master/select options supported
 */
enum class whal_master_slave_select
{
    MASTER,
    SLAVE,
};

/**
 * @brief SPI communication type
 */
enum class whal_spi_mode
{
    THREE_WIRE_HALF_DUPLEX,
    FOUR_WIRE_FULL_DUPLEX,
    DUAL_HALF_DUPLEX,
    QUAD_HALF_DUPLEX,
};

/**
 * @brief Initializes a SPI interface
 *
 * @param channel SPI channel, must be less than max channels
 * @param mode SPI mode
 * @param frequency interface frequency
 * @param slave_address address of this device (only applicable if slave mode)
 * @return wHalErrorCode SPI_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_init(uint8_t                  channel,
                             whal_spi_mode            spi_mode,
                             whal_master_slave_select master_slave_select,
                             whal_voltage_select      voltage_select,
                             uint32_t                 frequency,
                             uint8_t                  num_slaves);

/**
 * @brief Switch to different spi width
 *
 * @param channel SPI channel, must be less than max channels
 * @param spi_mode mode to switch to
 * @return wHalErrorCode SPI_INITIALISATION_FAILED if something went wrong,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_set_mode(int channel, whal_spi_mode spi_mode);

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
w_hal_spi_write_data(uint8_t channel, const uint8_t *p_data, uint32_t length);

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
w_hal_spi_read_data(uint8_t channel, uint8_t *p_data, uint32_t length);

/**
 * @brief Check if current outstanding operation is done
 *
 * @param channel SPI channel, must be less than max channels
 * @return wHalErrorCode SPI_WRITE_COMPLETE if write is done,
 * SPI_WRITE_IN_PROGRESS if write is in progress, SPI_READ_COMPLETE if read is
 * done, SPI_READ_IN_PROGRESS if read in progress, SPI_NO_ACTIVE_OPERATION
 * otherwise
 */
wHalErrorCode w_hal_spi_check_status(uint8_t channel);

/**
 * @brief Clear completion status for channel
 *
 * @param channel SPI channel, must be less than max channels
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_spi_clear_completion_status(uint8_t channel);

/**
 * @brief SPI Teardown function
 *
 * @return wHalErrorCode SPI_SHUTDOWN_FAILED if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_spi_teardown(uint8_t channel);

#endif // LIB_HAL_INCLUDE_W_HAL_SPI_HPP