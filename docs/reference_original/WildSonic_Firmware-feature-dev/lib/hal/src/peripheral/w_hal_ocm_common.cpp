#if __CORE__ == OFF_CHIP_MEMORY_CORE

#include "w_hal_error_code.hpp"
#include "w_hal_ocm.hpp"
#include "w_hal_ocm_internal.hpp"
#include "w_hal_spi.hpp"

#include "assert.hpp"

#include <climits>

/**
 * @brief Blocking send of data over SPI
 *
 * @param spi_channel spi channel number
 * @param p_buffer buffer data
 * @param length length in bytes
 */
void w_hal_ocm_init_send_data(uint8_t  spi_channel,
                              uint8_t *p_buffer,
                              uint32_t length)
{
    wHalErrorCode error = w_hal_spi_write_data(spi_channel, p_buffer, length);

    if (error != wHalErrorCode::SPI_WRITE_COMPLETE)
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::SPI_WRITE_IN_PROGRESS);

        // Wait for completion
        while (error == wHalErrorCode::SPI_WRITE_IN_PROGRESS)
        {
            error = w_hal_spi_check_status(spi_channel);
        }
    }

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::SPI_WRITE_COMPLETE);

    error = w_hal_spi_clear_completion_status(spi_channel);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
}

/**
 * @brief Blocking receive of data over SPI
 *
 * @param spi_channel spi channel number
 * @param p_buffer buffer data
 * @param length length in bytes
 */
void w_hal_ocm_init_receive_data(uint8_t  spi_channel,
                                 uint8_t *p_buffer,
                                 uint32_t length)
{
    wHalErrorCode error = w_hal_spi_read_data(spi_channel, p_buffer, length);

    if (error != wHalErrorCode::SPI_READ_COMPLETE)
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::SPI_READ_IN_PROGRESS);

        // Wait for completion
        while (error == wHalErrorCode::SPI_READ_IN_PROGRESS)
        {
            error = w_hal_spi_check_status(spi_channel);
        }
    }

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::SPI_READ_COMPLETE);

    error = w_hal_spi_clear_completion_status(spi_channel);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
}

/**
 * @brief Blocking dummy cycles over SPI
 *
 * @param spi_channel spi channel number
 * @param p_dummy_buffer dummy buffer data
 * @param cycles number of cycles
 */
void w_hal_ocm_init_dummy_cycles(uint8_t  spi_channel,
                                 uint8_t *p_dummy_buffer,
                                 uint32_t cycles)
{
    w_hal_ocm_init_receive_data(spi_channel,
                                p_dummy_buffer,
                                (cycles / CHAR_BIT) +
                                    ((cycles & (CHAR_BIT - 1)) != 0));
}

/**
 * @brief Wait for outstanding spi write to complete and clear completion
 *
 * @param spi_channel spi channel number
 *
 * @return true if outstanding spi write is complete
 * @return false otherwise
 */
bool w_hal_ocm_check_for_spi_write_complete(uint8_t spi_channel)
{
    // Wait for completion
    wHalErrorCode error = w_hal_spi_check_status(spi_channel);

    if (error == wHalErrorCode::SPI_WRITE_IN_PROGRESS)
    {
        return false;
    }

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::SPI_WRITE_COMPLETE);

    error = w_hal_spi_clear_completion_status(spi_channel);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    return true;
}

/**
 * @brief Wait for outstanding spi read to complete and clear completion
 *
 * @param spi_channel spi channel number
 *
 * @return true if outstanding spi read is complete
 * @return false otherwise
 */
bool w_hal_ocm_check_for_spi_read_complete(uint8_t spi_channel)
{
    wHalErrorCode error = w_hal_spi_check_status(spi_channel);

    if (error == wHalErrorCode::SPI_READ_IN_PROGRESS)
    {
        return false;
    }

    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::SPI_READ_COMPLETE);

    error = w_hal_spi_clear_completion_status(spi_channel);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    return true;
}

#endif // __CORE__ == OFF_CHIP_MEMORY_CORE