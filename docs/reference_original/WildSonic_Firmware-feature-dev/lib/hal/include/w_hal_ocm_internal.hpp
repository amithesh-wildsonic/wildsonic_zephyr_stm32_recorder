#ifndef LIB_HAL_W_HAL_OCM_INTERNAL_HPP
#define LIB_HAL_W_HAL_OCM_INTERNAL_HPP

#include "w_hal_error_code.hpp"
#include "w_hal_spi.hpp"

/**
 * @brief Off-chip memory HAL state
 */
enum class ocm_state_t : uint8_t
{
    NONE,        // No operation is going on
    IN_PROGRESS, // Operation in progress
    DONE         // Transfer is done
};

/**
 * @brief Off-chip memory command information structure
 */
struct ocm_info_st
{
    ocm_state_t state; // state

    uint32_t    memory_address;   // memory address to read or write
    void *      p_current_buffer; // request buffer
    uint32_t    buffer_size;      // request size

    bool        read_write; // true if write, false if read
};

extern ocm_info_st ocm_info;

/**
 * @brief Blocking send of data over SPI
 *
 * @param spi_channel spi channel number
 * @param p_buffer buffer data
 * @param length length in bytes
 */
void               w_hal_ocm_init_send_data(uint8_t  spi_channel,
                                            uint8_t *p_buffer,
                                            uint32_t length);

/**
 * @brief Blocking receive of data over SPI
 *
 * @param spi_channel spi channel number
 * @param p_buffer buffer data
 * @param length length in bytes
 */
void               w_hal_ocm_init_receive_data(uint8_t  spi_channel,
                                               uint8_t *p_buffer,
                                               uint32_t length);

/**
 * @brief Blocking dummy cycles over SPI
 *
 * @param spi_channel spi channel number
 * @param p_dummy_buffer dummy buffer data
 * @param cycles number of cycles
 */
void               w_hal_ocm_init_dummy_cycles(uint8_t  spi_channel,
                                               uint8_t *p_dummy_buffer,
                                               uint32_t cycles);

/**
 * @brief Off-chip memory peripheral initialization
 *
 * @param data_bit_rate requested data rate in bits per second
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode      w_hal_ocm_init_peripheral(uint32_t data_bit_rate);

/**
 * @brief Wait for outstanding spi write to complete and clear completion
 *
 * @param spi_channel spi channel number
 *
 * @return true if outstanding spi write is complete
 * @return false otherwise
 */
bool               w_hal_ocm_check_for_spi_write_complete(uint8_t spi_channel);

/**
 * @brief Wait for outstanding spi read to complete and clear completion
 *
 * @param spi_channel spi channel number
 *
 * @return true if outstanding spi read is complete
 * @return false otherwise
 */
bool               w_hal_ocm_check_for_spi_read_complete(uint8_t spi_channel);

/**
 * @brief Set up the chip select pin
 */
void               w_hal_ocm_setup_cs();

/**
 * @brief Enable chip select for SRAM
 *
 * @param select set enable for SRAM
 */
void               w_hal_ocm_cs(bool select);

/**
 * @brief Set up hold pin
 */
void               w_hal_ocm_setup_hold();

/**
 * @brief Enable hold for SRAM
 *
 * @param select select for hold
 */
void               w_hal_ocm_set_hold(bool select);

/**
 * @brief Teardown the OCM peripheral
 *
 * @return wHalErrorCode SPI_SHUTDOWN_FAILED if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode      w_hal_ocm_teardown_peripheral();

#endif // LIB_HAL_W_HAL_OCM_INTERNAL_HPP