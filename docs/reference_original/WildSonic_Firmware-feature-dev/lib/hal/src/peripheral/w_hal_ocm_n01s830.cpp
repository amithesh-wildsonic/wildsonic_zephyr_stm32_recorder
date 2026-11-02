#if __CORE__ == OFF_CHIP_MEMORY_CORE

#include "w_hal_error_code.hpp"
#include "w_hal_ocm.hpp"
#include "w_hal_ocm_internal.hpp"
#include "w_hal_spi.hpp"
#include "w_hal_spi_shared.hpp"

#include "assert.hpp"

#include <climits>
#include <string.h>

/**
 * @brief instruction opcodes for serial sram
 */
enum class sram_opcode_t : uint8_t
{
    WRMR  = 0x01, // Write mode register
    WRITE = 0x02, // Write (program) data to memory starting at selected address
    READ  = 0x03, // Read data from memory starting at selected address
    RDMR  = 0x05, // Read mode register
    EQIO  = 0x38, // Enable QUAD I/O access
    EDIO  = 0x3B, // Enable DUAL I/O access
    RSTQIO = 0xFF, // Reset from QUAD and DUAL to SPI I/O access
};

/**
 * @brief Off-chip memory Internal HAL state
 */
enum class ocm_tracker_state_t : uint8_t
{
    SEQUENCE_FIRST,
    GET_BATON = SEQUENCE_FIRST,          // Get shared SPI baton
    DESELECTED_DUMMY_CYCLES,             // Dummy cycles while device deselected
    WAIT_FOR_DESELECTED_DUMMY_CYCLES,    // Wait for dummy cycles
    CHIP_SELECT,                         // Select chip
    SEND_COMMAND_SEQUENCE,               // Send a read or write command
    WAIT_FOR_SEND_COMMAND_SEQUENCE_DONE, // Wait for read or write command to be
                                         // sent
    DUMMY_CYCLES,                        // Create dummy cycles for reads
                                         // in dual and quad modes
    WAIT_FOR_DUMMY_CYCLES,               // Wait for dummy cycles
    READ_WRITE_DATA,                     // Send or receive requested data
    WAIT_FOR_READ_WRITE_DATA             // Wait for transfer done
};

static const uint32_t      SPI_CHANNEL       = 1;
static const uint32_t      SRAM_SIZE         = 0x20000;
static const uint32_t      DUAL_DUMMY_CYCLES = 4;
static const uint32_t      QUAD_DUMMY_CYCLES = 2;
static const whal_spi_mode spi_mode          = whal_spi_mode::QUAD_HALF_DUPLEX;

const uint32_t             SPI_SRAM_FREQ     = 30000000;

/**
 * @brief Off-chip memory information structure
 */
struct ocm_tracker_st
{
    ocm_tracker_state_t state;
    uint8_t             command_mem[4]; // command data
};

static ocm_tracker_st ocm_tracker;

/**
 * @brief Off-chip memory peripheral initialization
 *
 * @param data_bit_rate requested data rate in bits per second
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode         w_hal_ocm_init_peripheral(uint32_t data_bit_rate)
{
    const int num_slaves = 0;

    memset(&ocm_tracker, 0, sizeof(ocm_tracker));
    ocm_tracker.state = ocm_tracker_state_t::SEQUENCE_FIRST;

    w_hal_ocm_setup_cs();
    w_hal_ocm_setup_hold();

    w_hal_ocm_set_hold(false);
    w_hal_ocm_cs(false);

    uint32_t spi_sram_freq = SPI_SRAM_FREQ;
    if (data_bit_rate * 4 > SPI_SRAM_FREQ)
    {
        spi_sram_freq = data_bit_rate * 4;
    }

    // Initialize SPI interface as standard spi
    // If dual or quad mode desired, switch to it only after
    // sending reset and switch commands in single full-duplex
    // mode
    wHalErrorCode error        = w_hal_spi_init(SPI_CHANNEL,
                                         whal_spi_mode::FOUR_WIRE_FULL_DUPLEX,
                                         whal_master_slave_select::MASTER,
                                         whal_voltage_select::VOLTAGE_3V3,
                                         spi_sram_freq,
                                         num_slaves);

    // While chip is not selected, do dummy cycles over SPI
    ocm_tracker.command_mem[0] = 0xFF;
    w_hal_ocm_init_dummy_cycles(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    w_hal_ocm_cs(true);

    // Send command sequence for reset
    ocm_tracker.command_mem[0] = static_cast<uint8_t>(sram_opcode_t::RSTQIO);
    w_hal_ocm_init_send_data(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    w_hal_ocm_cs(false);

    // Enable Dual/Quad I/O access
    if ((spi_mode == whal_spi_mode::DUAL_HALF_DUPLEX) ||
        (spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX))
    {
        sram_opcode_t opcode = (spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX) ?
                                   sram_opcode_t::EQIO :
                                   sram_opcode_t::EDIO;

        w_hal_ocm_cs(true);

        // Send command sequence for mode change
        ocm_tracker.command_mem[0] = static_cast<uint8_t>(opcode);
        w_hal_ocm_init_send_data(SPI_CHANNEL, ocm_tracker.command_mem, 1);

        w_hal_ocm_cs(false);

        // Reinitialize SPI with Dual/Quad configuration
        error = w_hal_spi_init(SPI_CHANNEL,
                               spi_mode,
                               whal_master_slave_select::MASTER,
                               whal_voltage_select::VOLTAGE_3V3,
                               spi_sram_freq,
                               num_slaves);
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Get off-chip memory size
 *
 * @return uint32_t size of memory
 */
uint32_t w_hal_ocm_get_size()
{
    return SRAM_SIZE;
}

/**
 * @brief Handler for processing current OCM requests
 *
 * @return wHalErrorCode BUSY if ocm request outstanding, NO_ERROR otherwise
 */
wHalErrorCode w_hal_ocm_handler()
{
    wHalErrorCode return_error = wHalErrorCode::NO_ERROR;
    wHalErrorCode error;

    if (ocm_info.state == ocm_state_t::IN_PROGRESS)
    {
        return_error = wHalErrorCode::BUSY;

        switch (ocm_tracker.state)
        {
            case ocm_tracker_state_t::GET_BATON:
            {
                if (w_hal_spi_shared_request_baton_for_ocm() ==
                    wHalErrorCode::BUSY)
                {
                    return return_error;
                }

                ocm_tracker.state =
                    ocm_tracker_state_t::DESELECTED_DUMMY_CYCLES;
            }
            case ocm_tracker_state_t::DESELECTED_DUMMY_CYCLES:
            {
                w_hal_ocm_cs(false);

                ocm_tracker.command_mem[0] = 0;
                error                      = w_hal_spi_read_data(SPI_CHANNEL,
                                            ocm_tracker.command_mem,
                                            1);

                wHalErrorCode valid_read_errors[] = {
                    wHalErrorCode::SPI_READ_IN_PROGRESS,
                    wHalErrorCode::SPI_READ_COMPLETE};

                SYSTEM_ASSERT_IN_SET(error,
                                     valid_read_errors,
                                     sizeof(valid_read_errors) /
                                         sizeof(valid_read_errors[0]));

                ocm_tracker.state =
                    ocm_tracker_state_t::WAIT_FOR_DESELECTED_DUMMY_CYCLES;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::WAIT_FOR_DESELECTED_DUMMY_CYCLES:
            {
                if (!w_hal_ocm_check_for_spi_read_complete(SPI_CHANNEL))
                {
                    break;
                }

                ocm_tracker.state = ocm_tracker_state_t::CHIP_SELECT;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::CHIP_SELECT:
            {
                w_hal_ocm_cs(true);
                ocm_tracker.state = ocm_tracker_state_t::SEND_COMMAND_SEQUENCE;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::SEND_COMMAND_SEQUENCE:
            {
                sram_opcode_t opcode       = ocm_info.read_write ?
                                                 sram_opcode_t::WRITE :
                                                 sram_opcode_t::READ;

                ocm_tracker.command_mem[0] = static_cast<uint8_t>(opcode);
                for (uint8_t index = 1; index < sizeof(ocm_tracker.command_mem);
                     index++)
                {
                    ocm_tracker.command_mem[index] =
                        ocm_info.memory_address >>
                        (CHAR_BIT *
                         (sizeof(ocm_tracker.command_mem) - 1 - index));
                }

                // Send command sequence for Write/Read
                error = w_hal_spi_write_data(SPI_CHANNEL,
                                             ocm_tracker.command_mem,
                                             sizeof(ocm_tracker.command_mem));

                wHalErrorCode valid_write_errors[] = {
                    wHalErrorCode::SPI_WRITE_IN_PROGRESS,
                    wHalErrorCode::SPI_WRITE_COMPLETE};

                SYSTEM_ASSERT_IN_SET(error,
                                     valid_write_errors,
                                     sizeof(valid_write_errors) /
                                         sizeof(valid_write_errors[0]));
                ocm_tracker.state =
                    ocm_tracker_state_t::WAIT_FOR_SEND_COMMAND_SEQUENCE_DONE;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::WAIT_FOR_SEND_COMMAND_SEQUENCE_DONE:
            {
                if (!w_hal_ocm_check_for_spi_write_complete(SPI_CHANNEL))
                {
                    break;
                }

                ocm_tracker.state = ocm_tracker_state_t::DUMMY_CYCLES;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::DUMMY_CYCLES:
            {
                // Dummy cycles are only needed for reads in dual and quad modes
                if (((spi_mode == whal_spi_mode::DUAL_HALF_DUPLEX) ||
                     (spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX)) &&
                    !ocm_info.read_write)
                {
                    // Read dummy data
                    uint32_t bytes =
                        ((spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX) ?
                             (QUAD_DUMMY_CYCLES * 4) :
                             (DUAL_DUMMY_CYCLES * 2)) /
                        CHAR_BIT;

                    ocm_tracker.command_mem[0] = 0;
                    error = w_hal_spi_read_data(SPI_CHANNEL,
                                                ocm_tracker.command_mem,
                                                bytes);

                    wHalErrorCode valid_read_errors[] = {
                        wHalErrorCode::SPI_READ_IN_PROGRESS,
                        wHalErrorCode::SPI_READ_COMPLETE};

                    SYSTEM_ASSERT_IN_SET(error,
                                         valid_read_errors,
                                         sizeof(valid_read_errors) /
                                             sizeof(valid_read_errors[0]));

                    ocm_tracker.state =
                        ocm_tracker_state_t::WAIT_FOR_DUMMY_CYCLES;
                }
                else
                {
                    ocm_tracker.state = ocm_tracker_state_t::READ_WRITE_DATA;
                    break;
                }
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::WAIT_FOR_DUMMY_CYCLES:
            {
                if (!w_hal_ocm_check_for_spi_read_complete(SPI_CHANNEL))
                {
                    break;
                }

                ocm_tracker.state = ocm_tracker_state_t::READ_WRITE_DATA;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::READ_WRITE_DATA:
            {
                if (ocm_info.read_write)
                {
                    error = w_hal_spi_write_data(
                        SPI_CHANNEL,
                        static_cast<uint8_t *>(ocm_info.p_current_buffer),
                        ocm_info.buffer_size);

                    wHalErrorCode valid_write_errors[] = {
                        wHalErrorCode::SPI_WRITE_IN_PROGRESS,
                        wHalErrorCode::SPI_WRITE_COMPLETE};

                    SYSTEM_ASSERT_IN_SET(error,
                                         valid_write_errors,
                                         sizeof(valid_write_errors) /
                                             sizeof(valid_write_errors[0]));
                }
                else
                {
                    error = w_hal_spi_read_data(
                        SPI_CHANNEL,
                        static_cast<uint8_t *>(ocm_info.p_current_buffer),
                        ocm_info.buffer_size);

                    wHalErrorCode valid_read_errors[] = {
                        wHalErrorCode::SPI_READ_IN_PROGRESS,
                        wHalErrorCode::SPI_READ_COMPLETE};

                    SYSTEM_ASSERT_IN_SET(error,
                                         valid_read_errors,
                                         sizeof(valid_read_errors) /
                                             sizeof(valid_read_errors[0]));
                }

                ocm_tracker.state =
                    ocm_tracker_state_t::WAIT_FOR_READ_WRITE_DATA;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::WAIT_FOR_READ_WRITE_DATA:
            {
                if (ocm_info.read_write)
                {
                    if (!w_hal_ocm_check_for_spi_write_complete(SPI_CHANNEL))
                    {
                        break;
                    }
                }
                else
                {
                    if (!w_hal_ocm_check_for_spi_read_complete(SPI_CHANNEL))
                    {
                        break;
                    }
                }

                w_hal_ocm_cs(false);

                error = w_hal_spi_shared_release_baton_for_ocm();
                SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

                ocm_tracker.state = ocm_tracker_state_t::SEQUENCE_FIRST;
                ocm_info.state    = ocm_state_t::DONE;
                return_error      = wHalErrorCode::NO_ERROR;
                break;
            }
            default:
            {
                SYSTEM_ASSERT_TRUE(false);
                return_error = wHalErrorCode::NO_ERROR;
                break;
            }
        }
    }

    return return_error;
}

/**
 * @brief Teardown the OCM peripheral
 *
 * @return wHalErrorCode SPI_SHUTDOWN_FAILED if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_ocm_teardown_peripheral()
{
    // Hold the ocm in reset state
    w_hal_ocm_set_hold(1);

    // Do dummy cycles over SPI
    ocm_tracker.command_mem[0] = 0xFF;
    w_hal_ocm_init_dummy_cycles(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    return w_hal_spi_teardown(SPI_CHANNEL);
}

#endif // __CORE__ == OFF_CHIP_MEMORY_CORE