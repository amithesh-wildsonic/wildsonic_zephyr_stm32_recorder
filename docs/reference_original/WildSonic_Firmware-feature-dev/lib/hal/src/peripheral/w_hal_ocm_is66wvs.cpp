#if __CORE__ == OFF_CHIP_MEMORY_CORE

#include "w_hal_error_code.hpp"
#include "w_hal_ocm.hpp"
#include "w_hal_ocm_internal.hpp"
#include "w_hal_spi.hpp"
#include "w_hal_spi_shared.hpp"
#include "w_hal_system.hpp"

#include "assert.hpp"

#include <algorithm>
#include <climits>
#include <string.h>

/**
 * @brief instruction opcodes for serial sram
 */
enum class sram_opcode_t : uint8_t
{
    WRITE = 0x02, // Write (program) data to memory starting at selected address
    READ  = 0x03, // Read data from memory starting at selected address
    FAST_READ        = 0x0B,
    ENTER_QPI_MODE   = 0x35,
    QUAD_IO_WRITE    = 0x38,
    RESET_ENABLE     = 0x66,
    RESET            = 0x99,
    READ_ID          = 0x9F,
    DPD_MODE_ENTRY   = 0xB9,
    SET_BURST_LENGTH = 0xC0,
    QUAD_IO_READ     = 0xEB,
    EXIT_QPI_MODE    = 0xF5,
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
                                         // in quad modes
    WAIT_FOR_DUMMY_CYCLES,               // Wait for dummy cycles
    READ_WRITE_DATA,                     // Send or receive requested data
    WAIT_FOR_READ_WRITE_DATA,            // Wait for transfer done
    TERMINATE_DUMMY_CYCLES,              // Create a dummy cycle with
                                         // chip not selected to
                                         // terminate command
    WAIT_FOR_TERMINATE_DUMMY_CYCLES,     // Wait for termination dummy cycles
};

static const uint32_t SPI_CHANNEL       = 1;
static const uint32_t MIN_SRAM_SIZE     = 0x100000; // Smallest density in bytes
static const uint32_t QUAD_DUMMY_CYCLES = 4;
static const whal_spi_mode spi_mode     = whal_spi_mode::QUAD_HALF_DUPLEX;
static const uint32_t      PAGE_LENGTH  = 1024; // Burst length before wrap
static const uint8_t       MANUFACTURER_ID        = 0x9D; // ISSI
static const uint8_t       KNOWN_GOOD_DIE_PASS    = 0x5D;
static const uint8_t READ_ID_ADDRESS_DUMMY_CYCLES = 24; // One cycle per bit

const uint32_t       SPI_SRAM_FREQ                = 30000000;

/**
 * @brief Read ID Structure (in order of how it is read on the data lines)
 */
struct read_id_st
{
    uint8_t mfid; // Manufacturer ID
    uint8_t kgd;  // Known Good Die Test Result
    union
    {
        uint8_t byte;
        struct
        {
            uint8_t rsvd    : 5;
            uint8_t density : 3; // Device Density
        } bits;
    } byte3;
    uint8_t rsvd[5];
};

/**
 * @brief Off-chip memory information structure
 */
struct ocm_tracker_st
{
    ocm_tracker_state_t state; // Tracker state
    uint32_t   current_offset; // Current operating offset into the buffer for
                               // read/write
    read_id_st read_id;        // Read ID Structure
    uint8_t    command_mem[4]; // command data
    uint32_t   sram_size;      // SRAM Size in bytes
};

static ocm_tracker_st ocm_tracker;

/**
 * @brief Reset sequence on power-up
 */
static void           w_hal_ocm_init_reset()
{
    w_hal_ocm_cs(true);

    // Send command sequence for reset enable
    ocm_tracker.command_mem[0] =
        static_cast<uint8_t>(sram_opcode_t::RESET_ENABLE);
    w_hal_ocm_init_send_data(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    w_hal_ocm_cs(false);
    w_hal_system_delay_ms(1);
    w_hal_ocm_cs(true);

    // Send command sequence for reset
    ocm_tracker.command_mem[0] = static_cast<uint8_t>(sram_opcode_t::RESET);
    w_hal_ocm_init_send_data(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    w_hal_ocm_cs(false);
}

/**
 * @brief Initialization Read ID Register Sequence
 */
static void w_hal_ocm_init_readid()
{
    w_hal_ocm_cs(true);

    // Send command sequence for Read ID
    ocm_tracker.command_mem[0] = static_cast<uint8_t>(sram_opcode_t::READ_ID);
    w_hal_ocm_init_send_data(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    // Do dummy cycles for don't care address
    ocm_tracker.command_mem[0] = 0xFF;
    w_hal_ocm_init_dummy_cycles(SPI_CHANNEL,
                                ocm_tracker.command_mem,
                                READ_ID_ADDRESS_DUMMY_CYCLES);

    // Read ID Data
    w_hal_ocm_init_receive_data(
        SPI_CHANNEL,
        reinterpret_cast<uint8_t *>(&ocm_tracker.read_id),
        sizeof(ocm_tracker.read_id));

    w_hal_ocm_cs(false);

    SYSTEM_ASSERT_EQUAL(ocm_tracker.read_id.mfid, MANUFACTURER_ID);
    SYSTEM_ASSERT_EQUAL(ocm_tracker.read_id.kgd, KNOWN_GOOD_DIE_PASS);

    ocm_tracker.sram_size = MIN_SRAM_SIZE
                            << ocm_tracker.read_id.byte3.bits.density;
}

/**
 * @brief Off-chip memory peripheral initialization
 *
 * @param data_bit_rate requested data rate in bits per second
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_ocm_init_peripheral(uint32_t data_bit_rate)
{
    const int num_slaves = 0;

    // Wait 150us for initialization
    w_hal_system_delay_us(150);

    memset(&ocm_tracker, 0, sizeof(ocm_tracker));
    ocm_tracker.state = ocm_tracker_state_t::SEQUENCE_FIRST;

    w_hal_ocm_setup_cs();
    w_hal_ocm_cs(false);

    uint32_t spi_sram_freq = SPI_SRAM_FREQ;
    if (data_bit_rate * 4 > SPI_SRAM_FREQ)
    {
        spi_sram_freq = data_bit_rate * 4;
    }

    wHalErrorCode error;

    /* If quad mode is configured, the external memory might still be in QPI
     * mode from before a warm MCU reset. Send EXIT_QPI_MODE in quad mode to
     * return it to SPI mode. */
    if (spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX)
    {
        error = w_hal_spi_init(SPI_CHANNEL,
                               whal_spi_mode::QUAD_HALF_DUPLEX,
                               whal_master_slave_select::MASTER,
                               whal_voltage_select::VOLTAGE_3V3,
                               spi_sram_freq,
                               num_slaves);
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

        /* At least one clock pulse of CE# HIGH is needed before beginning any
         * operations. Do dummy cycles over SPI */
        ocm_tracker.command_mem[0] = 0xFF;
        w_hal_ocm_init_dummy_cycles(SPI_CHANNEL, ocm_tracker.command_mem, 1);

        w_hal_ocm_cs(true);

        // Send command sequence for mode change to exit QPI mode
        ocm_tracker.command_mem[0] =
            static_cast<uint8_t>(sram_opcode_t::EXIT_QPI_MODE);
        w_hal_ocm_init_send_data(SPI_CHANNEL, ocm_tracker.command_mem, 1);

        w_hal_ocm_cs(false);

        w_hal_system_delay_us(50);
    }

    /* Initialize SPI interface as standard spi if quad mode is desired, switch
     * to it only after sending reset and switch commands in single full-duplex
     * mode */
    error = w_hal_spi_init(SPI_CHANNEL,
                           whal_spi_mode::FOUR_WIRE_FULL_DUPLEX,
                           whal_master_slave_select::MASTER,
                           whal_voltage_select::VOLTAGE_3V3,
                           spi_sram_freq,
                           num_slaves);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // At least one clock pulse of CE# HIGH is needed before beginning any
    // operations. Do dummy cycles over SPI
    ocm_tracker.command_mem[0] = 0xFF;
    w_hal_ocm_init_dummy_cycles(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    w_hal_ocm_init_reset();

    w_hal_ocm_init_readid();

    static_assert((spi_mode == whal_spi_mode::FOUR_WIRE_FULL_DUPLEX) ||
                      (spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX),
                  "Only SPI and QPI mode supported");

    // Enable QPI access
    if (spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX)
    {
        w_hal_ocm_cs(true);

        // Send command sequence for mode change
        ocm_tracker.command_mem[0] =
            static_cast<uint8_t>(sram_opcode_t::ENTER_QPI_MODE);
        w_hal_ocm_init_send_data(SPI_CHANNEL, ocm_tracker.command_mem, 1);

        w_hal_ocm_cs(false);

        // Reinitialize with QPI configuration
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
    return ocm_tracker.sram_size;
}

/**
 * @brief Get length for next page aligned transfer
 *
 * @return uint32_t length to transfer from buffer
 */
static uint32_t w_hal_ocm_get_next_transfer_length()
{
    uint32_t remaining_bytes_in_request =
        ocm_info.buffer_size - ocm_tracker.current_offset;
    uint32_t remaining_bytes_in_page =
        PAGE_LENGTH - ((ocm_info.memory_address + ocm_tracker.current_offset) &
                       (PAGE_LENGTH - 1));

    return std::min(remaining_bytes_in_request, remaining_bytes_in_page);
}

/**
 * @brief Handler for processing current OCM requests
 *
 * @return wHalErrorCode BUSY if ocm request outstanding, NO_ERROR otherwise
 */
wHalErrorCode w_hal_ocm_handler()
{
OCM_HANDLER_BEGIN:
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
                sram_opcode_t opcode = ocm_info.read_write ?
                                           sram_opcode_t::WRITE :
                                           sram_opcode_t::READ;

                uint32_t      memory_address =
                    ocm_info.memory_address + ocm_tracker.current_offset;

                ocm_tracker.command_mem[0] = static_cast<uint8_t>(opcode);
                for (uint8_t index = 1; index < sizeof(ocm_tracker.command_mem);
                     index++)
                {
                    ocm_tracker.command_mem[index] =
                        memory_address >>
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
                // Dummy cycles are only needed for reads in quad modes
                if ((spi_mode == whal_spi_mode::QUAD_HALF_DUPLEX) &&
                    !ocm_info.read_write)
                {
                    // Read dummy data
                    uint32_t bytes = QUAD_DUMMY_CYCLES * 4 / CHAR_BIT;

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
                uint32_t transfer_size = w_hal_ocm_get_next_transfer_length();

                if (ocm_info.read_write)
                {
                    error = w_hal_spi_write_data(
                        SPI_CHANNEL,
                        static_cast<uint8_t *>(ocm_info.p_current_buffer) +
                            ocm_tracker.current_offset,
                        transfer_size);

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
                        static_cast<uint8_t *>(ocm_info.p_current_buffer) +
                            ocm_tracker.current_offset,
                        transfer_size);

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

                ocm_tracker.state = ocm_tracker_state_t::TERMINATE_DUMMY_CYCLES;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::TERMINATE_DUMMY_CYCLES:
            {
                // All reads and writes must be followed immediately by a clock
                // pulse of CE# HIGH in order to terminate the read and writes,
                // and set the device into standby status
                w_hal_ocm_cs(false);

                // Read dummy data
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
                    ocm_tracker_state_t::WAIT_FOR_TERMINATE_DUMMY_CYCLES;
            }
            // FALL-THROUGH
            case ocm_tracker_state_t::WAIT_FOR_TERMINATE_DUMMY_CYCLES:
            {
                if (!w_hal_ocm_check_for_spi_read_complete(SPI_CHANNEL))
                {
                    break;
                }

                ocm_tracker.current_offset +=
                    w_hal_ocm_get_next_transfer_length();
                if (ocm_tracker.current_offset < ocm_info.buffer_size)
                {
                    // Finished transferring only up to page boundary
                    // Need to perform additional transfers
                    // Go back to re-selecting chip and performing operation
                    ocm_tracker.state = ocm_tracker_state_t::CHIP_SELECT;
                    goto OCM_HANDLER_BEGIN;
                }
                else
                {
                    // Have finished transferring entire buffer
                    // Can reset and report done
                    ocm_tracker.current_offset = 0;

                    error = w_hal_spi_shared_release_baton_for_ocm();
                    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

                    ocm_tracker.state = ocm_tracker_state_t::SEQUENCE_FIRST;
                    ocm_info.state    = ocm_state_t::DONE;
                    return_error      = wHalErrorCode::NO_ERROR;
                }

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
    wHalErrorCode error = w_hal_spi_shared_request_baton_for_ocm();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    // Do dummy cycles over SPI
    ocm_tracker.command_mem[0] = 0xFF;
    w_hal_ocm_init_dummy_cycles(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    w_hal_ocm_cs(true);

    // Send command sequence for mode change
    ocm_tracker.command_mem[0] =
        static_cast<uint8_t>(sram_opcode_t::EXIT_QPI_MODE);
    w_hal_ocm_init_send_data(SPI_CHANNEL, ocm_tracker.command_mem, 1);

    w_hal_ocm_cs(false);

    // Reinitialize with Normal configuration
    error = w_hal_spi_init(SPI_CHANNEL,
                           whal_spi_mode::FOUR_WIRE_FULL_DUPLEX,
                           whal_master_slave_select::MASTER,
                           whal_voltage_select::VOLTAGE_3V3,
                           SPI_SRAM_FREQ,
                           0);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    return wHalErrorCode::NO_ERROR;
}

#endif // __CORE__ == OFF_CHIP_MEMORY_CORE
