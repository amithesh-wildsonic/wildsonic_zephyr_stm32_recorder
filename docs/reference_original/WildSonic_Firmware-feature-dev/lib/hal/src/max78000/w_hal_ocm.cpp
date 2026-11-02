#if OFF_CHIP_MEMORY_CORE != INVALID_CORE_NUM
#include "w_hal_ocm.hpp"
#include "w_hal_error_code.hpp" //Header file to enum of errors

#if __CORE__ == OFF_CHIP_MEMORY_CORE

#include "w_hal_ocm_internal.hpp"
#include "w_hal_spi_shared.hpp"

#include "assert.hpp"

#include "board.h"
#include "spi.h"

#include <string.h>

#if defined(BOARD_WS_Rev1)
static const uint32_t GPIO_CS_MASK = MXC_GPIO_PIN_11;
#else
static const uint32_t GPIO_CS_MASK = MXC_GPIO_PIN_10;
#endif
static const uint32_t GPIO_HOLD_MASK = MXC_GPIO_PIN_9;

ocm_info_st           ocm_info;

/**
 * @brief Set up the chip select pin
 */
void                  w_hal_ocm_setup_cs()
{
    mxc_gpio_cfg_t gpio;

    gpio.port  = MXC_GPIO0;
    gpio.mask  = GPIO_CS_MASK;
    gpio.func  = MXC_GPIO_FUNC_OUT;
    gpio.pad   = MXC_GPIO_PAD_NONE;
    gpio.vssel = MXC_GPIO_VSSEL_VDDIOH;
    MXC_GPIO_Config(&gpio);
}

/**
 * @brief Enable chip select for SRAM
 *
 * @param select set enable for SRAM
 */
void w_hal_ocm_cs(bool select)
{
    if (select)
    {
        MXC_GPIO0->out_clr = GPIO_CS_MASK;
    }
    else
    {
        MXC_GPIO0->out_set = GPIO_CS_MASK;
    }
}

/**
 * @brief Set up hold pin
 */
void w_hal_ocm_setup_hold()
{
    mxc_gpio_cfg_t gpio;

    gpio.port  = MXC_GPIO0;
    gpio.mask  = GPIO_HOLD_MASK;
    gpio.func  = MXC_GPIO_FUNC_OUT;
    gpio.pad   = MXC_GPIO_PAD_NONE;
    gpio.vssel = MXC_GPIO_VSSEL_VDDIOH;
    MXC_GPIO_Config(&gpio);
}

/**
 * @brief Enable hold for SRAM
 *
 * @param select select for hold
 */
void w_hal_ocm_set_hold(bool select)
{
    if (select)
    {
        MXC_GPIO0->out_clr = GPIO_HOLD_MASK;
    }
    else
    {
        MXC_GPIO0->out_set = GPIO_HOLD_MASK;
    }
}

/**
 * @brief Off-chip memory initialization
 *
 * @param device_config device configuration
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_ocm_init(DeviceConfig::config_page_st &device_config)
{
    // Initialize structures
    memset(&ocm_info, 0, sizeof(ocm_info));
    ocm_info.state      = ocm_state_t::NONE;

    wHalErrorCode error = w_hal_spi_shared_initialize();
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    const uint32_t audio_data_bit_rate =
        DeviceConfig::get_data_bit_rate(device_config);
    error = w_hal_ocm_init_peripheral(audio_data_bit_rate);
    SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Setup and perform read or write of data from/to address in memory
 *
 * @param address byte address in memory
 * @param p_buffer buffer data
 * @param size buffer size
 * @param read_write true if write, false if read
 * @return wHalErrorCode OCM_ADDRESS_OUT_OF_RANGE if address out of range, BUSY
 * if command is already pending, OCM_WRITE_IN_PROGRESS if read_write=true,
 * OCM_READ_IN_PROGRESS if read_write=false
 */
static wHalErrorCode w_hal_ocm_read_write_data(uint32_t address,
                                               void *   p_buffer,
                                               uint32_t size,
                                               bool     read_write)
{
    uint32_t sram_size = w_hal_ocm_get_size();

    if ((address >= sram_size) || (address + size > sram_size))
    {
        return wHalErrorCode::OCM_ADDRESS_OUT_OF_RANGE;
    }

    if (ocm_info.state != ocm_state_t::NONE)
    {
        return wHalErrorCode::BUSY;
    }

    SYSTEM_ASSERT_NOT_EQUAL(p_buffer, static_cast<void *>(nullptr));
    SYSTEM_ASSERT_NOT_EQUAL(size, 0);

    ocm_info.memory_address   = address;
    ocm_info.p_current_buffer = p_buffer;
    ocm_info.buffer_size      = size;
    ocm_info.read_write       = read_write;
    ocm_info.state            = ocm_state_t::IN_PROGRESS;

    wHalErrorCode error       = w_hal_ocm_handler();
    if (ocm_info.state == ocm_state_t::IN_PROGRESS)
    {
        SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::BUSY);
    }
    else
    {
        SYSTEM_ASSERT_EQUAL(ocm_info.state, ocm_state_t::DONE);
    }

    if (read_write)
    {
        return wHalErrorCode::OCM_WRITE_IN_PROGRESS;
    }
    else
    {
        return wHalErrorCode::OCM_READ_IN_PROGRESS;
    }
}

/**
 * @brief Setup and perform read of data from address in memory
 *
 * @param address byte address in memory
 * @param p_buffer buffer data
 * @param size buffer size
 * @return wHalErrorCode OCM_ADDRESS_OUT_OF_RANGE if address out of range, BUSY
 * if command is already pending, OCM_READ_IN_PROGRESS otherwise
 */
wHalErrorCode
w_hal_ocm_read_data(uint32_t address, void *p_buffer, uint32_t size)
{
    return w_hal_ocm_read_write_data(address, p_buffer, size, false);
}

/**
 * @brief Setup and perform write of data to address in memory
 *
 * @param address byte address in memory
 * @param p_buffer buffer data
 * @param size buffer size
 * @return wHalErrorCode OCM_ADDRESS_OUT_OF_RANGE if address out of range, BUSY
 * if command is already pending, OCM_WRITE_IN_PROGRESS otherwise
 */
wHalErrorCode
w_hal_ocm_write_data(uint32_t address, void *p_buffer, uint32_t size)
{
    return w_hal_ocm_read_write_data(address, p_buffer, size, true);
}

/**
 * @brief Check if ocm request is done
 *
 * @return wHalErrorCode OCM_COMMAND_DONE if done, BUSY otherwise
 */
wHalErrorCode w_hal_ocm_check_status()
{
    return (ocm_info.state == ocm_state_t::DONE) ?
               wHalErrorCode::OCM_COMMAND_DONE :
               wHalErrorCode::BUSY;
}

/**
 * @brief Clear completion status after ocm request is done
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_ocm_clear_completion_status()
{
    SYSTEM_ASSERT_EQUAL(ocm_info.state, ocm_state_t::DONE);

    ocm_info.state = ocm_state_t::NONE;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Teardown the OCM
 *
 * @return wHalErrorCode SPI_SHUTDOWN_FAILED if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_ocm_teardown()
{
    wHalErrorCode error = wHalErrorCode::BUSY;

    while (error == wHalErrorCode::BUSY)
    {
        error = w_hal_ocm_handler();
    }

    // Deselect the peripheral
    w_hal_ocm_cs(false);

    error = w_hal_ocm_teardown_peripheral();

    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    // Release SPI baton for OCM use
    error = wHalErrorCode::BUSY;

    while (error == wHalErrorCode::BUSY)
    {
        error = w_hal_spi_shared_release_baton_for_ocm();
    }

    return wHalErrorCode::NO_ERROR;
}

#endif // __CORE__ == OFF_CHIP_MEMORY_CORE

/**
 * @brief : Validate the OCM Configuration
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
w_hal_ocm_validate_config(DeviceConfig::config_page_st &device_config, bool fix)
{
    // If there is a limit to the max SPI clock frequency, check that the
    // requested data rate times 2 is not greater than the max.
    return wHalErrorCode::NO_ERROR;
}

#endif // OFF_CHIP_MEMORY_CORE != INVALID_CORE_NUM