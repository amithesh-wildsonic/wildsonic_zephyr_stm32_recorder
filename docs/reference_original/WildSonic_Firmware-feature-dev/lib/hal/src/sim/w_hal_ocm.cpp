#include "w_hal_ocm.hpp"

#include "assert.hpp"
#include <string.h>

static const uint32_t MEMORY_STORE_SIZE = 0x40000;
static uint8_t        memory_store[MEMORY_STORE_SIZE];

/**
 * @brief Off-chip memory HAL state
 */
enum class ocm_state_t : uint8_t
{
    NONE, // No operation is going on
    DONE  // Transfer is done
};

ocm_state_t ocm_state;

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
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Off-chip memory initialization
 *
 * @param device_config device configuration
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_ocm_init(DeviceConfig::config_page_st &device_config)
{
    memset(memory_store, 0, sizeof(memory_store));

    ocm_state = ocm_state_t::NONE;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Get off-chip memory size
 *
 * @return uint32_t size of memory
 */
uint32_t w_hal_ocm_get_size()
{
    return MEMORY_STORE_SIZE;
}

/**
 * @brief Handler for processing current OCM requests
 *
 * @return wHalErrorCode BUSY if ocm request outstanding, NO_ERROR otherwise
 */
wHalErrorCode w_hal_ocm_handler()
{
    return wHalErrorCode::NO_ERROR;
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
    if ((address >= MEMORY_STORE_SIZE) || (address + size > MEMORY_STORE_SIZE))
    {
        return wHalErrorCode::OCM_ADDRESS_OUT_OF_RANGE;
    }

    if (ocm_state != ocm_state_t::NONE)
    {
        return wHalErrorCode::BUSY;
    }

    memcpy(p_buffer, &memory_store[address], size);

    ocm_state = ocm_state_t::DONE;

    return wHalErrorCode::OCM_READ_IN_PROGRESS;
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
    if ((address >= MEMORY_STORE_SIZE) || (address + size > MEMORY_STORE_SIZE))
    {
        return wHalErrorCode::OCM_ADDRESS_OUT_OF_RANGE;
    }

    if (ocm_state != ocm_state_t::NONE)
    {
        return wHalErrorCode::BUSY;
    }

    memcpy(&memory_store[address], p_buffer, size);

    ocm_state = ocm_state_t::DONE;

    return wHalErrorCode::OCM_WRITE_IN_PROGRESS;
}

/**
 * @brief Check if ocm request is done
 *
 * @return wHalErrorCode OCM_COMMAND_DONE if done, BUSY otherwise
 */
wHalErrorCode w_hal_ocm_check_status()
{
    SYSTEM_ASSERT_EQUAL(ocm_state, ocm_state_t::DONE);
    return wHalErrorCode::OCM_COMMAND_DONE;
}

/**
 * @brief Clear completion status after ocm request is done
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_ocm_clear_completion_status()
{
    SYSTEM_ASSERT_EQUAL(ocm_state, ocm_state_t::DONE);
    ocm_state = ocm_state_t::NONE;

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
    memset(memory_store, 0, sizeof(memory_store));
    ocm_state = ocm_state_t::NONE;
    return wHalErrorCode::NO_ERROR;
}