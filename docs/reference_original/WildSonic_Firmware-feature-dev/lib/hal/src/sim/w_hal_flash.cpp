#include "w_hal_flash.hpp"

static const uint32_t PAGE_SIZE = 0x2000;
static const uint32_t MEM_SIZE  = PAGE_SIZE * 2; // No code space
                                                 // in this simulation

static uint8_t        flash_memory[MEM_SIZE];

/**
 * @brief Initialize flash
 *
 * @return wHalErrorCode FLASH_INITIALIZATION_FAILED if flash initialization
 * failed
 * @return wHalErrorCode NO_ERROR if flash initialization succeeded
 */
wHalErrorCode         w_hal_flash_init(void)
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Teardown flash
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_flash_teardown(void)
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Get cpu address of flash base
 *
 * @return uint32_t cpu address of flash base
 */
uint32_t w_hal_flash_get_cpu_base_address()
{
    return 0;
}

/**
 * @brief Get flash size
 *
 * @param page_size page size
 * @param memory_size memory size
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_flash_get_parameters(uint32_t &page_size,
                                         uint32_t &memory_size)
{
    page_size   = PAGE_SIZE;
    memory_size = MEM_SIZE;

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Erase flash page
 *
 * @param address flash address
 * @return wHalErrorCode FLASH_ERASE_FAILED if flash erase failed
 * @return wHalErrorCode NO_ERROR if flash erase succeeded
 */
wHalErrorCode w_hal_flash_page_erase(uint32_t address)
{
    uint32_t page_address = address & ~(PAGE_SIZE - 1);

    for (uint32_t i = 0; i < PAGE_SIZE; i++)
    {
        flash_memory[page_address + i] = 0xFF;
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Program flash
 *
 * @param address flash address
 * @param p_data buffer data
 * @param size length of buffer data
 * @return wHalErrorCode FLASH_WRITE_FAILED if flash write failed
 * @return wHalErrorCode NO_ERROR if flash write succeeded
 */
wHalErrorCode
w_hal_flash_program(uint32_t address, uint8_t *p_data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        flash_memory[address + i] = p_data[i];
    }

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Read flash
 *
 * @param address flash address
 * @param p_data buffer data
 * @param size length of buffer data
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_flash_read(uint32_t address, uint8_t *p_data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        p_data[i] = flash_memory[address + i];
    }

    return wHalErrorCode::NO_ERROR;
}