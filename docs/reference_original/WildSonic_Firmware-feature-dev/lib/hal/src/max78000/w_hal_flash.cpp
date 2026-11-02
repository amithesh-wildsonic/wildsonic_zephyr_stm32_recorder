#include "w_hal_flash.hpp"
#include "w_hal_error_code.hpp"

#include "trace.hpp"

#include "flc.h"

/**
 * @brief Initialize flash
 *
 * @return wHalErrorCode FLASH_INITIALIZATION_FAILED if flash initialization
 * failed
 * @return wHalErrorCode NO_ERROR if flash initialization succeeded
 */
wHalErrorCode w_hal_flash_init(void)
{
    int error = MXC_FLC_Init();

    if (error != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_FLASH_CPP,
                          "MXC_FLC_Init() failed with error code %d\n",
                          error);

        return wHalErrorCode::FLASH_INITIALIZATION_FAILED;
    }
    else
    {
        return wHalErrorCode::NO_ERROR;
    }
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
    return MXC_FLASH_MEM_BASE;
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
    page_size   = MXC_FLASH_PAGE_SIZE;
    memory_size = MXC_FLASH_MEM_SIZE;

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
    int error = MXC_FLC_PageErase(MXC_FLASH_MEM_BASE + address);

    if (error != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_FLASH_CPP,
                          "MXC_FLC_PageErase() failed with error code %d\n",
                          error);

        return wHalErrorCode::FLASH_ERASE_FAILED;
    }
    else
    {
        return wHalErrorCode::NO_ERROR;
    }
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    uint32_t *p_data_32 = reinterpret_cast<uint32_t *>(p_data);
#pragma GCC diagnostic pop

    int error = MXC_FLC_Write(MXC_FLASH_MEM_BASE + address, size, p_data_32);

    if (error != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_FLASH_CPP,
                          "MXC_FLC_Write() failed with error code %d\n",
                          error);

        return wHalErrorCode::FLASH_WRITE_FAILED;
    }
    else
    {
        return wHalErrorCode::NO_ERROR;
    }
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
    MXC_FLC_Read(MXC_FLASH_MEM_BASE + address, p_data, size);

    return wHalErrorCode::NO_ERROR;
}