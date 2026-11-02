#include "w_hal_error_code.hpp"

/**
 * @brief Initialize flash
 *
 * @return wHalErrorCode FLASH_INITIALIZATION_FAILED if flash initialization
 * failed
 * @return wHalErrorCode NO_ERROR if flash initialization succeeded
 */
wHalErrorCode w_hal_flash_init(void);

/**
 * @brief Teardown flash
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_flash_teardown(void);

/**
 * @brief Get cpu address of flash base
 *
 * @return uint32_t cpu address of flash base
 */
uint32_t      w_hal_flash_get_cpu_base_address();

/**
 * @brief Get flash size
 *
 * @param page_size page size
 * @param memory_size memory size
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_flash_get_parameters(uint32_t &page_size,
                                         uint32_t &memory_size);

/**
 * @brief Erase flash page
 *
 * @param address flash address
 * @return wHalErrorCode FLASH_ERASE_FAILED if flash erase failed
 * @return wHalErrorCode NO_ERROR if flash erase succeeded
 */
wHalErrorCode w_hal_flash_page_erase(uint32_t address);

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
w_hal_flash_program(uint32_t address, uint8_t *p_data, uint32_t size);

/**
 * @brief Read flash
 *
 * @param address flash address
 * @param p_data buffer data
 * @param size length of buffer data
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode
w_hal_flash_read(uint32_t address, uint8_t *p_data, uint32_t size);