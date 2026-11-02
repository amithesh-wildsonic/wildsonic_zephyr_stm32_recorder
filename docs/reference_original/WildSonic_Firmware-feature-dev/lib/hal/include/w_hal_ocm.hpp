#ifndef LIB_HAL_W_HAL_OCM_HPP
#define LIB_HAL_W_HAL_OCM_HPP

#include "device_config.hpp"

#include "w_hal_error_code.hpp"

/**
 * @brief : Validate the OCM Configuration
 *
 * @param device_config device configuration
 * @param fix : fix the configuration if possible
 * @return wHalErrorCode : return enum type error code value
 */
wHalErrorCode
w_hal_ocm_validate_config(DeviceConfig::config_page_st &device_config,
                          bool                          fix);

/**
 * @brief Off-chip memory initialization
 *
 * @param device_config device configuration
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_ocm_init(DeviceConfig::config_page_st &device_config);

/**
 * @brief Get off-chip memory size
 *
 * @return uint32_t size of memory
 */
uint32_t      w_hal_ocm_get_size();

/**
 * @brief Handler for processing current OCM requests
 *
 * @return wHalErrorCode BUSY if ocm request outstanding, NO_ERROR otherwise
 */
wHalErrorCode w_hal_ocm_handler();

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
w_hal_ocm_read_data(uint32_t address, void *p_buffer, uint32_t size);

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
w_hal_ocm_write_data(uint32_t address, void *p_buffer, uint32_t size);

/**
 * @brief Check if ocm request is done
 *
 * @return wHalErrorCode OCM_COMMAND_DONE if done, BUSY otherwise
 */
wHalErrorCode w_hal_ocm_check_status();

/**
 * @brief Clear completion status after ocm request is done
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode w_hal_ocm_clear_completion_status();

/**
 * @brief Teardown the OCM
 *
 * @return wHalErrorCode SPI_SHUTDOWN_FAILED if teardown cannot be complete,
 * NO_ERROR otherwise
 */
wHalErrorCode w_hal_ocm_teardown();

#endif // LIB_HAL_W_HAL_OCM_HPP