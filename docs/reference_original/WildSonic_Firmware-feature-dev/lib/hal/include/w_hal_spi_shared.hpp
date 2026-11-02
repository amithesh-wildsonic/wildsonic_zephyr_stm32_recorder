#ifndef LIB_HAL_W_HAL_SPI_SHARED_HPP
#define LIB_HAL_W_HAL_SPI_SHARED_HPP

#include "w_hal_error_code.hpp"

extern const uint32_t SPI_SRAM_FREQ;

/**
 * @brief Initialize spi shared variables
 *
 * @return wHalErrorCode NO_ERROR
 */
wHalErrorCode         w_hal_spi_shared_initialize();

/**
 * @brief Request spi baton for OCM use
 *
 * @return wHalErrorCode BUSY if baton cannot be awarded, NO_ERROR otherwise
 */
wHalErrorCode         w_hal_spi_shared_request_baton_for_ocm();

/**
 * @brief Request spi baton for SD card use
 *
 * @return wHalErrorCode BUSY if baton cannot be awarded, NO_ERROR otherwise
 */
wHalErrorCode         w_hal_spi_shared_request_baton_for_sd_card();

/**
 * @brief Release spi baton after OCM use
 *
 * @return wHalErrorCode BUSY if baton cannot be released or is not in
 * possession, NO_ERROR otherwise
 */
wHalErrorCode         w_hal_spi_shared_release_baton_for_ocm();

/**
 * @brief Release spi baton after sd card use
 *
 * @return wHalErrorCode BUSY if baton cannot be released or is not in
 * possession, NO_ERROR otherwise
 */
wHalErrorCode         w_hal_spi_shared_release_baton_for_sd_card();

#endif // LIB_HAL_W_HAL_SPI_SHARED_HPP