#ifndef LIB_HAL_W_HAL_GSM_INTERNAL_HPP
#define LIB_HAL_W_HAL_GSM_INTERNAL_HPP

#include "w_hal_common.hpp"

// Only maximum one of these macros should be set to 1
#define SIMCOM             (0)
#define MIKROE             (1)

#define USE_RING_INDICATOR ((SIMCOM) || (MIKROE))

extern const uint8_t GSM_UART_CHANNEL;

/**
 * @brief Reset the gsm module
 */
void                 w_hal_gsm_reset();

/**
 * @brief Voltage will be selected based on the board used
 *
 * @param voltage_select whal_voltage_select[out]: Parameter to select voltage
 */
void                 w_hal_gsm_get_voltage(whal_voltage_select &voltage_select);

/**
 * @brief Enter to sleep mode
 *
 * @param sleep sleep mode setting flag
 */
void                 w_hal_gsm_enter_sleep_mode(bool sleep);

/**
 * @brief To check if it received an ring or not
 *
 * @return true if SMS received
 * @return false if SMS not received
 */
bool                 w_hal_gsm_has_ring();

#endif // LIB_HAL_W_HAL_GSM_INTERNAL_HPP