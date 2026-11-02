#if __CORE__ == COMM_CORE

#include "w_hal_gsm.hpp"
#include "w_hal_gsm_internal.hpp"
#include "w_hal_uart.hpp"

#include "trace.hpp"

#include "board.h"
#include "gpio.h"
#include "max78000.h"
#include "mxc_delay.h"
#include "mxc_pins.h"

#define ENABLE_GSM_RESET_SEQUENCE   (1)

// This options is used for WS LARA Board V1 Rev1 only, which does not have a
// pmos to pull down the reset pin.
#define RESET_SEQUENCE_LOW_HIGH_LOW (0)

// This option is for SIM800L module
#define SIMCOM_RESET_SEQUENCE       (SIMCOM)

#define VDDIO_3V3                   ((SIMCOM) || (MIKROE))

/* This file contains gsm code that references the MAX78000 platform */

#if defined(BOARD_EVKIT_V1)
const uint8_t GSM_UART_CHANNEL = 1;
#elif defined(BOARD_FTHR_REVA)
const uint8_t               GSM_UART_CHANNEL     = 2;
#elif defined(BOARD_WS_Rev1)
const uint8_t GSM_UART_CHANNEL = 0;
#else
#error "Unknown Board: Only support MAX78000 Ev Kit, FTHR board, or WS SOC card"
#endif

#if defined(BOARD_WS_Rev1)
// Use P1_7 for GSM Power PIN
static const mxc_gpio_cfg_t gsm_power_pin_config = {MXC_GPIO1,
                                                    MXC_GPIO_PIN_7,
                                                    MXC_GPIO_FUNC_OUT,
                                                    MXC_GPIO_PAD_NONE,
#if VDDIO_3V3
                                                    MXC_GPIO_VSSEL_VDDIOH
#else
                                                    MXC_GPIO_VSSEL_VDDIO
#endif
};

// Use P1_9 for GSM Reset PIN
static const mxc_gpio_cfg_t gsm_reset_pin_config = {MXC_GPIO1,
                                                    MXC_GPIO_PIN_9,
                                                    MXC_GPIO_FUNC_OUT,
                                                    MXC_GPIO_PAD_NONE,
#if VDDIO_3V3
                                                    MXC_GPIO_VSSEL_VDDIOH
#else
                                                    MXC_GPIO_VSSEL_VDDIO
#endif
};
#elif defined(BOARD_FTHR_REVA)
// Use P1_6 for GSM Reset PIN
static const mxc_gpio_cfg_t gsm_power_pin_config = {MXC_GPIO1,
                                                    MXC_GPIO_PIN_6,
                                                    MXC_GPIO_FUNC_OUT,
                                                    MXC_GPIO_PAD_NONE,
#if VDDIO_3V3
                                                    MXC_GPIO_VSSEL_VDDIOH
#else
                                                    MXC_GPIO_VSSEL_VDDIO
#endif
};
#endif

#if USE_RING_INDICATOR
static const mxc_gpio_cfg_t gsm_interrupt_pin_config = {MXC_GPIO1,
                                                        MXC_GPIO_PIN_8,
                                                        MXC_GPIO_FUNC_IN,
                                                        MXC_GPIO_PAD_PULL_UP,
#if VDDIO_3V3
                                                        MXC_GPIO_VSSEL_VDDIOH
#else
                                                        MXC_GPIO_VSSEL_VDDIO
#endif
};

// interrupt flag for gsm wakeup
static bool gsm_ring_received;

// ISR function for GSM interrupt
void        gsm_ring_isr(void *cbdata)
{
    gsm_ring_received = true;
}
#endif // USE_RING_INDICATOR

/**
 * @brief Reset the ublox module
 */
void w_hal_gsm_reset()
{
#if USE_RING_INDICATOR
    gsm_ring_received = false;
    // GPIO configuration for ring interrupt
    MXC_GPIO_Config(&gsm_interrupt_pin_config);
    MXC_GPIO_RegisterCallback(&gsm_interrupt_pin_config,
                              gsm_ring_isr,
                              &gsm_ring_received);
    MXC_GPIO_IntConfig(&gsm_interrupt_pin_config, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(gsm_interrupt_pin_config.port,
                       gsm_interrupt_pin_config.mask);
    NVIC_EnableIRQ(
        MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(gsm_interrupt_pin_config.port)));
#endif // USE_RING_INDICATOR

#if defined(BOARD_FTHR_REVA)
    // UART2 does not support HW flow control properly so manually drive RTS low
    w_hal_uart_assert_rts(GSM_UART_CHANNEL);
#endif

    MXC_GPIO_Config(&gsm_power_pin_config);
#if ENABLE_GSM_RESET_SEQUENCE
    LOG_MSG(TraceFile::W_HAL_GSM_MAXIM_CPP, "Resetting GSM...\n");

#if SIMCOM_RESET_SEQUENCE
    MXC_GPIO_OutSet(gsm_power_pin_config.port, gsm_power_pin_config.mask);
    MXC_Delay(MSEC(5000));
    MXC_GPIO_OutClr(gsm_power_pin_config.port, gsm_power_pin_config.mask);
#elif RESET_SEQUENCE_LOW_HIGH_LOW
    MXC_GPIO_OutClr(gsm_power_pin_config.port, gsm_power_pin_config.mask);
    MXC_Delay(MSEC(1));
    MXC_GPIO_OutSet(gsm_power_pin_config.port, gsm_power_pin_config.mask);
    MXC_Delay(MSEC(1));
    MXC_GPIO_OutClr(gsm_power_pin_config.port, gsm_power_pin_config.mask);
#else
    MXC_GPIO_OutSet(gsm_power_pin_config.port, gsm_power_pin_config.mask);
    MXC_Delay(MSEC(1));
    MXC_GPIO_OutClr(gsm_power_pin_config.port, gsm_power_pin_config.mask);
    MXC_Delay(MSEC(1));
    MXC_GPIO_OutSet(gsm_power_pin_config.port, gsm_power_pin_config.mask);
#endif

    LOG_MSG(TraceFile::W_HAL_GSM_MAXIM_CPP, "Reset GSM Done!\n");
#else
    // Turn on GSM
    MXC_GPIO_OutClr(gsm_power_pin_config.port, gsm_power_pin_config.mask);
#endif
}

/**
 * @brief Voltage will be selected based on the board used
 *
 * @param voltage_select whal_voltage_select[out]: Parameter to select voltage
 */
void w_hal_gsm_get_voltage(whal_voltage_select &voltage_select)
{
#if VDDIO_3V3
    voltage_select = whal_voltage_select::VOLTAGE_3V3;
#else
    voltage_select = whal_voltage_select::VOLTAGE_1V8;
#endif
}

/**
 * @brief Enter to sleep mode
 *
 * @param sleep sleep mode setting flag
 */
void w_hal_gsm_enter_sleep_mode(bool sleep)
{
#if USE_RING_INDICATOR

#if (SIMCOM)
    if (sleep)
    {
        MXC_GPIO_OutSet(gsm_power_pin_config.port, gsm_power_pin_config.mask);
    }
    else
    {
        MXC_GPIO_OutClr(gsm_power_pin_config.port, gsm_power_pin_config.mask);
    }
#endif // SIMCOM

#endif // USE_RING_INDICATOR
}

/**
 * @brief To check if it received an ring or not
 *
 * @return true if SMS received
 * @return false if SMS not received
 */
bool w_hal_gsm_has_ring()
{
    bool ring = false;
#if USE_RING_INDICATOR
    __disable_irq();

    ring              = gsm_ring_received;
    gsm_ring_received = false;

    __enable_irq();
#endif

    // return the status of the interrupt
    return ring;
}

#endif // __CORE__ == COMM_CORE