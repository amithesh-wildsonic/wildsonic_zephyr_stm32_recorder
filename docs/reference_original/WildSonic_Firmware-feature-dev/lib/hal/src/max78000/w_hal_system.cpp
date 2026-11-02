#include "w_hal_system.hpp"
#include "w_hal_error_code.hpp"
#include "w_hal_pm.hpp"
#include "w_hal_rtc.hpp"

#include "assert.hpp"
#include "trace.hpp"

#include "board.h"
#include "dma.h"
#include "fcr_regs.h"
#include "gcr_regs.h"
#include "icc.h"
#include "led.h"
#include "mxc_assert.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#if BOARD_REVISION >= 5
#include "pb.h"
#endif
#include "sema.h"

#include <cstring>
#include <time.h>

extern volatile void const *__FlashStart_; // Defined in linker file
extern volatile void const *__StackLimit;  // Defined in linker file

static const uint32_t       STACK_GUARD_VALUES[] = {0xDEADFEED,
                                              0x01234567,
                                              0x89ABCEDE,
                                              0xAABBCCDD,
                                              0xEEFF0011,
                                              0x22334455,
                                              0x66778899,
                                              0xFEEDDEAD};
#if __CORE__ == CORE_0

// Keep track of the state of the LEDs set by the app layer even when the LEDs
// are asleep.
static bool activity_led_on;
static bool power_led_on;

#if BOARD_REVISION >= 5
static bool   led_asleep; // Flag to indicate if LEDs are asleep
static time_t led_status_button_pressed_time; // Time when button was pressed
#endif

// Internal functions
static void set_activity_led(bool on);
static void set_power_led(bool on);

extern "C" void __attribute__((interrupt("machine"))) DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
}

extern "C" void __attribute__((interrupt("machine"))) DMA1_IRQHandler(void)
{
    MXC_DMA_Handler();
}

extern "C" void __attribute__((interrupt("machine"))) DMA2_IRQHandler(void)
{
    MXC_DMA_Handler();
}

extern "C" void __attribute__((interrupt("machine"))) DMA3_IRQHandler(void)
{
    MXC_DMA_Handler();
}

#if BOARD_REVISION >= 5
/**
 * @brief Callback function for push button
 *
 * @param callback_data Callback data
 */
static void led_status_push_button_callback(void *callback_data)
{
    led_status_button_pressed_time = w_hal_rtc_get_time();
}
#endif // BOARD_REVISION >= 5

#endif // __CORE__ == CORE_0

#if __arm__

/**
 * @brief Jump to new entry point
 *
 * @param entry_point address of new entry point
 * @return wHalErrorCode error code if failure, NO_ERROR if
 * successful
 */
wHalErrorCode w_hal_system_jump_to_new_entry_point(uint32_t entry_point)
{
#if __CORE__ == CORE_0
    // Disable clock on other core
    MXC_GCR->pclkdis1 |= MXC_F_GCR_PCLKDIS1_CPU1;
#endif // __CORE__ == CORE_0

    __disable_irq();

    __DSB();

    // Set the stack pointer to the value at the new entry point
    __set_MSP(*(reinterpret_cast<volatile uint32_t *>(entry_point)));

    __DSB();

    // Set the PC to the reset vector of the new entry point
    uint32_t reset_vector =
        *reinterpret_cast<volatile uint32_t *>(entry_point + 4);

    // Jump to the new entry point
    asm volatile("bx %0" ::"r"(reset_vector));

    // Should never reach here
    return wHalErrorCode::NO_ERROR;
}

#endif // __arm__

/**
 * @brief Reset the system
 *
 * @return wHalErrorCode wHalErrorCode error code if failure, NO_ERROR if
 * successful
 */
wHalErrorCode w_hal_system_reset()
{
#if __CORE__ == CORE_0
    // Disable RISC-V clock
    MXC_GCR->pclkdis1 |= MXC_F_GCR_PCLKDIS1_CPU1;
#endif
    __disable_irq();

    NVIC_SystemReset();

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief
 * General system-wide initialization
 *
 * @param device_config device configuration
 * @param p_time time variable to update
 * @param p_time_update flag to indicate time updated
 * @return wHalErrorCode error code if failure, NO_ERROR if successful
 */
wHalErrorCode w_hal_system_init(DeviceConfig::config_page_st &device_config,
                                w_hal_rtc_time_st *           p_time,
                                uint32_t *                    p_time_update)
{
#if __CORE__ == CORE_0
#if BOARD_REVISION >= 5
    led_asleep = false;
#endif // BOARD_REVISION >= 5
    activity_led_on = false;
    power_led_on    = false;

    w_hal_system_set_power_led(true);
#endif // __CORE__ == CORE_0

#if defined(BOARD_FTHR_REVA) && (__CORE__ == CORE_0)
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(MXC_DELAY_MSEC(200));
#endif

#if __CORE__ == CORE_0

    MXC_SEMA_Init();

    // Enable cache
    MXC_ICC_Enable(MXC_ICC0);

    // Zero initialize ICC0 instance
    MXC_GCR->memz |= MXC_F_GCR_MEMZ_ICC0;
    while (MXC_GCR->memz & MXC_F_GCR_MEMZ_ICC0)
    {
    }

    // Don't enable IPO clock so that IBRO Clock at 60MHz can
    // divide down to 3Mhz for GSM
#endif // __CORE__ == CORE_0

#if __CORE__ == CORE_1
    // The assigned pins for RISCV debug overlap with UART2 pins
    // These need to be changed before enabling below line
#if ENABLE_RISCV_DEBUG
    Debug_Init(); // Function defined for RISC-V only
#endif

    // Before enabling ICC1 as a cache controller,
    // system sram3 should be zero initialized.
    MXC_GCR->memz |= MXC_F_GCR_MEMZ_RAM3;
    while (MXC_GCR->memz & MXC_F_GCR_MEMZ_RAM3)
    {
    }

    // Enable cache
    MXC_ICC_Enable(MXC_ICC1);

    // Zero initialize ICC1 instance
    MXC_GCR->memz |= MXC_F_GCR_MEMZ_ICC1;
    while (MXC_GCR->memz & MXC_F_GCR_MEMZ_ICC1)
    {
    }
#endif // __CORE__ == CORE_1

#if __CORE__ == CORE_0
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_EnableIRQ(DMA1_IRQn);
    NVIC_EnableIRQ(DMA2_IRQn);
    NVIC_EnableIRQ(DMA3_IRQn);
    for (uint32_t channel = 0; channel < MXC_DMA_CHANNELS; channel++)
    {
        MXC_DMA_ReleaseChannel(channel);
    }

    // Set up time from device config
    if ((device_config.program_time > p_time->time_seconds) &&
        (device_config.program_time != w_hal_rtc_time_st::INVALID_SECONDS))
    {
        *p_time = device_config.program_time;
    }
#endif

    // Enable all interrupts
    __enable_irq();

    wHalErrorCode error =
        w_hal_rtc_init(p_time, p_time_update, device_config.time_zone_offset);
    if (error != wHalErrorCode::NO_ERROR)
    {
        return error;
    }

    LOG_MSG(TraceFile::W_HAL_SYSTEM_CPP, "Basic System Init Done\n");

#if __CORE__ == PM_CORE
    w_hal_pm_init();
    w_hal_pm_print_info();
#endif // __CORE__ == PM_CORE

    // Initialize stack guard region to a known value
    // The stack guard is located at the bottom of the stack
    // and is 32 bytes long
    uint32_t *stack_guard =
        reinterpret_cast<uint32_t *>(const_cast<void **>(&__StackLimit));
    memcpy(stack_guard, STACK_GUARD_VALUES, sizeof(STACK_GUARD_VALUES));

    return error;
}

/**
 * @brief
 * Second-stage general system-wide initialization
 *
 * @return wHalErrorCode error code if failure, NO_ERROR if successful
 */
wHalErrorCode w_hal_system_second_init()
{
#if __CORE__ == CORE_0
    MXC_FCR->urvbootaddr = (uint32_t)&__FlashStart_; // Set RISC-V boot address
    MXC_GCR->pclkdis1 &= ~MXC_F_GCR_PCLKDIS1_CPU1;   // Enable RISC-V clock
#endif

    wHalErrorCode error = w_hal_rtc_post_init();

    LOG_MSG(TraceFile::W_HAL_SYSTEM_CPP, "2nd-stage System Init Done\n");

    return error;
}

/**
 * @brief
 * Finish general system-wide initialization
 *
 * @return wHalErrorCode error code if failure, NO_ERROR if successful
 */
wHalErrorCode w_hal_system_last_init()
{
#if (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)
    led_status_button_pressed_time = w_hal_rtc_get_time();

    // Initialize the push button
    int err                        = PB_Init();
    if (err != E_NO_ERROR)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_SYSTEM_CPP,
                          "Push Button Initialisation Failed with error %d\n",
                          err);
        return wHalErrorCode::PUSH_BUTTON_INITIALISATION_FAILED;
    }

    const unsigned int pb_index = 0;
    PB_RegisterCallback(pb_index, led_status_push_button_callback);
    PB_IntEnable(pb_index);
#endif // (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)

    LOG_MSG(TraceFile::W_HAL_SYSTEM_CPP, "Last System Init Done\n");

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief
 * Check integrity of the stack memory guard region
 */
void w_hal_system_check_stack_guard()
{
    // Disable warning for stringop overflow
    // This is a false positive as the stack guard is 32 bytes long
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
    uint32_t *stack_guard =
        reinterpret_cast<uint32_t *>(const_cast<void **>(&__StackLimit));
    int compare =
        memcmp(stack_guard, STACK_GUARD_VALUES, sizeof(STACK_GUARD_VALUES));
    SYSTEM_ASSERT_EQUAL(compare, 0);
#pragma GCC diagnostic pop
}

/**
 * @brief Delay for microseconds of time
 *
 * @param us number of microseconds to delay
 */
void w_hal_system_delay_us(uint32_t us)
{
    MXC_Delay(USEC(us));
}

/**
 * @brief Delay for milliseconds of time
 *
 * @param ms number of milliseconds to delay
 */
void w_hal_system_delay_ms(uint32_t ms)
{
    MXC_Delay(MSEC(ms));
}

/**
 * @brief Delay for seconds of time
 *
 * @param sec number of seconds to delay
 */
void w_hal_system_delay(uint32_t sec)
{
    MXC_Delay(SEC(sec));
}

/**
 * @brief Attempts to get the semaphore
 *
 * @param index index of semaphore
 * @return wHalErrorCode NO_ERROR if successful, BUSY otherwise
 */
wHalErrorCode w_hal_system_get_semaphore(uint16_t index)
{

    int retVal = MXC_SEMA_GetSema(index);

    if (retVal == E_NO_ERROR)
    {
        return wHalErrorCode::NO_ERROR;
    }
    else
    {
        return wHalErrorCode::BUSY;
    }
}

/**
 * @brief Free the semaphore
 *
 * @param index index of semaphore
 */
void w_hal_system_free_semaphore(uint16_t index)
{
    MXC_SEMA_FreeSema(index);
}

/**
 * @brief Get the last time the led button was pressed
 *
 * @return time_t last time the led button was pressed
 */
time_t w_hal_system_get_last_led_button_press_time()
{
#if (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)
    return led_status_button_pressed_time;
#else
    return (time_t)0;
#endif
}

#if (__CORE__ == BLE_CORE) && (BOARD_REVISION >= 5)
/**
 * @brief Check if the LED status button is currently being held down
 *        and has been held for at least the specified number of seconds.
 *
 * @param seconds minimum hold duration to check
 * @return true if button is physically pressed AND held >= seconds
 */
bool w_hal_system_is_button_held_for(uint32_t seconds)
{
    if (!PB_Get(0))
    {
        // Button is not currently pressed
        return false;
    }
    time_t now      = w_hal_rtc_get_time();
    time_t held_for = now - led_status_button_pressed_time;
    return (held_for >= static_cast<time_t>(seconds));
}
#endif // (__CORE__ == BLE_CORE) && (BOARD_REVISION >= 5)

/**
 * @brief Put the LEDs to sleep. LEDs will be waken up if the button is
 * pressed.
 */
void w_hal_system_led_sleep()
{
#if (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)
    if (!led_asleep)
    {
        led_asleep = true;

        set_activity_led(false);
        set_power_led(false);
    }
#endif // (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)
}

/**
 * @brief Wake up the LEDs. Last button press time is updated when the LED
 * status button is pressed. Thereafter, the app layer will call this function
 * to wake up the LEDs.
 */
void w_hal_system_led_wake()
{
#if (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)
    if (led_asleep)
    {
        led_asleep = false;

        if (activity_led_on)
        {
            set_activity_led(true);
        }

        if (power_led_on)
        {
            set_power_led(true);
        }
    }
#endif // (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)
}

/**
 * @brief Set error led
 *
 * @param on true if on, false if off
 */
void w_hal_system_set_error_led(bool on)
{
    if (on)
    {
        // Other led need to be off for red to show
        LED_On(LED_RED);
#ifdef BOARD_FTHR_REVA
        LED_Off(LED_BLUE);
        LED_Off(LED_GREEN);
#endif
    }
    else
    {
        LED_Off(LED_RED);
#ifdef BOARD_FTHR_REVA
        LED_On(LED_BLUE);
#endif
    }
}

#if __CORE__ == CORE_0

/**
 * @brief Set the activity led. Internal function to set the activity led.
 * This does not check software sleep state.
 *
 * @param on true if on, false if off
 */
static void set_activity_led(bool on)
{
    if (on)
    {
#ifdef BOARD_FTHR_REVA
        // Other led need to be off for green to show
        LED_Off(LED_BLUE);
#endif
        LED_On(LED_GREEN);
    }
    else
    {
#ifdef BOARD_FTHR_REVA
        LED_On(LED_BLUE);
#endif
        LED_Off(LED_GREEN);
    }
}

/**
 * @brief Set activity led
 *
 * @param on true if on, false if off
 */
void w_hal_system_set_activity_led(bool on)
{
    activity_led_on = on;

#if BOARD_REVISION >= 5
    if (led_asleep)
    {
        // LED is asleep, don't turn on unless button is pressed
        return;
    }
#endif // BOARD_REVISION >= 5

    set_activity_led(on);
}

/**
 * @brief Invert the activity led
 */
void w_hal_system_toggle_activity_led()
{
    if (activity_led_on)
    {
        w_hal_system_set_activity_led(false);
    }
    else
    {
        w_hal_system_set_activity_led(true);
    }
}

/**
 * @brief Set the power led. Internal function to set the power led.
 * This does not check software sleep state.
 */
static void set_power_led(bool on)
{
    if (on)
    {
        LED_On(LED_BLUE);
    }
    else
    {
        LED_Off(LED_BLUE);
    }
}

/**
 * @brief Set power led
 *
 * @param on true if on, false if off
 */
void w_hal_system_set_power_led(bool on)
{
    power_led_on = on;

#if BOARD_REVISION >= 5
    if (led_asleep)
    {
        // LED is asleep, don't turn on unless button is pressed
        return;
    }
#endif // BOARD_REVISION >= 5

    set_power_led(on);
}

/**
 * @brief Invert the power led
 */
void w_hal_system_toggle_power_led()
{
    if (power_led_on)
    {
        w_hal_system_set_power_led(false);
    }
    else
    {
        w_hal_system_set_power_led(true);
    }
}

#endif // __CORE__ == CORE_0

/**
 * @brief Get the device id and device unique id as device info
 *
 * @param device_id[out]:The device id
 * @param device_id_size[in]:size of the device id
 * @param device_unique_id[out]:The device unique id
 * @param device_unique_id_size[in]:size of the device unique id
 */
void w_hal_system_get_device_info(char *  device_id,
                                  uint8_t device_id_size,
                                  char *  device_unique_id,
                                  uint8_t device_unique_id_size)
{
    const uint8_t max_byte_number = 13; // The total number of bytes
                                        // in unique serial number
    const uint8_t ascii_number =
        max_byte_number * 2; // The total number of bytes
                             // as a nibble in unique serial number
                             //  for ascii code
    uint8_t
        device_unique_id_buffer[max_byte_number]; // temporary variable for
                                                  // device_unique_serial_number

    // Updating device_usn
    strncpy(device_id, "MAX78000EXG+", device_id_size);

    if (device_unique_id_size > 0)
    {
        int copy_limit = ascii_number;
        if (copy_limit > static_cast<int>(device_unique_id_size) - 1)
        {
            copy_limit = device_unique_id_size - 1;
        }
        copy_limit /= 2;

        // Function to get device unique serial number
        MXC_SYS_GetUSN(device_unique_id_buffer, NULL);
        // to store 13 byte of USN in device_unique_id buffer
        for (uint8_t index = 0; index < copy_limit; index++)
        {
            snprintf(&device_unique_id[index * 2],
                     ascii_number - index * 2,
                     "%02X",
                     device_unique_id_buffer[index]);
        }
        device_unique_id[ascii_number] = '\0';
    }
}

/**
 * @brief Teardown the system
 *
 */
void w_hal_system_teardown()
{
#if (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)
    // Disable push button interrupt
    PB_IntDisable(0);
    PB_IntClear(0);
#endif // (__CORE__ == CORE_0) && (BOARD_REVISION >= 5)

#if __CORE__ == PM_CORE
    w_hal_pm_teardown();
#endif // __CORE__ == PM_CORE

    wHalErrorCode error = wHalErrorCode::BUSY;
    if (error == wHalErrorCode::BUSY)
    {
        error = w_hal_rtc_teardown();
    }

#if __CORE__ == CORE_0
    NVIC_DisableIRQ(DMA0_IRQn);
    NVIC_DisableIRQ(DMA1_IRQn);
    NVIC_DisableIRQ(DMA2_IRQn);
    NVIC_DisableIRQ(DMA3_IRQn);
#endif

#if __CORE__ == CORE_1
    // Disable cache
    MXC_ICC_Disable(MXC_ICC1);
#endif

#if __CORE__ == CORE_0
    // Disable cache
    MXC_ICC_Disable(MXC_ICC0);

    MXC_SEMA_Shutdown();
#endif

    // stop all interrupts
    __disable_irq();

#if __CORE__ == CORE_0
    w_hal_system_set_power_led(false);
    w_hal_system_set_error_led(false);
    w_hal_system_set_activity_led(false);
#endif
}