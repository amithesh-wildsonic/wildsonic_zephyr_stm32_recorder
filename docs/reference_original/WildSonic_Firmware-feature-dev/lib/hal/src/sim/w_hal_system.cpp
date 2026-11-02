#include "w_hal_system.hpp"
#include "w_hal_rtc.hpp"

#include <string.h>

/**
 * @brief Jump to new entry point
 *
 * @param entry_point address of new entry point
 * @return wHalErrorCode error code if failure, NO_ERROR if
 * successful
 */
wHalErrorCode w_hal_system_jump_to_new_entry_point(uint32_t entry_point)
{
    return wHalErrorCode::NOT_IMPLEMENTED;
}

/**
 * @brief Reset the system
 *
 * @return wHalErrorCode wHalErrorCode error code if failure, NO_ERROR if
 * successful
 */
wHalErrorCode w_hal_system_reset()
{
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
    return w_hal_rtc_init(p_time,
                          p_time_update,
                          device_config.time_zone_offset);
}

/**
 * @brief
 * Second-stage general system-wide initialization
 *
 * @return wHalErrorCode error code if failure, NO_ERROR if successful
 */
wHalErrorCode w_hal_system_second_init()
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief
 * Finish general system-wide initialization
 *
 * @return wHalErrorCode error code if failure, NO_ERROR if successful
 */
wHalErrorCode w_hal_system_last_init()
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief
 * Check integrity of the stack memory guard region
 */
void w_hal_system_check_stack_guard()
{
}

/**
 * @brief Attempts to get the semaphore
 *
 * @param index index of semaphore
 * @return wHalErrorCode NO_ERROR if successful, BUSY otherwise
 */
wHalErrorCode w_hal_system_get_semaphore(uint16_t index)
{
    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief Free the semaphore
 *
 * @param index index of semaphore
 */
void w_hal_system_free_semaphore(uint16_t index)
{
}

/**
 * @brief Set error led
 *
 * @param on true if on, false if off
 */
void w_hal_system_set_error_led(bool on)
{
}

/**
 * @brief Set activity led
 *
 * @param on true if on, false if off
 */
void w_hal_system_set_activity_led(bool on)
{
}

/**
 * @brief Invert the activity led
 */
void w_hal_system_toggle_activity_led()
{
}

/**
 * @brief Set power led
 *
 * @param on true if on, false if off
 */
void w_hal_system_set_power_led(bool on)
{
}

/**
 * @brief Invert the power led
 */
void w_hal_system_toggle_power_led()
{
}

/**
 * @brief Get the last time the led button was pressed
 *
 * @return time_t last time the led button was pressed
 */
time_t w_hal_system_get_last_led_button_press_time()
{
    return 0;
}

/**
 * @brief Put the LEDs to sleep. LEDs will be waken up if the button is pressed.
 */
void w_hal_system_led_sleep()
{
}

/**
 * @brief Wake up the LEDs. Last button press time is updated when the LED
 * status button is pressed. Thereafter, the app layer will call this function
 * to wake up the LEDs.
 */
void w_hal_system_led_wake()
{
}

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
    strncpy(device_id, "SIM", device_id_size);
    strncpy(device_unique_id, "1234", device_unique_id_size);
}

/**
 * @brief Teardown the system
 *
 */
void w_hal_system_teardown()
{
}