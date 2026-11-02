#include "AdminTask.hpp"
#include "AudioTask.hpp"

#include "BleComm.hpp"
#include "device_config.hpp"
#include "shared.hpp"
#include "trace.hpp"
#include "w_hal_pm.hpp"
#include "w_hal_rtc.hpp"
#include "w_hal_system.hpp"

/**
 * @brief
 * Initialize member variables
 */
void AdminTask::initialize()
{
    Task::initialize();
#if __CORE__ == PM_CORE
    m_first_run = true;
    m_last_power_info_logged_time.clear();
    m_low_battery_power_led_toggle_time.clear();
    m_num_consecutive_low_battery_readings = 0;
#elif (__CORE__ == CORE_0) && (PM_CORE == INVALID_CORE_NUM)
    shared_glbl.power_mgmt_info.voltage              = 3700;
    shared_glbl.power_mgmt_info.current              = 100;
    shared_glbl.power_mgmt_info.full_charge_capacity = 5000;
    shared_glbl.power_mgmt_info.remaining_capacity =
        shared_glbl.power_mgmt_info.full_charge_capacity;
    shared_glbl.power_mgmt_info.average_current =
        shared_glbl.power_mgmt_info.current;
    shared_glbl.power_mgmt_info.time_to_empty = INT32_MAX;
    shared_glbl.power_mgmt_info.average_power =
        shared_glbl.power_mgmt_info.average_current *
        shared_glbl.power_mgmt_info.voltage / 1000;
    shared_glbl.power_mgmt_info.temperature = 25;
#endif
#if __CORE__ == BLE_CORE
    m_ble_pair_triggered_this_hold = false;
#endif
}

/**
 * @brief
 * Admin task entry point function
 * @return TaskErrorCode NoError always
 */
TaskErrorCode AdminTask::run()
{
#if (__CORE__ == CORE_0) || (__CORE__ == PM_CORE)
    w_hal_rtc_time_st current_time;
#endif // (__CORE__ == CORE_0) || (__CORE__ == PM_CORE)

#if __CORE__ == CORE_0
    w_hal_rtc_get_time(current_time);

    // To save power and reduce light emitting from the
    // device, allow the LEDs to sleep unless the button
    // has been pressed in the last 60 seconds.
    const time_t last_led_button_press_time =
        w_hal_system_get_last_led_button_press_time();
    if (current_time - last_led_button_press_time >= 60)
    {
        w_hal_system_led_sleep();
    }
    else
    {
        w_hal_system_led_wake();
    }

#endif // __CORE__ == CORE_0
#if __CORE__ == BLE_CORE
#if !defined(BLE_CORE_CUSTOM)
    // Stock HM10 only: MAX78000 button controls BLE pairing window.
    // Custom HM10 handles its own button ISR on P1.3 internally.
    uint8_t const MAX_PRESS_DURATION = 5; // seconds
    // --- 5-second button hold: trigger BLE pairing window ---
    if (w_hal_system_is_button_held_for(MAX_PRESS_DURATION))
    {
        if (!m_ble_pair_triggered_this_hold)
        {
            m_ble_pair_triggered_this_hold = true;
            shared_glbl.ble_pairing_window_request_seconds =
                BLE_PAIR_WINDOW_DEFAULT_SECONDS; // 30 seconds
            shared_glbl.ble_pairing_window_request_pending = 1;
            LOG_MSG(TraceFile::ADMIN_TASK_CPP,
                    "5s button hold detected: BLE pairing window requested\n");
        }
    }
    else
    {
        // Button released or not held long enough — reset the latch
        // No partial request should linger if button was released early
        // (request is only valid if the full hold completed)
        m_ble_pair_triggered_this_hold = false;
    }
#endif // !defined(BLE_CORE_CUSTOM)
    process_ble_pending_commands();
    update_ble_status();
#endif // __CORE__ == BLE_CORE

#if __CORE__ == PM_CORE
    w_hal_pm_handler();
    w_hal_rtc_get_time(current_time);

    // To update the power management info for every 10 seconds
    w_hal_rtc_time_delta_st time_since_last_power_info_logged;
    current_time.subtract(m_last_power_info_logged_time,
                          time_since_last_power_info_logged);
    if (time_since_last_power_info_logged.seconds >= 10)
    {
        wHalErrorCode error = w_hal_pm_get_fuel_gauge_info(
            shared_glbl.power_mgmt_info.voltage,
            shared_glbl.power_mgmt_info.current,
            shared_glbl.power_mgmt_info.remaining_capacity,
            shared_glbl.power_mgmt_info.full_charge_capacity,
            shared_glbl.power_mgmt_info.average_current,
            shared_glbl.power_mgmt_info.time_to_empty,
            shared_glbl.power_mgmt_info.average_power,
            shared_glbl.power_mgmt_info.temperature);

        if (error != wHalErrorCode::NO_ERROR)
        {
            if (!m_first_run)
            {
                LOG_MSG_WITH_ARGS_NO_PRINT(
                    TraceFile::ADMIN_TASK_CPP,
                    "Power management info could not be read. Error code: %d\n",
                    error);
            }
        }
        // Checking whether the power is sufficient to run properly.
        // Check if the voltage value is valid before comparing it with the
        // threshold.
        else
        {
            if (!m_first_run && !shared_glbl.power_mgmt_info.low_battery_flag &&
                (shared_glbl.power_mgmt_info.voltage <
                 shared_glbl.device_config.low_battery_voltage_cutoff))
            {
                // Make sure the reading is consistent for a number of
                // consecutive readings before setting the flag.
                if (++m_num_consecutive_low_battery_readings >=
                    LOW_BATT_NUM_SEQ_READINGS)
                {
                    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_PM_CPP,
                                      "Battery too low to run.  Halted writing "
                                      "to SD card.\n");
                    shared_glbl.power_mgmt_info.low_battery_flag = true;
                    m_num_consecutive_low_battery_readings       = 0;
                }
            }
            else
            {
                m_num_consecutive_low_battery_readings = 0;
            }

            // If the power is greater than threshold the system will again
            // start recording
            if (shared_glbl.power_mgmt_info.low_battery_flag &&
                (shared_glbl.power_mgmt_info.voltage >=
                 static_cast<int>(LOW_BATTERY_RESTORE_VOLTAGE)))
            {
                LOG_MSG_WITH_ARGS(TraceFile::W_HAL_PM_CPP,
                                  "The battery power is restored. Resumed "
                                  "writing to SD Card\n");
                shared_glbl.power_mgmt_info.low_battery_flag = false;

                // During low battery, the power LED is periodically toggled.
                // When the battery charge is restored, the power LED should
                // be turned on and stay on.
                w_hal_system_set_power_led(true);
            }

            // To log the power management info for every 10 seconds
            LOG_MSG_WITH_ARGS_NO_PRINT(
                TraceFile::ADMIN_TASK_CPP,
                "Battery_voltage: %d, Battery_current: "
                "%d, Battery_remaining_capacity: %d, "
                "Battery_full_charge_capacity: "
                "%d, Battery_average_current: %d, Battery_time_to_empty: "
                "%d, Battery_average_power: %d, Battery_temperature: %d\n",
                shared_glbl.power_mgmt_info.voltage,
                shared_glbl.power_mgmt_info.current,
                shared_glbl.power_mgmt_info.remaining_capacity,
                shared_glbl.power_mgmt_info.full_charge_capacity,
                shared_glbl.power_mgmt_info.average_current,
                shared_glbl.power_mgmt_info.time_to_empty,
                shared_glbl.power_mgmt_info.average_power,
                shared_glbl.power_mgmt_info.temperature);
        }
        m_last_power_info_logged_time = current_time;

        // Clearing the first run loop flag
        m_first_run                   = false;
    }

    if (shared_glbl.power_mgmt_info.low_battery_flag)
    {
        // Toggle the power LED to indicate low battery
        const w_hal_rtc_time_delta_st low_battery_led_toggle_period(1, 0);
        w_hal_rtc_time_delta_st       time_since_last_toggle;
        current_time.subtract(m_low_battery_power_led_toggle_time,
                              time_since_last_toggle);
        if (time_since_last_toggle >= low_battery_led_toggle_period)
        {
            w_hal_system_toggle_power_led();
            m_low_battery_power_led_toggle_time = current_time;
        }
    }
#endif // __CORE__ == PM_CORE

    switch (shared_glbl.shutdown_stage)
    {
        case shutdown_stage_t::Stage_8:
        {
            // This is the final shutdown stage, executed after all tasks went
            // to sleep
#if __CORE__ == PM_CORE
            wHalErrorCode error = w_hal_pm_handler();
            if (error == wHalErrorCode::BUSY)
            {
                break;
            }
#endif // __CORE__ == PM_CORE
            return TaskErrorCode::Sleep;
        }
        default:
        {
            break;
        }
    }

    return TaskErrorCode::NoError;
}