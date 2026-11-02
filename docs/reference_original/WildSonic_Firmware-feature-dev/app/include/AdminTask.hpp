#ifndef _ADMIN_TASK_HPP
#define _ADMIN_TASK_HPP

#include "Task.hpp"
#include <time.h>

#include "w_hal_rtc.hpp"

/**
 * @brief
 * Task responsible for all system level operations.
 * This task should never sleep.
 * E.g. handling interrupts, waking respective threads,
 * stack overflow checking, etc.
 */
class AdminTask :
    public Task
{
  public:
    AdminTask() = default;

    // Initialization of member variables
    void          initialize();

    TaskErrorCode run() override;

  private:
#if __CORE__ == PM_CORE
    static constexpr uint32_t LOW_BATT_NUM_SEQ_READINGS = 20; // Number of
                                                              // consecutive
                                                              // readings to
                                                              // consider the
                                                              // battery low

    bool                      m_first_run;                    // Flag to
                                                              // check
                                                              // whether
                                                              // it is
                                                              // first run
                                                              // loop or not
    w_hal_rtc_time_st         m_last_power_info_logged_time;  // Last time
                                                              // power
                                                              // management
                                                              // was queried
                                                              // for power
                                                              // parameters
    w_hal_rtc_time_st m_low_battery_power_led_toggle_time;    // Last time the
                                                              // power LED was
                                                              // toggled
    uint32_t          m_num_consecutive_low_battery_readings; // Number of
                                                              // consecutive
                                                              // low battery
                                                              // readings
#endif
#if __CORE__ == BLE_CORE
    bool m_ble_pair_triggered_this_hold;
#endif
};

#endif // _ADMIN_TASK_HPP