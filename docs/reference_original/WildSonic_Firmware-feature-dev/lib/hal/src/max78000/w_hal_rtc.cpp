#include "w_hal_rtc.hpp"
#include "w_hal_error_code.hpp"
#include "w_hal_rtc_internal.hpp"
#include "w_hal_system.hpp"

#include "trace.hpp"

#include "rtc.h"

#include <time.h>

/***** Definitions *****/

static const uint32_t SSEC_ALARM_TICKS_PER_SEC = 4096;
static const uint32_t ALARMS_PER_SECOND        = 512;
static_assert(
    (SSEC_ALARM_TICKS_PER_SEC % ALARMS_PER_SECOND) == 0,
    "SSEC_ALARM_TICKS_PER_SEC must be divisible by ALARMS_PER_SECOND");

static const uint32_t SUBSECOND_NSEC_0 =
    NANOSECONDS_PER_SECOND / ALARMS_PER_SECOND;

// Trim value for RTC. Parts per million (PPM) to adjust the RTC frequency.
// The value is in the range of -127 to 127.
static const int8_t       TRIM_ERROR_PPM        = -104; // RTC Oscillator
                                                        // is slower
                                                        // than expected.

// pointer to time variable to update
static w_hal_rtc_time_st *p_current_time        = nullptr;
static uint32_t *         p_current_time_update = nullptr;

#if __CORE__ == RTC_CORE

/**
 * @brief
 * Real-time clock Interrupt handler
 *
 */
extern "C" void __attribute__((interrupt("machine"))) RTC_IRQHandler(void)
{
    int flags = MXC_RTC_GetFlags();

    /* Check sub-second alarm flag. */
    if (flags & MXC_F_RTC_CTRL_SSEC_ALARM)
    {
        MXC_RTC_ClearFlags(MXC_F_RTC_CTRL_SSEC_ALARM);

        p_current_time->time_nanoseconds += SUBSECOND_NSEC_0;
        if (p_current_time->time_nanoseconds >= NANOSECONDS_PER_SECOND)
        {
            p_current_time->time_seconds++;
            p_current_time->time_nanoseconds -= NANOSECONDS_PER_SECOND;
        }
    }

    NVIC_ClearPendingIRQ(RTC_IRQn);
#ifdef __riscv
    NVIC_ClearPendingEVENT(RTC_IRQn);
#endif
}
#endif // __CORE__ == RTC_CORE

/**
 * @brief
 * RTC initialization
 *
 * @param p_time time variable to update (always in UTC)
 * @param p_time_update flag to indicate time updated
 * @param time_zone_offset time zone offset (for translation to local time)
 * @return wHalErrorCode error code if failure, NO_ERROR if successful
 */
wHalErrorCode w_hal_rtc_init(w_hal_rtc_time_st *p_time,
                             uint32_t *         p_time_update,
                             int                time_zone_offset)
{
    // Save pointer
    p_current_time        = p_time;
    p_current_time_update = p_time_update;

#if __CORE__ == RTC_CORE
    // stop all interrupts while setting up RTC
    __disable_irq();

    wHalErrorCode ext_rtc_error_code =
        w_hal_rtc_init_peripheral(*p_current_time);
    if (ext_rtc_error_code != wHalErrorCode::NO_ERROR &&
        ext_rtc_error_code != wHalErrorCode::RTC_TIME_UPDATED)
    {
        LOG_MSG_WITH_ARGS(
            TraceFile::W_HAL_RTC_CPP,
            "Failed RTC peripheral initialization with errorcode: %d\n",
            static_cast<int>(ext_rtc_error_code));
        return ext_rtc_error_code;
    }

    *p_current_time_update           = 0;

    wHalErrorCode int_rtc_error_code = wHalErrorCode::RTC_INITIALISATION_FAILED;
    uint32_t      retry;
    for (retry = 0; retry < 3; retry++)
    {
        if (MXC_RTC_Init(0, 0) != E_NO_ERROR)
        {
            LOG_MSG(TraceFile::W_HAL_RTC_CPP, "Failed RTC Initialization\n");

            int_rtc_error_code = wHalErrorCode::RTC_INITIALISATION_FAILED;
            continue;
        }

        // Trim the RTC
        if (MXC_RTC_Trim(TRIM_ERROR_PPM) != E_NO_ERROR)
        {
            LOG_MSG(TraceFile::W_HAL_RTC_CPP, "Failed RTC_Trim\n");

            int_rtc_error_code = wHalErrorCode::RTC_INITIALISATION_FAILED;
            continue;
        }

        // Disable alarm before setting it up
        if (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_SSEC_ALARM_IE) == E_BUSY)
        {
            int_rtc_error_code = wHalErrorCode::BUSY;
            continue;
        }

        // Converts a time to the equivalent register value.
        const uint32_t start_value = static_cast<uint32_t>(
            0x100000000ULL - (SSEC_ALARM_TICKS_PER_SEC / ALARMS_PER_SECOND));
        if (MXC_RTC_SetSubsecondAlarm(start_value) != E_NO_ERROR)
        {
            LOG_MSG(TraceFile::W_HAL_RTC_CPP, "Failed RTC_SetSubsecondAlarm\n");

            int_rtc_error_code = wHalErrorCode::RTC_INITIALISATION_FAILED;
            continue;
        }

        // Re-enable the alarm
        if (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_SSEC_ALARM_IE) == E_BUSY)
        {
            int_rtc_error_code = wHalErrorCode::BUSY;
            continue;
        }

        // Enable RTC interrupt and resume all interrupts
        NVIC_EnableIRQ(RTC_IRQn);
#ifdef __riscv
        NVIC_EnableEVENT(RTC_IRQn);
#endif
        __enable_irq();

        if (MXC_RTC_Start() != E_NO_ERROR)
        {
            LOG_MSG(TraceFile::W_HAL_RTC_CPP, "Failed RTC_Start\n");

            int_rtc_error_code = wHalErrorCode::RTC_INITIALISATION_FAILED;
            continue;
        }

        // Wait to ensure that the internal RTC is running
        w_hal_rtc_time_st last_time       = *p_current_time;

        bool              found_interrupt = false;
        const uint32_t    num_loops       = 100;
        const uint32_t    delay_ms        = 2;
        const uint32_t    total_delay_ms  = num_loops * delay_ms;
        for (uint32_t i = 0; i < num_loops; i++)
        {
            w_hal_system_delay_ms(delay_ms);

            // Get the time from the RTC
            if (last_time != *p_current_time)
            {
                found_interrupt = true;
                break;
            }
        }

        if (found_interrupt)
        {
            int_rtc_error_code = wHalErrorCode::NO_ERROR;
            break;
        }

        // No interrupt occurred in the last waiting period,
        // which means RTC is not running
        LOG_MSG(TraceFile::W_HAL_RTC_CPP,
                "RTC is not running after initialisation. Repeating "
                "initialisation...\n");

        // Since the RTC is not running, update the time to reflect the delay
        // in waiting for the RTC to spawn an interrupt.
        w_hal_rtc_time_delta_st time_delta;
        time_delta.seconds     = total_delay_ms / MILLISECONDS_PER_SECOND;
        time_delta.nanoseconds = (total_delay_ms % MILLISECONDS_PER_SECOND) *
                                 NANOSECONDS_PER_MILLISECOND;
        *p_current_time += time_delta;

        // Retry RTC initialization
        __disable_irq();
        while (w_hal_rtc_teardown() != wHalErrorCode::NO_ERROR)
        {
            // Wait for the RTC to be torn down
        }

        int_rtc_error_code = wHalErrorCode::RTC_INITIALISATION_FAILED;
    }

    if (int_rtc_error_code != wHalErrorCode::NO_ERROR)
    {
        return int_rtc_error_code;
    }

    if (retry > 0)
    {
        LOG_MSG_WITH_ARGS(TraceFile::W_HAL_RTC_CPP,
                          "RTC initialization successful after %d retries\n",
                          retry);
    }

    if (ext_rtc_error_code == wHalErrorCode::RTC_TIME_UPDATED)
    {
        LOG_MSG(TraceFile::W_HAL_RTC_CPP, "Using time from External RTC\n");
    }

    tm time_struct = {0};
    gmtime_r(&p_current_time->time_seconds, &time_struct);
    uint32_t milliseconds =
        p_current_time->time_nanoseconds / NANOSECONDS_PER_MILLISECOND;
    LOG_MSG_WITH_ARGS(TraceFile::W_HAL_RTC_CPP,
                      "RTC Time UTC: %04d-%02d-%02d %02d:%02d:%02d.%03d\n",
                      time_struct.tm_year + TIME_BASE_YEAR,
                      time_struct.tm_mon + TIME_BASE_MONTH,
                      time_struct.tm_mday,
                      time_struct.tm_hour,
                      time_struct.tm_min,
                      time_struct.tm_sec,
                      milliseconds);

    w_hal_rtc_time_st local_time = *p_current_time;
    local_time.time_seconds += time_zone_offset;

    char     time_zone_sign;
    uint16_t tz_hour;
    uint16_t tz_min;
    w_hal_rtc_get_time_zone_components(time_zone_offset,
                                       time_zone_sign,
                                       tz_hour,
                                       tz_min);

    gmtime_r(&local_time.time_seconds, &time_struct);
    milliseconds = local_time.time_nanoseconds / NANOSECONDS_PER_MILLISECOND;
    LOG_MSG_WITH_ARGS(
        TraceFile::W_HAL_RTC_CPP,
        "RTC Time Local: %04d-%02d-%02d %02d:%02d:%02d.%03d %c%02d:%02d\n",
        time_struct.tm_year + TIME_BASE_YEAR,
        time_struct.tm_mon + TIME_BASE_MONTH,
        time_struct.tm_mday,
        time_struct.tm_hour,
        time_struct.tm_min,
        time_struct.tm_sec,
        milliseconds,
        time_zone_sign,
        tz_hour,
        tz_min);
#endif // __CORE__ == RTC_CORE

    return wHalErrorCode::NO_ERROR;
}

/**
 * @brief RTC post initialization
 *
 * @return wHalErrorCode NO_ERROR if successful, error code otherwise
 */
wHalErrorCode w_hal_rtc_post_init()
{
#if __CORE__ == RTC_CORE
    // If the time has been updated, update the RTC time on the RTC core
    if (*p_current_time_update)
    {
        // Clear the flag to indicate that the time has been updated in the RTC
        // core
        *p_current_time_update = 0;
        return w_hal_rtc_peripheral_set_time(*p_current_time);
    }
    else
#endif // __CORE__ == RTC_CORE
    {
        return wHalErrorCode::NO_ERROR;
    }
}

/**
 * @brief
 * Sets base time. This will reset timer count as well
 *
 * @param time base time to set
 * @return wHalErrorCode BUSY if could not set up time, NO_ERROR otherwise
 */
wHalErrorCode w_hal_rtc_set_time(w_hal_rtc_time_st &time)
{
    wHalErrorCode error = wHalErrorCode::NO_ERROR;

    // Disable alarm before setting it up
    if (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_SSEC_ALARM_IE) == E_BUSY)
    {
        return wHalErrorCode::BUSY;
    }

    // stop all interrupts
    __disable_irq();

#if __CORE__ != RTC_CORE
    // Wait for interrupts to clear on RTC core
    w_hal_system_delay_ms(20);
#endif

    *p_current_time = time;

#if __CORE__ == RTC_CORE
    // Set the time on the RTC core
    *p_current_time_update = 0;
    error                  = w_hal_rtc_peripheral_set_time(*p_current_time);
#else
    // Set a flag to indicate that the time has been updated
    // This will be used to update the RTC time on the RTC core
    *p_current_time_update = 1;
#endif

    // Re-enable interrupts
    __enable_irq();

    // Re-enable the alarm
    if (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_SSEC_ALARM_IE) == E_BUSY)
    {
        return wHalErrorCode::BUSY;
    }

    return error;
}

/**
 * @brief Get the time from the system
 */
time_t w_hal_rtc_get_time()
{
    if (p_current_time == nullptr)
    {
        return static_cast<time_t>(0);
    }

    return *p_current_time;
}

/**
 * @brief Get the time in nanoseconds from the system
 *
 * @param time [out]: time structure to populate
 */
void w_hal_rtc_get_time(w_hal_rtc_time_st &time)
{
    if (p_current_time == nullptr)
    {
        time.time_seconds     = 0;
        time.time_nanoseconds = 0;
        return;
    }

    time.time_seconds     = p_current_time->time_seconds;
    time.time_nanoseconds = p_current_time->time_nanoseconds;
}

/**
 * @brief Teardown of the RTC
 *
 * @return wHalErrorCode RTC_TERMINATION_FAILED if teardown cannot be completed,
 * BUSY if busy, NO_ERROR otherwise
 */
wHalErrorCode w_hal_rtc_teardown()
{
#if __CORE__ == RTC_CORE

    if (MXC_RTC_Stop() != E_NO_ERROR)
    {
        return wHalErrorCode::RTC_TERMINATION_FAILED;
    }

#ifdef __riscv
    NVIC_DisableEVENT(RTC_IRQn);
#endif
    NVIC_DisableIRQ(RTC_IRQn);

    // Disable alarm
    if (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_SSEC_ALARM_IE) == E_BUSY)
    {
        return wHalErrorCode::BUSY;
    }

#endif // __CORE__ == RTC_CORE

    return wHalErrorCode::NO_ERROR;
}