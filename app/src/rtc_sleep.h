/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RTC_SLEEP_H_
#define RTC_SLEEP_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the STM32 RTC Wakeup Timer hardware.
 *
 * Configures backup domain access, selects LSE/LSI as RTC clock source,
 * configures EXTI Line 20, and enables RTC_WKUP_IRQn interrupt.
 *
 * @return 0 on success, negative errno on failure.
 */
int rtc_sleep_init(void);

/**
 * @brief Program the STM32 RTC Wakeup Timer for a specified duration.
 *
 * @param seconds Sleep duration in seconds (1 to 65535).
 * @return 0 on success, negative errno on failure.
 */
int rtc_sleep_arm(uint32_t seconds);

/**
 * @brief Disable and disarm the STM32 RTC Wakeup Timer.
 */
void rtc_sleep_disarm(void);

enum rtc_wake_source {
	RTC_WAKE_NONE = 0,
	RTC_WAKE_TIMER,
	RTC_WAKE_USER_BUTTON,
	RTC_WAKE_CONSOLE_ABORT,
};

/**
 * @brief Enter true STM32L476 Stop 2 low-power mode for specified duration.
 *
 * Configures RTC Wakeup Timer on 1 Hz ck_spre, configures User Button (B1 / PC13)
 * as EXTI Line 13 wakeup source, flushes UART, turns off user LED (PA5),
 * suspends SysTick, and executes __WFI() into Stop 2 mode.
 * Upon wakeup (via RTC timer or B1 button), clears deep-sleep, restores SysTick,
 * re-initializes 72 MHz PLL system clock tree via stm32_clock_control_init(),
 * and disarms the RTC timer.
 *
 * @param seconds Sleep duration in seconds (1 to 65535). If 0, returns 0 immediately.
 * @return 0 on RTC wakeup, -EINTR if user aborted via button/console, negative errno on error.
 */
int rtc_sleep_enter_stop2(uint32_t seconds);

/**
 * @brief Query the source of the most recent wakeup event.
 */
enum rtc_wake_source rtc_sleep_get_last_wake_source(void);

/**
 * @brief Wait for RTC wakeup event or user cancellation (Stop 2 mode).
 *
 * @param timeout_seconds Maximum time to wait in seconds.
 * @return 0 on RTC wakeup, -EINTR if user aborted, negative errno on error.
 */
int rtc_sleep_wait(uint32_t timeout_seconds);

#ifdef __cplusplus
}
#endif

#endif /* RTC_SLEEP_H_ */

