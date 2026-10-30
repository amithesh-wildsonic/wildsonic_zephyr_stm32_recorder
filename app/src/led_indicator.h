/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LED_INDICATOR_H_
#define LED_INDICATOR_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum led_indicator_state {
	LED_IND_OFF = 0,
	LED_IND_RECORDING,	/* 1 Hz blink: 500 ms ON / 500 ms OFF */
	LED_IND_ERROR,		/* 5 Hz rapid alert: 100 ms ON / 100 ms OFF */
};

/**
 * @brief Initialize the board user LED (User LD2 / green_led_2 on PA5).
 * @return 0 on success, or negative errno on failure.
 */
int led_indicator_init(void);

/**
 * @brief Set the LED indicator state non-blockingly.
 * @param state Desired indicator state.
 */
void led_indicator_set(enum led_indicator_state state);

/**
 * @brief Get the current LED indicator state.
 * @return Current indicator state.
 */
enum led_indicator_state led_indicator_get(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_INDICATOR_H_ */

