/* SPDX-License-Identifier: Apache-2.0 */
#include "led_indicator.h"

#if defined(__ZEPHYR__)
#include "wildsonic_trace.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#if DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
static const struct gpio_dt_spec s_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#define HAS_LED 1
#else
#define HAS_LED 0
#endif

static struct k_work_delayable s_led_work;
static bool s_initialized = false;
static enum led_indicator_state s_state = LED_IND_OFF;

static void led_work_handler(struct k_work *work)
{
#if HAS_LED
	if (!s_initialized) {
		return;
	}

	switch (s_state) {
	case LED_IND_ERROR:
		gpio_pin_toggle_dt(&s_led);
		k_work_reschedule(&s_led_work, K_MSEC(100));
		break;
	case LED_IND_RECORDING:
	case LED_IND_OFF:
	default:
		gpio_pin_set_dt(&s_led, 0);
		break;
	}
#else
	ARG_UNUSED(work);
#endif
}

int led_indicator_init(void)
{
#if HAS_LED
	if (!gpio_is_ready_dt(&s_led)) {
		TRACE("User LED gpio device not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&s_led, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		TRACE("Failed to configure User LED gpio: %d", ret);
		return ret;
	}

	k_work_init_delayable(&s_led_work, led_work_handler);
	s_initialized = true;
	s_state = LED_IND_OFF;
	return 0;
#else
	s_initialized = true;
	s_state = LED_IND_OFF;
	return 0;
#endif
}

void led_indicator_set(enum led_indicator_state state)
{
	s_state = state;

#if HAS_LED
	if (!s_initialized) {
		return;
	}

	switch (state) {
	case LED_IND_OFF:
	case LED_IND_RECORDING:
		/* Production/Test parity: keep LED OFF during capture */
		k_work_cancel_delayable(&s_led_work);
		gpio_pin_set_dt(&s_led, 0);
		break;
	case LED_IND_ERROR:
		gpio_pin_set_dt(&s_led, 1);
		k_work_reschedule(&s_led_work, K_MSEC(100));
		break;
	}
#endif
}

enum led_indicator_state led_indicator_get(void)
{
	return s_state;
}

#else /* !defined(__ZEPHYR__) */

static enum led_indicator_state s_sim_state = LED_IND_OFF;

int led_indicator_init(void)
{
	s_sim_state = LED_IND_OFF;
	return 0;
}

void led_indicator_set(enum led_indicator_state state)
{
	s_sim_state = state;
}

enum led_indicator_state led_indicator_get(void)
{
	return s_sim_state;
}

#endif

