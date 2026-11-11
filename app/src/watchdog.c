/* SPDX-License-Identifier: Apache-2.0 */
#include "watchdog.h"

#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#if defined(CONFIG_WILDSONIC_WATCHDOG)
#include <stm32l4xx_ll_iwdg.h>
#include <stm32l4xx_ll_bus.h>
#endif

void watchdog_init(void)
{
#if defined(CONFIG_WILDSONIC_WATCHDOG)
	/* IWDG clock runs on LSI (~32 kHz).
	 * Prescaler = 256 -> 32000 / 256 = 125 Hz (8 ms per tick).
	 * Reload value 4095 -> 4095 * 8 ms = ~32.7 seconds timeout.
	 */
	LL_IWDG_Enable(IWDG);
	LL_IWDG_EnableWriteAccess(IWDG);
	LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_256);
	LL_IWDG_SetReloadCounter(IWDG, 4095);
	while (!LL_IWDG_IsReady(IWDG)) {
		/* Wait until registers are updated */
	}
	LL_IWDG_ReloadCounter(IWDG);
#endif
}

void watchdog_feed(void)
{
#if defined(CONFIG_WILDSONIC_WATCHDOG)
	LL_IWDG_ReloadCounter(IWDG);
#endif
}

#else

void watchdog_init(void) {}
void watchdog_feed(void) {}

#endif
