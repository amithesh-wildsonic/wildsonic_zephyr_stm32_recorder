/* SPDX-License-Identifier: Apache-2.0 */
#include "rtc_sleep.h"
#include "recorder.h"
#include "psram_is66wvs.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rtc_sleep, LOG_LEVEL_INF);

#if defined(CONFIG_SOC_SERIES_STM32L4X)

#include <soc.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_rtc.h>
#include <stm32_ll_exti.h>
#include <stm32_ll_cortex.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <stm32_ll_usart.h>

/* Forward declaration of Zephyr STM32 clock control restoration function */
extern int stm32_clock_control_init(const struct device *dev);

static struct k_sem rtc_wake_sem;
static volatile bool rtc_timer_active = false;
static bool rtc_initialized = false;
static volatile bool user_button_pressed = false;
static volatile enum rtc_wake_source last_wake_source = RTC_WAKE_NONE;

static void rtc_wkup_isr(const void *arg)
{
	ARG_UNUSED(arg);

	/* Clear RTC Wakeup Timer Flag (WUTF) */
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableWriteProtection(RTC);

	/* Clear EXTI Line 20 pending bit */
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);

	rtc_timer_active = false;
	last_wake_source = RTC_WAKE_TIMER;
	k_sem_give(&rtc_wake_sem);
}

enum rtc_wake_source rtc_sleep_get_last_wake_source(void)
{
	return last_wake_source;
}

int rtc_sleep_init(void)
{
	if (rtc_initialized) {
		return 0;
	}
	k_sem_init(&rtc_wake_sem, 0, 1);

	/* Enable Power Controller clock and Backup Domain write access */
	LL_PWR_EnableBkUpAccess();

	/* Enable RTC clock source if not already running */
	if (!LL_RCC_IsEnabledRTC()) {
		if (LL_RCC_LSE_IsReady()) {
			LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
			LOG_INF("RTC clock source configured: LSE (32.768 kHz external crystal)");
		} else {
			LL_RCC_LSI_Enable();
			uint32_t t = 10000;
			while (!LL_RCC_LSI_IsReady() && --t) {
				k_busy_wait(10);
			}
			LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
			LOG_INF("RTC clock source configured: LSI (~32 kHz internal RC oscillator)");
		}
		LL_RCC_EnableRTC();
	}

	/* Configure EXTI Line 20 for RTC Wakeup (Interrupt, Event, Rising Edge) */
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_20);
	LL_EXTI_EnableEvent_0_31(LL_EXTI_LINE_20);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_20);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);

	/* Connect and enable NVIC interrupt for RTC_WKUP_IRQn */
	IRQ_CONNECT(RTC_WKUP_IRQn, 1, rtc_wkup_isr, NULL, 0);
	irq_enable(RTC_WKUP_IRQn);

	/* Configure User Button B1 (PC13) as hardware wakeup source via EXTI Line 13 Event */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_13, LL_GPIO_PULL_UP);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);
	LL_EXTI_EnableEvent_0_31(LL_EXTI_LINE_13);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_13);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);

	rtc_initialized = true;
	LOG_INF("RTC Wakeup Timer initialized (EXTI Line 20, IRQ %d; B1 Wakeup EXTI Line 13)",
	        RTC_WKUP_IRQn);
	return 0;
}

int rtc_sleep_arm(uint32_t seconds)
{
	if (!rtc_initialized) {
		int ret = rtc_sleep_init();
		if (ret) {
			return ret;
		}
	}

	if (seconds == 0) {
		return 0;
	}
	if (seconds > 65535) {
		seconds = 65535;
	}

	/* Reset wakeup semaphore */
	k_sem_reset(&rtc_wake_sem);

	/* Disable RTC register write protection */
	LL_RTC_DisableWriteProtection(RTC);

	/* Disable Wakeup Timer to allow writing to WUTR */
	LL_RTC_WAKEUP_Disable(RTC);

	/* Wait until WUTWF (Wakeup Timer Write Flag) is set */
	uint32_t to = 10000;
	while (!LL_RTC_IsActiveFlag_WUTW(RTC) && --to) {
		k_busy_wait(10);
	}
	if (to == 0) {
		LL_RTC_EnableWriteProtection(RTC);
		LOG_ERR("Timeout waiting for RTC WUTWF");
		return -ETIMEDOUT;
	}

	/* Set clock to ck_spre (1 Hz synchronous prescaler clock) */
	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);

	/* Set auto-reload value (period = Value + 1 seconds) */
	LL_RTC_WAKEUP_SetAutoReload(RTC, seconds - 1);

	/* Clear pending flags */
	LL_RTC_ClearFlag_WUT(RTC);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);

	/* Enable Wakeup Timer interrupt and timer itself */
	LL_RTC_EnableIT_WUT(RTC);
	LL_RTC_WAKEUP_Enable(RTC);

	/* Re-enable write protection */
	LL_RTC_EnableWriteProtection(RTC);

	rtc_timer_active = true;
	return 0;
}

void rtc_sleep_disarm(void)
{
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	LL_RTC_DisableIT_WUT(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableWriteProtection(RTC);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);
	rtc_timer_active = false;
}

int rtc_sleep_enter_stop2(uint32_t seconds)
{
	if (seconds == 0) {
		return 0;
	}

	if (!rtc_initialized) {
		int ret = rtc_sleep_init();
		if (ret) {
			return ret;
		}
	}

	if (recorder_is_abort_requested() || console_check_abort_requested()) {
		last_wake_source = RTC_WAKE_CONSOLE_ABORT;
		return -EINTR;
	}

	/* Arm RTC Wakeup Timer */
	int rc = rtc_sleep_arm(seconds);
	if (rc != 0) {
		return rc;
	}

	user_button_pressed = false;
	last_wake_source = RTC_WAKE_NONE;
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);

	/* Ensure User LED (LD2 / PA5) is completely OFF */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);

	/* Flush USART2 transmit buffer before entering Stop 2 so no characters are lost */
	uint32_t uart_flush_start_ms = k_uptime_get_32();
	while (!LL_USART_IsActiveFlag_TC(USART2)) {
		if ((uint32_t)(k_uptime_get_32() - uart_flush_start_ms) >= 1000U) {
			LOG_ERR("USART2 transmit-complete timeout; refusing Stop2 entry");
			rtc_sleep_disarm();
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
	}

	/* Configure STM32L4 Power controller for Stop 2 Mode */
	LL_PWR_EnableBkUpAccess();
	LL_PWR_ClearFlag_WU();
#if STM32_SYSCLK_SRC_MSI
	LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);
#else
	LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);
#endif
#ifdef PWR_CR1_RRSTP
	LL_PWR_DisableSRAM3Retention();
#endif
	LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);

	/* Enable Cortex-M4 Deep Sleep (sets SCB->SCR SLEEPDEEP bit) */
	LL_LPM_EnableDeepSleep();

	/* Suspend SysTick to prevent kernel tick interrupts during Stop 2 */
	uint32_t saved_systick_ctrl = SysTick->CTRL;
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
	SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;

	/* Clear pending flags immediately prior to entering Stop 2 */
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableWriteProtection(RTC);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
	NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

	/*
	 * =========================================================================
	 * ENTER STM32L476 STOP 2 LOW-POWER MODE
	 * High-speed clocks (PLL, HSI, HSE) are halted.
	 * VCORE regulator operates in low-power mode. SRAM1 & SRAM2 are retained.
	 * Only RTC (LSE/LSI) and EXTI lines remain active.
	 * MCU core current through JP6 (IDD) drops to ultra-low Stop 2 level.
	 * =========================================================================
	 */
	__DSB();
	__WFI();
	__ISB();

	/*
	 * =========================================================================
	 * WAKE-UP SEQUENCE FROM STOP 2 MODE
	 * =========================================================================
	 */
	/* 1. Clear SLEEPDEEP bit in Cortex-M4 SCR */
	LL_LPM_EnableSleep();

	/* 2. Restore SysTick */
	SysTick->CTRL = saved_systick_ctrl;

	/* 3. Re-initialize 72 MHz PLL clock tree via Zephyr clock control */
	stm32_clock_control_init(NULL);

	/* 4. Clear pending EXTI and RTC flags */
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableWriteProtection(RTC);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);

	/* 5. Disarm RTC Wakeup Timer */
	rtc_sleep_disarm();

	/* 6. Check wake source */
	if (user_button_pressed || LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) ||
	    LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13) == 0) {
		last_wake_source = RTC_WAKE_USER_BUTTON;
		return -EINTR;
	}
	if (recorder_is_abort_requested() || console_check_abort_requested()) {
		last_wake_source = RTC_WAKE_CONSOLE_ABORT;
		return -EINTR;
	}

	last_wake_source = RTC_WAKE_TIMER;
	return 0;
}

int rtc_sleep_wait(uint32_t timeout_seconds)
{
	return rtc_sleep_enter_stop2(timeout_seconds);
}

#else /* !CONFIG_SOC_SERIES_STM32L4X */

enum rtc_wake_source rtc_sleep_get_last_wake_source(void) { return RTC_WAKE_NONE; }
int rtc_sleep_init(void) { return 0; }
int rtc_sleep_arm(uint32_t seconds) { ARG_UNUSED(seconds); return 0; }
void rtc_sleep_disarm(void) { }
int rtc_sleep_enter_stop2(uint32_t seconds)
{
	if (seconds == 0) return 0;
	k_sleep(K_SECONDS(seconds));
	return 0;
}
int rtc_sleep_wait(uint32_t timeout_seconds)
{
	if (timeout_seconds == 0) return 0;
	for (uint32_t s = 0; s < timeout_seconds; s++) {
		if (recorder_is_abort_requested() || console_check_abort_requested()) {
			return -EINTR;
		}
		k_sleep(K_SECONDS(1));
	}
	return 0;
}

#endif /* CONFIG_SOC_SERIES_STM32L4X */
