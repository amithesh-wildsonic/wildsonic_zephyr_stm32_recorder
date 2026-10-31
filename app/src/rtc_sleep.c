/* SPDX-License-Identifier: Apache-2.0 */
#include "rtc_sleep.h"
#include "recorder.h"
#include "psram_is66wvs.h"
#include "console.h"
#include "ble_comm.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/timer/system_timer.h>

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(rtc_sleep, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(rtc_sleep, LOG_LEVEL_WRN);
#endif

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

static inline void rtc_access_enable(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableBkUpAccess();
}

static inline void rtc_write_protect_disable(void)
{
	rtc_access_enable();
	LL_RTC_DisableWriteProtection(RTC);
}

static inline void rtc_write_protect_enable(void)
{
	LL_RTC_EnableWriteProtection(RTC);
}

static void rtc_wkup_isr(const void *arg)
{
	ARG_UNUSED(arg);

	rtc_access_enable();

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
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableBkUpAccess();

	/* Enable RTC clock source if not already running */
	if (!LL_RCC_IsEnabledRTC()) {
		/* Attempt LSE initialization with bounded timeout */
		if (!LL_RCC_LSE_IsReady()) {
			LL_RCC_LSE_Enable();
			uint32_t lse_to = 50000;
			while (!LL_RCC_LSE_IsReady() && --lse_to) {
				k_busy_wait(10);
			}
		}

		if (LL_RCC_LSE_IsReady()) {
			LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
			LOG_INF("RTC clock source configured: LSE (32.768 kHz external crystal)");
		} else {
			LOG_WRN("LSE oscillator not ready; attempting LSI fallback (~32 kHz internal RC)");
			LL_RCC_LSI_Enable();
			uint32_t lsi_to = 10000;
			while (!LL_RCC_LSI_IsReady() && --lsi_to) {
				k_busy_wait(10);
			}
			if (!LL_RCC_LSI_IsReady()) {
				LOG_ERR("LSI oscillator startup timed out; RTC initialization failed");
				return -ETIMEDOUT;
			}
			LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
			LOG_WRN("RTC clock source configured: LSI (~32 kHz internal RC oscillator; calibration limitations apply)");
		}
		LL_RCC_EnableRTC();
	}

	/* Verify RTC is running */
	if (!LL_RCC_IsEnabledRTC()) {
		LOG_ERR("Failed to enable RTC peripheral");
		return -EIO;
	}

	/* Disable RTC write protection to configure prescalers */
	LL_RTC_DisableWriteProtection(RTC);

	/* Configure prescalers if not already initialized (Asynch 127, Synch 255 for 32.768 kHz -> 1 Hz ck_spre) */
	if (!LL_RTC_IsActiveFlag_INITS(RTC)) {
		LL_RTC_EnableInitMode(RTC);
		uint32_t init_to = 10000;
		while (!LL_RTC_IsActiveFlag_INIT(RTC) && --init_to) {
			k_busy_wait(10);
		}
		if (init_to == 0) {
			LL_RTC_EnableWriteProtection(RTC);
			LOG_ERR("Timeout waiting for RTC INIT mode in rtc_sleep_init");
			return -ETIMEDOUT;
		}
		LL_RTC_SetAsynchPrescaler(RTC, 127);
		LL_RTC_SetSynchPrescaler(RTC, 255);
		LL_RTC_DisableInitMode(RTC);
	}
	LL_RTC_EnableWriteProtection(RTC);

	/* Configure EXTI Line 20 for RTC Wakeup (Interrupt, Event, Rising Edge) */
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_20);
	LL_EXTI_EnableEvent_0_31(LL_EXTI_LINE_20);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_20);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);

	/* Connect and enable NVIC interrupt for RTC_WKUP_IRQn */
	IRQ_CONNECT(RTC_WKUP_IRQn, 1, rtc_wkup_isr, NULL, 0);
	irq_enable(RTC_WKUP_IRQn);

	/* Configure User Button B1 (PC13) as GPIO input, but STRICTLY DISABLE it as a wake source */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_13, LL_GPIO_PULL_UP);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);
	LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_13);
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_13);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);

	/* Disable all STM32 PWR wakeup pins (WKUP1..WKUP5) */
	LL_PWR_DisableWakeUpPin(LL_PWR_WAKEUP_PIN1 | LL_PWR_WAKEUP_PIN2 | LL_PWR_WAKEUP_PIN3 |
	                        LL_PWR_WAKEUP_PIN4 | LL_PWR_WAKEUP_PIN5);
	LL_PWR_ClearFlag_WU();

	rtc_initialized = true;
	LOG_INF("RTC Wakeup Timer initialized (EXTI Line 20, IRQ %d; Strict RTC-only wake enforced)",
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

	/* Enable Power Controller clock and Backup Domain write access */
	rtc_write_protect_disable();

	/* Disable Wakeup Timer only if it was enabled */
	if (LL_RTC_WAKEUP_IsEnabled(RTC)) {
		LL_RTC_WAKEUP_Disable(RTC);
	}

	/* Wait until WUTWF (Wakeup Timer Write Flag) is set (takes up to 2 RTCCLK cycles) */
	uint32_t to = 10000;
	while (!LL_RTC_IsActiveFlag_WUTW(RTC) && --to) {
		k_busy_wait(10);
	}
	if (to == 0) {
		rtc_write_protect_enable();
		LOG_ERR("Timeout waiting for RTC WUTWF (ISR=0x%08X, CR=0x%08X)",
		        (unsigned int)RTC->ISR, (unsigned int)RTC->CR);
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
	rtc_write_protect_enable();

	/* Verify timer is armed */
	if (!LL_RTC_WAKEUP_IsEnabled(RTC)) {
		LOG_ERR("Failed to arm RTC Wakeup Timer (CR=0x%08X)", (unsigned int)RTC->CR);
		return -EIO;
	}

	rtc_timer_active = true;
	return 0;
}

void rtc_sleep_disarm(void)
{
	rtc_write_protect_disable();
	LL_RTC_WAKEUP_Disable(RTC);
	LL_RTC_DisableIT_WUT(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	rtc_write_protect_enable();
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);
	NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
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

	if (recorder_is_abort_requested()) {
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

	/* Ensure User LED (LD2 / PA5) is completely OFF */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);

	/* Flush USART2 transmit buffer before entering Stop 2 so no characters are lost */
	bool usart_ue = (USART2->CR1 & USART_CR1_UE) != 0;
	bool usart_te = (USART2->CR1 & USART_CR1_TE) != 0;

	if (usart_ue && usart_te) {
		uint32_t uart_flush_start_ms = k_uptime_get_32();
		while (!LL_USART_IsActiveFlag_TC(USART2)) {
			if ((uint32_t)(k_uptime_get_32() - uart_flush_start_ms) >= 1000U) {
				uint32_t isr = USART2->ISR;
				uint32_t cr1 = USART2->CR1;
				uint32_t cr3 = USART2->CR3;
				uint32_t brr = USART2->BRR;
				bool tc_flag = (isr & USART_ISR_TC) != 0;
				bool txe_flag = (isr & USART_ISR_TXE) != 0;
				LOG_ERR("USART2 transmit-complete timeout; refusing Stop2 entry (ISR=0x%08X, CR1=0x%08X, CR3=0x%08X, BRR=0x%08X, TC=%d, TXE=%d, UE=%d, TE=%d)",
				        (unsigned int)isr, (unsigned int)cr1, (unsigned int)cr3, (unsigned int)brr,
				        tc_flag, txe_flag, usart_ue, usart_te);
				rtc_sleep_disarm();
				return -ETIMEDOUT;
			}
			k_busy_wait(10);
		}
	}

	/*
	 * =========================================================================
	 * STRICT RTC-ONLY WAKE SOURCE ENFORCEMENT
	 * =========================================================================
	 * 1. Suspend console UART RX interrupts and ring buffer.
	 * 2. Disable USART2 wake-from-stop and interrupts so UART RX cannot wake MCU.
	 * 3. Disable all STM32 PWR wakeup pins (WKUP1..WKUP5).
	 * 4. Mask all EXTI lines except EXTI Line 20 (RTC Wakeup Timer).
	 * 5. Mask all NVIC interrupts except RTC_WKUP_IRQn.
	 * 6. Suspend SysTick.
	 * =========================================================================
	 */
	console_async_suspend();
	ble_comm_suspend();
	LL_USART_DisableInStopMode(USART2);
	LL_USART_DisableIT_WKUP(USART2);
	LL_USART_ClearFlag_WKUP(USART2);
	LL_USART_ClearFlag_ORE(USART2);
	LL_USART_ClearFlag_FE(USART2);
	LL_USART_ClearFlag_NE(USART2);
	LL_USART_ClearFlag_PE(USART2);
	LL_USART_ClearFlag_IDLE(USART2);

	if (LL_USART_IsActiveFlag_RXNE(USART2)) {
		(void)USART2->RDR;
	}

	LL_PWR_DisableWakeUpPin(LL_PWR_WAKEUP_PIN1 | LL_PWR_WAKEUP_PIN2 | LL_PWR_WAKEUP_PIN3 |
	                        LL_PWR_WAKEUP_PIN4 | LL_PWR_WAKEUP_PIN5);
	LL_PWR_ClearFlag_WU();

	/* Save active EXTI configuration */
	uint32_t saved_exti_imr1 = EXTI->IMR1;
	uint32_t saved_exti_emr1 = EXTI->EMR1;
	uint32_t saved_exti_imr2 = EXTI->IMR2;
	uint32_t saved_exti_emr2 = EXTI->EMR2;

	/* Strictly mask all EXTI lines except Line 20 (RTC Wakeup) */
	EXTI->IMR1 = LL_EXTI_LINE_20;
	EXTI->EMR1 = 0;
	EXTI->IMR2 = 0;
	EXTI->EMR2 = 0;
	EXTI->PR1 = 0xFFFFFFFF;
	EXTI->PR2 = 0xFFFFFFFF;

	/* Save active NVIC interrupt enable configuration */
	uint32_t saved_nvic_iser[3];
	saved_nvic_iser[0] = NVIC->ISER[0];
	saved_nvic_iser[1] = NVIC->ISER[1];
	saved_nvic_iser[2] = NVIC->ISER[2];

	/* Disable all NVIC interrupts except RTC_WKUP_IRQn (IRQ 3) */
	NVIC->ICER[0] = 0xFFFFFFFF & ~(1UL << (RTC_WKUP_IRQn & 0x1F));
	NVIC->ICER[1] = 0xFFFFFFFF;
	NVIC->ICER[2] = 0xFFFFFFFF;

	NVIC->ISER[0] = (1UL << (RTC_WKUP_IRQn & 0x1F));

	/* Clear all pending NVIC flags */
	NVIC->ICPR[0] = 0xFFFFFFFF;
	NVIC->ICPR[1] = 0xFFFFFFFF;
	NVIC->ICPR[2] = 0xFFFFFFFF;

	/* Suspend SysTick to prevent kernel tick interrupts during Stop 2 */
	uint32_t saved_systick_ctrl = SysTick->CTRL;
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
	SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;

	/* Configure STM32L4 Power controller for Stop 2 Mode */
	rtc_access_enable();
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

	/* Clear pending flags immediately prior to entering Stop 2 */
	rtc_write_protect_disable();
	LL_RTC_ClearFlag_WUT(RTC);
	rtc_write_protect_enable();
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
	NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);

	/* Verify RTC Wakeup Timer is armed before WFI */
	if (!LL_RTC_WAKEUP_IsEnabled(RTC)) {
		LOG_ERR("Stop 2 aborted: RTC Wakeup Timer is not enabled");
		LL_LPM_EnableSleep();
		rtc_sleep_disarm();
		return -EIO;
	}

	/*
	 * =========================================================================
	 * ENTER STM32L476 STOP 2 LOW-POWER MODE
	 * =========================================================================
	 */
	__disable_irq();
	bool pre_wake_event = (LL_RTC_IsActiveFlag_WUT(RTC) != 0) ||
	                      (NVIC_GetPendingIRQ(RTC_WKUP_IRQn) != 0);
	if (!pre_wake_event) {
		__DSB();
		__WFI();
		__ISB();
	}

	/*
	 * =========================================================================
	 * WAKE-UP SEQUENCE FROM STOP 2 MODE (STRICT RTC WAKE RECOVERY)
	 * Inspect hardware status BEFORE enabling interrupts or taking any ISR!
	 * =========================================================================
	 */
	bool rtc_wut_flag = (LL_RTC_IsActiveFlag_WUT(RTC) != 0);
	bool exti20_flag = (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_20) != 0);
	bool nvic_rtc_flag = (NVIC_GetPendingIRQ(RTC_WKUP_IRQn) != 0);

	uint32_t exti_pr1_snap = EXTI->PR1;
	uint32_t exti_pr2_snap = EXTI->PR2;
	uint32_t pwr_sr1_snap = PWR->SR1;
	uint32_t nvic_ispr0_snap = NVIC->ISPR[0];
	uint32_t nvic_ispr1_snap = NVIC->ISPR[1];
	uint32_t nvic_ispr2_snap = NVIC->ISPR[2];

	/* Clear SLEEPDEEP bit in Cortex-M4 SCR */
	LL_LPM_EnableSleep();

	/* Enable interrupts: vectors immediately to rtc_wkup_isr() if pending */
	__enable_irq();

	/* Determine wake status */
	int wake_rc = 0;
	if (rtc_wut_flag || exti20_flag || nvic_rtc_flag || pre_wake_event || (last_wake_source == RTC_WAKE_TIMER)) {
		last_wake_source = RTC_WAKE_TIMER;
		wake_rc = 0;
	} else {
		LOG_WRN("Unexpected wake event: RTC WUT flag not asserted EXTI_PR1=0x%08X, PR2=0x%08X, PWR_SR1=0x%08X, NVIC_ISPR=[0x%08X, 0x%08X, 0x%08X]",
		        exti_pr1_snap, exti_pr2_snap, pwr_sr1_snap, nvic_ispr0_snap, nvic_ispr1_snap, nvic_ispr2_snap);
		last_wake_source = RTC_WAKE_NONE;
		wake_rc = -EIO;
	}

	/* Re-enable PWR clock and Backup domain access after Stop 2 */
	rtc_access_enable();

	/* Re-initialize 72 MHz PLL clock tree via Zephyr clock control */
	int clk_ret = stm32_clock_control_init(NULL);
	if (clk_ret != 0 || LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
		LOG_ERR("Clock restoration failed after Stop 2 (ret=%d, src=0x%02X)",
		        clk_ret, LL_RCC_GetSysClkSource());
		wake_rc = -EIO;
	}

	/* Restore SysTick and compensate kernel uptime ticks */
	SysTick->CTRL = saved_systick_ctrl;
	if (seconds > 0 && wake_rc == 0) {
		uint64_t elapsed_ticks = (uint64_t)seconds * CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		while (elapsed_ticks > 0) {
			uint32_t chunk_ticks = (elapsed_ticks > (UINT32_MAX / 4))
			                           ? (UINT32_MAX / 4)
			                           : (uint32_t)elapsed_ticks;
			sys_clock_announce(chunk_ticks);
			elapsed_ticks -= chunk_ticks;
		}
	}

	/* Restore NVIC interrupt enable configuration */
	NVIC->ISER[0] = saved_nvic_iser[0];
	NVIC->ISER[1] = saved_nvic_iser[1];
	NVIC->ISER[2] = saved_nvic_iser[2];

	/* Restore EXTI mask configuration */
	EXTI->IMR1 = saved_exti_imr1;
	EXTI->EMR1 = saved_exti_emr1;
	EXTI->IMR2 = saved_exti_imr2;
	EXTI->EMR2 = saved_exti_emr2;

	/* Restore console UART RX interrupts after Stop 2 wakeup */
	console_async_resume();
	ble_comm_resume();

	/* Disarm RTC Wakeup Timer and clear pending flags */
	rtc_sleep_disarm();
	rtc_write_protect_disable();
	LL_RTC_ClearFlag_WUT(RTC);
	rtc_write_protect_enable();
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
	NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
	LL_PWR_ClearFlag_WU();

	return wake_rc;
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
		if (recorder_is_abort_requested()) {
			return -EINTR;
		}
		k_sleep(K_SECONDS(1));
	}
	return 0;
}

#endif /* CONFIG_SOC_SERIES_STM32L4X */
