/* SPDX-License-Identifier: Apache-2.0 */
#include "ble_comm.h"
#include "console.h"
#include "main.h"
#include "recorder.h"
#include "rtc_calendar.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>

#if defined(CONFIG_SOC_SERIES_STM32L4X)
#include <stm32_ll_usart.h>
#endif

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>

LOG_MODULE_REGISTER(ws_ble, CONFIG_LOG_DEFAULT_LEVEL);

#define BLE_RX_BUF_SIZE 512
#define BLE_TX_BUF_SIZE 1024
#define BLE_COMM_STACK_SIZE 2048
#define BLE_COMM_PRIORITY 8
#define BLE_RESPONSE_PACING_MS 20

RING_BUF_DECLARE(s_ble_rx_rb, BLE_RX_BUF_SIZE);
RING_BUF_DECLARE(s_ble_tx_rb, BLE_TX_BUF_SIZE);

static struct k_sem s_ble_rx_sem;
static const struct device *s_ble_uart_dev = NULL;
static bool s_ble_active = false;
static bool s_ble_connected = false;
static bool s_ble_module_awake = false;
static bool s_pairing_window_active = false;
static int64_t s_pairing_window_end_ms = 0;

static char s_ble_cmd_buf[BLE_UART_CMD_MAX_LEN];
static size_t s_ble_cmd_len = 0;
static bool s_ble_overflow = false;

K_THREAD_STACK_DEFINE(s_ble_stack, BLE_COMM_STACK_SIZE);
static struct k_thread s_ble_thread;
static bool s_ble_thread_started = false;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
static void ble_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev)) {
		uint8_t rx_byte;
		if (uart_fifo_read(dev, &rx_byte, 1) <= 0) {
			break;
		}
		if (ring_buf_put(&s_ble_rx_rb, &rx_byte, 1) > 0) {
			k_sem_give(&s_ble_rx_sem);
		}
	}
	(void)uart_err_check(dev);
}
#endif

int ble_comm_tx(const uint8_t *data, size_t len)
{
	if (!data || len == 0) {
		return 0;
	}

	/* Copy to host-test drain ring buffer */
	ring_buf_put(&s_ble_tx_rb, data, len);

	if (s_ble_uart_dev && s_ble_active) {
		for (size_t i = 0; i < len; i++) {
			uart_poll_out(s_ble_uart_dev, data[i]);
		}
	}
	return (int)len;
}

int ble_comm_printf(const char *format, ...)
{
	char buf[BLE_UART_CMD_MAX_LEN];
	va_list args;
	va_start(args, format);
	int num = vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	if (num <= 0) {
		return num;
	}
	if ((size_t)num >= sizeof(buf)) {
		num = sizeof(buf) - 1;
	}
	return ble_comm_tx((const uint8_t *)buf, (size_t)num);
}

static void ble_comm_process_line(char *line)
{
	if (!line) return;

	/* Trim leading whitespace */
	char *p = line;
	while (*p && isspace((unsigned char)*p)) p++;

	/* Trim trailing whitespace */
	size_t len = strlen(p);
	while (len > 0 && isspace((unsigned char)p[len - 1])) {
		p[--len] = '\0';
	}
	if (len == 0) return;

	/* 1. BLE Module Supervisor Events */
	if (strcmp(p, "OK+CONN") == 0) {
		s_ble_connected = true;
		s_ble_module_awake = true;
		s_pairing_window_active = false;
		LOG_INF("[BLE] Device connected from central");
		return;
	}
	if (strcmp(p, "OK+LOST") == 0) {
		s_ble_connected = false;
		LOG_INF("[BLE] Device disconnected from central");
		ble_comm_lock_pairing();
		return;
	}
	if (strcmp(p, "OK+WAKE") == 0 || strcmp(p, "WS+ALIVE") == 0) {
		s_ble_module_awake = true;
		return;
	}
	if (strcmp(p, "OK+SLEEP") == 0) {
		s_ble_module_awake = false;
		return;
	}
	if (strncmp(p, "OK+", 3) == 0 || strcmp(p, "OK") == 0) {
		/* Command acknowledgment from module */
		return;
	}

	/* 2. Mobile Companion Application Requests */
	if (strcasecmp(p, "GET_DEV_INFO") == 0) {
		s_ble_connected = true;
		s_ble_module_awake = true;
		ble_comm_send_dev_info();
		return;
	}
	if (strcasecmp(p, "GET_STATUS_INFO") == 0 || strcasecmp(p, "STATUS") == 0) {
		s_ble_connected = true;
		s_ble_module_awake = true;
		ble_comm_send_status_info();
		return;
	}

	/* 3. Configuration & Control Commands */
	s_ble_connected = true;
	s_ble_module_awake = true;

	char resp_buf[64] = {0};
	int rc = console_process_line_source(p, true, resp_buf, sizeof(resp_buf));
	if (rc == 0) {
		if (resp_buf[0] != '\0') {
			ble_comm_printf("%s\r\n", resp_buf);
		} else {
			ble_comm_printf("OK\r\n");
		}
	} else {
		ble_comm_printf("ERR\r\n");
	}
}

static void ble_comm_handle_rx_byte(char c)
{
	/* Backspace */
	if (c == '\b') {
		if (s_ble_cmd_len > 0) {
			s_ble_cmd_len--;
		}
		return;
	}

	/* Escape cancels line */
	if (c == 0x1B) {
		s_ble_cmd_len = 0;
		s_ble_overflow = false;
		return;
	}

	/* Line delimiter */
	if (c == '\r' || c == '\n') {
		if (s_ble_overflow) {
			s_ble_overflow = false;
			s_ble_cmd_len = 0;
			return;
		}
		if (s_ble_cmd_len > 0) {
			s_ble_cmd_buf[s_ble_cmd_len] = '\0';
			ble_comm_process_line(s_ble_cmd_buf);
			s_ble_cmd_len = 0;
		}
		return;
	}

	/* Data byte */
	if (s_ble_cmd_len < sizeof(s_ble_cmd_buf) - 1) {
		s_ble_cmd_buf[s_ble_cmd_len++] = c;
	} else {
		s_ble_overflow = true;
		LOG_WRN("[BLE] Command line oversized (>%u bytes), dropping excess",
		        (unsigned int)sizeof(s_ble_cmd_buf));
	}
}

static void ble_comm_thread_fn(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		int ret = k_sem_take(&s_ble_rx_sem, K_MSEC(100));

		/* Service pairing window timeout */
		if (s_pairing_window_active && !s_ble_connected) {
			int64_t now = k_uptime_get();
			if (now >= s_pairing_window_end_ms) {
				s_pairing_window_active = false;
				ble_comm_lock_pairing();
				printk("\nBLE Access: LOCKED (pairing window expired)\n>");
			}
		}

		if (ret != 0) {
			continue;
		}

		uint8_t c;
		while (ring_buf_get(&s_ble_rx_rb, &c, 1) == 1) {
			ble_comm_handle_rx_byte((char)c);
		}
	}
}

int ble_comm_init(const struct device *uart_dev)
{
	if (!uart_dev || !device_is_ready(uart_dev)) {
		LOG_WRN("BLE UART (USART1) device not ready or NULL; BLE disabled");
		return -ENODEV;
	}

	s_ble_uart_dev = uart_dev;
	k_sem_init(&s_ble_rx_sem, 0, BLE_RX_BUF_SIZE);
	ring_buf_reset(&s_ble_rx_rb);
	ring_buf_reset(&s_ble_tx_rb);
	s_ble_connected = false;
	s_ble_module_awake = false;
	s_pairing_window_active = false;
	s_ble_cmd_len = 0;
	s_ble_overflow = false;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_callback_user_data_set(uart_dev, ble_uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);
	s_ble_active = true;
#endif

	if (!s_ble_thread_started) {
		k_thread_create(&s_ble_thread, s_ble_stack,
		                K_THREAD_STACK_SIZEOF(s_ble_stack),
		                ble_comm_thread_fn, NULL, NULL, NULL,
		                K_PRIO_PREEMPT(BLE_COMM_PRIORITY), 0, K_NO_WAIT);
		k_thread_name_set(&s_ble_thread, "ble_comm");
		s_ble_thread_started = true;
	}

	/* Bounded probe with BLE module: up to 3 attempts, 50 ms delay */
	bool probed = false;
	for (int i = 0; i < 3; i++) {
		ble_comm_tx((const uint8_t *)"AT\r\n", 4);
		k_msleep(50);
		if (s_ble_module_awake || s_ble_connected) {
			probed = true;
			break;
		}
	}

	if (probed) {
		LOG_INF("BLE module detected and responsive on USART1");
		struct wildsonic_config *cfg = console_get_config();
		char name_cmd[96];
		snprintf(name_cmd, sizeof(name_cmd), "AT+NAMEWS:%s-%s\r\n",
		         cfg->device_name[0] ? cfg->device_name : "WS",
		         cfg->serial_number[0] ? cfg->serial_number : "01");
		ble_comm_tx((const uint8_t *)name_cmd, strlen(name_cmd));
		k_msleep(50);
		ble_comm_lock_pairing();
	} else {
		LOG_INF("BLE module probe timed out; proceeding normally without BLE");
	}

	return 0;
}

bool ble_comm_is_active(void)
{
	return s_ble_active;
}

bool ble_comm_is_connected(void)
{
	return s_ble_connected;
}

bool ble_comm_is_module_awake(void)
{
	return s_ble_module_awake;
}

bool ble_comm_is_pairing_window_active(void)
{
	return s_pairing_window_active;
}

uint32_t ble_comm_get_pairing_window_remaining(void)
{
	if (!s_pairing_window_active) {
		return 0;
	}
	int64_t now = k_uptime_get();
	if (now >= s_pairing_window_end_ms) {
		return 0;
	}
	return (uint32_t)((s_pairing_window_end_ms - now) / 1000LL);
}

void ble_comm_open_pairing_window(uint32_t seconds)
{
	if (seconds == 0) {
		seconds = BLE_PAIR_WINDOW_DEFAULT_SECONDS;
	} else if (seconds > BLE_PAIR_WINDOW_MAX_SECONDS) {
		seconds = BLE_PAIR_WINDOW_MAX_SECONDS;
	}

	s_pairing_window_active = true;
	s_pairing_window_end_ms = k_uptime_get() + (int64_t)seconds * 1000LL;

	/* Wake module if in sleep mode */
	ble_comm_tx((const uint8_t *)"AT\r\n", 4);
	k_msleep(50);
	/* Set connectable advertising */
	ble_comm_tx((const uint8_t *)"AT+ADTY0\r\n", 10);
	k_msleep(50);
	ble_comm_tx((const uint8_t *)"AT+RESET\r\n", 10);
	s_ble_module_awake = true;

	LOG_INF("BLE Access: UNLOCKED (%u s)", (unsigned int)seconds);
}

void ble_comm_lock_pairing(void)
{
	s_pairing_window_active = false;
	s_pairing_window_end_ms = 0;

	/* Set non-connectable scannable advertising (ADTY 2) */
	ble_comm_tx((const uint8_t *)"AT+ADTY2\r\n", 10);
	k_msleep(50);
	/* Enter low-power sleep */
	ble_comm_tx((const uint8_t *)"AT+SLEEP\r\n", 10);
	s_ble_module_awake = false;

	LOG_INF("BLE Access: LOCKED");
}

void ble_comm_suspend(void)
{
	if (!s_ble_active) return;

	/* If disconnected and module is awake, command it to sleep */
	if (!s_ble_connected && s_ble_module_awake) {
		ble_comm_tx((const uint8_t *)"AT+SLEEP\r\n", 10);
		k_msleep(30);
		s_ble_module_awake = false;
	}

	if (s_ble_uart_dev) {
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
		uart_irq_rx_disable(s_ble_uart_dev);
#endif
	}

#if defined(CONFIG_SOC_SERIES_STM32L4X)
	/* Drain USART1 TX buffer before Stop 2 */
	if ((USART1->CR1 & USART_CR1_UE) && (USART1->CR1 & USART_CR1_TE)) {
		uint32_t start_ms = k_uptime_get_32();
		while (!LL_USART_IsActiveFlag_TC(USART1)) {
			if ((uint32_t)(k_uptime_get_32() - start_ms) >= 200U) {
				break;
			}
			k_busy_wait(10);
		}
	}

	LL_USART_DisableInStopMode(USART1);
	LL_USART_DisableIT_WKUP(USART1);
	LL_USART_ClearFlag_WKUP(USART1);
	LL_USART_ClearFlag_ORE(USART1);
	LL_USART_ClearFlag_FE(USART1);
	LL_USART_ClearFlag_NE(USART1);
	LL_USART_ClearFlag_PE(USART1);
	LL_USART_ClearFlag_IDLE(USART1);
	if (LL_USART_IsActiveFlag_RXNE(USART1)) {
		(void)USART1->RDR;
	}
#endif
}

void ble_comm_resume(void)
{
	if (!s_ble_active || !s_ble_uart_dev) return;

#if defined(CONFIG_SOC_SERIES_STM32L4X)
	LL_USART_ClearFlag_ORE(USART1);
	LL_USART_ClearFlag_FE(USART1);
	LL_USART_ClearFlag_NE(USART1);
	LL_USART_ClearFlag_PE(USART1);
	LL_USART_ClearFlag_IDLE(USART1);
	LL_USART_ClearFlag_WKUP(USART1);
	if (LL_USART_IsActiveFlag_RXNE(USART1)) {
		(void)USART1->RDR;
	}
#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_rx_enable(s_ble_uart_dev);
#endif
}

void ble_comm_send_dev_info(void)
{
	struct wildsonic_config *cfg = console_get_config();

	ble_comm_printf("SAMF %u\r\n", cfg->sampling_rate);
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("BTPS %u\r\n", cfg->audio_sampling_width);
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("NUMCH %u\r\n", cfg->num_audio_channels);
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("GAIN %d\r\n", cfg->mic_gain_times_ten);
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("LCUTOFF %u\r\n", cfg->lower_cutoff_frequency);
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("UCUTOFF %u\r\n", cfg->upper_cutoff_frequency);
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("RECT %u\r\n", cfg->recording_length);
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("SLEEP %u\r\n", cfg->sleep_pause_interval);
	k_msleep(BLE_RESPONSE_PACING_MS);

	ble_comm_printf("VER 2.1.0-prod, SHA d7a4f91, DT Oct 30 2026\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);

	uint64_t sd_used = 0, sd_total = 0;
	struct fs_statvfs stat;
	if (fs_statvfs("/SD:", &stat) == 0) {
		sd_total = (uint64_t)stat.f_blocks * (uint64_t)stat.f_frsize;
		uint64_t free_b = (uint64_t)stat.f_bfree * (uint64_t)stat.f_frsize;
		sd_used = (sd_total > free_b) ? (sd_total - free_b) : 0;
	}
	ble_comm_printf("SDUSED %llu, SDCAP %llu\r\n",
	                (unsigned long long)sd_used, (unsigned long long)sd_total);
	k_msleep(BLE_RESPONSE_PACING_MS);

	const char *fldr_str = (cfg->folder_structure == 1) ? "daily" :
	                       (cfg->folder_structure == 2) ? "hourly" : "flat";
	ble_comm_printf("FLDR %s\r\n", fldr_str);
	k_msleep(BLE_RESPONSE_PACING_MS);

	/* Battery metrics: unmeasured on Nucleo board, reported as 0 */
	ble_comm_printf("VBATT 0, CBATT 0, REMBATT 0\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("FULLBATT 0, REMTIME 0, AVGPWR 0\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("NBAT 0\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);

	struct rtc_calendar_time now_cal;
	if (rtc_calendar_get(&now_cal) != 0 || !rtc_calendar_is_valid()) {
		now_cal.year = 2026; now_cal.month = 10; now_cal.day = 30;
		now_cal.hour = 12; now_cal.min = 0; now_cal.sec = 0;
	}
	ble_comm_printf("DATETIME %04d-%02d-%02d %02d:%02d:%02d.000 +00:00\r\n",
	                now_cal.year, now_cal.month, now_cal.day,
	                now_cal.hour, now_cal.min, now_cal.sec);
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("TIMEZONE +00:00\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);

	ble_comm_printf("DATERANGE 2026/01/01-2099/12/31\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("SUNMODE Disabled\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);

	ble_comm_printf("TIMEOFDAY 0 00:00:00-23:59:59\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);
	for (int i = 1; i < 10; i++) {
		ble_comm_printf("TIMEOFDAY %d 00:00:00-00:00:00\r\n", i);
		k_msleep(15);
	}

	ble_comm_printf("DAYSOFWEEK MTWRFSU\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);

	/* Environmental status: terminating line that releases mobile app busy lock */
	ble_comm_printf("TEMP 0.00, HUMID 0.00\r\n");
}

void ble_comm_send_status_info(void)
{
	uint64_t sd_used = 0, sd_total = 0;
	struct fs_statvfs stat;
	if (fs_statvfs("/SD:", &stat) == 0) {
		sd_total = (uint64_t)stat.f_blocks * (uint64_t)stat.f_frsize;
		uint64_t free_b = (uint64_t)stat.f_bfree * (uint64_t)stat.f_frsize;
		sd_used = (sd_total > free_b) ? (sd_total - free_b) : 0;
	}
	ble_comm_printf("SDUSED %llu, SDCAP %llu\r\n",
	                (unsigned long long)sd_used, (unsigned long long)sd_total);
	k_msleep(BLE_RESPONSE_PACING_MS);

	ble_comm_printf("VBATT 0, CBATT 0, REMBATT 0\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);
	ble_comm_printf("FULLBATT 0, REMTIME 0, AVGPWR 0\r\n");
	k_msleep(BLE_RESPONSE_PACING_MS);

	/* Terminating line */
	ble_comm_printf("TEMP 0.00, HUMID 0.00\r\n");
}

int ble_comm_feed_rx_bytes(const uint8_t *data, size_t len)
{
	if (!data || len == 0) return 0;
	for (size_t i = 0; i < len; i++) {
		ble_comm_handle_rx_byte((char)data[i]);
	}
	return (int)len;
}

int ble_comm_drain_tx_bytes(uint8_t *buf, size_t max_len)
{
	if (!buf || max_len == 0) return 0;
	return (int)ring_buf_get(&s_ble_tx_rb, buf, max_len);
}

void ble_comm_reset_test_state(void)
{
	ring_buf_reset(&s_ble_rx_rb);
	ring_buf_reset(&s_ble_tx_rb);
	s_ble_connected = false;
	s_ble_module_awake = false;
	s_pairing_window_active = false;
	s_pairing_window_end_ms = 0;
	s_ble_cmd_len = 0;
	s_ble_overflow = false;
}
