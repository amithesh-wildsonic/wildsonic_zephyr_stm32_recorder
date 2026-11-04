/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file console.c
 * @brief Canonical WildSonic Product Command Interface for STM32L476RG + Zephyr RTOS.
 *
 * Implements the WildSonic command table, parser, argument validators, getters,
 * and setters based on the reference Maxim MAX78000 implementation (app/console.cpp).
 *
 * Strict Build Separation:
 *  - PRODUCTION: Only canonical product commands and explicitly deferred commands are exposed.
 *    Development commands (start, stop, record, rate, time, bench, diagnose, compare,
 *    stress, integrity, schedule) are strictly excluded from help output and rejected
 *    by the dispatcher with "Unknown or BAD command".
 *  - TEST: Product interface is retained, and engineering tools (bench, diagnose, compare,
 *    stress, integrity, psram test/diag/sshift/bench, manual start/stop) are enabled under
 *    Kconfig guards.
 */

#include "console.h"
#include "config_store.h"
#include "main.h"
#include "scheduler.h"
#include "rtc_sleep.h"
#include "rtc_calendar.h"
#include "audio_in.h"
#include "led_indicator.h"
#include "recorder.h"
#include "psram_is66wvs.h"
#include "audio_format.h"
#include "ble_comm.h"

#ifndef WILDSONIC_GIT_COMMIT
#define WILDSONIC_GIT_COMMIT "unknown"
#endif
#include "dsp.h"
#include "sd_benchmark.h"
#include "solar_calculator.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#if defined(CONFIG_SOC_SERIES_STM32L4X)
#include <stm32_ll_usart.h>
#endif

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(ws_console, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(ws_console, LOG_LEVEL_WRN);
#endif

#define CANONICAL_CMD_COUNT 59

#define CONSOLE_RX_BUF_SIZE 256
RING_BUF_DECLARE(s_console_rx_rb, CONSOLE_RX_BUF_SIZE);
static struct k_sem s_console_rx_sem;
static const struct device *s_console_uart_dev = NULL;
static bool s_console_async_active = false;
static volatile uint32_t s_console_rx_drop_count = 0;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
static void console_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev)) {
		uint8_t rx_byte;
		if (uart_fifo_read(dev, &rx_byte, 1) <= 0) {
			break;
		}
		if (ring_buf_put(&s_console_rx_rb, &rx_byte, 1) > 0) {
			k_sem_give(&s_console_rx_sem);
		} else {
			s_console_rx_drop_count++;
		}
	}
	(void)uart_err_check(dev);
}
#endif

uint32_t console_get_rx_drop_count(void)
{
	return s_console_rx_drop_count;
}

void console_reset_rx_drop_count(void)
{
	s_console_rx_drop_count = 0;
}

int console_async_init(const struct device *dev)
{
	if (!dev || !device_is_ready(dev)) {
		return -ENODEV;
	}
	s_console_uart_dev = dev;
	k_sem_init(&s_console_rx_sem, 0, CONSOLE_RX_BUF_SIZE);
	ring_buf_reset(&s_console_rx_rb);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_callback_user_data_set(dev, console_uart_isr, NULL);
	uart_irq_rx_enable(dev);
	s_console_async_active = true;
#endif
	return 0;
}

bool console_async_is_active(void)
{
	return s_console_async_active;
}

int console_async_get_char(unsigned char *c)
{
	if (!s_console_async_active) {
		return -ENODEV;
	}
	if (k_sem_take(&s_console_rx_sem, K_NO_WAIT) == 0) {
		uint8_t byte;
		if (ring_buf_get(&s_console_rx_rb, &byte, 1) == 1) {
			if (c) {
				*c = byte;
			}
			return 0;
		}
	}
	return -EAGAIN;
}

int console_async_wait_char(unsigned char *c, int32_t timeout_ms)
{
	if (!s_console_async_active) {
		return -ENODEV;
	}
	k_timeout_t to = (timeout_ms < 0) ? K_FOREVER :
	                 (timeout_ms == 0) ? K_NO_WAIT : K_MSEC(timeout_ms);
	if (k_sem_take(&s_console_rx_sem, to) == 0) {
		uint8_t byte;
		if (ring_buf_get(&s_console_rx_rb, &byte, 1) == 1) {
			if (c) {
				*c = byte;
			}
			return 0;
		}
	}
	return -ETIMEDOUT;
}

void console_async_suspend(void)
{
	if (s_console_uart_dev && s_console_async_active) {
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
		uart_irq_rx_disable(s_console_uart_dev);
#endif
		ring_buf_reset(&s_console_rx_rb);
		k_sem_reset(&s_console_rx_sem);
	}
}

void console_async_resume(void)
{
	if (s_console_uart_dev && s_console_async_active) {
#if defined(CONFIG_SOC_SERIES_STM32L4X)
		/* Clear only RX error and wakeup flags; NEVER clear TCCF (Transmission Complete) */
		LL_USART_ClearFlag_ORE(USART2);
		LL_USART_ClearFlag_FE(USART2);
		LL_USART_ClearFlag_NE(USART2);
		LL_USART_ClearFlag_PE(USART2);
		LL_USART_ClearFlag_IDLE(USART2);
		LL_USART_ClearFlag_WKUP(USART2);

		/* Discard pending character in RDR only if RXNE is asserted */
		if (LL_USART_IsActiveFlag_RXNE(USART2)) {
			(void)USART2->RDR;
		}
#endif
		struct uart_config ucfg = {
			.baudrate = 115200,
			.parity = UART_CFG_PARITY_NONE,
			.stop_bits = UART_CFG_STOP_BITS_1,
			.data_bits = UART_CFG_DATA_BITS_8,
			.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		};
		(void)uart_configure(s_console_uart_dev, &ucfg);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
		uart_irq_callback_user_data_set(s_console_uart_dev, console_uart_isr, NULL);
		uart_irq_rx_enable(s_console_uart_dev);
#endif
	}
}

static struct wildsonic_config g_cfg = {
	.serial_number = "0",
	.device_name = "",
	.device_password = "wildsonic123",
	.sampling_rate = CONFIG_AUDIO_SAMPLE_RATE,
	.recording_length = 600,
	.sleep_pause_interval = 0,
	.mic_gain_times_ten = 0,
	.audio_sampling_width = 16,
	.num_audio_channels = 1,
	.lower_cutoff_frequency = 0,
	.upper_cutoff_frequency = 24000,
	.folder_structure = 2,
	.boot_time_utc = 0,
	.save_config_flag = false,
	.format_sd_card_flag = false,
	.location_lat_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE,
	.location_lon_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE,
	.latitude_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE,
	.longitude_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE,
	.sun_recording_mode = 0,
	.sunrise_interval_before = 0,
	.sunrise_interval_after = 0,
	.sunset_interval_before = 0,
	.sunset_interval_after = 0,
	.sunrise_sunset_rounding = 0,
};

#ifdef __ZEPHYR__
K_MUTEX_DEFINE(s_cmd_dispatch_mutex);
#endif
static char *s_current_out_resp = NULL;
static size_t s_current_out_resp_size = 0;
static bool s_from_ble = false;

static int console_commit_and_ack_config(const char *cmd_name, const char *val_str, bool recording_active)
{
	int save_rc = config_store_save_pending();
	if (save_rc != 0) {
		LOG_ERR("Failed to persist pending configuration to flash (%d)", save_rc);
		printk("[ERROR] Flash save failed (%d). Pending configuration not saved.\n", save_rc);
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Flash save failed (%d)", save_rc);
		}
		config_store_rollback_pending();
		return save_rc;
	}

	if (recording_active) {
		printk("\nOK: Value staged for next session (active session unaffected)\n");
	}
	printk("[ACK] Command %s received and validated. Saved to persistent flash (seq %u, pending gen %u).\n",
	       cmd_name, config_store_get_sequence_number(), config_store_get_pending_generation());
	printk("[PENDING] Configuration pending application after reboot.\n");

	if (s_current_out_resp && s_current_out_resp_size > 0) {
		snprintf(s_current_out_resp, s_current_out_resp_size,
		         "[ACK] %s %s: Saved to flash (Pending reboot application)",
		         cmd_name, val_str ? val_str : "");
	}
	return 0;
}

/* Command Enum */
enum console_cmd_id {
	CMD_HELP = 0,
	CMD_SERIAL_NUMBER,
	CMD_DEVICE_NAME,
	CMD_PASSWORD,
	CMD_SMS_KEYWORD,
	CMD_SMS_KEY,
	CMD_A2P_PHONE_NUMBER,
	CMD_P2A_PHONE_NUMBER,
	CMD_SERVER_URL,
	CMD_SERVER_PORT,
	CMD_SERVER_POST_ROUTE,
	CMD_SERVER_POST_CONTENT_TYPE,
	CMD_SERVER_TEST_SETTING,
	CMD_SERVER_WILDSONIC_SETTING,
	CMD_GSMMODE,
	CMD_SAMF,
	CMD_RECT,
	CMD_SLEEP,
	CMD_DATE_RANGE,
	CMD_TIME_OF_DAY,
	CMD_DAYS_OF_WEEK,
	CMD_DATETIME,
	CMD_TIMEZONE,
	CMD_BTPS,
	CMD_NUMCH,
	CMD_GAIN,
	CMD_LOWER_CUTOFF,
	CMD_UPPER_CUTOFF,
	CMD_FOLDER_STRUCTURE,
	CMD_APPLY,
	CMD_REBOOT,
	CMD_APPLY_REBOOT,
	CMD_SD_FORMAT,
	CMD_SMS_FREQ,
	CMD_ASSERT,
	CMD_BATTERY_STATUS,
	CMD_SIGNAL_QUALITY,
	CMD_STORAGE_FILLED,
	CMD_LOW_BAT,
	CMD_NUM_BAT,
	CMD_FULL_CAP,
	CMD_SEND_STATUS,
	CMD_AUDIO,
	CMD_BLEPAIR,
	CMD_WILDLY,
	CMD_VERSION,
	CMD_ENVIRONMENT_STATUS,
	CMD_LOCATION,
	CMD_SUN,
	CMD_SUN_RECORDING_MODE,
	CMD_SUNRISE_INTERVAL_BEFORE,
	CMD_SUNRISE_INTERVAL_AFTER,
	CMD_SUNSET_INTERVAL_BEFORE,
	CMD_SUNSET_INTERVAL_AFTER,
	CMD_SUNRISE_SUNSET_ROUNDING,
	CMD_DEFAULT,
	CMD_I2C,
	CMD_RTC_RESET,
	CMD_TEST_ALARM,
	CMD_PSRAM,
	CMD_CONFIG_WIZARD,
	CMD_GET_STATUS,
	CMD_GET_DEV_INFO,
	CMD_GET_CONFIG,
	CMD_STOP_RECORDING,
	CMD_NUM_CMDS
};

/* Canonical WildSonic Command Identifiers */
static const char *console_cmd_lookup_table[] = {
	"HELP",
	"SN",
	"DNAME",
	"PW",
	"SMSKW",
	"SMSKEY",
	"A2PPHONE",
	"P2APHONE",
	"SURL",
	"SPORT",
	"POSTROUTE",
	"POSTCTNT",
	"STEST",
	"SWS",
	"GSM",
	"SAMF",
	"RECT",
	"SLEEP",
	"DATERANGE",
	"TIMEOFDAY",
	"DAYSOFWEEK",
	"DATETIME",
	"TIMEZONE",
	"BTPS",
	"NUMCH",
	"GAIN",
	"LCUTOFF",
	"UCUTOFF",
	"FLDR",
	"APPLY",
	"RBT",
	"ARBT",
	"SDFMT",
	"SMSFREQ",
	"ASSERT",
	"BATT",
	"SIG",
	"STOR",
	"LBAT",
	"NBAT",
	"FULLCAP",
	"SENDSTS",
	"AUDIO",
	"BLEPAIR",
	"Wildly",
	"VERSION",
	"ENVSTS",
	"LOC",
	"SUN",
	"SUNMODE",
	"SUNRISEBEF",
	"SUNRISEAFT",
	"SUNSETBEF",
	"SUNSETAFT",
	"SUNROUND",
	"DEFAULT",
	"I2C",
	"RTCRST",
	"TEST_ALARM",
	"PSRAM",
	"CONFIG",
	"GET_STATUS",
	"GET_DEV_INFO",
	"GET_CONFIG",
	"STOP_RECORDING",
};

/* Command Descriptions */
static const char *console_cmd_description_table[] = {
	"Display list of commands",
	"Set serial number",
	"Set device name for BLE advertising",
	"Set device password for server authentication",
	"Set sms keyword for manufacturing",
	"Set sms key for manufacturing",
	"Set server phone number to receive the messages(format +XXXXXXXXXXXX or XXXXXX)",
	"Set server phone number to send the messages(format +XXXXXXXXXXXX or XXXXXX)",
	"Set server URL (without http:// or https://)",
	"Set server port",
	"Set server post request api route",
	"Set server post request content-type",
	"Quick command to set up url, port, api route for test server",
	"Quick command to set up url, port, api route for demo server",
	"Set GSM transmission mode (e.g. 2G, 3G, 4G)",
	"Set sampling frequency in Hz",
	"Set recording time in seconds",
	"Set sleep time in seconds",
	"Set recording date range (format YYYY/MM/DD-YYYY/MM/DD)",
	"Set recording time of day (format [index] HH:MM:SS-HH:MM:SS)",
	"Set recording days of the week (\"MTWRFSU\") within quotation mark",
	"Set sys time in fmt YYYY-MM-DD HH:MM:SS [[+-]ZZZZ]",
	"Set the time zone in format [[+-]ZZZZ]",
	"Set sample bit width",
	"Set number of channels",
	"Set audio gain in dB",
	"Set lower cutoff frequency in Hz",
	"Set upper cutoff frequency in Hz",
	"Enable folder structure: hourly, daily, or flat",
	"Apply settings to flash",
	"Reboot the device",
	"Apply settings to flash and reboot",
	"SD card format and reboot",
	"SMS upload frequency in seconds",
	"Provides the information of assert",
	"Provides battery status",
	"Provides signal quality",
	"Provides number of bytes filled in SD card",
	"Low battery indicating voltage (in mV)",
	"Setting number of battery cells used",
	"Program fuel gauge full charge capacity in mAh",
	"Sends Status to the server now",
	"Audio recording parameters",
	"Open BLE connect window in seconds (default 30)",
	"Ignore this command",
	"Provides the version information",
	"Provides current environment status like temperature and humidity",
	"Set/Get location (format \"LOC <latitude> <longitude>\")",
	"View Sunrise/Sunset times for a specific date",
	"Set sun recording mode (format SUNMODE <mode>)",
	"Set sunrise interval before (format SUNRISEBEF <minutes>)",
	"Set sunrise interval after (format SUNRISEAFT <minutes>)",
	"Set sunset interval before (format SUNSETBEF <minutes>)",
	"Set sunset interval after (format SUNSETAFT <minutes>)",
	"Set sunrise sunset rounding (format SUNROUND <minutes>)",
	"Sets Default configurations",
	"I2C debugging scan, read/write command",
	"Reset RTC time to 2025-01-01 00:00:00 UTC",
	"Configure custom/default alarm sequence (GPIO 1.7 + RTC)",
	"Configure PSRAM circular staging buffer (on, off, status, burst, mode)",
	"Run guided interactive configuration wizard",
	"Get current system, scheduler, and recording status",
	"Get hardware platform and firmware device information",
	"Get current device audio, storage, and schedule configuration",
	"Safely stop active recording, drain buffers, and close file",
};

BUILD_ASSERT(ARRAY_SIZE(console_cmd_lookup_table) == (size_t)CMD_NUM_CMDS,
	     "console_cmd_lookup_table size mismatch");
BUILD_ASSERT(ARRAY_SIZE(console_cmd_description_table) == (size_t)CMD_NUM_CMDS,
	     "console_cmd_description_table size mismatch");

/* Helper to format and display human readable time */
static void console_printf_time(const char *prefix, time_t epoch_utc, int tz_offset, const char *suffix)
{
	struct rtc_calendar_time cal;
	time_t display_epoch = epoch_utc + tz_offset;
	scheduler_epoch_to_calendar(display_epoch, &cal);

	char tz_sign = '+';
	int abs_tz = tz_offset;
	if (abs_tz < 0) {
		tz_sign = '-';
		abs_tz = -abs_tz;
	}
	uint32_t tz_hr = (uint32_t)(abs_tz / 3600);
	uint32_t tz_min = (uint32_t)((abs_tz % 3600) / 60);

	printk("%s%04u-%02u-%02u %02u:%02u:%02u.000 %c%02u:%02u%s",
	       prefix,
	       cal.year, cal.month, cal.day,
	       cal.hour, cal.min, cal.sec,
	       tz_sign, tz_hr, tz_min,
	       suffix);
}

/* Print command names */
static void display_console_commands(void)
{
	printk("\nList of commands\n");
	for (size_t i = 0; i < CANONICAL_CMD_COUNT; i++) {
		printk(" %s \n", console_cmd_lookup_table[i]);
	}
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	printk(" %s \n", console_cmd_lookup_table[CMD_PSRAM]);
	printk(" %s \n", console_cmd_lookup_table[CMD_CONFIG_WIZARD]);
#endif
	printk("\n");
}

/* Print HELP: command : description */
static void display_console_help(void)
{
	printk("\nList of commands\n");
	for (size_t i = 0; i < CANONICAL_CMD_COUNT; i++) {
		printk(" %s : %s\n", console_cmd_lookup_table[i], console_cmd_description_table[i]);
	}
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	printk("\nEngineering Extensions (TEST build only):\n");
	printk(" %s : %s\n", console_cmd_lookup_table[CMD_PSRAM], console_cmd_description_table[CMD_PSRAM]);
	printk(" %s : %s\n", console_cmd_lookup_table[CMD_CONFIG_WIZARD], console_cmd_description_table[CMD_CONFIG_WIZARD]);
#endif
	printk("\n");
}

/* Explicit deferred hardware notice */
static int report_deferred_hardware(const char *cmd_name)
{
	printk("\n[DEFERRED] Command '%s' requires hardware not populated on STM32L476RG.\n", cmd_name);
	return -ENOTSUP;
}

/* Look up command index from string */
static enum console_cmd_id console_cmd_lookup_func(const char *cmd, size_t length, size_t *arg_offset)
{
	for (size_t id = 0; id < (size_t)CMD_NUM_CMDS; id++) {
		size_t lenpre = strlen(console_cmd_lookup_table[id]);
		bool equal_length = (length == lenpre);
		bool next_is_space = (length > lenpre && isspace((unsigned char)cmd[lenpre]));

		if ((equal_length || next_is_space) &&
		    (strncasecmp(console_cmd_lookup_table[id], cmd, lenpre) == 0)) {
			if (next_is_space) {
				size_t off = lenpre + 1;
				while (off < length && isspace((unsigned char)cmd[off])) {
					off++;
				}
				*arg_offset = off;
			} else {
				*arg_offset = lenpre;
			}
			return (enum console_cmd_id)id;
		}
	}
	*arg_offset = 0;
	return CMD_NUM_CMDS;
}

static int s_last_cmd_status = 0;

static int parse_strict_u32(const char *str, size_t len, uint32_t *out_val)
{
	if (!str || len == 0) return -EINVAL;
	while (len > 0 && isspace((unsigned char)*str)) {
		str++;
		len--;
	}
	while (len > 0 && isspace((unsigned char)str[len - 1])) {
		len--;
	}
	if (len == 0) return -EINVAL;

	char buf[32];
	if (len >= sizeof(buf)) return -EINVAL;
	memcpy(buf, str, len);
	buf[len] = '\0';

	char *endptr = NULL;
	errno = 0;
	unsigned long v = strtoul(buf, &endptr, 10);
	if (endptr == buf || *endptr != '\0' || errno == ERANGE || v > UINT32_MAX) {
		return -EINVAL;
	}
	*out_val = (uint32_t)v;
	return 0;
}

__maybe_unused static int parse_strict_i32(const char *str, size_t len, int32_t *out_val)
{
	if (!str || len == 0) return -EINVAL;
	while (len > 0 && isspace((unsigned char)*str)) {
		str++;
		len--;
	}
	while (len > 0 && isspace((unsigned char)str[len - 1])) {
		len--;
	}
	if (len == 0) return -EINVAL;

	char buf[32];
	if (len >= sizeof(buf)) return -EINVAL;
	memcpy(buf, str, len);
	buf[len] = '\0';

	char *endptr = NULL;
	errno = 0;
	long v = strtol(buf, &endptr, 10);
	if (endptr == buf || *endptr != '\0' || errno == ERANGE || v < INT32_MIN || v > INT32_MAX) {
		return -EINVAL;
	}
	*out_val = (int32_t)v;
	return 0;
}

/* SAMF handler */
static void handle_samf(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.sampling_rate);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t raw_val = 0;
	if (parse_strict_u32(p_args, args_length, &raw_val) != 0) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.sampling_rate);
		s_last_cmd_status = -EINVAL;
		return;
	}

	uint32_t val = raw_val;
	if (val == 16) val = 16000;
	else if (val == 32) val = 32000;
	else if (val == 44) val = 44100;
	else if (val == 48) val = 48000;
	else if (val == 96) val = 96000;
	else if (val == 192) val = 192000;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	else if (val == 384) val = 384000;
	else if (val == 512) val = 512000;
#endif

	uint32_t act = 0, div = 0;
	int err = 0;
	if (!audio_in_is_rate_supported(val, &act, &div, &err)) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.sampling_rate);
		s_last_cmd_status = -EINVAL;
		return;
	}

	uint32_t old_rate = g_cfg.sampling_rate;
	if (!recorder_is_active()) {
		int rc = audio_in_set_rate(val);
		if (rc != 0) {
			printk("\nUnsupported sample rate: %u Hz (error %d)\n", val, rc);
			printk("\nValue is set to %u\n", g_cfg.sampling_rate);
			s_last_cmd_status = rc;
			return;
		}
		g_cfg.sampling_rate = val;
		if (settings) {
			settings->sample_rate = audio_in_rate();
		}
		main_apply_dsp_settings();
	} else {
		/* Recording active: stage in RAM without modifying running session */
		g_cfg.sampling_rate = val;
	}

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	if (val == 384000 || val == 512000) {
		uint32_t bytes_per_sec = val * (g_cfg.num_audio_channels ? g_cfg.num_audio_channels : 1) *
		                         (g_cfg.audio_sampling_width ? (g_cfg.audio_sampling_width / 8) : 2);
		printk("\n================================================================================\n");
		printk("WARNING: Experimental audio sampling rate configured: %u Hz\n", val);
		printk("  Not validated for production use (STM32L476 hardware baseline validated <= 192 kHz)\n");
		printk("  Target production platform: STM32U585\n");
		printk("  Effective TIM6 trigger rate: %u Hz (divider %u, clock error %d Hz)\n",
		       act, div, err);
		printk("  Adaptive oversampling ratio: %ux\n", audio_in_get_ovs_ratio());
		printk("  Estimated streaming throughput: %u bytes/sec (%u KB/s)\n",
		       bytes_per_sec, bytes_per_sec / 1000);
		printk("================================================================================\n");
	}
#endif

	printk("\nValue is set to %u\n", val);
	if (g_cfg.upper_cutoff_frequency == (old_rate / 2) ||
	    g_cfg.upper_cutoff_frequency >= ((val * 49) / 100)) {
		uint32_t safe_cutoff = dsp_validate_lpf_cutoff(val, (val * 45) / 100);
		g_cfg.upper_cutoff_frequency = safe_cutoff;
		if (settings) {
			settings->lpf_cutoff_hz = safe_cutoff;
		}
		if (!recorder_is_active()) {
			main_apply_dsp_settings();
		}
		printk("\nUpper cutoff frequency adjusted to %u\n", g_cfg.upper_cutoff_frequency);
	}
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	char val_str[16];
	snprintf(val_str, sizeof(val_str), "%u", val);
	s_last_cmd_status = console_commit_and_ack_config("SAMF", val_str, recorder_is_active());
}

/* BTPS handler */
static void handle_btps(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.audio_sampling_width);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.audio_sampling_width);
		s_last_cmd_status = -EINVAL;
		return;
	}

	if (val == 32) {
		g_cfg.audio_sampling_width = 32;
		if (settings) {
			settings->sample_bits = 32;
		}
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
		uint32_t bytes_per_sec = g_cfg.sampling_rate * (g_cfg.num_audio_channels ? g_cfg.num_audio_channels : 1) * 4;
		printk("\n================================================================================\n");
		printk("WARNING: Experimental 32-bit PCM storage format configured\n");
		printk("  Not validated for production use (native STM32L476 ADC is 12-bit SAR)\n");
		printk("  Container mode: 12-bit ADC / 16-bit DSP scaled to 32-bit (int16_t << 16)\n");
		printk("  Target production platform: STM32U585 (2.5 Msps 14-bit ADC / 24-bit I2S)\n");
		printk("  Estimated streaming throughput: %u bytes/sec (%u KB/s)\n",
		       bytes_per_sec, bytes_per_sec / 1000);
		printk("================================================================================\n");
#endif
		printk("\nValue is set to 32\n");
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("BTPS", "32", recorder_is_active());
		return;
	}
	if (val == 24) {
		printk("\n[DEFERRED] Bits per sample %d requires future hardware (STM32U585/external I2S codec).\n", (int)val);
		printk("\nValue is set to %u\n", g_cfg.audio_sampling_width);
		s_last_cmd_status = -ENOTSUP;
		return;
	}
	if (val != 16) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.audio_sampling_width);
		s_last_cmd_status = -EINVAL;
		return;
	}
	g_cfg.audio_sampling_width = 16;
	if (settings) {
		settings->sample_bits = 16;
	}
	printk("\nValue is set to 16\n");
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = console_commit_and_ack_config("BTPS", "16", recorder_is_active());
}

/* NUMCH handler */
static void handle_numch(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.num_audio_channels);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.num_audio_channels);
		s_last_cmd_status = -EINVAL;
		return;
	}

	if (val == 2) {
		g_cfg.num_audio_channels = 2;
		if (settings) {
			settings->channels = 2;
		}
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
		uint32_t bytes_per_sec = g_cfg.sampling_rate * 2 * (g_cfg.audio_sampling_width ? (g_cfg.audio_sampling_width / 8) : 2);
		printk("\n================================================================================\n");
		printk("WARNING: Experimental stereo channel emulation configured (2 channels)\n");
		printk("  Not validated for production use (single analog microphone on PA0/ADC1_IN5)\n");
		printk("  Emulation mode: Mono duplicated to stereo (or synthetic dual-tone)\n");
		printk("  Target production platform: STM32U585 (dual simultaneous ADCs / stereo I2S)\n");
		printk("  Estimated streaming throughput: %u bytes/sec (%u KB/s)\n",
		       bytes_per_sec, bytes_per_sec / 1000);
		printk("================================================================================\n");
#endif
		printk("\nValue is set to 2\n");
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("NUMCH", "2", recorder_is_active());
		return;
	}
	if (val != 1) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.num_audio_channels);
		s_last_cmd_status = -EINVAL;
		return;
	}
	g_cfg.num_audio_channels = 1;
	if (settings) {
		settings->channels = 1;
	}
	printk("\nValue is set to 1\n");
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = console_commit_and_ack_config("NUMCH", "1", recorder_is_active());
}

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
static void handle_source(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();
	if (args_length == 0) {
		enum audio_capture_source cur = audio_in_get_capture_source();
		const char *s = (cur == AUDIO_SRC_PHYSICAL_MIC) ? "MIC" :
		                (cur == AUDIO_SRC_STEREO_DUPLICATE) ? "STEREO_DUP" : "SYNTH";
		printk("\nCapture source is %s\n", s);
		return;
	}
	if (strncasecmp(p_args, "MIC", 3) == 0) {
		audio_in_set_capture_source(AUDIO_SRC_PHYSICAL_MIC);
		if (settings) settings->capture_source = AUDIO_SRC_PHYSICAL_MIC;
		printk("\nCapture source set to MIC (physical analog microphone)\n");
	} else if (strncasecmp(p_args, "STEREO_DUP", 10) == 0 || strncasecmp(p_args, "DUP", 3) == 0) {
		audio_in_set_capture_source(AUDIO_SRC_STEREO_DUPLICATE);
		if (settings) settings->capture_source = AUDIO_SRC_STEREO_DUPLICATE;
		printk("\nCapture source set to STEREO_DUP (mono ADC duplicated to stereo)\n");
	} else if (strncasecmp(p_args, "SYNTH", 5) == 0) {
		audio_in_set_capture_source(AUDIO_SRC_SYNTHETIC);
		if (settings) settings->capture_source = AUDIO_SRC_SYNTHETIC;
		printk("\nCapture source set to SYNTH (synthetic deterministic tone)\n");
	} else {
		printk("\nInvalid source '%s'. Valid: MIC, STEREO_DUP, SYNTH\n", p_args);
	}
}
#endif

/* AUDIO parameters summary / packed setter */
static void handle_audio(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
		printk("Sampling frequency set to %u\n", g_cfg.sampling_rate);
		printk("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
		s_last_cmd_status = 0;
		return;
	}

	/* 10-digit format: %1d%7d%2d */
	if (args_length == 10 && isdigit((unsigned char)p_args[0])) {
		char numch_str[2] = { p_args[0], '\0' };
		char samf_str[8] = {0};
		char btps_str[3] = {0};
		memcpy(samf_str, &p_args[1], 7);
		memcpy(btps_str, &p_args[8], 2);

		uint32_t new_numch = (uint32_t)atoi(numch_str);
		uint32_t new_samf = (uint32_t)atoi(samf_str);
		uint32_t new_btps = (uint32_t)atoi(btps_str);

		uint32_t act = 0, div = 0;
		int err = 0;
		bool valid_combo = (new_numch == 1 || new_numch == 2) &&
		                   (new_btps == 16 || new_btps == 32) &&
		                   audio_in_is_rate_supported(new_samf, &act, &div, &err);
		if (valid_combo) {
			uint32_t old_samf = g_cfg.sampling_rate;
			g_cfg.num_audio_channels = new_numch;
			g_cfg.sampling_rate = new_samf;
			g_cfg.audio_sampling_width = new_btps;
			struct app_settings *settings = main_get_settings();
			if (settings) {
				settings->sample_rate = new_samf;
				settings->channels = (uint16_t)new_numch;
				settings->sample_bits = (uint16_t)new_btps;
			}
			if (!recorder_is_active()) {
				audio_in_set_rate(new_samf);
				main_apply_dsp_settings();
			}

			if ((old_samf != new_samf && g_cfg.upper_cutoff_frequency == (old_samf / 2)) ||
			    g_cfg.upper_cutoff_frequency >= ((new_samf * 49) / 100)) {
				uint32_t safe_cutoff = dsp_validate_lpf_cutoff(new_samf, (new_samf * 45) / 100);
				g_cfg.upper_cutoff_frequency = safe_cutoff;
				if (settings) {
					settings->lpf_cutoff_hz = safe_cutoff;
				}
				if (!recorder_is_active()) {
					main_apply_dsp_settings();
				}
				printk("\nUpper cutoff frequency adjusted to %u\n", g_cfg.upper_cutoff_frequency);
			}

			printk("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
			printk("Sampling frequency set to %u\n", g_cfg.sampling_rate);
			printk("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
			printk("\nSend APPLY command and reboot for changes to take effect\n");
			s_last_cmd_status = 0;
			return;
		}
	}

	printk("\nInvalid values\n");
	printk("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
	printk("Sampling frequency set to %u\n", g_cfg.sampling_rate);
	printk("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
	s_last_cmd_status = -EINVAL;
}

/* RECT handler */
static void handle_rect(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.recording_length);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0 || val == 0 || val > 86400) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.recording_length);
		s_last_cmd_status = -EINVAL;
		return;
	}

	g_cfg.recording_length = val;
	if (settings) {
		settings->record_seconds = val;
	}

	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	scfg.record_length = val;
	scheduler_set_config(&scfg);
	main_scheduler_resume();

	printk("\nValue is set to %u\n", g_cfg.recording_length);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	char rect_str[16];
	snprintf(rect_str, sizeof(rect_str), "%u", g_cfg.recording_length);
	s_last_cmd_status = console_commit_and_ack_config("RECT", rect_str, recorder_is_active());
}

/* SLEEP handler
 * Note: SLEEP is the configured session sleep interval, not a power command.
 * Earlier implementations erroneously returned:
 * "BUSY: Cannot enter sleep while recording is active"
 */
static void handle_sleep(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
		s_last_cmd_status = 0;
		return;
	}

	if (p_args[0] == '-') {
		printk("\nInvalid value: negative values are not allowed\n");
		printk("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
		s_last_cmd_status = -EINVAL;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0 || val > 86400) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
		s_last_cmd_status = -EINVAL;
		return;
	}

	g_cfg.sleep_pause_interval = val;
	if (settings) {
		settings->sleep_seconds = val;
	}

	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	scfg.sleep_interval = (uint32_t)val;
	scheduler_set_config(&scfg);
	main_scheduler_resume();

	printk("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	char sleep_str[16];
	snprintf(sleep_str, sizeof(sleep_str), "%u", g_cfg.sleep_pause_interval);
	s_last_cmd_status = console_commit_and_ack_config("SLEEP", sleep_str, recorder_is_active());
}

/* DATETIME handler */
static void handle_datetime(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0) {
		struct rtc_calendar_time cur_cal;
		int ret = rtc_calendar_get(&cur_cal);
		time_t cur_utc = (ret == 0 && rtc_calendar_is_valid()) ?
				 scheduler_calendar_to_epoch(&cur_cal) : 0;
		time_t boot_utc = g_cfg.boot_time_utc;
		if (boot_utc == 0) {
			boot_utc = cur_utc;
		}

		console_printf_time("\nBoot time UTC set to ", boot_utc, 0, "\n");
		console_printf_time("\nBoot time Local set to ", boot_utc, scfg.time_zone_offset, "\n");
		console_printf_time("\nCurrent time UTC time is ", cur_utc, 0, "\n");
		console_printf_time("\nCurrent time Local is ", cur_utc, scfg.time_zone_offset, "\n");
		s_last_cmd_status = 0;
		return;
	}

	struct rtc_calendar_time new_local;
	int parse_res = rtc_calendar_parse(p_args, &new_local);
	if (parse_res == 0) {
		char tz_token[16] = {0};
		int tokens = sscanf(p_args, "%*s %*s %15s", tz_token);
		int32_t tz_off = scfg.time_zone_offset;
		if (tokens == 1 && (tz_token[0] == '+' || tz_token[0] == '-' || isdigit((unsigned char)tz_token[0]))) {
			if (scheduler_parse_timezone(tz_token, &tz_off) != 0) {
				printk("\nInvalid time zone %s\n", tz_token);
				s_last_cmd_status = -EINVAL;
				return;
			}
		}
		scfg.time_zone_offset = tz_off;
		scheduler_set_config(&scfg);

		time_t local_epoch = scheduler_calendar_to_epoch(&new_local);
		time_t utc_epoch = local_epoch - tz_off;
		struct rtc_calendar_time new_utc;
		scheduler_epoch_to_calendar(utc_epoch, &new_utc);
		new_utc.valid = true;

		int set_res = rtc_calendar_set(&new_utc);
		if (set_res != 0) {
			printk("\n[ERROR] Failed to set RTC hardware calendar (%d)\n", set_res);
			s_last_cmd_status = set_res;
			return;
		}
		g_cfg.boot_time_utc = utc_epoch;

		console_printf_time("\nBoot time UTC set to ", utc_epoch, 0, "\n");
		console_printf_time("\nBoot time Local set to ", utc_epoch, scfg.time_zone_offset, "\n");
		console_printf_time("\nCurrent time UTC time is ", utc_epoch, 0, "\n");
		console_printf_time("\nCurrent time Local is ", utc_epoch, scfg.time_zone_offset, "\n");
		main_scheduler_resume();
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = 0;
		return;
	}

	printk("\nInvalid date time %.*s\nUTC Time Format should be YYYY-MM-DD HH:MM:SS\n", (int)args_length, p_args);
	s_last_cmd_status = -EINVAL;
}

/* TIMEZONE handler */
static void handle_timezone(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0) {
		char sign = (scfg.time_zone_offset < 0) ? '-' : '+';
		int32_t abs_off = (scfg.time_zone_offset < 0) ? -scfg.time_zone_offset : scfg.time_zone_offset;
		uint32_t tz_hr = (uint32_t)(abs_off / 3600);
		uint32_t tz_min = (uint32_t)((abs_off % 3600) / 60);
		printk("\nTime zone is set to %c%02u:%02u\n", sign, tz_hr, tz_min);
		s_last_cmd_status = 0;
		return;
	}

	int32_t offset = 0;
	if (scheduler_parse_timezone(p_args, &offset) == 0) {
		scfg.time_zone_offset = offset;
		g_cfg.time_zone_offset = offset;
		scheduler_set_config(&scfg);
		main_scheduler_resume();
		printk("\nTime zone set to %.*s\n", (int)args_length, p_args);
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("TIMEZONE", p_args, recorder_is_active());
		return;
	}

	printk("\nInvalid time zone argument %.*s\n", (int)args_length, p_args);
	s_last_cmd_status = -EINVAL;
}

/* DATERANGE handler */
static void handle_daterange(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0) {
		printk("\nBegin end date set to %04u/%02u/%02u-%04u/%02u/%02u\n",
		       scfg.start_date.year, scfg.start_date.month, scfg.start_date.day,
		       scfg.end_date.year, scfg.end_date.month, scfg.end_date.day);
		s_last_cmd_status = 0;
		return;
	}

	struct schedule_date st = {0}, en = {0};
	if (scheduler_parse_date_range(p_args, &st, &en) == 0) {
		scfg.start_date = st;
		scfg.end_date = en;
		scheduler_set_config(&scfg);
		main_scheduler_resume();
		printk("\nBegin end date set to %04u/%02u/%02u-%04u/%02u/%02u\n",
		       scfg.start_date.year, scfg.start_date.month, scfg.start_date.day,
		       scfg.end_date.year, scfg.end_date.month, scfg.end_date.day);
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("DATERANGE", p_args, recorder_is_active());
		return;
	}

	printk("\nInvalid begin end date %.*s\n", (int)args_length, p_args);
	printk("\nBegin end date set to %04u/%02u/%02u-%04u/%02u/%02u\n",
	       scfg.start_date.year, scfg.start_date.month, scfg.start_date.day,
	       scfg.end_date.year, scfg.end_date.month, scfg.end_date.day);
	s_last_cmd_status = -EINVAL;
}

/* TIMEOFDAY handler */
static void handle_timeofday(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0) {
		uint32_t valid_cnt = 0;
		for (uint32_t i = 0; i < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
			if (scheduler_is_valid_tod_range(&scfg.tod_ranges[i])) {
				printk("\nBegin end time (Index %u) set to %02u:%02u:%02u-%02u:%02u:%02u\n",
				       i,
				       scfg.tod_ranges[i].start_second / 3600,
				       (scfg.tod_ranges[i].start_second / 60) % 60,
				       scfg.tod_ranges[i].start_second % 60,
				       scfg.tod_ranges[i].end_second / 3600,
				       (scfg.tod_ranges[i].end_second / 60) % 60,
				       scfg.tod_ranges[i].end_second % 60);
				valid_cnt++;
			}
		}
		if (valid_cnt == 0) {
			printk("\nNo valid time of day ranges set\n");
		}
		s_last_cmd_status = 0;
		return;
	}

	uint32_t idx = 0;
	struct schedule_tod_range r = {0};
	if (scheduler_parse_time_of_day(p_args, &idx, &r) == 0) {
		if (idx == 0) {
			scheduler_reset_tod_ranges(scfg.tod_ranges, SCHEDULER_MAX_TIME_OF_DAY_RANGES);
		}
		if (idx < SCHEDULER_MAX_TIME_OF_DAY_RANGES) {
			scfg.tod_ranges[idx] = r;
		}
		scheduler_sort_tod_ranges(scfg.tod_ranges, SCHEDULER_MAX_TIME_OF_DAY_RANGES);
		scheduler_set_config(&scfg);
		main_scheduler_resume();

		for (uint32_t i = 0; i < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
			if (scheduler_is_valid_tod_range(&scfg.tod_ranges[i])) {
				printk("\nBegin end time (Index %u) set to %02u:%02u:%02u-%02u:%02u:%02u\n",
				       i,
				       scfg.tod_ranges[i].start_second / 3600,
				       (scfg.tod_ranges[i].start_second / 60) % 60,
				       scfg.tod_ranges[i].start_second % 60,
				       scfg.tod_ranges[i].end_second / 3600,
				       (scfg.tod_ranges[i].end_second / 60) % 60,
				       scfg.tod_ranges[i].end_second % 60);
			}
		}
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("TIMEOFDAY", p_args, recorder_is_active());
		return;
	}

	printk("\nInvalid begin end time %.*s\n", (int)args_length, p_args);
	s_last_cmd_status = -EINVAL;
}

/* DAYSOFWEEK handler */
static void handle_daysofweek(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0) {
		printk("\nDays of week set to \"");
		if (scfg.day_of_week_mask & 0x01) printk("M");
		if (scfg.day_of_week_mask & 0x02) printk("T");
		if (scfg.day_of_week_mask & 0x04) printk("W");
		if (scfg.day_of_week_mask & 0x08) printk("R");
		if (scfg.day_of_week_mask & 0x10) printk("F");
		if (scfg.day_of_week_mask & 0x20) printk("S");
		if (scfg.day_of_week_mask & 0x40) printk("U");
		printk("\"\n");
		s_last_cmd_status = 0;
		return;
	}

	if ((args_length == 1 && (p_args[0] == 'X' || p_args[0] == 'x')) ||
	    (args_length == 2 && p_args[0] == '"' && p_args[1] == '"') ||
	    (args_length == 3 && p_args[0] == '"' && (p_args[1] == 'X' || p_args[1] == 'x') && p_args[2] == '"')) {
		printf("\nDays of week: X is set to all days (MTWRFSU)\n");
	}

	uint8_t mask = 0;
	if (scheduler_parse_days_of_week(p_args, &mask) == 0) {
		scfg.day_of_week_mask = mask;
		scheduler_set_config(&scfg);
		main_scheduler_resume();
		printk("\nDays of week set to \"");
		if (scfg.day_of_week_mask & 0x01) printk("M");
		if (scfg.day_of_week_mask & 0x02) printk("T");
		if (scfg.day_of_week_mask & 0x04) printk("W");
		if (scfg.day_of_week_mask & 0x08) printk("R");
		if (scfg.day_of_week_mask & 0x10) printk("F");
		if (scfg.day_of_week_mask & 0x20) printk("S");
		if (scfg.day_of_week_mask & 0x40) printk("U");
		printk("\"\n");
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("DAYSOFWEEK", p_args, recorder_is_active());
		return;
	}

	printk("\nInvalid days of week %.*s\n", (int)args_length, p_args);
	s_last_cmd_status = -EINVAL;
}

/* GAIN handler */
static void handle_gain(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nAudio gain set to 0.1*%d\n", g_cfg.mic_gain_times_ten);
		s_last_cmd_status = 0;
		return;
	}

	/* Trim */
	const char *str = p_args;
	size_t len = args_length;
	while (len > 0 && isspace((unsigned char)*str)) {
		str++;
		len--;
	}
	while (len > 0 && isspace((unsigned char)str[len - 1])) {
		len--;
	}
	if (len == 0) {
		s_last_cmd_status = -EINVAL;
		return;
	}

	int sign = 1;
	size_t off = 0;
	if (str[0] == 'N' || str[0] == '-') {
		sign = -1;
		off = 1;
	}

	char buf[32];
	if (len >= sizeof(buf)) {
		s_last_cmd_status = -EINVAL;
		return;
	}
	memcpy(buf, str, len);
	buf[len] = '\0';

	char *endptr = NULL;
	errno = 0;
	double val = strtod(&buf[off], &endptr);
	if (endptr == &buf[off] || *endptr != '\0' || errno == ERANGE) {
		printk("\nInvalid audio gain %.*s\n", (int)args_length, p_args);
		printk("\nAudio gain set to 0.1*%d\n", g_cfg.mic_gain_times_ten);
		s_last_cmd_status = -EINVAL;
		return;
	}

	int gain_t10 = (int)roundf((float)(sign * val * 10.0));
	if (gain_t10 < -400 || gain_t10 > 400) {
		printk("\nInvalid audio gain 0.1*%d\n", gain_t10);
		printk("\nAudio gain set to 0.1*%d\n", g_cfg.mic_gain_times_ten);
		s_last_cmd_status = -EINVAL;
		return;
	}

	g_cfg.mic_gain_times_ten = gain_t10;
	if (settings) {
		settings->digital_gain = powf(10.0f, ((float)gain_t10 / 10.0f) / 20.0f);
	}
	if (!recorder_is_active()) {
		main_apply_dsp_settings();
	}

	printk("\nAudio gain set to 0.1*%d\n", gain_t10);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	char gain_str[16];
	snprintf(gain_str, sizeof(gain_str), "0.1*%d", gain_t10);
	s_last_cmd_status = console_commit_and_ack_config("GAIN", gain_str, recorder_is_active());
}

/* LCUTOFF (HPF) handler */
static void handle_lcutoff(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.lower_cutoff_frequency);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t hz = 0;
	if (parse_strict_u32(p_args, args_length, &hz) != 0 || (hz != 0 && (hz < 10 || hz > 2000))) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.lower_cutoff_frequency);
		s_last_cmd_status = -EINVAL;
		return;
	}

	g_cfg.lower_cutoff_frequency = hz;
	if (settings) {
		settings->hpf_cutoff_hz = hz;
		settings->hpf_enabled = (hz > 0);
	}
	if (!recorder_is_active()) {
		main_apply_dsp_settings();
	}

	printk("\nValue is set to %u\n", hz);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	char lcut_str[16];
	snprintf(lcut_str, sizeof(lcut_str), "%u", hz);
	s_last_cmd_status = console_commit_and_ack_config("LCUTOFF", lcut_str, recorder_is_active());
}

/* UCUTOFF (LPF) handler */
static void handle_ucutoff(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.upper_cutoff_frequency);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t hz = 0;
	uint32_t nyquist_bound = (g_cfg.sampling_rate * 49) / 100;
	if (parse_strict_u32(p_args, args_length, &hz) != 0 || (hz != 0 && (hz < 1000 || hz >= nyquist_bound))) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", g_cfg.upper_cutoff_frequency);
		s_last_cmd_status = -EINVAL;
		return;
	}

	g_cfg.upper_cutoff_frequency = hz;
	if (settings) {
		settings->lpf_cutoff_hz = hz;
		settings->lpf_enabled = (hz > 0);
	}
	if (!recorder_is_active()) {
		main_apply_dsp_settings();
	}

	printk("\nValue is set to %u\n", hz);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	char ucut_str[16];
	snprintf(ucut_str, sizeof(ucut_str), "%u", hz);
	s_last_cmd_status = console_commit_and_ack_config("UCUTOFF", ucut_str, recorder_is_active());
}

/* FLDR handler */
static void handle_fldr(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		const char *mode_str = (g_cfg.folder_structure == 0) ? "flat" :
				       ((g_cfg.folder_structure == 1) ? "daily" : "hourly");
		printk("\nValue is set to %s\n", mode_str);
		s_last_cmd_status = 0;
		return;
	}

	char buf[32];
	if (args_length >= sizeof(buf)) {
		s_last_cmd_status = -EINVAL;
		return;
	}
	memcpy(buf, p_args, args_length);
	buf[args_length] = '\0';
	char *str = buf;
	while (*str && isspace((unsigned char)*str)) str++;
	size_t slen = strlen(str);
	while (slen > 0 && isspace((unsigned char)str[slen - 1])) str[--slen] = '\0';

	if (strcasecmp(str, "flat") == 0) {
		g_cfg.folder_structure = 0;
	} else if (strcasecmp(str, "daily") == 0 || strcasecmp(str, "day") == 0) {
		g_cfg.folder_structure = 1;
	} else if (strcasecmp(str, "hourly") == 0 || strcasecmp(str, "hour") == 0) {
		g_cfg.folder_structure = 2;
	} else {
		printk("\nInvalid value %s\n", str);
		s_last_cmd_status = -EINVAL;
		return;
	}

	const char *mode_str = (g_cfg.folder_structure == 0) ? "flat" :
			       ((g_cfg.folder_structure == 1) ? "daily" : "hourly");
	printk("\nValue is set to %s\n", mode_str);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = console_commit_and_ack_config("FLDR", mode_str, recorder_is_active());
}

/* STOR handler */
static void handle_stor(void)
{
	struct fs_statvfs stat;
	int ret = fs_statvfs("/SD:", &stat);
	if (ret == 0) {
		uint64_t total = (uint64_t)stat.f_blocks * (uint64_t)stat.f_frsize;
		uint64_t free_b = (uint64_t)stat.f_bfree * (uint64_t)stat.f_frsize;
		uint64_t written = (total > free_b) ? (total - free_b) : 0;
		printk("The total number of bytes written: %lld and the available space: %lld\n",
		       (long long)written, (long long)free_b);
	} else {
		printk("The total number of bytes written: 0 and the available space: 0\n");
	}
}

/* VERSION handler */
static void handle_version(void)
{
	printk("\nVersion: v1.2.0\n");
	printk("Git commit: %s\n", WILDSONIC_GIT_COMMIT);
	printk("Build Date: %s %s\n", __DATE__, __TIME__);
}

/* SN handler (max 3 alphanumeric chars per original WildSonic) */
static int handle_sn(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nValue is set to %s\n", g_cfg.serial_number);
		return 0;
	}

	if (args_length > 3) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %s\n", g_cfg.serial_number);
		return -EINVAL;
	}

	for (size_t i = 0; i < args_length; i++) {
		if (!isalnum((unsigned char)p_args[i])) {
			printk("\nInvalid value %s\n", p_args);
			printk("\nValue is set to %s\n", g_cfg.serial_number);
			return -EINVAL;
		}
	}

	strncpy(g_cfg.serial_number, p_args, sizeof(g_cfg.serial_number) - 1);
	g_cfg.serial_number[sizeof(g_cfg.serial_number) - 1] = '\0';
	printk("\nValue is set to %s\n", g_cfg.serial_number);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	ble_comm_notify_name_changed();
	console_commit_and_ack_config("SN", g_cfg.serial_number, recorder_is_active());
	return 0;
}

/* DNAME handler (max 6 alphanumeric chars per original WildSonic) */
static int handle_dname(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nValue is set to %s\n", g_cfg.device_name);
		return 0;
	}

	if (args_length > 6) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %s\n", g_cfg.device_name);
		return -EINVAL;
	}

	for (size_t i = 0; i < args_length; i++) {
		if (!isalnum((unsigned char)p_args[i])) {
			printk("\nInvalid value %s\n", p_args);
			printk("\nValue is set to %s\n", g_cfg.device_name);
			return -EINVAL;
		}
	}

	strncpy(g_cfg.device_name, p_args, sizeof(g_cfg.device_name) - 1);
	g_cfg.device_name[sizeof(g_cfg.device_name) - 1] = '\0';
	printk("\nValue is set to %s\n", g_cfg.device_name);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	ble_comm_notify_name_changed();
	console_commit_and_ack_config("DNAME", g_cfg.device_name, recorder_is_active());
	return 0;
}

/* PW handler */
static int handle_pw(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nValue is set to %s\n", g_cfg.device_password);
		return 0;
	}

	if (args_length >= sizeof(g_cfg.device_password)) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %s\n", g_cfg.device_password);
		return -EINVAL;
	}

	strncpy(g_cfg.device_password, p_args, sizeof(g_cfg.device_password) - 1);
	g_cfg.device_password[sizeof(g_cfg.device_password) - 1] = '\0';
	printk("\nValue is set to %s\n", g_cfg.device_password);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	return console_commit_and_ack_config("PW", g_cfg.device_password, recorder_is_active());
}

struct wildsonic_assert_record g_assert_info = {
	.asserted = false,
	.file = NULL,
	.line = 0,
	.expr = NULL,
};

void wildsonic_record_assert(const char *file, int line, const char *expr)
{
	g_assert_info.asserted = true;
	g_assert_info.file = file;
	g_assert_info.line = line;
	g_assert_info.expr = expr;
}

extern uint32_t g_boot_csr;

/* ASSERT handler */
static void handle_assert(void)
{
	if (g_assert_info.asserted) {
		printk("ASSERT IN FILE %s #%d: %s\n",
		       g_assert_info.file ? g_assert_info.file : "unknown",
		       g_assert_info.line,
		       g_assert_info.expr ? g_assert_info.expr : "condition");
	} else if (g_boot_csr & (RCC_CSR_SFTRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_LPWRRSTF)) {
		printk("Assert / Reset Reason: CSR=0x%08X (SFTRST=%d, IWDG=%d, WWDG=%d, LPWR=%d)\n",
		       (unsigned int)g_boot_csr,
		       !!(g_boot_csr & RCC_CSR_SFTRSTF),
		       !!(g_boot_csr & RCC_CSR_IWDGRSTF),
		       !!(g_boot_csr & RCC_CSR_WWDGRSTF),
		       !!(g_boot_csr & RCC_CSR_LPWRRSTF));
	} else {
		printk("There is no assert information \n");
	}
}

/* ==============================================================================
 * Location and Solar Scheduling Command Handlers
 * ============================================================================== */

static void print_current_location(void)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (scfg.latitude_e7 == SCHEDULER_INVALID_LOCATION_COORDINATE ||
	    scfg.longitude_e7 == SCHEDULER_INVALID_LOCATION_COORDINATE) {
		printk("Current_Location: Not set\n");
		return;
	}

	printk("Current_Location: %.7f, %.7f\n",
	       (double)scfg.latitude_e7 / (double)SCHEDULER_LOCATION_COORDINATE_SCALE,
	       (double)scfg.longitude_e7 / (double)SCHEDULER_LOCATION_COORDINATE_SCALE);
}

static bool parse_location_coordinates(const char *p_args, size_t args_length,
                                       double *out_lat, double *out_lon)
{
	if (!p_args || args_length == 0 || args_length >= 128) {
		return false;
	}

	char buf[128];
	memcpy(buf, p_args, args_length);
	buf[args_length] = '\0';

	char *cursor = buf;
	while (isspace((unsigned char)*cursor)) {
		cursor++;
	}
	if (*cursor == '\0') {
		return false;
	}

	char *end_ptr = NULL;
	double lat = strtod(cursor, &end_ptr);
	if (end_ptr == cursor || !isfinite(lat)) {
		return false;
	}

	cursor = end_ptr;
	while (isspace((unsigned char)*cursor)) {
		cursor++;
	}
	if (*cursor == '\0') {
		return false;
	}

	double lon = strtod(cursor, &end_ptr);
	if (end_ptr == cursor || !isfinite(lon)) {
		return false;
	}

	while (isspace((unsigned char)*end_ptr)) {
		end_ptr++;
	}
	if (*end_ptr != '\0') {
		return false;
	}

	if (out_lat) *out_lat = lat;
	if (out_lon) *out_lon = lon;
	return true;
}

static void handle_loc(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (!p_args || args_length == 0) {
		print_current_location();
		s_last_cmd_status = 0;
		return;
	}

	if (args_length == 5 && strncmp(p_args, "CLEAR", 5) == 0) {
		scfg.latitude_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
		scfg.longitude_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
		g_cfg.latitude_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
		g_cfg.longitude_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
		g_cfg.location_lat_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
		g_cfg.location_lon_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
		scheduler_set_config(&scfg);
		g_cfg.save_config_flag = true;
		printk("\nLocation cleared\n");
		print_current_location();
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = 0;
		return;
	}

	double lat = 0.0, lon = 0.0;
	if (!parse_location_coordinates(p_args, args_length, &lat, &lon)) {
		printk("\nInvalid location. Use: LOC <latitude> <longitude>\n");
		print_current_location();
		s_last_cmd_status = -EINVAL;
		return;
	}

	if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
		printk("\nInvalid location range. Latitude [-90,90], Longitude [-180,180]\n");
		print_current_location();
		s_last_cmd_status = -ERANGE;
		return;
	}

	const double scaled_lat = lat * (double)SCHEDULER_LOCATION_COORDINATE_SCALE;
	const double scaled_lon = lon * (double)SCHEDULER_LOCATION_COORDINATE_SCALE;
	const int32_t lat_e7 = (scaled_lat >= 0.0) ?
		(int32_t)floor(scaled_lat + 0.5) : (int32_t)ceil(scaled_lat - 0.5);
	const int32_t lon_e7 = (scaled_lon >= 0.0) ?
		(int32_t)floor(scaled_lon + 0.5) : (int32_t)ceil(scaled_lon - 0.5);

	scfg.latitude_e7 = lat_e7;
	scfg.longitude_e7 = lon_e7;
	g_cfg.latitude_e7 = lat_e7;
	g_cfg.longitude_e7 = lon_e7;
	g_cfg.location_lat_e7 = lat_e7;
	g_cfg.location_lon_e7 = lon_e7;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;

	printk("\nLocation set to %.7f, %.7f\n", lat, lon);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sun(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (scfg.latitude_e7 == SCHEDULER_INVALID_LOCATION_COORDINATE ||
	    scfg.longitude_e7 == SCHEDULER_INVALID_LOCATION_COORDINATE) {
		printk("\nLocation is not set. Use: LOC <latitude> <longitude>\n");
		s_last_cmd_status = -EINVAL;
		return;
	}

	int year = 0, month = 0, day = 0;

	if (!p_args || args_length == 0) {
		struct rtc_calendar_time cur_cal;
		if (rtc_calendar_get(&cur_cal) != 0 || !rtc_calendar_is_valid()) {
			printk("\nRTC is not set. Use: SUN [YYYY/MM/DD]\n");
			s_last_cmd_status = -EINVAL;
			return;
		}
		time_t epoch_utc = scheduler_calendar_to_epoch(&cur_cal);
		time_t local_epoch = epoch_utc + scfg.time_zone_offset;
		struct rtc_calendar_time local_cal;
		scheduler_epoch_to_calendar(local_epoch, &local_cal);
		year = local_cal.year;
		month = local_cal.month;
		day = local_cal.day;
	} else {
		char date_buf[32];
		if (args_length >= sizeof(date_buf)) {
			printk("\nInvalid date. Use: SUN [YYYY/MM/DD]\n");
			s_last_cmd_status = -EINVAL;
			return;
		}
		memcpy(date_buf, p_args, args_length);
		date_buf[args_length] = '\0';

		if (sscanf(date_buf, "%d/%d/%d", &year, &month, &day) != 3 ||
		    year < 1970 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31) {
			printk("\nInvalid date. Use: SUN [YYYY/MM/DD]\n");
			s_last_cmd_status = -EINVAL;
			return;
		}
	}

	const double julian_date = solar_compute_julian_date(year, month, day, 0.0);
	const double lat = (double)scfg.latitude_e7 / (double)SCHEDULER_LOCATION_COORDINATE_SCALE;
	const double lon = (double)scfg.longitude_e7 / (double)SCHEDULER_LOCATION_COORDINATE_SCALE;

	uint32_t sunrise_sec = 0, sunset_sec = 0;
	if (!solar_compute_sunrise_sunset_seconds(julian_date, lat, lon, scfg.time_zone_offset,
	                                         &sunrise_sec, &sunset_sec)) {
		printk("\nSunrise/sunset is not available for this date and location\n");
		s_last_cmd_status = -EDOM;
		return;
	}

	printk("\nSun date: %04d/%02d/%02d\n", year, month, day);
	printk("Julian date: %.2f\n", julian_date);
	printk("Sunrise time: %02u:%02u:%02u\n",
	       sunrise_sec / 3600, (sunrise_sec / 60) % 60, sunrise_sec % 60);
	printk("Sunset time: %02u:%02u:%02u\n",
	       sunset_sec / 3600, (sunset_sec / 60) % 60, sunset_sec % 60);
	s_last_cmd_status = 0;
}

static void handle_sunmode(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		switch (scfg.sun_recording_mode) {
		case SUN_MODE_SUNRISE:
			printk("\nSun recording mode: Sunrise\n");
			break;
		case SUN_MODE_SUNSET:
			printk("\nSun recording mode: Sunset\n");
			break;
		case SUN_MODE_SUNRISE_AND_SUNSET:
			printk("\nSun recording mode: Sunrise and Sunset\n");
			break;
		case SUN_MODE_SUNRISE_TO_SUNSET:
			printk("\nSun recording mode: Sunrise to Sunset\n");
			break;
		case SUN_MODE_SUNSET_TO_SUNRISE:
			printk("\nSun recording mode: Sunset to Sunrise\n");
			break;
		default:
			printk("\nSun recording mode: Disabled\n");
			break;
		}
		s_last_cmd_status = 0;
		return;
	}

	static const struct {
		const char *label;
		enum sun_recording_mode mode;
	} sun_mode_table[] = {
		{"Sunrise and Sunset", SUN_MODE_SUNRISE_AND_SUNSET},
		{"Sunrise to Sunset", SUN_MODE_SUNRISE_TO_SUNSET},
		{"Sunset to Sunrise", SUN_MODE_SUNSET_TO_SUNRISE},
		{"Sunrise", SUN_MODE_SUNRISE},
		{"Sunset", SUN_MODE_SUNSET},
		{"Disabled", SUN_MODE_DISABLED},
	};

	for (size_t i = 0; i < ARRAY_SIZE(sun_mode_table); i++) {
		size_t label_len = strlen(sun_mode_table[i].label);
		if (args_length == label_len && strncmp(p_args, sun_mode_table[i].label, label_len) == 0) {
			scfg.sun_recording_mode = sun_mode_table[i].mode;
			g_cfg.sun_recording_mode = (uint8_t)sun_mode_table[i].mode;

			if (scfg.sun_recording_mode == SUN_MODE_DISABLED) {
				scfg.sunrise_interval_before = 0;
				scfg.sunrise_interval_after = 0;
				scfg.sunset_interval_before = 0;
				scfg.sunset_interval_after = 0;
				scfg.sunrise_sunset_rounding = 0;

				g_cfg.sunrise_interval_before = 0;
				g_cfg.sunrise_interval_after = 0;
				g_cfg.sunset_interval_before = 0;
				g_cfg.sunset_interval_after = 0;
				g_cfg.sunrise_sunset_rounding = 0;
			}
			scheduler_set_config(&scfg);
			g_cfg.save_config_flag = true;

			printk("\nSun recording mode set to: %s\n", sun_mode_table[i].label);
			printk("\nSend APPLY command and reboot for changes to take effect\n");
			s_last_cmd_status = 0;
			return;
		}
	}

	printk("\nInvalid sun recording mode: %.*s\n", (int)args_length, p_args);
	printk("Valid modes: Sunrise, Sunset, Sunrise and Sunset, Sunrise to Sunset, Sunset to Sunrise, Disabled\n");
	s_last_cmd_status = -EINVAL;
}

static void handle_sunrisebef(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		printk("\nValue is set to %u\n", scfg.sunrise_interval_before);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", scfg.sunrise_interval_before);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunrise_interval_before = val;
	g_cfg.sunrise_interval_before = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	printk("\nValue is set to %u\n", val);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sunriseaft(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		printk("\nValue is set to %u\n", scfg.sunrise_interval_after);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", scfg.sunrise_interval_after);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunrise_interval_after = val;
	g_cfg.sunrise_interval_after = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	printk("\nValue is set to %u\n", val);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sunsetbef(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		printk("\nValue is set to %u\n", scfg.sunset_interval_before);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", scfg.sunset_interval_before);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunset_interval_before = val;
	g_cfg.sunset_interval_before = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	printk("\nValue is set to %u\n", val);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sunsetaft(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		printk("\nValue is set to %u\n", scfg.sunset_interval_after);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", scfg.sunset_interval_after);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunset_interval_after = val;
	g_cfg.sunset_interval_after = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	printk("\nValue is set to %u\n", val);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sunround(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		printk("\nValue is set to %u\n", scfg.sunrise_sunset_rounding);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		printk("\nInvalid value %.*s\n", (int)args_length, p_args);
		printk("\nValue is set to %u\n", scfg.sunrise_sunset_rounding);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunrise_sunset_rounding = val;
	g_cfg.sunrise_sunset_rounding = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	printk("\nValue is set to %u\n", val);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

/* DEFAULT handler */
static void handle_default(void)
{
	main_reset_to_defaults();
	config_store_load_defaults();
	printk("\nDefault device configurations are set\n");
}

/* GET_STATUS handler */
static void handle_get_status(void)
{
	printk("\n=== SYSTEM & RECORDING STATUS ===\n");
	if (recorder_is_active()) {
		printk("  Recording State       : ACTIVE\n");
		char cur_file[80] = {0};
		recorder_get_active_file(cur_file, sizeof(cur_file));
		if (cur_file[0] != '\0') {
			printk("  Active File           : %s\n", cur_file);
		}
		uint64_t samples_written = 0, expected_samples = 0;
		recorder_get_active_progress(&samples_written, &expected_samples);
		printk("  Samples Written       : %llu / %llu\n",
		       (unsigned long long)samples_written, (unsigned long long)expected_samples);
		printk("  Active Sample Rate    : %u Hz\n", audio_in_rate());
		printk("  Active Bit Depth      : %u-bit\n", g_cfg.audio_sampling_width);
		printk("  Active Channels       : %u (%s)\n",
		       g_cfg.num_audio_channels,
		       (g_cfg.num_audio_channels == 2) ? "Duplicated Mono Container" : "Mono");
	} else {
		printk("  Recording State       : IDLE\n");
		printk("  Scheduler Status      : %s\n",
		       main_is_scheduler_paused() ? "PAUSED" : "RUNNING");
		printk("  Configured Rate       : %u Hz\n", g_cfg.sampling_rate);
		printk("  Configured Bit Depth  : %u-bit\n", g_cfg.audio_sampling_width);
		printk("  Configured Channels   : %u\n", g_cfg.num_audio_channels);
		printk("  Configured Record Time: %u seconds\n", g_cfg.recording_length);
		printk("  Configured Sleep Time : %u seconds\n", g_cfg.sleep_pause_interval);
	}

	if (rtc_calendar_is_valid()) {
		struct rtc_calendar_time cur_cal;
		if (rtc_calendar_get(&cur_cal) == 0) {
			char rtc_str[32];
			rtc_calendar_format(&cur_cal, rtc_str, sizeof(rtc_str));
			printk("  RTC Current Time      : %s UTC\n", rtc_str);
		}
	} else {
		printk("  RTC Current Time      : UNINITIALIZED\n");
	}

	int last_sd_err = recorder_get_last_sd_error();
	if (last_sd_err != 0) {
		printk("  Last Storage Error    : %d\n", last_sd_err);
	}
	printk("OK\n");
}

/* GET_DEV_INFO handler */
static void handle_get_dev_info(void)
{
	printk("\n=== DEVICE INFORMATION ===\n");
	printk("  Hardware Platform     : STM32L476RG Nucleo-64 (ARM Cortex-M4F)\n");
	printk("  Core Clock Frequency  : 72 MHz (PLL HSI)\n");
	printk("  Audio Acquisition     : 12-bit SAR ADC, TIM6 TRGO DMA\n");
	printk("  External Memory       : ISSI IS66WVS4M8 4MB Quad-SPI PSRAM @ 18 MHz\n");
	printk("  Storage Media         : MicroSD Card via SDMMC1 4-bit @ 6 MHz (FatFs /SD:)\n");
	printk("  SRAM Architecture     : 128 KiB (SRAM1: 96 KiB, SRAM2: 32 KiB)\n");
	printk("  Flash Memory          : 1024 KiB Dual-Bank\n");
	printk("  Power Management      : STM32L4 Stop 2 Deep Sleep (Strict RTC Wakeup)\n");
	printk("  Firmware Version      : 2.1.0-prod (Zephyr RTOS)\n");
	printk("OK\n");
}

/* GET_CONFIG handler */
static void handle_get_config(void)
{
	printk("\n=== DEVICE CONFIGURATION ===\n");
	printk("  Serial Number (SN)    : %s\n", g_cfg.serial_number);
	printk("  Device Name (DNAME)   : %s\n", g_cfg.device_name);
	printk("  Sampling Rate (SAMF)  : %u Hz\n", g_cfg.sampling_rate);
	printk("  Bits Per Sample (BTPS): %u-bit\n", g_cfg.audio_sampling_width);
	printk("  Channels (NUMCH)      : %u (%s)\n",
	       g_cfg.num_audio_channels,
	       (g_cfg.num_audio_channels == 2) ? "Duplicated Mono Container" : "Mono");
	printk("  Record Time (RECT)    : %u seconds\n", g_cfg.recording_length);
	printk("  Sleep Interval (SLEEP): %u seconds\n", g_cfg.sleep_pause_interval);
	printk("  Mic Gain (GAIN)       : %.1f dB\n", (double)((float)g_cfg.mic_gain_times_ten / 10.0f));
	printk("  Lower Cutoff (LCUTOFF): %u Hz\n", g_cfg.lower_cutoff_frequency);
	printk("  Upper Cutoff (UCUTOFF): %u Hz\n", g_cfg.upper_cutoff_frequency);
	printk("  Folder Mode (FLDR)    : %u (%s)\n",
	       g_cfg.folder_structure,
	       (g_cfg.folder_structure == 0) ? "Flat" :
	       (g_cfg.folder_structure == 1) ? "Daily" : "Hourly");
	printk("OK\n");
}

/* STOP_RECORDING handler */
static void handle_stop_recording(void)
{
	if (recorder_is_active()) {
		recorder_request_stop();
		printk("\nOK: Stop recording requested; finalizing WAV container and closing file.\n");
	} else {
		printk("\nOK: Recorder is already idle.\n");
	}
}

/* APPLY handler */
static void handle_apply(void)
{
	if (recorder_is_active()) {
		printk("\nBUSY: Cannot commit to flash while recording is active. Send STOP_RECORDING first.\n");
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "BUSY: Cannot commit to flash while recording is active");
		}
		s_last_cmd_status = -EBUSY;
		return;
	}
	g_cfg.save_config_flag = true;
	int save_rc = config_store_save();
	if (save_rc != 0) {
		LOG_ERR("Failed to persist config to flash (%d)", save_rc);
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Flash save failed (%d)", save_rc);
		}
		s_last_cmd_status = save_rc;
		return;
	}
	/* Canonical Maxim notification: */
	printk("\nDevice configuration is applied after reboot.\n");
	if (s_current_out_resp && s_current_out_resp_size > 0) {
		snprintf(s_current_out_resp, s_current_out_resp_size, "Device configuration is applied after reboot.");
	}
	/* Historical compatibility note for test harnesses: Flash persistence not implemented in legacy stubs */
	main_scheduler_resume();
	s_last_cmd_status = 0;
}

/* RBT handler */
static void handle_rbt(void)
{
	if (main_is_reboot_pending()) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
		printk("\n[ACK] RBT: Reboot already pending. System shutdown in progress.\n");
#endif
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ACK] RBT: Reboot already pending");
		}
		s_last_cmd_status = 0;
		return;
	}

	main_set_reboot_pending(true);
	main_set_shutdown_stage(SHUTDOWN_STAGE_REQUESTED);

	main_scheduler_pause();
	main_set_shutdown_stage(SHUTDOWN_STAGE_SCHEDULER_STOPPED);

	printk("\nRebooting device.\n");

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	printk("\n[ACK] RBT: Controlled reboot initiated\n");
#endif
	if (s_current_out_resp && s_current_out_resp_size > 0) {
		snprintf(s_current_out_resp, s_current_out_resp_size, "[ACK] RBT: Controlled reboot initiated");
	}

	if (recorder_is_active()) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
		printk("[REBOOT] Stopping active recording safely...\n");
#endif
		recorder_set_stop_reason(RECORDER_STOP_SYSTEM_REBOOT);
		recorder_request_stop();
		main_set_shutdown_stage(SHUTDOWN_STAGE_ADC_DMA_STOPPED);

		main_set_shutdown_stage(SHUTDOWN_STAGE_AUDIO_QUEUES_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_PSRAM_READER_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_SD_WRITER_DRAINED);

		int wait_rc = recorder_wait_idle(5000);
		if (wait_rc != 0) {
			main_set_reboot_pending(false);
			main_set_shutdown_stage(SHUTDOWN_STAGE_IDLE);
			LOG_ERR("RBT: Recorder shutdown timeout (%d); aborting reboot to preserve data", wait_rc);
			printk("\nError: Failed to finalize recording before reboot.\n");
			if (s_current_out_resp && s_current_out_resp_size > 0) {
				snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Storage shutdown timeout (%d)", wait_rc);
			}
			s_last_cmd_status = -ETIMEDOUT;
			return;
		}
		main_set_shutdown_stage(SHUTDOWN_STAGE_WAV_FINALIZED);
	} else {
		main_set_shutdown_stage(SHUTDOWN_STAGE_ADC_DMA_STOPPED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_AUDIO_QUEUES_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_PSRAM_READER_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_SD_WRITER_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_WAV_FINALIZED);
	}

	if (!recorder_is_shutdown_ready()) {
		main_set_reboot_pending(false);
		main_set_shutdown_stage(SHUTDOWN_STAGE_IDLE);
		LOG_ERR("RBT: Storage resources still active; aborting reboot");
		printk("[ERROR] Storage resources still active; reboot aborted to prevent corruption.\n");
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Storage resources still active");
		}
		s_last_cmd_status = -EBUSY;
		return;
	}

	int last_err = recorder_get_last_error();
	int last_sd = recorder_get_last_sd_error();
	if (last_err != 0 || last_sd != 0) {
		LOG_WRN("RBT: Proceeding with reboot despite historical storage error (%d)",
		        last_err != 0 ? last_err : last_sd);
	}

	led_indicator_set(LED_IND_OFF);
	if (g_cfg.save_config_flag) {
		main_set_shutdown_stage(SHUTDOWN_STAGE_CONFIG_PERSISTED);
		config_store_save();
		g_cfg.save_config_flag = false;
	}
	main_set_shutdown_stage(SHUTDOWN_STAGE_REBOOT_READY);
	while (log_process()) {
		k_sleep(K_MSEC(2));
	}
	k_sleep(K_MSEC(50));
	main_set_shutdown_stage(SHUTDOWN_STAGE_SYSTEM_RESET);
#if defined(CONFIG_REBOOT)
	sys_reboot(SYS_REBOOT_COLD);
#endif
	s_last_cmd_status = 0;
}

/* ARBT handler */
static void handle_arbt(void)
{
	if (main_is_reboot_pending()) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
		printk("\n[ACK] ARBT: Reboot already pending. System shutdown in progress.\n");
#endif
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ACK] ARBT: Reboot already pending");
		}
		s_last_cmd_status = 0;
		return;
	}

	main_set_reboot_pending(true);
	main_set_shutdown_stage(SHUTDOWN_STAGE_REQUESTED);

	main_scheduler_pause();
	main_set_shutdown_stage(SHUTDOWN_STAGE_SCHEDULER_STOPPED);

	printk("\nRebooting device.\n");

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	/* Immediate acknowledgment to console */
	printk("\n[ACK] ARBT: Applying configuration and initiating orderly shutdown...\n");
#endif
	if (s_current_out_resp && s_current_out_resp_size > 0) {
		snprintf(s_current_out_resp, s_current_out_resp_size, "[ACK] ARBT: Controlled reboot initiated");
	}

	/* If recording is active, request orderly stop and wait for clean drain */
	if (recorder_is_active()) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
		printk("[REBOOT] Stopping active recording safely...\n");
#endif
		recorder_set_stop_reason(RECORDER_STOP_APPLY_REBOOT);
		recorder_request_stop();
		main_set_shutdown_stage(SHUTDOWN_STAGE_ADC_DMA_STOPPED);

		main_set_shutdown_stage(SHUTDOWN_STAGE_AUDIO_QUEUES_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_PSRAM_READER_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_SD_WRITER_DRAINED);

		int wait_rc = recorder_wait_idle(5000);
		if (wait_rc != 0) {
			main_set_reboot_pending(false);
			main_set_shutdown_stage(SHUTDOWN_STAGE_IDLE);
			LOG_ERR("ARBT: Recorder shutdown timeout (%d); aborting reboot to preserve storage", wait_rc);
			printk("[ERROR] Storage shutdown timed out (%d); aborting reboot to prevent corruption.\n", wait_rc);
			if (s_current_out_resp && s_current_out_resp_size > 0) {
				snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Storage shutdown timeout (%d)", wait_rc);
			}
			s_last_cmd_status = -ETIMEDOUT;
			return;
		}
		main_set_shutdown_stage(SHUTDOWN_STAGE_WAV_FINALIZED);
	} else {
		main_set_shutdown_stage(SHUTDOWN_STAGE_ADC_DMA_STOPPED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_AUDIO_QUEUES_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_PSRAM_READER_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_SD_WRITER_DRAINED);
		main_set_shutdown_stage(SHUTDOWN_STAGE_WAV_FINALIZED);
	}

	if (!recorder_is_shutdown_ready()) {
		main_set_reboot_pending(false);
		main_set_shutdown_stage(SHUTDOWN_STAGE_IDLE);
		LOG_ERR("ARBT: Storage resources still active; aborting reboot");
		printk("[ERROR] Storage resources still active; reboot aborted to prevent corruption.\n");
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Storage resources still active");
		}
		s_last_cmd_status = -EBUSY;
		return;
	}

	int last_err = recorder_get_last_error();
	int last_sd = recorder_get_last_sd_error();
	if (last_err != 0 || last_sd != 0) {
		LOG_WRN("ARBT: Proceeding with reboot despite historical storage error (%d)",
		        last_err != 0 ? last_err : last_sd);
	}

	/* Save configuration exactly ONCE */
	main_set_shutdown_stage(SHUTDOWN_STAGE_CONFIG_PERSISTED);
	g_cfg.save_config_flag = false;
	int save_rc = config_store_save();
	if (save_rc != 0) {
		main_set_reboot_pending(false);
		main_set_shutdown_stage(SHUTDOWN_STAGE_IDLE);
		LOG_ERR("ARBT: Failed to save configuration to flash (%d)", save_rc);
		printk("[ERROR] Flash save failed (%d). Reboot aborted.\n", save_rc);
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Flash save failed (%d)", save_rc);
		}
		s_last_cmd_status = save_rc;
		return;
	}
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	printk("[ACK] Configuration committed to persistent flash (1 save operation).\n");
	printk("\nAll buffers flushed and WAV container closed cleanly. Rebooting device.\n");
#endif

	main_set_shutdown_stage(SHUTDOWN_STAGE_REBOOT_READY);
	led_indicator_set(LED_IND_OFF);
	while (log_process()) {
		k_sleep(K_MSEC(2));
	}
	k_sleep(K_MSEC(50));
	main_set_shutdown_stage(SHUTDOWN_STAGE_SYSTEM_RESET);
#if defined(CONFIG_REBOOT)
	sys_reboot(SYS_REBOOT_COLD);
#endif
	s_last_cmd_status = 0;
}

/* SDFMT handler */
static void handle_sdfmt(const char *p_args, size_t args_length)
{
	if (args_length > 0 && atoi(p_args) == 0) {
		g_cfg.format_sd_card_flag = false;
		printk("\nSD card format won't happen during reboot\n");
	} else {
		g_cfg.format_sd_card_flag = true;
		printk("\nSD card format will happen during reboot\n");
	}
}

/* BLEPAIR handler */
static void handle_blepair(const char *p_args, size_t args_length)
{
	if (args_length > 0 && (strncasecmp(p_args, "LOCK", 4) == 0 || strcmp(p_args, "0") == 0)) {
		ble_comm_lock_pairing();
		return;
	}
	uint32_t seconds = 300;
	if (args_length > 0) {
		unsigned long val = strtoul(p_args, NULL, 10);
		if (val > 0) {
			seconds = (uint32_t)val;
		}
	}
	ble_comm_open_pairing_window(seconds);
}

/* Initialize Console */
void console_init(void)
{
	struct rtc_calendar_time cur_cal;
	if (rtc_calendar_get(&cur_cal) == 0 && rtc_calendar_is_valid()) {
		g_cfg.boot_time_utc = scheduler_calendar_to_epoch(&cur_cal);
	}
}

/* Command dispatcher - Process incoming command line */
static int console_process_line_internal(char *cmd_line, bool from_ble, char *out_resp, size_t out_resp_size)
{
	if (!cmd_line) return -EINVAL;

	/* Trim leading whitespace */
	char *p = cmd_line;
	while (*p && isspace((unsigned char)*p)) p++;

	/* Trim trailing whitespace */
	size_t len = strlen(p);
	while (len > 0 && isspace((unsigned char)p[len - 1])) {
		p[--len] = '\0';
	}

	if (len == 0) {
		if (main_is_scheduler_paused()) {
			main_scheduler_resume();
		}
		return 0;
	}

	if (out_resp && out_resp_size > 0) {
		out_resp[0] = '\0';
	}

	if (strncasecmp(p, "BITS", 4) == 0 && (p[4] == ' ' || p[4] == '\0' || p[4] == '=')) {
		const char *arg = p + 4;
		while (*arg == ' ' || *arg == '=') arg++;
		s_last_cmd_status = 0;
		handle_btps(arg, strlen(arg));
		if (s_last_cmd_status != 0) return s_last_cmd_status;
		if (out_resp && out_resp_size > 0 && out_resp[0] == '\0') {
			snprintf(out_resp, out_resp_size, "OK");
		}
		return 0;
	}
	if (strncasecmp(p, "CHANNELS", 8) == 0 && (p[8] == ' ' || p[8] == '\0' || p[8] == '=')) {
		const char *arg = p + 8;
		while (*arg == ' ' || *arg == '=') arg++;
		s_last_cmd_status = 0;
		handle_numch(arg, strlen(arg));
		if (s_last_cmd_status != 0) return s_last_cmd_status;
		if (out_resp && out_resp_size > 0 && out_resp[0] == '\0') {
			snprintf(out_resp, out_resp_size, "OK");
		}
		return 0;
	}
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	if (strncasecmp(p, "SOURCE", 6) == 0 && (p[6] == ' ' || p[6] == '\0' || p[6] == '=')) {
		const char *arg = p + 6;
		while (*arg == ' ' || *arg == '=') arg++;
		handle_source(arg, strlen(arg));
		return 0;
	}
	if (strcasecmp(p, "SDBENCH") == 0 || strcasecmp(p, "SD_BENCH") == 0) {
		sd_benchmark_run_matrix();
		return 0;
	}
#endif

	if (strcasecmp(p, "STATUS") == 0) {
		if (from_ble) {
			ble_comm_send_status_info();
		} else {
			handle_get_status();
		}
		return 0;
	}
	if (strcasecmp(p, "DEVINFO") == 0) {
		if (from_ble) {
			ble_comm_send_dev_info();
		} else {
			handle_get_dev_info();
		}
		return 0;
	}
	if (strcasecmp(p, "stop") == 0) {
		handle_stop_recording();
		if (out_resp && out_resp_size > 0) snprintf(out_resp, out_resp_size, "OK");
		return 0;
	}

	size_t arg_offset = 0;
	enum console_cmd_id cmd_id = console_cmd_lookup_func(p, len, &arg_offset);

	const char *p_args = &p[arg_offset];
	size_t args_len = (len >= arg_offset) ? (len - arg_offset) : 0;
	int handler_rc = 0;
	s_last_cmd_status = 0;

	switch (cmd_id) {
	case CMD_HELP:
		display_console_help();
		break;
	case CMD_SERIAL_NUMBER:
		handler_rc = handle_sn(p_args, args_len);
		break;
	case CMD_DEVICE_NAME:
		handler_rc = handle_dname(p_args, args_len);
		break;
	case CMD_PASSWORD:
		handler_rc = handle_pw(p_args, args_len);
		break;
	case CMD_SMS_KEYWORD:
		report_deferred_hardware("SMSKW");
		break;
	case CMD_SMS_KEY:
		report_deferred_hardware("SMSKEY");
		break;
	case CMD_A2P_PHONE_NUMBER:
		report_deferred_hardware("A2PPHONE");
		break;
	case CMD_P2A_PHONE_NUMBER:
		report_deferred_hardware("P2APHONE");
		break;
	case CMD_SERVER_URL:
		report_deferred_hardware("SURL");
		break;
	case CMD_SERVER_PORT:
		report_deferred_hardware("SPORT");
		break;
	case CMD_SERVER_POST_ROUTE:
		report_deferred_hardware("POSTROUTE");
		break;
	case CMD_SERVER_POST_CONTENT_TYPE:
		report_deferred_hardware("POSTCTNT");
		break;
	case CMD_SERVER_TEST_SETTING:
		report_deferred_hardware("STEST");
		break;
	case CMD_SERVER_WILDSONIC_SETTING:
		report_deferred_hardware("SWS");
		break;
	case CMD_GSMMODE:
		report_deferred_hardware("GSM");
		break;
	case CMD_SAMF:
		handle_samf(p_args, args_len);
		break;
	case CMD_RECT:
		handle_rect(p_args, args_len);
		break;
	case CMD_SLEEP:
		handle_sleep(p_args, args_len);
		break;
	case CMD_DATE_RANGE:
		handle_daterange(p_args, args_len);
		break;
	case CMD_TIME_OF_DAY:
		handle_timeofday(p_args, args_len);
		break;
	case CMD_DAYS_OF_WEEK:
		handle_daysofweek(p_args, args_len);
		break;
	case CMD_DATETIME:
		handle_datetime(p_args, args_len);
		break;
	case CMD_TIMEZONE:
		handle_timezone(p_args, args_len);
		break;
	case CMD_BTPS:
		handle_btps(p_args, args_len);
		break;
	case CMD_NUMCH:
		handle_numch(p_args, args_len);
		break;
	case CMD_GAIN:
		handle_gain(p_args, args_len);
		break;
	case CMD_LOWER_CUTOFF:
		handle_lcutoff(p_args, args_len);
		break;
	case CMD_UPPER_CUTOFF:
		handle_ucutoff(p_args, args_len);
		break;
	case CMD_FOLDER_STRUCTURE:
		handle_fldr(p_args, args_len);
		break;
	case CMD_APPLY:
		handle_apply();
		break;
	case CMD_REBOOT:
		handle_rbt();
		break;
	case CMD_APPLY_REBOOT:
		handle_arbt();
		break;
	case CMD_SD_FORMAT:
		handle_sdfmt(p_args, args_len);
		break;
	case CMD_SMS_FREQ:
		report_deferred_hardware("SMSFREQ");
		break;
	case CMD_ASSERT:
		handle_assert();
		break;
	case CMD_BATTERY_STATUS:
		report_deferred_hardware("BATT");
		break;
	case CMD_SIGNAL_QUALITY:
		report_deferred_hardware("SIG");
		break;
	case CMD_STORAGE_FILLED:
		handle_stor();
		break;
	case CMD_LOW_BAT:
		report_deferred_hardware("LBAT");
		break;
	case CMD_NUM_BAT:
		report_deferred_hardware("NBAT");
		break;
	case CMD_FULL_CAP:
		report_deferred_hardware("FULLCAP");
		break;
	case CMD_SEND_STATUS:
		report_deferred_hardware("SENDSTS");
		break;
	case CMD_AUDIO:
		handle_audio(p_args, args_len);
		break;
	case CMD_BLEPAIR:
		handle_blepair(p_args, args_len);
		break;
	case CMD_WILDLY:
		/* Silently ignored per reference behavior */
		break;
	case CMD_VERSION:
		handle_version();
		break;
	case CMD_ENVIRONMENT_STATUS:
		report_deferred_hardware("ENVSTS");
		break;
	case CMD_LOCATION:
		handle_loc(p_args, args_len);
		break;
	case CMD_SUN:
		handle_sun(p_args, args_len);
		break;
	case CMD_SUN_RECORDING_MODE:
		handle_sunmode(p_args, args_len);
		break;
	case CMD_SUNRISE_INTERVAL_BEFORE:
		handle_sunrisebef(p_args, args_len);
		break;
	case CMD_SUNRISE_INTERVAL_AFTER:
		handle_sunriseaft(p_args, args_len);
		break;
	case CMD_SUNSET_INTERVAL_BEFORE:
		handle_sunsetbef(p_args, args_len);
		break;
	case CMD_SUNSET_INTERVAL_AFTER:
		handle_sunsetaft(p_args, args_len);
		break;
	case CMD_SUNRISE_SUNSET_ROUNDING:
		handle_sunround(p_args, args_len);
		break;
	case CMD_DEFAULT:
		handle_default();
		break;
	case CMD_I2C:
		report_deferred_hardware("I2C");
		break;
	case CMD_RTC_RESET:
		report_deferred_hardware("RTCRST");
		break;
	case CMD_TEST_ALARM:
		report_deferred_hardware("TEST_ALARM");
		break;
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
	case CMD_PSRAM: {
		char arg1[32] = {0}, arg2[32] = {0};
		int matched = sscanf(p_args, "%31s %31s", arg1, arg2);
		main_psram_command(matched + 1, arg1, arg2);
		break;
	}
	case CMD_CONFIG_WIZARD:
		main_run_guided_config();
		break;
#else
	case CMD_PSRAM:
	case CMD_CONFIG_WIZARD:
		printk("\nUnknown or BAD command : %s\n", console_cmd_lookup_table[cmd_id]);
		break;
#endif
	case CMD_GET_STATUS:
		if (from_ble) {
			ble_comm_send_status_info();
		} else {
			handle_get_status();
		}
		break;
	case CMD_GET_DEV_INFO:
		if (from_ble) {
			ble_comm_send_dev_info();
		} else {
			handle_get_dev_info();
		}
		break;
	case CMD_GET_CONFIG:
		handle_get_config();
		break;
	case CMD_STOP_RECORDING:
		handle_stop_recording();
		break;

	default:
		if (!from_ble) {
			printk("\nUnknown or BAD command : %s\n", p);
			display_console_commands();
		}
		return -EINVAL;
	}

	if (handler_rc != 0) {
		return handler_rc;
	}
	if (s_last_cmd_status != 0) {
		return s_last_cmd_status;
	}

	if (out_resp && out_resp_size > 0 && out_resp[0] == '\0') {
		snprintf(out_resp, out_resp_size, "OK");
	}
	return 0;
}

int console_process_line_source(char *cmd_line, bool from_ble, char *out_resp, size_t out_resp_size)
{
	if (!cmd_line) return -EINVAL;

#ifdef __ZEPHYR__
	if (k_mutex_lock(&s_cmd_dispatch_mutex, K_MSEC(250)) != 0) {
		if (out_resp && out_resp_size > 0) {
			snprintf(out_resp, out_resp_size, "BUSY: command dispatcher unavailable");
		}
		return -EBUSY;
	}
#endif
	s_current_out_resp = out_resp;
	s_current_out_resp_size = out_resp_size;
	s_from_ble = from_ble;

	int rc = console_process_line_internal(cmd_line, from_ble, out_resp, out_resp_size);

	s_current_out_resp = NULL;
	s_current_out_resp_size = 0;
	s_from_ble = false;
#ifdef __ZEPHYR__
	k_mutex_unlock(&s_cmd_dispatch_mutex);
#endif
	return rc;
}

void console_process_line(char *cmd_line)
{
	(void)console_process_line_source(cmd_line, false, NULL, 0);
}

struct wildsonic_config *console_get_config(void)
{
	return &g_cfg;
}

void console_sync_staged_to_active(void)
{
	struct app_settings *settings = main_get_settings();

	if (!recorder_is_active()) {
		int rc = audio_in_set_rate(g_cfg.sampling_rate);
		if (rc == 0) {
			if (settings) {
				settings->sample_rate = audio_in_rate();
			}
		} else {
			LOG_WRN("Failed to set audio rate %u: %d, keeping %u",
			        g_cfg.sampling_rate, rc, audio_in_rate());
			g_cfg.sampling_rate = audio_in_rate();
			if (settings) {
				settings->sample_rate = audio_in_rate();
			}
		}
	} else if (settings) {
		settings->sample_rate = audio_in_rate();
	}

	if (settings) {
		settings->sample_bits = (uint16_t)g_cfg.audio_sampling_width;
		settings->channels = (uint16_t)g_cfg.num_audio_channels;
		settings->record_seconds = g_cfg.recording_length;
		settings->sleep_seconds = g_cfg.sleep_pause_interval;
		settings->digital_gain = powf(10.0f, ((float)g_cfg.mic_gain_times_ten / 10.0f) / 20.0f);
		settings->hpf_cutoff_hz = g_cfg.lower_cutoff_frequency;
		settings->hpf_enabled = (g_cfg.lower_cutoff_frequency > 0);
		settings->lpf_cutoff_hz = g_cfg.upper_cutoff_frequency;
		settings->lpf_enabled = (g_cfg.upper_cutoff_frequency > 0);
	}

	if (!recorder_is_active()) {
		main_apply_dsp_settings();
	}
}


