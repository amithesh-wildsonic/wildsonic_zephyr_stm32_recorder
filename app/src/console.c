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
#include "sd_fault_inject.h"
#include "sd_resilient.h"
#include "audio_in.h"
#include "led_indicator.h"
#include "recorder.h"
#include "psram_is66wvs.h"
#include "audio_format.h"
#include "ble_comm.h"
#include "wildsonic_trace.h"
#include "retained_reboot.h"
#include "system_assert.h"

#ifndef WILDSONIC_GIT_COMMIT
#define WILDSONIC_GIT_COMMIT "unknown"
#endif
#include "dsp.h"
#include "rate_check.h"
#if defined(CONFIG_WILDSONIC_BENCHMARKS)
#include "sd_benchmark.h"
#endif
#include "solar_calculator.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/fs/fs.h>
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
	config_store_stage_pending();

	if (recording_active) {
		TRACE_RAW("\nOK: Value staged for next session (active session unaffected)\n");
	}
	TRACE_RAW("[ACK] Command %s received and validated. Staged in RAM (pending gen %u).\n",
	          cmd_name, config_store_get_pending_generation());
	TRACE_RAW("[PENDING] Send APPLY or ARBT to save to flash.\n");

	if (s_current_out_resp && s_current_out_resp_size > 0) {
		snprintf(s_current_out_resp, s_current_out_resp_size,
		         "[ACK] %s %s: Staged in RAM (send APPLY/ARBT to save)",
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
	CMD_CONFIG_WIZARD,
	CMD_GET_STATUS,
	CMD_GET_DEV_INFO,
	CMD_GET_CONFIG,
	CMD_STOP_RECORDING,
	CMD_STATS,
	CMD_RATECHECK,
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	CMD_PSRAM,
	CMD_BATCHLOG,
#endif
#if defined(CONFIG_WILDSONIC_SD_FAULT_INJECT)
	CMD_FAULT,
#endif
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
	"CONFIG",
	"GET_STATUS",
	"GET_DEV_INFO",
	"GET_CONFIG",
	"STOP_RECORDING",
	"STATS",
	"RATECHECK",
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	"PSRAM",
	"BATCHLOG",
#endif
#if defined(CONFIG_WILDSONIC_SD_FAULT_INJECT)
	"FAULT",
#endif
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
	"Show active configuration / run guided wizard",
	"Get current system, scheduler, and recording status",
	"Get hardware platform and firmware device information",
	"Get current device audio, storage, and schedule configuration",
	"Safely stop active recording, drain buffers, and close file",
	"Print last recording session accounting statistics",
	"Check feasibility of sample rate, bits, and duration: RATECHECK <rate> <bits> [dur_s]",
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	"Configure PSRAM circular staging buffer (on, off, status, burst, mode)",
	"Set batch log output mode: live | end | off",
#endif
#if defined(CONFIG_WILDSONIC_SD_FAULT_INJECT)
	"SD storage fault injection and resilience testing",
#endif
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

	TRACE_RAW("%s%04u-%02u-%02u %02u:%02u:%02u.000 %c%02u:%02u%s",
	          prefix,
	          cal.year, cal.month, cal.day,
	          cal.hour, cal.min, cal.sec,
	          tz_sign, tz_hr, tz_min,
	          suffix);
}

/* Print command names */
static void display_console_commands(void)
{
	TRACE_RAW("\nList of commands\n");
	for (size_t i = 0; i < CANONICAL_CMD_COUNT; i++) {
		TRACE_RAW(" %s \n", console_cmd_lookup_table[i]);
	}
	TRACE_RAW(" %s \n", console_cmd_lookup_table[CMD_CONFIG_WIZARD]);
	TRACE_RAW(" %s \n", console_cmd_lookup_table[CMD_STATS]);
	TRACE_RAW(" %s \n", console_cmd_lookup_table[CMD_RATECHECK]);
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	TRACE_RAW(" %s \n", console_cmd_lookup_table[CMD_PSRAM]);
	TRACE_RAW(" %s \n", console_cmd_lookup_table[CMD_BATCHLOG]);
#endif
	TRACE_RAW("\n");
}

/* Print HELP: command : description */
static void display_console_help(void)
{
	TRACE_RAW("\nList of commands\n");
	for (size_t i = 0; i < CANONICAL_CMD_COUNT; i++) {
		TRACE_RAW(" %s : %s\n", console_cmd_lookup_table[i], console_cmd_description_table[i]);
	}
	TRACE_RAW(" %s : %s\n", console_cmd_lookup_table[CMD_CONFIG_WIZARD], console_cmd_description_table[CMD_CONFIG_WIZARD]);
	TRACE_RAW(" %s : %s\n", console_cmd_lookup_table[CMD_STATS], console_cmd_description_table[CMD_STATS]);
	TRACE_RAW(" %s : %s\n", console_cmd_lookup_table[CMD_RATECHECK], console_cmd_description_table[CMD_RATECHECK]);
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	TRACE_RAW("\nEngineering Extensions (TEST build only):\n");
	TRACE_RAW(" %s : %s\n", console_cmd_lookup_table[CMD_PSRAM], console_cmd_description_table[CMD_PSRAM]);
	TRACE_RAW(" %s : %s\n", console_cmd_lookup_table[CMD_BATCHLOG], console_cmd_description_table[CMD_BATCHLOG]);
#if defined(CONFIG_WILDSONIC_SD_FAULT_INJECT)
	TRACE_RAW(" %s : %s\n", console_cmd_lookup_table[CMD_FAULT], console_cmd_description_table[CMD_FAULT]);
#endif
#endif
	TRACE_RAW("\n");
}

/* Explicit deferred hardware notice */
static int report_deferred_hardware(const char *cmd_name)
{
	TRACE_RAW("\n[DEFERRED] Command '%s' requires hardware not populated on STM32L476RG.\n", cmd_name);
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

static void console_fill_rate_check_params(rate_check_params_t *p,
                                           uint32_t rate_hz,
                                           uint16_t bits,
                                           uint16_t channels,
                                           uint32_t duration_s,
                                           uint32_t lpf_hz)
{
	struct app_settings *settings = main_get_settings();
	rate_check_init_params(p);
	p->nominal_rate_hz = rate_hz;
	p->bits = bits;
	p->channels = channels;
	p->duration_s = duration_s;
	p->psram_enabled = recorder_is_psram_staging_enabled() &&
	                   psram_is_present() && psram_is_quad_mode();
	if (settings && !settings->dsp_enabled) {
		p->dsp_cfg.mode = RATE_DSP_BYPASS_ALL;
	} else {
		p->dsp_cfg.mode = RATE_DSP_AUTO;
	}
	p->dsp_cfg.hpf_hz = g_cfg.lower_cutoff_frequency;
	p->dsp_cfg.lpf_hz = lpf_hz;
	p->dsp_cfg.notch_enabled = settings ? (settings->dsp_enabled && settings->notch_enabled) : false;
}

static void console_print_audio_warning(uint32_t rate_hz,
                                        uint16_t bits,
                                        uint16_t channels,
                                        uint32_t duration_s,
                                        const rate_check_result_t *res)
{
	const rate_check_margins_t *m = &res->margins;
	TRACE_RAW("\n[WARNING] %u Hz, %u-bit, %uch, dur %u s%s - will run\n",
	          rate_hz, bits, channels, duration_s,
	          m->experimental ? " [EXPERIMENTAL]" : "");
	if (res->actual_rate > 0) {
		TRACE_RAW("  Actual rate : %u Hz (TIM6 div %u, %d ppm)\n",
		          res->actual_rate, m->tim_divider, m->rate_err_ppm);
	}
	if (m->warn_storage) {
		if (m->max_duration_s > 0U && duration_s > m->max_duration_s) {
			TRACE_RAW("  Storage     : stream %u KB/s vs SD %u KB/s, expect overload after about %u s\n",
			          m->stream_kbps, m->sd_sustained_kbps, m->max_duration_s);
		} else if (m->max_duration_s > 0U) {
			TRACE_RAW("  Storage     : stream %u KB/s vs SD %u KB/s, PSRAM buffer ~%u s\n",
			          m->stream_kbps, m->sd_sustained_kbps, m->max_duration_s);
		} else {
			TRACE_RAW("  Storage     : stream %u KB/s vs SD %u KB/s, immediate overload\n",
			          m->stream_kbps, m->sd_sustained_kbps);
		}
	} else if (res->reason[0] != '\0') {
		TRACE_RAW("  Note        : %s\n", res->reason);
	}
}

static void console_print_rate_check_details(uint32_t rate_hz,
                                             uint16_t bits,
                                             uint16_t channels,
                                             uint32_t duration_s,
                                             const rate_check_result_t *res)
{
	const rate_check_margins_t *m = &res->margins;
	TRACE_RAW("\n[%s] %u Hz, %u-bit, %uch, dur %u s%s%s\n",
	          rate_status_to_str(res->status), rate_hz, bits, channels, duration_s,
	          m->experimental ? " [EXPERIMENTAL]" : "",
	          (res->status == RATE_STATUS_WARNING) ? " - will run" : "");
	if (res->actual_rate > 0) {
		TRACE_RAW("  Actual rate : %u Hz (TIM6 div %u, %d ppm)\n",
		          res->actual_rate, m->tim_divider, m->rate_err_ppm);
	}
	if (m->ovs_selected > 0) {
		TRACE_RAW("  ADC / OVS   : %ux (eff %u-bit ENOB), conv %u/%u ns (%u%% duty)\n",
		          m->ovs_selected, m->adc_effective_bits,
		          m->adc_conv_ns, m->sample_period_ns, m->adc_budget_pct);
	}
	if (m->block_samples > 0) {
		TRACE_RAW("  DSP / Block : %s (%u ns/smp, %u%% CPU), block %u/%u us (%u smp)\n",
		          rate_dsp_mode_to_str(m->effective_dsp_mode),
		          m->dsp_ns_per_sample, m->dsp_cpu_pct,
		          m->dsp_block_us, m->block_deadline_us, m->block_samples);
	}
	if (m->stream_kbps > 0) {
		TRACE_RAW("  Storage     : stream %u KB/s vs SD %u KB/s (margin %d%%), pool %u ms, max %u s\n",
		          m->stream_kbps, m->sd_sustained_kbps, m->sd_margin_pct,
		          m->pool_ms, m->max_duration_s);
	}
	if (res->reason[0] != '\0') {
		TRACE_RAW("  Reason      : %s\n", res->reason);
	}
}

static bool console_check_audio_config(uint32_t rate_hz,
                                       uint16_t bits,
                                       uint16_t channels,
                                       uint32_t duration_s,
                                       uint32_t lpf_hz,
                                       rate_check_result_t *res_out)
{
	rate_check_params_t p;
	rate_check_result_t local_res;
	rate_check_result_t *res = res_out ? res_out : &local_res;
	console_fill_rate_check_params(&p, rate_hz, bits, channels, duration_s, lpf_hz);
	rate_config_check_ex(&p, res);
	if (res->status == RATE_STATUS_IMPOSSIBLE) {
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "BLOCKED: %s", res->reason);
		}
		return false;
	}
	return true;
}

static void handle_ratecheck(const char *p_args, size_t args_length)
{
	uint32_t rate_hz = g_cfg.sampling_rate;
	uint32_t bits = g_cfg.audio_sampling_width ? g_cfg.audio_sampling_width : 16U;
	uint32_t dur_s = g_cfg.recording_length;
	uint32_t ch = g_cfg.num_audio_channels ? g_cfg.num_audio_channels : 1U;

	if (p_args && args_length > 0) {
		char argbuf[64];
		size_t n = (args_length < sizeof(argbuf) - 1U) ? args_length : (sizeof(argbuf) - 1U);
		memcpy(argbuf, p_args, n);
		argbuf[n] = '\0';
		unsigned long r = 0, b = bits, d = dur_s, c = ch;
		int parsed = sscanf(argbuf, "%lu %lu %lu %lu", &r, &b, &d, &c);
		if (parsed < 2) {
			TRACE_RAW("\nUsage: RATECHECK <rate_hz> <bits> [duration_s] [channels]\n");
			s_last_cmd_status = -EINVAL;
			return;
		}
		rate_hz = (uint32_t)r;
		bits = (uint32_t)b;
		dur_s = (parsed >= 3) ? (uint32_t)d : g_cfg.recording_length;
		ch = (parsed >= 4) ? (uint32_t)c : ch;
	}

	uint32_t lpf_hz = dsp_validate_lpf_cutoff(rate_hz, g_cfg.upper_cutoff_frequency);
	rate_check_params_t p;
	rate_check_result_t res;
	console_fill_rate_check_params(&p, rate_hz, (uint16_t)bits, (uint16_t)ch, dur_s, lpf_hz);
	rate_config_check_ex(&p, &res);
	console_print_rate_check_details(rate_hz, (uint16_t)bits, (uint16_t)ch, dur_s, &res);
	TRACE_RAW("  Verdict     : %s\n", rate_status_to_str(res.status));
	if (s_current_out_resp && s_current_out_resp_size > 0) {
		snprintf(s_current_out_resp, s_current_out_resp_size, "%s %uHz %ub max=%us%s%s",
		         rate_status_to_str(res.status),
		         res.actual_rate ? res.actual_rate : rate_hz,
		         (unsigned)bits, res.margins.max_duration_s,
		         res.reason[0] ? ": " : "", res.reason);
	}
	s_last_cmd_status = (res.status == RATE_STATUS_IMPOSSIBLE) ? -ERANGE : 0;
}

/* SAMF handler */
static void handle_samf(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %u\n", g_cfg.sampling_rate);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t raw_val = 0;
	if (parse_strict_u32(p_args, args_length, &raw_val) != 0) {
		TRACE_RAW("\n[BLOCKED] Invalid value %.*s. Value unchanged: %u\n",
		          (int)args_length, p_args, g_cfg.sampling_rate);
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
	else if (val == 384) val = 384000;
	else if (val == 512) val = 512000;
	else if (val == 768) val = 768000;

	uint16_t cur_bits = (uint16_t)(g_cfg.audio_sampling_width ? g_cfg.audio_sampling_width : 16U);
	uint16_t cur_ch = (uint16_t)(g_cfg.num_audio_channels ? g_cfg.num_audio_channels : 1U);
	uint32_t cand_lpf = g_cfg.upper_cutoff_frequency;
	if (cand_lpf == (g_cfg.sampling_rate / 2U) || cand_lpf >= ((val * 49U) / 100U)) {
		cand_lpf = dsp_validate_lpf_cutoff(val, (val * 45U) / 100U);
	}
	rate_check_result_t rc_res;
	if (!console_check_audio_config(val, cur_bits, cur_ch, g_cfg.recording_length,
	                                cand_lpf, &rc_res)) {
		TRACE_RAW("\n[BLOCKED] %s. Value unchanged: %u\n",
		          rc_res.reason, g_cfg.sampling_rate);
		s_last_cmd_status = -ERANGE;
		return;
	}

	uint32_t old_rate = g_cfg.sampling_rate;
	if (!recorder_is_active()) {
		int rc = audio_in_set_rate(val);
		if (rc != 0) {
			TRACE_RAW("\n[BLOCKED] Unsupported sample rate %u Hz (err %d). Value unchanged: %u\n",
			          val, rc, g_cfg.sampling_rate);
			s_last_cmd_status = rc;
			return;
		}
		g_cfg.sampling_rate = val;
		if (settings) {
			settings->sample_rate = val;
		}
		main_apply_dsp_settings();
	} else {
		/* Recording active: stage in RAM without modifying running session */
		g_cfg.sampling_rate = val;
	}

	if (rc_res.status == RATE_STATUS_WARNING) {
		console_print_audio_warning(val, cur_bits, cur_ch, g_cfg.recording_length, &rc_res);
		TRACE_RAW("  Value set to %u\n", val);
	} else {
		TRACE_RAW("\nValue is set to %u\n", val);
	}
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
		TRACE_RAW("\nUpper cutoff frequency adjusted to %u\n", g_cfg.upper_cutoff_frequency);
	}
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	char val_str[16];
	snprintf(val_str, sizeof(val_str), "%u", val);
	s_last_cmd_status = console_commit_and_ack_config("SAMF", val_str, recorder_is_active());
}

/* BTPS handler */
static void handle_btps(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %u\n", g_cfg.audio_sampling_width);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		TRACE_RAW("\n[BLOCKED] Invalid value %.*s. Value unchanged: %u\n",
		          (int)args_length, p_args, g_cfg.audio_sampling_width);
		s_last_cmd_status = -EINVAL;
		return;
	}

	if (val == 24) {
		TRACE_RAW("\n[DEFERRED] Bits per sample %d requires future hardware (STM32U585/external I2S codec).\n", (int)val);
		TRACE_RAW("[BLOCKED] Bit depth 24-bit not implemented. Value unchanged: %u\n",
		          g_cfg.audio_sampling_width);
		s_last_cmd_status = -ENOTSUP;
		return;
	}
	if (val != 16 && val != 32) {
		TRACE_RAW("\n[BLOCKED] Bit depth %u unsupported (supported: 16, 32). Value unchanged: %u\n",
		          val, g_cfg.audio_sampling_width);
		s_last_cmd_status = -EINVAL;
		return;
	}

	uint16_t cur_ch = (uint16_t)(g_cfg.num_audio_channels ? g_cfg.num_audio_channels : 1U);
	rate_check_result_t rc_res;
	if (!console_check_audio_config(g_cfg.sampling_rate, (uint16_t)val, cur_ch,
	                                g_cfg.recording_length,
	                                g_cfg.upper_cutoff_frequency, &rc_res)) {
		TRACE_RAW("\n[BLOCKED] %s. Value unchanged: %u\n",
		          rc_res.reason, g_cfg.audio_sampling_width);
		s_last_cmd_status = -ERANGE;
		return;
	}

	g_cfg.audio_sampling_width = val;
	if (settings && !recorder_is_active()) {
		settings->sample_bits = (uint16_t)val;
	}
	if (rc_res.status == RATE_STATUS_WARNING) {
		console_print_audio_warning(g_cfg.sampling_rate, (uint16_t)val, cur_ch,
		                            g_cfg.recording_length, &rc_res);
		TRACE_RAW("  Value set to %u\n", val);
	} else {
		TRACE_RAW("\nValue is set to %u\n", val);
	}
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	char bstr[8];
	snprintf(bstr, sizeof(bstr), "%u", val);
	s_last_cmd_status = console_commit_and_ack_config("BTPS", bstr, recorder_is_active());
}

/* NUMCH handler */
static void handle_numch(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %u\n", g_cfg.num_audio_channels);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0 || (val != 1 && val != 2)) {
		TRACE_RAW("\n[BLOCKED] Invalid channel count %.*s (supported: 1, 2). Value unchanged: %u\n",
		          (int)args_length, p_args, g_cfg.num_audio_channels);
		s_last_cmd_status = -EINVAL;
		return;
	}

	uint16_t cur_bits = (uint16_t)(g_cfg.audio_sampling_width ? g_cfg.audio_sampling_width : 16U);
	rate_check_result_t rc_res;
	if (!console_check_audio_config(g_cfg.sampling_rate, cur_bits, (uint16_t)val,
	                                g_cfg.recording_length,
	                                g_cfg.upper_cutoff_frequency, &rc_res)) {
		TRACE_RAW("\n[BLOCKED] %s. Value unchanged: %u\n",
		          rc_res.reason, g_cfg.num_audio_channels);
		s_last_cmd_status = -ERANGE;
		return;
	}

	g_cfg.num_audio_channels = val;
	if (settings && !recorder_is_active()) {
		settings->channels = (uint16_t)val;
	}
	if (rc_res.status == RATE_STATUS_WARNING) {
		console_print_audio_warning(g_cfg.sampling_rate, cur_bits, (uint16_t)val,
		                            g_cfg.recording_length, &rc_res);
		TRACE_RAW("  Value set to %u\n", val);
	} else {
		TRACE_RAW("\nValue is set to %u\n", val);
	}
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	char cstr[8];
	snprintf(cstr, sizeof(cstr), "%u", val);
	s_last_cmd_status = console_commit_and_ack_config("NUMCH", cstr, recorder_is_active());
}

#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
static void handle_source(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();
	if (args_length == 0) {
		enum audio_capture_source cur = audio_in_get_capture_source();
		const char *s = (cur == AUDIO_SRC_PHYSICAL_MIC) ? "MIC" :
		                (cur == AUDIO_SRC_STEREO_DUPLICATE) ? "STEREO_DUP" : "SYNTH";
		TRACE_RAW("\nCapture source is %s\n", s);
		return;
	}
	if (strncasecmp(p_args, "MIC", 3) == 0) {
		audio_in_set_capture_source(AUDIO_SRC_PHYSICAL_MIC);
		if (settings) settings->capture_source = AUDIO_SRC_PHYSICAL_MIC;
		TRACE_RAW("\nCapture source set to MIC (physical analog microphone)\n");
	} else if (strncasecmp(p_args, "STEREO_DUP", 10) == 0 || strncasecmp(p_args, "DUP", 3) == 0) {
		audio_in_set_capture_source(AUDIO_SRC_STEREO_DUPLICATE);
		if (settings) settings->capture_source = AUDIO_SRC_STEREO_DUPLICATE;
		TRACE_RAW("\nCapture source set to STEREO_DUP (mono ADC duplicated to stereo)\n");
	} else if (strncasecmp(p_args, "SYNTH", 5) == 0) {
		audio_in_set_capture_source(AUDIO_SRC_SYNTHETIC);
		if (settings) settings->capture_source = AUDIO_SRC_SYNTHETIC;
		TRACE_RAW("\nCapture source set to SYNTH (synthetic deterministic tone)\n");
	} else {
		TRACE_RAW("\nInvalid source '%s'. Valid: MIC, STEREO_DUP, SYNTH\n", p_args);
	}
}
#endif

/* AUDIO parameters summary / packed setter */
static void handle_audio(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		TRACE_RAW("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
		TRACE_RAW("Sampling frequency set to %u\n", g_cfg.sampling_rate);
		TRACE_RAW("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
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

		uint32_t cand_lpf = g_cfg.upper_cutoff_frequency;
		if (cand_lpf == (g_cfg.sampling_rate / 2U) || cand_lpf >= ((new_samf * 49U) / 100U)) {
			cand_lpf = dsp_validate_lpf_cutoff(new_samf, (new_samf * 45U) / 100U);
		}
		rate_check_result_t audio_rc;
		if ((new_numch == 1 || new_numch == 2) &&
		    (new_btps == 16 || new_btps == 32)) {
			if (!console_check_audio_config(new_samf, (uint16_t)new_btps, (uint16_t)new_numch,
			                                g_cfg.recording_length, cand_lpf, &audio_rc)) {
				TRACE_RAW("\n[BLOCKED] %s. Value unchanged.\n", audio_rc.reason);
				TRACE_RAW("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
				TRACE_RAW("Sampling frequency set to %u\n", g_cfg.sampling_rate);
				TRACE_RAW("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
				s_last_cmd_status = -ERANGE;
				return;
			}
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

			if (audio_rc.status == RATE_STATUS_WARNING) {
				console_print_audio_warning(new_samf, (uint16_t)new_btps, (uint16_t)new_numch,
				                            g_cfg.recording_length, &audio_rc);
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
				TRACE_RAW("\nUpper cutoff frequency adjusted to %u\n", g_cfg.upper_cutoff_frequency);
			}

			TRACE_RAW("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
			TRACE_RAW("Sampling frequency set to %u\n", g_cfg.sampling_rate);
			TRACE_RAW("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
			config_store_stage_pending();
			TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
			s_last_cmd_status = 0;
			return;
		}
	}

	TRACE_RAW("\nInvalid values\n");
	TRACE_RAW("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
	TRACE_RAW("Sampling frequency set to %u\n", g_cfg.sampling_rate);
	TRACE_RAW("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
	s_last_cmd_status = -EINVAL;
}

/* RECT handler */
static void handle_rect(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %u\n", g_cfg.recording_length);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0 || val == 0 || val > 86400) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", g_cfg.recording_length);
		s_last_cmd_status = -EINVAL;
		return;
	}

	uint16_t cur_bits = (uint16_t)(g_cfg.audio_sampling_width ? g_cfg.audio_sampling_width : 16U);
	uint16_t cur_ch = (uint16_t)(g_cfg.num_audio_channels ? g_cfg.num_audio_channels : 1U);
	rate_check_result_t rc_res;
	if (!console_check_audio_config(g_cfg.sampling_rate, cur_bits, cur_ch, val,
	                                g_cfg.upper_cutoff_frequency, &rc_res)) {
		TRACE_RAW("\n[BLOCKED] %s. Value unchanged: %u\n",
		          rc_res.reason, g_cfg.recording_length);
		s_last_cmd_status = -ERANGE;
		return;
	}

	g_cfg.recording_length = val;
	if (settings && !recorder_is_active() &&
	    settings->sample_rate == g_cfg.sampling_rate &&
	    settings->sample_bits == (uint16_t)g_cfg.audio_sampling_width) {
		settings->record_seconds = val;

		struct schedule_config scfg;
		scheduler_get_config(&scfg);
		scfg.record_length = val;
		scheduler_set_config(&scfg);
		main_scheduler_resume();
	}

	if (rc_res.status == RATE_STATUS_WARNING) {
		console_print_audio_warning(g_cfg.sampling_rate, cur_bits, cur_ch, val, &rc_res);
		TRACE_RAW("  Value set to %u\n", g_cfg.recording_length);
	} else {
		TRACE_RAW("\nValue is set to %u\n", g_cfg.recording_length);
	}
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
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
		TRACE_RAW("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
		s_last_cmd_status = 0;
		return;
	}

	if (p_args[0] == '-') {
		TRACE_RAW("\nInvalid value: negative values are not allowed\n");
		TRACE_RAW("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
		s_last_cmd_status = -EINVAL;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0 || val > 86400) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
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

	TRACE_RAW("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
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
				TRACE_RAW("\nInvalid time zone %s\n", tz_token);
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
			TRACE_RAW("\n[ERROR] Failed to set RTC hardware calendar (%d)\n", set_res);
			s_last_cmd_status = set_res;
			return;
		}
		g_cfg.boot_time_utc = utc_epoch;

		console_printf_time("\nBoot time UTC set to ", utc_epoch, 0, "\n");
		console_printf_time("\nBoot time Local set to ", utc_epoch, scfg.time_zone_offset, "\n");
		console_printf_time("\nCurrent time UTC time is ", utc_epoch, 0, "\n");
		console_printf_time("\nCurrent time Local is ", utc_epoch, scfg.time_zone_offset, "\n");
		main_scheduler_resume();
		TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = 0;
		return;
	}

	TRACE_RAW("\nInvalid date time %.*s\nUTC Time Format should be YYYY-MM-DD HH:MM:SS\n", (int)args_length, p_args);
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
		TRACE_RAW("\nTime zone is set to %c%02u:%02u\n", sign, tz_hr, tz_min);
		s_last_cmd_status = 0;
		return;
	}

	int32_t offset = 0;
	if (scheduler_parse_timezone(p_args, &offset) == 0) {
		scfg.time_zone_offset = offset;
		g_cfg.time_zone_offset = offset;
		scheduler_set_config(&scfg);
		main_scheduler_resume();
		TRACE_RAW("\nTime zone set to %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("TIMEZONE", p_args, recorder_is_active());
		return;
	}

	TRACE_RAW("\nInvalid time zone argument %.*s\n", (int)args_length, p_args);
	s_last_cmd_status = -EINVAL;
}

/* DATERANGE handler */
static void handle_daterange(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0) {
		TRACE_RAW("\nBegin end date set to %04u/%02u/%02u-%04u/%02u/%02u\n",
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
		TRACE_RAW("\nBegin end date set to %04u/%02u/%02u-%04u/%02u/%02u\n",
		          scfg.start_date.year, scfg.start_date.month, scfg.start_date.day,
		          scfg.end_date.year, scfg.end_date.month, scfg.end_date.day);
		TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("DATERANGE", p_args, recorder_is_active());
		return;
	}

	TRACE_RAW("\nInvalid begin end date %.*s\n", (int)args_length, p_args);
	TRACE_RAW("\nBegin end date set to %04u/%02u/%02u-%04u/%02u/%02u\n",
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
				TRACE_RAW("\nBegin end time (Index %u) set to %02u:%02u:%02u-%02u:%02u:%02u\n",
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
			TRACE_RAW("\nNo valid time of day ranges set\n");
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
				TRACE_RAW("\nBegin end time (Index %u) set to %02u:%02u:%02u-%02u:%02u:%02u\n",
				          i,
				          scfg.tod_ranges[i].start_second / 3600,
				          (scfg.tod_ranges[i].start_second / 60) % 60,
				          scfg.tod_ranges[i].start_second % 60,
				          scfg.tod_ranges[i].end_second / 3600,
				          (scfg.tod_ranges[i].end_second / 60) % 60,
				          scfg.tod_ranges[i].end_second % 60);
			}
		}
		TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("TIMEOFDAY", p_args, recorder_is_active());
		return;
	}

	TRACE_RAW("\nInvalid begin end time %.*s\n", (int)args_length, p_args);
	s_last_cmd_status = -EINVAL;
}

static void format_days_of_week(uint8_t mask, char *out, size_t out_size)
{
	static const char letters[] = "MTWRFSU";
	size_t idx = 0;

	if (!out || out_size == 0) {
		return;
	}
	for (size_t i = 0; i < 7U && idx + 1U < out_size; i++) {
		if (mask & (1U << i)) {
			out[idx++] = letters[i];
		}
	}
	out[idx] = '\0';
}

/* DAYSOFWEEK handler */
static void handle_daysofweek(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	char dow[8];

	if (args_length == 0) {
		format_days_of_week(scfg.day_of_week_mask, dow, sizeof(dow));
		TRACE_RAW("\nDays of week set to \"%s\"\n", dow);
		s_last_cmd_status = 0;
		return;
	}

	if ((args_length == 1 && (p_args[0] == 'X' || p_args[0] == 'x')) ||
	    (args_length == 2 && p_args[0] == '"' && p_args[1] == '"') ||
	    (args_length == 3 && p_args[0] == '"' && (p_args[1] == 'X' || p_args[1] == 'x') && p_args[2] == '"')) {
		TRACE_RAW("\nDays of week: X is set to all days (MTWRFSU)\n");
	}

	uint8_t mask = 0;
	if (scheduler_parse_days_of_week(p_args, &mask) == 0) {
		scfg.day_of_week_mask = mask;
		scheduler_set_config(&scfg);
		main_scheduler_resume();
		format_days_of_week(scfg.day_of_week_mask, dow, sizeof(dow));
		TRACE_RAW("\nDays of week set to \"%s\"\n", dow);
		TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = console_commit_and_ack_config("DAYSOFWEEK", p_args, recorder_is_active());
		return;
	}

	TRACE_RAW("\nInvalid days of week %.*s\n", (int)args_length, p_args);
	s_last_cmd_status = -EINVAL;
}

/* GAIN handler */
static void handle_gain(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		TRACE_RAW("\nAudio gain set to 0.1*%d\n", g_cfg.mic_gain_times_ten);
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
		TRACE_RAW("\nInvalid audio gain %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nAudio gain set to 0.1*%d\n", g_cfg.mic_gain_times_ten);
		s_last_cmd_status = -EINVAL;
		return;
	}

	int gain_t10 = (int)roundf((float)(sign * val * 10.0));
	if (gain_t10 < -400 || gain_t10 > 400) {
		TRACE_RAW("\nInvalid audio gain 0.1*%d\n", gain_t10);
		TRACE_RAW("\nAudio gain set to 0.1*%d\n", g_cfg.mic_gain_times_ten);
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

	TRACE_RAW("\nAudio gain set to 0.1*%d\n", gain_t10);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	char gain_str[16];
	snprintf(gain_str, sizeof(gain_str), "0.1*%d", gain_t10);
	s_last_cmd_status = console_commit_and_ack_config("GAIN", gain_str, recorder_is_active());
}

/* LCUTOFF (HPF) handler */
static void handle_lcutoff(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %u\n", g_cfg.lower_cutoff_frequency);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t hz = 0;
	if (parse_strict_u32(p_args, args_length, &hz) != 0 || (hz != 0 && (hz < 10 || hz > 2000))) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", g_cfg.lower_cutoff_frequency);
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

	TRACE_RAW("\nValue is set to %u\n", hz);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	char lcut_str[16];
	snprintf(lcut_str, sizeof(lcut_str), "%u", hz);
	s_last_cmd_status = console_commit_and_ack_config("LCUTOFF", lcut_str, recorder_is_active());
}

/* UCUTOFF (LPF) handler */
static void handle_ucutoff(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %u\n", g_cfg.upper_cutoff_frequency);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t hz = 0;
	uint32_t nyquist_bound = (g_cfg.sampling_rate * 49) / 100;
	if (parse_strict_u32(p_args, args_length, &hz) != 0 || (hz != 0 && (hz < 1000 || hz >= nyquist_bound))) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", g_cfg.upper_cutoff_frequency);
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

	TRACE_RAW("\nValue is set to %u\n", hz);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
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
		TRACE_RAW("\nValue is set to %s\n", mode_str);
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
		TRACE_RAW("\nInvalid value %s\n", str);
		s_last_cmd_status = -EINVAL;
		return;
	}

	const char *mode_str = (g_cfg.folder_structure == 0) ? "flat" :
			       ((g_cfg.folder_structure == 1) ? "daily" : "hourly");
	TRACE_RAW("\nValue is set to %s\n", mode_str);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
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
		TRACE_RAW("The total number of bytes written: %lld and the available space: %lld\n",
		          (long long)written, (long long)free_b);
	} else {
		TRACE_RAW("The total number of bytes written: 0 and the available space: 0\n");
	}
}

/* VERSION handler */
static void handle_version(void)
{
	TRACE_RAW("\nVersion: v1.2.0\n");
	TRACE_RAW("Git commit: %s\n", WILDSONIC_GIT_COMMIT);
	TRACE_RAW("Build Date: %s %s\n", __DATE__, __TIME__);
}

/* SN handler (max 3 alphanumeric chars per original WildSonic) */
static int handle_sn(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %s\n", g_cfg.serial_number);
		return 0;
	}

	if (args_length > 3) {
		TRACE_RAW("\nInvalid value %s\n", p_args);
		TRACE_RAW("\nValue is set to %s\n", g_cfg.serial_number);
		return -EINVAL;
	}

	for (size_t i = 0; i < args_length; i++) {
		if (!isalnum((unsigned char)p_args[i])) {
			TRACE_RAW("\nInvalid value %s\n", p_args);
			TRACE_RAW("\nValue is set to %s\n", g_cfg.serial_number);
			return -EINVAL;
		}
	}

	strncpy(g_cfg.serial_number, p_args, sizeof(g_cfg.serial_number) - 1);
	g_cfg.serial_number[sizeof(g_cfg.serial_number) - 1] = '\0';
	TRACE_RAW("\nValue is set to %s\n", g_cfg.serial_number);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	ble_comm_notify_name_changed();
	console_commit_and_ack_config("SN", g_cfg.serial_number, recorder_is_active());
	return 0;
}

/* DNAME handler (max 6 alphanumeric chars per original WildSonic) */
static int handle_dname(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %s\n", g_cfg.device_name);
		return 0;
	}

	if (args_length > 6) {
		TRACE_RAW("\nInvalid value %s\n", p_args);
		TRACE_RAW("\nValue is set to %s\n", g_cfg.device_name);
		return -EINVAL;
	}

	for (size_t i = 0; i < args_length; i++) {
		if (!isalnum((unsigned char)p_args[i])) {
			TRACE_RAW("\nInvalid value %s\n", p_args);
			TRACE_RAW("\nValue is set to %s\n", g_cfg.device_name);
			return -EINVAL;
		}
	}

	strncpy(g_cfg.device_name, p_args, sizeof(g_cfg.device_name) - 1);
	g_cfg.device_name[sizeof(g_cfg.device_name) - 1] = '\0';
	TRACE_RAW("\nValue is set to %s\n", g_cfg.device_name);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	ble_comm_notify_name_changed();
	console_commit_and_ack_config("DNAME", g_cfg.device_name, recorder_is_active());
	return 0;
}

/* PW handler */
static int handle_pw(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		TRACE_RAW("\nValue is set to %s\n", g_cfg.device_password);
		return 0;
	}

	if (args_length >= sizeof(g_cfg.device_password)) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %s\n", g_cfg.device_password);
		return -EINVAL;
	}

	strncpy(g_cfg.device_password, p_args, sizeof(g_cfg.device_password) - 1);
	g_cfg.device_password[sizeof(g_cfg.device_password) - 1] = '\0';
	TRACE_RAW("\nValue is set to %s\n", g_cfg.device_password);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
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
static void handle_assert(const char *p_args, size_t args_length)
{
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	if (p_args && args_length >= 4 && strncasecmp(p_args, "test", 4) == 0 &&
	    (args_length == 4 || isspace((unsigned char)p_args[4]))) {
		SYSTEM_ASSERT(ASSERT_ID_GENERIC_SAFETY_FAULT, false);
		return;
	}
#else
	ARG_UNUSED(p_args);
	ARG_UNUSED(args_length);
#endif

	struct retained_reboot_info rinfo;
	if (retained_reboot_get(&rinfo) == REBOOT_REASON_ASSERT && rinfo.assert_id != 0U) {
		TRACE_RAW("ASSERT 0x%08X %s #%u\n",
		          (unsigned int)rinfo.assert_id,
		          rinfo.assert_file[0] ? rinfo.assert_file : "unknown",
		          (unsigned int)rinfo.assert_line);
		return;
	}

	if (g_assert_info.asserted) {
		TRACE_RAW("ASSERT IN FILE %s #%d: %s\n",
		          g_assert_info.file ? g_assert_info.file : "unknown",
		          g_assert_info.line,
		          g_assert_info.expr ? g_assert_info.expr : "condition");
	} else if (g_boot_csr & (RCC_CSR_SFTRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_LPWRRSTF)) {
		TRACE_RAW("Assert / Reset Reason: CSR=0x%08X (SFTRST=%d, IWDG=%d, WWDG=%d, LPWR=%d)\n",
		          (unsigned int)g_boot_csr,
		          !!(g_boot_csr & RCC_CSR_SFTRSTF),
		          !!(g_boot_csr & RCC_CSR_IWDGRSTF),
		          !!(g_boot_csr & RCC_CSR_WWDGRSTF),
		          !!(g_boot_csr & RCC_CSR_LPWRRSTF));
	} else {
		TRACE_RAW("There is no assert information \n");
	}
}

/* STATS handler */
static void handle_stats(void)
{
	recorder_print_last_stats();
	recorder_print_stack_free_stats();
}

#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
/* BATCHLOG handler */
static void handle_batchlog(const char *p_args, size_t args_length)
{
	if (!p_args || args_length == 0) {
		enum recorder_batchlog_mode mode = recorder_get_batchlog_mode();
		const char *mode_str = (mode == BATCHLOG_LIVE) ? "live" :
				       (mode == BATCHLOG_END) ? "end" : "off";
		TRACE_RAW("batchlog is %s\n", mode_str);
		s_last_cmd_status = 0;
		return;
	}

	if (args_length >= 4 && strncasecmp(p_args, "live", 4) == 0 &&
	    (args_length == 4 || isspace((unsigned char)p_args[4]))) {
		recorder_set_batchlog_mode(BATCHLOG_LIVE);
		TRACE_RAW("batchlog set to live\n");
	} else if (args_length >= 3 && strncasecmp(p_args, "end", 3) == 0 &&
	           (args_length == 3 || isspace((unsigned char)p_args[3]))) {
		recorder_set_batchlog_mode(BATCHLOG_END);
		TRACE_RAW("batchlog set to end\n");
	} else if (args_length >= 3 && strncasecmp(p_args, "off", 3) == 0 &&
	           (args_length == 3 || isspace((unsigned char)p_args[3]))) {
		recorder_set_batchlog_mode(BATCHLOG_OFF);
		TRACE_RAW("batchlog set to off\n");
	} else {
		TRACE_RAW("Usage: batchlog [live|end|off]\n");
		s_last_cmd_status = -EINVAL;
		return;
	}
	s_last_cmd_status = 0;
}
#endif /* CONFIG_WILDSONIC_TEST_OUTPUT */

/* ==============================================================================
 * Location and Solar Scheduling Command Handlers
 * ============================================================================== */

static void print_current_location(void)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (scfg.latitude_e7 == SCHEDULER_INVALID_LOCATION_COORDINATE ||
	    scfg.longitude_e7 == SCHEDULER_INVALID_LOCATION_COORDINATE) {
		TRACE_RAW("Current_Location: Not set\n");
		return;
	}

	TRACE_RAW("Current_Location: %.7f, %.7f\n",
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
		TRACE_RAW("\nLocation cleared\n");
		print_current_location();
		TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
		s_last_cmd_status = 0;
		return;
	}

	double lat = 0.0, lon = 0.0;
	if (!parse_location_coordinates(p_args, args_length, &lat, &lon)) {
		TRACE_RAW("\nInvalid location. Use: LOC <latitude> <longitude>\n");
		print_current_location();
		s_last_cmd_status = -EINVAL;
		return;
	}

	if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
		TRACE_RAW("\nInvalid location range. Latitude [-90,90], Longitude [-180,180]\n");
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

	TRACE_RAW("\nLocation set to %.7f, %.7f\n", lat, lon);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sun(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (scfg.latitude_e7 == SCHEDULER_INVALID_LOCATION_COORDINATE ||
	    scfg.longitude_e7 == SCHEDULER_INVALID_LOCATION_COORDINATE) {
		TRACE_RAW("\nLocation is not set. Use: LOC <latitude> <longitude>\n");
		s_last_cmd_status = -EINVAL;
		return;
	}

	int year = 0, month = 0, day = 0;

	if (!p_args || args_length == 0) {
		struct rtc_calendar_time cur_cal;
		if (rtc_calendar_get(&cur_cal) != 0 || !rtc_calendar_is_valid()) {
			TRACE_RAW("\nRTC is not set. Use: SUN [YYYY/MM/DD]\n");
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
			TRACE_RAW("\nInvalid date. Use: SUN [YYYY/MM/DD]\n");
			s_last_cmd_status = -EINVAL;
			return;
		}
		memcpy(date_buf, p_args, args_length);
		date_buf[args_length] = '\0';

		if (sscanf(date_buf, "%d/%d/%d", &year, &month, &day) != 3 ||
		    year < 1970 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31) {
			TRACE_RAW("\nInvalid date. Use: SUN [YYYY/MM/DD]\n");
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
		TRACE_RAW("\nSunrise/sunset is not available for this date and location\n");
		s_last_cmd_status = -EDOM;
		return;
	}

	TRACE_RAW("\nSun date: %04d/%02d/%02d\n", year, month, day);
	TRACE_RAW("Julian date: %.2f\n", julian_date);
	TRACE_RAW("Sunrise time: %02u:%02u:%02u\n",
	          sunrise_sec / 3600, (sunrise_sec / 60) % 60, sunrise_sec % 60);
	TRACE_RAW("Sunset time: %02u:%02u:%02u\n",
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
			TRACE_RAW("\nSun recording mode: Sunrise\n");
			break;
		case SUN_MODE_SUNSET:
			TRACE_RAW("\nSun recording mode: Sunset\n");
			break;
		case SUN_MODE_SUNRISE_AND_SUNSET:
			TRACE_RAW("\nSun recording mode: Sunrise and Sunset\n");
			break;
		case SUN_MODE_SUNRISE_TO_SUNSET:
			TRACE_RAW("\nSun recording mode: Sunrise to Sunset\n");
			break;
		case SUN_MODE_SUNSET_TO_SUNRISE:
			TRACE_RAW("\nSun recording mode: Sunset to Sunrise\n");
			break;
		default:
			TRACE_RAW("\nSun recording mode: Disabled\n");
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

			TRACE_RAW("\nSun recording mode set to: %s\n", sun_mode_table[i].label);
			TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
			s_last_cmd_status = 0;
			return;
		}
	}

	TRACE_RAW("\nInvalid sun recording mode: %.*s\n", (int)args_length, p_args);
	TRACE_RAW("Valid modes: Sunrise, Sunset, Sunrise and Sunset, Sunrise to Sunset, Sunset to Sunrise, Disabled\n");
	s_last_cmd_status = -EINVAL;
}

static void handle_sunrisebef(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		TRACE_RAW("\nValue is set to %u\n", scfg.sunrise_interval_before);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", scfg.sunrise_interval_before);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunrise_interval_before = val;
	g_cfg.sunrise_interval_before = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	TRACE_RAW("\nValue is set to %u\n", val);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sunriseaft(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		TRACE_RAW("\nValue is set to %u\n", scfg.sunrise_interval_after);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", scfg.sunrise_interval_after);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunrise_interval_after = val;
	g_cfg.sunrise_interval_after = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	TRACE_RAW("\nValue is set to %u\n", val);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sunsetbef(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		TRACE_RAW("\nValue is set to %u\n", scfg.sunset_interval_before);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", scfg.sunset_interval_before);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunset_interval_before = val;
	g_cfg.sunset_interval_before = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	TRACE_RAW("\nValue is set to %u\n", val);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sunsetaft(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		TRACE_RAW("\nValue is set to %u\n", scfg.sunset_interval_after);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", scfg.sunset_interval_after);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunset_interval_after = val;
	g_cfg.sunset_interval_after = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	TRACE_RAW("\nValue is set to %u\n", val);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

static void handle_sunround(const char *p_args, size_t args_length)
{
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

	if (args_length == 0 || !p_args) {
		TRACE_RAW("\nValue is set to %u\n", scfg.sunrise_sunset_rounding);
		s_last_cmd_status = 0;
		return;
	}

	uint32_t val = 0;
	if (parse_strict_u32(p_args, args_length, &val) != 0) {
		TRACE_RAW("\nInvalid value %.*s\n", (int)args_length, p_args);
		TRACE_RAW("\nValue is set to %u\n", scfg.sunrise_sunset_rounding);
		s_last_cmd_status = -EINVAL;
		return;
	}

	scfg.sunrise_sunset_rounding = val;
	g_cfg.sunrise_sunset_rounding = val;
	scheduler_set_config(&scfg);
	g_cfg.save_config_flag = true;
	TRACE_RAW("\nValue is set to %u\n", val);
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

/* DEFAULT handler */
static void handle_default(void)
{
	main_reset_to_defaults();
	config_store_load_defaults();
	config_store_stage_pending();
	TRACE_RAW("\nDefault device configurations are set\n");
	TRACE_RAW("\nSend APPLY command and reboot for changes to take effect\n");
	s_last_cmd_status = 0;
}

/* GET_STATUS handler */
static void handle_get_status(void)
{
	TRACE_RAW("\n=== SYSTEM & RECORDING STATUS ===\n");
	if (recorder_is_active()) {
		TRACE_RAW("  Recording State       : ACTIVE\n");
		char cur_file[80] = {0};
		recorder_get_active_file(cur_file, sizeof(cur_file));
		if (cur_file[0] != '\0') {
			TRACE_RAW("  Active File           : %s\n", cur_file);
		}
		uint64_t samples_written = 0, expected_samples = 0;
		recorder_get_active_progress(&samples_written, &expected_samples);
		TRACE_RAW("  Samples Written       : %llu / %llu\n",
		          (unsigned long long)samples_written, (unsigned long long)expected_samples);
		TRACE_RAW("  Active Sample Rate    : %u Hz\n", audio_in_rate());
		TRACE_RAW("  Active Bit Depth      : %u-bit\n", g_cfg.audio_sampling_width);
		TRACE_RAW("  Active Channels       : %u (%s)\n",
		          g_cfg.num_audio_channels,
		          (g_cfg.num_audio_channels == 2) ? "Duplicated Mono Container" : "Mono");
	} else {
		TRACE_RAW("  Recording State       : IDLE\n");
		TRACE_RAW("  Scheduler Status      : %s\n",
		          main_is_scheduler_paused() ? "PAUSED" : "RUNNING");
		TRACE_RAW("  Configured Rate       : %u Hz\n", g_cfg.sampling_rate);
		TRACE_RAW("  Configured Bit Depth  : %u-bit\n", g_cfg.audio_sampling_width);
		TRACE_RAW("  Configured Channels   : %u\n", g_cfg.num_audio_channels);
		TRACE_RAW("  Configured Record Time: %u seconds\n", g_cfg.recording_length);
		TRACE_RAW("  Configured Sleep Time : %u seconds\n", g_cfg.sleep_pause_interval);
	}

	if (rtc_calendar_is_valid()) {
		struct rtc_calendar_time cur_cal;
		if (rtc_calendar_get(&cur_cal) == 0) {
			char rtc_str[32];
			rtc_calendar_format(&cur_cal, rtc_str, sizeof(rtc_str));
			TRACE_RAW("  RTC Current Time      : %s UTC\n", rtc_str);
		}
	} else {
		TRACE_RAW("  RTC Current Time      : UNINITIALIZED\n");
	}

	int last_sd_err = recorder_get_last_sd_error();
	if (last_sd_err != 0) {
		TRACE_RAW("  Last Storage Error    : %d\n", last_sd_err);
	}
	TRACE_RAW("OK\n");
}

/* GET_DEV_INFO handler */
static void handle_get_dev_info(void)
{
	TRACE_RAW("\n=== DEVICE INFORMATION ===\n");
	TRACE_RAW("  Hardware Platform     : STM32L476RG Nucleo-64 (ARM Cortex-M4F)\n");
	TRACE_RAW("  Core Clock Frequency  : 72 MHz (PLL HSI)\n");
	TRACE_RAW("  Audio Acquisition     : 12-bit SAR ADC, TIM6 TRGO DMA\n");
	TRACE_RAW("  External Memory       : ISSI IS66WVS4M8 4MB Quad-SPI PSRAM @ 18 MHz\n");
	TRACE_RAW("  Storage Media         : MicroSD Card via SDMMC1 4-bit @ 6 MHz (FatFs /SD:)\n");
	TRACE_RAW("  SRAM Architecture     : 128 KiB (SRAM1: 96 KiB, SRAM2: 32 KiB)\n");
	TRACE_RAW("  Flash Memory          : 1024 KiB Dual-Bank\n");
	TRACE_RAW("  Power Management      : STM32L4 Stop 2 Deep Sleep (Strict RTC Wakeup)\n");
	TRACE_RAW("  Firmware Version      : 2.1.0-prod (Zephyr RTOS)\n");
	TRACE_RAW("OK\n");
}

/* GET_CONFIG handler */
static void handle_get_config(void)
{
	TRACE_RAW("\n=== DEVICE CONFIGURATION ===\n");
	TRACE_RAW("  Serial Number (SN)    : %s\n", g_cfg.serial_number);
	TRACE_RAW("  Device Name (DNAME)   : %s\n", g_cfg.device_name);
	TRACE_RAW("  Sampling Rate (SAMF)  : %u Hz\n", g_cfg.sampling_rate);
	TRACE_RAW("  Bits Per Sample (BTPS): %u-bit\n", g_cfg.audio_sampling_width);
	TRACE_RAW("  Channels (NUMCH)      : %u (%s)\n",
	          g_cfg.num_audio_channels,
	          (g_cfg.num_audio_channels == 2) ? "Duplicated Mono Container" : "Mono");
	TRACE_RAW("  Record Time (RECT)    : %u seconds\n", g_cfg.recording_length);
	TRACE_RAW("  Sleep Interval (SLEEP): %u seconds\n", g_cfg.sleep_pause_interval);
	TRACE_RAW("  Mic Gain (GAIN)       : %.1f dB\n", (double)((float)g_cfg.mic_gain_times_ten / 10.0f));
	TRACE_RAW("  Lower Cutoff (LCUTOFF): %u Hz\n", g_cfg.lower_cutoff_frequency);
	TRACE_RAW("  Upper Cutoff (UCUTOFF): %u Hz\n", g_cfg.upper_cutoff_frequency);
	TRACE_RAW("  Folder Mode (FLDR)    : %u (%s)\n",
	          g_cfg.folder_structure,
	          (g_cfg.folder_structure == 0) ? "Flat" :
	          (g_cfg.folder_structure == 1) ? "Daily" : "Hourly");
	TRACE_RAW("OK\n");
}

/* STOP_RECORDING handler */
static void handle_stop_recording(void)
{
	if (recorder_is_active()) {
		recorder_request_stop();
		TRACE_RAW("\nOK: Stop recording requested; finalizing WAV container and closing file.\n");
	} else {
		TRACE_RAW("\nOK: Recorder is already idle.\n");
	}
}

#if defined(CONFIG_WILDSONIC_SD_FAULT_INJECT)
static void handle_fault(const char *args)
{
	char subcmd[16] = {0};
	uint32_t val = 0;
	int count = sscanf(args, "%15s %u", subcmd, &val);

	if (count < 1 || strcasecmp(subcmd, "status") == 0) {
		sd_fault_inject_status();
		struct sd_resilient_stats st;
		sd_resilient_get_stats(&st);
		TRACE_RAW("\n=== SD RESILIENT STORAGE STATS ===\n");
		TRACE_RAW("  Health State        : %s\n", sd_resilient_health_str(sd_resilient_get_health()));
		TRACE_RAW("  Retry OK / Fail     : %u / %u\n", st.retry_ok, st.retry_fail);
		TRACE_RAW("  Reopen OK / Fail    : %u / %u\n", st.reopen_ok, st.reopen_fail);
		TRACE_RAW("  Errors (CRC/TO/Oth) : %u (CRC:%u, TO:%u, Other:%u)\n",
		          st.err_total, st.err_crc, st.err_timeout, st.err_other);
		TRACE_RAW("  Discards (Blk/Samp) : %u blocks, %llu samples\n",
		          st.blocks_discarded, (unsigned long long)st.samples_discarded);
		TRACE_RAW("  Recovery Time       : %llu us\n", (unsigned long long)st.time_in_recovery_us);
		TRACE_RAW("  Gaps in RAM ring    : %u\n", sd_resilient_get_gap_count());
		TRACE_RAW("OK\r\n");
		return;
	}

	if (strcasecmp(subcmd, "f1") == 0) {
		sd_fault_inject_f1();
		TRACE_RAW("OK\r\n");
	} else if (strcasecmp(subcmd, "f2") == 0) {
		if (count < 2 || val == 0) {
			val = 1;
		}
		sd_fault_inject_f2(val);
		TRACE_RAW("OK\r\n");
	} else if (strcasecmp(subcmd, "f3") == 0) {
		sd_fault_inject_f3();
		TRACE_RAW("OK\r\n");
	} else if (strcasecmp(subcmd, "f4") == 0) {
		sd_fault_inject_f4();
		TRACE_RAW("OK\r\n");
	} else if (strcasecmp(subcmd, "f5") == 0) {
		if (count < 2 || val == 0) {
			val = 1;
		}
		sd_fault_inject_f5(val);
		TRACE_RAW("OK\r\n");
	} else if (strcasecmp(subcmd, "clear") == 0) {
		sd_fault_inject_clear();
		TRACE_RAW("OK\r\n");
	} else {
		TRACE_RAW("\nUnknown fault subcommand: %s (usage: fault f1|f2 <n>|f3|f4|f5 <m>|clear|status)\n", subcmd);
	}
}
#endif

/* APPLY handler */
static void handle_apply(void)
{
	if (recorder_is_active()) {
		TRACE_RAW("\nBUSY: Cannot commit to flash while recording is active. Send STOP_RECORDING first.\n");
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "BUSY: Cannot commit to flash while recording is active");
		}
		s_last_cmd_status = -EBUSY;
		return;
	}
	uint16_t cur_bits = (uint16_t)(g_cfg.audio_sampling_width ? g_cfg.audio_sampling_width : 16U);
	uint16_t cur_ch = (uint16_t)(g_cfg.num_audio_channels ? g_cfg.num_audio_channels : 1U);
	rate_check_result_t rc_res;
	if (!console_check_audio_config(g_cfg.sampling_rate, cur_bits, cur_ch,
	                                g_cfg.recording_length,
	                                g_cfg.upper_cutoff_frequency, &rc_res)) {
		TRACE_RAW("\n[BLOCKED] %s. Refused APPLY.\n", rc_res.reason);
		s_last_cmd_status = -ERANGE;
		return;
	}
	if (rc_res.status == RATE_STATUS_WARNING) {
		console_print_audio_warning(g_cfg.sampling_rate, cur_bits, cur_ch,
		                            g_cfg.recording_length, &rc_res);
	}
	g_cfg.save_config_flag = true;
	int save_rc = config_store_save();
	if (save_rc != 0) {
		TRACE("Failed to persist config to flash (%d)", save_rc);
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Flash save failed (%d)", save_rc);
		}
		s_last_cmd_status = save_rc;
		return;
	}
	/* Canonical Maxim notification: */
	TRACE_RAW("\nDevice configuration is applied after reboot.\n");
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
		TRACE_RAW_TEST("\n[ACK] RBT: Reboot already pending. System shutdown in progress.\n");
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

	TRACE_RAW("\nRebooting device.\n");
	TRACE_RAW_TEST("\n[ACK] RBT: Controlled reboot initiated\n");
	if (s_current_out_resp && s_current_out_resp_size > 0) {
		snprintf(s_current_out_resp, s_current_out_resp_size, "[ACK] RBT: Controlled reboot initiated");
	}

	if (recorder_is_active()) {
		TRACE_RAW_TEST("[REBOOT] Stopping active recording safely...\n");
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
			TRACE("RBT: Recorder shutdown timeout (%d); aborting reboot to preserve data", wait_rc);
			TRACE_RAW("\nError: Failed to finalize recording before reboot.\n");
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
		TRACE("RBT: Storage resources still active; aborting reboot");
		TRACE_RAW("[ERROR] Storage resources still active; reboot aborted to prevent corruption.\n");
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Storage resources still active");
		}
		s_last_cmd_status = -EBUSY;
		return;
	}

	int last_err = recorder_get_last_error();
	int last_sd = recorder_get_last_sd_error();
	if (last_err != 0 || last_sd != 0) {
		TRACE("RBT: Proceeding with reboot despite historical storage error (%d)",
		      last_err != 0 ? last_err : last_sd);
	}

	led_indicator_set(LED_IND_OFF);
	if (g_cfg.save_config_flag) {
		main_set_shutdown_stage(SHUTDOWN_STAGE_CONFIG_PERSISTED);
		config_store_save();
		g_cfg.save_config_flag = false;
	}
	main_set_shutdown_stage(SHUTDOWN_STAGE_REBOOT_READY);
	retained_reboot_set_reason(REBOOT_REASON_USER_RBT);
	wildsonic_trace_set_async(false);
	wildsonic_trace_flush();
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
		TRACE_RAW_TEST("\n[ACK] ARBT: Reboot already pending. System shutdown in progress.\n");
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

	TRACE_RAW("\nRebooting device.\n");
	TRACE_RAW_TEST("\n[ACK] ARBT: Applying configuration and initiating orderly shutdown...\n");
	if (s_current_out_resp && s_current_out_resp_size > 0) {
		snprintf(s_current_out_resp, s_current_out_resp_size, "[ACK] ARBT: Controlled reboot initiated");
	}

	/* If recording is active, request orderly stop and wait for clean drain */
	if (recorder_is_active()) {
		TRACE_RAW_TEST("[REBOOT] Stopping active recording safely...\n");
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
			TRACE("ARBT: Recorder shutdown timeout (%d); aborting reboot to preserve storage", wait_rc);
			TRACE_RAW("[ERROR] Storage shutdown timed out (%d); aborting reboot to prevent corruption.\n", wait_rc);
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
		TRACE("ARBT: Storage resources still active; aborting reboot");
		TRACE_RAW("[ERROR] Storage resources still active; reboot aborted to prevent corruption.\n");
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Storage resources still active");
		}
		s_last_cmd_status = -EBUSY;
		return;
	}

	int last_err = recorder_get_last_error();
	int last_sd = recorder_get_last_sd_error();
	if (last_err != 0 || last_sd != 0) {
		TRACE("ARBT: Proceeding with reboot despite historical storage error (%d)",
		      last_err != 0 ? last_err : last_sd);
	}

	/* Save configuration exactly ONCE */
	main_set_shutdown_stage(SHUTDOWN_STAGE_CONFIG_PERSISTED);
	g_cfg.save_config_flag = false;
	int save_rc = config_store_save();
	if (save_rc != 0) {
		main_set_reboot_pending(false);
		main_set_shutdown_stage(SHUTDOWN_STAGE_IDLE);
		TRACE("ARBT: Failed to save configuration to flash (%d)", save_rc);
		TRACE_RAW("[ERROR] Flash save failed (%d). Reboot aborted.\n", save_rc);
		if (s_current_out_resp && s_current_out_resp_size > 0) {
			snprintf(s_current_out_resp, s_current_out_resp_size, "[ERROR] Flash save failed (%d)", save_rc);
		}
		s_last_cmd_status = save_rc;
		return;
	}
	TRACE_RAW_TEST("[ACK] Configuration committed to persistent flash (1 save operation).\n");
	TRACE_RAW_TEST("\nAll buffers flushed and WAV container closed cleanly. Rebooting device.\n");

	main_set_shutdown_stage(SHUTDOWN_STAGE_REBOOT_READY);
	led_indicator_set(LED_IND_OFF);
	retained_reboot_set_reason(REBOOT_REASON_USER_ARBT);
	wildsonic_trace_set_async(false);
	wildsonic_trace_flush();
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
		TRACE_RAW("\nSD card format won't happen during reboot\n");
	} else {
		g_cfg.format_sd_card_flag = true;
		TRACE_RAW("\nSD card format will happen during reboot\n");
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
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	if (strncasecmp(p, "SOURCE", 6) == 0 && (p[6] == ' ' || p[6] == '\0' || p[6] == '=')) {
		const char *arg = p + 6;
		while (*arg == ' ' || *arg == '=') arg++;
		handle_source(arg, strlen(arg));
		return 0;
	}
#endif
#if defined(CONFIG_WILDSONIC_BENCHMARKS)
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
		handle_assert(p_args, args_len);
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
	case CMD_CONFIG_WIZARD:
		main_run_guided_config();
		break;
	case CMD_STATS:
		handle_stats();
		break;
	case CMD_RATECHECK:
		handle_ratecheck(p_args, args_len);
		break;
#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
	case CMD_PSRAM: {
		char arg1[32] = {0}, arg2[32] = {0};
		int matched = sscanf(p_args, "%31s %31s", arg1, arg2);
		main_psram_command(matched + 1, arg1, arg2);
		break;
	}
	case CMD_BATCHLOG:
		handle_batchlog(p_args, args_len);
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
#if defined(CONFIG_WILDSONIC_SD_FAULT_INJECT)
	case CMD_FAULT:
		handle_fault(p_args);
		break;
#endif

	default:
		if (!from_ble) {
			TRACE_RAW("\nUnknown or BAD command : %s\n", p);
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
		rate_check_params_t rcp;
		rate_check_result_t rc_res;
		console_fill_rate_check_params(&rcp, g_cfg.sampling_rate,
		                               (uint16_t)(g_cfg.audio_sampling_width ? g_cfg.audio_sampling_width : 16U),
		                               (uint16_t)(g_cfg.num_audio_channels ? g_cfg.num_audio_channels : 1U),
		                               g_cfg.recording_length,
		                               g_cfg.upper_cutoff_frequency);
		rate_config_check_ex(&rcp, &rc_res);
		if (rc_res.status == RATE_STATUS_IMPOSSIBLE) {
			TRACE("Staged config invalid (%s); reverting to %u Hz/16-bit",
			      rc_res.reason, (uint32_t)CONFIG_AUDIO_SAMPLE_RATE);
			g_cfg.sampling_rate = CONFIG_AUDIO_SAMPLE_RATE;
			g_cfg.audio_sampling_width = 16U;
			g_cfg.num_audio_channels = 1U;
		}
		if (audio_in_rate() != 0U) {
			int rc = audio_in_set_rate(g_cfg.sampling_rate);
			if (rc != 0) {
				TRACE("Failed to set audio rate %u: %d, reverting to %u",
				      g_cfg.sampling_rate, rc, CONFIG_AUDIO_SAMPLE_RATE);
				g_cfg.sampling_rate = CONFIG_AUDIO_SAMPLE_RATE;
			}
		}
	}

	if (settings) {
		settings->sample_rate = g_cfg.sampling_rate;
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

	if (!recorder_is_active() && audio_in_rate() != 0U) {
		main_apply_dsp_settings();
	}
}


