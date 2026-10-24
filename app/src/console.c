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
#include "main.h"
#include "scheduler.h"
#include "rtc_sleep.h"
#include "rtc_calendar.h"
#include "audio_in.h"
#include "recorder.h"
#include "psram_is66wvs.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>

LOG_MODULE_REGISTER(ws_console, CONFIG_LOG_DEFAULT_LEVEL);

/* Maximum string lengths */
#define MAX_SERIAL_LEN 32
#define MAX_DEVNAME_LEN 32
#define MAX_PASSWORD_LEN 32

/* Device configuration model */
struct wildsonic_config {
	char serial_number[MAX_SERIAL_LEN];
	char device_name[MAX_DEVNAME_LEN];
	char device_password[MAX_PASSWORD_LEN];
	uint32_t sampling_rate;
	uint32_t recording_length;
	uint32_t sleep_pause_interval;
	int32_t mic_gain_times_ten;
	uint32_t audio_sampling_width;
	uint32_t num_audio_channels;
	uint32_t lower_cutoff_frequency;
	uint32_t upper_cutoff_frequency;
	uint8_t folder_structure; /* 0 = flat, 1 = daily, 2 = hourly */
	time_t boot_time_utc;
	bool save_config_flag;
	bool format_sd_card_flag;
};

static struct wildsonic_config g_cfg = {
	.serial_number = "WS4760001",
	.device_name = "WildSonic",
	.device_password = "wildsonic123",
	.sampling_rate = CONFIG_AUDIO_SAMPLE_RATE,
	.recording_length = 20,
	.sleep_pause_interval = 60,
	.mic_gain_times_ten = 0,
	.audio_sampling_width = 16,
	.num_audio_channels = 1,
	.lower_cutoff_frequency = 80,
	.upper_cutoff_frequency = 20000,
	.folder_structure = 0,
	.boot_time_utc = 0,
	.save_config_flag = false,
	.format_sd_card_flag = false,
};

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
#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS || CONFIG_AUDIO_INTEGRITY_REPLAY
	/* Test-only engineering commands: strictly restricted to TEST builds */
	CMD_BENCH,
	CMD_DIAGNOSE,
	CMD_COMPARE,
	CMD_STRESS,
	CMD_INTEGRITY,
	CMD_START,
	CMD_STOP,
	CMD_SCHEDULE,
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
	"PSRAM",
	"CONFIG",
#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS || CONFIG_AUDIO_INTEGRITY_REPLAY
	"BENCH",
	"DIAGNOSE",
	"COMPARE",
	"STRESS",
	"INTEGRITY",
	"START",
	"STOP",
	"SCHEDULE",
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
	"Configure PSRAM circular staging buffer (on, off, status, burst)",
	"Run guided interactive configuration wizard",
#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS || CONFIG_AUDIO_INTEGRITY_REPLAY
	"Performance benchmarks (ram, cpu, adc, psram, sd, pipeline, status)",
	"Stage isolation diagnostics (Stage A..E)",
	"Dual-cycle Raw vs Filtered comparison",
	"Multi-cycle stress test",
	"PCM to SD CRC readback integrity test",
	"Start manual recording session (engineering test)",
	"Stop active recording session (engineering test)",
	"Scheduler diagnosis and manual execution (engineering test)",
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
	for (size_t i = 0; i < (size_t)CMD_NUM_CMDS; i++) {
		printk(" %s \n", console_cmd_lookup_table[i]);
	}
	printk("\n");
}

/* Print HELP: command : description */
static void display_console_help(void)
{
	printk("\nList of commands\n");
	for (size_t i = 0; i < (size_t)CMD_NUM_CMDS; i++) {
		printk(" %s : %s\n", console_cmd_lookup_table[i], console_cmd_description_table[i]);
	}
	printk("\n");
}

/* Explicit deferred hardware notice */
static void report_deferred_hardware(const char *cmd_name)
{
	printk("\n[DEFERRED] Command '%s' requires hardware not populated on STM32L476RG.\n", cmd_name);
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

/* SAMF handler */
static void handle_samf(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.sampling_rate);
		return;
	}

	uint32_t val = (uint32_t)strtoul(p_args, NULL, 10);
	if (val == 16) val = 16000;
	else if (val == 32) val = 32000;
	else if (val == 44) val = 44100;
	else if (val == 48) val = 48000;
	else if (val == 96) val = 96000;
	else if (val == 192) val = 192000;

	uint32_t act = 0, div = 0;
	int err = 0;
	if (!audio_in_is_rate_supported(val, &act, &div, &err)) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %u\n", g_cfg.sampling_rate);
		return;
	}

	uint32_t old_rate = g_cfg.sampling_rate;
	g_cfg.sampling_rate = val;
	settings->sample_rate = val;
	audio_in_set_rate(val);
	main_apply_dsp_settings();

	printk("\nValue is set to %u\n", val);
	if (g_cfg.upper_cutoff_frequency == (old_rate / 2)) {
		g_cfg.upper_cutoff_frequency = val / 2;
		settings->lpf_cutoff_hz = val / 2;
		main_apply_dsp_settings();
		printk("\nUpper cutoff frequency adjusted to %u\n", g_cfg.upper_cutoff_frequency);
	}
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* BTPS handler */
static void handle_btps(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.audio_sampling_width);
		return;
	}
	int val = atoi(p_args);
	if (val != 16) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %u\n", g_cfg.audio_sampling_width);
		return;
	}
	g_cfg.audio_sampling_width = 16;
	printk("\nValue is set to 16\n");
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* NUMCH handler */
static void handle_numch(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.num_audio_channels);
		return;
	}
	int val = atoi(p_args);
	if (val != 1) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %u\n", g_cfg.num_audio_channels);
		return;
	}
	g_cfg.num_audio_channels = 1;
	printk("\nValue is set to 1\n");
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* AUDIO parameters summary / packed setter */
static void handle_audio(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
		printk("Sampling frequency set to %u\n", g_cfg.sampling_rate);
		printk("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
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
		if (new_numch == 1 && new_btps == 16 &&
		    audio_in_is_rate_supported(new_samf, &act, &div, &err)) {
			uint32_t old_samf = g_cfg.sampling_rate;
			g_cfg.num_audio_channels = new_numch;
			g_cfg.sampling_rate = new_samf;
			g_cfg.audio_sampling_width = new_btps;
			struct app_settings *settings = main_get_settings();
			settings->sample_rate = new_samf;
			audio_in_set_rate(new_samf);
			main_apply_dsp_settings();

			if (old_samf != new_samf && g_cfg.upper_cutoff_frequency == (old_samf / 2)) {
				g_cfg.upper_cutoff_frequency = new_samf / 2;
				settings->lpf_cutoff_hz = new_samf / 2;
				main_apply_dsp_settings();
				printk("\nUpper cutoff frequency adjusted to %u\n", g_cfg.upper_cutoff_frequency);
			}

			printk("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
			printk("Sampling frequency set to %u\n", g_cfg.sampling_rate);
			printk("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
			printk("\nSend APPLY command and reboot for changes to take effect\n");
			return;
		}
	}

	printk("\nInvalid values\n");
	printk("\nNumber of channels set to %u\n", g_cfg.num_audio_channels);
	printk("Sampling frequency set to %u\n", g_cfg.sampling_rate);
	printk("Bits per sample set to %u\n", g_cfg.audio_sampling_width);
}

/* RECT handler */
static void handle_rect(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.recording_length);
		return;
	}

	long val = strtol(p_args, NULL, 10);
	if (val <= 0 || val > 86400) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %u\n", g_cfg.recording_length);
		return;
	}

	g_cfg.recording_length = (uint32_t)val;
	settings->record_seconds = (uint32_t)val;

	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	scfg.record_length = (uint32_t)val;
	scheduler_set_config(&scfg);
	main_scheduler_resume();

	printk("\nValue is set to %u\n", g_cfg.recording_length);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* SLEEP handler */
static void handle_sleep(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
		return;
	}

	if (p_args[0] == '-') {
		printk("\nInvalid value: negative values are not allowed\n");
		printk("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
		return;
	}

	long val = strtol(p_args, NULL, 10);
	if (val < 0 || val > 86400) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
		return;
	}

	g_cfg.sleep_pause_interval = (uint32_t)val;
	settings->sleep_seconds = (uint32_t)val;

	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	scfg.sleep_interval = (uint32_t)val;
	scheduler_set_config(&scfg);
	main_scheduler_resume();

	printk("\nValue is set to %u\n", g_cfg.sleep_pause_interval);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
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
		return;
	}

	struct rtc_calendar_time new_cal;
	int parse_res = rtc_calendar_parse(p_args, &new_cal);
	if (parse_res == 0) {
		rtc_calendar_set(&new_cal);
		time_t new_utc = scheduler_calendar_to_epoch(&new_cal);
		g_cfg.boot_time_utc = new_utc;

		/* Check optional timezone token */
		char tz_token[16] = {0};
		int tokens = sscanf(p_args, "%*s %*s %15s", tz_token);
		if (tokens == 1 && (tz_token[0] == '+' || tz_token[0] == '-' || isdigit((unsigned char)tz_token[0]))) {
			int32_t tz_off = 0;
			if (scheduler_parse_timezone(tz_token, &tz_off) == 0) {
				scfg.time_zone_offset = tz_off;
				scheduler_set_config(&scfg);
			}
		}

		console_printf_time("\nBoot time UTC set to ", new_utc, 0, "\n");
		console_printf_time("\nBoot time Local set to ", new_utc, scfg.time_zone_offset, "\n");
		console_printf_time("\nCurrent time UTC time is ", new_utc, 0, "\n");
		console_printf_time("\nCurrent time Local is ", new_utc, scfg.time_zone_offset, "\n");
		main_scheduler_resume();
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		return;
	}

	printk("\nInvalid date time %s\nUTC Time Format should be YYYY-MM-DD HH:MM:SS\n", p_args);
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
		return;
	}

	int32_t offset = 0;
	if (scheduler_parse_timezone(p_args, &offset) == 0) {
		scfg.time_zone_offset = offset;
		scheduler_set_config(&scfg);
		main_scheduler_resume();
		printk("\nTime zone set to %s\n", p_args);
		printk("\nSend APPLY command and reboot for changes to take effect\n");
		return;
	}

	printk("\nInvalid time zone argument %s\n", p_args);
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
		return;
	}

	printk("\nInvalid begin end date %s\n", p_args);
	printk("\nBegin end date set to %04u/%02u/%02u-%04u/%02u/%02u\n",
	       scfg.start_date.year, scfg.start_date.month, scfg.start_date.day,
	       scfg.end_date.year, scfg.end_date.month, scfg.end_date.day);
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
		return;
	}

	printk("\nInvalid begin end time %s\n", p_args);
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
		return;
	}

	printk("\nInvalid days of week %s\n", p_args);
}

/* GAIN handler */
static void handle_gain(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nAudio gain set to 0.1*%d\n", g_cfg.mic_gain_times_ten);
		return;
	}

	int sign = 1;
	size_t off = 0;
	if (p_args[0] == 'N' || p_args[0] == '-') {
		sign = -1;
		off = 1;
	}
	float val = (float)strtod(&p_args[off], NULL);
	int gain_t10 = (int)roundf(sign * val * 10.0f);

	if (gain_t10 < -400 || gain_t10 > 400) {
		printk("\nInvalid audio gain 0.1*%d\n", gain_t10);
		printk("\nAudio gain set to 0.1*%d\n", g_cfg.mic_gain_times_ten);
		return;
	}

	g_cfg.mic_gain_times_ten = gain_t10;
	settings->digital_gain = powf(10.0f, ((float)gain_t10 / 10.0f) / 20.0f);
	main_apply_dsp_settings();

	printk("\nAudio gain set to 0.1*%d\n", gain_t10);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* LCUTOFF (HPF) handler */
static void handle_lcutoff(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.lower_cutoff_frequency);
		return;
	}

	int hz = atoi(p_args);
	if (hz != 0 && (hz < 10 || hz > 2000)) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %u\n", g_cfg.lower_cutoff_frequency);
		return;
	}

	g_cfg.lower_cutoff_frequency = (uint32_t)hz;
	settings->hpf_cutoff_hz = (uint32_t)hz;
	settings->hpf_enabled = (hz > 0);
	main_apply_dsp_settings();

	printk("\nValue is set to %u\n", (uint32_t)hz);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* UCUTOFF (LPF) handler */
static void handle_ucutoff(const char *p_args, size_t args_length)
{
	struct app_settings *settings = main_get_settings();

	if (args_length == 0) {
		printk("\nValue is set to %u\n", g_cfg.upper_cutoff_frequency);
		return;
	}

	int hz = atoi(p_args);
	if (hz != 0 && (hz < 1000 || (uint32_t)hz > g_cfg.sampling_rate / 2)) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %u\n", g_cfg.upper_cutoff_frequency);
		return;
	}

	g_cfg.upper_cutoff_frequency = (uint32_t)hz;
	settings->lpf_cutoff_hz = (uint32_t)hz;
	settings->lpf_enabled = (hz > 0);
	main_apply_dsp_settings();

	printk("\nValue is set to %u\n", (uint32_t)hz);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* FLDR handler */
static void handle_fldr(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		const char *mode_str = (g_cfg.folder_structure == 0) ? "flat" :
				       ((g_cfg.folder_structure == 1) ? "daily" : "hourly");
		printk("\nValue is set to %s\n", mode_str);
		return;
	}

	if (strcasecmp(p_args, "flat") == 0) {
		g_cfg.folder_structure = 0;
	} else if (strcasecmp(p_args, "daily") == 0 || strcasecmp(p_args, "day") == 0) {
		g_cfg.folder_structure = 1;
	} else if (strcasecmp(p_args, "hourly") == 0 || strcasecmp(p_args, "hour") == 0) {
		g_cfg.folder_structure = 2;
	} else {
		printk("\nInvalid value %s\n", p_args);
		return;
	}

	printk("\nValue is updated to %s\n", p_args);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
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
	printk("\nWildSonic STM32L476RG Firmware v1.2.0 (Zephyr RTOS v4.5.0-rc2, Built %s %s)\n",
	       __DATE__, __TIME__);
}

/* SN handler */
static void handle_sn(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nValue is set to %s\n", g_cfg.serial_number);
		return;
	}

	if (args_length >= sizeof(g_cfg.serial_number)) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %s\n", g_cfg.serial_number);
		return;
	}

	for (size_t i = 0; i < args_length; i++) {
		if (!isalnum((unsigned char)p_args[i])) {
			printk("\nInvalid value %s\n", p_args);
			printk("\nValue is set to %s\n", g_cfg.serial_number);
			return;
		}
	}

	strncpy(g_cfg.serial_number, p_args, sizeof(g_cfg.serial_number) - 1);
	g_cfg.serial_number[sizeof(g_cfg.serial_number) - 1] = '\0';
	printk("\nValue is set to %s\n", g_cfg.serial_number);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* PW handler */
static void handle_pw(const char *p_args, size_t args_length)
{
	if (args_length == 0) {
		printk("\nValue is set to %s\n", g_cfg.device_password);
		return;
	}

	if (args_length >= sizeof(g_cfg.device_password)) {
		printk("\nInvalid value %s\n", p_args);
		printk("\nValue is set to %s\n", g_cfg.device_password);
		return;
	}

	strncpy(g_cfg.device_password, p_args, sizeof(g_cfg.device_password) - 1);
	g_cfg.device_password[sizeof(g_cfg.device_password) - 1] = '\0';
	printk("\nValue is set to %s\n", g_cfg.device_password);
	printk("\nSend APPLY command and reboot for changes to take effect\n");
}

/* ASSERT handler */
static void handle_assert(void)
{
	uint32_t csr = RCC->CSR;
	if (csr & (RCC_CSR_SFTRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_LPWRRSTF)) {
		printk("Assert / Reset Reason: CSR=0x%08X (SFTRST=%d, IWDG=%d, WWDG=%d, LPWR=%d)\n",
		       (unsigned int)csr,
		       !!(csr & RCC_CSR_SFTRSTF),
		       !!(csr & RCC_CSR_IWDGRSTF),
		       !!(csr & RCC_CSR_WWDGRSTF),
		       !!(csr & RCC_CSR_LPWRRSTF));
	} else {
		printk("There is no assert information \n");
	}
}

/* DEFAULT handler */
static void handle_default(void)
{
	main_reset_to_defaults();
	g_cfg.sampling_rate = 192000;
	g_cfg.recording_length = 20;
	g_cfg.sleep_pause_interval = 60;
	g_cfg.mic_gain_times_ten = 0;
	g_cfg.audio_sampling_width = 16;
	g_cfg.num_audio_channels = 1;
	g_cfg.lower_cutoff_frequency = 80;
	g_cfg.upper_cutoff_frequency = 20000;
	g_cfg.folder_structure = 0;

	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	scfg.start_date.year = 0;
	scfg.end_date.year = 0;
	scheduler_reset_tod_ranges(scfg.tod_ranges, SCHEDULER_MAX_TIME_OF_DAY_RANGES);
	scfg.day_of_week_mask = SCHEDULER_FULL_WEEK_MASK;
	scheduler_set_config(&scfg);

	printk("\nDefault device configurations are set\n");
}

/* APPLY handler */
static void handle_apply(void)
{
	g_cfg.save_config_flag = true;
	printk("\n[DEFERRED] Flash persistence not implemented; active settings are held in RAM.\n");
	main_scheduler_resume();
}

/* RBT handler */
static void handle_rbt(void)
{
	printk("\nRebooting device.\n");
	while (log_process()) {
		k_sleep(K_MSEC(2));
	}
#if defined(CONFIG_REBOOT)
	sys_reboot(SYS_REBOOT_COLD);
#endif
}

/* ARBT handler */
static void handle_arbt(void)
{
	g_cfg.save_config_flag = true;
	printk("\n[DEFERRED] Flash persistence not implemented; rebooting will reload defaults.\n");
	handle_rbt();
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

/* Initialize Console */
void console_init(void)
{
	struct rtc_calendar_time cur_cal;
	if (rtc_calendar_get(&cur_cal) == 0 && rtc_calendar_is_valid()) {
		g_cfg.boot_time_utc = scheduler_calendar_to_epoch(&cur_cal);
	}
}

/* Process incoming command line */
void console_process_line(char *cmd_line)
{
	if (!cmd_line) return;

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
		return;
	}

	size_t arg_offset = 0;
	enum console_cmd_id cmd_id = console_cmd_lookup_func(p, len, &arg_offset);

	const char *p_args = &p[arg_offset];
	size_t args_len = (len >= arg_offset) ? (len - arg_offset) : 0;

	switch (cmd_id) {
	case CMD_HELP:
		display_console_help();
		break;
	case CMD_SERIAL_NUMBER:
		handle_sn(p_args, args_len);
		break;
	case CMD_DEVICE_NAME:
		report_deferred_hardware("DNAME");
		break;
	case CMD_PASSWORD:
		handle_pw(p_args, args_len);
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
		report_deferred_hardware("BLEPAIR");
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
		report_deferred_hardware("LOC");
		break;
	case CMD_SUN:
		report_deferred_hardware("SUN");
		break;
	case CMD_SUN_RECORDING_MODE:
		report_deferred_hardware("SUNMODE");
		break;
	case CMD_SUNRISE_INTERVAL_BEFORE:
		report_deferred_hardware("SUNRISEBEF");
		break;
	case CMD_SUNRISE_INTERVAL_AFTER:
		report_deferred_hardware("SUNRISEAFT");
		break;
	case CMD_SUNSET_INTERVAL_BEFORE:
		report_deferred_hardware("SUNSETBEF");
		break;
	case CMD_SUNSET_INTERVAL_AFTER:
		report_deferred_hardware("SUNSETAFT");
		break;
	case CMD_SUNRISE_SUNSET_ROUNDING:
		report_deferred_hardware("SUNROUND");
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
	case CMD_PSRAM: {
		char arg1[32] = {0}, arg2[32] = {0};
		int matched = sscanf(p_args, "%31s %31s", arg1, arg2);
		main_psram_command(matched + 1, arg1, arg2);
		break;
	}
	case CMD_CONFIG_WIZARD:
		main_run_guided_config();
		break;

#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS || CONFIG_AUDIO_INTEGRITY_REPLAY
	case CMD_BENCH: {
#if CONFIG_AUDIO_BENCHMARK
		char arg1[32] = {0}, arg2[32] = {0}, arg3[32] = {0};
		int matched = sscanf(p_args, "%31s %31s %31s", arg1, arg2, arg3);
		main_bench_command(matched + 1, arg1, arg2, arg3);
#else
		printk("\nBenchmark module not enabled in current build.\n");
#endif
		break;
	}
	case CMD_DIAGNOSE: {
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
		char arg1[32] = {0};
		int matched = sscanf(p_args, "%31s", arg1);
		main_diagnose_command(matched + 1, arg1);
#else
		printk("\nDiagnostics module not enabled in current build.\n");
#endif
		break;
	}
	case CMD_COMPARE: {
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
		main_start_compare();
#else
		printk("\nCompare mode not enabled in current build.\n");
#endif
		break;
	}
	case CMD_STRESS: {
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
		main_start_stress();
#else
		printk("\nStress mode not enabled in current build.\n");
#endif
		break;
	}
	case CMD_INTEGRITY: {
#if CONFIG_AUDIO_INTEGRITY_REPLAY
		char arg1[32] = {0}, arg2[32] = {0};
		int matched = sscanf(p_args, "%31s %31s", arg1, arg2);
		main_integrity_command(matched + 1, arg1, arg2);
#else
		printk("\nIntegrity replay not enabled in current build.\n");
#endif
		break;
	}
	case CMD_START:
		main_start_recording();
		break;
	case CMD_STOP:
		main_stop_recording();
		break;
	case CMD_SCHEDULE: {
		char arg1[32] = {0};
		sscanf(p_args, "%31s", arg1);
		if (strcmp(arg1, "run") == 0) {
			main_run_scheduled_recording();
		} else {
			struct schedule_config scfg;
			scheduler_get_config(&scfg);
			struct rtc_calendar_time cur_cal;
			rtc_calendar_get(&cur_cal);
			time_t now_utc = scheduler_calendar_to_epoch(&cur_cal);
			struct schedule_interval next_inv;
			int res = scheduler_calculate_next(&scfg, now_utc, 0, &next_inv);
			bool active = (res >= 0);
			printk("\nScheduler Status: %s\n", active ? "ACTIVE" : "IDLE/UNSCHEDULED");
			if (active) {
				char tbuf[32];
				struct rtc_calendar_time next_cal;
				scheduler_epoch_to_calendar(next_inv.start_time_utc, &next_cal);
				rtc_calendar_format(&next_cal, tbuf, sizeof(tbuf));
				printk("  Next Session UTC : %s (wait: %lld s, duration: %u s)\n",
				       tbuf, (long long)next_inv.wait_seconds, next_inv.duration_seconds);
			}
		}
		break;
	}
#endif

	default:
		printk("\nUnknown or BAD command : %s\n", p);
		display_console_commands();
		break;
	}
}

