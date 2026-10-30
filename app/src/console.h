/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_UART_CMD_LEN 128
#define MAX_SERIAL_LEN 32
#define MAX_DEVNAME_LEN 32
#define MAX_PASSWORD_LEN 32

/* Device configuration model matching reference Maxim MAX78000 */
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

/**
 * @brief Initialize console state and command structures.
 */
void console_init(void);

/**
 * @brief Process an entire command line entered by user or test harness.
 * 
 * @param cmd_line Null-terminated input string.
 */
void console_process_line(char *cmd_line);

/**
 * @brief Get pointer to staged configuration in RAM.
 */
struct wildsonic_config *console_get_config(void);

/**
 * @brief Synchronize staged RAM configuration with audio pipeline and scheduler.
 */
void console_sync_staged_to_active(void);

struct wildsonic_assert_record {
	bool asserted;
	const char *file;
	int line;
	const char *expr;
};

void wildsonic_record_assert(const char *file, int line, const char *expr);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H_ */
