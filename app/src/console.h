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
 * @brief Process an entire command line from a specific source (Console or BLE).
 *
 * @param cmd_line Null-terminated input string.
 * @param from_ble True if invoked from BLE transport, false if from console UART.
 * @param out_resp Optional output buffer for response back to source.
 * @param out_resp_size Size of out_resp buffer.
 * @return 0 on success, negative errno on error.
 */
int console_process_line_source(char *cmd_line, bool from_ble, char *out_resp, size_t out_resp_size);

/**
 * @brief Get pointer to staged configuration in RAM.
 */
struct wildsonic_config *console_get_config(void);

/**
 * @brief Synchronize staged RAM configuration with audio pipeline and scheduler.
 */
void console_sync_staged_to_active(void);

struct device;

/**
 * @brief Initialize asynchronous interrupt-driven console UART RX with ring buffer.
 *
 * @param dev UART device instance.
 * @return 0 on success, negative errno on error.
 */
int console_async_init(const struct device *dev);

/**
 * @brief Check whether asynchronous interrupt-driven console RX is active.
 *
 * @return true if active, false otherwise.
 */
bool console_async_is_active(void);

/**
 * @brief Get a character from the async console RX ring buffer (non-blocking).
 *
 * @param c Output character pointer.
 * @return 0 if character retrieved, -EAGAIN if ring buffer is empty.
 */
int console_async_get_char(unsigned char *c);

/**
 * @brief Wait for a character from the async console RX ring buffer with timeout in milliseconds.
 *
 * @param c Output character pointer.
 * @param timeout_ms Maximum wait time in milliseconds (-1 for forever, 0 for no wait).
 * @return 0 if character retrieved, -ETIMEDOUT if timed out.
 */
int console_async_wait_char(unsigned char *c, int32_t timeout_ms);

/**
 * @brief Suspend console UART RX interrupts (e.g. prior to entering Stop 2 sleep).
 */
void console_async_suspend(void);

/**
 * @brief Resume console UART RX interrupts (e.g. after waking from Stop 2 sleep).
 */
void console_async_resume(void);

/**
 * @brief Get total number of console UART RX bytes dropped due to ring buffer overflow.
 */
uint32_t console_get_rx_drop_count(void);

/**
 * @brief Reset console UART RX overflow counter.
 */
void console_reset_rx_drop_count(void);

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
