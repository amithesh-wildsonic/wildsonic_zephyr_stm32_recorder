/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BLE_COMM_H_
#define BLE_COMM_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_UART_CMD_MAX_LEN 128
#define BLE_PAIR_WINDOW_DEFAULT_SECONDS 300
#define BLE_PAIR_WINDOW_MAX_SECONDS 600

/**
 * @brief Initialize BLE communication on USART1 (PA9/PA10).
 *
 * Configures USART1, sets s_ble_uart_ready = true, probes supervisor with bare AT,
 * enforces AT+WS=0 (normal transparent mode), audits/configures AT+SLPM=1,
 * sets advertised name/type/pass, locks connectability (AT+ADTY2), and enters AT+SLEEP.
 *
 * @param uart_dev Pointer to USART1 device instance.
 * @return 0 on success (or probe timeout proceeding without BLE), negative errno on hard error.
 */
int ble_comm_init(const struct device *uart_dev);

/**
 * @brief Check if BLE UART hardware driver is ready.
 */
bool ble_comm_is_uart_ready(void);

/**
 * @brief Check if BLE supervisor module is online and responsive.
 */
bool ble_comm_is_supervisor_online(void);

/**
 * @brief Check if BLE communication subsystem is active (alias for supervisor online).
 */
bool ble_comm_is_active(void);

/**
 * @brief Check if a central device is currently connected via BLE.
 */
bool ble_comm_is_connected(void);

/**
 * @brief Check if BLE module is awake.
 */
bool ble_comm_is_module_awake(void);

/**
 * @brief Check if pairing / advertising connectability window is active.
 */
bool ble_comm_is_pairing_window_active(void);

/**
 * @brief Get remaining pairing window time in seconds.
 */
uint32_t ble_comm_get_pairing_window_remaining(void);

/**
 * @brief Open pairing / connectability window for specified duration.
 * Transmits AT+ADTY0 and validates OK+Set:0 acknowledgement.
 *
 * @param seconds Duration in seconds (0 for default 300 s, max 600 s).
 */
void ble_comm_open_pairing_window(uint32_t seconds);

/**
 * @brief Lock BLE connectability.
 * If disconnected: transmits AT+ADTY2, validates OK+Set:2, and commands AT+SLEEP.
 * If connected: defers lock until central disconnects.
 */
void ble_comm_lock_pairing(void);

/**
 * @brief Build advertised name matching original WildSonic format WS:<DNAME>-<SN>.
 *
 * Enforces fallback 'XXX' for empty/invalid DNAME (max 6 alphanumeric) and
 * '00' for empty/invalid SN (max 3 alphanumeric). Total length strictly <= 12.
 *
 * @param dname Device name.
 * @param sn Serial number.
 * @param out Output buffer.
 * @param max_len Buffer capacity.
 * @return 0 on success, -EINVAL if constructed name exceeds 12 chars or parameters invalid.
 */
int ble_comm_build_advertised_name(const char *dname, const char *sn, char *out, size_t max_len);

/**
 * @brief Notify BLE module that device name or serial number changed.
 * Updates advertised name immediately if disconnected, or defers until central disconnects.
 */
void ble_comm_notify_name_changed(void);

/**
 * @brief Apply configured advertised name to BLE supervisor immediately.
 */
void ble_comm_apply_advertised_name(void);

/**
 * @brief Calculate 32-bit persistent configuration fingerprint for BLE settings.
 *
 * Hashes transparent mode, sleep policy, connectability, bond security, PIN, and advertised name.
 * Excludes audio parameters.
 *
 * @param adv_name Advertised name string (e.g. "WS:XXX-00").
 * @param pin Pairing PIN string (e.g. "123456").
 * @param ws_mode Transparent mode (0).
 * @param slpm Sleep mode (1).
 * @param adty Advertising type (0).
 * @param type Bond security type (3).
 * @return 32-bit configuration fingerprint.
 */
uint32_t ble_comm_calc_fingerprint(const char *adv_name, const char *pin, uint8_t ws_mode, uint8_t slpm, uint8_t adty, uint8_t type);

/**
 * @brief Synchronous command transaction handler with bounded timeout.
 * Transmits exact command with CRLF and validates expected acknowledgement.
 *
 * @param cmd AT command string (with or without CRLF).
 * @param expected_ack Expected ACK string or prefix (e.g. "OK", "OK+Set:0", "OK+SLPM:").
 * @param resp_out Optional output buffer for received response line.
 * @param resp_max Maximum size of resp_out buffer.
 * @param timeout_ms Maximum time to wait for response.
 * @return 0 on match, -ETIMEDOUT on timeout, -EBUSY if central is connected or mutex locked.
 */
int ble_comm_send_cmd_expect(const char *cmd, const char *expected_ack, char *resp_out, size_t resp_max, uint32_t timeout_ms);

/**
 * @brief Query current BLE supervisor sleep policy (AT+SLPM?).
 *
 * @param mode Output mode pointer (1=light sleep, 2=UART blocked, 3=radio off).
 * @return 0 on success, negative errno on error.
 */
int ble_comm_query_sleep_policy(uint8_t *mode);

/**
 * @brief Configure BLE supervisor sleep policy (AT+SLPM=mode).
 *
 * @param mode Target policy (Mode 1 required for STM32 UART wake).
 * @return 0 on success, negative errno on error.
 */
int ble_comm_set_sleep_policy(uint8_t mode);

/**
 * @brief Query advertising connectability type (AT+ADTY?).
 *
 * @param adty Output pointer (0=connectable, 2=non-connectable scannable).
 * @return 0 on success, negative errno on error.
 */
int ble_comm_query_adv_type(uint8_t *adty);

/**
 * @brief Transmit raw bytes to BLE module over USART1.
 *
 * @param data Byte buffer.
 * @param len Number of bytes.
 * @return Number of bytes transmitted or negative errno.
 */
int ble_comm_tx(const uint8_t *data, size_t len);

/**
 * @brief Format and transmit string to BLE module over USART1.
 *
 * @param format Printf format string.
 * @return Number of characters transmitted or negative errno.
 */
int ble_comm_printf(const char *format, ...);

/**
 * @brief Suspend BLE UART before entering Stop 2 low-power sleep.
 * Sends AT+SLEEP under Mode 1 policy, drains TX, and isolates USART1.
 */
void ble_comm_suspend(void);

/**
 * @brief Resume BLE UART after waking from Stop 2 low-power sleep.
 * Restores clocks, sends AT ping, and validates OK response.
 */
void ble_comm_resume(void);

/**
 * @brief Transmit standard device information block (matching Mobile Companion app).
 */
void ble_comm_send_dev_info(void);

/**
 * @brief Transmit standard status information block (matching Mobile Companion app).
 */
void ble_comm_send_status_info(void);

/**
 * @brief Feed incoming raw bytes into BLE RX parser (used for host unit testing).
 *
 * @param data Byte buffer.
 * @param len Number of bytes.
 * @return Number of bytes accepted.
 */
int ble_comm_feed_rx_bytes(const uint8_t *data, size_t len);

/**
 * @brief Drain transmitted bytes from TX ring buffer (used for host unit testing).
 *
 * @param buf Output buffer.
 * @param max_len Maximum bytes to drain.
 * @return Number of bytes drained.
 */
int ble_comm_drain_tx_bytes(uint8_t *buf, size_t max_len);

/**
 * @brief Enable or disable test capture of TX bytes into s_ble_tx_rb.
 *
 * @param enabled True to capture in ring buffer, false to bypass (production default).
 */
void ble_comm_set_test_capture(bool enabled);

/**
 * @brief Reset BLE communication state (for unit tests).
 */
void ble_comm_reset_test_state(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_COMM_H_ */
