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
 * @param uart_dev Pointer to USART1 device instance.
 * @return 0 on success, negative errno on error.
 */
int ble_comm_init(const struct device *uart_dev);

/**
 * @brief Check if BLE communication subsystem is active.
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
 *
 * @param seconds Duration in seconds (0 for default 300 s, max 600 s).
 */
void ble_comm_open_pairing_window(uint32_t seconds);

/**
 * @brief Lock BLE connectability and put module in low-power idle.
 */
void ble_comm_lock_pairing(void);

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
 */
void ble_comm_suspend(void);

/**
 * @brief Resume BLE UART after waking from Stop 2 low-power sleep.
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
 * @brief Reset BLE communication state (for unit tests).
 */
void ble_comm_reset_test_state(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_COMM_H_ */

