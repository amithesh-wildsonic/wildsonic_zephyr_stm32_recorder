#ifndef _BLE_COMM_HPP
#define _BLE_COMM_HPP

#include <stdint.h>

/**
 * @brief Initialize BLE communication
 */
void ble_init();

/**
 * @brief Update BLE status
 * If BLE module is awake and disconnected, send a sleep command to put it to
 * sleep
 */
void update_ble_status();
void process_ble_pending_commands();
#endif // _BLE_COMM_HPP