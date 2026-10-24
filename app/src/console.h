/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_UART_CMD_LEN 128

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

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H_ */

