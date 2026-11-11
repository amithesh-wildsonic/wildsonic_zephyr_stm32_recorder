/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void watchdog_init(void);
void watchdog_feed(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H_ */
