/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RETAINED_REBOOT_H_
#define RETAINED_REBOOT_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	REBOOT_REASON_NONE = 0,
	REBOOT_REASON_USER_RBT,
	REBOOT_REASON_USER_ARBT,
	REBOOT_REASON_ASSERT,
	REBOOT_REASON_WDT,
	REBOOT_REASON_SLEEP_FAIL,
	REBOOT_REASON_STORAGE_FAIL,
	REBOOT_REASON_PIN_RESET,
	REBOOT_REASON_POWER_ON_RESET,
	REBOOT_REASON_SOFTWARE_RESET,
	REBOOT_REASON_UNKNOWN
} reboot_reason_t;

struct retained_reboot_info {
	reboot_reason_t reason;
	uint32_t assert_id;
	uint32_t assert_line;
	char assert_file[32];
};

void retained_reboot_init(void);
void retained_reboot_set_reason(reboot_reason_t reason);
void retained_reboot_set_assert(uint32_t id, const char *file, uint32_t line);
reboot_reason_t retained_reboot_get(struct retained_reboot_info *out_info);
const char *retained_reboot_reason_str(reboot_reason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* RETAINED_REBOOT_H_ */
