/* SPDX-License-Identifier: Apache-2.0 */
#include "system_assert.h"
#include "wildsonic_trace.h"
#include <stdio.h>
#include <string.h>

#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

void system_assert_record_id(uint32_t id, const char *file, uint32_t line, const char *expr)
{
	const char *slash = file ? strrchr(file, '/') : NULL;
	const char *base = slash ? (slash + 1) : (file ? file : "unknown");

	wildsonic_trace_set_async(false);
	wildsonic_raw_emit("\nASSERT 0x%08X %s #%u: %s\n", (unsigned int)id, base, (unsigned int)line, expr ? expr : "");
	wildsonic_trace_flush();
	retained_reboot_set_assert(id, base, line);

#if defined(CONFIG_REBOOT)
	sys_reboot(SYS_REBOOT_COLD);
#else
	while (1) {
		k_cpu_idle();
	}
#endif
}

#else

void system_assert_record_id(uint32_t id, const char *file, uint32_t line, const char *expr)
{
	const char *slash = file ? strrchr(file, '/') : NULL;
	const char *base = slash ? (slash + 1) : (file ? file : "unknown");
	printf("\nASSERT 0x%08X %s #%u: %s\n", (unsigned int)id, base, (unsigned int)line, expr ? expr : "");
	retained_reboot_set_assert(id, base, line);
}

#endif
