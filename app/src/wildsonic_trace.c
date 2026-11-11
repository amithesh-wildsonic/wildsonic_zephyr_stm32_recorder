/* SPDX-License-Identifier: Apache-2.0 */
#include "wildsonic_trace.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

K_MUTEX_DEFINE(s_wildsonic_trace_mutex);

void wildsonic_trace_init(void)
{
	/* Mutex statically initialized */
}

void wildsonic_trace_emit(const char *file, int line, const char *fmt, ...)
{
	if (!file || !fmt) {
		return;
	}

	const char *slash = strrchr(file, '/');
	const char *base = slash ? slash + 1 : file;

	if (k_is_in_isr()) {
		printk("C:0 [F:(%s), L:%d] ", base, line);
		va_list ap;
		va_start(ap, fmt);
		vprintk(fmt, ap);
		va_end(ap);
		size_t len = strlen(fmt);
		if (len == 0 || fmt[len - 1] != '\n') {
			printk("\n");
		}
		return;
	}

	k_mutex_lock(&s_wildsonic_trace_mutex, K_FOREVER);
	printk("C:0 [F:(%s), L:%d] ", base, line);

	va_list ap;
	va_start(ap, fmt);
	vprintk(fmt, ap);
	va_end(ap);

	size_t len = strlen(fmt);
	if (len == 0 || fmt[len - 1] != '\n') {
		printk("\n");
	}

	k_mutex_unlock(&s_wildsonic_trace_mutex);
}

void wildsonic_raw_emit(const char *fmt, ...)
{
	if (!fmt) {
		return;
	}

	if (k_is_in_isr()) {
		va_list ap;
		va_start(ap, fmt);
		vprintk(fmt, ap);
		va_end(ap);
		return;
	}

	k_mutex_lock(&s_wildsonic_trace_mutex, K_FOREVER);
	va_list ap;
	va_start(ap, fmt);
	vprintk(fmt, ap);
	va_end(ap);
	k_mutex_unlock(&s_wildsonic_trace_mutex);
}

#else

void wildsonic_trace_init(void) {}

void wildsonic_trace_emit(const char *file, int line, const char *fmt, ...)
{
	const char *slash = strrchr(file, '/');
	const char *base = slash ? slash + 1 : file;
	printf("C:0 [F:(%s), L:%d] ", base, line);
	va_list ap;
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	size_t len = strlen(fmt);
	if (len == 0 || fmt[len - 1] != '\n') {
		printf("\n");
	}
}

void wildsonic_raw_emit(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}

#endif
