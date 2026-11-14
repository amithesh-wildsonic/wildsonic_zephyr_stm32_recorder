/* SPDX-License-Identifier: Apache-2.0 */
#include "wildsonic_trace.h"
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if defined(__ZEPHYR__)
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#define TRACE_LINE_MAX      200U
#define TRACE_RAW_MAX       640U
#define TRACE_RB_BYTES      1024U
#define TRACE_PUMP_STACK    1024U
#define TRACE_PUMP_PRIORITY 12

static const struct device *const s_console_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

K_MUTEX_DEFINE(s_trace_tx_mutex);
RING_BUF_DECLARE(s_trace_rb, TRACE_RB_BYTES);
static struct k_spinlock s_rb_lock;
static K_SEM_DEFINE(s_trace_sem, 0, K_SEM_MAX_LIMIT);

static K_THREAD_STACK_DEFINE(s_trace_pump_stack, TRACE_PUMP_STACK);
static struct k_thread s_trace_pump_thread;
static k_tid_t s_trace_pump_tid = NULL;

static atomic_t s_async_enabled = ATOMIC_INIT(0);
static atomic_t s_pump_started = ATOMIC_INIT(0);
static atomic_t s_print_dropped = ATOMIC_INIT(0);
static volatile bool s_panic_mode = false;

static void uart_write_bytes(const char *buf, size_t len)
{
	if (!buf || len == 0) {
		return;
	}

	bool uart_ok = (s_console_dev != NULL) && device_is_ready(s_console_dev);
	if (uart_ok) {
		for (size_t i = 0; i < len; i++) {
			if (buf[i] == '\n') {
				uart_poll_out(s_console_dev, '\r');
			}
			uart_poll_out(s_console_dev, (unsigned char)buf[i]);
		}
	} else {
		for (size_t i = 0; i < len; i++) {
			printk("%c", buf[i]);
		}
	}
}

static void drain_rb_locked(void)
{
	uint8_t chunk[64];

	while (1) {
		k_spinlock_key_t key = k_spin_lock(&s_rb_lock);
		uint32_t got = ring_buf_get(&s_trace_rb, chunk, sizeof(chunk));
		k_spin_unlock(&s_rb_lock, key);
		if (got == 0) {
			break;
		}
		uart_write_bytes((const char *)chunk, got);
	}
}

static void submit_formatted(const char *buf, size_t len)
{
	if (!buf || len == 0) {
		return;
	}

	if (!s_panic_mode && atomic_get(&s_async_enabled) != 0) {
		bool queued = false;
		k_spinlock_key_t key = k_spin_lock(&s_rb_lock);
		if (ring_buf_space_get(&s_trace_rb) >= (uint32_t)len) {
			(void)ring_buf_put(&s_trace_rb, (const uint8_t *)buf, (uint32_t)len);
			queued = true;
		} else {
			atomic_inc(&s_print_dropped);
		}
		k_spin_unlock(&s_rb_lock, key);
		if (queued) {
			k_sem_give(&s_trace_sem);
		}
		return;
	}

	/* Synchronous boot / panic / pre-sleep path */
	if (k_is_in_isr() || s_panic_mode) {
		uart_write_bytes(buf, len);
		return;
	}

	k_mutex_lock(&s_trace_tx_mutex, K_FOREVER);
	drain_rb_locked();
	uart_write_bytes(buf, len);
	k_mutex_unlock(&s_trace_tx_mutex);
}

#if defined(CONFIG_LOG) && !defined(CONFIG_LOG_BACKEND_UART)
static uint8_t s_log_out_buf[64];
static char s_log_line_buf[160];
static size_t s_log_line_len = 0;

static void flush_log_line(void)
{
	if (s_log_line_len == 0) {
		return;
	}
	s_log_line_buf[s_log_line_len] = '\0';
	char prefix[32];
	int plen = snprintf(prefix, sizeof(prefix), "C:0 [F:(zephyr), L:0] ");
	if (plen > 0) {
		uart_write_bytes(prefix, (size_t)plen);
	}
	uart_write_bytes(s_log_line_buf, s_log_line_len);
	uart_write_bytes("\n", 1);
	s_log_line_len = 0;
}

static int wildsonic_log_char_out(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);

	for (size_t i = 0; i < length; i++) {
		char c = (char)data[i];
		if (c == '\r') {
			continue;
		}
		if (c == '\n') {
			flush_log_line();
		} else if (s_log_line_len < sizeof(s_log_line_buf) - 1U) {
			s_log_line_buf[s_log_line_len++] = c;
		}
	}
	return (int)length;
}

LOG_OUTPUT_DEFINE(s_wildsonic_log_output, wildsonic_log_char_out,
		  s_log_out_buf, sizeof(s_log_out_buf));

static void ws_log_process(const struct log_backend *const backend,
			   union log_msg_generic *msg)
{
	ARG_UNUSED(backend);
	uint32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_CRLF_LFONLY;

	log_output_msg_process(&s_wildsonic_log_output, &msg->log, flags);
	if (s_log_line_len > 0) {
		flush_log_line();
	}
}

static void ws_log_dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);
	atomic_add(&s_print_dropped, (atomic_val_t)cnt);
}

static void ws_log_panic(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);
	s_panic_mode = true;
	atomic_set(&s_async_enabled, 0);
}

static void ws_log_init(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);
}

static const struct log_backend_api s_wildsonic_log_api = {
	.process = ws_log_process,
	.dropped = ws_log_dropped,
	.panic = ws_log_panic,
	.init = ws_log_init,
};

LOG_BACKEND_DEFINE(wildsonic_log, s_wildsonic_log_api, true);
#endif /* CONFIG_LOG && !CONFIG_LOG_BACKEND_UART */

static void trace_pump_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&s_trace_sem, K_MSEC(50));

		k_mutex_lock(&s_trace_tx_mutex, K_FOREVER);
		drain_rb_locked();
#if defined(CONFIG_LOG)
		while (log_process()) {
		}
#endif
		k_mutex_unlock(&s_trace_tx_mutex);
	}
}

void wildsonic_trace_init(void)
{
	/* Statically initialized */
}

void wildsonic_trace_start_pump(void)
{
	if (atomic_cas(&s_pump_started, 0, 1)) {
		s_trace_pump_tid = k_thread_create(
			&s_trace_pump_thread, s_trace_pump_stack,
			K_THREAD_STACK_SIZEOF(s_trace_pump_stack),
			trace_pump_fn, NULL, NULL, NULL,
			TRACE_PUMP_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(s_trace_pump_tid, "trace_pump");
	}
	atomic_set(&s_async_enabled, 1);
}

void wildsonic_trace_set_async(bool enable)
{
	if (!enable) {
		atomic_set(&s_async_enabled, 0);
		wildsonic_trace_flush();
	} else {
		atomic_set(&s_async_enabled, 1);
		k_sem_give(&s_trace_sem);
	}
}

void wildsonic_trace_flush(void)
{
	if (k_is_in_isr()) {
		return;
	}
	k_mutex_lock(&s_trace_tx_mutex, K_FOREVER);
	drain_rb_locked();
#if defined(CONFIG_LOG)
	while (log_process()) {
	}
#endif
	k_mutex_unlock(&s_trace_tx_mutex);
}

uint32_t wildsonic_trace_get_dropped(void)
{
	return (uint32_t)atomic_get(&s_print_dropped);
}

void wildsonic_trace_emit(const char *file, int line, const char *fmt, ...)
{
	if (!file || !fmt) {
		return;
	}

	const char *slash = strrchr(file, '/');
	const char *base = slash ? slash + 1 : file;
	char buf[TRACE_LINE_MAX];

	int hdr = snprintf(buf, sizeof(buf), "C:0 [F:(%s), L:%d] ", base, line);
	if (hdr < 0) {
		return;
	}
	size_t pos = (size_t)hdr;
	if (pos >= sizeof(buf) - 2U) {
		pos = sizeof(buf) - 2U;
	}

	va_list ap;
	va_start(ap, fmt);
	int body = vsnprintf(&buf[pos], sizeof(buf) - pos - 1U, fmt, ap);
	va_end(ap);

	if (body > 0) {
		size_t avail = sizeof(buf) - pos - 1U;
		pos += ((size_t)body < avail) ? (size_t)body : (avail - 1U);
	}

	if (pos == 0 || buf[pos - 1U] != '\n') {
		buf[pos++] = '\n';
	}
	buf[pos] = '\0';

	submit_formatted(buf, pos);
}

void wildsonic_raw_emit(const char *fmt, ...)
{
	if (!fmt) {
		return;
	}

	char buf[TRACE_RAW_MAX];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if (n <= 0) {
		return;
	}
	size_t len = ((size_t)n < sizeof(buf)) ? (size_t)n : (sizeof(buf) - 1U);
	submit_formatted(buf, len);
}

int wildsonic_trace_get_stack_free(size_t *total_bytes, size_t *unused_bytes)
{
	if (total_bytes) {
		*total_bytes = TRACE_PUMP_STACK;
	}
	if (!unused_bytes) {
		return -EINVAL;
	}
	if (!s_trace_pump_tid) {
		*unused_bytes = TRACE_PUMP_STACK;
		return -ENODEV;
	}
	return k_thread_stack_space_get(s_trace_pump_tid, unused_bytes);
}

#else

static uint32_t s_host_dropped = 0;

void wildsonic_trace_init(void) {}
void wildsonic_trace_start_pump(void) {}
void wildsonic_trace_flush(void) {}
void wildsonic_trace_set_async(bool enable) { (void)enable; }
uint32_t wildsonic_trace_get_dropped(void) { return s_host_dropped; }
int wildsonic_trace_get_stack_free(size_t *total_bytes, size_t *unused_bytes)
{
	if (total_bytes) *total_bytes = 0;
	if (unused_bytes) *unused_bytes = 0;
	return -ENOTSUP;
}

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
