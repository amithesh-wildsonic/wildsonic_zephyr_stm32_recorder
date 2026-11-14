/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WILDSONIC_TRACE_H_
#define WILDSONIC_TRACE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void wildsonic_trace_init(void);
void wildsonic_trace_start_pump(void);
void wildsonic_trace_flush(void);
void wildsonic_trace_set_async(bool enable);
uint32_t wildsonic_trace_get_dropped(void);
int wildsonic_trace_get_stack_free(size_t *total_bytes, size_t *unused_bytes);

void wildsonic_trace_emit(const char *file, int line, const char *fmt, ...);
void wildsonic_raw_emit(const char *fmt, ...);

#define WILDSONIC_TRACE(fmt, ...) \
	wildsonic_trace_emit(__FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define TRACE(fmt, ...) \
	wildsonic_trace_emit(__FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define TRACE_RAW(fmt, ...) \
	wildsonic_raw_emit(fmt, ##__VA_ARGS__)

#if defined(CONFIG_WILDSONIC_TEST_OUTPUT)
#define TRACE_TEST(fmt, ...) \
	wildsonic_trace_emit(__FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define TRACE_RAW_TEST(fmt, ...) \
	wildsonic_raw_emit(fmt, ##__VA_ARGS__)
#else
#define TRACE_TEST(fmt, ...) do { } while (0)
#define TRACE_RAW_TEST(fmt, ...) do { } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* WILDSONIC_TRACE_H_ */
