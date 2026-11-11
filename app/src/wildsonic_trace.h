/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WILDSONIC_TRACE_H_
#define WILDSONIC_TRACE_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void wildsonic_trace_init(void);
void wildsonic_trace_emit(const char *file, int line, const char *fmt, ...);
void wildsonic_raw_emit(const char *fmt, ...);

#define WILDSONIC_TRACE(fmt, ...) \
	wildsonic_trace_emit(__FILE__, __LINE__, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* WILDSONIC_TRACE_H_ */
