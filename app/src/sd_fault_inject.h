/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SD_FAULT_INJECT_H_
#define SD_FAULT_INJECT_H_

#include <stdbool.h>
#include <stdint.h>

#if defined(CONFIG_WILDSONIC_SD_FAULT_INJECT)

/*
 * Fault injection modes:
 * F1: Fail the next write once (CRC)
 * F2: Fail every write for N attempts then succeed
 * F3: Return -9 for every FatFs call until a full re-init happens
 * F4: Fail all writes permanently (card dead)
 * F5: Fail mount for M attempts after wake
 */

void sd_fault_inject_f1(void);
void sd_fault_inject_f2(uint32_t attempts);
void sd_fault_inject_f3(void);
void sd_fault_inject_f4(void);
void sd_fault_inject_f5(uint32_t attempts);
void sd_fault_inject_clear(void);
void sd_fault_inject_status(void);

/* Invoked by Layer 1 & 2 */
int sd_fault_inject_check_write(uint32_t sector, uint32_t count);
int sd_fault_inject_check_mount(void);
void sd_fault_inject_notify_reinit(void);

bool sd_fault_inject_is_f3_active(void);
bool sd_fault_inject_is_f4_active(void);

#else

static inline void sd_fault_inject_f1(void) {}
static inline void sd_fault_inject_f2(uint32_t attempts) { (void)attempts; }
static inline void sd_fault_inject_f3(void) {}
static inline void sd_fault_inject_f4(void) {}
static inline void sd_fault_inject_f5(uint32_t attempts) { (void)attempts; }
static inline void sd_fault_inject_clear(void) {}
static inline void sd_fault_inject_status(void) {}

static inline int sd_fault_inject_check_write(uint32_t sector, uint32_t count) {
  (void)sector; (void)count; return 0;
}
static inline int sd_fault_inject_check_mount(void) { return 0; }
static inline void sd_fault_inject_notify_reinit(void) {}
static inline bool sd_fault_inject_is_f3_active(void) { return false; }
static inline bool sd_fault_inject_is_f4_active(void) { return false; }

#endif /* CONFIG_WILDSONIC_SD_FAULT_INJECT */

#endif /* SD_FAULT_INJECT_H_ */
