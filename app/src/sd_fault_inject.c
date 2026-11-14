/* SPDX-License-Identifier: Apache-2.0 */
#include "sd_fault_inject.h"
#include <errno.h>
#include <zephyr/kernel.h>
#include "wildsonic_trace.h"

#if defined(CONFIG_WILDSONIC_SD_FAULT_INJECT)

static struct k_spinlock s_fault_lock;

static bool s_f1_active = false;
static uint32_t s_f2_attempts_left = 0;
static bool s_f3_active = false;
static bool s_f4_active = false;
static uint32_t s_f5_attempts_left = 0;

void sd_fault_inject_f1(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  s_f1_active = true;
  k_spin_unlock(&s_fault_lock, key);
  TRACE_RAW_TEST("[FAULT_INJECT] F1 armed: Next write will fail with CRC error (-EIO)\n");
}

void sd_fault_inject_f2(uint32_t attempts) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  s_f2_attempts_left = attempts;
  k_spin_unlock(&s_fault_lock, key);
  TRACE_RAW_TEST("[FAULT_INJECT] F2 armed: Next %u writes will fail with -EIO then succeed\n",
                 (unsigned int)attempts);
}

void sd_fault_inject_f3(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  s_f3_active = true;
  k_spin_unlock(&s_fault_lock, key);
  TRACE_RAW_TEST("[FAULT_INJECT] F3 armed: Dead handle simulation (returns -9/-EBADF until card reinit)\n");
}

void sd_fault_inject_f4(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  s_f4_active = true;
  k_spin_unlock(&s_fault_lock, key);
  TRACE_RAW_TEST("[FAULT_INJECT] F4 armed: Permanent card failure (card dead)\n");
}

void sd_fault_inject_f5(uint32_t attempts) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  s_f5_attempts_left = attempts;
  k_spin_unlock(&s_fault_lock, key);
  TRACE_RAW_TEST("[FAULT_INJECT] F5 armed: Next %u mount attempts will fail (-EIO)\n",
                 (unsigned int)attempts);
}

void sd_fault_inject_clear(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  s_f1_active = false;
  s_f2_attempts_left = 0;
  s_f3_active = false;
  s_f4_active = false;
  s_f5_attempts_left = 0;
  k_spin_unlock(&s_fault_lock, key);
  TRACE_RAW_TEST("[FAULT_INJECT] All fault injection hooks cleared\n");
}

void sd_fault_inject_status(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  bool f1 = s_f1_active;
  uint32_t f2 = s_f2_attempts_left;
  bool f3 = s_f3_active;
  bool f4 = s_f4_active;
  uint32_t f5 = s_f5_attempts_left;
  k_spin_unlock(&s_fault_lock, key);

  TRACE_RAW_TEST("=== SD FAULT INJECTION STATUS ===\n");
  TRACE_RAW_TEST("  F1 (Fail next write CRC)     : %s\n", f1 ? "ARMED" : "OFF");
  TRACE_RAW_TEST("  F2 (Fail N writes)           : %u attempts remaining\n", (unsigned int)f2);
  TRACE_RAW_TEST("  F3 (Dead handle, return -9)  : %s\n", f3 ? "ARMED" : "OFF");
  TRACE_RAW_TEST("  F4 (Permanent failure)       : %s\n", f4 ? "ARMED" : "OFF");
  TRACE_RAW_TEST("  F5 (Fail M mounts after wake): %u attempts remaining\n", (unsigned int)f5);
}

int sd_fault_inject_check_write(uint32_t sector, uint32_t count) {
  ARG_UNUSED(sector);
  ARG_UNUSED(count);

  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  if (s_f4_active) {
    k_spin_unlock(&s_fault_lock, key);
    TRACE_TEST("[FAULT_INJECT] F4 triggered -> -EIO");
    return -EIO;
  }

  if (s_f3_active) {
    k_spin_unlock(&s_fault_lock, key);
    TRACE_TEST("[FAULT_INJECT] F3 triggered -> -EBADF (-9)");
    return -EBADF;
  }

  if (s_f1_active) {
    s_f1_active = false;
    k_spin_unlock(&s_fault_lock, key);
    TRACE_TEST("[FAULT_INJECT] F1 triggered -> -EIO");
    return -EIO;
  }

  if (s_f2_attempts_left > 0) {
    s_f2_attempts_left--;
    uint32_t rem = s_f2_attempts_left;
    k_spin_unlock(&s_fault_lock, key);
    TRACE_TEST("[FAULT_INJECT] F2 triggered (rem %u) -> -EIO", rem);
    return -EIO;
  }

  k_spin_unlock(&s_fault_lock, key);
  return 0;
}

int sd_fault_inject_check_mount(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  if (s_f4_active) {
    k_spin_unlock(&s_fault_lock, key);
    TRACE_TEST("[FAULT_INJECT] F4 mount -> -EIO");
    return -EIO;
  }

  if (s_f5_attempts_left > 0) {
    s_f5_attempts_left--;
    uint32_t rem = s_f5_attempts_left;
    k_spin_unlock(&s_fault_lock, key);
    TRACE_TEST("[FAULT_INJECT] F5 mount (rem %u) -> -EIO", rem);
    return -EIO;
  }

  k_spinlock_key_t key2 = key;
  k_spin_unlock(&s_fault_lock, key2);
  return 0;
}

void sd_fault_inject_notify_reinit(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  if (s_f3_active) {
    s_f3_active = false;
    TRACE_TEST("[FAULT_INJECT] Re-init: F3 cleared");
  }
  k_spin_unlock(&s_fault_lock, key);
}

bool sd_fault_inject_is_f3_active(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  bool ret = s_f3_active;
  k_spin_unlock(&s_fault_lock, key);
  return ret;
}

bool sd_fault_inject_is_f4_active(void) {
  k_spinlock_key_t key = k_spin_lock(&s_fault_lock);
  bool ret = s_f4_active;
  k_spin_unlock(&s_fault_lock, key);
  return ret;
}

#endif /* CONFIG_WILDSONIC_SD_FAULT_INJECT */
