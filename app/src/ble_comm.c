/* SPDX-License-Identifier: Apache-2.0 */
#include "ble_comm.h"

#ifndef WILDSONIC_GIT_COMMIT
#define WILDSONIC_GIT_COMMIT "unknown"
#endif
#include "console.h"
#include "main.h"
#include "recorder.h"
#include "rtc_calendar.h"
#include "scheduler.h"
#include "scheduler_retention.h"

#include <zephyr/drivers/uart.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#if defined(CONFIG_SOC_SERIES_STM32L4X)
#include <stm32_ll_usart.h>
#endif

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(ws_ble, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(ws_ble, LOG_LEVEL_WRN);
#endif

#define BLE_RX_BUF_SIZE 512
#define BLE_TX_BUF_SIZE 1024
#define BLE_COMM_STACK_SIZE 3072
#define BLE_COMM_PRIORITY 8
/* Keep BLE status/telemetry responses immediate. Per-line sleep pacing blocks the
 * UART command-processing thread and stretches mobile command latency without
 * changing the actual protocol framing.
 */
#define BLE_RESPONSE_PACING_MS 0

RING_BUF_DECLARE(s_ble_rx_rb, BLE_RX_BUF_SIZE);
RING_BUF_DECLARE(s_ble_tx_rb, BLE_TX_BUF_SIZE);

static struct k_sem s_ble_rx_sem;
static struct k_mutex s_ble_cmd_mutex;
static struct k_mutex s_ble_report_mutex;
static struct k_sem s_ble_ack_sem;
static struct k_work s_ble_lock_work;

static const struct device *s_ble_uart_dev = NULL;
static bool s_ble_uart_ready = false;
static bool s_ble_supervisor_online = false;
static bool s_ble_connected = false;
static bool s_ble_module_awake = false;
static bool s_pairing_window_active = false;
static int64_t s_pairing_window_end_ms = 0;
static bool s_lock_pending_on_disconnect = false;
static bool s_ble_test_capture_enabled = false;

static uint8_t s_current_slpm = 1;
static uint8_t s_current_adty = 0;
static bool s_name_update_pending = false;

static char s_ble_cmd_buf[BLE_UART_CMD_MAX_LEN];
static size_t s_ble_cmd_len = 0;
static bool s_ble_discard_until_terminator = false;
static bool s_ble_last_was_cr = false;
static uint32_t s_ble_overflow_line_count = 0;
static uint32_t s_ble_rx_drop_count = 0;

#if defined(CONFIG_AUDIO_DIAGNOSTIC_MODE) || defined(CONFIG_AUDIO_STAGE_DIAGNOSTICS)
static uint8_t s_ble_overflow_diag[48] = {0};
static size_t s_ble_overflow_diag_len = 0;
static void ble_comm_capture_overflow_diag(const uint8_t *data, size_t len)
{
  size_t n = len;
  if (n > sizeof(s_ble_overflow_diag)) {
    n = sizeof(s_ble_overflow_diag);
  }
  memset(s_ble_overflow_diag, 0, sizeof(s_ble_overflow_diag));
  if (data && n > 0) {
    memcpy(s_ble_overflow_diag, data, n);
  }
  s_ble_overflow_diag_len = n;
}
#endif

static const char *s_expected_ack = NULL;
static char s_ack_resp_buf[64] = {0};
static bool s_ack_matched = false;

K_THREAD_STACK_DEFINE(s_ble_stack, BLE_COMM_STACK_SIZE);
static struct k_thread s_ble_thread;
static bool s_ble_thread_started = false;

static void ble_lock_work_fn(struct k_work *work);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
static void ble_uart_isr(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);

  uart_irq_update(dev);
  while (uart_irq_rx_ready(dev)) {
    uint8_t rx_byte;
    if (uart_fifo_read(dev, &rx_byte, 1) <= 0) {
      break;
    }
    if (ring_buf_put(&s_ble_rx_rb, &rx_byte, 1) == 0) {
      s_ble_rx_drop_count++;
      continue;
    }
    k_sem_give(&s_ble_rx_sem);
  }
  (void)uart_err_check(dev);
}
#endif

int ble_comm_tx(const uint8_t *data, size_t len) {
  if (!data || len == 0) {
    return 0;
  }

  /* Copy to host-test drain ring buffer only when test capture is enabled */
  if (s_ble_test_capture_enabled) {
    ring_buf_put(&s_ble_tx_rb, data, len);
  }

  if (s_ble_uart_dev && s_ble_uart_ready) {
    for (size_t i = 0; i < len; i++) {
      uart_poll_out(s_ble_uart_dev, data[i]);
    }
  }
  return (int)len;
}

int ble_comm_printf(const char *format, ...) {
  char buf[BLE_UART_CMD_MAX_LEN];
  va_list args;
  va_start(args, format);
  int num = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (num <= 0) {
    return num;
  }
  if ((size_t)num >= sizeof(buf)) {
    num = sizeof(buf) - 1;
  }
  return ble_comm_tx((const uint8_t *)buf, (size_t)num);
}

uint32_t ble_comm_calc_fingerprint(const char *adv_name, const char *pin,
                                   uint8_t ws_mode, uint8_t slpm, uint8_t adty,
                                   uint8_t type) {
  uint32_t h = 0x811C9DC5U;
  const uint32_t prime = 0x01000193U;
  h = (h ^ ws_mode) * prime;
  h = (h ^ slpm) * prime;
  h = (h ^ adty) * prime;
  h = (h ^ type) * prime;
  if (adv_name) {
    for (const char *p = adv_name; *p; p++) {
      h = (h ^ (uint8_t)*p) * prime;
    }
  }
  if (pin) {
    for (const char *p = pin; *p; p++) {
      h = (h ^ (uint8_t)*p) * prime;
    }
  }
  return h;
}

int ble_comm_send_cmd_expect(const char *cmd, const char *expected_ack,
                             char *resp_out, size_t resp_max,
                             uint32_t timeout_ms) {
  if ((!s_ble_uart_ready && !s_ble_test_capture_enabled) || !cmd ||
      !expected_ack) {
    return -EINVAL;
  }

  /* Guard: AT commands are strictly blocked while BLE is connected */
  if (s_ble_connected) {
    LOG_WRN("AT command '%s' rejected: BLE connection active", cmd);
    return -EBUSY;
  }

  if (k_mutex_lock(&s_ble_cmd_mutex, K_MSEC(timeout_ms)) != 0) {
    return -EBUSY;
  }

  s_expected_ack = expected_ack;
  s_ack_matched = false;
  s_ack_resp_buf[0] = '\0';
  k_sem_reset(&s_ble_ack_sem);

  /* Transmit exact command over USART1 with CRLF if not already present */
  size_t cmd_len = strlen(cmd);
  ble_comm_tx((const uint8_t *)cmd, cmd_len);
  if (cmd_len < 2 || cmd[cmd_len - 1] != '\n') {
    ble_comm_tx((const uint8_t *)"\r\n", 2);
  }

  /* Wait for expected response with bounded timeout */
  int rc = k_sem_take(&s_ble_ack_sem, K_MSEC(timeout_ms));

  /* If first transmission woke a sleeping Mode 1 module (indicated by
   * s_ble_module_awake),
   * but the command itself timed out or was missed during wake transition,
   * retry once */
  if ((rc != 0 || !s_ack_matched) && s_ble_module_awake && !s_ble_connected) {
    k_sem_reset(&s_ble_ack_sem);
    s_expected_ack = expected_ack;
    s_ack_matched = false;
    s_ack_resp_buf[0] = '\0';

    ble_comm_tx((const uint8_t *)cmd, cmd_len);
    if (cmd_len < 2 || cmd[cmd_len - 1] != '\n') {
      ble_comm_tx((const uint8_t *)"\r\n", 2);
    }
    rc = k_sem_take(&s_ble_ack_sem, K_MSEC(timeout_ms));
  }

  s_expected_ack = NULL;

  if (rc == 0 && s_ack_matched) {
    if (resp_out && resp_max > 0) {
      strncpy(resp_out, s_ack_resp_buf, resp_max - 1);
      resp_out[resp_max - 1] = '\0';
    }
    k_mutex_unlock(&s_ble_cmd_mutex);
    return 0;
  }

  k_mutex_unlock(&s_ble_cmd_mutex);
  return -ETIMEDOUT;
}

static void ble_comm_process_line(char *line) {
  if (!line)
    return;

  /* Trim leading whitespace */
  char *p = line;
  while (*p && isspace((unsigned char)*p))
    p++;

  /* Trim trailing whitespace */
  size_t len = strlen(p);
  while (len > 0 && isspace((unsigned char)p[len - 1])) {
    p[--len] = '\0';
  }
  if (len == 0)
    return;

  /* 1. BLE Module Supervisor Modem Events */
  if (strcmp(p, "OK+CONN") == 0) {
    s_ble_connected = true;
    s_ble_module_awake = true;
    s_ble_supervisor_online = true;
    s_lock_pending_on_disconnect = false;
    LOG_INF("[BLE] Central connected (timestamp=%lld ms)", k_uptime_get());
    return;
  }
  if (strcmp(p, "OK+LOST") == 0) {
    s_ble_connected = false;
    s_ble_module_awake = false;
    LOG_INF("[BLE] Central disconnected (timestamp=%lld ms)", k_uptime_get());
    if (s_name_update_pending) {
      s_name_update_pending = false;
      ble_comm_apply_advertised_name();
    }
    if (s_lock_pending_on_disconnect) {
      s_lock_pending_on_disconnect = false;
      s_pairing_window_active = false;
      k_work_submit(&s_ble_lock_work);
    }
    return;
  }
  if (strcmp(p, "OK+WAKE") == 0) {
    s_ble_module_awake = true;
    s_ble_supervisor_online = true;
    if (s_expected_ack != NULL && strcmp(s_expected_ack, "OK+WAKE") == 0) {
      strncpy(s_ack_resp_buf, p, sizeof(s_ack_resp_buf) - 1);
      s_ack_resp_buf[sizeof(s_ack_resp_buf) - 1] = '\0';
      s_ack_matched = true;
      k_sem_give(&s_ble_ack_sem);
    }
    return;
  }

  /* 2. Command Transaction ACK Matching */
  if (s_expected_ack != NULL) {
    bool match = false;
    size_t exp_len = strlen(s_expected_ack);
    if (s_expected_ack[exp_len - 1] == ':') {
      /* Prefix match, e.g. "OK+SLPM:", "OK+Get:", "OK+Set:" */
      if (strncmp(p, s_expected_ack, exp_len) == 0) {
        match = true;
      }
    } else {
      /* Exact match, e.g. "OK", "OK+Set:0", "OK+Set:1", "OK+Set:2", "OK+SLEEP",
       * "OK+RESET" */
      if (strcmp(p, s_expected_ack) == 0) {
        match = true;
      }
    }

    if (match) {
      strncpy(s_ack_resp_buf, p, sizeof(s_ack_resp_buf) - 1);
      s_ack_resp_buf[sizeof(s_ack_resp_buf) - 1] = '\0';
      s_ack_matched = true;
      k_sem_give(&s_ble_ack_sem);
      return;
    }
  }

  if (strcmp(p, "OK+SLEEP") == 0) {
    s_ble_module_awake = false;
    if (s_expected_ack != NULL && strcmp(s_expected_ack, "OK+SLEEP") == 0) {
      strncpy(s_ack_resp_buf, p, sizeof(s_ack_resp_buf) - 1);
      s_ack_resp_buf[sizeof(s_ack_resp_buf) - 1] = '\0';
      s_ack_matched = true;
      k_sem_give(&s_ble_ack_sem);
    }
    return;
  }
  if (strcmp(p, "OK") == 0 || strncmp(p, "OK+", 3) == 0) {
    /* Unmatched or late ACK, ignore cleanly without forwarding to dispatcher */
    return;
  }

  /* 3. Mobile Companion Application Requests */
  s_ble_connected = true;
  s_ble_module_awake = true;
  s_ble_supervisor_online = true;

  if (strcmp(p, "WS+ALIVE") == 0) {
    s_pairing_window_active = true;
    s_lock_pending_on_disconnect = false;
    LOG_INF("[BLE] RX: WS+ALIVE (keep-alive, pairing window active)");
    return;
  }

  if (strcasecmp(p, "GET_DEV_INFO") == 0) {
    s_ble_module_awake = true;
    LOG_INF("[BLE] RX: GET_DEV_INFO (dispatching formatted device report)");
    ble_comm_send_dev_info();
    return;
  }
  if (strcasecmp(p, "GET_STATUS_INFO") == 0 || strcasecmp(p, "STATUS") == 0) {
    s_ble_module_awake = true;
    LOG_INF("[BLE] RX: GET_STATUS_INFO (dispatching formatted status report)");
    ble_comm_send_status_info();
    return;
  }

  /* 4. Configuration & Control Commands (Forwarded to existing dispatcher) */
  s_ble_module_awake = true;

  /* Show the command source in the local console without exposing its value
   * (for example, a password or other credential-bearing argument). */
  const char *cmd_end = p;
  while (*cmd_end && !isspace((unsigned char)*cmd_end)) {
    cmd_end++;
  }

  char resp_buf[128] = {0};
  int rc = console_process_line_source(p, true, resp_buf, sizeof(resp_buf));
  printk("[BLE] Command %.*s %s\n", (int)(cmd_end - p), p,
         rc == 0 ? "accepted" : "rejected");
  if (rc == 0) {
    if (resp_buf[0] != '\0') {
      ble_comm_printf("%s\r\n", resp_buf);
    } else {
      ble_comm_printf("OK\r\n");
    }
  } else {
    if (resp_buf[0] != '\0') {
      ble_comm_printf("%s\r\n", resp_buf);
    } else {
      ble_comm_printf("ERR\r\n");
    }
  }
}

static void ble_comm_handle_rx_byte(char c) {
  /* Discard malformed oversized lines until a fresh terminator arrives. */
  if (s_ble_discard_until_terminator) {
    if (c == '\r' || c == '\n') {
      s_ble_last_was_cr = (c == '\r');
      s_ble_cmd_len = 0;
      memset(s_ble_cmd_buf, 0, sizeof(s_ble_cmd_buf));
      s_ble_discard_until_terminator = false;
    }
    return;
  }

  if (c == '\r') {
    if (s_ble_last_was_cr) {
      s_ble_last_was_cr = false;
      return;
    }
    if (s_ble_cmd_len >= sizeof(s_ble_cmd_buf) - 1) {
      s_ble_discard_until_terminator = true;
      s_ble_overflow_line_count++;
#if defined(CONFIG_AUDIO_DIAGNOSTIC_MODE) || defined(CONFIG_AUDIO_STAGE_DIAGNOSTICS)
      ble_comm_capture_overflow_diag((const uint8_t *)s_ble_cmd_buf,
                                    s_ble_cmd_len);
#endif
      LOG_WRN("[BLE] Command line oversized (>%u bytes), dropping excess",
              (unsigned int)sizeof(s_ble_cmd_buf));
      s_ble_last_was_cr = true;
      s_ble_cmd_len = 0;
      memset(s_ble_cmd_buf, 0, sizeof(s_ble_cmd_buf));
      return;
    }
    if (s_ble_cmd_len > 0) {
      s_ble_cmd_buf[s_ble_cmd_len] = '\0';
      ble_comm_process_line(s_ble_cmd_buf);
    }
    s_ble_cmd_len = 0;
    memset(s_ble_cmd_buf, 0, sizeof(s_ble_cmd_buf));
    s_ble_last_was_cr = true;
    return;
  }

  if (c == '\n') {
    if (s_ble_last_was_cr) {
      s_ble_last_was_cr = false;
      return;
    }
    if (s_ble_cmd_len >= sizeof(s_ble_cmd_buf) - 1) {
      s_ble_discard_until_terminator = true;
      s_ble_overflow_line_count++;
#if defined(CONFIG_AUDIO_DIAGNOSTIC_MODE) || defined(CONFIG_AUDIO_STAGE_DIAGNOSTICS)
      ble_comm_capture_overflow_diag((const uint8_t *)s_ble_cmd_buf,
                                    s_ble_cmd_len);
#endif
      LOG_WRN("[BLE] Command line oversized (>%u bytes), dropping excess",
              (unsigned int)sizeof(s_ble_cmd_buf));
      s_ble_cmd_len = 0;
      memset(s_ble_cmd_buf, 0, sizeof(s_ble_cmd_buf));
      return;
    }
    if (s_ble_cmd_len > 0) {
      s_ble_cmd_buf[s_ble_cmd_len] = '\0';
      ble_comm_process_line(s_ble_cmd_buf);
    }
    s_ble_cmd_len = 0;
    memset(s_ble_cmd_buf, 0, sizeof(s_ble_cmd_buf));
    return;
  }

  s_ble_last_was_cr = false;
  if (s_ble_cmd_len >= sizeof(s_ble_cmd_buf) - 1) {
    s_ble_discard_until_terminator = true;
    s_ble_overflow_line_count++;
#if defined(CONFIG_AUDIO_DIAGNOSTIC_MODE) || defined(CONFIG_AUDIO_STAGE_DIAGNOSTICS)
    ble_comm_capture_overflow_diag((const uint8_t *)s_ble_cmd_buf,
                                  s_ble_cmd_len);
#endif
    LOG_WRN("[BLE] Command line oversized (>%u bytes), dropping excess",
            (unsigned int)sizeof(s_ble_cmd_buf));
    return;
  }

  s_ble_cmd_buf[s_ble_cmd_len++] = c;
}

static void ble_lock_work_fn(struct k_work *work) {
  ARG_UNUSED(work);
  ble_comm_lock_pairing();
}

static void ble_comm_thread_fn(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  while (1) {
    int ret = k_sem_take(&s_ble_rx_sem, K_MSEC(100));

    /* Service pairing window timeout */
    if (s_pairing_window_active) {
      int64_t now = k_uptime_get();
      if (now >= s_pairing_window_end_ms) {
        s_pairing_window_active = false;
        if (s_ble_connected) {
          s_lock_pending_on_disconnect = true;
          LOG_INF("[BLE] Pairing window expired while connected; will lock on "
                  "disconnect");
        } else {
          k_work_submit(&s_ble_lock_work);
        }
      }
    }

    if (ret != 0) {
      continue;
    }

    uint8_t c;
    while (ring_buf_get(&s_ble_rx_rb, &c, 1) == 1) {
      ble_comm_handle_rx_byte((char)c);
    }
  }
}

int ble_comm_init(const struct device *uart_dev) {
  if (!uart_dev || !device_is_ready(uart_dev)) {
    return -ENODEV;
  }
  s_ble_uart_dev = uart_dev;

  k_mutex_init(&s_ble_cmd_mutex);
  k_mutex_init(&s_ble_report_mutex);
  k_sem_init(&s_ble_rx_sem, 0, UINT_MAX);
  k_sem_init(&s_ble_ack_sem, 0, 1);
  k_work_init(&s_ble_lock_work, ble_lock_work_fn);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
  uart_irq_callback_user_data_set(s_ble_uart_dev, ble_uart_isr, NULL);
  uart_irq_rx_enable(s_ble_uart_dev);
#endif

  /* Mark UART hardware as ready so initial probes can transmit */
  s_ble_uart_ready = true;

  if (!s_ble_thread_started) {
    k_thread_create(&s_ble_thread, s_ble_stack,
                    K_THREAD_STACK_SIZEOF(s_ble_stack), ble_comm_thread_fn,
                    NULL, NULL, NULL, BLE_COMM_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&s_ble_thread, "ble_comm");
    s_ble_thread_started = true;
  }

  /*
   * Compute target BLE provisioning fingerprint.
   * Hashes BLE-specific settings only: ws_mode=0, slpm=1, adty=0, type=3,
   * pin="123456", adv_name. Excludes audio parameters.
   */
  struct wildsonic_config *cfg = console_get_config();
  char adv_name[16];
  ble_comm_build_advertised_name(cfg ? cfg->device_name : NULL,
                                 cfg ? cfg->serial_number : NULL, adv_name,
                                 sizeof(adv_name));

  uint32_t target_fp =
      ble_comm_calc_fingerprint(adv_name, "123456", 0, 1, 0, 3);

  /* Check retained provisioning fingerprint in RTC backup registers */
  uint32_t retained_fp = 0;
  if (ble_retention_restore_fingerprint(&retained_fp) == 0 &&
      retained_fp == target_fp) {
    /*
     * Fingerprint matches: BLE supervisor was previously provisioned and holds
     * persistent configuration in its NVS. It is currently in Mode 1 low-power
     * connectable advertising. Do NOT send AT probes, pings, or configuration
     * commands.
     */
    s_ble_supervisor_online = true;
    s_ble_module_awake = false;
    s_current_adty = 0;
    s_current_slpm = 1;
    s_pairing_window_active = false;
    s_pairing_window_end_ms = 0;
    s_lock_pending_on_disconnect = false;
    LOG_INF("[BLE] Access: UNLOCKED (initial boot state)");
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[BLE] Access: UNLOCKED (initial boot state)\n");
#endif
    return 0;
  }

  /*
   * Phase 1 & 2: Bounded probe for BLE supervisor module
   * Send bare "AT\r\n" and wait for exact "OK\r\n".
   * Up to 5 attempts with 100 ms timeout per attempt.
   */
  bool probed = false;
  for (int i = 0; i < 5; i++) {
    if (ble_comm_send_cmd_expect("AT\r\n", "OK", NULL, 0, 100) == 0) {
      probed = true;
      break;
    }
    k_msleep(30);
  }

  if (!probed) {
    s_ble_supervisor_online = false;
    s_ble_module_awake = false;
    LOG_INF("ws_ble: BLE module probe timed out; supervisor UART wake path "
            "unavailable; proceeding normally without BLE");
    return 0;
  }

  s_ble_supervisor_online = true;
  s_ble_module_awake = true;
  LOG_INF("[BLE] Supervisor detected and responsive on USART1");

  /*
   * Phase 3: Configure normal transparent mode (AT+WS=0)
   * WildSonic BLE supervisor: AT+WS=0 sets m_ws_mode = 0 (transparent data
   * bridge).
   */
  bool prov_ok = true;
  if (ble_comm_send_cmd_expect("AT+WS=0\r\n", "OK+Set:0", NULL, 0, 150) == 0) {
    LOG_INF("[BLE] Configured normal transparent mode (AT+WS=0)");
  } else {
    LOG_WRN("[BLE] Warning: AT+WS=0 returned unexpected response");
    prov_ok = false;
  }

  /*
   * Keep the BLE supervisor in its trusted Mode 1 light-sleep state. The module
   * is already provisioned: AT+SLPM=1 preserves UART RX wake-on-interrupt while
   * the alternate sleep mode is intentionally avoided to keep the wake path
   * unchanged and preserve the existing host protocol.
   */
  s_current_slpm = 1;

  /*
   * Phase 5: Configure advertised name, bond authentication, PIN, and initial
   * unlock
   */
  char name_cmd[64];
  snprintf(name_cmd, sizeof(name_cmd), "AT+NAME%s\r\n", adv_name);
  if (ble_comm_send_cmd_expect(name_cmd, "OK+Set:", NULL, 0, 150) == 0) {
    LOG_INF("[BLE] Configured advertised name: %s", adv_name);
  } else {
    LOG_WRN("[BLE] Warning: AT+NAME failed");
    prov_ok = false;
  }

  /* Configure bond authentication: AT+TYPE3 */
  if (ble_comm_send_cmd_expect("AT+TYPE3\r\n", "OK+Set:3", NULL, 0, 150) != 0) {
    LOG_WRN("[BLE] Warning: AT+TYPE3 failed");
    prov_ok = false;
  }

  /* Configure pairing PIN: AT+PASS123456 (do not log PIN) */
  if (ble_comm_send_cmd_expect("AT+PASS123456\r\n", "OK+Set:", NULL, 0, 150) !=
      0) {
    LOG_WRN("[BLE] Warning: AT+PASS failed");
    prov_ok = false;
  }

  /* Start in unlocked connectable state: AT+ADTY0 */
  if (ble_comm_send_cmd_expect("AT+ADTY0\r\n", "OK+Set:0", NULL, 0, 150) == 0) {
    s_current_adty = 0;
    s_pairing_window_active = false;
    s_pairing_window_end_ms = 0;
    s_lock_pending_on_disconnect = false;
    LOG_INF("[BLE] Access: UNLOCKED (initial boot state)");
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[BLE] Access: UNLOCKED (initial boot state)\n");
#endif
  } else {
    LOG_WRN("[BLE] Warning: initial unlock (AT+ADTY0) failed");
    prov_ok = false;
  }

  /*
   * Phase 6: Put module in Mode 1 low-power sleep
   */
  if (ble_comm_send_cmd_expect("AT+SLEEP\r\n", "OK+SLEEP", NULL, 0, 100) == 0) {
    s_ble_module_awake = false;
  } else {
    LOG_WRN("[BLE] Warning: AT+SLEEP failed");
    prov_ok = false;
  }

  /* Commit fingerprint only upon 100% successful provisioning */
  if (prov_ok) {
    ble_retention_save_fingerprint(target_fp);
    LOG_INF("[BLE] Provisioning successfully persisted (fp=0x%08X)", target_fp);
  }

  return 0;
}

static int ble_comm_wake_module(void) {
  if (s_ble_module_awake) {
    return 0;
  }
  for (int i = 0; i < 3; i++) {
    if (ble_comm_send_cmd_expect("AT\r\n", "OK", NULL, 0, 100) == 0) {
      s_ble_module_awake = true;
      return 0;
    }
    k_msleep(20);
  }
  return -ETIMEDOUT;
}

int ble_comm_build_advertised_name(const char *dname, const char *sn, char *out,
                                   size_t max_len) {
  if (!out || max_len == 0) {
    return -EINVAL;
  }

  const char *p_name = dname;
  const char *p_sn = sn;
  size_t name_len = p_name ? strlen(p_name) : 0;
  size_t sn_len = p_sn ? strlen(p_sn) : 0;

  bool name_valid = (name_len > 0 && name_len <= 6);
  for (size_t i = 0; name_valid && i < name_len; i++) {
    if (!isalnum((unsigned char)p_name[i])) {
      name_valid = false;
    }
  }

  bool sn_valid = (sn_len > 0 && sn_len <= 3);
  for (size_t i = 0; sn_valid && i < sn_len; i++) {
    if (!isalnum((unsigned char)p_sn[i])) {
      sn_valid = false;
    }
  }

  const char *use_name = name_valid ? p_name : "XXX";
  const char *use_sn = sn_valid ? p_sn : "00";

  char temp[32];
  int written = snprintf(temp, sizeof(temp), "WS:%s-%s", use_name, use_sn);
  if (written < 0 || (size_t)written > 12) {
    return -EINVAL;
  }
  if ((size_t)written >= max_len) {
    return -EINVAL;
  }

  memcpy(out, temp, (size_t)written + 1);
  return 0;
}

void ble_comm_apply_advertised_name(void) {
  if (!s_ble_supervisor_online || !s_ble_uart_ready) {
    return;
  }
  if (s_ble_connected) {
    s_name_update_pending = true;
    LOG_INF(
        "[BLE] Central connected: deferring AT+NAME update until disconnect");
    return;
  }

  struct wildsonic_config *cfg = console_get_config();
  char adv_name[16];
  if (ble_comm_build_advertised_name(cfg ? cfg->device_name : NULL,
                                     cfg ? cfg->serial_number : NULL, adv_name,
                                     sizeof(adv_name)) != 0) {
    LOG_ERR("[BLE] Failed to build advertised name (exceeds max length)");
    return;
  }

  bool was_awake = s_ble_module_awake;
  if (!s_ble_module_awake) {
    if (ble_comm_wake_module() != 0) {
      LOG_ERR("[BLE] Failed to wake module for AT+NAME update");
      return;
    }
  }

  char cmd[48];
  snprintf(cmd, sizeof(cmd), "AT+NAME%s\r\n", adv_name);
  if (ble_comm_send_cmd_expect(cmd, "OK+Set:", NULL, 0, 150) == 0) {
    LOG_INF("[BLE] Advertised name updated to: %s", adv_name);
    uint32_t new_fp = ble_comm_calc_fingerprint(adv_name, "123456", 0, 1, 0, 3);
    ble_retention_save_fingerprint(new_fp);
  } else {
    LOG_WRN("[BLE] AT+NAME command failed");
  }

  if (!was_awake) {
    ble_comm_send_cmd_expect("AT+SLEEP\r\n", "OK+SLEEP", NULL, 0, 100);
    s_ble_module_awake = false;
  }
  s_name_update_pending = false;
}

void ble_comm_notify_name_changed(void) {
  if (s_ble_connected) {
    s_name_update_pending = true;
    LOG_INF("[BLE] Device name/SN updated while connected; AT+NAME deferred");
  } else {
    ble_comm_apply_advertised_name();
  }
}

bool ble_comm_is_uart_ready(void) { return s_ble_uart_ready; }

bool ble_comm_is_supervisor_online(void) { return s_ble_supervisor_online; }

bool ble_comm_is_active(void) { return s_ble_supervisor_online; }

bool ble_comm_is_connected(void) { return s_ble_connected; }

bool ble_comm_is_module_awake(void) { return s_ble_module_awake; }

bool ble_comm_is_pairing_window_active(void) {
  if (s_current_adty != 0) {
    return false;
  }
  if (s_pairing_window_active) {
    int64_t now = k_uptime_get();
    if (now >= s_pairing_window_end_ms) {
      return false;
    }
  }
  return true;
}

uint32_t ble_comm_get_pairing_window_remaining(void) {
  if (!s_pairing_window_active) {
    return 0;
  }
  int64_t rem_ms = s_pairing_window_end_ms - k_uptime_get();
  if (rem_ms <= 0) {
    return 0;
  }
  return (uint32_t)(rem_ms / 1000);
}

int ble_comm_query_sleep_policy(uint8_t *mode) {
  char resp[32] = {0};
  int rc = ble_comm_send_cmd_expect("AT+SLPM?\r\n", "OK+SLPM:", resp,
                                    sizeof(resp), 150);
  if (rc == 0) {
    char *colon = strchr(resp, ':');
    if (colon && colon[1] >= '1' && colon[1] <= '3') {
      if (mode)
        *mode = (uint8_t)(colon[1] - '0');
      s_current_slpm = (uint8_t)(colon[1] - '0');
      return 0;
    }
  }
  return rc;
}

int ble_comm_set_sleep_policy(uint8_t mode) {
  if (mode < 1 || mode > 3)
    return -EINVAL;
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+SLPM=%u\r\n", mode);
  char expected[32];
  snprintf(expected, sizeof(expected), "OK+Set:%u", mode);
  int rc = ble_comm_send_cmd_expect(cmd, expected, NULL, 0, 150);
  if (rc == 0) {
    s_current_slpm = mode;
  }
  return rc;
}

int ble_comm_query_adv_type(uint8_t *adty) {
  char resp[32] = {0};
  int rc = ble_comm_send_cmd_expect("AT+ADTY?\r\n", "OK+Get:", resp,
                                    sizeof(resp), 150);
  if (rc == 0) {
    char *colon = strchr(resp, ':');
    if (colon && colon[1] >= '0' && colon[1] <= '3') {
      if (adty)
        *adty = (uint8_t)(colon[1] - '0');
      s_current_adty = (uint8_t)(colon[1] - '0');
      return 0;
    }
  }
  return rc;
}

void ble_comm_open_pairing_window(uint32_t seconds) {
  if (seconds == 0) {
    seconds = BLE_PAIR_WINDOW_DEFAULT_SECONDS;
  }
  if (seconds > BLE_PAIR_WINDOW_MAX_SECONDS) {
    seconds = BLE_PAIR_WINDOW_MAX_SECONDS;
  }

  if (!s_ble_supervisor_online) {
    LOG_WRN("BLE supervisor offline; cannot open pairing window");
    return;
  }

  if (s_ble_connected) {
    LOG_INF("[BLE] Device already connected; extending active session");
    s_pairing_window_active = true;
    s_pairing_window_end_ms = k_uptime_get() + ((int64_t)seconds * 1000);
    s_lock_pending_on_disconnect = false;
    return;
  }

  /* If module is asleep, wake it with AT ping; abort if wake fails */
  if (ble_comm_wake_module() != 0) {
    LOG_ERR("BLE wake failed; aborting pairing unlock");
    printk("\n[ERROR] Failed to wake BLE module\n>");
    return;
  }

  /* Transmit AT+ADTY0\r\n and validate exact OK+Set:0\r\n */
  int rc = ble_comm_send_cmd_expect("AT+ADTY0\r\n", "OK+Set:0", NULL, 0, 150);
  if (rc == 0) {
    s_current_adty = 0;
    s_pairing_window_active = true;
    s_pairing_window_end_ms = k_uptime_get() + ((int64_t)seconds * 1000);
    s_lock_pending_on_disconnect = false;
    LOG_INF("BLE Access: UNLOCKED (%u s)", (unsigned int)seconds);
    printk("\nBLE Access: UNLOCKED (%u s)\n>", (unsigned int)seconds);
  } else {
    LOG_ERR("BLE unlock failed: no OK+Set:0 response (rc=%d)", rc);
    printk(
        "\n[ERROR] Failed to unlock BLE pairing (supervisor unresponsive)\n>");
  }
}

void ble_comm_lock_pairing(void) {
  if (!s_ble_supervisor_online) {
    s_pairing_window_active = false;
    s_pairing_window_end_ms = 0;
    return;
  }

  /* If a phone is currently connected, preserve active connection and defer
   * lock until disconnect */
  if (s_ble_connected) {
    s_lock_pending_on_disconnect = true;
    LOG_INF("[BLE] Connection active; deferring pairing lock until disconnect");
    return;
  }

  /* If module is asleep, wake it with AT ping; abort if wake fails */
  if (ble_comm_wake_module() != 0) {
    LOG_ERR("BLE wake failed; aborting pairing lock");
    return;
  }

  /* Transmit AT+ADTY2\r\n and validate exact OK+Set:2\r\n */
  int rc = ble_comm_send_cmd_expect("AT+ADTY2\r\n", "OK+Set:2", NULL, 0, 150);
  if (rc == 0) {
    s_current_adty = 2;
    s_pairing_window_active = false;
    s_pairing_window_end_ms = 0;
    s_lock_pending_on_disconnect = false;
    LOG_INF("BLE Access: LOCKED");
    printk("\nBLE Access: LOCKED\n>");

    /* Enter low-power sleep under Mode 1 */
    ble_comm_send_cmd_expect("AT+SLEEP\r\n", "OK+SLEEP", NULL, 0, 100);
    s_ble_module_awake = false;
  } else {
    LOG_ERR("BLE lock failed: no OK+Set:2 response (rc=%d)", rc);
  }
}

void ble_comm_suspend(void) {
  if (!s_ble_supervisor_online)
    return;

  /* If disconnected and module is awake, command it to sleep under Mode 1 */
  if (!s_ble_connected && s_ble_module_awake) {
    ble_comm_send_cmd_expect("AT+SLEEP\r\n", "OK+SLEEP", NULL, 0, 100);
    s_ble_module_awake = false;
  }

  if (s_ble_uart_dev) {
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
    uart_irq_rx_disable(s_ble_uart_dev);
#endif
  }

#if defined(CONFIG_SOC_SERIES_STM32L4X)
  /* Drain USART1 TX buffer before Stop 2 */
  if ((USART1->CR1 & USART_CR1_UE) && (USART1->CR1 & USART_CR1_TE)) {
    uint32_t start_ms = k_uptime_get_32();
    while (!LL_USART_IsActiveFlag_TC(USART1)) {
      if ((uint32_t)(k_uptime_get_32() - start_ms) >= 200U) {
        break;
      }
      k_busy_wait(10);
    }
  }

  LL_USART_DisableInStopMode(USART1);
  LL_USART_DisableIT_WKUP(USART1);
  LL_USART_ClearFlag_WKUP(USART1);
  LL_USART_ClearFlag_ORE(USART1);
  LL_USART_ClearFlag_FE(USART1);
  LL_USART_ClearFlag_NE(USART1);
  LL_USART_ClearFlag_PE(USART1);
  LL_USART_ClearFlag_IDLE(USART1);
  if (LL_USART_IsActiveFlag_RXNE(USART1)) {
    (void)USART1->RDR;
  }
#endif
}

void ble_comm_resume(void) {
  if (!s_ble_supervisor_online || !s_ble_uart_dev || !s_ble_uart_ready)
    return;

#if defined(CONFIG_SOC_SERIES_STM32L4X)
  LL_USART_ClearFlag_ORE(USART1);
  LL_USART_ClearFlag_FE(USART1);
  LL_USART_ClearFlag_NE(USART1);
  LL_USART_ClearFlag_PE(USART1);
  LL_USART_ClearFlag_IDLE(USART1);
  LL_USART_ClearFlag_WKUP(USART1);
  if (LL_USART_IsActiveFlag_RXNE(USART1)) {
    (void)USART1->RDR;
  }
#endif

  struct uart_config bcfg = {
    .baudrate = 115200,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
  };
  (void)uart_configure(s_ble_uart_dev, &bcfg);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
  uart_irq_callback_user_data_set(s_ble_uart_dev, ble_uart_isr, NULL);
  uart_irq_rx_enable(s_ble_uart_dev);
#endif

  /*
   * Stop 2 resume requirement: Do NOT send AT pings or commands to BLE
   * supervisor! The supervisor maintains its own Mode 1 low-power connectable
   * advertising.
   */
}

void ble_comm_send_dev_info(void) {
  if (k_mutex_lock(&s_ble_report_mutex, K_MSEC(2000)) != 0) {
    return;
  }

  struct wildsonic_config *cfg = console_get_config();
  struct app_settings *settings = main_get_settings();
  uint32_t sleep_sec = cfg->sleep_pause_interval
                           ? cfg->sleep_pause_interval
                           : (settings ? settings->sleep_seconds : 60);

  /* 1. Audio config status */
  ble_comm_printf("SAMF %u\r\n", cfg->sampling_rate);
  ble_comm_printf("BTPS %u\r\n", cfg->audio_sampling_width);
  ble_comm_printf("NUMCH %u\r\n", cfg->num_audio_channels);
  ble_comm_printf("GAIN %d\r\n", cfg->mic_gain_times_ten);
  ble_comm_printf("LCUTOFF %u\r\n", cfg->lower_cutoff_frequency);
  ble_comm_printf("UCUTOFF %u\r\n", cfg->upper_cutoff_frequency);
  ble_comm_printf("RECT %u\r\n", cfg->recording_length);
  ble_comm_printf("SLEEP %u\r\n", sleep_sec);

  /* 2. Version info */
  ble_comm_printf("VER 1.2.0, SHA d7a4f91, DT %s %s\r\n", __DATE__, __TIME__);

  /* 3. Storage status */
  uint64_t sd_used = 0, sd_total = 0;
  struct fs_statvfs stat;
  if (fs_statvfs("/SD:", &stat) == 0) {
    sd_total = (uint64_t)stat.f_blocks * (uint64_t)stat.f_frsize;
    uint64_t free_b = (uint64_t)stat.f_bfree * (uint64_t)stat.f_frsize;
    sd_used = (sd_total > free_b) ? (sd_total - free_b) : 0;
  }
  ble_comm_printf("SDUSED %llu, SDCAP %llu\r\n", (unsigned long long)sd_used,
                  (unsigned long long)sd_total);

  /* 4. Folder structure */
  const char *fldr_str = (cfg->folder_structure == 1) ? "daily" :
                         ((cfg->folder_structure == 2) ? "hourly" : "flat");
  ble_comm_printf("FLDR %s\r\n", fldr_str);

  /* 5. Power management status */
  ble_comm_printf("VBATT N/A, CBATT N/A, REMBATT N/A\r\n");
  ble_comm_printf("FULLBATT N/A, REMTIME N/A, AVGPWR N/A\r\n");
  ble_comm_printf("NBAT 1\r\n");

  /* 6. Time and schedule status with local timezone offset */
  struct rtc_calendar_time now_cal;
  if (rtc_calendar_get(&now_cal) == 0 && rtc_calendar_is_valid()) {
    time_t cur_utc = scheduler_calendar_to_epoch(&now_cal);
    time_t cur_loc = cur_utc + cfg->time_zone_offset;
    struct rtc_calendar_time loc_cal;
    scheduler_epoch_to_calendar(cur_loc, &loc_cal);

    char tz_sign = '+';
    int32_t tz_abs = cfg->time_zone_offset;
    if (tz_abs < 0) {
      tz_sign = '-';
      tz_abs = -tz_abs;
    }
    uint16_t tz_h = (uint16_t)(tz_abs / 3600);
    uint16_t tz_m = (uint16_t)((tz_abs % 3600) / 60);

    ble_comm_printf("DATETIME %04d-%02d-%02d %02d:%02d:%02d.%03d %c%02d:%02d\r\n",
                    loc_cal.year, loc_cal.month, loc_cal.day,
                    loc_cal.hour, loc_cal.min, loc_cal.sec,
                    now_cal.subsec,
                    tz_sign, tz_h, tz_m);
    ble_comm_printf("TIMEZONE %c%02d:%02d\r\n", tz_sign, tz_h, tz_m);
  } else {
    ble_comm_printf("DATETIME N/A\r\n");
    ble_comm_printf("TIMEZONE +00:00\r\n");
  }

  /* 7. Schedule date range */
  struct schedule_config scfg;
  scheduler_get_config(&scfg);
  ble_comm_printf("DATERANGE %04u/%02u/%02u-%04u/%02u/%02u\r\n",
                  scfg.start_date.year ? scfg.start_date.year : 2026,
                  scfg.start_date.month ? scfg.start_date.month : 1,
                  scfg.start_date.day ? scfg.start_date.day : 1,
                  scfg.end_date.year ? scfg.end_date.year : 2099,
                  scfg.end_date.month ? scfg.end_date.month : 12,
                  scfg.end_date.day ? scfg.end_date.day : 31);

  /* 8. Sun mode */
  if (scfg.sun_recording_mode != SUN_MODE_DISABLED) {
    if (scfg.sun_recording_mode == SUN_MODE_SUNRISE) {
      ble_comm_printf("SUNMODE Sunrise\r\n");
    } else if (scfg.sun_recording_mode == SUN_MODE_SUNSET) {
      ble_comm_printf("SUNMODE Sunset\r\n");
    } else if (scfg.sun_recording_mode == SUN_MODE_SUNRISE_AND_SUNSET) {
      ble_comm_printf("SUNMODE Sunrise and Sunset\r\n");
    } else if (scfg.sun_recording_mode == SUN_MODE_SUNRISE_TO_SUNSET) {
      ble_comm_printf("SUNMODE Sunrise to Sunset\r\n");
    } else if (scfg.sun_recording_mode == SUN_MODE_SUNSET_TO_SUNRISE) {
      ble_comm_printf("SUNMODE Sunset to Sunrise\r\n");
    }
    k_sleep(K_MSEC(10));
    ble_comm_printf("SUNRISEAFT %u\r\n", scfg.sunrise_interval_after);
    k_sleep(K_MSEC(10));
    ble_comm_printf("SUNSETBEF %u\r\n", scfg.sunset_interval_before);
    k_sleep(K_MSEC(10));
    ble_comm_printf("SUNSETAFT %u\r\n", scfg.sunset_interval_after);
    k_sleep(K_MSEC(10));
    ble_comm_printf("SUNROUND %u\r\n", scfg.sunrise_sunset_rounding);
    k_sleep(K_MSEC(10));
  } else {
    ble_comm_printf("SUNMODE Disabled\r\n");
    k_sleep(K_MSEC(10));
  }

  /* 9. Schedule time of day ranges (indices 0..9) */
  for (int i = 0; i < 10; i++) {
    uint32_t start_sec = scfg.tod_ranges[i].start_second;
    uint32_t end_sec = scfg.tod_ranges[i].end_second;
    ble_comm_printf("TIMEOFDAY %d %02d:%02d:%02d-%02d:%02d:%02d\r\n",
                    i,
                    start_sec / 3600, (start_sec / 60) % 60, start_sec % 60,
                    end_sec / 3600, (end_sec / 60) % 60, end_sec % 60);
  }

  /* 10. Days of week */
  char days_str[8] = {0};
  int d_idx = 0;
  uint8_t dow_mask = scfg.day_of_week_mask ? scfg.day_of_week_mask : 0x7F;
  if (dow_mask & 0x01) days_str[d_idx++] = 'M';
  if (dow_mask & 0x02) days_str[d_idx++] = 'T';
  if (dow_mask & 0x04) days_str[d_idx++] = 'W';
  if (dow_mask & 0x08) days_str[d_idx++] = 'R';
  if (dow_mask & 0x10) days_str[d_idx++] = 'F';
  if (dow_mask & 0x20) days_str[d_idx++] = 'S';
  if (dow_mask & 0x40) days_str[d_idx++] = 'U';
  days_str[d_idx] = '\0';
  ble_comm_printf("DAYSOFWEEK %s\r\n", days_str);

  /* 11. Legacy compatibility tags for test suites and mobile ID */
  ble_comm_printf("MODEL WS-STM32L476\r\n");
  ble_comm_printf("FWVER 1.2.0-BLE\r\n");
  ble_comm_printf("MIC ANALOG\r\n");
  ble_comm_printf("BATTCAP 0\r\n");

  /* 12. Environmental sensor status & report termination */
  ble_comm_printf("TEMP 0.00, HUMID 0.00\r\n");

  k_mutex_unlock(&s_ble_report_mutex);
}

void ble_comm_send_status_info(void) {
  if (k_mutex_lock(&s_ble_report_mutex, K_MSEC(2000)) != 0) {
    return;
  }

  uint64_t sd_used = 0, sd_total = 0;
  struct fs_statvfs stat;
  if (fs_statvfs("/SD:", &stat) == 0) {
    sd_total = (uint64_t)stat.f_blocks * (uint64_t)stat.f_frsize;
    uint64_t free_b = (uint64_t)stat.f_bfree * (uint64_t)stat.f_frsize;
    sd_used = (sd_total > free_b) ? (sd_total - free_b) : 0;
  }
  ble_comm_printf("SDUSED %llu, SDCAP %llu\r\n", (unsigned long long)sd_used,
                  (unsigned long long)sd_total);


  ble_comm_printf("VBATT N/A, CBATT N/A, REMBATT N/A\r\n");

  ble_comm_printf("FULLBATT N/A, REMTIME N/A, AVGPWR N/A\r\n");


  /* Terminating line */
  ble_comm_printf("TEMP 0.00, HUMID 0.00\r\n");

  k_mutex_unlock(&s_ble_report_mutex);
}

int ble_comm_feed_rx_bytes(const uint8_t *data, size_t len) {
  if (!data || len == 0)
    return 0;
  for (size_t i = 0; i < len; i++) {
    ble_comm_handle_rx_byte((char)data[i]);
  }
  return (int)len;
}

int ble_comm_drain_tx_bytes(uint8_t *buf, size_t max_len) {
  if (!buf || max_len == 0)
    return 0;
  return (int)ring_buf_get(&s_ble_tx_rb, buf, max_len);
}

void ble_comm_set_test_capture(bool enabled) {
  s_ble_test_capture_enabled = enabled;
}

void ble_comm_reset_test_state(void) {
  ring_buf_reset(&s_ble_rx_rb);
  ring_buf_reset(&s_ble_tx_rb);
  s_ble_uart_ready = false;
  s_ble_supervisor_online = false;
  s_ble_connected = false;
  s_ble_module_awake = false;
  s_pairing_window_active = false;
  s_pairing_window_end_ms = 0;
  s_lock_pending_on_disconnect = false;
  s_ble_cmd_len = 0;
  memset(s_ble_cmd_buf, 0, sizeof(s_ble_cmd_buf));
  s_ble_discard_until_terminator = false;
  s_ble_last_was_cr = false;
  s_ble_overflow_line_count = 0;
  s_ble_rx_drop_count = 0;
  s_expected_ack = NULL;
  s_ack_matched = false;
  s_current_slpm = 1;
  s_current_adty = 0;
  s_name_update_pending = false;
  s_ble_test_capture_enabled = false;
#if defined(CONFIG_AUDIO_DIAGNOSTIC_MODE) || defined(CONFIG_AUDIO_STAGE_DIAGNOSTICS)
  memset(s_ble_overflow_diag, 0, sizeof(s_ble_overflow_diag));
  s_ble_overflow_diag_len = 0;
#endif
}
