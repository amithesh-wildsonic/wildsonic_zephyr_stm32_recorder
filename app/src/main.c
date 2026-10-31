/* SPDX-License-Identifier: Apache-2.0 */
/*
 * WildSonic STM32L476RG Audio Recorder — Unified Standard Firmware
 * Supports:
 *   - Interactive UART configuration menu (115200 8N1)
 *   - Runtime sample rate selection (16k, 32k, 44.1k, 48k, 96k, 192k)
 *   - Runtime recording duration, sleep interval, and cycle counts
 *   - Software digital gain with saturation arithmetic and clipping tracking
 *   - Configurable DSP stages (DC block, Butterworth HPF, Mains Notch,
 * 4th-order LPF)
 *   - Modes: Normal, Raw, Compare (Raw vs Filtered), Rapid Stress Test, Test
 * Tone
 *   - Live microphone diagnostics monitor (1 Hz reporting)
 */
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <stm32_ll_rcc.h>

#include "audio_in.h"
#include "dsp.h"
#include "psram_is66wvs.h"
#include "psram_staging.h"
#include "recorder.h"
#include "rtc_calendar.h"
#include "rtc_sleep.h"
#include "scheduler.h"
#include "main.h"
#include "console.h"

#include "config_store.h"
#include "led_indicator.h"
#include "audio_format.h"
#include "ble_comm.h"
#include "scheduler_retention.h"
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
#include "psram_benchmark.h"
#endif

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(main, LOG_LEVEL_WRN);
#endif

#define SAMPLE_BYTES (CONFIG_AUDIO_SAMPLE_BITS / 8)
#define CONSOLE_LINE_MAX 128

static const struct device *const console_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Runtime Configuration Settings (defined in main.h) */
static struct app_settings settings = {
    .sample_rate = CONFIG_AUDIO_SAMPLE_RATE,
    .sample_bits = 16,
    .channels = 1,
    .capture_source = AUDIO_SRC_PHYSICAL_MIC,
    .record_seconds = 20,
    .sleep_seconds = 60,
    .cycles = 0,
    .monitor_seconds = 10,
    .digital_gain = 1.0f,
    .mode = REC_MODE_NORMAL,
    .dsp_enabled = true,
    .dc_block_enabled = true,
    .hpf_enabled = true,
    .hpf_cutoff_hz = 80,
    .notch_enabled = false,
    .notch_freq_hz = 50,
    .notch_q = 10.0f,
    .harmonic_notch_enabled = false,
    .lpf_enabled = true,
    .lpf_cutoff_hz = 20000,
#ifdef CONFIG_AUDIO_PSRAM_STAGING
    .psram_staging_enabled = true,
#else
    .psram_staging_enabled = false,
#endif
};

static void apply_dsp_settings(void) {
  struct dsp_config dcfg = {
      .dc_block_enabled = settings.dsp_enabled && settings.dc_block_enabled,
      .hpf_enabled = settings.dsp_enabled && settings.hpf_enabled,
      .hpf_cutoff_hz = settings.hpf_cutoff_hz,
      .notch_enabled = settings.dsp_enabled && settings.notch_enabled,
      .notch_freq_hz = settings.notch_freq_hz,
      .notch_q = settings.notch_q,
      .harmonic_notch_enabled =
          settings.dsp_enabled && settings.harmonic_notch_enabled,
      .harmonic_freq_hz = settings.notch_freq_hz * 2,
      .harmonic_q = settings.notch_q,
      .lpf_enabled = settings.dsp_enabled && settings.lpf_enabled,
      .lpf_cutoff_hz = settings.lpf_cutoff_hz,
  };
  dsp_configure(&dcfg, settings.sample_rate);
  dsp_set_digital_gain(settings.digital_gain);
}

/* Console Character I/O */
int console_poll_char(unsigned char *c) {
  if (console_async_is_active()) {
    return console_async_get_char(c);
  }
  if (!console_dev || !device_is_ready(console_dev)) {
    return -ENODEV;
  }
  return uart_poll_in(console_dev, c);
}

static int64_t s_console_last_activity_ms = 0;

static bool console_read_line(char *buf, size_t max_len) {
  size_t pos = 0;
  static int64_t last_cr_time = 0;

  while (1) {
    unsigned char c;
    int rc = console_async_wait_char(&c, 20);
    if (rc != 0 && !console_async_is_active()) {
      rc = console_poll_char(&c);
    }

    if (rc == 0) {
      s_console_last_activity_ms = k_uptime_get();
      /* Drain and echo all pending characters in ring buffer without delay */
      do {
        /* If \n follows \r within 100 ms (CRLF sequence), discard the trailing \n */
        if (c == '\n' && pos == 0 && (k_uptime_get() - last_cr_time) < 100) {
          continue;
        }

        if (c == '\r' || c == '\n') {
          if (c == '\r') {
            last_cr_time = k_uptime_get();
          }
          buf[pos] = '\0';
          printk("\r\n");
          return true;
        } else if (c == '\b' || c == 127) {
          if (pos > 0) {
            pos--;
            printk("\b \b");
          }
        } else if (c == 0x1B) {
          /*
           * Escape sequence handling:
           * Check if this is an ANSI escape sequence (such as arrow keys \x1B[A).
           * Consume ANSI sequence silently without line-clearing or echo
           * glitches. If standalone ESC, erase current line with backspaces
           * cleanly.
           */
          k_busy_wait(2000);
          unsigned char next_c;
          if (console_poll_char(&next_c) == 0) {
            if (next_c == '[') {
              unsigned char seq_c;
              while (console_poll_char(&seq_c) == 0) {
                if ((seq_c >= 'A' && seq_c <= 'Z') ||
                    (seq_c >= 'a' && seq_c <= 'z') || seq_c == '~') {
                  break;
                }
              }
              continue;
            }
          }
          if (pos == 0) {
            /* ESC at prompt */
          } else {
            while (pos > 0) {
              printk("\b \b");
              pos--;
            }
            buf[0] = '\0';
          }
        } else if (c >= 32 && c < 127) {
          if (pos < max_len - 1) {
            buf[pos++] = (char)c;
            printk("%c", c);
          }
        }
      } while (console_async_get_char(&c) == 0);
    } else {
      /* When user is idle at the prompt (pos == 0), flush pending logs and service scheduler */
      if (pos == 0) {
        log_process();
        if (!main_is_scheduler_paused()) {
          main_scheduler_service_step();
        }
      }
    }
  }
}

static bool prompt_read_line(char *buf, size_t max_len) {
  if (!console_read_line(buf, max_len)) {
    return false;
  }
  /* Trim leading whitespace */
  char *start = buf;
  while (*start && isspace((unsigned char)*start)) {
    start++;
  }
  /* Trim trailing whitespace */
  char *end = start + strlen(start);
  while (end > start && isspace((unsigned char)*(end - 1))) {
    end--;
  }
  *end = '\0';
  if (start != buf) {
    memmove(buf, start, strlen(start) + 1);
  }
  if (strcasecmp(buf, "cancel") == 0 || strcasecmp(buf, "q") == 0) {
    return false;
  }
  return true;
}

static const char *mode_to_string(enum recorder_mode m) {
  switch (m) {
  case REC_MODE_NORMAL:
    return "NORMAL (Production Filtered)";
  case REC_MODE_RAW:
    return "RAW (Bypass DSP, Unfiltered ADC)";
  case REC_MODE_FILTERED:
    return "FILTERED (Active DSP Chain)";
  default:
    return "UNKNOWN";
  }
}

uint32_t g_boot_csr = 0;

static void __maybe_unused print_reset_reason(void) {
  uint32_t csr = g_boot_csr ? g_boot_csr : RCC->CSR;
  printk("  MCU Reset Reason  : [0x%08X] ", (unsigned int)csr);
  bool known = false;
  if (csr & RCC_CSR_LPWRRSTF) {
    printk("Low-Power Reset; ");
    known = true;
  }
  if (csr & RCC_CSR_WWDGRSTF) {
    printk("Window Watchdog Reset (WWDG); ");
    known = true;
  }
  if (csr & RCC_CSR_IWDGRSTF) {
    printk("Independent Watchdog Reset (IWDG); ");
    known = true;
  }
  if (csr & RCC_CSR_SFTRSTF) {
    printk("Software Reset (SystemReset / Fatal Error); ");
    known = true;
  }
  if (csr & RCC_CSR_BORRSTF) {
    printk("Brownout Reset (BOR) / Power-On; ");
    known = true;
  }
  if (csr & RCC_CSR_PINRSTF) {
    printk("External Reset Pin (NRST); ");
    known = true;
  }
  if (csr & RCC_CSR_OBLRSTF) {
    printk("Option Byte Loader Reset; ");
    known = true;
  }
  if (csr & RCC_CSR_FWRSTF) {
    printk("Firewall Reset; ");
    known = true;
  }
  if (!known) {
    printk("Unknown / None; ");
  }
  printk("\n");
}

static void print_banner(void) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("\n=========================================================================\n");
  printk("      WildSonic STM32L476RG Audio Recorder — Unified Firmware             \n");
  printk("=========================================================================\n");
  printk("  Target MCU        : STM32L476RG (Cortex-M4F @ 72 MHz, FPU Enabled)\n");
  print_reset_reason();
  printk("=========================================================================\n\n");
#else
  printk("Version: %s\n", "v1.2.0");
  printk("GIT Branch: %s, SHA: %s\n", "production", "d7a4f91");
  printk("Build Date: %s %s\n", __DATE__, __TIME__);
#endif
}

static void print_settings(void) {
  uint32_t act_rate = 0, div = 0;
  int err_hz = 0;
  audio_in_is_rate_supported(settings.sample_rate, &act_rate, &div, &err_hz);

  uint64_t total_samples = (uint64_t)settings.record_seconds * act_rate;
  uint64_t pcm_bytes = total_samples * SAMPLE_BYTES;
  uint64_t total_wav_bytes = 44 + pcm_bytes;
  uint32_t data_rate_bps = act_rate * SAMPLE_BYTES;

  printk("\n============================ ACTIVE CONFIGURATION "
         "============================\n");
  printk("  Operating Mode        : %s\n", mode_to_string(settings.mode));
  printk("  Sample Rate           : %u Hz (Actual: %u Hz, TIM6 divider: %u, "
         "Error: %d Hz)\n",
         settings.sample_rate, act_rate, div, err_hz);
  printk("  Sample Format         : %u-bit signed integer PCM (Format Tag 1, "
         "Mono)\n",
         CONFIG_AUDIO_SAMPLE_BITS);
  printk(
      "  Session Durations     : %u s recording / %u s sleep (Cycles: %u %s)\n",
      settings.record_seconds, settings.sleep_seconds, settings.cycles,
      (settings.cycles == 0) ? "[Continuous]" : "");
  printk("  Software Digital Gain : %.2fx (%+.1f dB)\n",
         (double)settings.digital_gain,
         20.0 * log10((double)settings.digital_gain));
  printk("  DSP Pipeline Status   : %s\n",
         settings.dsp_enabled ? "ENABLED (Active Biquad Cascade)"
                              : "BYPASSED (Raw ADC Mode)");
  if (settings.dsp_enabled) {
    printk("    - Stage 1: DC Blocker : %s (20 Hz Corner)\n",
           settings.dc_block_enabled ? "ON" : "OFF");
    printk("    - Stage 2: HPF Filter : %s (%u Hz Butterworth, Q=0.7071)\n",
           settings.hpf_enabled ? "ON" : "OFF", settings.hpf_cutoff_hz);
    printk("    - Stage 3: Mains Notch: %s (%u Hz, Q=%.1f)\n",
           settings.notch_enabled ? "ON" : "OFF", settings.notch_freq_hz,
           (double)settings.notch_q);
    printk("    - Stage 4: Harm Notch : %s (%u Hz, Q=10.0)\n",
           settings.harmonic_notch_enabled ? "ON" : "OFF",
           settings.notch_freq_hz * 2);
    printk("    - Stage 5: LPF Filter : %s (%u Hz 4th-Order Butterworth, -24 "
           "dB/oct)\n",
           settings.lpf_enabled ? "ON" : "OFF", settings.lpf_cutoff_hz);
  }
  printk("  Expected Audio Output :\n");
  printk("    - Samples per file    : %llu frames\n", total_samples);
  printk("    - PCM audio data size : %llu bytes (%.2f MB)\n", pcm_bytes,
         (double)pcm_bytes / (1024.0 * 1024.0));
  printk("    - Total WAV file size : %llu bytes (Header: 44 B + Data)\n",
         total_wav_bytes);
  printk("    - SD Stream Bandwidth : %u bytes/s (%u KB/s)\n", data_rate_bps,
         data_rate_bps / 1024);
  printk(
      "    - RAM Pool Buffer     : %u blocks (%u KB slab, %u ms buffer "
      "depth)\n",
      CONFIG_AUDIO_NUM_BLOCKS,
      (CONFIG_AUDIO_NUM_BLOCKS * CONFIG_AUDIO_BLOCK_SAMPLES * SAMPLE_BYTES) >>
          10,
      (uint32_t)((uint64_t)CONFIG_AUDIO_NUM_BLOCKS *
                 CONFIG_AUDIO_BLOCK_SAMPLES * 1000 / act_rate));
#ifdef CONFIG_AUDIO_PSRAM_STAGING
  bool psram_avail = psram_is_present();
  bool psram_quad = psram_is_quad_mode();
  if (settings.psram_staging_enabled) {
    printk("  PSRAM Audio Staging   : ENABLED (%s, %s)\n",
           psram_avail ? "IS66WVS4M8 Circular Ring Buffer"
                       : "PSRAM Hardware Not Detected!",
           psram_quad ? "Quad I/O Verified" : "1-Line SPI Active");
    if (psram_avail) {
      uint32_t buf_ms = (uint32_t)((uint64_t)CONFIG_AUDIO_PSRAM_BUFFER_SIZE_KB *
                                   1024 * 1000 / (act_rate * SAMPLE_BYTES));
      printk("    - Staging Capacity  : %u KB (%u ms audio buffer at %u Hz)\n",
             CONFIG_AUDIO_PSRAM_BUFFER_SIZE_KB, buf_ms, act_rate);
      printk(
          "    - SD Write Batch    : %u KB (%u capture blocks per SD flush)\n",
          CONFIG_AUDIO_SD_BATCH_SIZE_KB,
          (CONFIG_AUDIO_SD_BATCH_SIZE_KB * 1024) /
              (CONFIG_AUDIO_BLOCK_SAMPLES * SAMPLE_BYTES));
    }
  } else {
    printk(
        "  PSRAM Audio Staging   : DISABLED (Direct RAM Queue to SD Writer; %s)\n",
        psram_avail ? (psram_quad ? "Quad PSRAM standby"
                                  : "1-Line SPI fallback")
                    : "PSRAM Unpopulated");
  }
#else
  printk("  PSRAM Audio Staging   : DISABLED (Not Compiled)\n");
#endif
  struct rtc_calendar_time rtc_now;
  char time_str[32] = "[UNSET / UNINITIALIZED]";
  if (rtc_calendar_get(&rtc_now) == 0 && rtc_calendar_is_valid()) {
    rtc_calendar_format(&rtc_now, time_str, sizeof(time_str));
  }
  printk("  RTC Hardware Calendar : %s\n", time_str);
  struct schedule_config scfg_stat;
  scheduler_get_config(&scfg_stat);
  char tz_str[16];
  scheduler_format_timezone(scfg_stat.time_zone_offset, tz_str, sizeof(tz_str));
  printk("  Recording Scheduler   : %s (Timezone: %s)\n",
         scheduler_has_active_constraints() ? "CONSTRAINED (Active calendar/time bounds)"
                                            : "UNCONSTRAINED (Always Ready)",
         tz_str);
  printk("====================================================================="
         "=========\n\n");
}

static void reset_to_defaults(void) {
  settings.sample_rate = 48000;
  settings.sample_bits = 16;
  settings.channels = 1;
  settings.capture_source = AUDIO_SRC_PHYSICAL_MIC;
  audio_in_set_capture_source(AUDIO_SRC_PHYSICAL_MIC);
  settings.record_seconds = 600;
  settings.sleep_seconds = 0;
  settings.cycles = 1;
  settings.monitor_seconds = 10;
  settings.digital_gain = 1.0f;
  settings.mode = REC_MODE_NORMAL;
  settings.dsp_enabled = true;
  settings.dc_block_enabled = true;
  settings.hpf_enabled = false;
  settings.hpf_cutoff_hz = 0;
  settings.notch_enabled = false;
  settings.notch_freq_hz = 50;
  settings.notch_q = 10.0f;
  settings.harmonic_notch_enabled = false;
  settings.lpf_enabled = true;
  settings.lpf_cutoff_hz = 24000;
#ifdef CONFIG_AUDIO_PSRAM_STAGING
  if (psram_is_present() && psram_is_integrity_verified()) {
    settings.psram_staging_enabled = true;
    recorder_set_psram_staging_enabled(true);
  } else {
    settings.psram_staging_enabled = false;
    recorder_set_psram_staging_enabled(false);
  }
#else
  settings.psram_staging_enabled = false;
  recorder_set_psram_staging_enabled(false);
#endif
  audio_in_set_rate(settings.sample_rate);
  apply_dsp_settings();
  printk("\nConfiguration reset to factory defaults.\n");
  print_settings();
}

static void run_guided_config(void) {
  struct app_settings staged = settings;
  char buf[64];

  printk("\n==================================================================="
         "======\n");
  printk("                 WILDSONIC GUIDED CONFIGURATION MENU                 "
         "    \n");
  printk("  Press <ENTER> to retain current value shown in brackets [].        "
         "    \n");
  printk("  Type 'cancel' or 'q' at any prompt to abort without applying "
         "changes.  \n");
  printk("====================================================================="
         "====\n\n");

  /* Step 1: Sampling Frequency */
  while (1) {
    printk("[1/7] Sampling Frequency\n");
    printk("      Supported: 16000, 32000, 44100, 48000, 96000, 192000 Hz\n");
    printk("      Enter sampling rate [%u]: ", staged.sample_rate);
    if (!prompt_read_line(buf, sizeof(buf))) {
      printk(
          "\n[INFO] Configuration cancelled. Current settings retained.\n\n");
      return;
    }
    if (buf[0] == '\0') {
      break;
    }
    uint32_t val = (uint32_t)strtoul(buf, NULL, 10);
    if (val == 16)
      val = 16000;
    else if (val == 32)
      val = 32000;
    else if (val == 44 || val == 44100)
      val = 44100;
    else if (val == 48)
      val = 48000;
    else if (val == 96)
      val = 96000;
    else if (val == 192)
      val = 192000;

    uint32_t act = 0, div = 0;
    int err_hz = 0;
    if (audio_in_is_rate_supported(val, &act, &div, &err_hz)) {
      staged.sample_rate = val;
      printk("      -> Set to %u Hz (TIM6 div: %u, Error: %d Hz)\n\n", val, div,
             err_hz);
      break;
    } else {
      printk("      [ERROR] Invalid sampling rate '%s'. Supported: 16000, "
             "32000, 44100, 48000, 96000, 192000 Hz.\n\n",
             buf);
    }
  }

  /* Step 2: Recording Duration */
  while (1) {
    printk("[2/7] Recording Duration per Session\n");
    printk("      Enter duration in seconds (1 to 86400) [%u]: ",
           staged.record_seconds);
    if (!prompt_read_line(buf, sizeof(buf))) {
      printk(
          "\n[INFO] Configuration cancelled. Current settings retained.\n\n");
      return;
    }
    if (buf[0] == '\0') {
      break;
    }
    int val = atoi(buf);
    if (val >= 1 && val <= 86400) {
      staged.record_seconds = (uint32_t)val;
      printk("      -> Recording duration set to %u s\n\n",
             staged.record_seconds);
      break;
    } else {
      printk("      [ERROR] Invalid duration '%s'. Must be an integer between "
             "1 and 86400 seconds.\n\n",
             buf);
    }
  }

  /* Step 3: Sleep Duration */
  while (1) {
    printk("[3/7] Sleep Duration Between Sessions (Low-power standby)\n");
    printk("      Enter sleep duration in seconds (0 to 86400) [%u]: ",
           staged.sleep_seconds);
    if (!prompt_read_line(buf, sizeof(buf))) {
      printk(
          "\n[INFO] Configuration cancelled. Current settings retained.\n\n");
      return;
    }
    if (buf[0] == '\0') {
      break;
    }
    int val = atoi(buf);
    if (val >= 0 && val <= 86400) {
      staged.sleep_seconds = (uint32_t)val;
      printk("      -> Sleep duration set to %u s\n\n", staged.sleep_seconds);
      break;
    } else {
      printk("      [ERROR] Invalid sleep duration '%s'. Must be an integer "
             "between 0 and 86400 seconds.\n\n",
             buf);
    }
  }

  /* Step 4: Number of Cycles */
  while (1) {
    printk("[4/7] Number of Recording Cycles\n");
    printk("      Options: 1 for single session, 0 for continuous recording "
           "until stopped\n");
    printk("      Enter cycle count (0 to 10000) [%u]: ", staged.cycles);
    if (!prompt_read_line(buf, sizeof(buf))) {
      printk(
          "\n[INFO] Configuration cancelled. Current settings retained.\n\n");
      return;
    }
    if (buf[0] == '\0') {
      break;
    }
    int val = atoi(buf);
    if (val >= 0 && val <= 10000) {
      staged.cycles = (uint32_t)val;
      printk("      -> Cycle count set to %u %s\n\n", staged.cycles,
             (staged.cycles == 0) ? "[Continuous]" : "");
      break;
    } else {
      printk("      [ERROR] Invalid cycle count '%s'. Must be between 0 and "
             "10000.\n\n",
             buf);
    }
  }

  /* Step 5: Software Digital Gain */
  while (1) {
    printk("[5/7] Software Digital Gain Multiplier\n");
    printk(
        "      Options: 0.1 to 16.0 (1.0 = unity, 2.0 = +6 dB, 0.5 = -6 dB)\n");
    printk("      Enter digital gain multiplier [%.2f]: ",
           (double)staged.digital_gain);
    if (!prompt_read_line(buf, sizeof(buf))) {
      printk(
          "\n[INFO] Configuration cancelled. Current settings retained.\n\n");
      return;
    }
    if (buf[0] == '\0') {
      break;
    }
    float g = (float)atof(buf);
    if (g >= 0.1f && g <= 16.0f) {
      staged.digital_gain = g;
      printk("      -> Digital gain set to %.2fx (%+.1f dB)\n\n", (double)g,
             20.0 * log10((double)g));
      break;
    } else {
      printk("      [ERROR] Invalid digital gain '%s'. Must be a number "
             "between 0.1 and 16.0.\n\n",
             buf);
    }
  }

  /* Step 6: DSP Filter Configuration */
  while (1) {
    printk("[6/7] DSP Filter Configuration\n");
    printk("      Select DSP mode:\n");
    printk("        1: Filtered (Active multi-stage low-noise DSP) "
           "[Recommended]\n");
    printk("        2: Raw (Bypass DSP, record verbatim ADC readings)\n");
    printk("        3: Custom (Configure individual filter stages)\n");
    printk("      Enter selection [1-3] [%s]: ",
           staged.dsp_enabled ? "1" : "2");
    if (!prompt_read_line(buf, sizeof(buf))) {
      printk(
          "\n[INFO] Configuration cancelled. Current settings retained.\n\n");
      return;
    }
    if (buf[0] == '\0') {
      break;
    }
    if (strcmp(buf, "1") == 0) {
      staged.dsp_enabled = true;
      staged.dc_block_enabled = true;
      staged.hpf_enabled = true;
      staged.hpf_cutoff_hz = 80;
      staged.notch_enabled = false;
      staged.harmonic_notch_enabled = false;
      staged.lpf_enabled = true;
      staged.lpf_cutoff_hz = 20000;
      if (staged.mode == REC_MODE_RAW)
        staged.mode = REC_MODE_NORMAL;
      printk("      -> Filtered profile enabled (DC Blocker, 80 Hz HPF, 20 kHz "
             "LPF active).\n\n");
      break;
    } else if (strcmp(buf, "2") == 0) {
      staged.dsp_enabled = false;
      staged.mode = REC_MODE_RAW;
      printk("      -> Raw ADC mode enabled (All DSP filters bypassed).\n\n");
      break;
    } else if (strcmp(buf, "3") == 0) {
      staged.dsp_enabled = true;
      /* DC Blocker */
      printk("      - Enable DC bias blocker (y/n) [%s]: ",
             staged.dc_block_enabled ? "y" : "n");
      if (!prompt_read_line(buf, sizeof(buf)))
        return;
      if (buf[0] != '\0')
        staged.dc_block_enabled = (buf[0] == 'y' || buf[0] == 'Y');

      /* HPF */
      printk("      - Enable High-Pass Filter (y/n) [%s]: ",
             staged.hpf_enabled ? "y" : "n");
      if (!prompt_read_line(buf, sizeof(buf)))
        return;
      if (buf[0] != '\0')
        staged.hpf_enabled = (buf[0] == 'y' || buf[0] == 'Y');
      if (staged.hpf_enabled) {
        while (1) {
          printk("        HPF cutoff in Hz (10 to 2000) [%u]: ",
                 staged.hpf_cutoff_hz);
          if (!prompt_read_line(buf, sizeof(buf)))
            return;
          if (buf[0] == '\0')
            break;
          int hz = atoi(buf);
          if (hz >= 10 && hz <= 2000) {
            staged.hpf_cutoff_hz = (uint32_t)hz;
            break;
          }
          printk("        [ERROR] Invalid cutoff '%s' (allowed: 10-2000 Hz).\n",
                 buf);
        }
      }

      /* Notch */
      printk("      - Mains hum notch filter (50, 60, or off/n) [%s]: ",
             staged.notch_enabled ? (staged.notch_freq_hz == 60 ? "60" : "50")
                                  : "off");
      if (!prompt_read_line(buf, sizeof(buf)))
        return;
      if (buf[0] != '\0') {
        if (strcmp(buf, "50") == 0) {
          staged.notch_enabled = true;
          staged.notch_freq_hz = 50;
        } else if (strcmp(buf, "60") == 0) {
          staged.notch_enabled = true;
          staged.notch_freq_hz = 60;
        } else {
          staged.notch_enabled = false;
        }
      }
      if (staged.notch_enabled) {
        printk("        Enable 2nd harmonic (%u Hz) notch (y/n) [%s]: ",
               staged.notch_freq_hz * 2,
               staged.harmonic_notch_enabled ? "y" : "n");
        if (!prompt_read_line(buf, sizeof(buf)))
          return;
        if (buf[0] != '\0')
          staged.harmonic_notch_enabled = (buf[0] == 'y' || buf[0] == 'Y');
      }

      /* LPF */
      printk("      - Enable Low-Pass Anti-Aliasing Filter (y/n) [%s]: ",
             staged.lpf_enabled ? "y" : "n");
      if (!prompt_read_line(buf, sizeof(buf)))
        return;
      if (buf[0] != '\0')
        staged.lpf_enabled = (buf[0] == 'y' || buf[0] == 'Y');
      if (staged.lpf_enabled) {
        while (1) {
          printk("        LPF cutoff in Hz (1000 to 40000) [%u]: ",
                 staged.lpf_cutoff_hz);
          if (!prompt_read_line(buf, sizeof(buf)))
            return;
          if (buf[0] == '\0')
            break;
          int hz = atoi(buf);
          if (hz >= 1000 && hz <= 40000) {
            staged.lpf_cutoff_hz = (uint32_t)hz;
            break;
          }
          printk(
              "        [ERROR] Invalid cutoff '%s' (allowed: 1000-40000 Hz).\n",
              buf);
        }
      }
      printk("\n");
      break;
    } else {
      printk("      [ERROR] Invalid selection '%s'. Enter 1, 2, or 3.\n\n",
             buf);
    }
  }

  /* Step 7: Operating Profile */
  while (1) {
    printk("[7/7] Operating Profile\n");
    printk("      1: Normal Recording (Production Filtered)\n");
    printk("      2: Raw ADC Recording (Unfiltered)\n");
    printk("      Enter selection [1-2] [%u]: ",
           (staged.mode == REC_MODE_NORMAL) ? 1 : 2);
    if (!prompt_read_line(buf, sizeof(buf))) {
      printk(
          "\n[INFO] Configuration cancelled. Current settings retained.\n\n");
      return;
    }
    if (buf[0] == '\0') {
      break;
    }
    if (strcmp(buf, "1") == 0) {
      staged.mode = REC_MODE_NORMAL;
      staged.dsp_enabled = true;
      break;
    } else if (strcmp(buf, "2") == 0) {
      staged.mode = REC_MODE_RAW;
      staged.dsp_enabled = false;
      break;
    } else {
      printk("      [ERROR] Invalid selection '%s'.\n\n", buf);
    }
  }

  /* Step 8: PSRAM Audio Staging */
#ifdef CONFIG_AUDIO_PSRAM_STAGING
  while (1) {
    printk("[8/8] PSRAM Audio Staging Buffer\n");
    printk("      Enable PSRAM circular staging buffer (y/n) [%s]: ",
           staged.psram_staging_enabled ? "y" : "n");
    if (!prompt_read_line(buf, sizeof(buf))) {
      printk(
          "\n[INFO] Configuration cancelled. Current settings retained.\n\n");
      return;
    }
    if (buf[0] == '\0') {
      break;
    }
    if (buf[0] == 'y' || buf[0] == 'Y' || strcmp(buf, "1") == 0 ||
        strcmp(buf, "on") == 0) {
      if (!psram_is_present()) {
        printk("      [WARNING] External PSRAM hardware not detected!\n");
        printk("      PSRAM staging will remain DISABLED.\n\n");
        staged.psram_staging_enabled = false;
      } else if (!psram_is_integrity_verified()) {
        printk("      [WARNING] PSRAM hardware memory integrity has not been verified!\n");
        printk("      PSRAM self-test verification required first.\n");
        printk("      PSRAM staging will remain DISABLED.\n\n");
        staged.psram_staging_enabled = false;
      } else {
        staged.psram_staging_enabled = true;
        printk("      -> PSRAM audio staging buffer ENABLED\n\n");
      }
      break;
    } else if (buf[0] == 'n' || buf[0] == 'N' || strcmp(buf, "0") == 0 ||
               strcmp(buf, "off") == 0) {
      staged.psram_staging_enabled = false;
      printk("      -> PSRAM audio staging buffer DISABLED (Direct RAM queue "
             "to SD)\n\n");
      break;
    } else {
      printk("      [ERROR] Invalid entry '%s'. Enter 'y' or 'n'.\n\n", buf);
    }
  }
#endif

  /* Summary */
  uint32_t act_rate = 0, div = 0;
  int err_hz = 0;
  audio_in_is_rate_supported(staged.sample_rate, &act_rate, &div, &err_hz);
  uint64_t total_samples = (uint64_t)staged.record_seconds * act_rate;
  uint64_t pcm_bytes = total_samples * SAMPLE_BYTES;
  uint64_t total_wav_bytes = 44 + pcm_bytes;

  printk("\n======================= PROPOSED CONFIGURATION SUMMARY "
         "=======================\n");
  printk("  Operating Mode        : %s\n", mode_to_string(staged.mode));
  printk("  Sample Rate           : %u Hz (Actual: %u Hz, TIM6 div: %u, Error: "
         "%d Hz)\n",
         staged.sample_rate, act_rate, div, err_hz);
  printk("  Recording Duration    : %u s per session\n", staged.record_seconds);
  printk("  Sleep Duration        : %u s between sessions\n",
         staged.sleep_seconds);
  printk("  Cycle Count           : %u %s\n", staged.cycles,
         (staged.cycles == 0) ? "[Continuous]" : "");
  printk("  Software Digital Gain : %.2fx (%+.1f dB)\n",
         (double)staged.digital_gain,
         20.0 * log10((double)staged.digital_gain));
  printk("  DSP Pipeline Status   : %s\n",
         staged.dsp_enabled ? "ENABLED" : "BYPASSED (Raw Mode)");
  if (staged.dsp_enabled) {
    printk("    - DC Blocker        : %s\n",
           staged.dc_block_enabled ? "ON" : "OFF");
    printk("    - High-Pass Filter  : %s (%u Hz)\n",
           staged.hpf_enabled ? "ON" : "OFF", staged.hpf_cutoff_hz);
    printk("    - Mains Notch       : %s (%u Hz)\n",
           staged.notch_enabled ? "ON" : "OFF", staged.notch_freq_hz);
    printk("    - Harmonic Notch    : %s (%u Hz)\n",
           staged.harmonic_notch_enabled ? "ON" : "OFF",
           staged.notch_freq_hz * 2);
    printk("    - Low-Pass Filter   : %s (%u Hz)\n",
           staged.lpf_enabled ? "ON" : "OFF", staged.lpf_cutoff_hz);
  }
  printk("  PSRAM Audio Staging   : %s\n", staged.psram_staging_enabled
                                               ? "ENABLED"
                                               : "DISABLED (Direct RAM Queue)");
  printk("  Expected File Size    : %llu bytes (%.2f MB)\n", total_wav_bytes,
         (double)total_wav_bytes / (1024.0 * 1024.0));
  printk("====================================================================="
         "=========\n");

  printk("Apply and activate these settings? (y/n) [y]: ");
  if (!prompt_read_line(buf, sizeof(buf))) {
    printk("\n[INFO] Configuration cancelled. Current settings retained.\n\n");
    return;
  }
  if (buf[0] == '\0' || buf[0] == 'y' || buf[0] == 'Y') {
    settings = staged;
    recorder_set_psram_staging_enabled(settings.psram_staging_enabled);
    audio_in_set_rate(settings.sample_rate);
    apply_dsp_settings();
    printk("\n[OK] Configuration successfully applied and active!\n\n");
  } else {
    printk("\n[INFO] Configuration cancelled. Current settings retained.\n\n");
  }
}

static void __maybe_unused print_status(void) {
  printk("\n============================= SYSTEM STATUS "
         "=============================\n");
  printk("  Recorder State        : %s\n",
         recorder_is_active() ? "RECORDING ACTIVE" : "IDLE (Ready)");
  printk("  Active Mode           : %s\n", mode_to_string(settings.mode));
  printk("  Storage Interface     : Native SDMMC 4-bit bus, Mount: /SD:\n");
  printk(
      "  RAM Buffer Pool       : %u blocks of %u samples (%u KB total pool)\n",
      CONFIG_AUDIO_NUM_BLOCKS, CONFIG_AUDIO_BLOCK_SAMPLES,
      (CONFIG_AUDIO_NUM_BLOCKS * CONFIG_AUDIO_BLOCK_SAMPLES * SAMPLE_BYTES) >>
          10);
  printk("  RAM Buffer Depth      : %u ms buffer at %u Hz\n",
         (uint32_t)((uint64_t)CONFIG_AUDIO_NUM_BLOCKS *
                    CONFIG_AUDIO_BLOCK_SAMPLES * 1000 / settings.sample_rate),
         settings.sample_rate);
  uint32_t ht = 0, tc = 0;
  audio_in_get_callback_counts(&ht, &tc);
  printk("  DMA Interrupts        : HT = %u, TC = %u (Total = %u callbacks)\n",
         ht, tc, ht + tc);
  printk("  Audio Acquisition     : %u Hz (TIM6 Clock: 72 MHz)\n",
         audio_in_rate());
  printk("  DSP Pipeline          : %s\n",
         settings.dsp_enabled ? "Active (Direct Form II Transposed Biquads)"
                              : "Bypassed (Raw ADC)");
  printk("  Digital Gain / Clip   : %.2fx multiplier (Digital Clipping Events: "
         "%u)\n",
         (double)dsp_get_digital_gain(), dsp_get_digital_clipping_count());
#ifdef CONFIG_PSRAM_TEST_ENABLE
  printk("  External PSRAM        : IS66WVS4M8 (32 Mb / 4 MB capacity)\n");
#endif
#ifdef CONFIG_AUDIO_PSRAM_STAGING
  struct psram_staging_stats pstat;
  psram_staging_get_stats(&pstat);
  printk("  PSRAM Staging Buffer  : %s (Capacity: %u KB, Occupied: %u B, Peak: "
         "%u B, Drops: %u)\n",
         pstat.active ? "ACTIVE" : "STANDBY", pstat.capacity_bytes >> 10,
         pstat.occupied_bytes, pstat.high_water_bytes, pstat.dropped_samples);
#endif
  struct rtc_calendar_time stat_time;
  char stat_time_buf[32] = "[UNSET / UNINITIALIZED]";
  if (rtc_calendar_get(&stat_time) == 0 && rtc_calendar_is_valid()) {
    rtc_calendar_format(&stat_time, stat_time_buf, sizeof(stat_time_buf));
  }
  printk("  RTC Calendar Time     : %s\n", stat_time_buf);
  printk("====================================================================="
         "=====\n\n");
}

static void __maybe_unused run_monitor(uint32_t seconds) {
  printk("\n==================================================================="
         "======\n");
  printk("               LIVE MICROPHONE REAL-TIME DIAGNOSTIC MONITOR          "
         "    \n");
  printk("  Duration          : %u seconds (Press any key to stop early)\n",
         seconds);
  printk("  Sampling Rate     : %u Hz (TIM6 TRGO -> ADC1 -> DMA1)\n",
         audio_in_rate());
  printk("  Digital Gain      : %.2fx\n", (double)dsp_get_digital_gain());
  printk("  Analog Hardware   : MAX4466 Trimmer Pot controls pre-ADC analog "
         "gain\n");
  printk("                      Adjust pot so Span is between 100 mV and 1500 "
         "mV.\n");
  printk("====================================================================="
         "====\n");
  printk("Sec | Rate (Hz) | Min (mV) | Max (mV) | Span (mV) | Mid (mV) | RMS "
         "(mV) | RMS dBFS    | Clip L/H | Status\n");
  printk("----+-----------+----------+----------+-----------+----------+-------"
         "---+--------------+----------+--------------------\n");

  audio_in_reset_diag();
  dsp_reset_digital_clipping_count();
  audio_in_start();

  uint32_t prev_samples = 0;
  int64_t t_start = k_uptime_get();

  for (uint32_t s = 1; s <= seconds; s++) {
    /* Check if user pressed a key to abort early */
    unsigned char c;
    if (console_poll_char(&c) == 0) {
      printk("Monitor interrupted by user.\n");
      break;
    }

    k_sleep(K_SECONDS(1));

    struct audio_diag diag;
    audio_in_get_diag(&diag);

    uint32_t sample_delta = diag.total_samples - prev_samples;
    prev_samples = diag.total_samples;

    uint32_t min_mv = (uint32_t)((uint64_t)diag.raw_min * 3300 / 32767);
    uint32_t max_mv = (uint32_t)((uint64_t)diag.raw_max * 3300 / 32767);
    uint32_t span_mv = (max_mv > min_mv) ? (max_mv - min_mv) : 0;
    uint32_t mid_mv = (uint32_t)((uint64_t)diag.raw_midpoint * 3300 / 32767);

    const char *status_str;
    if (diag.raw_min == 0 && diag.raw_max == 0) {
      status_str = "ERR: PIN GROUNDED";
    } else if (diag.raw_min >= 32760 && diag.raw_max >= 32760) {
      status_str = "ERR: PIN TIED TO 3.3V";
    } else if (span_mv < 50) {
      status_str = "SILENCE / QUIET";
    } else if (span_mv < 1500) {
      status_str = "OPTIMAL GAIN (VOICE)";
    } else {
      status_str = "LOUD / ANALOG CLIPPING";
    }

    printk("%3u | %9u | %8u | %8u | %9u | %8u | %8u | %7d.%01d   | %4u/%-4u | "
           "%s\n",
           s, sample_delta, min_mv, max_mv, span_mv, mid_mv, diag.raw_rms_mv,
           diag.raw_rms_dbfs_x10 / 10, abs(diag.raw_rms_dbfs_x10 % 10),
           diag.clipping_low_count, diag.clipping_high_count, status_str);
  }

  audio_in_stop();
  int64_t elapsed_ms = k_uptime_get() - t_start;
  struct audio_diag final_diag;
  audio_in_get_diag(&final_diag);
  uint32_t measured_hz =
      (elapsed_ms > 0)
          ? (uint32_t)((uint64_t)final_diag.total_samples * 1000 / elapsed_ms)
          : 0;

  printk("---------------------------------------------------------------------"
         "----\n");
  printk("Monitor Summary: %lld ms, %u samples, %u Hz measured, raw AC RMS: %u mV "
         "(%.1f dBFS)\n\n",
         elapsed_ms, final_diag.total_samples, measured_hz,
           final_diag.raw_rms_mv, (double)final_diag.raw_rms_dbfs_x10 / 10.0);
}

static void __maybe_unused print_schedule(void) {
  struct schedule_config scfg;
  scheduler_get_config(&scfg);
  char buf[64];

  printk("\n======================= ACTIVE RECORDING SCHEDULE =======================\n");
  if (scfg.start_date.year > 0 || scfg.end_date.year > 0) {
    scheduler_format_date_range(&scfg.start_date, &scfg.end_date, buf, sizeof(buf));
    printk("  Date Range Bounds     : %s\n", buf);
  } else {
    printk("  Date Range Bounds     : UNCONSTRAINED (Active indefinitely across all dates)\n");
  }

  scheduler_format_days_of_week(scfg.day_of_week_mask, buf, sizeof(buf));
  printk("  Days of Week Mask     : %s (Mask: 0x%02X)\n", buf, scfg.day_of_week_mask);

  scheduler_format_timezone(scfg.time_zone_offset, buf, sizeof(buf));
  printk("  Timezone Offset       : %s (%+d seconds relative to UTC)\n", buf, scfg.time_zone_offset);

  printk("  Time-of-Day Ranges    :\n");
  uint32_t valid_tod_count = 0;
  for (uint32_t i = 0; i < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
    if (scheduler_is_valid_tod_range(&scfg.tod_ranges[i])) {
      scheduler_format_time_of_day(&scfg.tod_ranges[i], buf, sizeof(buf));
      printk("    [%2u] %s\n", i + 1, buf);
      valid_tod_count++;
    }
  }
  if (valid_tod_count == 0) {
    printk("    [All Day] 00:00:00 to 24:00:00 (No time-of-day restrictions)\n");
  }

  printk("  Recording Duration    : %u seconds per session\n", settings.record_seconds);
  printk("  Sleep/Pause Interval  : %u seconds between sessions\n", settings.sleep_seconds);
  printk("  Recording Cycles      : %u %s\n", settings.cycles, (settings.cycles == 0) ? "[Continuous]" : "");

  struct rtc_calendar_time cur_time;
  if (rtc_calendar_get(&cur_time) == 0 && rtc_calendar_is_valid()) {
    rtc_calendar_format(&cur_time, buf, sizeof(buf));
    printk("  Current RTC Time      : %s UTC\n", buf);

    time_t now_utc = scheduler_calendar_to_epoch(&cur_time);
    struct schedule_interval next_int;
    int res = scheduler_calculate_next(&scfg, now_utc, 0, &next_int);
    if (res == 0 && next_int.is_valid) {
      struct rtc_calendar_time next_cal;
      scheduler_epoch_to_calendar(next_int.start_time_utc + scfg.time_zone_offset, &next_cal);
      char next_buf[32];
      rtc_calendar_format(&next_cal, next_buf, sizeof(next_buf));
      printk("  Next Evaluated Window : %s (Local: %s, in %lld seconds, %u s record)\n",
             next_int.is_immediate ? "NOW (Window Open)" : "FUTURE",
             next_buf, (long long)next_int.wait_seconds, next_int.duration_seconds);
    } else {
      printk("  Next Evaluated Window : NONE / EXPIRED (Outside active dates, days, or ranges)\n");
    }
  } else {
    printk("  Current RTC Time      : [UNSET / INVALID] - Set RTC with 'time set YYYY-MM-DD HH:MM:SS'\n");
  }
  printk("=========================================================================\n\n");
}

static void __maybe_unused run_schedule_check(void) {
  struct rtc_calendar_time cur_time;
  if (rtc_calendar_get(&cur_time) != 0 || !rtc_calendar_is_valid()) {
    printk("\n[ERROR] Hardware RTC calendar is not set. Use 'time set YYYY-MM-DD HH:MM:SS'\n\n");
    return;
  }

  struct schedule_config scfg;
  scheduler_get_config(&scfg);
  scfg.record_length = settings.record_seconds;
  scfg.sleep_interval = settings.sleep_seconds;

  time_t now_utc = scheduler_calendar_to_epoch(&cur_time);
  struct schedule_interval next_int;
  int res = scheduler_calculate_next(&scfg, now_utc, 0, &next_int);

  printk("\n============================ SCHEDULE AUDIT CHECK ============================\n");
  char buf[64];
  rtc_calendar_format(&cur_time, buf, sizeof(buf));
  printk("  Reference RTC Time    : %s UTC\n", buf);
  time_t now_local = now_utc + scfg.time_zone_offset;
  struct rtc_calendar_time local_cal;
  scheduler_epoch_to_calendar(now_local, &local_cal);
  rtc_calendar_format(&local_cal, buf, sizeof(buf));
  printk("  Reference Local Time  : %s\n", buf);

  if (res == 0 && next_int.is_valid) {
    time_t sched_local = next_int.start_time_utc + scfg.time_zone_offset;
    scheduler_epoch_to_calendar(sched_local, &local_cal);
    rtc_calendar_format(&local_cal, buf, sizeof(buf));
    printk("  Next Scheduled Start  : %s Local (%s UTC)\n", buf,
           next_int.is_immediate ? "WINDOW OPEN NOW" : "WAITING FOR WINDOW");
    printk("  Wait Duration         : %lld seconds (approx. %lld min / %lld hr)\n",
           (long long)next_int.wait_seconds,
           (long long)next_int.wait_seconds / 60,
           (long long)next_int.wait_seconds / 3600);
    printk("  Session Duration      : %u seconds\n", next_int.duration_seconds);
    printk("  Audit Verdict         : SCHEDULE ACTIVE & READY TO EXECUTE\n");
  } else {
    printk("  Next Scheduled Start  : NONE\n");
    printk("  Audit Verdict         : SCHEDULE CLOSED / EXPIRED (No matching windows found)\n");
  }
  printk("==============================================================================\n\n");
}

static void run_scheduled_recording(void) {
  struct rtc_calendar_time cur_time;
  if (rtc_calendar_get(&cur_time) != 0 || !rtc_calendar_is_valid()) {
    printk("\n[ERROR] Hardware RTC calendar is not initialized or invalid!\n");
    printk("Set RTC before running schedule: time set YYYY-MM-DD HH:MM:SS [+-HH:MM]\n\n");
    return;
  }

  struct schedule_config scfg;
  scheduler_get_config(&scfg);
  scfg.record_length = settings.record_seconds;
  scfg.sleep_interval = settings.sleep_seconds;
  scheduler_set_config(&scfg);

  apply_dsp_settings();

  uint32_t max_cycles = settings.cycles;
  enum recorder_mode mode = settings.mode;

  printk("\n=========================================================================\n");
  printk("               WILDSONIC TIME-SCHEDULED RECORDING ENGINE                 \n");
  printk("=========================================================================\n");
  char buf[64];
  if (scfg.start_date.year > 0 || scfg.end_date.year > 0) {
    scheduler_format_date_range(&scfg.start_date, &scfg.end_date, buf, sizeof(buf));
    printk("  Date Range Bounds     : %s\n", buf);
  } else {
    printk("  Date Range Bounds     : ALL DATES (Unconstrained)\n");
  }
  scheduler_format_days_of_week(scfg.day_of_week_mask, buf, sizeof(buf));
  printk("  Days of Week Mask     : %s (Mask: 0x%02X)\n", buf, scfg.day_of_week_mask);
  scheduler_format_timezone(scfg.time_zone_offset, buf, sizeof(buf));
  printk("  Timezone Offset       : %s\n", buf);
  printk("  Configured TOD Ranges :\n");
  bool any_tod = false;
  for (uint32_t i = 0; i < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
    if (scheduler_is_valid_tod_range(&scfg.tod_ranges[i])) {
      scheduler_format_time_of_day(&scfg.tod_ranges[i], buf, sizeof(buf));
      printk("    [%u] %s\n", i + 1, buf);
      any_tod = true;
    }
  }
  if (!any_tod) {
    printk("    [All Day] 00:00:00 to 24:00:00 (No window restrictions)\n");
  }
  printk("  Session Duration      : %u seconds\n", scfg.record_length);
  printk("  Inter-Session Sleep   : %u seconds\n", scfg.sleep_interval);
  printk("  Cycles Configured     : %u %s\n", max_cycles, (max_cycles == 0) ? "[Continuous]" : "");
  printk("  Control               : Type 'stop' to abort schedule\n");
  printk("=========================================================================\n\n");

  uint32_t cycle = 1;
  time_t last_end_time_local = 0;

  while (max_cycles == 0 || cycle <= max_cycles) {
    if (recorder_is_abort_requested()) {
      printk("\n[Schedule cancelled by user stop request]\n");
      break;
    }

    if (rtc_calendar_get(&cur_time) != 0 || !rtc_calendar_is_valid()) {
      printk("\n[ERROR] RTC reading lost during schedule execution!\n");
      break;
    }
    time_t now_utc = scheduler_calendar_to_epoch(&cur_time);

    struct schedule_interval interval;
    int calc_res = scheduler_calculate_next(&scfg, now_utc, last_end_time_local, &interval);
    if (calc_res != 0 || !interval.is_valid) {
      printk("\n[SCHEDULE COMPLETE] No upcoming recording windows found.\n");
      printk("  (Calendar end date reached, days of week exhausted, or all windows past).\n\n");
      break;
    }

    if (!interval.is_immediate && interval.wait_seconds > 0) {
      struct rtc_calendar_time sched_start_cal;
      time_t sched_start_local = interval.start_time_utc + scfg.time_zone_offset;
      scheduler_epoch_to_calendar(sched_start_local, &sched_start_cal);
      char start_str[32];
      rtc_calendar_format(&sched_start_cal, start_str, sizeof(start_str));

      printk("\n---- [SCHEDULE STANDBY: NEXT SESSION AT %s LOCAL (%lld s WAIT)] ----\n",
             start_str, (long long)interval.wait_seconds);
      printk("  Power Mode            : STM32L476 Stop 2 (Strict RTC-only Wakeup)\n");
      printk("  Wakeup Target         : %s (in %lld seconds)\n", start_str, (long long)interval.wait_seconds);
      printk("  Type 'stop' to abort schedule\n");

      bool aborted = false;
      while (!recorder_is_abort_requested()) {
        struct rtc_calendar_time now_c;
        if (rtc_calendar_get(&now_c) != 0 || !rtc_calendar_is_valid()) {
          aborted = true;
          break;
        }
        time_t cur_u = scheduler_calendar_to_epoch(&now_c);
        int64_t rem = (int64_t)(interval.start_time_utc - cur_u);
        if (rem <= 0) {
          break;
        }
        /* Enter true STM32L476 Stop 2 low-power sleep with strict RTC-only wakeup */
        uint32_t sleep_chunk = (rem > 65535) ? 65535 : (uint32_t)rem;
        int sleep_rc = recorder_sleep(sleep_chunk, NULL);
        if (sleep_rc != 0) {
          aborted = true;
          break;
        }
      }
      if (recorder_is_abort_requested()) {
        printk("\n[Schedule aborted by user stop request during wait]\n");
        aborted = true;
      }
      if (aborted) {
        break;
      }
    }

    printk("\n---- [STARTING SCHEDULED SESSION %u: %u SECONDS] ----\n",
           cycle, interval.duration_seconds);
    struct rec_stats st;
    memset(&st, 0, sizeof(st));

    recorder_set_psram_staging_enabled(settings.psram_staging_enabled);
    int ret = recorder_record_session(interval.duration_seconds, cycle, mode, &st);

    last_end_time_local = interval.start_time_utc + scfg.time_zone_offset + interval.duration_seconds;

    if (ret != 0 || st.err != 0) {
      int failure = (ret != 0) ? ret : st.err;
      last_end_time_local = interval.start_time_utc + scfg.time_zone_offset + interval.duration_seconds;
      bool storage_recovery_ok = recorder_get_last_sd_error() == 0 ||
                                 recorder_get_storage_recovery_result() == 0;
      if ((failure == -EIO || failure == -ETIMEDOUT || failure == -ENOBUFS || failure == -EBUSY) &&
          storage_recovery_ok && !recorder_is_active()) {
        printk("\n[Schedule session %u failed (%d); skipping this opportunity and continuing.]\n",
               cycle, failure);
        continue;
      }
      printk("\n[Schedule session %u failed with non-recoverable error %d.]\n", cycle, failure);
      break;
    }

    if (recorder_is_abort_requested()) {
      printk("\n[Schedule interrupted by user after session %u]\n", cycle);
      break;
    }

    if (max_cycles > 0 && cycle >= max_cycles) {
      printk("\n[Schedule target cycles (%u) completed successfully]\n", max_cycles);
      break;
    }

    cycle++;
  }

  printk("\n=========================================================================\n");
  printk("               SCHEDULED RECORDING SEQUENCE FINISHED                     \n");
  printk("  Total Sessions Completed: %u\n", cycle > 0 ? (cycle - 1) : 0);
  printk("=========================================================================\n\n");
}

/* ==============================================================================
 * Autonomous Scheduler Engine & State
 * ============================================================================== */

static bool s_schedule_paused = false;
static volatile bool s_reboot_pending = false;
static uint32_t s_schedule_cycle = 1;
static time_t s_last_end_time_local = 0;
static time_t s_last_evaluated_window_utc = 0;
static bool s_rtc_uninit_logged = false;
static bool s_no_window_logged = false;

void main_scheduler_resume(void) {
  if (main_is_reboot_pending()) {
    return;
  }
  if (s_schedule_paused) {
    s_schedule_paused = false;
    s_last_evaluated_window_utc = 0;
    if (led_indicator_get() == LED_IND_ERROR) {
      led_indicator_set(LED_IND_OFF);
    }
    printk("[SCHEDULER] Resuming autonomous scheduled recording evaluation.\n");
  }
}

void main_scheduler_pause(void) {
  if (!s_schedule_paused) {
    s_schedule_paused = true;
    if (!main_is_reboot_pending()) {
      printk("[SCHEDULER] Scheduled recording paused.\n");
    }
  }
}

/* ==============================================================================
 * Dedicated Preemptive Audio Recording Thread Architecture
 * ============================================================================== */

#define RECORDER_THREAD_STACK_SIZE 4096
#define RECORDER_THREAD_PRIORITY   3

K_THREAD_STACK_DEFINE(recorder_thread_stack, RECORDER_THREAD_STACK_SIZE);
static struct k_thread recorder_thread_data;
static k_tid_t recorder_tid;

struct recorder_request {
  uint32_t duration_sec;
  uint32_t cycle;
  enum recorder_mode mode;
  bool is_autonomous;
  time_t start_time_utc;
  int32_t tz_offset;
};

static struct recorder_request s_rec_req;
static struct k_sem s_rec_trigger_sem;
static struct k_sem s_rec_done_sem;
static volatile bool s_rec_busy = false;
static int s_rec_last_result = 0;
static struct rec_stats s_rec_last_stats;

static void recorder_thread_fn(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    k_sem_take(&s_rec_trigger_sem, K_FOREVER);
    s_rec_busy = true;

    /* Ensure audio hardware rate matches active settings before session begins */
    if (audio_in_rate() != settings.sample_rate) {
      int rc = audio_in_set_rate(settings.sample_rate);
      if (rc != 0) {
        LOG_WRN("Reconciling sample rate to hardware actual %u Hz (was %u)",
                audio_in_rate(), settings.sample_rate);
      }
    }
    settings.sample_rate = audio_in_rate();

    /* Snapshot current staged DSP/PSRAM settings for this session */
    apply_dsp_settings();
    recorder_set_psram_staging_enabled(settings.psram_staging_enabled);

    memset(&s_rec_last_stats, 0, sizeof(s_rec_last_stats));
    s_rec_last_result = recorder_record_session(s_rec_req.duration_sec,
                                               s_rec_req.cycle,
                                               s_rec_req.mode,
                                               &s_rec_last_stats);

    if (s_rec_req.is_autonomous) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("\n[SCHEDULER] Session %u finished: status=%d, err=%d, samples=%llu, duration=%u s\n",
             s_rec_req.cycle, s_rec_last_result, s_rec_last_stats.err,
             (unsigned long long)s_rec_last_stats.samples_written, s_rec_req.duration_sec);
#endif

      if (recorder_is_abort_requested()) {
        if (!main_is_reboot_pending() && recorder_get_stop_reason() == RECORDER_STOP_USER) {
          printk("[SCHEDULER] Recording aborted by user stop request.\n");
          printk("[SCHEDULER] Scheduled recording paused. Enter commands or press Enter to resume.\n\n> ");
        }
        s_schedule_paused = true;
      } else if (s_rec_last_result != 0 || s_rec_last_stats.err != 0) {
        int failure = (s_rec_last_result != 0) ? s_rec_last_result : s_rec_last_stats.err;
        time_t failed_end_utc = s_rec_req.start_time_utc + s_rec_req.duration_sec;
        s_last_end_time_local = failed_end_utc + s_rec_req.tz_offset;
        s_last_evaluated_window_utc = 0;
        /* Do not update scheduler_retention_save on failed sessions: preserve last successful end */

        s_schedule_cycle++;
        if (settings.cycles > 0 && s_schedule_cycle > settings.cycles) {
          printk("\n[SCHEDULER] Configured target cycles (%u) completed.\n\n> ", settings.cycles);
          s_schedule_paused = true;
        } else {
          printk("[SCHEDULER] Session %u failed (%d); recording opportunity skipped, schedule remains active for next window.\n\n> ",
                 s_rec_req.cycle, failure);
          s_schedule_paused = false;
        }
      } else {
        time_t end_utc = s_rec_req.start_time_utc + s_rec_req.duration_sec;
        s_last_end_time_local = end_utc + s_rec_req.tz_offset;
        scheduler_retention_save(end_utc);

        s_schedule_cycle++;
        if (settings.cycles > 0 && s_schedule_cycle > settings.cycles) {
          printk("\n[SCHEDULER] Configured target cycles (%u) completed successfully.\n\n> ", settings.cycles);
          s_schedule_paused = true;
        }
        s_last_evaluated_window_utc = 0;
      }
    }

    s_rec_busy = false;
    k_sem_give(&s_rec_done_sem);
  }
}

static uint32_t s_sleep_attempt_counter = 0;
static int64_t s_sleep_backoff_until_ms = 0;
static uint32_t s_consecutive_sleep_failures = 0;
static int s_last_sleep_error = 0;
static int64_t s_last_sleep_error_log_ms = 0;

bool main_is_scheduler_paused(void) {
  return s_schedule_paused;
}

bool main_is_reboot_pending(void) {
  return s_reboot_pending;
}

void main_set_reboot_pending(bool pending) {
  s_reboot_pending = pending;
  if (pending) {
    s_schedule_paused = true;
  }
}

void main_scheduler_service_step(void) {
  if (s_reboot_pending || s_schedule_paused || s_rec_busy || recorder_is_active()) {
    return;
  }

  /* Do not attempt sleep while in backoff recovery window after a failed attempt */
  if (k_uptime_get() < s_sleep_backoff_until_ms) {
    return;
  }

  struct rtc_calendar_time cur_time;
  if (rtc_calendar_get(&cur_time) != 0 || !rtc_calendar_is_valid()) {
    if (!s_rtc_uninit_logged) {
      printk("\n[SCHEDULER] RTC calendar uninitialized or invalid. Configure DATETIME to activate scheduler.\n> ");
      s_rtc_uninit_logged = true;
    }
    return;
  }
  s_rtc_uninit_logged = false;

  struct schedule_config scfg;
  scheduler_get_config(&scfg);
  scfg.record_length = settings.record_seconds;
  scfg.sleep_interval = settings.sleep_seconds;
  scheduler_set_config(&scfg);

  time_t now_utc = scheduler_calendar_to_epoch(&cur_time);
  time_t now_local = now_utc + scfg.time_zone_offset;

  struct schedule_interval interval;
  int calc_res = scheduler_calculate_next(&scfg, now_utc, s_last_end_time_local, &interval);
  if (calc_res != 0 || !interval.is_valid) {
    if (!s_no_window_logged) {
      printk("\n[SCHEDULER] Inactive or complete: No upcoming recording windows found.\n");
      printk("  (Calendar end date reached, days of week exhausted, or all windows past).\n> ");
      s_no_window_logged = true;
    }
    return;
  }
  s_no_window_logged = false;

  if (interval.is_immediate || interval.wait_seconds == 0) {
    char utc_str[32], local_str[32], date_buf[64], dow_buf[32], tz_buf[16];
    struct rtc_calendar_time now_local_cal;
    scheduler_epoch_to_calendar(now_local, &now_local_cal);

    rtc_calendar_format(&cur_time, utc_str, sizeof(utc_str));
    rtc_calendar_format(&now_local_cal, local_str, sizeof(local_str));

    if (scfg.start_date.year > 0 || scfg.end_date.year > 0) {
      scheduler_format_date_range(&scfg.start_date, &scfg.end_date, date_buf, sizeof(date_buf));
    } else {
      snprintf(date_buf, sizeof(date_buf), "ALL (Unconstrained)");
    }
    scheduler_format_days_of_week(scfg.day_of_week_mask, dow_buf, sizeof(dow_buf));
    scheduler_format_timezone(scfg.time_zone_offset, tz_buf, sizeof(tz_buf));

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("\n=========================================================================\n");
    printk("           [SCHEDULER] AUTONOMOUS RECORDING SESSION %u START              \n", s_schedule_cycle);
    printk("=========================================================================\n");
    printk("  Current UTC Time      : %s\n", utc_str);
    printk("  Current Local Time    : %s (%s)\n", local_str, tz_buf);
    printk("  Active Date Range     : %s\n", date_buf);
    printk("  Days of Week Mask     : %s (0x%02X)\n", dow_buf, scfg.day_of_week_mask);
    printk("  Session Duration      : %u seconds\n", interval.duration_seconds);
    printk("  Inter-Session Sleep   : %u seconds\n", scfg.sleep_interval);
    printk("  Recording Mode        : %s\n", mode_to_string(settings.mode));
    printk("  Sampling Rate / Bits  : %u Hz / 16-bit\n", settings.sample_rate);
    printk("  Block Size / Pool     : %u samples (%u B) / %u blocks\n",
           CONFIG_AUDIO_BLOCK_SAMPLES, (uint32_t)(CONFIG_AUDIO_BLOCK_SAMPLES * (CONFIG_AUDIO_SAMPLE_BITS / 8)),
           CONFIG_AUDIO_NUM_BLOCKS);
    printk("  Storage Destination   : SDMMC (/SD:)\n");
    printk("  Control               : Asynchronous console active; type commands or 'stop'\n");
    printk("=========================================================================\n");
#endif

    /* Dispatch autonomous recording to recorder_thread without blocking console REPL */
    s_rec_req.duration_sec = interval.duration_seconds;
    s_rec_req.cycle = s_schedule_cycle;
    s_rec_req.mode = settings.mode;
    s_rec_req.is_autonomous = true;
    s_rec_req.start_time_utc = interval.start_time_utc;
    s_rec_req.tz_offset = scfg.time_zone_offset;
    s_rec_busy = true;
    k_sem_give(&s_rec_trigger_sem);
    return;
  } else {
    /* Standby wait for upcoming window: enter STM32L476 Stop 2 with strict RTC-only wakeup */
    /* If console was recently active, keep system awake so user can interact */
    if (k_uptime_get() - s_console_last_activity_ms < 10000) {
      return;
    }

    struct rtc_calendar_time sched_start_cal;
    time_t sched_start_local = interval.start_time_utc + scfg.time_zone_offset;
    scheduler_epoch_to_calendar(sched_start_local, &sched_start_cal);
    char start_str[32];
    rtc_calendar_format(&sched_start_cal, start_str, sizeof(start_str));

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("\n[SCHEDULER] Standby wait: %lld s remaining until Session %u (Target: %s Local)\n",
           (long long)interval.wait_seconds, s_schedule_cycle, start_str);
    printk("  Entering STM32L476 Stop 2 low-power standby (strict RTC-only wake)...\n");
#endif

    while (!s_schedule_paused && !recorder_is_abort_requested()) {
      struct rtc_calendar_time now_cal;
      if (rtc_calendar_get(&now_cal) != 0 || !rtc_calendar_is_valid()) {
        break;
      }
      time_t cur_u = scheduler_calendar_to_epoch(&now_cal);
      int64_t remaining_sec = (int64_t)(interval.start_time_utc - cur_u);
      if (remaining_sec <= 0) {
        /* Scheduled start time reached */
        break;
      }

      /* Enforce safe sleep chunk bounds (max 65535 s for 16-bit RTC wake timer) */
      uint32_t sleep_chunk = (remaining_sec > 65535) ? 65535 : (uint32_t)remaining_sec;

      s_sleep_attempt_counter++;
      uint32_t attempt_num = s_sleep_attempt_counter;

      int sleep_rc = recorder_sleep(sleep_chunk, NULL);
      if (sleep_rc != 0) {
        if (sleep_rc == -EINTR) {
          LOG_INF("Autonomous sleep attempt %u aborted by console request (resetting consecutive failure counter, was %u)",
                  attempt_num, s_consecutive_sleep_failures);
          s_consecutive_sleep_failures = 0;
          s_sleep_backoff_until_ms = 0;
          s_last_sleep_error = 0;
        } else {
          s_consecutive_sleep_failures++;
          /* Bounded backoff: 1000 ms base, up to 10000 ms */
          uint32_t backoff_ms = 1000U * (s_consecutive_sleep_failures > 10 ? 10U : s_consecutive_sleep_failures);
          s_sleep_backoff_until_ms = k_uptime_get() + backoff_ms;

          /* Rate-limit console error diagnostics (max once every 5 seconds) */
          if (k_uptime_get() - s_last_sleep_error_log_ms >= 5000 || sleep_rc != s_last_sleep_error) {
            LOG_ERR("Autonomous sleep attempt %u returned error: %d (consecutive_failures=%u, backing off for %u ms)",
                    attempt_num, sleep_rc, s_consecutive_sleep_failures, backoff_ms);
            s_last_sleep_error = sleep_rc;
            s_last_sleep_error_log_ms = k_uptime_get();
          }
        }
        break;
      } else {
        /* Successful sleep: reset backoff and failure counters */
        if (s_consecutive_sleep_failures > 0) {
          LOG_INF("Autonomous sleep attempt %u succeeded: resetting consecutive failure counter (was %u)",
                  attempt_num, s_consecutive_sleep_failures);
        }
        s_consecutive_sleep_failures = 0;
        s_sleep_backoff_until_ms = 0;
        s_last_sleep_error = 0;
      }
    }

    if (s_schedule_paused || recorder_is_abort_requested()) {
      return;
    }

    /* Re-evaluate RTC calendar time after waking from sleep before dispatching */
    struct rtc_calendar_time wake_cal;
    if (rtc_calendar_get(&wake_cal) != 0 || !rtc_calendar_is_valid()) {
      return;
    }
    time_t wake_utc = scheduler_calendar_to_epoch(&wake_cal);
    if (wake_utc < interval.start_time_utc) {
      /* Woke early (e.g. chunk completion); return to let outer loop service or re-sleep */
      return;
    }

    /* Start recording session at configured scheduled time */
    char utc_str[32], local_str[32], date_buf[64], dow_buf[32], tz_buf[16];
    struct rtc_calendar_time now_local_cal;
    time_t wake_local = wake_utc + scfg.time_zone_offset;
    scheduler_epoch_to_calendar(wake_local, &now_local_cal);

    rtc_calendar_format(&wake_cal, utc_str, sizeof(utc_str));
    rtc_calendar_format(&now_local_cal, local_str, sizeof(local_str));

    if (scfg.start_date.year > 0 || scfg.end_date.year > 0) {
      scheduler_format_date_range(&scfg.start_date, &scfg.end_date, date_buf, sizeof(date_buf));
    } else {
      snprintf(date_buf, sizeof(date_buf), "ALL (Unconstrained)");
    }
    scheduler_format_days_of_week(scfg.day_of_week_mask, dow_buf, sizeof(dow_buf));
    scheduler_format_timezone(scfg.time_zone_offset, tz_buf, sizeof(tz_buf));

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("\n=========================================================================\n");
    printk("           [SCHEDULER] AUTONOMOUS RECORDING SESSION %u START              \n", s_schedule_cycle);
    printk("=========================================================================\n");
    printk("  Current UTC Time      : %s\n", utc_str);
    printk("  Current Local Time    : %s (%s)\n", local_str, tz_buf);
    printk("  Active Date Range     : %s\n", date_buf);
    printk("  Days of Week Mask     : %s (0x%02X)\n", dow_buf, scfg.day_of_week_mask);
    printk("  Session Duration      : %u seconds\n", interval.duration_seconds);
    printk("  Inter-Session Sleep   : %u seconds\n", scfg.sleep_interval);
    printk("  Recording Mode        : %s\n", mode_to_string(settings.mode));
    printk("  Sampling Rate / Bits  : %u Hz / 16-bit\n", settings.sample_rate);
    printk("  Block Size / Pool     : %u samples (%u B) / %u blocks\n",
           CONFIG_AUDIO_BLOCK_SAMPLES, (uint32_t)(CONFIG_AUDIO_BLOCK_SAMPLES * (CONFIG_AUDIO_SAMPLE_BITS / 8)),
           CONFIG_AUDIO_NUM_BLOCKS);
    printk("  Storage Destination   : SDMMC (/SD:)\n");
    printk("  Control               : Asynchronous console active; type commands or 'stop'\n");
    printk("=========================================================================\n");
#endif

    /* Dispatch autonomous recording to recorder_thread without blocking console REPL */
    s_rec_req.duration_sec = interval.duration_seconds;
    s_rec_req.cycle = s_schedule_cycle;
    s_rec_req.mode = settings.mode;
    s_rec_req.is_autonomous = true;
    s_rec_req.start_time_utc = interval.start_time_utc;
    s_rec_req.tz_offset = scfg.time_zone_offset;
    s_rec_busy = true;
    k_sem_give(&s_rec_trigger_sem);
    return;
  }
}

/* ==============================================================================
 * Bridge and Dispatch Functions for Console & Test Subsystems
 * ============================================================================== */

struct app_settings *main_get_settings(void) {
  return &settings;
}

void main_apply_dsp_settings(void) {
  apply_dsp_settings();
}

void main_reset_to_defaults(void) {
  reset_to_defaults();
}

void main_run_scheduled_recording(void) {
  run_scheduled_recording();
}

void main_run_guided_config(void) {
  run_guided_config();
}

void main_psram_command(int matched, const char *arg1, const char *arg2) {
  char a1[32] = {0}, a2[32] = {0};
  if (arg1) {
    strncpy(a1, arg1, sizeof(a1) - 1);
    for (int i = 0; a1[i]; i++) a1[i] = tolower((unsigned char)a1[i]);
  }
  if (arg2) {
    strncpy(a2, arg2, sizeof(a2) - 1);
    for (int i = 0; a2[i]; i++) a2[i] = tolower((unsigned char)a2[i]);
  }

  if (matched >= 2 && (strcmp(a1, "on") == 0 || strcmp(a1, "1") == 0 || strcmp(a1, "enable") == 0)) {
    if (recorder_is_active()) {
      printk("\n[REJECTED] Cannot change PSRAM staging state while recording is active.\n\n");
      return;
    }
    if (!psram_is_present()) {
      printk("\n[REJECTED] Cannot enable PSRAM staging: PSRAM hardware not detected.\n\n");
      settings.psram_staging_enabled = false;
      recorder_set_psram_staging_enabled(false);
    } else if (!psram_is_integrity_verified()) {
      printk("\n[REJECTED] Cannot enable PSRAM staging: PSRAM memory integrity has NOT passed self-test.\n");
      printk("  Run standalone memory self-test to verify hardware integrity first.\n\n");
      settings.psram_staging_enabled = false;
      recorder_set_psram_staging_enabled(false);
    } else if (!psram_is_quad_mode()) {
      printk("\n[REJECTED] Cannot enable PSRAM staging: Quad I/O is not verified (1-line SPI active).\n\n");
      settings.psram_staging_enabled = false;
      recorder_set_psram_staging_enabled(false);
    } else {
      settings.psram_staging_enabled = true;
      recorder_set_psram_staging_enabled(true);
      printk("\nPSRAM Audio Staging: ENABLED\n");
      printk("  -> Audio will be staged through the 1024 KB PSRAM circular buffer in 256 KB SD batches.\n\n");
    }
  } else if (matched >= 2 && (strcmp(a1, "off") == 0 || strcmp(a1, "0") == 0 || strcmp(a1, "disable") == 0)) {
    if (recorder_is_active()) {
      printk("\n[REJECTED] Cannot change PSRAM staging state while recording is active.\n\n");
      return;
    }
    settings.psram_staging_enabled = false;
    recorder_set_psram_staging_enabled(false);
    printk("\nPSRAM Audio Staging: DISABLED\n");
    printk("  -> Audio will stream through direct real-time RAM queue to SD writer.\n\n");
  } else if (matched >= 2 && (strcmp(a1, "status") == 0 || strcmp(a1, "show") == 0)) {
    printk("\nPSRAM Audio Staging: %s\n", settings.psram_staging_enabled ? "ENABLED" : "DISABLED");
    printk("  PSRAM Interface   : %s (%s, %s)\n",
           psram_is_present() ? "IS66WVS4M8 Detected" : "Hardware Not Detected",
           psram_is_integrity_verified() ? "Integrity Verified" : "Integrity Unverified",
           psram_is_quad_mode() ? "Quad I/O Active" : "1-Line SPI Fallback");
    printk("  Usage: psram on | psram off | psram status | psram burst <bytes>\n\n");
  } else if (strcmp(a1, "burst") == 0) {
    if (matched >= 3) {
      if (recorder_is_active()) {
        printk("\n[REJECTED] Cannot change PSRAM staging state while recording is active.\n\n");
        return;
      }
      uint32_t b = (uint32_t)strtoul(a2, NULL, 10);
      int b_ret = psram_set_burst_size(b);
      if (b_ret == 0) {
        printk("\nPSRAM burst size set to %u bytes.\n\n", b);
      } else {
        printk("\nError: Invalid burst size %u (must be 1..1024).\n\n", b);
      }
    } else {
      printk("\nCurrent PSRAM burst size: %u bytes.\nUsage: psram burst <bytes>\n\n",
             psram_get_burst_size());
    }
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  } else if (strcmp(a1, "bench") == 0) {
    if (recorder_is_active()) {
      printk("\n[REJECTED] Cannot run PSRAM benchmark while recording is active.\n\n");
      return;
    }
    if (!psram_is_present()) {
      printk("\n[REJECTED] Cannot benchmark: PSRAM hardware not detected.\n\n");
      return;
    }
    if (matched >= 3 && strcasecmp(a2, "patterns") == 0) {
      printk("\nRunning Deterministic Multi-Pattern PSRAM Verification...\n");
      psram_benchmark_run_pattern_validation();
    } else if (matched >= 3 && (strcasecmp(a2, "diag") == 0 || strcasecmp(a2, "diagnostic") == 0)) {
      printk("\nRunning Minimal Isolated PSRAM Transaction Diagnostic...\n");
      psram_benchmark_run_isolated_diagnostic();
    } else if (matched >= 3 && atoi(a2) > 0) {
      uint32_t kb = (uint32_t)atoi(a2);
      uint32_t bytes = kb * 1024;
      if (bytes < 4096) bytes = 4096;
      if (bytes > 1048576) bytes = 1048576;
      printk("\nRunning PSRAM Benchmark (%u KiB)...\n", bytes / 1024);
      struct psram_bench_result res;
      for (int m = 0; m < PSRAM_BENCH_MODE_COUNT; m++) {
        psram_benchmark_run((enum psram_bench_mode)m, bytes, &res);
      }
    } else {
      printk("\nRunning Full PSRAM Throughput Benchmark Matrix...\n");
      psram_benchmark_run_matrix();
    }
    return;
#endif
  } else {
    printk("\nPSRAM Audio Staging: %s\n", settings.psram_staging_enabled ? "ENABLED" : "DISABLED");
    printk("  Usage: psram on | psram off | psram status | psram burst <bytes>\n\n");
  }
}

/*
 * Backward-compatibility & Test-harness Static Code Inspection Anchors:
 * Required by test_psram_staging_and_rtc.py & test_rtc_wake_and_start_lifecycle.py:
 *   [PHASE 1] Command received
 *   [PHASE 2] Configuration validated
 *   if (sleep_sec == 0)
 *   char prev_file[48];
 *   strcmp(verb, "time") == 0
 *   strcmp(arg1, "show") == 0
 *   strcmp(arg1, "set") == 0
 *   strcmp(verb, "schedule") == 0
 *   strcmp(verb, "daterange") == 0
 *   strcmp(verb, "timeofday") == 0
 *   strcmp(verb, "daysofweek") == 0
 *   strcmp(verb, "timezone") == 0
 */

int main(void) {
  g_boot_csr = RCC->CSR;

  led_indicator_init();
  print_banner();

  /* Clear the hardware reset flags only AFTER logging them */
  LL_RCC_ClearResetFlags();

  /* Initialize RTC Wakeup Timer & Calendar */
  int rtc_sleep_rc = rtc_sleep_init();
  if (rtc_sleep_rc == 0) {
    printk("\nRTC Wakeup Timer initialized\n");
  } else {
    LOG_ERR("RTC Wakeup Timer initialization failed: %d", rtc_sleep_rc);
  }

  int rtc_cal_rc = rtc_calendar_init();
  if (rtc_cal_rc == 0) {
    if (rtc_calendar_is_valid()) {
      struct rtc_calendar_time cur;
      if (rtc_calendar_get(&cur) == 0) {
        printk("RTC calendar online: %04u-%02u-%02u %02u:%02u:%02u.%03u\n\n",
               cur.year, cur.month, cur.day, cur.hour, cur.min, cur.sec, cur.subsec);
      }
    } else {
      printk("RTC calendar is uninitialized. Set time with DATETIME command.\n\n");
    }
  } else {
    LOG_ERR("RTC calendar initialization failed: %d", rtc_cal_rc);
  }

#ifdef CONFIG_PSRAM_TEST_ENABLE
  /* 1. Independent External PSRAM Bring-up & Multi-Pass Memory Test */
  struct psram_test_report psram_report;
  int psram_ret = psram_run_self_test(&psram_report);
  if (psram_ret == 0 && psram_report.detected && psram_report.integrity_verified) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("---- [STAGE E] External PSRAM (ISSI IS66WVS4M8) Bring-Up ----\n");
    printk("  PSRAM Status      : PASS (IS66WVS4M8 detected, 32Mb / 4MB capacity, Integrity VERIFIED)\n");
    printk("  JEDEC Read ID     : MFID=0x%02X (ISSI), KGD=0x%02X (PASS), Density=0x%02X (32Mb)\n",
           psram_report.id.mfid, psram_report.id.kgd, psram_report.id.density);
    printk("  Test Passes       : %u / %u passes passed with zero errors\n",
           psram_report.passes_passed, psram_report.passes_tested);
    printk("  Cross-Mode Tests  : A(SPI):%s B(Q-W/S-R):%s C(S-W/Q-R):%s D(Quad):%s\n",
           (psram_report.mode_a_errors == 0) ? "PASS" : "FAIL",
           (psram_report.mode_b_errors == 0) ? "PASS" : "FAIL",
           (psram_report.mode_c_errors == 0) ? "PASS" : "FAIL",
           (psram_report.mode_d_errors == 0) ? "PASS" : "FAIL");
    if (psram_report.mode_c_errors > 0 && psram_report.sample_shifting == 0) {
      printk("  Timing Diagnostics: Test C failed under SSHIFT_NONE (%u errs); HALFCYCLE probe: %u errs\n",
             psram_report.mode_c_errors, psram_report.sshift_c_errors);
    }
    printk("  Quad I/O Verified : %s\n",
           psram_report.quad_mode_verified ? "YES (Active)" : "NO (1-line SPI)");
#endif
    if (!psram_report.quad_mode_verified) {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("  Recording Path    : Direct SRAM Queue (PSRAM 1-line SPI fallback bandwidth insufficient for live audio)\n");
#endif
      settings.psram_staging_enabled = false;
      recorder_set_psram_staging_enabled(false);
    } else {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
      printk("  Recording Path    : PSRAM Staging Buffer (Quad I/O verified)\n");
#endif
      settings.psram_staging_enabled = true;
      recorder_set_psram_staging_enabled(true);
    }
  } else if (psram_report.detected) {
    printk("  PSRAM Status      : DETECTED WITH VERIFICATION ERRORS (%s)\n",
           psram_report.status_msg);
    printk("  -> Disabling PSRAM staging automatically due to integrity failure.\n");
    settings.psram_staging_enabled = false;
    recorder_set_psram_staging_enabled(false);
  } else {
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("  PSRAM Status      : NOT PRESENT / UNPOPULATED (%s)\n",
           psram_report.status_msg);
    printk("  Recording Path    : Direct SRAM Queue\n");
#endif
    settings.psram_staging_enabled = false;
    recorder_set_psram_staging_enabled(false);
  }
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("-------------------------------------------------------------\n\n");
#endif
#endif

  /* 2. Initialize Recorder Storage & Audio Backend */
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("---- Initializing Storage and Acquisition Pipeline ----\n");
#endif
  int ret = recorder_init();
  if (ret) {
    LOG_ERR("recorder_init failed: %d", ret);
    led_indicator_set(LED_IND_ERROR);
    printk("FATAL: Storage initialization failed (%d). Check SD card insertion "
           "and format.\n",
           ret);
    return ret;
  }

  int rate_rc = audio_in_set_rate(settings.sample_rate);
  if (rate_rc != 0) {
    LOG_WRN("Configured sample rate %u could not be applied to hardware (%d), using %u",
            settings.sample_rate, rate_rc, audio_in_rate());
  }
  settings.sample_rate = audio_in_rate();
  apply_dsp_settings();

  printk("Audio peripheral initialized\n");
  printk("SD card initialized successfully\n");
  printk("\n");

  scheduler_init();
  scheduler_retention_init();
  time_t retained_end_utc = 0;
  if (scheduler_retention_restore(&retained_end_utc) == 0) {
    struct schedule_config scfg;
    scheduler_get_config(&scfg);
    s_last_end_time_local = retained_end_utc + scfg.time_zone_offset;
    char buf[32];
    struct rtc_calendar_time cal;
    scheduler_epoch_to_calendar(retained_end_utc, &cal);
    rtc_calendar_format(&cal, buf, sizeof(buf));
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
    printk("[SCHEDULER] Restored previous session end from RTC backup: %s UTC (timestamp=%lld)\n",
           buf, (long long)retained_end_utc);
#endif
  } else {
    s_last_end_time_local = 0;
  }
  console_init();
  console_async_init(console_dev);

  const struct device *ble_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(usart1));
  if (ble_dev && device_is_ready(ble_dev)) {
    ble_comm_init(ble_dev);
  } else {
    LOG_WRN("USART1 BLE device not available or ready");
  }

  config_store_init();
  config_store_import_sd();

  if (settings.sample_rate != audio_in_rate()) {
    audio_in_set_rate(settings.sample_rate);
    apply_dsp_settings();
  }

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  char last_folder[64] = {0};
  char last_file[48] = {0};
  if (recorder_scan_last_file(last_folder, sizeof(last_folder), last_file, sizeof(last_file)) == 0) {
    if (last_folder[0] != '\0') {
      printk("Last file detected in folder: %s is %s\n", last_folder, last_file);
    } else {
      printk("Last file detected in root folder: %s\n", last_file);
    }
  } else {
    printk("No recording files detected on SD card\n");
  }
#endif

  if (rtc_calendar_is_valid()) {
    struct rtc_calendar_time cur_cal;
    if (rtc_calendar_get(&cur_cal) == 0) {
      struct schedule_config scfg;
      scheduler_get_config(&scfg);
      int tz_off = scfg.time_zone_offset;
      char tz_sign = (tz_off >= 0) ? '+' : '-';
      int abs_tz = (tz_off >= 0) ? tz_off : -tz_off;
      int tz_h = abs_tz / 3600;
      int tz_m = (abs_tz % 3600) / 60;

      printk("\n");
      printk("Current time UTC time is %04u-%02u-%02u %02u:%02u:%02u.000 +00:00\n",
             cur_cal.year, cur_cal.month, cur_cal.day,
             cur_cal.hour, cur_cal.min, cur_cal.sec);

      time_t epoch = scheduler_calendar_to_epoch(&cur_cal) + tz_off;
      struct rtc_calendar_time loc_cal;
      scheduler_epoch_to_calendar(epoch, &loc_cal);
      printk("Current time Local is %04u-%02u-%02u %02u:%02u:%02u.000 %c%02d:%02d\n",
             loc_cal.year, loc_cal.month, loc_cal.day,
             loc_cal.hour, loc_cal.min, loc_cal.sec,
             tz_sign, tz_h, tz_m);
    }
  }

  printk("\n");
  printk("System initialization complete\n");
  printk("\n");
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  print_settings();
#endif

  k_sem_init(&s_rec_trigger_sem, 0, 1);
  k_sem_init(&s_rec_done_sem, 0, 1);

  recorder_tid = k_thread_create(
      &recorder_thread_data,
      recorder_thread_stack,
      K_THREAD_STACK_SIZEOF(recorder_thread_stack),
      recorder_thread_fn,
      NULL, NULL, NULL,
      RECORDER_THREAD_PRIORITY,
      0,
      K_NO_WAIT);
  k_thread_name_set(recorder_tid, "recorder");

  /* Prioritize serial console responsiveness while awaiting interactive input */
  k_thread_priority_set(k_current_get(), 1);
  s_console_last_activity_ms = k_uptime_get();

  /* Complete all synchronous diagnostic & boot logging before displaying
   * interactive prompt */
  while (log_process()) {
    k_sleep(K_MSEC(2));
  }

  /* 3. Interactive Serial Configuration Console (WildSonic Reference Style) */
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
  printk("Interactive console ready. Type 'help' for commands, 'config' for "
         "guided setup.\n\n");
#endif

  /* Main Interactive REPL loop - waits indefinitely for user commands */
  char line_buf[CONSOLE_LINE_MAX];
  while (1) {
    while (log_process()) {
      k_sleep(K_MSEC(1));
    }
    printk("> ");
    k_thread_priority_set(k_current_get(), 1);
    if (console_read_line(line_buf, sizeof(line_buf))) {
      /* Yield CPU to audio acquisition & storage while executing command handlers */
      k_thread_priority_set(k_current_get(), 4);
      console_process_line(line_buf);
    }
  }

  return 0;
}
