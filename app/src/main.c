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
#if CONFIG_AUDIO_BENCHMARK
#include "diagnostics/benchmark.h"
#endif
#include "dsp.h"
#include "psram_is66wvs.h"
#include "psram_staging.h"
#include "recorder.h"
#include "rtc_calendar.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define SAMPLE_BYTES (CONFIG_AUDIO_SAMPLE_BITS / 8)
#define CONSOLE_LINE_MAX 128

static const struct device *const console_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Runtime Configuration Settings */
struct app_settings {
  uint32_t sample_rate;        /* 16000, 32000, 44100, 48000, 96000, 192000 */
  uint32_t record_seconds;     /* Default 20 s */
  uint32_t sleep_seconds;      /* Default 60 s */
  uint32_t cycles;             /* Default 1, 0 = continuous */
  uint32_t monitor_seconds;    /* Default 10 s */
  float digital_gain;          /* Default 1.0x */
  enum recorder_mode mode;     /* REC_MODE_NORMAL, REC_MODE_COMPARE, etc. */
  bool dsp_enabled;            /* true = active filtering, false = raw */
  bool dc_block_enabled;       /* Stage 1 */
  bool hpf_enabled;            /* Stage 2 */
  uint32_t hpf_cutoff_hz;      /* Default 80 Hz */
  bool notch_enabled;          /* Stage 3 */
  uint32_t notch_freq_hz;      /* 50 or 60 Hz */
  float notch_q;               /* Default 10.0f */
  bool harmonic_notch_enabled; /* Stage 4 */
  bool lpf_enabled;            /* Stage 5 */
  uint32_t lpf_cutoff_hz;      /* Default 20000 Hz */
  bool psram_staging_enabled;  /* true = PSRAM circular buffer, false = direct
                                  RAM queue */
};

static struct app_settings settings = {
    .sample_rate = CONFIG_AUDIO_SAMPLE_RATE,
    .record_seconds = 20,
    .sleep_seconds = 60,
    .cycles = 1,
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
  if (!console_dev || !device_is_ready(console_dev)) {
    return -ENODEV;
  }
  return uart_poll_in(console_dev, c);
}

/* Check if user requested stop via UART ('q', Ctrl+C, ESC, or "stop") */
bool console_check_abort_requested(void) {
  static char stop_buf[8];
  static size_t stop_pos = 0;
  unsigned char c;

  while (console_poll_char(&c) == 0) {
    if (c == 'q' || c == 'Q' || c == 0x03 || c == 0x1B) {
      stop_pos = 0;
      return true;
    }
    if (c == '\r' || c == '\n') {
      stop_buf[stop_pos] = '\0';
      if (strcasecmp(stop_buf, "stop") == 0) {
        stop_pos = 0;
        return true;
      }
      stop_pos = 0;
    } else if (stop_pos < sizeof(stop_buf) - 1) {
      stop_buf[stop_pos++] = (char)c;
    }
  }
  return false;
}

static bool console_read_line(char *buf, size_t max_len) {
  size_t pos = 0;
  static int64_t last_cr_time = 0;

  while (1) {
    unsigned char c;
    if (console_poll_char(&c) == 0) {
      /* If \n follows \r within 100 ms (CRLF sequence), discard the trailing \n
       */
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
        while (pos > 0) {
          printk("\b \b");
          pos--;
        }
        buf[0] = '\0';
      } else if (c >= 32 && c < 127) {
        if (pos < max_len - 1) {
          buf[pos++] = (char)c;
          printk("%c", c);
        }
      }
    } else {
      /* When user is idle at the prompt (pos == 0), flush pending logs without
       * corrupting input */
      if (pos == 0 && log_process()) {
        printk("> ");
      }
      k_sleep(K_MSEC(10));
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
  case REC_MODE_COMPARE:
    return "COMPARE (Cycle 1 Raw, Cycle 2 Filtered)";
  case REC_MODE_STRESS:
    return "STRESS TEST (5s Record / 3s Sleep x3)";
  case REC_MODE_TEST_SIGNAL:
    return "TEST SIGNAL (Synthetic Calibration Tone)";
  default:
    return "UNKNOWN";
  }
}

static void print_reset_reason(void) {
  uint32_t csr = RCC->CSR;
  printk("  MCU Reset Reason  : [0x%08X] ", csr);
  bool known = false;
  if (LL_RCC_IsActiveFlag_LPWRRST()) {
    printk("Low-Power Reset; ");
    known = true;
  }
  if (LL_RCC_IsActiveFlag_WWDGRST()) {
    printk("Window Watchdog Reset (WWDG); ");
    known = true;
  }
  if (LL_RCC_IsActiveFlag_IWDGRST()) {
    printk("Independent Watchdog Reset (IWDG); ");
    known = true;
  }
  if (LL_RCC_IsActiveFlag_SFTRST()) {
    printk("Software Reset (SystemReset / Fatal Error); ");
    known = true;
  }
  if (LL_RCC_IsActiveFlag_BORRST()) {
    printk("Brownout Reset (BOR) / Power-On; ");
    known = true;
  }
  if (LL_RCC_IsActiveFlag_PINRST()) {
    printk("External Reset Pin (NRST); ");
    known = true;
  }
  if (LL_RCC_IsActiveFlag_OBLRST()) {
    printk("Option Byte Loader Reset; ");
    known = true;
  }
  if (LL_RCC_IsActiveFlag_FWRST()) {
    printk("Firewall Reset; ");
    known = true;
  }
  if (!known) {
    printk("Unknown / None; ");
  }
  printk("\n");
  LL_RCC_ClearResetFlags();
}

static void print_banner(void) {
  printk("\n=========================================================================\n");
  printk("      WildSonic STM32L476RG Audio Recorder — Unified Firmware             \n");
  printk("=========================================================================\n");
  printk("  Target MCU        : STM32L476RG (Cortex-M4F @ 72 MHz, FPU Enabled)\n");
  print_reset_reason();
  printk("  Storage Interface : Native SDMMC 4-bit bus, exFAT Filesystem\n");
  printk("  Microphone Front  : MAX4466 Electret Preamp -> ADC1_IN5 (PA0)\n");
  printk("  Acquisition Engine: ADC1 + TIM6 TRGO + DMA1 Ch1 Circular (Zero Jitter)\n");
  printk("  DSP Architecture  : Direct Form II Transposed Biquads (Hardware FPU)\n");
  printk("=========================================================================\n\n");
}

static void print_help(void) {
  printk("\nList of commands\n");
  printk(" HELP          : Display list of commands and usage examples\n");
  printk(" SHOW          : Display all current configuration settings\n");
  printk(" CONFIG        : Enter interactive guided configuration menu\n");
  printk(
      " START         : Start audio recording session with current settings\n");
  printk(
      " STOP          : Stop active recording safely and finalize WAV file\n");
  printk(
      " STATUS        : Show recording, storage, DMA, queue, and DSP status\n");
  printk(" MONITOR [sec] : Live microphone ADC diagnostic monitor (default 10 "
         "s)\n");
  printk(" DIAGNOSE [sec]: Low-overhead intermediate stage isolation "
         "diagnostics (A..E)\n");
  printk(" COMPARE       : Run raw-vs-filtered comparison (Cycle 1 Raw, Cycle "
         "2 Filtered)\n");
  printk(" STRESS        : Run rapid multi-cycle stress test (5s record / 3s "
         "sleep x3)\n");
  printk(" SAMF / RATE   : Set sampling frequency in Hz (16k, 32k, 44.1k, 48k, "
         "96k, 192k)\n");
  printk(" RECT / RECORD : Set recording duration in seconds\n");
  printk(" SLEEP         : Set sleep duration between sessions in seconds\n");
  printk(" CYCLES        : Set cycle count (0 = continuous)\n");
  printk(
      " GAIN          : Set software digital gain multiplier (0.1 to 16.0)\n");
  printk(" FILTER        : Configure DSP filters (on, raw, notch, hpf, lpf)\n");
  printk(" MODE          : Set operating mode (normal, raw, compare, stress, "
         "test)\n");
  printk(
      " TIME SHOW     : Display current hardware RTC calendar date and time\n");
  printk(" TIME SET <..> : Set hardware RTC calendar (YYYY-MM-DD HH:MM:SS)\n");
  printk(" CONFIG PSRAM  : Set PSRAM audio staging (config psram on / off)\n");
  printk(" PSRAM ON/OFF  : Enable/disable PSRAM circular staging buffer\n");
  printk(" PSRAM TEST    : Run complete multi-pass PSRAM hardware memory integrity test\n");
  printk(" PSRAM DIAG    : Run exhaustive 10-pattern multi-address diagnostic matrix\n");
  printk(" PSRAM SSHIFT  : Query or set QSPI sample shifting (psram sshift 0/1)\n");
  printk(" PSRAM [bench] : Benchmark PSRAM read/write throughput and 192 kHz margin\n");
  printk(" INTEGRITY REPLAY <s>: Test-only deterministic PCM to WAV/SD readback CRC (ADC build option required)\n");
  printk(" BENCH [cmd]   : Performance benchmarks (ram, cpu, adc, psram, sd, pipeline, status)\n");
  printk(" DEFAULT       : Reset all configuration parameters to factory "
         "defaults\n");
  printk("\nUsage Examples:\n");
  printk("  config          -> Guided step-by-step interactive setup wizard\n");
  printk("  config psram on -> Enable PSRAM circular staging buffer for "
         "recordings\n");
  printk("  config psram off-> Disable PSRAM staging (direct RAM queue to SD "
         "writer)\n");
  printk("  psram on / off  -> Quick toggle for PSRAM audio staging buffer\n");
#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS
  printk("  psram diag      -> Run exhaustive 10-pattern PSRAM diagnostic matrix\n");
  printk("  psram sshift 1  -> Enable QSPI sample shifting (HALFCYCLE)\n");
  printk("  psram sshift 0  -> Disable QSPI sample shifting (standard NONE)\n");
#endif
#if CONFIG_AUDIO_BENCHMARK
  printk("  bench ram       -> Internal SRAM read/write/copy benchmark\n");
  printk("  bench cpu       -> Integer, floating-point, and DSP workload benchmark\n");
  printk("  bench adc       -> 20-second-per-rate ADC/DMA benchmark\n");
  printk("  bench psram     -> External PSRAM read/write/copyback benchmark\n");
  printk("  bench sd        -> Quick MicroSD benchmark (4K, 16K, 64K; 512 KiB file; compact summary)\n");
  printk("  bench sd medium -> Medium MicroSD benchmark (4K, 16K, 64K; 4 MiB file; 1 warmup + 3 measured runs)\n");
  printk("  bench sd full   -> Full MicroSD benchmark (512B..64KB; 8 MiB file; 5 repetitions; opt-in)\n");
  printk("  bench pipeline  -> Synthetic paced SD drain simulation (not live ADC/DSP)\n");
  printk("  bench status    -> Summary of all memory and pipeline benchmarks\n");
#endif
#if CONFIG_AUDIO_INTEGRITY_REPLAY
  printk("  integrity replay 3 -> 3-second deterministic PCM integrity test (test build only)\n");
  printk("  integrity adc 3    -> 3-second live 192 kHz ADC/DSP/SD CRC audit (test build only)\n");
#endif
  printk("  start           -> Start recording with active profile\n");
  printk("  time show       -> Display current RTC date and time\n");
  printk("  time set 2026-10-21 12:00:00 -> Set RTC calendar date and time\n");
  printk("  rate 96000      -> Switch sample rate to 96 kHz\n");
  printk("  gain 2.0        -> Set digital gain to 2.0x (+6 dB)\n");
  printk("  monitor 15      -> Monitor microphone for 15 seconds\n");
  printk(
      "  stop            -> Abort recording or press 'q' during capture\n\n");
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
  printk("====================================================================="
         "=========\n\n");
}

static void reset_to_defaults(void) {
  settings.sample_rate = 192000;
  settings.record_seconds = 20;
  settings.sleep_seconds = 60;
  settings.cycles = 1;
  settings.monitor_seconds = 10;
  settings.digital_gain = 1.0f;
  settings.mode = REC_MODE_NORMAL;
  settings.dsp_enabled = true;
  settings.dc_block_enabled = true;
  settings.hpf_enabled = true;
  settings.hpf_cutoff_hz = 80;
  settings.notch_enabled = false;
  settings.notch_freq_hz = 50;
  settings.notch_q = 10.0f;
  settings.harmonic_notch_enabled = false;
  settings.lpf_enabled = true;
  settings.lpf_cutoff_hz = 20000;
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
    printk("[7/7] Operating Profile / Test Mode\n");
    printk("      1: Normal Recording (Production Filtered)\n");
    printk("      2: Raw ADC Recording (Unfiltered)\n");
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
    printk("      3: Compare Mode (Cycle 1 Raw, Cycle 2 Filtered)\n");
    printk(
        "      4: Rapid Multi-Cycle Stress Test (5s record / 3s sleep x3)\n");
#endif
#if CONFIG_AUDIO_SRC_SYNTH
    printk("      5: Synthetic Test Signal Tone\n");
#endif
    printk("      Enter selection [%s] [%u]: ",
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS && CONFIG_AUDIO_SRC_SYNTH
           "1-5",
#elif CONFIG_AUDIO_STAGE_DIAGNOSTICS
           "1-4",
#elif CONFIG_AUDIO_SRC_SYNTH
           "1-2, 5",
#else
           "1-2",
#endif
           staged.mode == REC_MODE_NORMAL    ? 1
           : staged.mode == REC_MODE_RAW     ? 2
           : staged.mode == REC_MODE_COMPARE ? 3
           : staged.mode == REC_MODE_STRESS  ? 4
                                             : 5);
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
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
    } else if (strcmp(buf, "3") == 0) {
      staged.mode = REC_MODE_COMPARE;
      break;
    } else if (strcmp(buf, "4") == 0) {
      staged.mode = REC_MODE_STRESS;
      break;
#endif
#if CONFIG_AUDIO_SRC_SYNTH
    } else if (strcmp(buf, "5") == 0) {
      staged.mode = REC_MODE_TEST_SIGNAL;
      break;
#endif
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
        printk("      Run 'psram test' or 'bench psram' to verify hardware memory integrity.\n");
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

static void print_status(void) {
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
  printk("  Hardware Registers    :\n");
  audio_in_dump_registers();
  printk("====================================================================="
         "=====\n\n");
}

static void run_monitor(uint32_t seconds) {
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

static void execute_recording_sessions(void) {
  apply_dsp_settings();

  uint32_t record_sec = settings.record_seconds;
  uint32_t sleep_sec = settings.sleep_seconds;
  uint32_t max_cycles = settings.cycles;
  enum recorder_mode mode = settings.mode;

  if (mode == REC_MODE_COMPARE) {
    record_sec = (settings.record_seconds > 0) ? settings.record_seconds : 20;
    sleep_sec = 2;
    max_cycles = 2;
    printk("Starting Comparison Mode: Cycle 1 = RAW, Cycle 2 = FILTERED (%u s "
           "each)\n",
           record_sec);
  } else if (mode == REC_MODE_STRESS) {
    record_sec = 5;
    sleep_sec = 3;
    max_cycles = 3;
    printk("Starting Rapid Stress Test: 3 cycles of 5s record / 3s sleep\n");
  }

  uint32_t cycle = 1;
  uint32_t total_blocks_written = 0;
  uint32_t total_blocks_dropped = 0;
  uint64_t total_bytes_written = 0;
  uint32_t successful_cycles = 0;
  char prev_file[48] = {0};

  uint32_t act_rate = 0, div = 0;
  int err_hz = 0;
  audio_in_is_rate_supported(settings.sample_rate, &act_rate, &div, &err_hz);
  printk("[PHASE 2] Configuration validated: %u Hz, %u-bit mono, %u s\n",
         act_rate, CONFIG_AUDIO_SAMPLE_BITS, record_sec);

  print_settings();

  while (max_cycles == 0 || cycle <= max_cycles) {
    const uint64_t want_samples = (uint64_t)record_sec * settings.sample_rate;
    const uint64_t want_bytes = want_samples * SAMPLE_BYTES;

    printk("\n================================================================="
           "========\n");
    printk("  RECORDING SESSION %u %s\n", cycle,
           (max_cycles > 0) ? "" : "(Continuous Mode)");
    printk("  Target Format     : %u Hz, %u-bit mono PCM (%u B/s)\n",
           settings.sample_rate, CONFIG_AUDIO_SAMPLE_BITS,
           settings.sample_rate * SAMPLE_BYTES);
    printk("  Session Target    : %u s (Expected: %llu samples / %llu bytes)\n",
           record_sec, want_samples, want_bytes);
    printk("  Storage Pipeline  : %s\n",
           settings.psram_staging_enabled
               ? "PSRAM Staging Buffer (1024 KB Circular Store, 256 KB Batches)"
               : "Direct RAM Queue to SD Writer (PSRAM staging BYPASSED)");
    printk(
        "  Control           : Type 'stop' or press 'q' to finalize early\n");
    printk("==================================================================="
           "======\n");

    struct rec_stats st;
    memset(&st, 0, sizeof(st));

    recorder_set_psram_staging_enabled(settings.psram_staging_enabled);
    int ret = recorder_record_session(record_sec, cycle, mode, &st);

    total_blocks_written += st.blocks_written;
    total_blocks_dropped += st.blocks_dropped;
    total_bytes_written += st.bytes_written;

    if (ret != 0 || st.err != 0) {
      int err_code = (ret != 0) ? ret : st.err;
      printk("\n==============================================================="
             "==========\n");
      printk("  SESSION %u ABORTED: Error %d\n", cycle, err_code);
      if (st.bytes_written > 0) {
        printk("  Partial Audio Committed : %.2f s (%llu bytes, %llu samples "
               "stored in %s)\n",
               (double)st.samples_written / st.sample_rate, st.bytes_written,
               st.samples_written, st.file);
        printk("  Session Verdict         : FAIL - RECORDING ABORTED EARLY "
               "WITH PARTIAL DATA COMMITTED\n");
      } else {
        printk("  Session Verdict         : FAIL - CAPTURE FAILED BEFORE DATA "
               "COMMITTED\n");
      }
      printk("================================================================="
             "========\n\n");
    }

    const uint32_t block_ms = (uint32_t)((uint64_t)CONFIG_AUDIO_BLOCK_SAMPLES *
                                         1000 / st.sample_rate);
    int64_t sample_deficit =
        (int64_t)st.expected_samples - (int64_t)st.samples_written;
    bool cycle_passed = (ret == 0 && st.err == 0 && st.blocks_dropped == 0 &&
                         st.dsp_overruns == 0 && sample_deficit == 0 &&
                         st.bytes_written == want_bytes && st.wav_verified);
    if (cycle_passed) {
      successful_cycles++;
    }

    uint32_t ht_cb = 0, tc_cb = 0;
    audio_in_get_callback_counts(&ht_cb, &tc_cb);

    printk("\n---- [SESSION %u REPORT] ----\n", cycle);
    printk("  Output File             : %s (result: %d)\n", st.file, ret);
    printk("  Recording Start Time    : %s\n",
           st.start_time_str[0] ? st.start_time_str : "N/A");
    printk("  Recording End Time      : %s (RTC Time: %s)\n",
           st.end_time_str[0] ? st.end_time_str : "N/A",
           st.rtc_time_valid ? "VALID" : "UNSET/FALLBACK");
    printk("  Configured Audio Format : %u Hz, %u-bit mono PCM (Block Align: "
           "%u B)\n",
           st.sample_rate, st.sample_bits, SAMPLE_BYTES);
    printk("  Measured Elapsed Time   : %u ms (%u.%03u s, monotonic wall-time "
           "%llu us)\n",
           st.elapsed_ms, st.elapsed_ms / 1000, st.elapsed_ms % 1000,
           st.session_wall_us);
    printk("  Effective Capture Rate  : %u Hz (error: %d Hz)\n",
           st.measured_sample_rate,
           (int)st.measured_sample_rate - (int)st.sample_rate);
    printk("  Requested Samples       : %llu\n", st.expected_samples);
    printk(
        "  Acquired Samples (ADC)  : %llu (%u DMA callbacks: HT=%u, TC=%u)\n",
        st.samples_acquired, ht_cb + tc_cb, ht_cb, tc_cb);
    printk("  Accepted Samples (Q)    : %llu\n", st.samples_accepted);
    printk("  Written Samples (SD)    : %llu\n", st.samples_written);
    if (st.samples_dropped > 0) {
      printk("  Dropped Samples (Total) : %llu samples\n", st.samples_dropped);
      if (st.dma_dropped_blocks > 0) {
        printk("    - DMA Pool Starvation : %u blocks (%llu samples)\n",
               st.dma_dropped_blocks, (uint64_t)st.dma_dropped_blocks * CONFIG_AUDIO_BLOCK_SAMPLES);
      }
      if (st.psram_used || st.psram_dropped_samples > 0) {
        printk("    - PSRAM Full Overflow : %u events (%u samples dropped)\n",
               st.psram_full_events, st.psram_dropped_samples);
      }
      if (st.sram_queue_dropped_blocks > 0) {
        printk("    - SRAM Queue Full     : %u blocks (%llu samples)\n",
               st.sram_queue_dropped_blocks, (uint64_t)st.sram_queue_dropped_blocks * CONFIG_AUDIO_BLOCK_SAMPLES);
      }
      if (st.queue_drained_blocks > 0) {
        printk("    - Discarded on Abort  : %u blocks (%u samples)\n",
               st.queue_drained_blocks, st.queue_drained_samples);
      }
    }
    if (st.samples_discarded_tail > 0) {
      printk("  Unused Final Block Tail : %llu samples (ADC stop residual)\n",
             st.samples_discarded_tail);
    }
    uint64_t total_accounted = st.samples_written + st.samples_dropped + st.samples_discarded_tail;
    printk("  Sample Reconciliation   : %llu Acquired = %llu Accounted (%llu Written + %llu Dropped + %llu Tail) %s\n",
           st.samples_acquired, total_accounted, st.samples_written, st.samples_dropped, st.samples_discarded_tail,
           (st.samples_acquired == total_accounted) ? "[RECONCILED: 100% AUDITED]" : "[UNRECONCILED DEFICIT]");
    printk("  Sample Deficit          : %lld samples %s\n", sample_deficit,
           (sample_deficit == 0) ? "[EXACT MATCH: ZERO LOSS/PADDING]"
                                 : "[MISMATCH]");
    printk("  Audio Blocks Processed  : %u blocks\n", st.blocks_processed);
    printk("  Full Blocks Written     : %u blocks (%u samples each)\n",
           st.full_blocks_written, CONFIG_AUDIO_BLOCK_SAMPLES);
    printk("  Partial Blocks Written  : %u blocks (%u valid samples)\n",
           st.partial_blocks_written, st.partial_block_samples);
    printk("  Total Blocks Written    : %u / expected %u\n", st.blocks_written,
           st.expected_blocks);
    printk("  DMA Callback Counts     : HT=%u, TC=%u (Sum=%u)\n", ht_cb, tc_cb,
           ht_cb + tc_cb);
    printk("  ADC/DMA Error Counters  : ADC OVR=%u, DMA errors=%u, acquisition events missed=%u\n",
           st.adc_overruns, st.dma_errors, st.acquisition_events_missed);
    printk("  DMA Event Interval      : min/avg/max=%u/%u/%u us\n",
           st.dma_interval_min_us, st.dma_interval_avg_us,
           st.dma_interval_max_us);
    printk("  Acquisition Samples     : processed=%u lost=%u accounted=%u\n",
           st.acquisition_samples_processed, st.acquisition_samples_lost,
           st.acquisition_samples_accounted);
    printk("  Blocks Dropped          : %u %s\n", st.blocks_dropped,
           (st.blocks_dropped == 0) ? "[PASS: ZERO LOSS]"
                                    : "[FAIL: DROPS DETECTED]");
     printk("  Audio Queue Peak        : %u of %d blocks (%u ms queued)\n",
            st.queue_high_water, CONFIG_AUDIO_NUM_BLOCKS,
            st.queue_high_water * block_ms);
    printk("  Audio Pool Peak         : %u of %d blocks; SD queue peak: %u blocks\n",
           st.pool_high_water, CONFIG_AUDIO_NUM_BLOCKS, st.sd_queue_high_water);
    if (st.psram_used) {
      printk("  PSRAM Staging Buffer    : ACTIVE (%u KB capacity, %u KB SD "
             "batch target)\n",
             st.psram_staging_capacity >> 10, st.sd_batch_bytes >> 10);
      printk("  PSRAM Staging Peak Used : %u bytes (%.1f %% buffer capacity)\n",
             st.psram_high_water_bytes,
             (double)st.psram_high_water_bytes * 100.0 /
                 (double)st.psram_staging_capacity);
      printk("  PSRAM Overflow Events   : %u (Dropped samples: %u)\n",
             st.psram_full_events, st.psram_dropped_samples);
    } else {
      printk("  PSRAM Staging Buffer    : BYPASSED (Direct SRAM -> SD)\n");
    }
    printk("  SD Write Operations     : %u writes (%llu us total write time)\n",
           st.write_count, st.total_write_us);
    printk("  SD Payload per Write    : %u bytes avg\n",
           st.bytes_per_write_avg);
    printk("  SD Write Latency (avg)  : %u us (%u ms)\n", st.avg_write_us,
           st.avg_write_us / 1000);
    printk("  SD Write Latency (worst): %u us (%u ms)\n", st.worst_write_us,
           st.worst_write_us / 1000);
    printk("  SD Write Error          : %d\n", st.sd_write_error);
    printk("  SD Failed Samples       : %llu\n", st.sd_failed_samples);
    printk("  SD Busy Duty Cycle      : %u.%u %% of session wall-time\n",
           st.sd_busy_permille / 10, st.sd_busy_permille % 10);
    printk(
        "  Audio Bytes Written     : %llu bytes (exact target: %llu bytes)\n",
        st.bytes_written, want_bytes);
    printk("  DSP Processing Mode     : %s\n",
           st.dsp_mode_str ? st.dsp_mode_str : "N/A");
    printk("  DSP Block Latency (avg) : %u us per block\n", st.dsp_avg_us);
    printk("  DSP Block Latency (max) : %u us per block\n", st.dsp_worst_us);
    printk("  DSP Deadline Overruns   : %u %s\n", st.dsp_overruns,
           (st.dsp_overruns == 0) ? "[PASS: ZERO OVERRUNS]"
                                  : "[FAIL: OVERRUN DETECTED]");
    printk("  Signal Saturation Audit :\n");
    printk("    - ADC Hardware Input  : Low (GND)=%u, High (3.3V)=%u %s\n",
           st.adc_clip_low_count, st.adc_clip_high_count,
           (st.adc_clip_low_count + st.adc_clip_high_count == 0)
               ? "[CLEAN: NO ADC CLIPPING]"
               : "[INPUT CLIPPED: REDUCE MAX4466 ANALOG TRIMMER GAIN]");
    printk("    - DSP Filter Overshoot: %u events\n",
           st.dsp_filter_overshoot_count);
    printk("    - Digital Gain Clip   : %u events (Gain: %.2fx)\n",
           st.dsp_gain_clipping_count, (double)st.digital_gain);
    printk("    - Total Output Clamped: %u events\n",
           st.digital_clipping_count);
    printk("  WAV Header On-Disk Check: %s\n",
           st.wav_verified
               ? "PASSED (RIFF, fmt, rate, bits, and exact size validated)"
               : "FAILED");
    printk("  Session Verdict         : %s\n",
           cycle_passed
               ? "PASS - 100% SAMPLES CAPTURED WITH ZERO LOSS"
               : "FAIL - AUDIO DROPS, TIMING ERROR, OR WAV VALIDATION FAILED");

    /* Previous file persistence check */
    if (prev_file[0] != '\0') {
      uint64_t prev_expected_bytes =
          (uint64_t)record_sec * st.sample_rate * SAMPLE_BYTES;
      int pv_ret = recorder_verify_file(prev_file, st.sample_rate,
                                        st.sample_bits, prev_expected_bytes);
      printk("  Previous WAV Persistence: %s -> %s\n", prev_file,
             (pv_ret == 0) ? "VERIFIED READABLE & VALID" : "CHECK FAILED");
    }
    snprintf(prev_file, sizeof(prev_file), "%s", st.file);

    if (ret != 0 || st.err != 0) {
      printk("\nSession %u encountered an error. Terminating multi-session "
             "sequence.\n",
             cycle);
      break;
    }

    if (recorder_is_abort_requested()) {
      printk("\nUser requested stop during session %u. Terminating sequence.\n",
             cycle);
      break;
    }

    if (max_cycles > 0 && cycle >= max_cycles) {
      break;
    }

    /* Sleep phase between sessions */
    if (sleep_sec == 0) {
      printk("\n---- [SESSION %u: Sleep interval is 0s; proceeding directly to "
             "next cycle] ----\n",
             cycle);
    } else {
      printk("\n---- [SESSION %u SLEEP PHASE: %u SECONDS] ----\n", cycle,
             sleep_sec);
      printk("  Power Mode              : STM32L476 Stop 2 Mode (VCORE "
             "Low-Power, SRAM Retained)\n");
      printk("  Peripherals Powered Down: TIM6 stopped, ADC1 deep power-down, "
             "DMA1 stopped, SDMMC clock halted\n");
      printk("  External Memory State   : IS66WVS PSRAM in Deep Power Down "
             "(0xB9) / Standby (CE# High)\n");
      printk("  User LED (LD2 / PA5)    : OFF\n");
      printk("  Wakeup Sources          : RTC Wakeup Timer (%u s alarm) or "
             "User Button B1 (PC13)\n",
             sleep_sec);
      printk("  (Press Blue Button B1 on Nucleo or send console abort to wake "
             "early)\n");

      uint32_t wake_overhead_us = 0;
      int sleep_ret = recorder_sleep(sleep_sec, &wake_overhead_us);
      if (sleep_ret == -EINTR || recorder_is_abort_requested() ||
          console_check_abort_requested()) {
        printk("\nSleep phase aborted by user stop request.\n");
        break;
      }
    }

    cycle++;
  }

  printk("\n==================================================================="
         "======\n");
  printk("                RECORDING SUMMARY ACROSS ALL SESSIONS                "
         "    \n");
  printk("  Total Sessions Configured: %u\n",
         (max_cycles > 0) ? max_cycles : cycle);
  printk("  Successful Sessions      : %u / %u\n", successful_cycles,
         (max_cycles > 0) ? max_cycles : cycle);
  uint64_t total_samples_all =
      (settings.sample_rate > 0) ? (total_bytes_written / SAMPLE_BYTES) : 0;
  if (total_bytes_written > 0 && settings.sample_rate > 0) {
    printk("  Total Audio Recorded     : %.2f seconds (%llu samples across %u "
           "sessions)\n",
           (double)total_samples_all / (double)settings.sample_rate,
           total_samples_all, cycle);
  } else {
    printk("  Total Audio Recorded     : 0 seconds (0 samples)\n");
  }
  printk("  Total Blocks Written     : %u\n", total_blocks_written);
  printk("  Total Blocks Dropped     : %u (must be 0)\n", total_blocks_dropped);
  printk("  Total Audio Bytes Stored : %llu bytes (%.2f MB)\n",
         total_bytes_written, (double)total_bytes_written / (1024.0 * 1024.0));
  printk("  Overall Verdict          : %s\n",
         (successful_cycles == ((max_cycles > 0) ? max_cycles : cycle) &&
          total_blocks_dropped == 0)
             ? "PASS - ALL SESSIONS COMPLETED WITH ZERO LOSS"
             : "FAIL - ONE OR MORE SESSIONS REPORTED DROPS OR CORRUPTION");
  printk("---------------------------------------------------------------------"
         "----\n");
  printk("POWER PROFILING DIAGNOSTICS & HARDWARE MEASUREMENT METHODOLOGY:\n");
  printk("  Active / Sleep Current   : NOT MEASURED (no on-board shunt monitor "
         "present)\n");
  printk("  MCU Measurement Guide    : Measure current across JP6 jumper on "
         "NUCLEO-L476RG.\n");
  printk("  Total System Current     : Measure in series with external supply "
         "(E5V/VIN pins).\n");
  printk("====================================================================="
         "====\n\n");
}

static void parse_and_execute_command(char *cmd) {
  /* Trim leading whitespace */
  while (isspace((unsigned char)*cmd))
    cmd++;
  if (*cmd == '\0')
    return;

  /* Trim trailing whitespace */
  char *end = cmd + strlen(cmd) - 1;
  while (end > cmd && isspace((unsigned char)*end))
    *end-- = '\0';

  char verb[32] = {0};
  char arg1[32] = {0};
  char arg2[32] = {0};
  char arg3[32] = {0};

  int matched = sscanf(cmd, "%31s %31s %31s %31s", verb, arg1, arg2, arg3);
  if (matched < 1)
    return;

  /* Convert verb to lowercase for uniform matching */
  for (int i = 0; verb[i]; i++)
    verb[i] = tolower((unsigned char)verb[i]);

#if CONFIG_AUDIO_INTEGRITY_REPLAY
  if (strcmp(verb, "integrity") == 0) {
    if (matched < 3 || (strcmp(arg1, "replay") != 0 &&
                        strcmp(arg1, "adc") != 0)) {
      printk("Usage: integrity replay|adc <seconds> (1..30; test build only)\n");
      return;
    }
    int sec = atoi(arg2);
    if (sec < 1 || sec > 30) {
      printk("Integrity test duration must be 1..30 seconds.\n");
      return;
    }
    bool replay = strcmp(arg1, "replay") == 0;
    if (!replay && audio_in_rate() != 192000U) {
      printk("Live integrity capture requires 192000 Hz. Set 'rate 192' first.\n");
      return;
    }
    struct rec_stats st = {0};
    printk("\n[INTEGRITY] Mode=%s; PSRAM staging=%s\n",
           replay ? "deterministic PCM replay (ADC data substituted; ADC counters are monitoring only)"
                  : "live ADC with production DSP/gain and pre-writer PCM CRC",
           recorder_is_psram_staging_enabled() ? "enabled" : "disabled");
    int rc = recorder_record_session((uint32_t)sec, 1,
                                     replay ? REC_MODE_INTEGRITY_REPLAY
                                            : REC_MODE_INTEGRITY_CAPTURE,
                                     &st);
    printk("[INTEGRITY] File=%s result=%d SD error=%d\n", st.file, rc,
           st.sd_write_error);
    printk("[INTEGRITY] Storage path used=%s\n",
           st.psram_used ? "PSRAM staging" : "direct RAM queue to SD");
    printk("[INTEGRITY] Duration=%u ms file size=%llu bytes WAV structure=%s payload readback CRC performed=%s\n",
           st.elapsed_ms, st.file_size_bytes,
           st.wav_verified ? "valid" : "invalid",
           st.integrity_readback_bytes ? "yes" : "no");
    printk("[INTEGRITY] Reference CRC=%08x pipeline CRC=%08x readback CRC=%08x; bytes=%llu/%llu samples=%llu/%llu blocks=%u/%u; %s\n",
           st.integrity_expected_crc32, st.integrity_pipeline_crc32,
           st.integrity_readback_crc32, st.integrity_readback_bytes,
           st.expected_samples * SAMPLE_BYTES, st.integrity_readback_samples,
           st.expected_samples, st.integrity_readback_blocks,
           st.expected_blocks,
           (rc == 0 && st.integrity_crc_match) ? "PASS" : "FAIL");
    printk("[INTEGRITY] ADC OVR=%u DMA errors=%u missed events=%u; acquired=%llu accepted=%llu written=%llu dropped=%llu residual=%llu\n",
           st.adc_overruns, st.dma_errors, st.acquisition_events_missed,
           st.samples_acquired, st.samples_accepted, st.samples_written,
           st.samples_dropped, st.samples_discarded_tail);
    printk("[INTEGRITY] DMA event interval min/avg/max=%u/%u/%u us; audio queue=%u, pool=%u, SD queue=%u blocks; SD write avg/max=%u/%u us; DSP avg/max=%u/%u us, deadline overruns=%u\n",
           st.dma_interval_min_us, st.dma_interval_avg_us,
           st.dma_interval_max_us, st.queue_high_water, st.pool_high_water,
           st.sd_queue_high_water, st.avg_write_us, st.worst_write_us,
           st.dsp_avg_us, st.dsp_worst_us, st.dsp_overruns);
    return;
  }
#endif

  /* Guard: Prevent configuration alterations if recording is active */
  if (recorder_is_active()) {
    if (strcmp(verb, "stop") == 0) {
      recorder_request_stop();
      printk("\n[Stop signal sent to active recording engine]\n");
    } else {
      printk("\nRecording is currently in progress! Type 'stop' or press 'q' "
             "to abort.\n");
    }
    return;
  }

  if (strcmp(verb, "help") == 0 || strcmp(verb, "?") == 0) {
    print_help();
  } else if (strcmp(verb, "show") == 0) {
    print_settings();
  } else if (strcmp(verb, "config") == 0) {
    for (int i = 0; arg1[i]; i++)
      arg1[i] = tolower((unsigned char)arg1[i]);
    for (int i = 0; arg2[i]; i++)
      arg2[i] = tolower((unsigned char)arg2[i]);
    if (matched >= 2 && strcmp(arg1, "psram") == 0) {
      if (matched >= 3 && (strcmp(arg2, "on") == 0 || strcmp(arg2, "1") == 0 ||
                           strcmp(arg2, "enable") == 0)) {
        if (!psram_is_present()) {
          printk("\n[REJECTED] Cannot enable PSRAM staging: PSRAM hardware not detected.\n\n");
          settings.psram_staging_enabled = false;
          recorder_set_psram_staging_enabled(false);
        } else if (!psram_is_integrity_verified()) {
          printk("\n[REJECTED] Cannot enable PSRAM staging: PSRAM memory integrity has NOT passed self-test.\n");
          printk("  Run 'psram test' or 'bench psram' to verify hardware memory integrity first.\n\n");
          settings.psram_staging_enabled = false;
          recorder_set_psram_staging_enabled(false);
        } else if (!psram_is_quad_mode()) {
          printk("\n[REJECTED] Cannot enable PSRAM staging: Quad I/O is not verified (1-line SPI active).\n");
          printk("  Real-time audio recording at %u Hz requires 384 kB/s throughput (concurrent write + read),\n",
                 settings.sample_rate);
          printk("  which single-line SPI with 4-byte bursts cannot sustain. Direct SRAM queue is selected.\n");
          printk("  To run non-real-time tests, use 'psram test' or 'bench psram'.\n\n");
          settings.psram_staging_enabled = false;
          recorder_set_psram_staging_enabled(false);
        } else {
          settings.psram_staging_enabled = true;
          recorder_set_psram_staging_enabled(true);
          printk("\nPSRAM Audio Staging: ENABLED\n");
          printk("  -> Audio will be staged through the 1024 KB PSRAM circular "
                 "buffer in 256 KB SD batches.\n");
          printk("  -> Setting takes effect on next recording session (persists "
                 "until reboot).\n\n");
        }
      } else if (matched >= 3 &&
                 (strcmp(arg2, "off") == 0 || strcmp(arg2, "0") == 0 ||
                  strcmp(arg2, "disable") == 0)) {
        settings.psram_staging_enabled = false;
        recorder_set_psram_staging_enabled(false);
        printk("\nPSRAM Audio Staging: DISABLED\n");
        printk("  -> Audio will bypass PSRAM staging and stream through direct "
               "real-time RAM queue to SD writer.\n");
        printk("  -> External PSRAM hardware remains initialized in low-power "
               "idle state.\n");
        printk("  -> Setting takes effect on next recording session (persists "
               "until reboot).\n\n");
      } else if (matched == 2 ||
                 (matched >= 3 &&
                  (strcmp(arg2, "status") == 0 || strcmp(arg2, "show") == 0))) {
        printk("\nPSRAM Audio Staging: %s\n",
               settings.psram_staging_enabled ? "ENABLED" : "DISABLED");
        printk("  PSRAM Interface   : %s (%s, %s)\n",
               psram_is_present() ? "IS66WVS4M8 Detected" : "Hardware Not Detected",
               psram_is_integrity_verified() ? "Integrity Verified" : "Integrity Unverified",
               psram_is_quad_mode() ? "Quad I/O Active" : "1-Line SPI Fallback");
        if (settings.psram_staging_enabled) {
          printk("  Selected Path     : IS66WVS4M8 circular staging buffer (1024 "
                 "KB) with 256 KB SD batch writes.\n");
        } else {
          printk("  Selected Path     : Direct bounded RAM queue (16 blocks = 64 "
                 "KB) to background SD writer.\n");
        }
        printk("  Usage: config psram on | config psram off\n\n");
      } else {
        printk("\nError: Invalid argument '%s'. Usage: config psram on | "
               "config psram off\n\n",
               arg2);
      }
    } else {
      run_guided_config();
    }
  } else if (strcmp(verb, "status") == 0) {
    print_status();
  } else if (strcmp(verb, "start") == 0) {
    printk("[PHASE 1] Command received: 'start'\n");
    execute_recording_sessions();
  } else if (strcmp(verb, "stop") == 0) {
    printk("Recording is not currently active.\n");
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
  } else if (strcmp(verb, "compare") == 0) {
    settings.mode = REC_MODE_COMPARE;
    printk("[PHASE 1] Command received: 'compare'\n");
    execute_recording_sessions();
  } else if (strcmp(verb, "stress") == 0) {
    settings.mode = REC_MODE_STRESS;
    printk("[PHASE 1] Command received: 'stress'\n");
    execute_recording_sessions();
#endif
  } else if (strcmp(verb, "default") == 0) {
    reset_to_defaults();
  } else if (strcmp(verb, "monitor") == 0) {
    uint32_t sec = settings.monitor_seconds;
    if (matched >= 2) {
      int val = atoi(arg1);
      if (val > 0 && val <= 3600) {
        sec = (uint32_t)val;
      }
    }
    run_monitor(sec);
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
  } else if (strcmp(verb, "diagnose") == 0 || strcmp(verb, "diag") == 0) {
    uint32_t sec = 5;
    if (matched >= 2) {
      int val = atoi(arg1);
      if (val > 0 && val <= 60) {
        sec = (uint32_t)val;
      }
    }
    recorder_run_stage_diagnostics(sec);
#endif
  } else if (strcmp(verb, "rate") == 0 || strcmp(verb, "samf") == 0) {
    if (matched < 2) {
      printk("\nSampling frequency is set to %u Hz\n", settings.sample_rate);
      return;
    }
    uint32_t r = (uint32_t)strtoul(arg1, NULL, 10);
    if (r == 16)
      r = 16000;
    else if (r == 32)
      r = 32000;
    else if (r == 44 || r == 44100)
      r = 44100;
    else if (r == 48)
      r = 48000;
    else if (r == 96)
      r = 96000;
    else if (r == 192)
      r = 192000;

    uint32_t act = 0, div = 0;
    int err = 0;
    if (!audio_in_is_rate_supported(r, &act, &div, &err)) {
      printk("\nError: Unsupported rate '%s'. Supported: 16000, 32000, 44100, "
             "48000, 96000, 192000 Hz.\n",
             arg1);
      return;
    }
    int res = audio_in_set_rate(r);
    if (res == 0) {
      settings.sample_rate = r;
      apply_dsp_settings();
      printk("\nSampling frequency set to %u Hz (TIM6 div: %u, Error: %d Hz)\n",
             r, div, err);
    } else {
      printk("\nError: Failed to reconfigure audio rate (code %d)\n", res);
    }
  } else if (strcmp(verb, "record") == 0 || strcmp(verb, "rect") == 0) {
    if (matched < 2) {
      printk("\nRecording time is set to %u seconds\n",
             settings.record_seconds);
      return;
    }
    int sec = atoi(arg1);
    if (sec <= 0 || sec > 86400) {
      printk(
          "\nError: Duration %d invalid. Allowed range: 1 to 86400 seconds.\n",
          sec);
      return;
    }
    settings.record_seconds = (uint32_t)sec;
    printk("\nRecording time set to %u seconds.\n", settings.record_seconds);
  } else if (strcmp(verb, "sleep") == 0) {
    if (matched < 2) {
      printk("\nSleep time is set to %u seconds\n", settings.sleep_seconds);
      return;
    }
    int sec = atoi(arg1);
    if (sec < 0 || sec > 86400) {
      printk("\nError: Sleep %d invalid. Allowed range: 0 to 86400 seconds.\n",
             sec);
      return;
    }
    settings.sleep_seconds = (uint32_t)sec;
    printk("\nSleep time set to %u seconds.\n", settings.sleep_seconds);
  } else if (strcmp(verb, "cycles") == 0) {
    if (matched < 2) {
      printk("\nCycle count is set to %u %s\n", settings.cycles,
             (settings.cycles == 0) ? "[Continuous]" : "");
      return;
    }
    int c = atoi(arg1);
    if (c < 0 || c > 10000) {
      printk("\nError: Cycle count must be between 0 and 10000.\n");
      return;
    }
    settings.cycles = (uint32_t)c;
    printk("\nCycle count set to %u %s.\n", settings.cycles,
           (settings.cycles == 0) ? "[Continuous]" : "");
  } else if (strcmp(verb, "gain") == 0) {
    if (matched < 2 || strcmp(arg1, "show") == 0) {
      printk("\nSoftware digital gain is set to %.2fx (%+.1f dB)\n",
             (double)settings.digital_gain,
             20.0 * log10((double)settings.digital_gain));
      printk("Hardware Analog Gain   : Controlled by MAX4466 trimmer "
             "potentiometer.\n");
      printk("                         Use 'monitor 10' to inspect "
             "peak-to-peak voltage.\n");
      return;
    }
    float g = (float)atof(arg1);
    if (g < 0.1f || g > 16.0f) {
      printk("\nError: Digital gain %.2f out of safe range [0.1 .. 16.0]\n",
             (double)g);
      return;
    }
    settings.digital_gain = g;
    dsp_set_digital_gain(g);
    printk("\nSoftware digital gain set to %.2fx (%+.1f dB)\n", (double)g,
           20.0 * log10((double)g));
  } else if (strcmp(verb, "filter") == 0) {
    if (matched < 2) {
      printk("\nDSP Filtering is %s\n",
             settings.dsp_enabled ? "ENABLED" : "BYPASSED");
      if (settings.dsp_enabled) {
        printk("  DC Blocker : %s\n", settings.dc_block_enabled ? "ON" : "OFF");
        printk("  HPF        : %s (%u Hz)\n",
               settings.hpf_enabled ? "ON" : "OFF", settings.hpf_cutoff_hz);
        printk("  Notch      : %s (%u Hz)\n",
               settings.notch_enabled ? "ON" : "OFF", settings.notch_freq_hz);
        printk("  LPF        : %s (%u Hz)\n",
               settings.lpf_enabled ? "ON" : "OFF", settings.lpf_cutoff_hz);
      }
      return;
    }
    for (int i = 0; arg1[i]; i++)
      arg1[i] = tolower((unsigned char)arg1[i]);
    if (strcmp(arg1, "raw") == 0) {
      settings.dsp_enabled = false;
      settings.mode = REC_MODE_RAW;
      apply_dsp_settings();
      printk(
          "\nDSP filtering DISABLED (Mode set to RAW unfiltered ADC PCM).\n");
    } else if (strcmp(arg1, "on") == 0) {
      settings.dsp_enabled = true;
      settings.mode = REC_MODE_NORMAL;
      apply_dsp_settings();
      printk("\nDSP filtering ENABLED (Mode set to PRODUCTION FILTERED).\n");
    } else if (strcmp(arg1, "notch") == 0) {
      if (matched < 3) {
        printk("\nUsage: filter notch <50|60|off>\n");
        return;
      }
      if (strcmp(arg2, "50") == 0) {
        settings.notch_enabled = true;
        settings.notch_freq_hz = 50;
        apply_dsp_settings();
        printk("\nMains notch filter set to 50 Hz (Q=%.1f).\n",
               (double)settings.notch_q);
      } else if (strcmp(arg2, "60") == 0) {
        settings.notch_enabled = true;
        settings.notch_freq_hz = 60;
        apply_dsp_settings();
        printk("\nMains notch filter set to 60 Hz (Q=%.1f).\n",
               (double)settings.notch_q);
      } else if (strcmp(arg2, "off") == 0) {
        settings.notch_enabled = false;
        apply_dsp_settings();
        printk("\nMains notch filter DISABLED.\n");
      } else {
        printk("\nError: Unknown notch argument '%s'. Use 50, 60, or off.\n",
               arg2);
      }
    } else if (strcmp(arg1, "hpf") == 0) {
      if (matched < 3) {
        printk("\nUsage: filter hpf <Hz>\n");
        return;
      }
      int hz = atoi(arg2);
      if (hz < 10 || hz > 2000) {
        printk("\nError: HPF cutoff %d invalid [10 .. 2000 Hz].\n", hz);
        return;
      }
      settings.hpf_cutoff_hz = (uint32_t)hz;
      settings.hpf_enabled = true;
      apply_dsp_settings();
      printk("\nHPF cutoff set to %u Hz (2nd-order Butterworth).\n",
             settings.hpf_cutoff_hz);
    } else if (strcmp(arg1, "lpf") == 0) {
      if (matched < 3) {
        printk("\nUsage: filter lpf <Hz>\n");
        return;
      }
      int hz = atoi(arg2);
      if (hz < 1000 || hz > 40000) {
        printk("\nError: LPF cutoff %d invalid [1000 .. 40000 Hz].\n", hz);
        return;
      }
      settings.lpf_cutoff_hz = (uint32_t)hz;
      settings.lpf_enabled = true;
      apply_dsp_settings();
      printk("\nLPF cutoff set to %u Hz (4th-order Butterworth).\n",
             settings.lpf_cutoff_hz);
    } else {
      printk("\nError: Unknown filter subcommand '%s'. Use on, raw, notch, "
             "hpf, or lpf.\n",
             arg1);
    }
  } else if (strcmp(verb, "mode") == 0) {
    if (matched < 2) {
      printk("\nOperating mode is set to %s\n", mode_to_string(settings.mode));
      return;
    }
    for (int i = 0; arg1[i]; i++)
      arg1[i] = tolower((unsigned char)arg1[i]);
    if (strcmp(arg1, "normal") == 0) {
      settings.mode = REC_MODE_NORMAL;
      settings.dsp_enabled = true;
      printk("\nMode set to NORMAL (Production Filtered).\n");
    } else if (strcmp(arg1, "raw") == 0) {
      settings.mode = REC_MODE_RAW;
      settings.dsp_enabled = false;
      printk("\nMode set to RAW (Unfiltered ADC).\n");
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS
    } else if (strcmp(arg1, "compare") == 0) {
      settings.mode = REC_MODE_COMPARE;
      printk("\nMode set to COMPARE (Cycle 1 Raw, Cycle 2 Filtered).\n");
    } else if (strcmp(arg1, "stress") == 0) {
      settings.mode = REC_MODE_STRESS;
      printk("\nMode set to STRESS (Rapid multi-cycle test).\n");
#endif
#if CONFIG_AUDIO_SRC_SYNTH
    } else if (strcmp(arg1, "test") == 0) {
      settings.mode = REC_MODE_TEST_SIGNAL;
      printk("\nMode set to TEST SIGNAL (Synthetic Tone).\n");
#endif
    } else {
#if CONFIG_AUDIO_STAGE_DIAGNOSTICS && CONFIG_AUDIO_SRC_SYNTH
      printk("\nError: Unknown mode '%s'. Use normal, raw, compare, stress, or "
             "test.\n",
             arg1);
#elif CONFIG_AUDIO_STAGE_DIAGNOSTICS
      printk("\nError: Unknown mode '%s'. Use normal, raw, compare, or stress.\n",
             arg1);
#elif CONFIG_AUDIO_SRC_SYNTH
      printk("\nError: Unknown mode '%s'. Use normal, raw, or test.\n", arg1);
#else
      printk("\nError: Unknown mode '%s'. Use normal or raw.\n", arg1);
#endif
    }
  } else if (strcmp(verb, "time") == 0) {
    for (int i = 0; arg1[i]; i++)
      arg1[i] = tolower((unsigned char)arg1[i]);
    if (matched < 2 || strcmp(arg1, "show") == 0) {
      struct rtc_calendar_time cur_time;
      int ret = rtc_calendar_get(&cur_time);
      if (ret == 0 && rtc_calendar_is_valid()) {
        char tbuf[32];
        rtc_calendar_format(&cur_time, tbuf, sizeof(tbuf));
        printk("\nRTC Calendar Time: %s (Synchronized)\n\n", tbuf);
      } else {
        printk("\nRTC Calendar Time: [NOT SET / UNINITIALIZED]\n");
        printk("Use 'time set YYYY-MM-DD HH:MM:SS' to initialize hardware "
               "calendar.\n\n");
      }
    } else if (strcmp(arg1, "set") == 0) {
      /* Look for start of date string after 'set' */
      char *p = strstr(cmd, "set");
      if (p) {
        p += 3;
        while (*p && isspace((unsigned char)*p))
          p++;
        struct rtc_calendar_time new_time;
        int parse_res = rtc_calendar_parse(p, &new_time);
        if (parse_res == 0) {
          int set_res = rtc_calendar_set(&new_time);
          if (set_res == 0) {
            char tbuf[32];
            rtc_calendar_format(&new_time, tbuf, sizeof(tbuf));
            printk("\n[OK] RTC hardware calendar successfully set to: %s\n\n",
                   tbuf);
          } else {
            printk("\n[ERROR] Failed to write to RTC hardware (error %d)\n\n",
                   set_res);
          }
        } else {
          printk("\n[ERROR] Invalid date/time format '%s'.\n", p);
          printk("Expected format: time set YYYY-MM-DD HH:MM:SS\n");
          printk("Example        : time set 2026-10-21 12:00:00\n\n");
        }
      }
    } else {
      printk("\nUsage: time show OR time set YYYY-MM-DD HH:MM:SS\n\n");
    }
  } else if (strcmp(verb, "psram") == 0) {
    for (int i = 0; arg1[i]; i++)
      arg1[i] = tolower((unsigned char)arg1[i]);
    if (matched >= 2 && (strcmp(arg1, "on") == 0 || strcmp(arg1, "1") == 0 ||
                         strcmp(arg1, "enable") == 0)) {
      if (!psram_is_present()) {
        printk("\n[REJECTED] Cannot enable PSRAM staging: PSRAM hardware not detected.\n\n");
        settings.psram_staging_enabled = false;
        recorder_set_psram_staging_enabled(false);
      } else if (!psram_is_integrity_verified()) {
        printk("\n[REJECTED] Cannot enable PSRAM staging: PSRAM memory integrity has NOT passed self-test.\n");
        printk("  Run 'psram test' or 'bench psram' to verify hardware memory integrity first.\n\n");
        settings.psram_staging_enabled = false;
        recorder_set_psram_staging_enabled(false);
      } else if (!psram_is_quad_mode()) {
        printk("\n[REJECTED] Cannot enable PSRAM staging: Quad I/O is not verified (1-line SPI active).\n");
        printk("  Real-time audio recording at %u Hz requires 384 kB/s throughput (concurrent write + read),\n",
               settings.sample_rate);
        printk("  which single-line SPI with 4-byte bursts cannot sustain. Direct SRAM queue is selected.\n");
        printk("  To run non-real-time tests, use 'psram test' or 'bench psram'.\n\n");
        settings.psram_staging_enabled = false;
        recorder_set_psram_staging_enabled(false);
      } else {
        settings.psram_staging_enabled = true;
        recorder_set_psram_staging_enabled(true);
        printk("\nPSRAM Audio Staging: ENABLED\n");
        printk("  -> Audio will be staged through the 1024 KB PSRAM circular "
               "buffer in 256 KB SD batches.\n");
        printk("  -> Setting takes effect on next recording session (persists "
               "until reboot).\n\n");
      }
    } else if (matched >= 2 &&
               (strcmp(arg1, "off") == 0 || strcmp(arg1, "0") == 0 ||
                strcmp(arg1, "disable") == 0)) {
      settings.psram_staging_enabled = false;
      recorder_set_psram_staging_enabled(false);
      printk("\nPSRAM Audio Staging: DISABLED\n");
      printk("  -> Audio will bypass PSRAM staging and stream through direct "
             "real-time RAM queue to SD writer.\n");
      printk("  -> External PSRAM hardware remains initialized in low-power "
             "idle state.\n");
      printk("  -> Setting takes effect on next recording session (persists "
             "until reboot).\n\n");
    } else if (matched >= 2 &&
               (strcmp(arg1, "status") == 0 || strcmp(arg1, "show") == 0)) {
      printk("\nPSRAM Audio Staging: %s\n",
             settings.psram_staging_enabled ? "ENABLED" : "DISABLED");
      printk("  PSRAM Interface   : %s (%s, %s)\n",
             psram_is_present() ? "IS66WVS4M8 Detected" : "Hardware Not Detected",
             psram_is_integrity_verified() ? "Integrity Verified" : "Integrity Unverified",
             psram_is_quad_mode() ? "Quad I/O Active" : "1-Line SPI Fallback");
      if (settings.psram_staging_enabled) {
        printk("  Selected Path     : IS66WVS4M8 circular staging buffer (1024 KB) "
               "with 256 KB SD batch writes.\n");
      } else {
        printk("  Selected Path     : Direct bounded RAM queue (16 blocks = 64 KB) "
               "to background SD writer.\n");
      }
#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS
      printk("  Usage: psram on | psram off | psram test | psram bench\n\n");
#else
      printk("  Usage: psram on | psram off | psram status | psram burst <bytes>\n\n");
#endif
#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS
    } else if (matched >= 2 &&
               (strcmp(arg1, "test") == 0 || strcmp(arg1, "selftest") == 0)) {
      if (recorder_is_active() || benchmark_is_running()) {
        printk("\nCannot run PSRAM test while recording or another benchmark is active.\n\n");
        return;
      }
      if (matched >= 3 && (strcmp(arg2, "full") == 0 || strcmp(arg2, "matrix") == 0 || strcmp(arg2, "diag") == 0)) {
        benchmark_set_running(true);
        struct psram_diag_report dr;
        psram_run_diagnostic_matrix(true, &dr);
        benchmark_set_running(false);
        return;
      }
      benchmark_set_running(true);
      printk("\nExecuting standalone PSRAM hardware memory test...\n");
      struct psram_test_report tr;
      int tret = psram_run_self_test(&tr);
      (void)tret;
      printk("  PSRAM Detected     : %s\n", tr.detected ? "YES" : "NO");
      printk("  MFID / KGD / Dens  : 0x%02X / 0x%02X / 0x%02X\n", tr.id.mfid,
             tr.id.kgd, tr.id.density);
      printk("  Sample Shifting    : %u (%s)\n", tr.sample_shifting,
             (tr.sample_shifting == 1) ? "HALFCYCLE" : "NONE");
      printk("  Passes Passed      : %u / %u\n", tr.passes_passed,
             tr.passes_tested);
      printk("  Decoupled Cross-Mode Test Summary:\n");
      printk("    - Test A (SPI-W  / SPI-R ) : %s (%u errors)\n",
             (tr.mode_a_errors == 0) ? "PASS" : "FAIL", tr.mode_a_errors);
      printk("    - Test B (Quad-W / SPI-R ) : %s (%u errors)\n",
             (tr.mode_b_errors == 0) ? "PASS" : "FAIL", tr.mode_b_errors);
      printk("    - Test C (SPI-W  / Quad-R) : %s (%u errors)\n",
             (tr.mode_c_errors == 0) ? "PASS" : "FAIL", tr.mode_c_errors);
      printk("    - Test D (Quad-W / Quad-R) : %s (%u errors)\n",
             (tr.mode_d_errors == 0) ? "PASS" : "FAIL", tr.mode_d_errors);
      if (tr.mode_c_errors > 0 && tr.sample_shifting == 0) {
        printk("    - Diagnostic Probe : Test C under HALFCYCLE: %u errors (%s)\n",
               tr.sshift_c_errors,
               (tr.sshift_c_errors == 0) ? "HALFCYCLE eliminates read timing error!"
                                         : "timing error persists");
      }
      printk("  Quad I/O Verified  : %s\n",
             tr.quad_mode_verified ? "YES (Active)" : "NO (1-line SPI)");
      printk("  Error Count        : %u\n", tr.error_count);
      if (tr.error_count != 0) {
        printk("  First Mismatch     : addr=0x%06X expected=0x%02X actual=0x%02X xor=0x%02X\n",
               tr.first_failing_addr, tr.expected_val, tr.observed_val,
               tr.failing_bitmask);
        printk("  Mismatch Context   : ");
        for (uint32_t i = 0; i < tr.ctx_len; i++) {
          printk("%02X/%02X%s", tr.ctx_expected[i], tr.ctx_observed[i],
                 (i + 1U == tr.ctx_len) ? "\n" : " ");
        }
      }
      printk("  Status Message     : %s\n\n", tr.status_msg);
      benchmark_set_running(false);
    } else if (matched >= 2 &&
               (strcmp(arg1, "diag") == 0 || strcmp(arg1, "diagnostic") == 0)) {
      if (recorder_is_active() || benchmark_is_running()) {
        printk("\nCannot run PSRAM diagnostic while recording or another benchmark is active.\n\n");
        return;
      }
      benchmark_set_running(true);
      bool full = (matched >= 3 && (strcmp(arg2, "full") == 0 || strcmp(arg2, "all") == 0));
      struct psram_diag_report dr;
      psram_run_diagnostic_matrix(full, &dr);
      benchmark_set_running(false);
    } else if (matched >= 2 &&
               (strcmp(arg1, "sshift") == 0 || strcmp(arg1, "shift") == 0)) {
      if (matched >= 3) {
        uint32_t s = (uint32_t)strtoul(arg2, NULL, 10);
        uint32_t shift_val = (s != 0) ? 0x10000000U /* QUADSPI_CR_SSHIFT */ : 0U;
        int s_ret = psram_set_sample_shifting(shift_val);
        if (s_ret == 0) {
          printk("\nPSRAM QSPI Sample Shifting set to %u (%s).\n", s,
                 (shift_val != 0) ? "HALFCYCLE" : "NONE");
          printk("Run 'psram test' to re-evaluate Quad I/O under this timing setting.\n\n");
        } else {
          printk("\nError: Failed to set Sample Shifting (%d).\n\n", s_ret);
        }
      } else {
        uint32_t cur_s = psram_get_sample_shifting();
        printk("\nCurrent PSRAM QSPI Sample Shifting: %u (%s)\n", (cur_s != 0) ? 1 : 0,
               (cur_s != 0) ? "HALFCYCLE" : "NONE");
        printk("Usage: psram sshift <0|1>  (0 = NONE, 1 = HALFCYCLE)\n\n");
      }
#endif
#if CONFIG_AUDIO_BENCHMARK
    } else if (matched >= 2 &&
               (strcmp(arg1, "bench") == 0 || strcmp(arg1, "benchmark") == 0)) {
      if (recorder_is_active() || benchmark_is_running()) {
        printk("\nCannot run PSRAM benchmark while recording or another benchmark is active.\n\n");
        return;
      }
      benchmark_set_running(true);
      printk("\nExecuting PSRAM read/write bandwidth and real-time "
             "benchmark...\n");
      struct psram_bench_report br;
      int bret = psram_run_benchmark(&br);
      if (bret == 0 && br.success) {
        printk("  Operating Mode     : %s (Burst: %u bytes)\n",
               br.quad_mode ? "Quad I/O (1-4-4, CMD 0x38/0xEB)"
                            : "Standard 1-line SPI",
               br.burst_size);
        printk("  4096-Byte Block Write : %u us (Throughput: %u KB/s)\n",
               br.write_4k_us, br.write_throughput_kb_s);
        printk("  4096-Byte Block Read  : %u us (Throughput: %u KB/s)\n",
               br.read_4k_us, br.read_throughput_kb_s);
        printk("  Data Integrity Check  : %s\n", br.integrity_verified
                                                     ? "PASS (Zero Bit Errors)"
                                                     : "FAIL (Mismatch)");
        printk("  192 kHz Real-time Budget: %u us arrival deadline per 4096B "
               "block\n",
               br.audio_period_192k_us);
        printk("  Real-time Write Margin: %+d us headroom (PSRAM bus load: "
               "%.1f%%)\n",
               br.write_margin_us,
               (double)br.write_4k_us * 100.0 /
                   (double)br.audio_period_192k_us);
        printk("  Representative Chunks :\n");
        for (int c = 0; c < 4; c++) {
          printk(
              "    - %4u B: Write %5u us (%5u KB/s) | Read %5u us (%5u KB/s)\n",
              br.chunk_sizes[c], br.chunk_write_us[c],
              (br.chunk_write_us[c] > 0)
                  ? (uint32_t)(((uint64_t)br.chunk_sizes[c] * 1000000ULL) /
                               (br.chunk_write_us[c] * 1024ULL))
                  : 0,
              br.chunk_read_us[c],
              (br.chunk_read_us[c] > 0)
                  ? (uint32_t)(((uint64_t)br.chunk_sizes[c] * 1000000ULL) /
                               (br.chunk_read_us[c] * 1024ULL))
                  : 0);
        }
        printk("  Benchmark Verdict  : PASS - tested PSRAM transfers passed integrity; "
               "this is not an end-to-end recording test.\n\n");
      } else {
        printk("  Benchmark Verdict  : FAIL (%s)\n\n", br.status_msg);
      }
      benchmark_set_running(false);
#endif
    } else if (strcmp(arg1, "burst") == 0) {
      if (matched >= 3) {
        uint32_t b = (uint32_t)strtoul(arg2, NULL, 10);
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
    } else if (matched >= 2) {
#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS
      printk("\nError: Unknown PSRAM subcommand '%s'. Usage: psram on | psram "
             "off | psram test | psram bench | psram burst <bytes>\n\n",
             arg1);
#else
      printk("\nError: Unknown PSRAM subcommand '%s'. Usage: psram on | psram "
             "off | psram status | psram burst <bytes>\n\n",
             arg1);
#endif
    } else {
      /* No arguments provided: print current status and usage */
      printk("\nPSRAM Audio Staging: %s\n",
             settings.psram_staging_enabled ? "ENABLED" : "DISABLED");
      if (settings.psram_staging_enabled) {
        printk("  -> Active path: IS66WVS4M8 circular staging buffer (1024 KB) "
               "with 256 KB SD batch writes.\n");
      } else {
        printk("  -> Active path: Direct bounded RAM queue (16 blocks = 64 KB) "
               "to background SD writer.\n");
      }
#if CONFIG_AUDIO_BENCHMARK || CONFIG_AUDIO_STAGE_DIAGNOSTICS
      printk("  Usage: psram on | psram off | psram test | psram bench | psram burst <bytes>\n\n");
#else
      printk("  Usage: psram on | psram off | psram status | psram burst <bytes>\n\n");
#endif
    }
#if CONFIG_AUDIO_BENCHMARK
  } else if (strcmp(verb, "bench") == 0 || strcmp(verb, "benchmark") == 0) {
    for (int i = 0; arg1[i]; i++)
      arg1[i] = tolower((unsigned char)arg1[i]);
    for (int i = 0; arg2[i]; i++)
      arg2[i] = tolower((unsigned char)arg2[i]);
    for (int i = 0; arg3[i]; i++)
      arg3[i] = tolower((unsigned char)arg3[i]);

    if (matched < 2 || strcmp(arg1, "status") == 0 || strcmp(arg1, "show") == 0) {
      bench_print_status();
    } else if (strcmp(arg1, "ram") == 0 || strcmp(arg1, "sram") == 0) {
      bench_run_ram();
    } else if (strcmp(arg1, "cpu") == 0) {
      bench_run_cpu();
    } else if (strcmp(arg1, "adc") == 0 || strcmp(arg1, "dma") == 0) {
      bench_run_adc();
    } else if (strcmp(arg1, "psram") == 0) {
      bench_run_psram();
    } else if (strcmp(arg1, "sd") == 0 || strcmp(arg1, "sdmmc") == 0 || strcmp(arg1, "card") == 0) {
      if (matched >= 3 && strcmp(arg2, "medium") == 0) {
        bench_run_sd_mode(SD_BENCH_MEDIUM);
      } else if (matched >= 3 && strcmp(arg2, "full") == 0) {
        bench_run_sd_mode(SD_BENCH_FULL);
      } else {
        bench_run_sd();
      }
    } else if (strcmp(arg1, "pipeline") == 0 || strcmp(arg1, "pipe") == 0) {
      if (matched >= 3 && strcmp(arg2, "all") == 0) {
        bench_run_pipeline_all();
      } else {
        uint32_t target_rate = 192000;
        if (matched >= 3 && isdigit((unsigned char)arg2[0])) {
          target_rate = (uint32_t)strtoul(arg2, NULL, 10);
        }
        bool use_psram = false;
        if (matched >= 4 && (strcmp(arg3, "ram") == 0 || strcmp(arg3, "sram") == 0)) {
          use_psram = false;
        } else if (matched >= 4 && strcmp(arg3, "psram") == 0) {
          use_psram = true;
        } else {
          use_psram = psram_is_present() && psram_is_integrity_verified();
        }
        bench_run_pipeline(target_rate, use_psram);
      }
    } else {
      printk("\nError: Unknown benchmark subcommand '%s'.\n", arg1);
      printk("Usage: bench [ram | cpu | adc | psram | sd [quick|medium|full] | pipeline [rate] [all] | status]\n\n");
    }
#endif
  } else {
    printk("\nUnknown or BAD command : %s\n", verb);
    printk("Type 'help' for available commands, 'config' for guided setup.\n");
  }
}

int main(void) {
  print_banner();

#ifdef CONFIG_PSRAM_TEST_ENABLE
  /* 1. Independent External PSRAM Bring-up & Multi-Pass Memory Test */
  printk("---- [STAGE E] External PSRAM (ISSI IS66WVS4M8) Bring-Up ----\n");
  struct psram_test_report psram_report;
  int psram_ret = psram_run_self_test(&psram_report);
  if (psram_ret == 0 && psram_report.detected && psram_report.integrity_verified) {
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
    if (!psram_report.quad_mode_verified) {
      printk("  Recording Path    : Direct SRAM Queue (PSRAM 1-line SPI fallback bandwidth insufficient for live audio)\n");
      settings.psram_staging_enabled = false;
      recorder_set_psram_staging_enabled(false);
    } else {
      printk("  Recording Path    : PSRAM Staging Buffer (Quad I/O verified)\n");
      settings.psram_staging_enabled = true;
      recorder_set_psram_staging_enabled(true);
    }
  } else if (psram_report.detected) {
    printk("  PSRAM Status      : DETECTED WITH VERIFICATION ERRORS (%s)\n",
           psram_report.status_msg);
    printk("  -> Disabling PSRAM staging automatically due to integrity failure.\n");
    printk("  Recording Path    : Direct SRAM Queue\n");
    settings.psram_staging_enabled = false;
    recorder_set_psram_staging_enabled(false);
  } else {
    printk("  PSRAM Status      : NOT PRESENT / UNPOPULATED (%s)\n",
           psram_report.status_msg);
    printk("  Recording Path    : Direct SRAM Queue\n");
    settings.psram_staging_enabled = false;
    recorder_set_psram_staging_enabled(false);
  }
  printk("-------------------------------------------------------------\n\n");
#endif

  /* 2. Initialize Recorder Storage & Audio Backend */
  printk("---- Initializing Storage and Acquisition Pipeline ----\n");
  int ret = recorder_init();
  if (ret) {
    LOG_ERR("recorder_init failed: %d", ret);
    printk("FATAL: Storage initialization failed (%d). Check SD card insertion "
           "and format.\n",
           ret);
    return ret;
  }

  apply_dsp_settings();
  print_settings();

  /* Complete all synchronous diagnostic & boot logging before displaying
   * interactive prompt */
  while (log_process()) {
    k_sleep(K_MSEC(2));
  }

  /* 3. Interactive Serial Configuration Console (WildSonic Reference Style) */
  printk("Interactive console ready. Type 'help' for commands, 'config' for "
         "guided setup.\n\n");

  /* Main Interactive REPL loop - waits indefinitely for user commands */
  char line_buf[CONSOLE_LINE_MAX];
  while (1) {
    while (log_process()) {
      k_sleep(K_MSEC(1));
    }
    printk("> ");
    if (console_read_line(line_buf, sizeof(line_buf))) {
      parse_and_execute_command(line_buf);
    }
  }

  return 0;
}
