/* SPDX-License-Identifier: Apache-2.0 */
#ifndef MAIN_H_
#define MAIN_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "recorder.h"
#include "audio_in.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Runtime Configuration Settings */
struct app_settings {
  uint32_t sample_rate;        /* 16000, 32000, 44100, 48000, 96000, 192000, (TEST: 384000, 512000) */
  uint16_t sample_bits;        /* 16 (default), (TEST: 32) */
  uint16_t channels;           /* 1 (default), (TEST: 2) */
  enum audio_capture_source capture_source; /* 0=MIC, 1=STEREO_DUP, 2=SYNTH */
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
  bool psram_staging_enabled;  /* true = PSRAM circular buffer, false = direct RAM queue */
};

/* Core application accessors and operations */
struct app_settings *main_get_settings(void);
void main_apply_dsp_settings(void);
void main_reset_to_defaults(void);
void main_run_scheduled_recording(void);
void main_run_guided_config(void);
void main_psram_command(int matched, const char *arg1, const char *arg2);
void main_scheduler_service_step(void);
void main_scheduler_resume(void);
void main_scheduler_pause(void);
bool main_is_scheduler_paused(void);



#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */

