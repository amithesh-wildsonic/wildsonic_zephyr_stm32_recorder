/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file config_store.h
 * @brief Flash Configuration Persistence and SD Import/Export for WildSonic STM32.
 *
 * Implements ping-pong A/B flash page persistence across reboot and power loss,
 * matching the canonical WildSonic MAX78000 flash architecture:
 *  - 0x13245768 magic number
 *  - Monotonic sequence numbers for active slot resolution
 *  - Additive 32-bit byte checksum validation
 *  - Fixed storage partition A/B slot allocation
 *  - SD card on-boot /config.ini import and /config_status.txt export
 */

#ifndef CONFIG_STORE_H_
#define CONFIG_STORE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_STORE_MAGIC       0x13245768U
#define CONFIG_STORE_VERSION     1U
#define CONFIG_STORE_SLOT_SIZE   2048U
#define CONFIG_STORE_RECORD_SIZE 512U

#define MAX_SERIAL_LEN   32
#define MAX_DEVNAME_LEN  32
#define MAX_PASSWORD_LEN 32

/* Persisted Configuration Structure (Exactly 512 bytes packed) */
struct __attribute__((packed)) wildsonic_persisted_config {
	uint32_t magic;                    /* 0x13245768 */
	uint32_t version;                  /* 1 */
	uint32_t sequence_number;          /* Monotonically increasing sequence */

	char serial_number[MAX_SERIAL_LEN];
	char device_name[MAX_DEVNAME_LEN];
	char device_password[MAX_PASSWORD_LEN];

	uint32_t sampling_rate;            /* 192000, 96000, 48000 */
	uint32_t recording_length;         /* Session duration in seconds */
	uint32_t sleep_pause_interval;     /* Sleep interval in seconds (0 = continuous) */
	int32_t mic_gain_times_ten;        /* 0 = 0.0 dB */
	uint32_t audio_sampling_width;     /* 16 */
	uint32_t num_audio_channels;       /* 1 */
	uint32_t lower_cutoff_frequency;   /* HPF cutoff Hz (e.g. 80) */
	uint32_t upper_cutoff_frequency;   /* LPF cutoff Hz (e.g. 20000) */
	uint8_t folder_structure;          /* 0 = flat, 1 = daily, 2 = hourly */
	uint8_t psram_staging_enabled;     /* 1 = enabled, 0 = disabled */

	/* Scheduler bounds and calendar rules */
	uint16_t start_year;               /* 0 = unconstrained */
	uint8_t start_month;
	uint8_t start_day;
	uint16_t end_year;                 /* 0 = unconstrained */
	uint8_t end_month;
	uint8_t end_day;
	uint8_t day_of_week_mask;          /* 0x7F = all days */
	int32_t time_zone_offset;          /* Signed seconds offset from UTC */
	uint8_t num_tod_ranges;            /* 0 .. 10 */
	uint32_t tod_start_seconds[10];
	uint32_t tod_end_seconds[10];

	/* Pending configuration staging & generation counters */
	uint8_t pending_apply;             /* 1 = pending configuration staged, awaiting reboot apply; 0 = active */
	uint32_t pending_generation;       /* Monotonic generation counter for pending config */
	uint32_t applied_generation;       /* Monotonic generation counter for applied config */
	uint8_t reserved[263];             /* Pad out to exactly 508 bytes before checksum (1 + 4 + 4 + 263 = 272) */
	uint32_t checksum;                 /* Additive byte sum of bytes 0..507 */
};

/**
 * @brief Calculate additive byte checksum matching Maxim reference.
 */
uint32_t config_store_calc_checksum(const struct wildsonic_persisted_config *cfg);

/**
 * @brief Initialize flash persistence, inspect A/B slots, and load configuration.
 *
 * @return 0 on success, negative error code on failure.
 */
int config_store_init(void);

/**
 * @brief Commit active configuration from RAM to flash using ping-pong slot alternation.
 *
 * @return 0 on success, negative error code on failure.
 */
int config_store_save(void);

/**
 * @brief Save pending configuration to persistent flash using ping-pong slot alternation.
 *
 * Marks pending_apply = 1, increments pending_generation, commits to flash and verifies.
 *
 * @return 0 on success, negative error code on failure.
 */
int config_store_save_pending(void);

/**
 * @brief Check whether valid pending configuration exists awaiting reboot.
 *
 * @return true if pending configuration is staged in flash, false otherwise.
 */
bool config_store_has_pending(void);

/**
 * @brief Get pending configuration generation counter.
 */
uint32_t config_store_get_pending_generation(void);

/**
 * @brief Get applied configuration generation counter.
 */
uint32_t config_store_get_applied_generation(void);

/**
 * @brief Apply pending configuration to active runtime system and clear pending marker in flash.
 *
 * @return 0 on success, negative error code on failure.
 */
int config_store_apply_pending(void);

/**
 * @brief Roll back staged pending configuration in RAM to last known valid applied configuration.
 */
void config_store_rollback_pending(void);

/**
 * @brief Reset staged RAM configuration and persisted flash record to factory defaults.
 */
void config_store_load_defaults(void);

/**
 * @brief Get currently active slot index (0 or 1, or -1 if uninitialized).
 */
int config_store_get_active_slot(void);

/**
 * @brief Get sequence number of the active configuration.
 */
uint32_t config_store_get_sequence_number(void);

/**
 * @brief Check SD card for /SD:/config.ini, parse if present, update config, and commit.
 * If config.ini is absent, writes current configuration to /SD:/config_status.txt.
 *
 * @return 0 on success, negative error code on file error.
 */
int config_store_import_sd(void);

/**
 * @brief Export current active configuration to a status text file on SD card.
 *
 * @param path Destination file path (e.g. "/SD:/config_status.txt").
 * @return 0 on success, negative error code on error.
 */
int config_store_write_status_file(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STORE_H_ */

