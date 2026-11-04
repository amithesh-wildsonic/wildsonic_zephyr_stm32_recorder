/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file config_store.c
 * @brief Flash Configuration Persistence and SD Import/Export for WildSonic STM32.
 */

#include "config_store.h"
#include "console.h"
#include "main.h"
#include "scheduler.h"
#include "recorder.h"
#include "audio_in.h"
#include "dsp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <math.h>

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(ws_config_store, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(ws_config_store, LOG_LEVEL_WRN);
#endif
#if defined(PARTITION_ID)
#define STORAGE_PARTITION_ID PARTITION_ID(storage_partition)
#else
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(storage_partition)
#endif
#define STORAGE_PARTITION_NODE STORAGE_PARTITION_ID
#else
#include <errno.h>
#define LOG_INF(...) do {} while (0)
#define LOG_WRN(...) do {} while (0)
#define LOG_ERR(...) do {} while (0)
#endif

static int s_active_slot = -1; /* 0 or 1, or -1 if uninitialized */
static uint32_t s_seq = 0;
static uint32_t s_pending_gen = 0;
static uint32_t s_applied_gen = 0;
static bool s_has_pending = false;
static struct wildsonic_persisted_config s_last_valid_applied_record;
static bool s_has_last_valid_applied = false;

static void unpack_record_to_system(const struct wildsonic_persisted_config *rec);
static void pack_system_to_record(struct wildsonic_persisted_config *rec);

static uint32_t crc32_for_byte(uint32_t byte_val)
{
	uint32_t crc = byte_val;
	for (int j = 0; j < 8; j++) {
		crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320U : 0);
	}
	return crc;
}

uint32_t config_store_calc_crc32(const struct wildsonic_persisted_config *cfg)
{
	if (!cfg) return 0;
	const uint8_t *p = (const uint8_t *)cfg;
	size_t check_len = offsetof(struct wildsonic_persisted_config, crc32);
	uint32_t crc = 0xFFFFFFFFU;

	for (size_t i = 0; i < check_len; i++) {
		crc = (crc >> 8) ^ crc32_for_byte((crc ^ p[i]) & 0xFFU);
	}
	return crc ^ 0xFFFFFFFFU;
}

uint32_t config_store_calc_checksum(const struct wildsonic_persisted_config *cfg)
{
	if (!cfg) return 0;
	uint32_t sum = 0;
	const uint8_t *p = (const uint8_t *)cfg;
	size_t check_len = offsetof(struct wildsonic_persisted_config, checksum);

	for (size_t i = 0; i < check_len; i++) {
		sum += p[i];
	}
	return sum;
}

int config_store_get_active_slot(void)
{
	return s_active_slot;
}

uint32_t config_store_get_sequence_number(void)
{
	return s_seq;
}

bool config_store_has_pending(void)
{
	return s_has_pending;
}

uint32_t config_store_get_pending_generation(void)
{
	return s_pending_gen;
}

uint32_t config_store_get_applied_generation(void)
{
	return s_applied_gen;
}

void config_store_rollback_pending(void)
{
	if (s_has_last_valid_applied) {
		unpack_record_to_system(&s_last_valid_applied_record);
		s_has_pending = false;
	}
}

void config_store_load_defaults(void)
{
	struct wildsonic_config *cfg = console_get_config();
	if (cfg) {
		strncpy(cfg->serial_number, "0", sizeof(cfg->serial_number) - 1);
		cfg->serial_number[sizeof(cfg->serial_number) - 1] = '\0';
		cfg->device_name[0] = '\0';
		strncpy(cfg->device_password, "wildsonic123", sizeof(cfg->device_password) - 1);
		cfg->device_password[sizeof(cfg->device_password) - 1] = '\0';

		cfg->sampling_rate = 48000;
		cfg->recording_length = 600;
		cfg->sleep_pause_interval = 0;
		cfg->mic_gain_times_ten = 0;
		cfg->audio_sampling_width = 16;
		cfg->num_audio_channels = 1;
		cfg->lower_cutoff_frequency = 0;
		cfg->upper_cutoff_frequency = 24000;
		cfg->folder_structure = 2;
		cfg->save_config_flag = false;
		cfg->format_sd_card_flag = false;
		cfg->location_lat_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
		cfg->location_lon_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
		cfg->sun_recording_mode = SUN_MODE_DISABLED;
		cfg->sunrise_sunset_rounding = 0;
		cfg->sunrise_interval_before = 0;
		cfg->sunrise_interval_after = 0;
		cfg->sunset_interval_before = 0;
		cfg->sunset_interval_after = 0;
	}

	struct app_settings *settings = main_get_settings();
	if (settings) {
		settings->sample_rate = 48000;
		settings->record_seconds = 600;
		settings->sleep_seconds = 0;
		settings->cycles = 0;
		settings->digital_gain = 1.0f;
		settings->dsp_enabled = true;
		settings->dc_block_enabled = true;
		settings->hpf_enabled = false;
		settings->hpf_cutoff_hz = 0;
		settings->notch_enabled = false;
		settings->notch_freq_hz = 50;
		settings->notch_q = 10.0f;
		settings->harmonic_notch_enabled = false;
		settings->lpf_enabled = true;
		settings->lpf_cutoff_hz = 24000;
		settings->psram_staging_enabled = true;
	}

	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	scfg.start_date.year = 0;
	scfg.start_date.month = 0;
	scfg.start_date.day = 0;
	scfg.end_date.year = 0;
	scfg.end_date.month = 0;
	scfg.end_date.day = 0;
	scheduler_reset_tod_ranges(scfg.tod_ranges, SCHEDULER_MAX_TIME_OF_DAY_RANGES);
	scfg.day_of_week_mask = SCHEDULER_FULL_WEEK_MASK;
	scfg.time_zone_offset = 0;
	scfg.record_length = 600;
	scfg.sleep_interval = 0;
	scfg.latitude_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
	scfg.longitude_e7 = SCHEDULER_INVALID_LOCATION_COORDINATE;
	scfg.sun_recording_mode = SUN_MODE_DISABLED;
	scfg.sunrise_sunset_rounding = 0;
	scfg.sunrise_interval_before = 0;
	scfg.sunrise_interval_after = 0;
	scfg.sunset_interval_before = 0;
	scfg.sunset_interval_after = 0;
	scheduler_set_config(&scfg);

	console_sync_staged_to_active();
}

static void unpack_record_to_system(const struct wildsonic_persisted_config *rec)
{
	uint32_t act = 0, div = 0;
	int err = 0;
	uint32_t eff_rate = rec->sampling_rate;
	if (!audio_in_is_rate_supported(eff_rate, &act, &div, &err)) {
		LOG_WRN("Persisted sampling rate %u unsupported by hardware, reverting to %u",
		        rec->sampling_rate, (uint32_t)CONFIG_AUDIO_SAMPLE_RATE);
		eff_rate = CONFIG_AUDIO_SAMPLE_RATE;
	}

	uint32_t eff_bits = (rec->audio_sampling_width == 32) ? 32 : 16;
	uint32_t eff_ch = (rec->num_audio_channels == 2) ? 2 : 1;

	struct wildsonic_config *cfg = console_get_config();
	if (cfg) {
		/* Migrate or sanitize legacy serial number and device name */
		if (strcmp(rec->serial_number, "WS4760001") == 0 || strlen(rec->serial_number) > 3) {
			strncpy(cfg->serial_number, "0", sizeof(cfg->serial_number) - 1);
		} else {
			strncpy(cfg->serial_number, rec->serial_number, sizeof(cfg->serial_number) - 1);
		}
		cfg->serial_number[sizeof(cfg->serial_number) - 1] = '\0';

		if (strcmp(rec->device_name, "WildSonic") == 0 || strlen(rec->device_name) > 6) {
			cfg->device_name[0] = '\0';
		} else {
			strncpy(cfg->device_name, rec->device_name, sizeof(cfg->device_name) - 1);
		}
		cfg->device_name[sizeof(cfg->device_name) - 1] = '\0';

		strncpy(cfg->device_password, rec->device_password, sizeof(cfg->device_password) - 1);
		cfg->device_password[sizeof(cfg->device_password) - 1] = '\0';

		cfg->sampling_rate = eff_rate;
		cfg->recording_length = rec->recording_length;
		cfg->sleep_pause_interval = rec->sleep_pause_interval;
		cfg->mic_gain_times_ten = rec->mic_gain_times_ten;
		cfg->audio_sampling_width = eff_bits;
		cfg->num_audio_channels = eff_ch;
		cfg->lower_cutoff_frequency = rec->lower_cutoff_frequency;
		uint32_t validated_lpf = dsp_validate_lpf_cutoff(eff_rate, rec->upper_cutoff_frequency);
		cfg->upper_cutoff_frequency = validated_lpf;
		cfg->folder_structure = rec->folder_structure;

		cfg->location_lat_e7 = rec->latitude_e7 ? rec->latitude_e7 : SCHEDULER_INVALID_LOCATION_COORDINATE;
		cfg->location_lon_e7 = rec->longitude_e7 ? rec->longitude_e7 : SCHEDULER_INVALID_LOCATION_COORDINATE;
		cfg->sun_recording_mode = rec->sun_recording_mode;
		cfg->sunrise_sunset_rounding = rec->sunrise_sunset_rounding;
		cfg->sunrise_interval_before = rec->sunrise_interval_before;
		cfg->sunrise_interval_after = rec->sunrise_interval_after;
		cfg->sunset_interval_before = rec->sunset_interval_before;
		cfg->sunset_interval_after = rec->sunset_interval_after;
	}

	struct app_settings *settings = main_get_settings();
	if (settings) {
		uint32_t validated_lpf = dsp_validate_lpf_cutoff(eff_rate, rec->upper_cutoff_frequency);
		settings->sample_rate = eff_rate;
		settings->sample_bits = (uint16_t)eff_bits;
		settings->channels = (uint16_t)eff_ch;
		settings->record_seconds = rec->recording_length;
		settings->sleep_seconds = rec->sleep_pause_interval;
		settings->digital_gain = powf(10.0f, ((float)rec->mic_gain_times_ten / 10.0f) / 20.0f);
		settings->hpf_cutoff_hz = rec->lower_cutoff_frequency;
		settings->hpf_enabled = (rec->lower_cutoff_frequency > 0);
		settings->lpf_cutoff_hz = validated_lpf;
		settings->lpf_enabled = (validated_lpf > 0);
		settings->psram_staging_enabled = (rec->psram_staging_enabled != 0);
	}

	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	scfg.start_date.year = rec->start_year;
	scfg.start_date.month = rec->start_month;
	scfg.start_date.day = rec->start_day;
	scfg.end_date.year = rec->end_year;
	scfg.end_date.month = rec->end_month;
	scfg.end_date.day = rec->end_day;
	scfg.day_of_week_mask = rec->day_of_week_mask ? rec->day_of_week_mask : SCHEDULER_FULL_WEEK_MASK;
	scfg.time_zone_offset = rec->time_zone_offset;
	scfg.record_length = rec->recording_length;
	scfg.sleep_interval = rec->sleep_pause_interval;

	scfg.latitude_e7 = rec->latitude_e7 ? rec->latitude_e7 : SCHEDULER_INVALID_LOCATION_COORDINATE;
	scfg.longitude_e7 = rec->longitude_e7 ? rec->longitude_e7 : SCHEDULER_INVALID_LOCATION_COORDINATE;
	scfg.sun_recording_mode = rec->sun_recording_mode;
	scfg.sunrise_sunset_rounding = rec->sunrise_sunset_rounding;
	scfg.sunrise_interval_before = rec->sunrise_interval_before;
	scfg.sunrise_interval_after = rec->sunrise_interval_after;
	scfg.sunset_interval_before = rec->sunset_interval_before;
	scfg.sunset_interval_after = rec->sunset_interval_after;

	scheduler_reset_tod_ranges(scfg.tod_ranges, SCHEDULER_MAX_TIME_OF_DAY_RANGES);
	for (int i = 0; i < SCHEDULER_MAX_TIME_OF_DAY_RANGES && i < rec->num_tod_ranges; i++) {
		scfg.tod_ranges[i].start_second = rec->tod_start_seconds[i];
		scfg.tod_ranges[i].end_second = rec->tod_end_seconds[i];
	}
	scheduler_set_config(&scfg);

	console_sync_staged_to_active();
}

static void pack_system_to_record(struct wildsonic_persisted_config *rec)
{
	memset(rec, 0, sizeof(*rec));
	rec->magic = CONFIG_STORE_MAGIC;
	rec->version = CONFIG_STORE_VERSION;
	rec->sequence_number = s_seq;

	struct wildsonic_config *cfg = console_get_config();
	if (cfg) {
		snprintf(rec->serial_number, sizeof(rec->serial_number), "%s", cfg->serial_number);
		snprintf(rec->device_name, sizeof(rec->device_name), "%s", cfg->device_name);
		snprintf(rec->device_password, sizeof(rec->device_password), "%s", cfg->device_password);
		rec->sampling_rate = cfg->sampling_rate;
		rec->recording_length = cfg->recording_length;
		rec->sleep_pause_interval = cfg->sleep_pause_interval;
		rec->mic_gain_times_ten = cfg->mic_gain_times_ten;
		rec->audio_sampling_width = cfg->audio_sampling_width;
		rec->num_audio_channels = cfg->num_audio_channels;
		rec->lower_cutoff_frequency = cfg->lower_cutoff_frequency;
		rec->upper_cutoff_frequency = cfg->upper_cutoff_frequency;
		rec->folder_structure = cfg->folder_structure;
	}

	struct app_settings *settings = main_get_settings();
	if (settings) {
		rec->psram_staging_enabled = settings->psram_staging_enabled ? 1 : 0;
	}

	struct schedule_config scfg;
	scheduler_get_config(&scfg);
	rec->start_year = scfg.start_date.year;
	rec->start_month = scfg.start_date.month;
	rec->start_day = scfg.start_date.day;
	rec->end_year = scfg.end_date.year;
	rec->end_month = scfg.end_date.month;
	rec->end_day = scfg.end_date.day;
	rec->day_of_week_mask = scfg.day_of_week_mask;
	rec->time_zone_offset = scfg.time_zone_offset;
	rec->latitude_e7 = scfg.latitude_e7;
	rec->longitude_e7 = scfg.longitude_e7;
	rec->sun_recording_mode = scfg.sun_recording_mode;
	rec->sunrise_sunset_rounding = scfg.sunrise_sunset_rounding;
	rec->sunrise_interval_before = scfg.sunrise_interval_before;
	rec->sunrise_interval_after = scfg.sunrise_interval_after;
	rec->sunset_interval_before = scfg.sunset_interval_before;
	rec->sunset_interval_after = scfg.sunset_interval_after;

	uint8_t count = 0;
	for (int i = 0; i < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
		if (scfg.tod_ranges[i].start_second != 0 || scfg.tod_ranges[i].end_second != 0) {
			rec->tod_start_seconds[count] = scfg.tod_ranges[i].start_second;
			rec->tod_end_seconds[count] = scfg.tod_ranges[i].end_second;
			count++;
		}
	}
	rec->num_tod_ranges = count;

	rec->pending_apply = s_has_pending ? 1 : 0;
	rec->pending_generation = s_pending_gen;
	rec->applied_generation = s_applied_gen;

	rec->version = CONFIG_STORE_VERSION_2;
	rec->crc32 = config_store_calc_crc32(rec);
	rec->checksum = config_store_calc_checksum(rec);
}

static bool config_record_is_valid(const struct wildsonic_persisted_config *rec)
{
	if (!rec) return false;
	if (rec->magic != CONFIG_STORE_MAGIC) return false;
	if (config_store_calc_checksum(rec) != rec->checksum) return false;
	if (rec->version >= CONFIG_STORE_VERSION_2) {
		if (config_store_calc_crc32(rec) != rec->crc32) {
			return false;
		}
	}

	uint32_t act = 0, div = 0;
	int err = 0;
	if (!audio_in_is_rate_supported(rec->sampling_rate, &act, &div, &err)) {
		return false;
	}
	if (rec->audio_sampling_width != 16 && rec->audio_sampling_width != 32) {
		return false;
	}
	if (rec->num_audio_channels != 1 && rec->num_audio_channels != 2) {
		return false;
	}
	if (rec->recording_length == 0 || rec->recording_length > 86400) {
		return false;
	}
	return true;
}

int config_store_init(void)
{
#ifdef __ZEPHYR__
	const struct flash_area *fa = NULL;
	int rc = flash_area_open(STORAGE_PARTITION_ID, &fa);
	if (rc != 0) {
		LOG_ERR("Failed to open storage partition: %d", rc);
		config_store_load_defaults();
		return rc;
	}

	struct wildsonic_persisted_config slot0, slot1;
	bool valid0 = false, valid1 = false;

	if (flash_area_read(fa, 0, &slot0, sizeof(slot0)) == 0) {
		if (slot0.magic == CONFIG_STORE_MAGIC &&
		    config_store_calc_checksum(&slot0) == slot0.checksum) {
			valid0 = true;
		}
	}

	if (flash_area_read(fa, CONFIG_STORE_SLOT_SIZE, &slot1, sizeof(slot1)) == 0) {
		if (slot1.magic == CONFIG_STORE_MAGIC &&
		    config_store_calc_checksum(&slot1) == slot1.checksum) {
			valid1 = true;
		}
	}

	flash_area_close(fa);

	struct wildsonic_persisted_config *primary = NULL;
	struct wildsonic_persisted_config *fallback = NULL;
	int primary_slot = -1;

	if (valid0 && valid1) {
		int32_t diff = (int32_t)(slot0.sequence_number - slot1.sequence_number);
		if (diff >= 0) {
			primary = &slot0;
			fallback = &slot1;
			primary_slot = 0;
		} else {
			primary = &slot1;
			fallback = &slot0;
			primary_slot = 1;
		}
	} else if (valid0) {
		primary = &slot0;
		fallback = NULL;
		primary_slot = 0;
	} else if (valid1) {
		primary = &slot1;
		fallback = NULL;
		primary_slot = 1;
	}

	if (primary) {
		s_active_slot = primary_slot;
		s_seq = primary->sequence_number;

		if (primary->pending_apply) {
			LOG_INF("Pending configuration detected in flash slot %d (seq: %u, gen: %u)",
			        primary_slot, primary->sequence_number, primary->pending_generation);

			if (config_record_is_valid(primary)) {
				if (primary->version == CONFIG_STORE_VERSION_1) {
					LOG_INF("Migrating config store record from Version 1 to Version 2");
				}
				unpack_record_to_system(primary);
				s_applied_gen = primary->pending_generation;
				s_pending_gen = primary->pending_generation;
				s_has_pending = false;
				s_last_valid_applied_record = *primary;
				s_last_valid_applied_record.pending_apply = 0;
				s_last_valid_applied_record.applied_generation = s_applied_gen;
				s_has_last_valid_applied = true;

				/* Clear pending marker in flash atomically by committing applied state */
				(void)config_store_save();
				LOG_INF("Applied pending configuration to active runtime (applied gen: %u)", s_applied_gen);
				return 0;
			} else {
				LOG_ERR("Pending configuration validation failed! Falling back to last known valid applied configuration.");
				if (fallback && config_record_is_valid(fallback) && !fallback->pending_apply) {
					unpack_record_to_system(fallback);
					s_applied_gen = fallback->applied_generation;
					s_pending_gen = fallback->applied_generation;
					s_has_pending = false;
					s_last_valid_applied_record = *fallback;
					s_has_last_valid_applied = true;
					LOG_INF("Recovered using fallback flash slot (seq: %u, applied gen: %u)",
					        fallback->sequence_number, s_applied_gen);
					return 0;
				} else {
					LOG_WRN("No valid fallback configuration available; loading factory defaults");
					config_store_load_defaults();
					s_applied_gen = 0;
					s_pending_gen = 0;
					s_has_pending = false;
					return 0;
				}
			}
		} else {
			/* Standard applied configuration */
			unpack_record_to_system(primary);
			s_applied_gen = primary->applied_generation;
			s_pending_gen = primary->pending_generation;
			s_has_pending = false;
			s_last_valid_applied_record = *primary;
			s_has_last_valid_applied = true;
			LOG_INF("Loaded config from flash slot %d (seq: %u, applied gen: %u)",
			        s_active_slot, s_seq, s_applied_gen);
			return 0;
		}
	} else {
		/* Neither valid: load factory defaults */
		s_active_slot = -1;
		s_seq = 0;
		s_applied_gen = 0;
		s_pending_gen = 0;
		s_has_pending = false;
		config_store_load_defaults();
		LOG_INF("No valid flash configuration found. Loaded factory defaults.");
		return 0;
	}
#else
	config_store_load_defaults();
	return 0;
#endif
}

int config_store_save(void)
{
	s_has_pending = false;
	s_applied_gen = s_pending_gen;
	s_seq++;
	struct wildsonic_persisted_config rec;
	pack_system_to_record(&rec);
	rec.pending_apply = 0;
	rec.applied_generation = s_applied_gen;
	rec.pending_generation = s_pending_gen;
	rec.sequence_number = s_seq;
	rec.checksum = config_store_calc_checksum(&rec);

#ifdef __ZEPHYR__
	const struct flash_area *fa = NULL;
	int rc = flash_area_open(STORAGE_PARTITION_ID, &fa);
	if (rc != 0) {
		LOG_ERR("Failed to open storage partition for write: %d", rc);
		return rc;
	}

	int target_slot = (s_active_slot == 0) ? 1 : 0;
	off_t target_offset = target_slot * CONFIG_STORE_SLOT_SIZE;

	rc = flash_area_erase(fa, target_offset, CONFIG_STORE_SLOT_SIZE);
	if (rc != 0) {
		LOG_ERR("flash_area_erase failed on slot %d: %d", target_slot, rc);
		flash_area_close(fa);
		return rc;
	}

	rc = flash_area_write(fa, target_offset, &rec, sizeof(rec));
	if (rc != 0) {
		LOG_ERR("flash_area_write failed on slot %d: %d", target_slot, rc);
		flash_area_close(fa);
		return rc;
	}

	/* Verify write by reading back and checking checksum */
	struct wildsonic_persisted_config verify;
	rc = flash_area_read(fa, target_offset, &verify, sizeof(verify));
	if (rc != 0 || verify.magic != CONFIG_STORE_MAGIC ||
	    verify.checksum != rec.checksum || memcmp(&verify, &rec, sizeof(rec)) != 0) {
		LOG_ERR("Verification failed for slot %d", target_slot);
		flash_area_close(fa);
		return -EIO;
	}

	flash_area_close(fa);
	s_active_slot = target_slot;
	s_last_valid_applied_record = rec;
	s_has_last_valid_applied = true;
	LOG_INF("Saved config to flash slot %d (seq: %u, applied gen: %u)", s_active_slot, s_seq, s_applied_gen);
	return 0;
#else
	s_active_slot = (s_active_slot == 0) ? 1 : 0;
	s_last_valid_applied_record = rec;
	s_has_last_valid_applied = true;
	return 0;
#endif
}

int config_store_save_pending(void)
{
	s_has_pending = true;
	s_pending_gen++;
	s_seq++;
	struct wildsonic_persisted_config rec;
	pack_system_to_record(&rec);
	rec.pending_apply = 1;
	rec.pending_generation = s_pending_gen;
	rec.applied_generation = s_applied_gen;
	rec.sequence_number = s_seq;
	rec.checksum = config_store_calc_checksum(&rec);

#ifdef __ZEPHYR__
	const struct flash_area *fa = NULL;
	int rc = flash_area_open(STORAGE_PARTITION_ID, &fa);
	if (rc != 0) {
		LOG_ERR("Failed to open storage partition for write: %d", rc);
		s_pending_gen--;
		s_seq--;
		return rc;
	}

	int target_slot = (s_active_slot == 0) ? 1 : 0;
	off_t target_offset = target_slot * CONFIG_STORE_SLOT_SIZE;

	rc = flash_area_erase(fa, target_offset, CONFIG_STORE_SLOT_SIZE);
	if (rc != 0) {
		LOG_ERR("flash_area_erase failed on slot %d: %d", target_slot, rc);
		flash_area_close(fa);
		s_pending_gen--;
		s_seq--;
		return rc;
	}

	rc = flash_area_write(fa, target_offset, &rec, sizeof(rec));
	if (rc != 0) {
		LOG_ERR("flash_area_write failed on slot %d: %d", target_slot, rc);
		flash_area_close(fa);
		s_pending_gen--;
		s_seq--;
		return rc;
	}

	/* Verify write by reading back and checking checksum */
	struct wildsonic_persisted_config verify;
	rc = flash_area_read(fa, target_offset, &verify, sizeof(verify));
	if (rc != 0 || verify.magic != CONFIG_STORE_MAGIC ||
	    verify.checksum != rec.checksum || memcmp(&verify, &rec, sizeof(rec)) != 0) {
		LOG_ERR("Verification failed for slot %d", target_slot);
		flash_area_close(fa);
		s_pending_gen--;
		s_seq--;
		return -EIO;
	}

	flash_area_close(fa);
	s_active_slot = target_slot;
	LOG_INF("Saved pending config to flash slot %d (seq: %u, pending gen: %u)",
	        s_active_slot, s_seq, s_pending_gen);
	return 0;
#else
	s_active_slot = (s_active_slot == 0) ? 1 : 0;
	return 0;
#endif
}

int config_store_apply_pending(void)
{
	if (!s_has_pending) {
		return 0;
	}
	return config_store_save();
}

int config_store_write_status_file(const char *path)
{
#ifdef __ZEPHYR__
	struct fs_file_t file;
	fs_file_t_init(&file);

	int rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
	if (rc != 0) {
		LOG_ERR("config_store_write_status_file: fs_open failed (%d)", rc);
		return rc;
	}

	/* Truncate if already exists */
	rc = fs_truncate(&file, 0);
	if (rc != 0) {
		LOG_ERR("config_store_write_status_file: fs_truncate failed (%d)", rc);
		fs_close(&file);
		return rc;
	}

	char buf[256];
	struct wildsonic_config *cfg = console_get_config();
	struct app_settings *settings = main_get_settings();
	struct schedule_config scfg;
	scheduler_get_config(&scfg);

#define STATUS_WRITE_LINE(fmt, ...) do { \
		int n = snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
		if (n > 0) { \
			ssize_t w = fs_write(&file, buf, (size_t)n); \
			if (w != (ssize_t)n) { \
				int err_code = (w < 0) ? (int)w : -EIO; \
				LOG_ERR("config_store_write_status_file: write failed (%d)", err_code); \
				fs_close(&file); \
				return err_code; \
			} \
		} \
	} while (0)

	STATUS_WRITE_LINE("sampling_frequency=%u\n", cfg ? cfg->sampling_rate : 192000);
	STATUS_WRITE_LINE("bits_per_sample=%u\n", cfg ? cfg->audio_sampling_width : 16);
	STATUS_WRITE_LINE("number_of_channels=%u\n", cfg ? cfg->num_audio_channels : 1);
	STATUS_WRITE_LINE("mic_gain=%d\n", cfg ? cfg->mic_gain_times_ten : 0);
	STATUS_WRITE_LINE("lower_cutoff_frequency=%u\n", cfg ? cfg->lower_cutoff_frequency : 80);
	STATUS_WRITE_LINE("upper_cutoff_frequency=%u\n", cfg ? cfg->upper_cutoff_frequency : 20000);
	STATUS_WRITE_LINE("record_length=%u\n", cfg ? cfg->recording_length : 20);
	STATUS_WRITE_LINE("sleep_interval=%u\n", cfg ? cfg->sleep_pause_interval : 60);

	char date_buf[64];
	scheduler_format_date_range(&scfg.start_date, &scfg.end_date, date_buf, sizeof(date_buf));
	STATUS_WRITE_LINE("date_range=%s, Note: The date range is inclusive of the start and end dates. 9999/99/99 means no date is selected.\n", date_buf);

	for (int i = 0; i < SCHEDULER_MAX_TIME_OF_DAY_RANGES; i++) {
		if (scfg.tod_ranges[i].start_second != 0 || scfg.tod_ranges[i].end_second != 0) {
			char tod_buf[32];
			scheduler_format_time_of_day(&scfg.tod_ranges[i], tod_buf, sizeof(tod_buf));
			STATUS_WRITE_LINE("time_of_day_%d=%s\n", i, tod_buf);
		}
	}

	char dow_buf[16];
	scheduler_format_days_of_week(scfg.day_of_week_mask, dow_buf, sizeof(dow_buf));
	STATUS_WRITE_LINE("days_of_week=%s, Note: 1 means active and 0 means inactive. The days are in the order of Monday to Sunday.\n", dow_buf);

	char tz_buf[16];
	scheduler_format_timezone(scfg.time_zone_offset, tz_buf, sizeof(tz_buf));
	STATUS_WRITE_LINE("time_zone=%s\n", tz_buf);

	const char *fldr_str = (cfg && cfg->folder_structure == 1) ? "daily" :
			       ((cfg && cfg->folder_structure == 2) ? "hourly" : "flat");
	STATUS_WRITE_LINE("folder_structure=%s\n", fldr_str);
	STATUS_WRITE_LINE("psram_staging=%s\n", (settings && settings->psram_staging_enabled) ? "enabled" : "disabled");

	if (scfg.latitude_e7 != SCHEDULER_INVALID_LOCATION_COORDINATE &&
	    scfg.longitude_e7 != SCHEDULER_INVALID_LOCATION_COORDINATE) {
		STATUS_WRITE_LINE("location=%.7f, %.7f\n",
		                  (double)scfg.latitude_e7 / 10000000.0,
		                  (double)scfg.longitude_e7 / 10000000.0);
	} else {
		STATUS_WRITE_LINE("location=Not set\n");
	}
	STATUS_WRITE_LINE("sun_recording_mode=%u\n", (unsigned int)scfg.sun_recording_mode);
	STATUS_WRITE_LINE("sunrise_interval_before=%u\n", (unsigned int)scfg.sunrise_interval_before);
	STATUS_WRITE_LINE("sunrise_interval_after=%u\n", (unsigned int)scfg.sunrise_interval_after);
	STATUS_WRITE_LINE("sunset_interval_before=%u\n", (unsigned int)scfg.sunset_interval_before);
	STATUS_WRITE_LINE("sunset_interval_after=%u\n", (unsigned int)scfg.sunset_interval_after);
	STATUS_WRITE_LINE("sunrise_sunset_rounding=%u\n", (unsigned int)scfg.sunrise_sunset_rounding);

#undef STATUS_WRITE_LINE

	rc = fs_close(&file);
	if (rc != 0) {
		LOG_ERR("config_store_write_status_file: fs_close failed (%d)", rc);
		return rc;
	}
	printk("Configuration status file has been created\n");
	return 0;
#else
	ARG_UNUSED(path);
	return 0;
#endif
}

int config_store_import_sd(void)
{
#ifdef __ZEPHYR__
	const char *ini_path = "/SD:/config.ini";
	const char *bak_path = "/SD:/config.bak";
	const char *status_path = "/SD:/config_status.txt";

	struct fs_dirent entry;
	if (fs_stat(ini_path, &entry) != 0) {
		/* config.ini not present on SD card; create config_status.txt with current settings */
		printk("Configuration file is missing. Using earlier programmed configuration.\n");
		config_store_write_status_file(status_path);
		return 0;
	}

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, ini_path, FS_O_READ);
	if (rc != 0) {
		LOG_ERR("Failed to open %s: %d", ini_path, rc);
		config_store_write_status_file(status_path);
		return rc;
	}

	struct wildsonic_config orig_cfg = *console_get_config();
	struct schedule_config orig_scfg;
	scheduler_get_config(&orig_scfg);

	char line[128];
	size_t line_pos = 0;
	char ch;
	bool updated = false;
	bool import_error = false;
	bool discarding_oversized = false;
	ssize_t read_bytes = 0;

	while ((read_bytes = fs_read(&file, &ch, 1)) == 1 || line_pos > 0) {
		if (read_bytes == 1) {
			if (discarding_oversized) {
				if (ch == '\n') {
					discarding_oversized = false;
					line_pos = 0;
				}
				continue;
			}

			if (ch == '\r') {
				continue;
			}

			if (ch != '\n') {
				if (line_pos >= sizeof(line) - 1) {
					LOG_WRN("config_store_import_sd: line exceeds 127 bytes, discarding");
					import_error = true;
					discarding_oversized = true;
					line_pos = 0;
					continue;
				}
				line[line_pos++] = ch;
				continue;
			}
		}

		/* We have reached a newline or EOF with pending characters */
		line[line_pos] = '\0';
		line_pos = 0;

		/* Trim comment */
		char *hash = strchr(line, '#');
		if (hash) *hash = '\0';

		/* Trim leading whitespace */
		char *start = line;
		while (*start && isspace((unsigned char)*start)) start++;

		/* Trim trailing whitespace */
		size_t len = strlen(start);
		while (len > 0 && isspace((unsigned char)start[len - 1])) {
			start[--len] = '\0';
		}

		if (len == 0) {
			if (read_bytes != 1) break;
			continue;
		}

		char *eq = strchr(start, '=');
		if (!eq) {
			printk("\nUnknown or BAD config option : %s\n", start);
			import_error = true;
			if (read_bytes != 1) break;
			continue;
		}

		*eq = '\0';
		char *key = start;
		char *val = eq + 1;

		/* Trim key trailing spaces */
		size_t klen = strlen(key);
		while (klen > 0 && isspace((unsigned char)key[klen - 1])) {
			key[--klen] = '\0';
		}

		/* Trim val leading spaces */
		while (*val && isspace((unsigned char)*val)) val++;
		/* Trim val trailing spaces */
		size_t vlen = strlen(val);
		while (vlen > 0 && isspace((unsigned char)val[vlen - 1])) {
			val[--vlen] = '\0';
		}

		char cmd_buf[128];
		cmd_buf[0] = '\0';

		if (strcasecmp(key, "sampling_frequency") == 0 || strcasecmp(key, "samf") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "SAMF %s", val);
		} else if (strcasecmp(key, "record_length") == 0 || strcasecmp(key, "rect") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "RECT %s", val);
		} else if (strcasecmp(key, "sleep_interval") == 0 || strcasecmp(key, "sleep") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "SLEEP %s", val);
		} else if (strcasecmp(key, "bits_per_sample") == 0 || strcasecmp(key, "btps") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "BTPS %s", val);
		} else if (strcasecmp(key, "number_of_channels") == 0 || strcasecmp(key, "numch") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "NUMCH %s", val);
		} else if (strcasecmp(key, "mic_gain") == 0 || strcasecmp(key, "gain") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "GAIN %s", val);
		} else if (strcasecmp(key, "lower_cutoff_frequency") == 0 || strcasecmp(key, "lcutoff") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "LCUTOFF %s", val);
		} else if (strcasecmp(key, "upper_cutoff_frequency") == 0 || strcasecmp(key, "ucutoff") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "UCUTOFF %s", val);
		} else if (strcasecmp(key, "date_range") == 0 || strcasecmp(key, "daterange") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "DATERANGE %s", val);
		} else if (strncasecmp(key, "time_of_day_", 12) == 0) {
			if (isdigit((unsigned char)key[12]) && key[13] == '\0') {
				int idx = key[12] - '0';
				snprintf(cmd_buf, sizeof(cmd_buf), "TIMEOFDAY %d %s", idx, val);
			} else {
				printk("\nUnknown or BAD config option : %s\n", key);
				import_error = true;
				if (read_bytes != 1) break;
				continue;
			}
		} else if (strcasecmp(key, "time_of_day") == 0 || strcasecmp(key, "timeofday") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "TIMEOFDAY %s", val);
		} else if (strcasecmp(key, "days_of_week") == 0 || strcasecmp(key, "daysofweek") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "DAYSOFWEEK %s", val);
		} else if (strcasecmp(key, "time_zone") == 0 || strcasecmp(key, "timezone") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "TIMEZONE %s", val);
		} else if (strcasecmp(key, "system_time") == 0 || strcasecmp(key, "datetime") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "DATETIME %s", val);
		} else if (strcasecmp(key, "folder_structure") == 0 || strcasecmp(key, "fldr") == 0) {
			snprintf(cmd_buf, sizeof(cmd_buf), "FLDR %s", val);
		} else if (strcasecmp(key, "transmission_mode") == 0 || strcasecmp(key, "gsm") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			if (cfg) cfg->transmission_mode = (uint32_t)strtoul(val, NULL, 10);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "location") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			if (cfg) {
				int lat = 0, lon = 0;
				if (sscanf(val, "%d %d", &lat, &lon) == 2) {
					cfg->location_lat_e7 = lat;
					cfg->location_lon_e7 = lon;
					updated = true;
					if (read_bytes != 1) break;
					continue;
				}
			}
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "sms_frequency") == 0 || strcasecmp(key, "smsfreq") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			if (cfg) cfg->sms_frequency = (uint32_t)strtoul(val, NULL, 10);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "low_battery_threshold") == 0 || strcasecmp(key, "lbat") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			if (cfg) cfg->low_battery_threshold = (uint16_t)strtoul(val, NULL, 10);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "latitude") == 0 || strcasecmp(key, "lat") == 0) {
			double lat_val = strtod(val, NULL);
			if (lat_val >= -90.0 && lat_val <= 90.0) {
				struct wildsonic_config *cfg = console_get_config();
				int32_t e7 = (int32_t)floor(lat_val * 10000000.0 + 0.5);
				if (cfg) cfg->location_lat_e7 = e7;
				struct schedule_config scfg;
				scheduler_get_config(&scfg);
				scfg.latitude_e7 = e7;
				scheduler_set_config(&scfg);
				updated = true;
			}
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "longitude") == 0 || strcasecmp(key, "lon") == 0) {
			double lon_val = strtod(val, NULL);
			if (lon_val >= -180.0 && lon_val <= 180.0) {
				struct wildsonic_config *cfg = console_get_config();
				int32_t e7 = (int32_t)floor(lon_val * 10000000.0 + 0.5);
				if (cfg) cfg->location_lon_e7 = e7;
				struct schedule_config scfg;
				scheduler_get_config(&scfg);
				scfg.longitude_e7 = e7;
				scheduler_set_config(&scfg);
				updated = true;
			}
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "sun_recording_mode") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			uint8_t m = (uint8_t)strtoul(val, NULL, 10);
			if (cfg) cfg->sun_recording_mode = m;
			struct schedule_config scfg;
			scheduler_get_config(&scfg);
			scfg.sun_recording_mode = m;
			scheduler_set_config(&scfg);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "sunrise_interval_before") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			uint8_t v = (uint8_t)strtoul(val, NULL, 10);
			if (cfg) cfg->sunrise_interval_before = v;
			struct schedule_config scfg;
			scheduler_get_config(&scfg);
			scfg.sunrise_interval_before = v;
			scheduler_set_config(&scfg);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "sunrise_interval_after") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			uint8_t v = (uint8_t)strtoul(val, NULL, 10);
			if (cfg) cfg->sunrise_interval_after = v;
			struct schedule_config scfg;
			scheduler_get_config(&scfg);
			scfg.sunrise_interval_after = v;
			scheduler_set_config(&scfg);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "sunset_interval_before") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			uint8_t v = (uint8_t)strtoul(val, NULL, 10);
			if (cfg) cfg->sunset_interval_before = v;
			struct schedule_config scfg;
			scheduler_get_config(&scfg);
			scfg.sunset_interval_before = v;
			scheduler_set_config(&scfg);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "sunset_interval_after") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			uint8_t v = (uint8_t)strtoul(val, NULL, 10);
			if (cfg) cfg->sunset_interval_after = v;
			struct schedule_config scfg;
			scheduler_get_config(&scfg);
			scfg.sunset_interval_after = v;
			scheduler_set_config(&scfg);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "sunrise_sunset_rounding") == 0) {
			struct wildsonic_config *cfg = console_get_config();
			uint8_t v = (uint8_t)strtoul(val, NULL, 10);
			if (cfg) cfg->sunrise_sunset_rounding = v;
			struct schedule_config scfg;
			scheduler_get_config(&scfg);
			scfg.sunrise_sunset_rounding = v;
			scheduler_set_config(&scfg);
			updated = true;
			if (read_bytes != 1) break;
			continue;
		} else if (strcasecmp(key, "build_version") == 0 ||
		           strcasecmp(key, "build_datetime") == 0 ||
		           strcasecmp(key, "build_sha") == 0) {
			LOG_INF("config_store_import_sd: ignoring status metadata '%s'", key);
			if (read_bytes != 1) break;
			continue;
		} else {
			printk("\nUnknown or BAD config option : %s\n", key);
			import_error = true;
			if (read_bytes != 1) break;
			continue;
		}

		if (cmd_buf[0] != '\0') {
			int rc = console_process_line_source(cmd_buf, false, NULL, 0);
			if (rc == 0) {
				updated = true;
			} else {
				LOG_WRN("config_store_import_sd: command '%s' failed (%d)", cmd_buf, rc);
				import_error = true;
			}
		}

		if (read_bytes != 1) break;
	}

	if (read_bytes < 0) {
		LOG_ERR("config_store_import_sd: fs_read failed (%d)", (int)read_bytes);
		import_error = true;
	}

	fs_close(&file);

	if (import_error) {
		LOG_ERR("config_store_import_sd: import failed, rolling back to previous configuration");
		*console_get_config() = orig_cfg;
		scheduler_set_config(&orig_scfg);
		config_store_rollback_pending();
		updated = false;
	}
	if (updated) {
		config_store_save();
	}
	if (updated) {
		console_sync_staged_to_active();
	}

	/* Rename config.ini to config.bak (or unlink if rename fails) */
	fs_unlink(bak_path);
	if (fs_rename(ini_path, bak_path) == 0) {
		printk("Configuration file has been archived to config.bak\n");
	} else {
		fs_unlink(ini_path);
		printk("Configuration file has been deleted\n");
	}

	config_store_write_status_file(status_path);
	return import_error ? -EINVAL : 0;
#else
	return 0;
#endif
}
