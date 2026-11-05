/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

/* Independent validity bits prevent a partial measurement from being
 * mistaken for a complete fuel-gauge snapshot. */
#define BATTERY_VALID_VOLTAGE             (1U << 0)
#define BATTERY_VALID_CURRENT             (1U << 1)
#define BATTERY_VALID_REMAINING_CAPACITY  (1U << 2)
#define BATTERY_VALID_FULL_CAPACITY       (1U << 3)
#define BATTERY_VALID_AVERAGE_CURRENT     (1U << 4)
#define BATTERY_VALID_TIME_TO_EMPTY       (1U << 5)
#define BATTERY_VALID_AVERAGE_POWER       (1U << 6)
#define BATTERY_VALID_TEMPERATURE         (1U << 7)

enum battery_measurement_source {
	BATTERY_SOURCE_UNAVAILABLE = 0,
	BATTERY_SOURCE_BQ34110,
	BATTERY_SOURCE_ADC_VOLTAGE,
};

struct battery_status {
	uint32_t voltage_mv;
	int32_t current_ma;
	uint32_t remaining_capacity_mah;
	uint32_t full_charge_capacity_mah;
	int32_t average_current_ma;
	uint32_t time_to_empty_seconds;
	int32_t average_power_mw;
	int32_t temperature_celsius;
	uint32_t valid_fields;
	bool valid_measurement;
	int64_t last_update_timestamp;
	enum battery_measurement_source measurement_source;
};

/* The default snapshot has no valid measurements. A hardware backend may
 * publish updates from a thread or work queue; callers receive a coherent copy. */
void battery_monitor_get_status(struct battery_status *status);
void battery_monitor_publish_status(const struct battery_status *status);
void battery_monitor_clear_status(void);
int battery_monitor_init(void);
void battery_monitor_suspend(void);
void battery_monitor_resume(void);

#endif /* BATTERY_MONITOR_H_ */
