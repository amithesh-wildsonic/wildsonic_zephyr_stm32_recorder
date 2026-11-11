/* SPDX-License-Identifier: Apache-2.0 */
#include "battery_monitor.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)
LOG_MODULE_REGISTER(battery_monitor, LOG_LEVEL_INF);
#else
LOG_MODULE_REGISTER(battery_monitor, LOG_LEVEL_WRN);
#endif

#define BQ34110_REG_VOLTAGE             0x08U
#define BQ34110_REG_CURRENT             0x0CU
#define BQ34110_REG_REMAINING_CAPACITY  0x10U
#define BQ34110_REG_FULL_CAPACITY       0x12U
#define BQ34110_REG_AVERAGE_CURRENT     0x14U
#define BQ34110_REG_TIME_TO_EMPTY       0x16U
#define BQ34110_REG_AVERAGE_POWER       0x24U
#define BQ34110_REG_INTERNAL_TEMP       0x28U
#define BATTERY_UPDATE_PERIOD           K_SECONDS(10)

static const struct i2c_dt_spec s_gauge = I2C_DT_SPEC_GET(DT_NODELABEL(bq34110));

K_MUTEX_DEFINE(s_battery_status_mutex);
K_MUTEX_DEFINE(s_monitor_state_mutex);
static struct battery_status s_battery_status;
static bool s_monitor_suspended;
static bool s_gauge_available;
static struct k_work_delayable s_update_work;

static int bq34110_read_word(uint8_t command, uint16_t *value)
{
	uint8_t rx[2];
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}

	/* SMBus Read Word: command, repeated START, low data byte, high byte.
	 * The STM32 I2C driver bounds bus waits and returns an error on timeout.
	 */
	ret = i2c_write_read_dt(&s_gauge, &command, sizeof(command), rx, sizeof(rx));
	if (ret != 0) {
		return ret;
	}

	*value = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
	return 0;
}

static void battery_monitor_read_cycle(void)
{
	struct battery_status previous;
	struct battery_status current;
	uint16_t raw;
	int rc;

	if (!s_gauge_available) {
		return;
	}

	if (!i2c_is_ready_dt(&s_gauge)) {
		LOG_WRN("I2C1 or BQ34110 node is not ready");
		s_gauge_available = false;
		return;
	}

	battery_monitor_get_status(&previous);
	current = previous;
	/* Values remain as last-known data, while validity tracks this poll's
	 * successful register reads so stale readings are never reported as fresh.
	 */
	current.valid_fields = 0U;
	current.valid_measurement = false;

	rc = bq34110_read_word(BQ34110_REG_VOLTAGE, &raw);
	if (rc == 0) {
		current.voltage_mv = raw; /* mV */
		current.valid_fields |= BATTERY_VALID_VOLTAGE;
	} else {
		LOG_WRN("BQ34110 voltage read failed: %d", rc);
		s_gauge_available = false;
		goto publish_snapshot;
	}

	rc = bq34110_read_word(BQ34110_REG_CURRENT, &raw);
	if (rc == 0) {
		current.current_ma = (int16_t)raw; /* signed mA */
		current.valid_fields |= BATTERY_VALID_CURRENT;
	} else {
		LOG_WRN("BQ34110 current read failed: %d", rc);
		s_gauge_available = false;
		goto publish_snapshot;
	}

	rc = bq34110_read_word(BQ34110_REG_REMAINING_CAPACITY, &raw);
	if (rc == 0) {
		/* TI reports pack capacity in mAh. Do not multiply by a presumed
		 * cell count; the target has no verified pack/cell configuration.
		 */
		current.remaining_capacity_mah = raw;
		current.valid_fields |= BATTERY_VALID_REMAINING_CAPACITY;
	} else {
		LOG_WRN("BQ34110 remaining-capacity read failed: %d", rc);
		s_gauge_available = false;
		goto publish_snapshot;
	}

	rc = bq34110_read_word(BQ34110_REG_FULL_CAPACITY, &raw);
	if (rc == 0) {
		current.full_charge_capacity_mah = raw;
		current.valid_fields |= BATTERY_VALID_FULL_CAPACITY;
	} else {
		LOG_WRN("BQ34110 full-capacity read failed: %d", rc);
		s_gauge_available = false;
		goto publish_snapshot;
	}

	rc = bq34110_read_word(BQ34110_REG_AVERAGE_CURRENT, &raw);
	if (rc == 0) {
		current.average_current_ma = (int16_t)raw; /* signed mA */
		current.valid_fields |= BATTERY_VALID_AVERAGE_CURRENT;
	} else {
		LOG_WRN("BQ34110 average-current read failed: %d", rc);
		s_gauge_available = false;
		goto publish_snapshot;
	}

	rc = bq34110_read_word(BQ34110_REG_TIME_TO_EMPTY, &raw);
	if (rc == 0) {
		current.time_to_empty_seconds = (uint32_t)raw * 60U; /* minutes -> s */
		current.valid_fields |= BATTERY_VALID_TIME_TO_EMPTY;
	} else {
		LOG_WRN("BQ34110 time-to-empty read failed: %d", rc);
		s_gauge_available = false;
		goto publish_snapshot;
	}

	rc = bq34110_read_word(BQ34110_REG_AVERAGE_POWER, &raw);
	if (rc == 0) {
		/* The TI BQ34110 TRM specifies AveragePower in mW. The MAX78000
		 * wrapper multiplies this register by ten; that conflicts with TI's
		 * unit and is intentionally not repeated here.
		 */
		current.average_power_mw = (int16_t)raw;
		current.valid_fields |= BATTERY_VALID_AVERAGE_POWER;
	} else {
		LOG_WRN("BQ34110 average-power read failed: %d", rc);
		s_gauge_available = false;
		goto publish_snapshot;
	}

	rc = bq34110_read_word(BQ34110_REG_INTERNAL_TEMP, &raw);
	if (rc == 0) {
		/* TI reports 0.1 kelvin. Match reference integer Celsius conversion. */
		current.temperature_celsius = ((int32_t)raw - 2732) / 10;
		current.valid_fields |= BATTERY_VALID_TEMPERATURE;
	} else {
		LOG_WRN("BQ34110 temperature read failed: %d", rc);
		s_gauge_available = false;
		goto publish_snapshot;
	}

	publish_snapshot:
	current.valid_measurement =
		(current.valid_fields & BATTERY_VALID_VOLTAGE) != 0U;
	current.measurement_source = BATTERY_SOURCE_BQ34110;
	current.last_update_timestamp = k_uptime_get() / MSEC_PER_SEC;
	battery_monitor_publish_status(&current);
}

static void battery_monitor_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_mutex_lock(&s_monitor_state_mutex, K_FOREVER);
	bool suspended = s_monitor_suspended;
	k_mutex_unlock(&s_monitor_state_mutex);
	if (suspended) {
		return;
	}

	battery_monitor_read_cycle();

	/* Stop polling if the gauge is no longer available (e.g. disconnected) */
	if (!s_gauge_available) {
		return;
	}

	k_mutex_lock(&s_monitor_state_mutex, K_FOREVER);
	suspended = s_monitor_suspended;
	k_mutex_unlock(&s_monitor_state_mutex);
	if (!suspended) {
		(void)k_work_schedule(&s_update_work, BATTERY_UPDATE_PERIOD);
	}
}

void battery_monitor_get_status(struct battery_status *status)
{
	if (status == NULL) {
		return;
	}

	k_mutex_lock(&s_battery_status_mutex, K_FOREVER);
	*status = s_battery_status;
	k_mutex_unlock(&s_battery_status_mutex);
}

void battery_monitor_publish_status(const struct battery_status *status)
{
	if (status == NULL) {
		return;
	}

	k_mutex_lock(&s_battery_status_mutex, K_FOREVER);
	s_battery_status = *status;
	k_mutex_unlock(&s_battery_status_mutex);
}

void battery_monitor_clear_status(void)
{
	k_mutex_lock(&s_battery_status_mutex, K_FOREVER);
	memset(&s_battery_status, 0, sizeof(s_battery_status));
	s_battery_status.measurement_source = BATTERY_SOURCE_UNAVAILABLE;
	k_mutex_unlock(&s_battery_status_mutex);
}

int battery_monitor_init(void)
{
	uint16_t probe;
	k_work_init_delayable(&s_update_work, battery_monitor_work_handler);
	s_gauge_available = false;
	if (!i2c_is_ready_dt(&s_gauge)) {
		LOG_WRN("Battery gauge unavailable at startup; continuing with N/A telemetry");
		return 0;
	}
	/* Probe the gauge: if the first register read fails, the device is
	 * not connected. Do not start polling to avoid I2C bus errors. */
	if (bq34110_read_word(BQ34110_REG_VOLTAGE, &probe) != 0) {
		LOG_WRN("BQ34110 not responding at 0x%02x; continuing with N/A telemetry",
			s_gauge.addr);
		return 0;
	}
	s_gauge_available = true;
	LOG_INF("BQ34110 configured on I2C1 at address 0x%02x", s_gauge.addr);
	(void)k_work_schedule(&s_update_work, K_NO_WAIT);
	return 0;
}

void battery_monitor_suspend(void)
{
	struct k_work_sync sync;

	k_mutex_lock(&s_monitor_state_mutex, K_FOREVER);
	s_monitor_suspended = true;
	k_mutex_unlock(&s_monitor_state_mutex);
	(void)k_work_cancel_delayable_sync(&s_update_work, &sync);
}

void battery_monitor_resume(void)
{
	k_mutex_lock(&s_monitor_state_mutex, K_FOREVER);
	s_monitor_suspended = false;
	k_mutex_unlock(&s_monitor_state_mutex);
	(void)k_work_schedule(&s_update_work, K_NO_WAIT);
}
