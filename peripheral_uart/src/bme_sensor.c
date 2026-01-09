/*
 * Copyright (c) 2023 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bme_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bme_sensor, LOG_LEVEL_INF);

/* Check if BME680 device exists in device tree */
#if DT_HAS_COMPAT_STATUS_OKAY(bosch_bme680)
/* Standard Zephyr BME680 driver */
#define BME_SENSOR_AVAILABLE 1
#define USE_STANDARD_DRIVER 1
#else
#define BME_SENSOR_AVAILABLE 0
#define USE_STANDARD_DRIVER 0
#endif

#if BME_SENSOR_AVAILABLE
/* Private variables */
static const struct device *bme_dev;
static bme_sensor_ble_send_cb ble_send_callback = NULL;

#define BME_SAMPLE_INTERVAL_MS 10000  /* 10 seconds */

void bme_sensor_set_ble_callback(bme_sensor_ble_send_cb callback)
{
	ble_send_callback = callback;
}

int bme_sensor_read(struct bme_sensor_data *data)
{
	int ret;

	if (!data || !bme_dev) {
		return -EINVAL;
	}

	ret = sensor_sample_fetch(bme_dev);
	if (ret) {
		LOG_ERR("Failed to fetch sample: %d", ret);
		return ret;
	}

	sensor_channel_get(bme_dev, SENSOR_CHAN_AMBIENT_TEMP, &data->temperature);
	sensor_channel_get(bme_dev, SENSOR_CHAN_PRESS, &data->pressure);
	sensor_channel_get(bme_dev, SENSOR_CHAN_HUMIDITY, &data->humidity);
	
#if USE_STANDARD_DRIVER
	/* Standard driver doesn't support IAQ, CO2, VOC - set to zero */
	data->iaq.val1 = 0;
	data->iaq.val2 = 0;
	data->co2.val1 = 0;
	data->co2.val2 = 0;
	data->voc.val1 = 0;
	data->voc.val2 = 0;
	sensor_channel_get(bme_dev, SENSOR_CHAN_GAS_RES, &data->voc); /* Gas resistance */
#else
	sensor_channel_get(bme_dev, SENSOR_CHAN_IAQ, &data->iaq);
	sensor_channel_get(bme_dev, SENSOR_CHAN_CO2, &data->co2);
	sensor_channel_get(bme_dev, SENSOR_CHAN_VOC, &data->voc);
#endif

	return 0;
}

void bme_sensor_log_data(const struct bme_sensor_data *data)
{
	if (!data) {
		return;
	}

#if USE_STANDARD_DRIVER
	printf("BME680: T=%d.%02dC, P=%d.%02dhPa, H=%d.%02d%%, Gas=%dOhms\n",
		data->temperature.val1, data->temperature.val2 / 10000,
		data->pressure.val1, data->pressure.val2 / 10000,
		data->humidity.val1, data->humidity.val2 / 10000,
		data->voc.val1);
#else
	printf("BME680: T=%d.%02dC, P=%d.%02dhPa, H=%d.%02d%%, IAQ=%d, CO2=%dppm, VOC=%dppb\n",
		data->temperature.val1, data->temperature.val2 / 10000,
		data->pressure.val1, data->pressure.val2 / 10000,
		data->humidity.val1, data->humidity.val2 / 10000,
		data->iaq.val1,
		data->co2.val1,
		data->voc.val1);
#endif
}

#if defined(CONFIG_APP_TRIGGER)
const struct sensor_trigger trig = {
	.chan = SENSOR_CHAN_ALL,
	.type = SENSOR_TRIG_TIMER,
};

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	struct bme_sensor_data data;

	if (bme_sensor_read(&data) == 0) {
		bme_sensor_log_data(&data);
	}
}
#endif /* defined(CONFIG_APP_TRIGGER) */

int bme_sensor_init(void)
{
	bme_dev = DEVICE_DT_GET_ANY(bosch_bme680);

	if (bme_dev == NULL) {
		LOG_ERR("No BME680 device found");
		return -ENODEV;
	}

	if (!device_is_ready(bme_dev)) {
		LOG_ERR("BME680 device is not ready");
		return -ENODEV;
	}

	LOG_INF("BME680 sensor initialized successfully");
	return 0;
}

int bme_sensor_start(void)
{
	int ret;

	if (!bme_dev) {
		LOG_ERR("Sensor not initialized");
		printf("BME sensor not initialized\n");
		return -EINVAL;
	}

	LOG_INF("Starting BME sensor");
	printf("Starting BME sensor thread...\n");

	/* Initial delay to allow sensor stabilization */
	k_sleep(K_SECONDS(5));
	printf("BME sensor stabilization complete\n");

#if defined(CONFIG_APP_TRIGGER)
	ret = sensor_trigger_set(bme_dev, &trig, trigger_handler);
	if (ret) {
		LOG_ERR("Failed to set trigger: %d", ret);
		return ret;
	}
	LOG_INF("BME sensor running in trigger mode");
#else
	LOG_INF("BME sensor running in polling mode");
	printf("BME sensor running in polling mode (10s interval)\n");
	while (1) {
		struct bme_sensor_data data;

		printf("Reading BME sensor data...\n");
		if (bme_sensor_read(&data) == 0) {
			printf("Sensor read successful, logging/sending...\n");
			/* Log sensor data to console */
			bme_sensor_log_data(&data);

			/* Send data over BLE if callback is registered */
			if (ble_send_callback) {
				char ble_msg[128];
				int len = snprintf(ble_msg, sizeof(ble_msg),
					"T:%d.%02dC P:%d.%02dhPa H:%d.%02d%% Gas:%luOhms\n",
					data.temperature.val1, abs(data.temperature.val2 / 10000),
					data.pressure.val1, abs(data.pressure.val2 / 10000),
					data.humidity.val1, abs(data.humidity.val2 / 10000),
					(unsigned long)data.gas_resistance.val1);
				
				if (len > 0 && len < sizeof(ble_msg)) {
					ble_send_callback((const uint8_t *)ble_msg, len);
				}
			}
		} else {
			printf("Sensor read failed\n");
		}

		k_sleep(K_MSEC(BME_SAMPLE_INTERVAL_MS));
	}
#endif /* defined(CONFIG_APP_TRIGGER) */

	return 0;
}

#else /* !BME_SENSOR_AVAILABLE */

void bme_sensor_set_ble_callback(bme_sensor_ble_send_cb callback)
{
	/* Do nothing */
}

int bme_sensor_init(void)
{
	printf("BME680 sensor not available in device tree\n");
	LOG_WRN("BME680 sensor not available in device tree");
	return -ENOTSUP;
}

int bme_sensor_start(void)
{
	printf("BME680 sensor not available - skipping\n");
	LOG_WRN("BME680 sensor not available");
	return -ENOTSUP;
}

int bme_sensor_read(struct bme_sensor_data *data)
{
	return -ENOTSUP;
}

void bme_sensor_log_data(const struct bme_sensor_data *data)
{
	/* Do nothing */
}

#endif /* BME_SENSOR_AVAILABLE */
