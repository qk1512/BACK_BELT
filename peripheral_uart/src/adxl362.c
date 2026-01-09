/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adxl362.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(adxl362_sensor, LOG_LEVEL_INF);

/* Sample interval in milliseconds */
#define ADXL362_SAMPLE_INTERVAL_MS 8000

/* Check if ADXL362 sensor is available in device tree */
#if DT_HAS_COMPAT_STATUS_OKAY(adi_adxl362)
#define ADXL362_SENSOR_AVAILABLE 1
#else
#define ADXL362_SENSOR_AVAILABLE 0
#endif

#if ADXL362_SENSOR_AVAILABLE

static const struct device *adxl362_dev;
static adxl362_sensor_ble_send_cb ble_send_callback;

void adxl362_sensor_set_ble_callback(adxl362_sensor_ble_send_cb callback)
{
	ble_send_callback = callback;
}

int adxl362_sensor_init(void)
{
	int ret;
	struct sensor_value odr;

	adxl362_dev = DEVICE_DT_GET_ONE(adi_adxl362);

	if (!device_is_ready(adxl362_dev)) {
		LOG_ERR("ADXL362 sensor not ready");
		printk("ADXL362 sensor not ready\n");
		return -ENODEV;
	}

	/* Set sampling frequency */
	ret = sensor_attr_get(adxl362_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	/* If we don't get a frequency > 0, we set one */
	if (ret != 0 || (odr.val1 == 0 && odr.val2 == 0)) {
		odr.val1 = 100;
		odr.val2 = 0;

		ret = sensor_attr_set(adxl362_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

		if (ret != 0) {
			LOG_WRN("Failed to set sampling frequency");
		}
	}

	LOG_INF("ADXL362 accelerometer sensor initialized successfully");
	printk("ADXL362 sensor initialized: %s\n", adxl362_dev->name);
	return 0;
}

int adxl362_sensor_read(struct adxl362_sensor_data *data)
{
	int ret;

	if (!adxl362_dev) {
		printk("❌ ADXL362 sensor not initialized\n");
		return -EINVAL;
	}

	if (!data) {
		return -EINVAL;
	}

	/* Fetch all sensor data */
	ret = sensor_sample_fetch(adxl362_dev);
	if (ret) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
		return ret;
	}

	/* Read accelerometer X axis */
	ret = sensor_channel_get(adxl362_dev, SENSOR_CHAN_ACCEL_X, &data->accel_x);
	if (ret) {
		LOG_ERR("Failed to get ACCEL_X: %d", ret);
		return ret;
	}

	/* Read accelerometer Y axis */
	ret = sensor_channel_get(adxl362_dev, SENSOR_CHAN_ACCEL_Y, &data->accel_y);
	if (ret) {
		LOG_ERR("Failed to get ACCEL_Y: %d", ret);
		return ret;
	}

	/* Read accelerometer Z axis */
	ret = sensor_channel_get(adxl362_dev, SENSOR_CHAN_ACCEL_Z, &data->accel_z);
	if (ret) {
		LOG_ERR("Failed to get ACCEL_Z: %d", ret);
		return ret;
	}

	return 0;
}

void adxl362_sensor_log_data(const struct adxl362_sensor_data *data)
{
	if (!data) {
		return;
	}

	printk("ADXL362: X=%d.%02d Y=%d.%02d Z=%d.%02d m/s^2\n",
		data->accel_x.val1, abs(data->accel_x.val2 / 10000),
		data->accel_y.val1, abs(data->accel_y.val2 / 10000),
		data->accel_z.val1, abs(data->accel_z.val2 / 10000));
}

int adxl362_sensor_start(void)
{
	int ret;

	if (!adxl362_dev) {
		printk("ADXL362 sensor not initialized\n");
		return -EINVAL;
	}

	LOG_INF("Starting ADXL362 sensor");
	printk("Starting ADXL362 accelerometer sensor thread...\n");

	/* Initial delay to allow sensor stabilization */
	k_sleep(K_SECONDS(2));
	printk("ADXL362 sensor stabilization complete\n");

	LOG_INF("ADXL362 sensor running in polling mode");
	printk("ADXL362 sensor running in polling mode (8s interval)\n");
	
	while (1) {
		struct adxl362_sensor_data data;

		printk("Reading ADXL362 accelerometer data...\n");
		if (adxl362_sensor_read(&data) == 0) {
			printk("Sensor read successful, logging/sending...\n");
			/* Log sensor data to console */
			adxl362_sensor_log_data(&data);

			/* Send data over BLE if callback is registered */
			if (ble_send_callback) {
				char ble_msg[128];
				int len = snprintf(ble_msg, sizeof(ble_msg),
					"X:%d.%02d Y:%d.%02d Z:%d.%02d m/s^2\n",
					data.accel_x.val1, abs(data.accel_x.val2 / 10000),
					data.accel_y.val1, abs(data.accel_y.val2 / 10000),
					data.accel_z.val1, abs(data.accel_z.val2 / 10000));
				
				if (len > 0 && len < sizeof(ble_msg)) {
					ble_send_callback((const uint8_t *)ble_msg, len);
				}
			}
		} else {
			printk("Sensor read failed\n");
		}

		k_sleep(K_MSEC(ADXL362_SAMPLE_INTERVAL_MS));
	}

	return 0;
}

#else /* !ADXL362_SENSOR_AVAILABLE */

/* Stub implementations when sensor is not available */

void adxl362_sensor_set_ble_callback(adxl362_sensor_ble_send_cb callback)
{
	ARG_UNUSED(callback);
}

int adxl362_sensor_init(void)
{
	LOG_WRN("ADXL362 sensor not available in device tree");
	return -ENOTSUP;
}

int adxl362_sensor_read(struct adxl362_sensor_data *data)
{
	ARG_UNUSED(data);
	return -ENOTSUP;
}

void adxl362_sensor_log_data(const struct adxl362_sensor_data *data)
{
	ARG_UNUSED(data);
}

int adxl362_sensor_start(void)
{
	LOG_WRN("ADXL362 sensor not available");
	return -ENOTSUP;
}

#endif /* ADXL362_SENSOR_AVAILABLE */
