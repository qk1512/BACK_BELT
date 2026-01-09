/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bmi270.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(bmi270_sensor, LOG_LEVEL_INF);

/* Sample interval in milliseconds */
#define BMI270_SAMPLE_INTERVAL_MS 6000

/* Check if BMI270 sensor is available in device tree */
#if DT_HAS_COMPAT_STATUS_OKAY(bosch_bmi270)
#define BMI270_SENSOR_AVAILABLE 1
#else
#define BMI270_SENSOR_AVAILABLE 0
#endif

#if BMI270_SENSOR_AVAILABLE

static const struct device *bmi270_dev;
static bmi270_sensor_ble_send_cb ble_send_callback;

void bmi270_sensor_set_ble_callback(bmi270_sensor_ble_send_cb callback)
{
	ble_send_callback = callback;
}

int bmi270_sensor_init(void)
{
	int ret;
	struct sensor_value full_scale, sampling_freq, oversampling;

	bmi270_dev = DEVICE_DT_GET_ONE(bosch_bmi270);

	if (!device_is_ready(bmi270_dev)) {
		LOG_ERR("BMI270 sensor not ready");
		printk("BMI270 sensor not ready\n");
		return -ENODEV;
	}

	/* Configure accelerometer: 2G range, 100Hz, normal mode */
	full_scale.val1 = 2; /* G */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100; /* Hz */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1; /* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
	sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

	/* Configure gyroscope: 500dps range, 100Hz, normal mode */
	full_scale.val1 = 500; /* dps */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100; /* Hz */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1; /* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
	sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

	LOG_INF("BMI270 IMU sensor initialized successfully");
	printk("BMI270 sensor initialized: %s\n", bmi270_dev->name);
	return 0;
}

int bmi270_sensor_read(struct bmi270_sensor_data *data)
{
	int ret;
	struct sensor_value acc[3], gyr[3];

	if (!bmi270_dev) {
		printk("BMI270 sensor not initialized\n");
		return -EINVAL;
	}

	if (!data) {
		return -EINVAL;
	}

	/* Fetch all sensor data */
	ret = sensor_sample_fetch(bmi270_dev);
	if (ret) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
		return ret;
	}

	/* Read accelerometer data */
	ret = sensor_channel_get(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
	if (ret) {
		LOG_ERR("Failed to get ACCEL data: %d", ret);
		return ret;
	}

	/* Read gyroscope data */
	ret = sensor_channel_get(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
	if (ret) {
		LOG_ERR("Failed to get GYRO data: %d", ret);
		return ret;
	}

	/* Copy data to output structure */
	data->accel_x = acc[0];
	data->accel_y = acc[1];
	data->accel_z = acc[2];
	data->gyro_x = gyr[0];
	data->gyro_y = gyr[1];
	data->gyro_z = gyr[2];

	return 0;
}

void bmi270_sensor_log_data(const struct bmi270_sensor_data *data)
{
	if (!data) {
		return;
	}

	printk("BMI270: AX=%d.%02d AY=%d.%02d AZ=%d.%02d GX=%d.%02d GY=%d.%02d GZ=%d.%02d\n",
		data->accel_x.val1, abs(data->accel_x.val2 / 10000),
		data->accel_y.val1, abs(data->accel_y.val2 / 10000),
		data->accel_z.val1, abs(data->accel_z.val2 / 10000),
		data->gyro_x.val1, abs(data->gyro_x.val2 / 10000),
		data->gyro_y.val1, abs(data->gyro_y.val2 / 10000),
		data->gyro_z.val1, abs(data->gyro_z.val2 / 10000));
}

int bmi270_sensor_start(void)
{
	int ret;

	if (!bmi270_dev) {
		printk("BMI270 sensor not initialized\n");
		return -EINVAL;
	}

	LOG_INF("Starting BMI270 sensor");
	printk("Starting BMI270 IMU sensor thread...\n");

	/* Initial delay to allow sensor stabilization */
	k_sleep(K_SECONDS(2));
	printk("BMI270 sensor stabilization complete\n");

	LOG_INF("BMI270 sensor running in polling mode");
	printk("BMI270 sensor running in polling mode (10s interval)\n");
	
	while (1) {
		struct bmi270_sensor_data data;

		printk("Reading BMI270 IMU sensor data...\n");
		if (bmi270_sensor_read(&data) == 0) {
			printk("Sensor read successful, logging/sending...\n");
			/* Log sensor data to console */
			bmi270_sensor_log_data(&data);

			/* Send data over BLE if callback is registered */
			if (ble_send_callback) {
				char ble_msg[256];
				int len = snprintf(ble_msg, sizeof(ble_msg),
					"AX:%d.%02d AY:%d.%02d AZ:%d.%02d GX:%d.%02d GY:%d.%02d GZ:%d.%02d\n",
					data.accel_x.val1, abs(data.accel_x.val2 / 10000),
					data.accel_y.val1, abs(data.accel_y.val2 / 10000),
					data.accel_z.val1, abs(data.accel_z.val2 / 10000),
					data.gyro_x.val1, abs(data.gyro_x.val2 / 10000),
					data.gyro_y.val1, abs(data.gyro_y.val2 / 10000),
					data.gyro_z.val1, abs(data.gyro_z.val2 / 10000));
				
				if (len > 0 && len < sizeof(ble_msg)) {
					ble_send_callback((const uint8_t *)ble_msg, len);
				}
			}
		} else {
			printk("Sensor read failed\n");
		}

		k_sleep(K_MSEC(BMI270_SAMPLE_INTERVAL_MS));
	}

	return 0;
}

#else /* !BMI270_SENSOR_AVAILABLE */

/* Stub implementations when sensor is not available */

void bmi270_sensor_set_ble_callback(bmi270_sensor_ble_send_cb callback)
{
	ARG_UNUSED(callback);
}

int bmi270_sensor_init(void)
{
	LOG_WRN("BMI270 sensor not available in device tree");
	return -ENOTSUP;
}

int bmi270_sensor_read(struct bmi270_sensor_data *data)
{
	ARG_UNUSED(data);
	return -ENOTSUP;
}

void bmi270_sensor_log_data(const struct bmi270_sensor_data *data)
{
	ARG_UNUSED(data);
}

int bmi270_sensor_start(void)
{
	LOG_WRN("BMI270 sensor not available");
	return -ENOTSUP;
}

#endif /* BMI270_SENSOR_AVAILABLE */
