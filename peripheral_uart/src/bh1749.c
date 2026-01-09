/*
 * Copyright (c) 2019 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bh1749.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(bh1749_sensor, LOG_LEVEL_INF);

/* Sample interval in milliseconds */
#define BH1749_SAMPLE_INTERVAL_MS 14000

/* Check if BH1749 sensor is available in device tree */
#if DT_HAS_COMPAT_STATUS_OKAY(rohm_bh1749)
#define BH1749_SENSOR_AVAILABLE 1
#else
#define BH1749_SENSOR_AVAILABLE 0
#endif

#if BH1749_SENSOR_AVAILABLE

static const struct device *bh1749_dev;
static bh1749_sensor_ble_send_cb ble_send_callback;

void bh1749_sensor_set_ble_callback(bh1749_sensor_ble_send_cb callback)
{
	ble_send_callback = callback;
}

int bh1749_sensor_init(void)
{
	bh1749_dev = DEVICE_DT_GET_ONE(rohm_bh1749);

	if (!device_is_ready(bh1749_dev)) {
		LOG_ERR("BH1749 sensor not ready");
		printk("BH1749 sensor not ready\n");
		return -ENODEV;
	}

	LOG_INF("BH1749 color sensor initialized successfully");
	printk("BH1749 sensor initialized: %s\n", bh1749_dev->name);
	return 0;
}

int bh1749_sensor_read(struct bh1749_sensor_data *data)
{
	int ret;

	if (!bh1749_dev) {
		printk("BH1749 sensor not initialized\n");
		return -EINVAL;
	}

	if (!data) {
		return -EINVAL;
	}

	/* Fetch all sensor data */
	ret = sensor_sample_fetch_chan(bh1749_dev, SENSOR_CHAN_ALL);
	if (ret) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
		return ret;
	}

	/* Read RED channel */
	ret = sensor_channel_get(bh1749_dev, SENSOR_CHAN_RED, &data->red);
	if (ret) {
		LOG_ERR("Failed to get RED channel: %d", ret);
		return ret;
	}

	/* Read GREEN channel */
	ret = sensor_channel_get(bh1749_dev, SENSOR_CHAN_GREEN, &data->green);
	if (ret) {
		LOG_ERR("Failed to get GREEN channel: %d", ret);
		return ret;
	}

	/* Read BLUE channel */
	ret = sensor_channel_get(bh1749_dev, SENSOR_CHAN_BLUE, &data->blue);
	if (ret) {
		LOG_ERR("Failed to get BLUE channel: %d", ret);
		return ret;
	}

	/* Read IR channel */
	ret = sensor_channel_get(bh1749_dev, SENSOR_CHAN_IR, &data->ir);
	if (ret) {
		LOG_ERR("Failed to get IR channel: %d", ret);
		return ret;
	}

	return 0;
}

void bh1749_sensor_log_data(const struct bh1749_sensor_data *data)
{
	if (!data) {
		return;
	}

	printk("BH1749: R=%d, G=%d, B=%d, IR=%d\n",
		data->red.val1,
		data->green.val1,
		data->blue.val1,
		data->ir.val1);
}

int bh1749_sensor_start(void)
{
	int ret;

	if (!bh1749_dev) {
		printk("BH1749 sensor not initialized\n");
		return -EINVAL;
	}

	LOG_INF("Starting BH1749 sensor");
	printk("Starting BH1749 color sensor thread...\n");

	/* Initial delay to allow sensor stabilization */
	k_sleep(K_SECONDS(2));
	printk("BH1749 sensor stabilization complete\n");

	LOG_INF("BH1749 sensor running in polling mode");
	printk("BH1749 sensor running in polling mode (10s interval)\n");
	
	while (1) {
		struct bh1749_sensor_data data;

		printk("Reading BH1749 color sensor data...\n");
		if (bh1749_sensor_read(&data) == 0) {
			printk("Sensor read successful, logging/sending...\n");
			/* Log sensor data to console */
			bh1749_sensor_log_data(&data);

			/* Send data over BLE if callback is registered */
			if (ble_send_callback) {
				char ble_msg[128];
				int len = snprintf(ble_msg, sizeof(ble_msg),
					"R:%d G:%d B:%d IR:%d\n",
					data.red.val1,
					data.green.val1,
					data.blue.val1,
					data.ir.val1);
				
				if (len > 0 && len < sizeof(ble_msg)) {
					ble_send_callback((const uint8_t *)ble_msg, len);
				}
			}
		} else {
			printk("Sensor read failed\n");
		}

		k_sleep(K_MSEC(BH1749_SAMPLE_INTERVAL_MS));
	}

	return 0;
}

#else /* !BH1749_SENSOR_AVAILABLE */

/* Stub implementations when sensor is not available */

void bh1749_sensor_set_ble_callback(bh1749_sensor_ble_send_cb callback)
{
	ARG_UNUSED(callback);
}

int bh1749_sensor_init(void)
{
	LOG_WRN("BH1749 sensor not available in device tree");
	return -ENOTSUP;
}

int bh1749_sensor_read(struct bh1749_sensor_data *data)
{
	ARG_UNUSED(data);
	return -ENOTSUP;
}

void bh1749_sensor_log_data(const struct bh1749_sensor_data *data)
{
	ARG_UNUSED(data);
}

int bh1749_sensor_start(void)
{
	LOG_WRN("BH1749 sensor not available");
	return -ENOTSUP;
}

#endif /* BH1749_SENSOR_AVAILABLE */
