/*
 * Copyright (c) 2023 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BME_SENSOR_H_
#define BME_SENSOR_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/**
 * @brief Structure to hold all BME680 sensor readings
 */
struct bme_sensor_data {
	struct sensor_value temperature;
	struct sensor_value pressure;
	struct sensor_value humidity;
	struct sensor_value iaq;
	struct sensor_value co2;
	struct sensor_value voc;
    struct sensor_value gas_resistance;
};

/**
 * @brief Callback function type for sending sensor data via BLE
 * 
 * @param data Pointer to buffer containing formatted sensor data
 * @param len Length of data to send
 * @return 0 on success, negative error code on failure
 */
typedef int (*bme_sensor_ble_send_cb)(const uint8_t *data, uint16_t len);

/**
 * @brief Set the BLE send callback function
 * 
 * @param callback Function to call when sending data via BLE
 */
void bme_sensor_set_ble_callback(bme_sensor_ble_send_cb callback);

/**
 * @brief Initialize the BME680 sensor
 * 
 * @return 0 on success, negative error code on failure
 */
int bme_sensor_init(void);

/**
 * @brief Read all sensor data from the BME680
 * 
 * @param data Pointer to structure to store sensor readings
 * @return 0 on success, negative error code on failure
 */
int bme_sensor_read(struct bme_sensor_data *data);

/**
 * @brief Log all sensor data
 * 
 * @param data Pointer to sensor data to log
 */
void bme_sensor_log_data(const struct bme_sensor_data *data);

/**
 * @brief Start the BME sensor thread/task
 * 
 * @return 0 on success, negative error code on failure
 */
int bme_sensor_start(void);

#endif /* BME_SENSOR_H_ */
