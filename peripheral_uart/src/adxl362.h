/*
 * Copyright (c) 2023 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ADXL362_SENSOR_H_
#define ADXL362_SENSOR_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/**
 * @brief Structure to hold all ADXL362 sensor readings (3-axis accelerometer)
 */
struct adxl362_sensor_data {
	struct sensor_value accel_x;
	struct sensor_value accel_y;
	struct sensor_value accel_z;
};

/**
 * @brief Callback function type for sending sensor data via BLE
 * 
 * @param data Pointer to buffer containing formatted sensor data
 * @param len Length of data to send
 * @return 0 on success, negative error code on failure
 */
typedef int (*adxl362_sensor_ble_send_cb)(const uint8_t *data, uint16_t len);

/**
 * @brief Set the BLE send callback function
 * 
 * @param callback Function to call when sending data via BLE
 */
void adxl362_sensor_set_ble_callback(adxl362_sensor_ble_send_cb callback);

/**
 * @brief Initialize the ADXL362 accelerometer sensor
 * 
 * @return 0 on success, negative error code on failure
 */
int adxl362_sensor_init(void);

/**
 * @brief Read all sensor data from the ADXL362
 * 
 * @param data Pointer to structure to store sensor readings
 * @return 0 on success, negative error code on failure
 */
int adxl362_sensor_read(struct adxl362_sensor_data *data);

/**
 * @brief Log all sensor data
 * 
 * @param data Pointer to sensor data to log
 */
void adxl362_sensor_log_data(const struct adxl362_sensor_data *data);

/**
 * @brief Start the ADXL362 sensor operation
 * 
 * This function will block and continuously read sensor data at regular intervals.
 * It should be called from a dedicated thread.
 * 
 * @return 0 on success, negative error code on failure
 */
int adxl362_sensor_start(void);

#endif /* ADXL362_SENSOR_H_ */