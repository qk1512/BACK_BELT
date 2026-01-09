/*
 * Copyright (c) 2023 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BMI270_SENSOR_H_
#define BMI270_SENSOR_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/**
 * @brief Structure to hold all BMI270 sensor readings (accelerometer and gyroscope)
 */
struct bmi270_sensor_data {
	struct sensor_value accel_x;
	struct sensor_value accel_y;
	struct sensor_value accel_z;
	struct sensor_value gyro_x;
	struct sensor_value gyro_y;
	struct sensor_value gyro_z;
};

/**
 * @brief Callback function type for sending sensor data via BLE
 * 
 * @param data Pointer to buffer containing formatted sensor data
 * @param len Length of data to send
 * @return 0 on success, negative error code on failure
 */
typedef int (*bmi270_sensor_ble_send_cb)(const uint8_t *data, uint16_t len);

/**
 * @brief Set the BLE send callback function
 * 
 * @param callback Function to call when sending data via BLE
 */
void bmi270_sensor_set_ble_callback(bmi270_sensor_ble_send_cb callback);

/**
 * @brief Initialize the BMI270 IMU sensor
 * 
 * @return 0 on success, negative error code on failure
 */
int bmi270_sensor_init(void);

/**
 * @brief Read all sensor data from the BMI270
 * 
 * @param data Pointer to structure to store sensor readings
 * @return 0 on success, negative error code on failure
 */
int bmi270_sensor_read(struct bmi270_sensor_data *data);

/**
 * @brief Log all sensor data
 * 
 * @param data Pointer to sensor data to log
 */
void bmi270_sensor_log_data(const struct bmi270_sensor_data *data);

/**
 * @brief Start the BMI270 sensor operation
 * 
 * This function will block and continuously read sensor data at regular intervals.
 * It should be called from a dedicated thread.
 * 
 * @return 0 on success, negative error code on failure
 */
int bmi270_sensor_start(void);

#endif /* BMI270_SENSOR_H_ */