/*
 * Copyright (c) 2023 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BH1749_SENSOR_H_
#define BH1749_SENSOR_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/**
 * @brief Structure to hold all BH1749 color sensor readings
 */
struct bh1749_sensor_data {
	struct sensor_value red;
	struct sensor_value green;
	struct sensor_value blue;
	struct sensor_value ir;
};

/**
 * @brief Callback function type for sending sensor data via BLE
 * 
 * @param data Pointer to buffer containing formatted sensor data
 * @param len Length of data to send
 * @return 0 on success, negative error code on failure
 */
typedef int (*bh1749_sensor_ble_send_cb)(const uint8_t *data, uint16_t len);

/**
 * @brief Set the BLE send callback function
 * 
 * @param callback Function to call when sending data via BLE
 */
void bh1749_sensor_set_ble_callback(bh1749_sensor_ble_send_cb callback);

/**
 * @brief Initialize the BH1749 color sensor
 * 
 * @return 0 on success, negative error code on failure
 */
int bh1749_sensor_init(void);

/**
 * @brief Read all sensor data from the BH1749
 * 
 * @param data Pointer to structure to store sensor readings
 * @return 0 on success, negative error code on failure
 */
int bh1749_sensor_read(struct bh1749_sensor_data *data);

/**
 * @brief Log all sensor data
 * 
 * @param data Pointer to sensor data to log
 */
void bh1749_sensor_log_data(const struct bh1749_sensor_data *data);

/**
 * @brief Start the BH1749 sensor operation
 * 
 * This function will block and continuously read sensor data at regular intervals.
 * It should be called from a dedicated thread.
 * 
 * @return 0 on success, negative error code on failure
 */
int bh1749_sensor_start(void);

#endif /* BH1749_SENSOR_H_ */