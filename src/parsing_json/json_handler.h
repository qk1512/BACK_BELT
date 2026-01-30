/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef JSON_HANDLER_H
#define JSON_HANDLER_H

#include <zephyr/kernel.h>
#include "../thread_handler/sensor_handler/sensor_handler.h"

/**
 * @brief Build JSON string from all sensor data
 *
 * @param sensor_data Pointer to sensor data structure
 * @param buffer Output buffer for JSON string
 * @param buffer_size Size of output buffer
 * @return Length of JSON string, or negative error code
 */
int json_build_sensor_data(const struct sensor_data_t *sensor_data,
                           char *buffer, size_t buffer_size);

/**
 * @brief Build JSON string for device information
 *
 * @param device_info Pointer to device info structure
 * @param buffer Output buffer for JSON string
 * @param buffer_size Size of output buffer
 * @return Length of JSON string, or negative error code
 */
int json_build_device_info(const struct device_info_t *device_info,
                           char *buffer, size_t buffer_size);

int json_build_bmi270_data(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                           int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                           char *buffer, size_t buffer_size);

#endif /* JSON_HANDLER_H */
