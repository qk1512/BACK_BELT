/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include "sensor_internal.h"

LOG_MODULE_DECLARE(sensor_handler);


/* BLE send callback for BMI270 sensor */
int bmi270_ble_send_callback_impl(const uint8_t *data, uint16_t len)
{
    if (sensor_data.bmi270.collecting == false)
    {
        return -1;
    }

    const char *str = (const char *)data;
    int ax1, ax2, ay1, ay2, az1, az2, gx1, gx2, gy1, gy2, gz1, gz2;

    if (sscanf(str, "AX:%d.%02d AY:%d.%02d AZ:%d.%02d GX:%d.%02d GY:%d.%02d GZ:%d.%02d",
               &ax1, &ax2, &ay1, &ay2, &az1, &az2, &gx1, &gx2, &gy1, &gy2, &gz1, &gz2) == 12)
    {
        k_mutex_lock(&sensor_data_mutex, K_FOREVER);
        sensor_data.bmi270.valid = true;
        /* Store raw values */
        int16_t raw_accel_x = ax1 * 100 + ax2;
        int16_t raw_accel_y = ay1 * 100 + ay2;
        int16_t raw_accel_z = az1 * 100 + az2;
        int16_t raw_gyro_x = gx1 * 100 + gx2;
        int16_t raw_gyro_y = gy1 * 100 + gy2;
        int16_t raw_gyro_z = gz1 * 100 + gz2;
        /* Apply calibration offsets */
        sensor_data.bmi270.accel_x = raw_accel_x - sensor_data.bmi270.offset_accel_x;
        sensor_data.bmi270.accel_y = raw_accel_y - sensor_data.bmi270.offset_accel_y;
        sensor_data.bmi270.accel_z = raw_accel_z - sensor_data.bmi270.offset_accel_z;
        sensor_data.bmi270.gyro_x = raw_gyro_x - sensor_data.bmi270.offset_gyro_x;
        sensor_data.bmi270.gyro_y = raw_gyro_y - sensor_data.bmi270.offset_gyro_y;
        sensor_data.bmi270.gyro_z = raw_gyro_z - sensor_data.bmi270.offset_gyro_z;
        k_mutex_unlock(&sensor_data_mutex);
    }
    return 0;
}

