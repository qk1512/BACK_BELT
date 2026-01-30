/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <zephyr/sys/timeutil.h>
#include "json_handler.h"


int json_build_bmi270_data(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                           int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                           char *buffer, size_t buffer_size)
{
    if (!buffer || buffer_size == 0)
    {
        return -EINVAL;
    }

    return snprintf(buffer, buffer_size,
                    "\"accel\":[%d.%02d,%d.%02d,%d.%02d],"
                    "\"gyro\":[%d.%02d,%d.%02d,%d.%02d]",
                    accel_x / 100, abs(accel_x % 100),
                    accel_y / 100, abs(accel_y % 100),
                    accel_z / 100, abs(accel_z % 100),
                    gyro_x / 100, abs(gyro_x % 100),
                    gyro_y / 100, abs(gyro_y % 100),
                    gyro_z / 100, abs(gyro_z % 100));
}



int json_build_sensor_data(const struct sensor_data_t *sensor_data,
                           char *buffer, size_t buffer_size)
{
    if (!sensor_data || !buffer || buffer_size == 0)
    {
        return -EINVAL;
    }

    int len = 0;
    bool has_data = false;

    /* Get uptime in milliseconds */
    uint64_t uptime_ms = k_uptime_get();
    /* Start JSON object with timestamp */
    // printk("Building JSON sensor data at uptime: %llu ms\n", uptime_ms);

    len = snprintf(buffer, buffer_size, "{\"sensor\":{\"time\":%ld,\"status\":%s", (long)uptime_ms, "true");
    if (len < 0 || len >= buffer_size)
    {
        return -ENOMEM;
    }
    has_data = true;

   
    /* Add BMI270 data */
    if (sensor_data->bmi270.valid && sensor_data->bmi270.collecting)
    {
        if (has_data)
        {
            len += snprintf(buffer + len, buffer_size - len, ",");
        }
        int ret = json_build_bmi270_data(
            sensor_data->bmi270.accel_x,
            sensor_data->bmi270.accel_y,
            sensor_data->bmi270.accel_z,
            sensor_data->bmi270.gyro_x,
            sensor_data->bmi270.gyro_y,
            sensor_data->bmi270.gyro_z,
            buffer + len,
            buffer_size - len);
        if (ret < 0)
        {
            return ret;
        }
        len += ret;
        has_data = true;
    }

    /* Close JSON object */
    len += snprintf(buffer + len, buffer_size - len, "}}");

    if (len >= buffer_size)
    {
        return -ENOMEM;
    }

    return len;
}

