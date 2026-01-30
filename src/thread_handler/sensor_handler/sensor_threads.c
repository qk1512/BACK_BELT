/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>
#include "sensor_internal.h"
#include "../../sensor/bmi270.h"
#include "../../parsing_json/json_handler.h"
#include <zephyr/drivers/sensor.h>
#include <math.h>

LOG_MODULE_DECLARE(sensor_handler);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7


/* BMI270 IMU sensor thread */
void bmi270_sensor_thread_impl(void)
{
    printk("BMI270 sensor thread started\n");

    /* Wait for BLE to be ready before starting sensor */
    printk("Waiting for BLE init...\n");
    int ret = k_sem_take(&ble_init_ok, K_SECONDS(30));
    if (ret != 0)
    {
        printk("Timeout waiting for BLE init: %d\n", ret);
        return;
    }
    printk("BLE ready, starting BMI270 sensor...\n");

    /* Register BLE send callback */
    bmi270_sensor_set_ble_callback(bmi270_ble_send_callback);
    printk("BMI270 BLE callback registered\n");

    /* Start BMI270 sensor operation */
    int err = bmi270_sensor_start();
    if (err)
    {
        LOG_ERR("BMI270 sensor start failed (err: %d)", err);
        printk("BMI270 sensor start failed: %d\n", err);
    }
    else
    {
        printk("BMI270 sensor started successfully\n");
    }
    printk("BMI270 sensor thread exiting\n");
}


/* Unified BLE sender thread */
void unified_ble_sender_thread_impl(void)
{
    printk("Unified BLE sender thread started\n");

    /* Wait for BLE to be ready */
    k_sem_take(&ble_init_ok, K_FOREVER);
    printk("BLE ready, starting unified sender...\n");

    while (1)
    {
        /* Wait 1 second before sending buffer */
        k_sleep(K_MSEC(1000));

        /* Skip auto-send if disabled */
        if (!sensor_get_auto_send_enabled())
        {
            continue;
        }

        /* Get averages from buffer */
        int16_t avg_ax, avg_ay, avg_az, avg_gx, avg_gy, avg_gz;
        int ret = sensor_buffer_get_average(&avg_ax, &avg_ay, &avg_az, &avg_gx, &avg_gy, &avg_gz);
        
        if (ret == 0)
        {
            /* Build JSON with average values */
            char msg[JSON_MSG_MAX_SIZE];
            int len = json_build_bmi270_data(avg_ax, avg_ay, avg_az, 
                                            avg_gx, avg_gy, avg_gz,
                                            msg, JSON_MSG_MAX_SIZE);
            
            if (len > 0 && len < JSON_MSG_MAX_SIZE)
            {
                /* Queue JSON message instead of sending directly */
                int err = json_queue_send_nonblock(msg, len, 0); /* priority=0 (periodic) */
                if (err == 0)
                {
                    printk("Queued averaged sensor data\n");
                }
            }
        }

        /* Yield to other threads */
        k_yield();
    }
}
