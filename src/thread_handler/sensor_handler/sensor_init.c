/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "sensor_internal.h"
#include "../../sensor/bmi270.h"

LOG_MODULE_DECLARE(sensor_handler);

void handler_device_info(char *mac_addr, size_t mac_addr_len,
                         char *device_name, size_t device_name_len)
{
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = ARRAY_SIZE(addrs);

    char addr_str[BT_ADDR_LE_STR_LEN] = "UNKNOWN";

    bt_id_get(addrs, &count);

    if (count > 0)
    {
        bt_addr_le_to_str(&addrs[0], addr_str, sizeof(addr_str));

        /* Remove " (random)" / " (public)" */
        char *sp = strchr(addr_str, ' ');
        if (sp)
        {
            *sp = '\0';
        }
    }

    /* addr_str now like: "D6:70:C8:9A:B5:71" */
    const char *last2 = (strlen(addr_str) >= 17) ? (addr_str + 12) : "00:00"; // "B5:71"

    char tmp[32];
    snprintf(tmp, sizeof(tmp), "BACKBELT_%s", last2); // "BACKBELT_B5:71"

    /* remove ':' -> "BACKBELT_B571" */
    char *col = strchr(tmp, ':');
    if (col)
    {
        memmove(col, col + 1, strlen(col)); // move includes '\0'
    }

    /* Output */
    if (mac_addr && mac_addr_len > 0)
    {
        strncpy(mac_addr, addr_str, mac_addr_len - 1);
        mac_addr[mac_addr_len - 1] = '\0';
    }

    if (device_name && device_name_len > 0)
    {
        strncpy(device_name, tmp, device_name_len - 1);
        device_name[device_name_len - 1] = '\0';
    }
}

/* Initialize all sensors and start their threads */
int sensor_handler_init_impl(void)
{
    int err;

    /* Initialize device info with default values */

    char mac_addr[32] = "UNKNOWN";
    char device_name[32] = "UNKNOWN";

    handler_device_info(mac_addr, sizeof(mac_addr), device_name, sizeof(device_name));

    strncpy(device_info.serial, mac_addr, sizeof(device_info.serial) - 1);
    strncpy(device_info.name, device_name, sizeof(device_info.name) - 1);

    device_info.active = true;
    device_info.time = 0; /* Can be updated with k_uptime_get() */
    strncpy(device_info.version, "1.0.0", sizeof(device_info.version) - 1);
    strncpy(device_info.hw_version, "nRF5340", sizeof(device_info.hw_version) - 1);
    strncpy(device_info.type, "BLE", sizeof(device_info.type) - 1);
    device_info.volt = 3.7f; /* Default battery voltage */
    // strncpy(device_info.volt, "1.0", sizeof(device_info.volt) - 1);
    device_info.period = 1000; /* Default 1000ms */
    device_info.status = true;

    LOG_INF("Device info initialized: %s (%s)", device_info.name, device_info.serial);


    /* Initialize BMI270 IMU sensor */
    err = bmi270_sensor_init();
    if (err)
    {
        LOG_WRN("BMI270 sensor initialization failed (err: %d), continuing without sensor", err);
    }
    else
    {
        sensor_data.bmi270.collecting = true;
        LOG_INF("BMI270 IMU sensor initialized, will print data every 10 seconds");
    }


    /* Signal all waiting threads (7 threads total) */
    k_sem_give(&ble_init_ok);
    k_sem_give(&ble_init_ok);
    k_sem_give(&ble_init_ok);

    return 0;
}
