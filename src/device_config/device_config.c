/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <string.h>
#include <stdio.h>
#include "device_config.h"

LOG_MODULE_REGISTER(device_config, LOG_LEVEL_INF);

static struct device_config config = {
    .device_name = "BACKBELT_0000",
    .mac_address = "00:00:00:00:00:00",
    .device_id = 0,
    .provisioned = false,
};

int device_config_init(void)
{
    LOG_INF("Initializing device configuration");

    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = ARRAY_SIZE(addrs);

    bt_id_get(addrs, &count);

    if (count > 0)
    {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addrs[0], addr_str, sizeof(addr_str));

        /* Remove " (random)" / " (public)" suffix */
        char *sp = strchr(addr_str, ' ');
        if (sp)
        {
            *sp = '\0';
        }

        strncpy(config.mac_address, addr_str, sizeof(config.mac_address) - 1);
        config.mac_address[sizeof(config.mac_address) - 1] = '\0';

        /* Extract last 2 bytes of MAC for device name */
        const char *last2 = (strlen(addr_str) >= 17) ? (addr_str + 12) : "0000";
        snprintf(config.device_name, sizeof(config.device_name), "BACKBELT_%s", last2);

        /* Remove colon from device name */
        char *col = strchr(config.device_name, ':');
        if (col)
        {
            memmove(col, col + 1, strlen(col));
        }

        LOG_INF("Device configured: %s (%s)", config.device_name, config.mac_address);
    }
    else
    {
        LOG_WRN("No Bluetooth address found, using defaults");
    }

    config.provisioned = true;
    return 0;
}

const char *device_config_get_name(void)
{
    return config.device_name;
}

const char *device_config_get_mac(void)
{
    return config.mac_address;
}

int device_config_format_with_mac(char *buffer, size_t size)
{
    if (!buffer || size == 0)
    {
        return -EINVAL;
    }

    return snprintf(buffer, size, "%s (%s)", config.device_name, config.mac_address);
}

int device_config_save(void)
{
    LOG_DBG("Saving device configuration");
    return 0;
}
