/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>
#include <string.h>
#include <stdio.h>
#include "command_handler.h"

LOG_MODULE_REGISTER(command_handler, LOG_LEVEL_INF);

/**
 * @brief Simple command parser for BLE commands
 *
 * Commands format: "CMD:value"
 * Examples:
 *   - "INFO" - request device info
 *   - "CALIBRATE" - start sensor calibration
 *   - "SENSOR:ON" - enable sensor
 *   - "SENSOR:OFF" - disable sensor
 */

int command_handler_process(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    if (!data || len == 0)
    {
        return -EINVAL;
    }

    char cmd_buffer[128];
    if (len >= sizeof(cmd_buffer))
    {
        LOG_WRN("Command too long: %u bytes", len);
        return -EMSGSIZE;
    }

    /* Copy and null-terminate command */
    memcpy(cmd_buffer, data, len);
    cmd_buffer[len] = '\0';

    LOG_INF("Processing command: %s", cmd_buffer);

    /* Parse commands */
    if (strncmp(cmd_buffer, "INFO", 4) == 0)
    {
        LOG_INF("INFO command received");
        /* Handle device info request */
    }
    else if (strncmp(cmd_buffer, "CALIBRATE", 9) == 0)
    {
        LOG_INF("CALIBRATE command received");
        /* Handle sensor calibration */
    }
    else if (strncmp(cmd_buffer, "SENSOR:ON", 9) == 0)
    {
        LOG_INF("SENSOR:ON command received");
        /* Enable sensor */
    }
    else if (strncmp(cmd_buffer, "SENSOR:OFF", 10) == 0)
    {
        LOG_INF("SENSOR:OFF command received");
        /* Disable sensor */
    }
    else
    {
        LOG_WRN("Unknown command: %s", cmd_buffer);
        return -ENOTSUP;
    }

    return 0;
}

int command_handler_init(void)
{
    LOG_INF("Command handler initialized");
    return 0;
}
