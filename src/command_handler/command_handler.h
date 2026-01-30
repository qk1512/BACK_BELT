/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>

/**
 * @brief Process incoming BLE command
 *
 * @param conn BLE connection
 * @param data Command data buffer
 * @param len Command data length
 * @return 0 on success, negative error code on failure
 */
int command_handler_process(struct bt_conn *conn, const uint8_t *data, uint16_t len);

/**
 * @brief Initialize command handler
 *
 * @return 0 on success, negative error code on failure
 */
int command_handler_init(void);

#endif /* COMMAND_HANDLER_H */
