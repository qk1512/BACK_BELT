/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef SENSOR_INTERNAL_H
#define SENSOR_INTERNAL_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include "sensor_handler.h"

/* Internal callback functions (implemented in sensor_callbacks.c) */
int bmi270_ble_send_callback_impl(const uint8_t *data, uint16_t len);

/* Internal thread functions (implemented in sensor_threads.c) */
void bmi270_sensor_thread_impl(void);
void unified_ble_sender_thread_impl(void);


/* Internal initialization (implemented in sensor_init.c) */
int sensor_handler_init_impl(void);

/* Shared data access - defined in sensor_handler.c */
extern struct bt_conn *sensor_get_current_conn(void);
extern bool sensor_get_auto_send_enabled(void);

#endif /* SENSOR_INTERNAL_H */
