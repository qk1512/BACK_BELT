/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/addr.h>

/**
 * @brief Device configuration structure
 */
struct device_config
{
    char device_name[32];
    char mac_address[18];
    uint32_t device_id;
    bool provisioned;
};

/**
 * @brief Initialize device configuration from flash/settings
 *
 * @return 0 on success, negative error code on failure
 */
int device_config_init(void);

/**
 * @brief Get device name
 *
 * @return Pointer to device name string
 */
const char *device_config_get_name(void);

/**
 * @brief Get MAC address
 *
 * @return Pointer to MAC address string
 */
const char *device_config_get_mac(void);

/**
 * @brief Format device name with MAC address
 *
 * @param buffer Output buffer
 * @param size Buffer size
 * @return Length of formatted string
 */
int device_config_format_with_mac(char *buffer, size_t size);

/**
 * @brief Save device configuration
 *
 * @return 0 on success, negative error code on failure
 */
int device_config_save(void);

#endif /* DEVICE_CONFIG_H */
