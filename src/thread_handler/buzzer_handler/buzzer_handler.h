/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BUZZER_HANDLER_H
#define BUZZER_HANDLER_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize the buzzer PWM peripheral
 *
 * @return 0 on success, negative errno code on failure
 */
int buzzer_handler_init(void);

/**
 * @brief Play a tone on the buzzer
 *
 * @param frequency_hz Frequency in Hz (100-10000 Hz typical range)
 * @param duration_ms Duration in milliseconds (0 for continuous)
 * @return 0 on success, negative errno code on failure
 */
int buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms);

/**
 * @brief Stop the buzzer
 *
 * @return 0 on success, negative errno code on failure
 */
int buzzer_stop(void);

/**
 * @brief Play a beep pattern
 *
 * @param beeps Number of beeps
 * @param beep_duration_ms Duration of each beep in ms
 * @param pause_duration_ms Pause between beeps in ms
 * @return 0 on success, negative errno code on failure
 */
int buzzer_play_pattern(uint8_t beeps, uint32_t beep_duration_ms, uint32_t pause_duration_ms);

/**
 * @brief Play predefined notification sounds
 */
enum buzzer_notification
{
    BUZZER_NOTIFY_BOOT,             /* Device boot sound */
    BUZZER_NOTIFY_BLE_CONNECTED,    /* BLE connection established */
    BUZZER_NOTIFY_BLE_DISCONNECTED, /* BLE disconnected */
    BUZZER_NOTIFY_ERROR,            /* Error notification */
    BUZZER_NOTIFY_SUCCESS,          /* Success notification */
};

int buzzer_play_notification(enum buzzer_notification type);

#endif /* BUZZER_HANDLER_H */
