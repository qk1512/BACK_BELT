/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) with AI Model Integration
 */

#include <zephyr/kernel.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>

#include "device_config/device_config.h"
#include "thread_handler/uart_handler/uart_handler.h"
#include "thread_handler/bluetooth_handler/bluetooth_handler.h"
#include "thread_handler/sensor_handler/sensor_handler.h"
#include "thread_handler/buzzer_handler/buzzer_handler.h"
#include "command_handler/command_handler.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

/* AI inference delay */
#define AI_INFERENCE_DELAY_MS 500

/* Export semaphore for sensor and BLE threads */
K_SEM_DEFINE(ble_init_ok, 0, 5);

/* Error handler */
static void error(void)
{
        dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

        while (true)
        {
                k_sleep(K_MSEC(1000));
        }
}

/* GPIO configuration */
static void configure_gpio(void)
{
        int err;
        err = dk_leds_init();
        if (err)
        {
                LOG_ERR("Cannot init LEDs (err: %d)", err);
        }
}

/* AI inference thread */
#define AI_STACK_SIZE 8192
#define AI_PRIORITY 7
K_THREAD_STACK_DEFINE(ai_stack, AI_STACK_SIZE);
static struct k_thread ai_thread;

static void ai_inference_thread(void *a, void *b, void *c)
{
        ARG_UNUSED(a);
        ARG_UNUSED(b);
        ARG_UNUSED(c);

        LOG_INF("AI inference thread started");

        /* Initialize AI model */
        setup();

        /* Continuous inference loop */
        while (true)
        {
                loop();
                k_msleep(AI_INFERENCE_DELAY_MS);
        }
}

/* Main function */
int main(void)
{
        int blink_status = 0;
        int err = 0;

        LOG_INF("Starting Nordic UART Service with AI Model Integration");

        /* Configure GPIO (buttons and LEDs) */
        configure_gpio();

        /* Initialize device configuration (load from flash) */
        err = device_config_init();
        if (err)
        {
                LOG_WRN("Device config initialization had issues (err: %d), using defaults", err);
        }

        /* Initialize UART */
        err = uart_init();
        if (err)
        {
                LOG_ERR("UART initialization failed");
                error();
        }

        /* Initialize Bluetooth and start advertising */
        err = bluetooth_handler_init();
        if (err)
        {
                LOG_ERR("Bluetooth initialization failed");
                error();
        }

        err = buzzer_handler_init();
        if (err)
        {
                LOG_ERR("Buzzer initialization failed (err: %d), continuing without buzzer", err);
        }
        else
        {
                LOG_INF("Buzzer initialized successfully");
                /* Play boot notification */
                buzzer_play_notification(BUZZER_NOTIFY_BOOT);
                k_sleep(K_MSEC(500));
        }

        /* Initialize all sensors */
        err = sensor_handler_init();
        if (err)
        {
                LOG_WRN("Sensor initialization had warnings (err: %d), continuing", err);
        }


        LOG_INF("Initialization complete");

}
