/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include "buzzer_handler.h"

LOG_MODULE_REGISTER(buzzer_handler);

/* PWM device for buzzer */
static const struct pwm_dt_spec buzzer_pwm = {
    .dev = DEVICE_DT_GET(DT_NODELABEL(pwm1)),
    .channel = 0,
    .period = 0,
    .flags = PWM_POLARITY_INVERTED};

/* Thread ID for buzzer control */
// static k_tid_t buzzer_timer_thread_id = NULL;

/* Control flags */
static volatile bool buzzer_active = false;
static struct k_timer buzzer_timer;

/* Timer callback to stop buzzer */
static void buzzer_timer_callback(struct k_timer *timer)
{
    buzzer_stop();
}

int buzzer_handler_init(void)
{
    if (!device_is_ready(buzzer_pwm.dev))
    {
        LOG_ERR("PWM device %s is not ready", buzzer_pwm.dev->name);
        return -ENODEV;
    }

    /* Initialize timer for duration control */
    k_timer_init(&buzzer_timer, buzzer_timer_callback, NULL);

    LOG_INF("Buzzer initialized successfully on PWM channel %d", buzzer_pwm.channel);
    return 0;
}

int buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    int ret;

    if (!device_is_ready(buzzer_pwm.dev))
    {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    if (frequency_hz < 100 || frequency_hz > 10000)
    {
        LOG_WRN("Frequency %u Hz out of recommended range (100-10000 Hz)", frequency_hz);
    }

    /* Calculate period and pulse width (90% duty cycle for more power) */
    uint32_t period_ns = NSEC_PER_SEC / frequency_hz;
    uint32_t pulse_ns = (period_ns * 9) / 10; // 90% duty cycle

    printk("Setting PWM: freq=%u Hz, period=%u ns, pulse=%u ns (90%% duty)\\n",
           frequency_hz, period_ns, pulse_ns);

    /* Set PWM */
    ret = pwm_set_dt(&buzzer_pwm, period_ns, pulse_ns);
    if (ret)
    {
        LOG_ERR("Failed to set PWM: %d", ret);
        printk("PWM set failed with error: %d\n", ret);
        return ret;
    }

    printk("PWM set successfully\n");

    buzzer_active = true;

    /* Start timer if duration is specified */
    if (duration_ms > 0)
    {
        k_timer_start(&buzzer_timer, K_MSEC(duration_ms), K_NO_WAIT);
    }

    LOG_DBG("Playing tone: %u Hz for %u ms", frequency_hz, duration_ms);
    return 0;
}

int buzzer_stop(void)
{
    int ret;

    if (!buzzer_active)
    {
        return 0;
    }

    /* Stop timer */
    k_timer_stop(&buzzer_timer);

    /* Turn off PWM (0% duty cycle) */
    ret = pwm_set_dt(&buzzer_pwm, 0, 0);
    if (ret)
    {
        LOG_ERR("Failed to stop PWM: %d", ret);
        return ret;
    }

    buzzer_active = false;
    LOG_DBG("Buzzer stopped");
    return 0;
}

int buzzer_play_pattern(uint8_t beeps, uint32_t beep_duration_ms, uint32_t pause_duration_ms)
{
    int ret;

    for (uint8_t i = 0; i < beeps; i++)
    {
        /* Play beep at 2000 Hz */
        ret = buzzer_play_tone(2000, 0);
        if (ret)
        {
            return ret;
        }

        k_sleep(K_MSEC(beep_duration_ms));
        buzzer_stop();

        /* Pause between beeps (except after last beep) */
        if (i < beeps - 1)
        {
            k_sleep(K_MSEC(pause_duration_ms));
        }
    }

    return 0;
}

int buzzer_play_notification(enum buzzer_notification type)
{
    int ret = 0;

    switch (type)
    {
    case BUZZER_NOTIFY_BOOT:
        /* Rising tone pattern */
        buzzer_play_tone(1000, 100);
        k_sleep(K_MSEC(100));
        buzzer_play_tone(1500, 100);
        k_sleep(K_MSEC(100));
        buzzer_play_tone(2000, 150);
        k_sleep(K_MSEC(150));
        buzzer_stop();
        break;

    case BUZZER_NOTIFY_BLE_CONNECTED:
        /* Double beep */
        ret = buzzer_play_pattern(2, 100, 100);
        break;

    case BUZZER_NOTIFY_BLE_DISCONNECTED:
        /* Descending tone */
        buzzer_play_tone(2000, 100);
        k_sleep(K_MSEC(100));
        buzzer_play_tone(1000, 150);
        k_sleep(K_MSEC(150));
        buzzer_stop();
        break;

    case BUZZER_NOTIFY_ERROR:
        /* Rapid beeps */
        ret = buzzer_play_pattern(3, 100, 100);
        break;

    case BUZZER_NOTIFY_SUCCESS:
        /* Single long beep */
        buzzer_play_tone(2500, 300);
        k_sleep(K_MSEC(300));
        buzzer_stop();
        break;

    default:
        LOG_WRN("Unknown notification type: %d", type);
        ret = -EINVAL;
        break;
    }

    return ret;
}