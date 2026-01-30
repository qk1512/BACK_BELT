#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <bluetooth/services/nus.h>
#include "sensor_handler.h"
#include "sensor_internal.h"

LOG_MODULE_REGISTER(sensor_handler);

/* Global sensor data and mutex */
K_MUTEX_DEFINE(sensor_data_mutex);
struct sensor_data_t sensor_data;

/* Global device info */
struct device_info_t device_info;

/* BLE semaphore - defined in main.c */
extern struct k_sem ble_init_ok;

/* JSON message queue - stores JSON strings ready to transmit */
K_MSGQ_DEFINE(json_tx_queue, sizeof(struct json_message), JSON_QUEUE_SIZE, 4);

/* Sensor buffer for aggregating samples */
static struct sensor_buffer_t sensor_buffer = {
    .index = 0,
    .count = 0,
};
static K_MUTEX_DEFINE(sensor_buffer_mutex);

/* Current connection - updated from bluetooth_handler */
static struct bt_conn *current_conn = NULL;
static K_MUTEX_DEFINE(current_conn_mutex); /* Protects current_conn access */

/* Auto-send control flag - default false (only send on command) */
static bool auto_send_enabled = true;

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

/* Public API: Set current BLE connection */
void sensor_handler_set_current_conn(struct bt_conn *conn)
{
    k_mutex_lock(&current_conn_mutex, K_FOREVER);

    /* Unref old connection if exists */
    if (current_conn && current_conn != conn)
    {
        bt_conn_unref(current_conn);
    }

    /* Ref new connection */
    current_conn = conn ? bt_conn_ref(conn) : NULL;

    k_mutex_unlock(&current_conn_mutex);
}

/* Public API: Enable/disable auto-send */
void sensor_handler_set_auto_send(bool enabled)
{
    auto_send_enabled = enabled;
    printk("Auto-send %s\n", enabled ? "enabled" : "disabled");
}

/* Internal access for split modules */
/* IMPORTANT: Caller must unref the returned connection when done */
struct bt_conn *sensor_get_current_conn(void)
{
    k_mutex_lock(&current_conn_mutex, K_FOREVER);
    struct bt_conn *conn = current_conn ? bt_conn_ref(current_conn) : NULL;
    k_mutex_unlock(&current_conn_mutex);
    return conn;
}

bool sensor_get_auto_send_enabled(void)
{
    return auto_send_enabled;
}


bool sensor_handler_is_bmi270_collecting(void)
{
    bool collecting;
    k_mutex_lock(&sensor_data_mutex, K_FOREVER);
    collecting = sensor_data.bmi270.collecting;
    k_mutex_unlock(&sensor_data_mutex);
    return collecting;
}

/* Sensor buffer management functions */

/* Add a sample to the buffer */
void sensor_buffer_add_sample(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                              int16_t gyro_x, int16_t gyro_y, int16_t gyro_z)
{
    k_mutex_lock(&sensor_buffer_mutex, K_FOREVER);
    
    sensor_buffer.samples[sensor_buffer.index].accel_x = accel_x;
    sensor_buffer.samples[sensor_buffer.index].accel_y = accel_y;
    sensor_buffer.samples[sensor_buffer.index].accel_z = accel_z;
    sensor_buffer.samples[sensor_buffer.index].gyro_x = gyro_x;
    sensor_buffer.samples[sensor_buffer.index].gyro_y = gyro_y;
    sensor_buffer.samples[sensor_buffer.index].gyro_z = gyro_z;
    
    sensor_buffer.index = (sensor_buffer.index + 1) % SENSOR_BUFFER_SIZE;
    if (sensor_buffer.count < SENSOR_BUFFER_SIZE)
    {
        sensor_buffer.count++;
    }
    
    k_mutex_unlock(&sensor_buffer_mutex);
}

/* Get average of buffer samples and reset buffer */
int sensor_buffer_get_average(int16_t *avg_accel_x, int16_t *avg_accel_y, int16_t *avg_accel_z,
                              int16_t *avg_gyro_x, int16_t *avg_gyro_y, int16_t *avg_gyro_z)
{
    k_mutex_lock(&sensor_buffer_mutex, K_FOREVER);
    
    if (sensor_buffer.count == 0)
    {
        k_mutex_unlock(&sensor_buffer_mutex);
        return -ENODATA;
    }
    
    /* Calculate averages */
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    
    for (uint16_t i = 0; i < sensor_buffer.count; i++)
    {
        sum_ax += sensor_buffer.samples[i].accel_x;
        sum_ay += sensor_buffer.samples[i].accel_y;
        sum_az += sensor_buffer.samples[i].accel_z;
        sum_gx += sensor_buffer.samples[i].gyro_x;
        sum_gy += sensor_buffer.samples[i].gyro_y;
        sum_gz += sensor_buffer.samples[i].gyro_z;
    }
    
    *avg_accel_x = (int16_t)(sum_ax / sensor_buffer.count);
    *avg_accel_y = (int16_t)(sum_ay / sensor_buffer.count);
    *avg_accel_z = (int16_t)(sum_az / sensor_buffer.count);
    *avg_gyro_x = (int16_t)(sum_gx / sensor_buffer.count);
    *avg_gyro_y = (int16_t)(sum_gy / sensor_buffer.count);
    *avg_gyro_z = (int16_t)(sum_gz / sensor_buffer.count);
    
    /* Reset buffer */
    sensor_buffer.index = 0;
    sensor_buffer.count = 0;
    
    k_mutex_unlock(&sensor_buffer_mutex);
    
    return 0;
}

/* JSON queue helper functions */
int json_queue_send(const char *json_data, uint16_t length, uint8_t priority)
{
    if (!json_data || length == 0 || length > JSON_MSG_MAX_SIZE)
    {
        LOG_ERR("Invalid JSON message: len=%u", length);
        return -EINVAL;
    }

    struct json_message msg;
    memcpy(msg.data, json_data, length);
    msg.length = length;
    msg.priority = priority;

    /* Blocking send - wait if queue is full */
    int ret = k_msgq_put(&json_tx_queue, &msg, K_FOREVER);
    if (ret != 0)
    {
        LOG_ERR("Failed to queue JSON message: %d", ret);
        return ret;
    }

    LOG_DBG("Queued JSON (len=%u, prio=%u)", length, priority);
    return 0;
}

int json_queue_send_nonblock(const char *json_data, uint16_t length, uint8_t priority)
{
    if (!json_data || length == 0 || length > JSON_MSG_MAX_SIZE)
    {
        LOG_ERR("Invalid JSON message: len=%u", length);
        return -EINVAL;
    }

    struct json_message msg;
    memcpy(msg.data, json_data, length);
    msg.length = length;
    msg.priority = priority;

    /* Non-blocking send - drop if queue is full */
    int ret = k_msgq_put(&json_tx_queue, &msg, K_NO_WAIT);
    if (ret == -ENOMSG)
    {
        LOG_WRN("JSON queue full, dropping message (len=%u)", length);
        return -ENOMSG;
    }
    else if (ret != 0)
    {
        LOG_ERR("Failed to queue JSON message: %d", ret);
        return ret;
    }

    LOG_DBG("Queued JSON non-block (len=%u, prio=%u)", length, priority);
    return 0;
}

int bmi270_ble_send_callback(const uint8_t *data, uint16_t len)
{
    return bmi270_ble_send_callback_impl(data, len);
}

void bmi270_sensor_thread(void)
{
    bmi270_sensor_thread_impl();
}

void unified_ble_sender_thread(void)
{
    unified_ble_sender_thread_impl();
}

/* BLE transmit thread - dequeues JSON messages and sends over BLE */
void ble_tx_thread(void)
{
    printk("BLE TX thread started\n");

    /* Wait for BLE init */
    k_sem_take(&ble_init_ok, K_FOREVER);
    printk("BLE TX thread ready\n");

    struct json_message msg;

    while (1)
    {
        /* Block waiting for message from queue */
        int ret = k_msgq_get(&json_tx_queue, &msg, K_FOREVER);
        if (ret != 0)
        {
            LOG_ERR("Failed to dequeue message: %d", ret);
            printk("ERROR: Failed to dequeue from json_tx_queue: %d\n", ret);
            k_msleep(100);
            continue;
        }

        printk("BLE TX: Dequeued message (len=%u, prio=%u)\n", msg.length, msg.priority);

        /* Get current connection (ref'd) */
        struct bt_conn *conn = sensor_get_current_conn();
        if (!conn)
        {
            LOG_WRN("No BLE connection, dropping message (len=%u)", msg.length);
            printk("WARN: No BLE connection, dropping message\n");
            k_msleep(100);
            continue;
        }

        printk("BLE TX: Got connection, preparing to send...\n");

        /* Add newline if not present */
        if (msg.data[msg.length - 1] != '\n')
        {
            if (msg.length < JSON_MSG_MAX_SIZE - 1)
            {
                msg.data[msg.length++] = '\n';
            }
        }

        /* Send over BLE with retry logic */
        int attempts = 0;
        const int max_attempts = 3;

        while (attempts < max_attempts)
        {
            ret = bt_nus_send(conn, (uint8_t *)msg.data, msg.length);
            if (ret == 0)
            {
                LOG_DBG("Sent JSON (len=%u, prio=%u)", msg.length, msg.priority);
                printk("BLE TX: Successfully sent (len=%u)\n", msg.length);
                break;
            }

            LOG_WRN("BLE send failed (attempt %d/%d): %d",
                    attempts + 1, max_attempts, ret);
            printk("BLE TX: Send failed (attempt %d/%d): %d\n",
                   attempts + 1, max_attempts, ret);
            attempts++;

            if (attempts < max_attempts)
            {
                k_msleep(50); /* Wait before retry */
            }
        }

        if (attempts >= max_attempts)
        {
            LOG_ERR("Dropped message after %d attempts", max_attempts);
        }

        /* Unref connection when done */
        if (conn)
        {
            bt_conn_unref(conn);
        }

        /* Small delay to prevent BLE buffer overflow */
        k_msleep(20);
    }
}

/* Public API: Initialization */
int sensor_handler_init(void)
{
    return sensor_handler_init_impl();
}


K_THREAD_DEFINE(bmi270_sensor_thread_id, STACKSIZE, bmi270_sensor_thread, NULL, NULL,
                NULL, PRIORITY - 1, 0, 0);

K_THREAD_DEFINE(unified_sender_thread_id, STACKSIZE, unified_ble_sender_thread, NULL, NULL,
                NULL, PRIORITY - 1, 0, 0);

/* BLE TX thread - handles queue dequeue and transmission */
K_THREAD_DEFINE(ble_tx_thread_id, STACKSIZE, ble_tx_thread, NULL, NULL,
                NULL, PRIORITY, 0, 0);
