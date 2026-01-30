#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>


/* JSON message queue configuration */
#define JSON_MSG_MAX_SIZE 512
#define JSON_QUEUE_SIZE 10

/* Sensor buffer configuration */
#define SENSOR_BUFFER_SIZE 10  /* 10 samples x 100ms = 1s */

/* Sensor sample buffer for aggregating data */
struct sensor_sample_t
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

/* Circular buffer for sensor samples */
struct sensor_buffer_t
{
    struct sensor_sample_t samples[SENSOR_BUFFER_SIZE];
    uint16_t index;      /* Current write position */
    uint16_t count;      /* Number of valid samples */
};

/* JSON message structure for queue */
struct json_message
{
    char data[JSON_MSG_MAX_SIZE];
    uint16_t length;
    uint8_t priority; /* 0=low (periodic), 1=high (command response) */
};

/* Device information structure */
struct device_info_t
{
    char serial[32];     // Device serial number
    char name[32];       // Device name
    bool active;         // Device active status
    uint32_t time;       // Current time (Unix timestamp)
    char version[16];    // Firmware version
    char hw_version[16]; // Hardware version
    char type[8];        // Device type
    float volt;          // Battery voltage
    uint16_t period;     // Sampling period (seconds)
    bool status;         // Device status
};

struct sensor_data_t
{
    struct
    {
        bool valid;
        bool collecting;
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        /* Calibration offsets */
        int16_t offset_accel_x;
        int16_t offset_accel_y;
        int16_t offset_accel_z;
        int16_t offset_gyro_x;
        int16_t offset_gyro_y;
        int16_t offset_gyro_z;
    } bmi270;
};

extern struct sensor_data_t sensor_data;
extern struct device_info_t device_info;
extern struct k_mutex sensor_data_mutex;
extern struct k_sem ble_init_ok;

/* JSON message queue */
extern struct k_msgq json_tx_queue;

/* Sensor buffer management */
void sensor_buffer_add_sample(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                              int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);

int sensor_buffer_get_average(int16_t *avg_accel_x, int16_t *avg_accel_y, int16_t *avg_accel_z,
                              int16_t *avg_gyro_x, int16_t *avg_gyro_y, int16_t *avg_gyro_z);

int bmi270_ble_send_callback(const uint8_t *data, uint16_t len);


/* Unified BLE sender */
void unified_ble_sender_thread(void);
void sensor_handler_set_current_conn(struct bt_conn *conn);
void sensor_handler_set_auto_send(bool enabled);

/* Collecting flag getters for sensor drivers */
bool sensor_handler_is_bmi270_collecting(void);

/* JSON queue helper functions */
int json_queue_send(const char *json_data, uint16_t length, uint8_t priority);
int json_queue_send_nonblock(const char *json_data, uint16_t length, uint8_t priority);

/* Sensor threads */
void bmi270_sensor_thread(void);

/* Initialize all sensors */
int sensor_handler_init(void);

#endif // SENSOR_HANDLER_H