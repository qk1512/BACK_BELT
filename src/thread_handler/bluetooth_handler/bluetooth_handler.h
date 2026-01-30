#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include <zephyr/bluetooth/conn.h>

/* Initialize Bluetooth subsystem and start advertising */
int bluetooth_handler_init(void);

/* Get current connection */
struct bt_conn *bluetooth_handler_get_current_conn(void);

/* Update device name at runtime */
void bluetooth_handler_update_device_name(void);

/* UART write thread for BLE */
void ble_write_thread(void);

#endif // BLUETOOTH_HANDLER_H