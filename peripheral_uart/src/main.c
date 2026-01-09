/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include "bme_sensor.h"
#include "bh1749.h"
#include "bmi270.h"
#include "adxl362.h"

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 5);

/* Shared sensor data structure */
struct {
	struct {
		bool valid;
		int16_t temp;
		int16_t pressure;
		int16_t humidity;
		uint32_t gas;
	} bme;
	struct {
		bool valid;
		int16_t red;
		int16_t green;
		int16_t blue;
		int16_t ir;
	} bh1749;
	struct {
		bool valid;
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;
		int16_t gyro_x;
		int16_t gyro_y;
		int16_t gyro_z;
	} bmi270;
	struct {
		bool valid;
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;
	} adxl362;
} sensor_data;

K_MUTEX_DEFINE(sensor_data_mutex);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		printf("⚡ UART_RX_RDY: Received %d bytes\r\n", evt->data.rx.len);
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;
		
		printf("📥 Buffer total length now: %d bytes\r\n", buf->len);

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			printf("🔚 Newline detected, disabling UART RX\r\n");
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		printf("🔄 UART_RX_DISABLED - Re-enabling reception\r\n");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		printf("📦 UART_RX_BUF_RELEASED\r\n");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data[0]);

		if (buf->len > 0) {
			printf("✅ Putting %d bytes into FIFO\r\n", buf->len);
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			printf("⚠️ Buffer empty, freeing it\r\n");
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service sample\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);

	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX)) {
		LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
	} else {
		LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
	}
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif




static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	/* In địa chỉ */
	printf("RX from %s, len=%d\r\n", addr, len);

	/* In payload dạng HEX (an toàn nhất) */
	printf("HEX: ");
	for (int i = 0; i < len; i++) {
		printf("%02X ", data[i]);
	}
	printf("\r\n");

	/* Nếu là ASCII/text thì in thêm dạng chuỗi */
	char buf[241];
	int n = MIN(len, sizeof(buf) - 1);
	memcpy(buf, data, n);
	buf[n] = '\0';
	printf("TXT: %s\r\n", buf);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	err = uart_init();
	if (err) {
		error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	/* Initialize BME680 sensor BEFORE signaling threads */
	err = bme_sensor_init();
	if (err) {
		LOG_WRN("BME sensor initialization failed (err: %d), continuing without sensor", err);
	} else {
		LOG_INF("BME sensor initialized, will print data every 10 seconds");
	}

	/* Initialize BH1749 color sensor */
	err = bh1749_sensor_init();
	if (err) {
		LOG_WRN("BH1749 sensor initialization failed (err: %d), continuing without sensor", err);
	} else {
		LOG_INF("BH1749 color sensor initialized, will print data every 10 seconds");
	}

	/* Initialize BMI270 IMU sensor */
	err = bmi270_sensor_init();
	if (err) {
		LOG_WRN("BMI270 sensor initialization failed (err: %d), continuing without sensor", err);
	} else {
		LOG_INF("BMI270 IMU sensor initialized, will print data every 10 seconds");
	}

	/* Initialize ADXL362 accelerometer sensor */
	err = adxl362_sensor_init();
	if (err) {
		LOG_WRN("ADXL362 sensor initialization failed (err: %d), continuing without sensor", err);
	} else {
		LOG_INF("ADXL362 accelerometer sensor initialized, will print data every 8 seconds");
	}

	/* Signal all waiting threads (ble_write, bme_sensor, bh1749, bmi270, adxl362) */
	k_sem_give(&ble_init_ok);
	k_sem_give(&ble_init_ok);
	k_sem_give(&ble_init_ok);
	k_sem_give(&ble_init_ok);
	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	struct uart_data_t nus_data = {
		.len = 0,
	};

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		printf("📤 Terminal input received: %d bytes\r\n", buf->len);

		int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
		int loc = 0;

		while (plen > 0) {
			memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
			nus_data.len += plen;
			loc += plen;

			if (nus_data.len >= sizeof(nus_data.data) ||
			   (nus_data.data[nus_data.len - 1] == '\n') ||
			   (nus_data.data[nus_data.len - 1] == '\r')) {
				
				/* In dữ liệu trước khi gửi */
				printf("Sending to BLE (%d bytes): ", nus_data.len);
				for (int i = 0; i < nus_data.len; i++) {
					printf("%02X ", nus_data.data[i]);
				}
				printf("\r\n");

				if (bt_nus_send(NULL, nus_data.data, nus_data.len)) {
					LOG_WRN("Failed to send data over BLE connection");
				} else {
					printf("Data sent to phone successfully!\r\n");
				}
				nus_data.len = 0;
			}

			plen = MIN(sizeof(nus_data.data), buf->len - loc);
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

/* BLE send callback for BME sensor */
static int bme_ble_send_callback(const uint8_t *data, uint16_t len)
{
	/* Parse and store data instead of sending */
	const char *str = (const char *)data;
	int t1, t2, p1, p2, h1, h2;
	unsigned long gas;
	
	if (sscanf(str, "T:%d.%02dC P:%d.%02dhPa H:%d.%02d%% Gas:%luOhms", 
		&t1, &t2, &p1, &p2, &h1, &h2, &gas) == 7) {
		k_mutex_lock(&sensor_data_mutex, K_FOREVER);
		sensor_data.bme.valid = true;
		sensor_data.bme.temp = t1 * 100 + t2;
		sensor_data.bme.pressure = p1 * 100 + p2;
		sensor_data.bme.humidity = h1 * 100 + h2;
		sensor_data.bme.gas = gas;
		k_mutex_unlock(&sensor_data_mutex);
		printk("BME data stored: T=%d.%02d P=%d.%02d H=%d.%02d Gas=%lu\n",
			t1, t2, p1, p2, h1, h2, gas);
	} else {
		printk("BME parse failed: %s\n", str);
	}
	return 0;
}

void bme_sensor_thread(void)
{
	printk("BME sensor thread started\n");
	
	/* Wait for BLE to be ready before starting sensor */
	printk("Waiting for BLE init...\n");
	int ret = k_sem_take(&ble_init_ok, K_SECONDS(30));
	if (ret != 0) {
		printk("Timeout waiting for BLE init: %d\n", ret);
		return;
	}
	printk("BLE ready, starting BME sensor...\n");

	/* Register BLE send callback */
	bme_sensor_set_ble_callback(bme_ble_send_callback);
	printk("BLE callback registered\n");

	/* Start BME sensor operation */
	int err = bme_sensor_start();
	if (err) {
		LOG_ERR("BME sensor start failed (err: %d)", err);
		printk("BME sensor start failed: %d\n", err);
	} else {
		printk("BME sensor started successfully\n");
	}
	printk("BME sensor thread exiting\n");
}

K_THREAD_DEFINE(bme_sensor_thread_id, STACKSIZE, bme_sensor_thread, NULL, NULL,
		NULL, PRIORITY - 1, 0, 0);

/* BLE send callback for BH1749 sensor */
static int bh1749_ble_send_callback(const uint8_t *data, uint16_t len)
{
	/* Parse and store data instead of sending */
	const char *str = (const char *)data;
	int r, g, b, ir;
	
	if (sscanf(str, "R:%d G:%d B:%d IR:%d", &r, &g, &b, &ir) == 4) {
		k_mutex_lock(&sensor_data_mutex, K_FOREVER);
		sensor_data.bh1749.valid = true;
		sensor_data.bh1749.red = r;
		sensor_data.bh1749.green = g;
		sensor_data.bh1749.blue = b;
		sensor_data.bh1749.ir = ir;
		k_mutex_unlock(&sensor_data_mutex);
	}
	return 0;
}

void bh1749_sensor_thread(void)
{
	printk("BH1749 sensor thread started\n");
	
	/* Wait for BLE to be ready before starting sensor */
	printk("Waiting for BLE init...\n");
	int ret = k_sem_take(&ble_init_ok, K_SECONDS(30));
	if (ret != 0) {
		printk("Timeout waiting for BLE init: %d\n", ret);
		return;
	}
	printk("BLE ready, starting BH1749 sensor...\n");

	/* Register BLE send callback */
	bh1749_sensor_set_ble_callback(bh1749_ble_send_callback);
	printk("BH1749 BLE callback registered\n");

	/* Start BH1749 sensor operation */
	int err = bh1749_sensor_start();
	if (err) {
		LOG_ERR("BH1749 sensor start failed (err: %d)", err);
		printk("BH1749 sensor start failed: %d\n", err);
	} else {
		printk("BH1749 sensor started successfully\n");
	}
	printk("BH1749 sensor thread exiting\n");
}

K_THREAD_DEFINE(bh1749_sensor_thread_id, STACKSIZE, bh1749_sensor_thread, NULL, NULL,
		NULL, PRIORITY - 1, 0, 0);

/* BLE send callback for BMI270 sensor */
static int bmi270_ble_send_callback(const uint8_t *data, uint16_t len)
{
	/* Parse and store data instead of sending */
	const char *str = (const char *)data;
	int ax1, ax2, ay1, ay2, az1, az2, gx1, gx2, gy1, gy2, gz1, gz2;
	
	if (sscanf(str, "AX:%d.%02d AY:%d.%02d AZ:%d.%02d GX:%d.%02d GY:%d.%02d GZ:%d.%02d",
		&ax1, &ax2, &ay1, &ay2, &az1, &az2, &gx1, &gx2, &gy1, &gy2, &gz1, &gz2) == 12) {
		k_mutex_lock(&sensor_data_mutex, K_FOREVER);
		sensor_data.bmi270.valid = true;
		sensor_data.bmi270.accel_x = ax1 * 100 + ax2;
		sensor_data.bmi270.accel_y = ay1 * 100 + ay2;
		sensor_data.bmi270.accel_z = az1 * 100 + az2;
		sensor_data.bmi270.gyro_x = gx1 * 100 + gx2;
		sensor_data.bmi270.gyro_y = gy1 * 100 + gy2;
		sensor_data.bmi270.gyro_z = gz1 * 100 + gz2;
		k_mutex_unlock(&sensor_data_mutex);
	}
	return 0;
}

void bmi270_sensor_thread(void)
{
	printk("BMI270 sensor thread started\n");
	
	/* Wait for BLE to be ready before starting sensor */
	printk("Waiting for BLE init...\n");
	int ret = k_sem_take(&ble_init_ok, K_SECONDS(30));
	if (ret != 0) {
		printk("Timeout waiting for BLE init: %d\n", ret);
		return;
	}
	printk("BLE ready, starting BMI270 sensor...\n");

	/* Register BLE send callback */
	bmi270_sensor_set_ble_callback(bmi270_ble_send_callback);
	printk("BMI270 BLE callback registered\n");

	/* Start BMI270 sensor operation */
	int err = bmi270_sensor_start();
	if (err) {
		LOG_ERR("BMI270 sensor start failed (err: %d)", err);
		printk("BMI270 sensor start failed: %d\n", err);
	} else {
		printk("BMI270 sensor started successfully\n");
	}
	printk("BMI270 sensor thread exiting\n");
}

K_THREAD_DEFINE(bmi270_sensor_thread_id, STACKSIZE, bmi270_sensor_thread, NULL, NULL,
		NULL, PRIORITY - 1, 0, 0);

/* BLE send callback for ADXL362 sensor */
static int adxl362_ble_send_callback(const uint8_t *data, uint16_t len)
{
	/* Parse and store data instead of sending */
	const char *str = (const char *)data;
	int x1, x2, y1, y2, z1, z2;
	
	if (sscanf(str, "X:%d.%02d Y:%d.%02d Z:%d.%02d", &x1, &x2, &y1, &y2, &z1, &z2) == 6) {
		k_mutex_lock(&sensor_data_mutex, K_FOREVER);
		sensor_data.adxl362.valid = true;
		sensor_data.adxl362.accel_x = x1 * 100 + x2;
		sensor_data.adxl362.accel_y = y1 * 100 + y2;
		sensor_data.adxl362.accel_z = z1 * 100 + z2;
		k_mutex_unlock(&sensor_data_mutex);
	}
	return 0;
}

void adxl362_sensor_thread(void)
{
	printk("ADXL362 sensor thread started\n");
	
	/* Wait for BLE to be ready before starting sensor */
	printk("Waiting for BLE init...\n");
	int ret = k_sem_take(&ble_init_ok, K_SECONDS(30));
	if (ret != 0) {
		printk("Timeout waiting for BLE init: %d\n", ret);
		return;
	}
	printk("BLE ready, starting ADXL362 sensor...\n");

	/* Register BLE send callback */
	adxl362_sensor_set_ble_callback(adxl362_ble_send_callback);
	printk("ADXL362 BLE callback registered\n");

	/* Start ADXL362 sensor operation */
	int err = adxl362_sensor_start();
	if (err) {
		LOG_ERR("ADXL362 sensor start failed (err: %d)", err);
		printk("ADXL362 sensor start failed: %d\n", err);
	} else {
		printk("ADXL362 sensor started successfully\n");
	}
	printk("ADXL362 sensor thread exiting\n");
}

K_THREAD_DEFINE(adxl362_sensor_thread_id, STACKSIZE, adxl362_sensor_thread, NULL, NULL,
		NULL, PRIORITY - 1, 0, 0);

/* Unified BLE sender thread - sends all sensor data in one message */
void unified_ble_sender_thread(void)
{
	printk("Unified BLE sender thread started\n");
	
	/* Wait for BLE to be ready */
	k_sem_take(&ble_init_ok, K_FOREVER);
	printk("BLE ready, starting unified sender...\n");

	while (1) {
		k_sleep(K_SECONDS(3)); /* Send every 3 seconds */

		if (!current_conn) {
			/* No connection, skip */
			continue;
		}

		/* Build combined message */
		char msg[512];
		int len = 0;

		k_mutex_lock(&sensor_data_mutex, K_FOREVER);

		/* BME680 */
		if (sensor_data.bme.valid) {
			len += snprintf(msg + len, sizeof(msg) - len,
				"BME T:%d.%02dC P:%d.%02dhPa H:%d.%02d%% Gas:%luOhm | ",
				sensor_data.bme.temp / 100, sensor_data.bme.temp % 100,
				sensor_data.bme.pressure / 100, sensor_data.bme.pressure % 100,
				sensor_data.bme.humidity / 100, sensor_data.bme.humidity % 100,
				(unsigned long)sensor_data.bme.gas);
		}

		/* BH1749 */
		if (sensor_data.bh1749.valid) {
			len += snprintf(msg + len, sizeof(msg) - len,
				"BH R:%d G:%d B:%d IR:%d | ",
				sensor_data.bh1749.red,
				sensor_data.bh1749.green,
				sensor_data.bh1749.blue,
				sensor_data.bh1749.ir);
		}

		/* BMI270 */
		if (sensor_data.bmi270.valid) {
			len += snprintf(msg + len, sizeof(msg) - len,
				"BMI AX:%d.%02d AY:%d.%02d AZ:%d.%02d GX:%d.%02d GY:%d.%02d GZ:%d.%02d | ",
				sensor_data.bmi270.accel_x / 100, abs(sensor_data.bmi270.accel_x % 100),
				sensor_data.bmi270.accel_y / 100, abs(sensor_data.bmi270.accel_y % 100),
				sensor_data.bmi270.accel_z / 100, abs(sensor_data.bmi270.accel_z % 100),
				sensor_data.bmi270.gyro_x / 100, abs(sensor_data.bmi270.gyro_x % 100),
				sensor_data.bmi270.gyro_y / 100, abs(sensor_data.bmi270.gyro_y % 100),
				sensor_data.bmi270.gyro_z / 100, abs(sensor_data.bmi270.gyro_z % 100));
		}

		/* ADXL362 */
		if (sensor_data.adxl362.valid) {
			len += snprintf(msg + len, sizeof(msg) - len,
				"ADXL X:%d.%02d Y:%d.%02d Z:%d.%02d",
				sensor_data.adxl362.accel_x / 100, abs(sensor_data.adxl362.accel_x % 100),
				sensor_data.adxl362.accel_y / 100, abs(sensor_data.adxl362.accel_y % 100),
				sensor_data.adxl362.accel_z / 100, abs(sensor_data.adxl362.accel_z % 100));
		}

		k_mutex_unlock(&sensor_data_mutex);

		if (len > 0) {
			msg[len++] = '\n';
			int err = bt_nus_send(current_conn, (uint8_t *)msg, len);
			if (err) {
				printk("Failed to send combined data: %d\n", err);
			} else {
				printk("Sent combined data (%d bytes)\n", len);
			}
		}
	}
}

K_THREAD_DEFINE(unified_sender_thread_id, STACKSIZE, unified_ble_sender_thread, NULL, NULL,
		NULL, PRIORITY - 1, 0, 0);
