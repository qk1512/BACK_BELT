#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>
#include <stdio.h>
#include <string.h>
#include "bluetooth_handler.h"
#include "../sensor_handler/sensor_handler.h"
#include "../buzzer_handler/buzzer_handler.h"
#include "../../command_handler/command_handler.h"
#include "../../device_config/device_config.h"

#define DEVICE_NAME_MAX_LEN 32
LOG_MODULE_REGISTER(bluetooth_handler);

#define CON_STATUS_LED DK_LED2

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7
#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

static struct bt_conn *current_conn = NULL;
static struct bt_conn *auth_conn = NULL;

/* Dynamic advertising data structure */
static struct
{
    uint8_t flags;
    uint8_t name_len;
    uint8_t name_type;
    char name[DEVICE_NAME_MAX_LEN];
} ad_data;

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, ad_data.name, 0), /* Length will be set dynamically */
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

struct uart_data_t
{
    void *fifo_reserved;
    uint8_t data[UART_BUF_SIZE];
    uint16_t len;
};

extern struct k_fifo fifo_uart_rx_data;
extern struct k_sem ble_init_ok;

struct bt_conn *bluetooth_handler_get_current_conn(void)
{
    return current_conn;
}

static void advertising_start(void)
{
    int err;
    char formatted_name[DEVICE_NAME_MAX_LEN];

    /* Format device name with BLE MAC address */
    err = device_config_format_with_mac(formatted_name, sizeof(formatted_name));
    if (err)
    {
        LOG_WRN("Failed to format device name with MAC: %d, using base name", err);
        strncpy(formatted_name, device_config_get_name(), sizeof(formatted_name) - 1);
        formatted_name[sizeof(formatted_name) - 1] = '\0';
    }

    /* Update advertising data with formatted device name */
    strncpy(ad_data.name, formatted_name, DEVICE_NAME_MAX_LEN - 1);
    ad_data.name[DEVICE_NAME_MAX_LEN - 1] = '\0';

    /* Update the advertising data structure with new name length */
    ad[1].data_len = strlen(ad_data.name);
    ad[1].data = (uint8_t *)ad_data.name;

    /* Also set the BT device name */
    err = bt_set_name(formatted_name);
    if (err)
    {
        LOG_WRN("Failed to set device name: %d", err);
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    // LOG_INF("Advertising successfully started with device name: %s", device_name);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err)
    {
        LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);

    current_conn = bt_conn_ref(conn);
    sensor_handler_set_current_conn(current_conn);

    dk_set_led_on(CON_STATUS_LED);

    /* Play BLE connected notification */
    buzzer_play_notification(BUZZER_NOTIFY_BLE_CONNECTED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

    if (auth_conn)
    {
        bt_conn_unref(auth_conn);
        auth_conn = NULL;
    }

    if (current_conn)
    {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        sensor_handler_set_current_conn(NULL);
        dk_set_led_off(CON_STATUS_LED);
    }

    /* Play BLE disconnected notification */
    buzzer_play_notification(BUZZER_NOTIFY_BLE_DISCONNECTED);
}

static void recycled_cb(void)
{
    LOG_INF("Connection object available from previous conn. Disconnect is complete!");
    advertising_start();
}

/* Update device name at runtime */
void bluetooth_handler_update_device_name(void)
{
    const char *device_name = device_config_get_name();
    int err;

    /* Update advertising data with new device name */
    strncpy(ad_data.name, device_name, DEVICE_NAME_MAX_LEN - 1);
    ad_data.name[DEVICE_NAME_MAX_LEN - 1] = '\0';
    ad[1].data_len = strlen(ad_data.name);
    ad[1].data = (uint8_t *)ad_data.name;

    /* Update BT device name */
    err = bt_set_name(device_name);
    if (err)
    {
        LOG_WRN("Failed to update device name: %d", err);
    }
    else
    {
        LOG_INF("Device name updated to: %s", device_name);
    }

    /* Restart advertising with new device name */
    if (current_conn == NULL)
    {
        bt_le_adv_stop();
        k_msleep(100);
        advertising_start();
    }
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err)
    {
        LOG_INF("Security changed: %s level %u", addr, level);
    }
    else
    {
        LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
                bt_security_err_to_str(err));
    }
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled_cb,
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

    if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX))
    {
        LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
    }
    else
    {
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
    .pairing_failed = pairing_failed};

static void num_comp_reply(bool accept)
{
    if (accept)
    {
        bt_conn_auth_passkey_confirm(auth_conn);
        LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
    }
    else
    {
        bt_conn_auth_cancel(auth_conn);
        LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
    }

    bt_conn_unref(auth_conn);
    auth_conn = NULL;
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printf("RX from %s, len=%d\r\n", addr, len);

    printf("HEX: ");
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", data[i]);
    }
    printf("\r\n");

    char buf[241];
    int n = MIN(len, sizeof(buf) - 1);
    memcpy(buf, data, n);
    buf[n] = '\0';
    printf("TXT: %s\r\n", buf);

    /* Process command */
    command_handler_process(conn, data, len);
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

int bluetooth_handler_init(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED))
    {
        err = bt_conn_auth_cb_register(&conn_auth_callbacks);
        if (err)
        {
            LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
            return err;
        }

        err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
        if (err)
        {
            LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
            return err;
        }
    }

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        settings_load();
    }

    err = bt_nus_init(&nus_cb);
    if (err)
    {
        LOG_ERR("Failed to initialize UART service (err: %d)", err);
        return err;
    }

    advertising_start();

    return 0;
}

void ble_write_thread(void)
{
    k_sem_take(&ble_init_ok, K_FOREVER);
    struct uart_data_t nus_data = {
        .len = 0,
    };

    for (;;)
    {
        struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);

        printf("Terminal input received: %d bytes\r\n", buf->len);

        int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
        int loc = 0;

        while (plen > 0)
        {
            memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
            nus_data.len += plen;
            loc += plen;

            if (nus_data.len >= sizeof(nus_data.data) ||
                (nus_data.data[nus_data.len - 1] == '\n') ||
                (nus_data.data[nus_data.len - 1] == '\r'))
            {

                printf("Sending to BLE (%d bytes): ", nus_data.len);
                for (int i = 0; i < nus_data.len; i++)
                {
                    printf("%02X ", nus_data.data[i]);
                }
                printf("\r\n");

                if (bt_nus_send(NULL, nus_data.data, nus_data.len))
                {
                    LOG_WRN("Failed to send data over BLE connection");
                }
                else
                {
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