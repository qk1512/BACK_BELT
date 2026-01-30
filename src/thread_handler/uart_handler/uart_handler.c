#include <uart_async_adapter.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include "uart_handler.h"

LOG_MODULE_REGISTER(uart_handler);

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t
{
    void *fifo_reserved;
    uint8_t data[UART_BUF_SIZE];
    uint16_t len;
};

K_FIFO_DEFINE(fifo_uart_tx_data);
K_FIFO_DEFINE(fifo_uart_rx_data);

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

    switch (evt->type)
    {
    case UART_TX_DONE:
        LOG_DBG("UART_TX_DONE");
        if ((evt->data.tx.len == 0) || (!evt->data.tx.buf))
        {
            return;
        }

        if (aborted_buf)
        {
            buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]);
            aborted_buf = NULL;
            aborted_len = 0;
        }
        else
        {
            buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data[0]);
        }

        k_free(buf);

        buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
        if (!buf)
        {
            return;
        }

        if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS))
        {
            LOG_WRN("Failed to send data over UART");
        }
        break;

    case UART_RX_RDY:
        printf("⚡ UART_RX_RDY: Received %d bytes\r\n", evt->data.rx.len);
        buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
        buf->len += evt->data.rx.len;

        printf("Buffer total length now: %d bytes\r\n", buf->len);

        if (disable_req)
        {
            return;
        }

        if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
            (evt->data.rx.buf[buf->len - 1] == '\r'))
        {
            printf("Newline detected, disabling UART RX\r\n");
            disable_req = true;
            uart_rx_disable(uart);
        }
        break;

    case UART_RX_DISABLED:
        printf("UART_RX_DISABLED - Re-enabling reception\r\n");
        disable_req = false;

        buf = k_malloc(sizeof(*buf));
        if (buf)
        {
            buf->len = 0;
        }
        else
        {
            LOG_WRN("Not able to allocate UART receive buffer");
            k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
            return;
        }

        uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
        break;

    case UART_RX_BUF_REQUEST:
        LOG_DBG("UART_RX_BUF_REQUEST");
        buf = k_malloc(sizeof(*buf));
        if (buf)
        {
            buf->len = 0;
            uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
        }
        else
        {
            LOG_WRN("Not able to allocate UART receive buffer");
        }
        break;

    case UART_RX_BUF_RELEASED:
        printf("UART_RX_BUF_RELEASED\r\n");
        buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]);

        if (buf->len > 0)
        {
            printf("Putting %d bytes into FIFO\r\n", buf->len);
            k_fifo_put(&fifo_uart_rx_data, buf);
        }
        else
        {
            printf("Buffer empty, freeing it\r\n");
            k_free(buf);
        }
        break;

    case UART_TX_ABORTED:
        LOG_DBG("UART_TX_ABORTED");
        if (!aborted_buf)
        {
            aborted_buf = (uint8_t *)evt->data.tx.buf;
        }

        aborted_len += evt->data.tx.len;
        buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t, data);

        uart_tx(uart, &buf->data[aborted_len], buf->len - aborted_len, SYS_FOREVER_MS);
        break;

    default:
        break;
    }
}

static void uart_work_handler(struct k_work *item)
{
    struct uart_data_t *buf;

    buf = k_malloc(sizeof(*buf));
    if (buf)
    {
        buf->len = 0;
    }
    else
    {
        LOG_WRN("Not able to allocate UART receive buffer");
        k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
        return;
    }

    uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
    const struct uart_driver_api *api = (const struct uart_driver_api *)dev->api;
    return (api->callback_set != NULL);
}

int uart_init(void)
{
    int err;
    int pos;
    struct uart_data_t *rx;
    struct uart_data_t *tx;

    if (!device_is_ready(uart))
    {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK))
    {
        err = usb_enable(NULL);
        if (err && (err != -EALREADY))
        {
            LOG_ERR("Failed to enable USB");
            return err;
        }
    }

    rx = k_malloc(sizeof(*rx));
    if (rx)
    {
        rx->len = 0;
    }
    else
    {
        return -ENOMEM;
    }

    k_work_init_delayable(&uart_work, uart_work_handler);

    if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart))
    {
        uart_async_adapter_init(async_adapter, uart);
        uart = async_adapter;
    }

    err = uart_callback_set(uart, uart_cb, NULL);
    if (err)
    {
        k_free(rx);
        LOG_ERR("Cannot initialize UART callback");
        return err;
    }

    if (IS_ENABLED(CONFIG_UART_LINE_CTRL))
    {
        LOG_INF("Wait for DTR");
        while (true)
        {
            uint32_t dtr = 0;
            uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
            if (dtr)
            {
                break;
            }
            k_sleep(K_MSEC(100));
        }
        LOG_INF("DTR set");
        err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
        if (err)
        {
            LOG_WRN("Failed to set DCD, ret code %d", err);
        }
        err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
        if (err)
        {
            LOG_WRN("Failed to set DSR, ret code %d", err);
        }
    }

    tx = k_malloc(sizeof(*tx));
    if (tx)
    {
        pos = snprintf(tx->data, sizeof(tx->data),
                       "Starting Nordic UART service sample\r\n");

        if ((pos < 0) || (pos >= sizeof(tx->data)))
        {
            k_free(rx);
            k_free(tx);
            LOG_ERR("snprintf returned %d", pos);
            return -ENOMEM;
        }

        tx->len = pos;
    }
    else
    {
        k_free(rx);
        return -ENOMEM;
    }

    err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
    if (err)
    {
        k_free(rx);
        k_free(tx);
        LOG_ERR("Cannot display welcome message (err: %d)", err);
        return err;
    }

    err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
    if (err)
    {
        LOG_ERR("Cannot enable uart reception (err: %d)", err);
        k_free(rx);
    }

    LOG_INF("UART initialized successfully");
    return err;
}