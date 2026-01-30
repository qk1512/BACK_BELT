#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <zephyr/kernel.h>

/* Initialize UART subsystem */
int uart_init(void);

/* UART RX data FIFO - exposed for bluetooth_handler */
extern struct k_fifo fifo_uart_rx_data;
extern struct k_fifo fifo_uart_tx_data;

#endif // UART_HANDLER_H