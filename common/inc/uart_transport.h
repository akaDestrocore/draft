#ifndef _UART_TRANSPORT_H
#define _UART_TRANSPORT_H

#include "transport.h"
#include "xmodem.h"
#include "ring_buffer.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// UART transport configuration
typedef struct {
    UART_HandleTypeDef* huart;
    uint32_t baudrate;
    uint32_t timeout;
    uint8_t use_xmodem;
    uint32_t app_addr;
    uint32_t updater_addr;
    uint32_t loader_addr;
    uint32_t image_hdr_size;
} UARTTransport_Config_t;

void USART2_IRQHandler(void);

// Initialize UART transport
int uart_transport_init(void* config);

// Send data via UART
int uart_transport_send(const uint8_t* data, size_t len);

// Send a single byte - useful for XMODEM
void uart_transport_send_byte(uint8_t byte);

// Check if transmission complete
int uart_transport_is_tx_complete(void);

// Receive data via UART
int uart_transport_receive(uint8_t* data, size_t len);

// Process UART events
int uart_transport_process(void);

// Clear RX buffer
void uart_transport_clear_rx(void);

// Deinitialize UART
int uart_transport_deinit(void);

// Start XMODEM receive
int uart_transport_xmodem_receive(uint32_t target_addr);

// Get XMODEM state
XmodemState_t uart_transport_xmodem_state(void);

// Access to buffers for IRQ handler
Ring_Buffer_t* get_uart_rx_buffer(void);
Ring_Buffer_t* get_uart_tx_buffer(void);

#endif /* _UART_TRANSPORT_H */