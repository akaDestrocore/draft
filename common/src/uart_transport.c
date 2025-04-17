#include "uart_transport.h"

// UART transport state
typedef struct {
    UARTTransport_Config_t* config;
    XmodemManager_t xmodem;
    RingBuffer_t tx_buffer;
    RingBuffer_t rx_buffer;
    uint8_t receive_mode;
} UARTTransport_State_t;

static UARTTransport_State_t uart_state;

// Initialize UART transport
int uart_transport_init(void* config) {
    UARTTransport_Config_t* uart_config = (UARTTransport_Config_t*)config;
    uart_state.config = uart_config;
    
    // Initialize ring buffers
    ring_buffer_init(&uart_state.tx_buffer);
    ring_buffer_init(&uart_state.rx_buffer);
    
    // Initialize UART
    if (HAL_UART_Init(uart_config->huart) != HAL_OK) {
        return -1;
    }
    
    // Enable UART receive interrupt
    __HAL_UART_ENABLE_IT(uart_config->huart, UART_IT_RXNE);
    
    // Initialize XMODEM if needed
    if (uart_config->use_xmodem) {
        XmodemConfig_t xmodem_config = {
            .app_addr = uart_config->app_addr,
            .updater_addr = uart_config->updater_addr,
            .loader_addr = uart_config->loader_addr,
            .image_hdr_size = uart_config->image_hdr_size
        };
        
        xmodem_init(&uart_state.xmodem, &xmodem_config);
    }
    
    uart_state.receive_mode = 0;
    
    return 0;
}

// Send data via UART
int uart_transport_send(const uint8_t* data, size_t len) {
    if (data == NULL || len == 0) {
        return 0;
    }
    
    size_t sent = 0;
    for (size_t i = 0; i < len; i++) {
        if (ring_buffer_write(&uart_state.tx_buffer, data[i])) {
            sent++;
        } else {
            break; // Buffer full
        }
    }
    
    // Start transmission if not already in progress
    if (!ring_buffer_is_empty(&uart_state.tx_buffer)) {
        __HAL_UART_ENABLE_IT(uart_state.config->huart, UART_IT_TXE);
    }
    
    return sent;
}

// Receive data via UART
int uart_transport_receive(uint8_t* data, size_t len) {
    if (data == NULL || len == 0) {
        return 0;
    }
    
    size_t received = 0;
    for (size_t i = 0; i < len; i++) {
        if (ring_buffer_read(&uart_state.rx_buffer, &data[i])) {
            received++;
        } else {
            break; // No more data
        }
    }
    
    return received;
}

// Check if TX is complete
int uart_transport_is_tx_complete(void) {
    return ring_buffer_is_empty(&uart_state.tx_buffer);
}

// Process UART events
int uart_transport_process(void) {
    // Process XMODEM if in receive mode
    if (uart_state.receive_mode && uart_state.config->use_xmodem) {
        uint8_t byte;
        
        // Process any bytes in the RX buffer
        while (ring_buffer_read(&uart_state.rx_buffer, &byte)) {
            XmodemError_t result = xmodem_process_byte(&uart_state.xmodem, byte);
            
            // Handle XMODEM response
            if (xmodem_should_send_byte(&uart_state.xmodem)) {
                uint8_t response = xmodem_get_response(&uart_state.xmodem);
                uart_transport_send(&response, 1);
            }
            
            // Check transfer status
            if (result == XMODEM_ERROR_TRANSFER_COMPLETE) {
                uart_state.receive_mode = 0;
                return 1; // Success
            } else if (result != XMODEM_ERROR_NONE) {
                // Any error other than NONE indicates transfer issues
                if (result != XMODEM_ERROR_CRC_ERROR && 
                    result != XMODEM_ERROR_SEQUENCE_ERROR) {
                    uart_state.receive_mode = 0;
                    return -1; // Error
                }
            }
            
            // Check state
            XmodemState_t state = xmodem_get_state(&uart_state.xmodem);
            if (state == XMODEM_STATE_COMPLETE || state == XMODEM_STATE_ERROR) {
                uart_state.receive_mode = 0;
                return (state == XMODEM_STATE_COMPLETE) ? 1 : -1;
            }
        }
    }
    
    return 0;
}

// Clear RX buffer
void uart_transport_clear_rx(void) {
    ring_buffer_clear(&uart_state.rx_buffer);
}

// Deinitialize UART
int uart_transport_deinit(void) {
    // Disable UART interrupts
    __HAL_UART_DISABLE_IT(uart_state.config->huart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(uart_state.config->huart, UART_IT_TXE);
    
    return (HAL_UART_DeInit(uart_state.config->huart) == HAL_OK) ? 0 : -1;
}

// Start XMODEM receive
int uart_transport_xmodem_receive(uint32_t target_addr) {
    if (!uart_state.config->use_xmodem) {
        return -1;
    }
    
    // Clear RX buffer
    ring_buffer_clear(&uart_state.rx_buffer);
    
    xmodem_start(&uart_state.xmodem, target_addr);
    uart_state.receive_mode = 1;
    
    return 0;
}

// Get XMODEM state
XmodemState_t uart_transport_xmodem_state(void) {
    return xmodem_get_state(&uart_state.xmodem);
}

// UART IRQ Handler - to be called from USART2_IRQHandler
void uart_transport_irq_handler(void) {
    UART_HandleTypeDef* huart = uart_state.config->huart;
    
    // Check for RXNE (Receive buffer not empty)
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) && 
       __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE)) {
        // Read byte from UART and store in RX buffer
        uint8_t byte = (uint8_t)(huart->Instance->DR & 0xFF);
        ring_buffer_write(&uart_state.rx_buffer, byte);
    }
    
    // Check for TXE (Transmit buffer empty)
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) && 
       __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE)) {
        uint8_t byte;
        if(ring_buffer_read(&uart_state.tx_buffer, &byte)) {
            // Send byte
            huart->Instance->DR = byte;
        } else {
            // No more data, disable TXE interrupt
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
        }
    }
}