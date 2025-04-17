#include "uart_transport.h"

// UART transport state
typedef struct {
    UARTTransport_Config_t* config;
    XmodemManager_t xmodem;
    uint8_t buffer[UART_BUFFER_SIZE];
    size_t buffer_index;
    size_t buffer_size;
    uint8_t receive_mode;
} UARTTransport_State_t;

static UARTTransport_State_t uart_state;

// Initialize UART transport
int uart_transport_init(void* config) {
    UARTTransport_Config_t* uart_config = (UARTTransport_Config_t*)config;
    uart_state.config = uart_config;
    
    // Initialize UART
    if (HAL_UART_Init(uart_config->huart) != HAL_OK) {
        return -1;
    }
    
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
    
    // Clear buffers
    memset(uart_state.buffer, 0, UART_BUFFER_SIZE);
    uart_state.buffer_index = 0;
    uart_state.buffer_size = 0;
    uart_state.receive_mode = 0;
    
    return 0;
}

// Send data via UART
int uart_transport_send(const uint8_t* data, size_t len) {
    if (HAL_UART_Transmit(uart_state.config->huart, (uint8_t*)data, len, uart_state.config->timeout) != HAL_OK) {
        return -1;
    }
    
    return len;
}

// Receive data via UART
int uart_transport_receive(uint8_t* data, size_t len) {
    if (uart_state.buffer_size > 0) {
        // We have data in buffer
        size_t copy_size = (len < uart_state.buffer_size) ? len : uart_state.buffer_size;
        memcpy(data, uart_state.buffer, copy_size);
        
        // Move remaining data to beginning of buffer
        memmove(uart_state.buffer, 
                uart_state.buffer + copy_size, 
                uart_state.buffer_size - copy_size);
                
        uart_state.buffer_size -= copy_size;
        return copy_size;
    }
    
    // No buffered data, read directly from UART
    if (HAL_UART_Receive(uart_state.config->huart, data, len, uart_state.config->timeout) != HAL_OK) {
        return -1;
    }
    
    return len;
}

// Process UART events
int uart_transport_process(void) {
    // Process XMODEM if in receive mode
    if (uart_state.receive_mode && uart_state.config->use_xmodem) {
        uint8_t byte;
        
        // Check if there's data to receive
        if (HAL_UART_Receive(uart_state.config->huart, &byte, 1, 0) == HAL_OK) {
            XmodemError_t result = xmodem_process_byte(&uart_state.xmodem, byte);
            
            // Handle XMODEM response
            if (xmodem_should_send_byte(&uart_state.xmodem)) {
                uint8_t response = xmodem_get_response(&uart_state.xmodem);
                HAL_UART_Transmit(uart_state.config->huart, &response, 1, uart_state.config->timeout);
            }
            
            // Check if transfer is complete or errored
            XmodemState_t state = xmodem_get_state(&uart_state.xmodem);
            if (state == XMODEM_STATE_COMPLETE || state == XMODEM_STATE_ERROR) {
                uart_state.receive_mode = 0;
                return (state == XMODEM_STATE_COMPLETE) ? 1 : -1;
            }
        }
    }
    
    return 0;
}

// Deinitialize UART
int uart_transport_deinit(void) {
    return (HAL_UART_DeInit(uart_state.config->huart) == HAL_OK) ? 0 : -1;
}

// Start XMODEM receive
int uart_transport_xmodem_receive(uint32_t target_addr) {
    if (!uart_state.config->use_xmodem) {
        return -1;
    }
    
    xmodem_start(&uart_state.xmodem, target_addr);
    uart_state.receive_mode = 1;
    
    return 0;
}

// Get XMODEM state
XmodemState_t uart_transport_xmodem_state(void) {
    return xmodem_get_state(&uart_state.xmodem);
}