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

void USART2_IRQHandler(void) {
    uart_transport_irq_handler();
}

// Initialize UART transport
int uart_transport_init(void* config) {
    UARTTransport_Config_t* uart_config = (UARTTransport_Config_t*)config;
    uart_state.config = uart_config;
    
    // Initialize ring buffers
    ring_buffer_init(&uart_state.tx_buffer);
    ring_buffer_init(&uart_state.rx_buffer);
    
    // Initialize USART using LL functions
    LL_USART_InitTypeDef USART_InitStruct = {0};
    
    // Configure parameters
    USART_InitStruct.BaudRate = uart_config->baudrate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    
    // Initialize USART
    if (LL_USART_Init(uart_config->usart, &USART_InitStruct) != SUCCESS) {
        return -1;
    }
    
    // Enable USART
    LL_USART_Enable(uart_config->usart);
    
    // Enable UART receive interrupt
    LL_USART_EnableIT_RXNE(uart_config->usart);
    
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
        LL_USART_EnableIT_TXE(uart_state.config->usart);
    }
    
    return sent;
}

// Send a single byte via UART
void uart_transport_send_byte(uint8_t byte) {
    // Wait until transmit buffer is empty
    while (!LL_USART_IsActiveFlag_TXE(uart_state.config->usart)) {}
    
    // Write data to transmit register
    LL_USART_TransmitData8(uart_state.config->usart, byte);
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
    return ring_buffer_is_empty(&uart_state.tx_buffer) && 
           LL_USART_IsActiveFlag_TC(uart_state.config->usart);
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
    LL_USART_DisableIT_RXNE(uart_state.config->usart);
    LL_USART_DisableIT_TXE(uart_state.config->usart);
    
    // Disable UART
    LL_USART_Disable(uart_state.config->usart);
    
    return 0;
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
    USART_TypeDef* usart = uart_state.config->usart;
    
    // Check for RXNE (Receive buffer not empty)
    if(LL_USART_IsActiveFlag_RXNE(usart) && 
       LL_USART_IsEnabledIT_RXNE(usart)) {
        // Read byte from USART and store in RX buffer
        uint8_t byte = LL_USART_ReceiveData8(usart);
        ring_buffer_write(&uart_state.rx_buffer, byte);
    }
    
    // Check for TXE (Transmit buffer empty)
    if(LL_USART_IsActiveFlag_TXE(usart) && 
       LL_USART_IsEnabledIT_TXE(usart)) {
        uint8_t byte;
        if(ring_buffer_read(&uart_state.tx_buffer, &byte)) {
            // Send byte
            LL_USART_TransmitData8(usart, byte);
        } else {
            // No more data, disable TXE interrupt
            LL_USART_DisableIT_TXE(usart);
        }
    }
    
    // Check for error flags
    if(LL_USART_IsActiveFlag_ORE(usart) || 
       LL_USART_IsActiveFlag_NE(usart) || 
       LL_USART_IsActiveFlag_FE(usart) || 
       LL_USART_IsActiveFlag_PE(usart)) {
        // Clear error flags
        LL_USART_ClearFlag_ORE(usart);
        LL_USART_ClearFlag_NE(usart);
        LL_USART_ClearFlag_FE(usart);
        LL_USART_ClearFlag_PE(usart);
    }
}

// Get ring buffers for direct access if needed
RingBuffer_t* get_uart_rx_buffer(void) {
    return &uart_state.rx_buffer;
}

RingBuffer_t* get_uart_tx_buffer(void) {
    return &uart_state.tx_buffer;
}