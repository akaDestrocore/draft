#include "transport.h"

// Initialize transport
int transport_init(Transport_t* transport, TransportType_t type, void* config) {
    transport->type = type;
    transport->config = config;
    
    // Set function pointers based on type
    switch (type) {
        case TRANSPORT_UART:
            transport->init = uart_transport_init;
            transport->send = uart_transport_send;
            transport->receive = uart_transport_receive;
            transport->process = uart_transport_process;
            transport->deinit = uart_transport_deinit;
            break;
        /*
        case TRANSPORT_I2C:
            transport->init = i2c_transport_init;
            transport->send = i2c_transport_send;
            transport->receive = i2c_transport_receive;
            transport->process = i2c_transport_process;
            transport->deinit = i2c_transport_deinit;
            break;
            
        case TRANSPORT_SPI:
            transport->init = spi_transport_init;
            transport->send = spi_transport_send;
            transport->receive = spi_transport_receive;
            transport->process = spi_transport_process;
            transport->deinit = spi_transport_deinit;
            break;
        */
        default:
            return -1; // Invalid transport type
    }
    
    // Initialize the selected transport
    if (transport->init(config) != 0) {
        return -1;
    }
    
    transport->initialized = 1;
    return 0;
}

// Send data
int transport_send(Transport_t* transport, const uint8_t* data, size_t len) {
    if (!transport->initialized) {
        return -1;
    }
    
    return transport->send(data, len);
}

// Receive data
int transport_receive(Transport_t* transport, uint8_t* data, size_t len) {
    if (!transport->initialized) {
        return -1;
    }
    
    return transport->receive(data, len);
}

// Process transport events
int transport_process(Transport_t* transport) {
    if (!transport->initialized) {
        return -1;
    }
    
    return transport->process();
}

// Deinitialize transport
int transport_deinit(Transport_t* transport) {
    if (!transport->initialized) {
        return -1;
    }
    
    int result = transport->deinit();
    if (result == 0) {
        transport->initialized = 0;
    }
    
    return result;
}