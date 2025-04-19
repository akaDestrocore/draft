#ifndef _TRANSPORT_H
#define _TRANSPORT_H

#include "uart_transport.h"
// #include "i2c_transport.h"
// #include "spi_transport.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>

// Transport types
typedef enum {
    TRANSPORT_UART,
    TRANSPORT_I2C,
    TRANSPORT_SPI
} TransportType_t;

// Transport handle
typedef struct {
    TransportType_t type;
    void* config;
    uint8_t initialized;
    int (*init)(void* config);
    int (*send)(const uint8_t* data, size_t len);
    int (*receive)(uint8_t* data, size_t len);
    int (*process)(void);
    int (*deinit)(void);
} Transport_t;

// Initialize transport
int transport_init(Transport_t* transport, TransportType_t type, void* config);

// Send data
int transport_send(Transport_t* transport, const uint8_t* data, size_t len);

// Receive data
int transport_receive(Transport_t* transport, uint8_t* data, size_t len);

// Process transport events
int transport_process(Transport_t* transport);

// Deinitialize transport
int transport_deinit(Transport_t* transport);

#endif /* _TRANSPORT_H */