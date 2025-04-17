#ifndef _I2C_TRANSPORT_H
#define _I2C_TRANSPORT_H

#include "transport.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define I2C_BUFFER_SIZE 256

// I2C transport configuration
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint16_t device_address;
    uint32_t timeout;
} I2CTransport_Config_t;

// Initialize I2C transport
int i2c_transport_init(void* config);

// Send data via I2C
int i2c_transport_send(const uint8_t* data, size_t len);

// Receive data via I2C
int i2c_transport_receive(uint8_t* data, size_t len);

// Process I2C events
int i2c_transport_process(void);

// Deinitialize I2C
int i2c_transport_deinit(void);

#endif /* _I2C_TRANSPORT_H */