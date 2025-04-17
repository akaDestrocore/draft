#ifndef _SPI_TRANSPORT_H
#define _SPI_TRANSPORT_H

#include "transport.h"
#include "stm32f4xx_hal.h"

#define SPI_BUFFER_SIZE 256

// SPI transport configuration
typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    uint32_t timeout;
} SPITransport_Config_t;

// Initialize SPI transport
int spi_transport_init(void* config);

// Send data via SPI
int spi_transport_send(const uint8_t* data, size_t len);

// Receive data via SPI
int spi_transport_receive(uint8_t* data, size_t len);

// Process SPI events
int spi_transport_process(void);

// Deinitialize SPI
int spi_transport_deinit(void);

#endif /* _SPI_TRANSPORT_H */