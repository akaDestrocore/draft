#include "spi_transport.h"
#include <string.h>

// SPI transport state
typedef struct {
    SPITransport_Config_t* config;
    uint8_t buffer[SPI_BUFFER_SIZE];
    size_t buffer_index;
    size_t buffer_size;
} SPITransport_State_t;

static SPITransport_State_t spi_state;

// Initialize SPI transport
int spi_transport_init(void* config) {
    SPITransport_Config_t* spi_config = (SPITransport_Config_t*)config;
    spi_state.config = spi_config;
    
    // Initialize SPI
    if (HAL_SPI_Init(spi_config->hspi) != HAL_OK) {
        return -1;
    }
    
    // Configure CS pin as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = spi_config->cs_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(spi_config->cs_port, &GPIO_InitStruct);
    
    // Set CS high (inactive)
    HAL_GPIO_WritePin(spi_config->cs_port, spi_config->cs_pin, GPIO_PIN_SET);
    
    // Clear buffer
    memset(spi_state.buffer, 0, SPI_BUFFER_SIZE);
    spi_state.buffer_index = 0;
    spi_state.buffer_size = 0;
    
    return 0;
}

// Send data via SPI
int spi_transport_send(const uint8_t* data, size_t len) {
    // Select device
    HAL_GPIO_WritePin(spi_state.config->cs_port, spi_state.config->cs_pin, GPIO_PIN_RESET);
    
    // Send data
    if (HAL_SPI_Transmit(spi_state.config->hspi, (uint8_t*)data, len, spi_state.config->timeout) != HAL_OK) {
        HAL_GPIO_WritePin(spi_state.config->cs_port, spi_state.config->cs_pin, GPIO_PIN_SET);
        return -1;
    }
    
    // Deselect device
    HAL_GPIO_WritePin(spi_state.config->cs_port, spi_state.config->cs_pin, GPIO_PIN_SET);
    
    return len;
}

// Receive data via SPI
int spi_transport_receive(uint8_t* data, size_t len) {
    if (spi_state.buffer_size > 0) {
        // We have data in buffer
        size_t copy_size = (len < spi_state.buffer_size) ? len : spi_state.buffer_size;
        memcpy(data, spi_state.buffer, copy_size);
        
        // Move remaining data to beginning of buffer
        memmove(spi_state.buffer, 
                spi_state.buffer + copy_size, 
                spi_state.buffer_size - copy_size);
                
        spi_state.buffer_size -= copy_size;
        return copy_size;
    }
    
    // Select device
    HAL_GPIO_WritePin(spi_state.config->cs_port, spi_state.config->cs_pin, GPIO_PIN_RESET);
    
    // Receive data
    if (HAL_SPI_Receive(spi_state.config->hspi, data, len, spi_state.config->timeout) != HAL_OK) {
        HAL_GPIO_WritePin(spi_state.config->cs_port, spi_state.config->cs_pin, GPIO_PIN_SET);
        return -1;
    }
    
    // Deselect device
    HAL_GPIO_WritePin(spi_state.config->cs_port, spi_state.config->cs_pin, GPIO_PIN_SET);
    
    return len;
}

// Process SPI events
int spi_transport_process(void) {
    // No background processing needed for SPI in this implementation
    return 0;
}

// Deinitialize SPI
int spi_transport_deinit(void) {
    return (HAL_SPI_DeInit(spi_state.config->hspi) == HAL_OK) ? 0 : -1;
}