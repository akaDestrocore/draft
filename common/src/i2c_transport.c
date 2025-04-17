#include "i2c_transport.h"

// I2C transport state
typedef struct {
    I2CTransport_Config_t* config;
    uint8_t buffer[I2C_BUFFER_SIZE];
    size_t buffer_index;
    size_t buffer_size;
} I2CTransport_State_t;

static I2CTransport_State_t i2c_state;

// Initialize I2C transport
int i2c_transport_init(void* config) {
    I2CTransport_Config_t* i2c_config = (I2CTransport_Config_t*)config;
    i2c_state.config = i2c_config;
    
    // Initialize I2C
    if (HAL_I2C_Init(i2c_config->hi2c) != HAL_OK) {
        return -1;
    }
    
    // Clear buffer
    memset(i2c_state.buffer, 0, I2C_BUFFER_SIZE);
    i2c_state.buffer_index = 0;
    i2c_state.buffer_size = 0;
    
    return 0;
}

// Send data via I2C
int i2c_transport_send(const uint8_t* data, size_t len) {
    if (HAL_I2C_Master_Transmit(i2c_state.config->hi2c, 
                               i2c_state.config->device_address, 
                               (uint8_t*)data, 
                               len, 
                               i2c_state.config->timeout) != HAL_OK) {
        return -1;
    }
    
    return len;
}

// Receive data via I2C
int i2c_transport_receive(uint8_t* data, size_t len) {
    if (i2c_state.buffer_size > 0) {
        // We have data in buffer
        size_t copy_size = (len < i2c_state.buffer_size) ? len : i2c_state.buffer_size;
        memcpy(data, i2c_state.buffer, copy_size);
        
        // Move remaining data to beginning of buffer
        memmove(i2c_state.buffer, 
                i2c_state.buffer + copy_size, 
                i2c_state.buffer_size - copy_size);
                
        i2c_state.buffer_size -= copy_size;
        return copy_size;
    }
    
    // No buffered data, read directly from I2C
    if (HAL_I2C_Master_Receive(i2c_state.config->hi2c, 
                              i2c_state.config->device_address, 
                              data, 
                              len, 
                              i2c_state.config->timeout) != HAL_OK) {
        return -1;
    }
    
    return len;
}

// Process I2C events
int i2c_transport_process(void) {
    // No background processing needed for I2C in this implementation
    return 0;
}

// Deinitialize I2C
int i2c_transport_deinit(void) {
    return (HAL_I2C_DeInit(i2c_state.config->hi2c) == HAL_OK) ? 0 : -1;
}