#include "ring_buffer.h"

// Initialize a ring buffer
void ring_buffer_init(RingBuffer_t* rb) {
    __disable_irq();
    
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    
    __enable_irq();
}

// Write a byte to the buffer
bool ring_buffer_write(RingBuffer_t* rb, uint8_t byte) {
    bool result = false;
    
    __disable_irq();
    
    if (rb->count < RING_BUFFER_SIZE) {
        // Buffer has space
        rb->buffer[rb->head] = byte;
        
        // Move head
        rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
        
        // Increment count
        rb->count++;
        
        result = true;
    }
    
    __enable_irq();
    
    return result;
}

// Read a byte from the buffer
bool ring_buffer_read(RingBuffer_t* rb, uint8_t* byte) {
    bool result = false;
    
    __disable_irq();
    
    if (rb->count > 0) {
        // Buffer has data
        *byte = rb->buffer[rb->tail];
        
        // Move tail
        rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
        
        // Decrement count
        rb->count--;
        
        result = true;
    }
    
    __enable_irq();
    
    return result;
}

// Check if the buffer is empty
bool ring_buffer_is_empty(RingBuffer_t* rb) {
    bool result;
    
    __disable_irq();
    result = (rb->count == 0);
    __enable_irq();
    
    return result;
}

// Check if the buffer is full
bool ring_buffer_is_full(RingBuffer_t* rb) {
    bool result;
    
    __disable_irq();
    result = (rb->count == RING_BUFFER_SIZE);
    __enable_irq();
    
    return result;
}

// Get number of bytes in the buffer
size_t ring_buffer_len(RingBuffer_t* rb) {
    size_t result;
    
    __disable_irq();
    result = rb->count;
    __enable_irq();
    
    return result;
}

// Clear the buffer
void ring_buffer_clear(RingBuffer_t* rb) {
    __disable_irq();
    
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    
    __enable_irq();
}

// Get tail position
size_t ring_buffer_get_tail(RingBuffer_t* rb) {
    size_t result;
    
    __disable_irq();
    result = rb->tail;
    __enable_irq();
    
    return result;
}

// Get head position
size_t ring_buffer_get_head(RingBuffer_t* rb) {
    size_t result;
    
    __disable_irq();
    result = rb->head;
    __enable_irq();
    
    return result;
}

// Get a byte from the buffer at a specific index without removing it
uint8_t ring_buffer_get_byte(RingBuffer_t* rb, size_t index) {
    uint8_t result;
    
    __disable_irq();
    result = rb->buffer[index % RING_BUFFER_SIZE];
    __enable_irq();
    
    return result;
}

// Peek at the next byte without removing it
bool ring_buffer_peek(RingBuffer_t* rb, uint8_t* byte) {
    bool result = false;
    
    __disable_irq();
    
    if (rb->count > 0) {
        *byte = rb->buffer[rb->tail];
        result = true;
    }
    
    __enable_irq();
    
    return result;
}