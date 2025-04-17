#ifndef _RING_BUFFER_H
#define _RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// Size of the ring buffer
#define RING_BUFFER_SIZE 2048

// Ring buffer structure
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile size_t head;
    volatile size_t tail;
    volatile size_t count;
} RingBuffer_t;

// Initialize a new ring buffer
void ring_buffer_init(RingBuffer_t* rb);

// Write a byte to the buffer
// Returns true if successful, false if buffer is full
bool ring_buffer_write(RingBuffer_t* rb, uint8_t byte);

// Read a byte from the buffer
// Returns true if a byte was read, false if buffer is empty
// The read byte is stored in the location pointed to by 'byte'
bool ring_buffer_read(RingBuffer_t* rb, uint8_t* byte);

// Check if the buffer is empty
bool ring_buffer_is_empty(RingBuffer_t* rb);

// Check if the buffer is full
bool ring_buffer_is_full(RingBuffer_t* rb);

// Get number of bytes in the buffer
size_t ring_buffer_len(RingBuffer_t* rb);

// Clear the buffer
void ring_buffer_clear(RingBuffer_t* rb);

// Get tail position
size_t ring_buffer_get_tail(RingBuffer_t* rb);

// Get head position
size_t ring_buffer_get_head(RingBuffer_t* rb);

// Get a byte from the buffer at a specific index without removing it
uint8_t ring_buffer_get_byte(RingBuffer_t* rb, size_t index);

// Peek at the next byte without removing it
// Returns true if a byte was peeked, false if buffer is empty
// The peeked byte is stored in the location pointed to by 'byte'
bool ring_buffer_peek(RingBuffer_t* rb, uint8_t* byte);

#endif /* _RING_BUFFER_H */