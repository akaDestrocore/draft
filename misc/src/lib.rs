#![no_std]

use core::cell::UnsafeCell;

pub const RING_BUFFER_SIZE: usize = 256;

#[repr(C)]
pub struct RingBuffer {
    buffer: UnsafeCell<[u8; RING_BUFFER_SIZE]>,
    head: UnsafeCell<usize>,
    tail: UnsafeCell<usize>,
    count: UnsafeCell<usize>,
}

// Safety: We ensure proper synchronization through interrupt-free sections
unsafe impl Sync for RingBuffer {}

impl RingBuffer {
    // Create a new, empty ring buffer
    pub const fn new() -> Self {
        Self {
            buffer: UnsafeCell::new([0; RING_BUFFER_SIZE]),
            head: UnsafeCell::new(0),
            tail: UnsafeCell::new(0),
            count: UnsafeCell::new(0),
        }
    }

    // Write a byte to the buffer, returns false if buffer is full
    pub fn write(&self, byte: u8) -> bool {
        // This function is called from both main context and interrupt
        cortex_m::interrupt::free(|_| unsafe {
            let count = *self.count.get();
            
            if count < RING_BUFFER_SIZE {
                // Buffer has space
                let head = &mut *self.head.get();
                let buffer = &mut *self.buffer.get();
                
                // Store byte at head position
                buffer[*head] = byte;
                
                // Move head (with wraparound)
                *head = (*head + 1) % RING_BUFFER_SIZE;
                
                // Increment count
                *self.count.get() += 1;
                
                true
            } else {
                false
            }
        })
    }

    // Read a byte from the buffer
    pub fn read(&self) -> Option<u8> {
        // This function is called from both main context and interrupt
        cortex_m::interrupt::free(|_| unsafe {
            let count = *self.count.get();
            
            if count > 0 {
                // Buffer has data
                let tail = &mut *self.tail.get();
                let buffer = &*self.buffer.get();
                
                // Read byte from tail position
                let byte = buffer[*tail];
                
                // Move tail (with wraparound)
                *tail = (*tail + 1) % RING_BUFFER_SIZE;
                
                // Decrement count
                *self.count.get() -= 1;
                
                Some(byte)
            } else {
                None
            }
        })
    }

    // Check if the buffer is empty
    pub fn is_empty(&self) -> bool {
        cortex_m::interrupt::free(|_| unsafe {
            *self.count.get() == 0
        })
    }

    // Check if the buffer is full
    pub fn is_full(&self) -> bool {
        cortex_m::interrupt::free(|_| unsafe {
            *self.count.get() == RING_BUFFER_SIZE
        })
    }

    // Get current number of bytes in the buffer
    pub fn len(&self) -> usize {
        cortex_m::interrupt::free(|_| unsafe {
            *self.count.get()
        })
    }

    // Clear the buffer
    pub fn clear(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            *self.head.get() = 0;
            *self.tail.get() = 0;
            *self.count.get() = 0;
        })
    }
}