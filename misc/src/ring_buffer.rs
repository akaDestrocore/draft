#![no_std]

use core::cell::UnsafeCell;

pub const RING_BUFFER_SIZE: usize = 1024;

#[repr(C)]
pub struct RingBuffer {
    buffer: UnsafeCell<[u8; RING_BUFFER_SIZE]>,
    head: UnsafeCell<usize>,
    tail: UnsafeCell<usize>,
    count: UnsafeCell<usize>,
}

unsafe impl Sync for RingBuffer {}

impl RingBuffer {
    pub const fn new() -> Self {
        Self {
            buffer: UnsafeCell::new([0; RING_BUFFER_SIZE]),
            head: UnsafeCell::new(0),
            tail: UnsafeCell::new(0),
            count: UnsafeCell::new(0),
        }
    }

    pub fn write(&self, byte: u8) -> bool {
        cortex_m::interrupt::free(|_| unsafe {
            let count: usize = *self.count.get();
            
            if count < RING_BUFFER_SIZE {
                // buffer has space
                let head: &mut usize = &mut *self.head.get();
                let buffer = &mut *self.buffer.get();
                
                // store byte at head position
                buffer[*head] = byte;
                
                // move head
                *head = (*head + 1) % RING_BUFFER_SIZE;
                
                *self.count.get() += 1;
                
                true
            } else {
                false
            }
        })
    }

    // read a byte from the buffer
    pub fn read(&self) -> Option<u8> {
        cortex_m::interrupt::free(|_| unsafe {
            let count: usize = *self.count.get();
            
            if count > 0 {
                // buffer has data
                let tail: &mut usize = &mut *self.tail.get();
                let buffer = &*self.buffer.get();
                
                // read byte from tail
                let byte: u8 = buffer[*tail];
                
                // move tail
                *tail = (*tail + 1) % RING_BUFFER_SIZE;
                
                *self.count.get() -= 1;
                
                Some(byte)
            } else {
                None
            }
        })
    }

    // check if the buffer is empty
    pub fn is_empty(&self) -> bool {
        cortex_m::interrupt::free(|_| unsafe {
            *self.count.get() == 0
        })
    }

    // check if the buffer is full
    pub fn is_full(&self) -> bool {
        cortex_m::interrupt::free(|_| unsafe {
            *self.count.get() == RING_BUFFER_SIZE
        })
    }

    // get number of bytes in the buffer
    pub fn len(&self) -> usize {
        cortex_m::interrupt::free(|_| unsafe {
            *self.count.get()
        })
    }

    // clear the buffer
    pub fn clear(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            *self.head.get() = 0;
            *self.tail.get() = 0;
            *self.count.get() = 0;
        })
    }

    // get tail position
    pub fn get_tail(&self) -> usize {
        cortex_m::interrupt::free(|_| unsafe {
            *self.tail.get()
        })
    }

    // get head position
    pub fn get_head(&self) -> usize {
        cortex_m::interrupt::free(|_| unsafe {
            *self.head.get()
        })
    }

    // get a byte from the buffer at a specific index without removing it
    pub fn get_buffer_byte(&self, index: usize) -> u8 {
        cortex_m::interrupt::free(|_| unsafe {
            let buffer = &*self.buffer.get();
            buffer[index % RING_BUFFER_SIZE]
        })
    }

    // peek at the next byte without removing it
    pub fn peek(&self) -> Option<u8> {
        cortex_m::interrupt::free(|_| unsafe {
            let count: usize = *self.count.get();
            if count > 0 {
                let tail: usize = *self.tail.get();
                let buffer = &*self.buffer.get();
                Some(buffer[tail])
            } else {
                None
            }
        })
    }

    // access raw buffer safely
    pub fn with_raw_buffer<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&[u8], usize, usize) -> R
    {
        cortex_m::interrupt::free(|_| unsafe {
            let buffer = &*self.buffer.get();
            let tail: usize = *self.tail.get();
            let head: usize = *self.head.get();
            f(buffer, tail, head)
        })
    }
}