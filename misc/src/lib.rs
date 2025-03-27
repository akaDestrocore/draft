#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};
use core::cell::UnsafeCell;

pub const RING_BUFFER_SIZE: usize = 64;

pub struct RingBuffer {
    buffer: UnsafeCell<[u8; RING_BUFFER_SIZE]>,
    head: UnsafeCell<usize>,
    tail: UnsafeCell<usize>,
    count: UnsafeCell<usize>,
    tx_in_progress: AtomicBool,
}

unsafe impl Sync for RingBuffer {}

impl RingBuffer {
    /// Create a new, empty ring buffer
    pub const fn new() -> Self {
        Self {
            buffer: UnsafeCell::new([0; RING_BUFFER_SIZE]),
            head: UnsafeCell::new(0),
            tail: UnsafeCell::new(0),
            count: UnsafeCell::new(0),
            tx_in_progress: AtomicBool::new(false),
        }
    }

    /// Returns true if the write was successful
    pub fn write(&self, data: u8) -> bool {
        cortex_m::interrupt::free(|_| {
            let count: usize = unsafe { *self.count.get() };
            
            if count < RING_BUFFER_SIZE {
                let buffer = unsafe { &mut *self.buffer.get() };
                let head: &mut usize = unsafe { &mut *self.head.get() };
                
                buffer[*head] = data;
                *head = (*head + 1) % RING_BUFFER_SIZE;
                
                unsafe {
                    *self.count.get() += 1;
                }
                
                true
            } else {
                false
            }
        })
    }

    /// Returns None if the buffer is empty
    pub fn read(&self) -> Option<u8> {
        cortex_m::interrupt::free(|_| {
            let count: usize = unsafe { *self.count.get() };
            
            if count > 0 {
                let buffer  = unsafe { &*self.buffer.get() };
                let tail: &mut usize = unsafe { &mut *self.tail.get() };
                
                let data: u8 = buffer[*tail];
                *tail = (*tail + 1) % RING_BUFFER_SIZE;
                
                unsafe {
                    *self.count.get() -= 1;
                }
                
                Some(data)
            } else {
                None
            }
        })
    }

    /// Check if the buffer is empty
    pub fn is_empty(&self) -> bool {
        cortex_m::interrupt::free(|_| unsafe { *self.count.get() == 0 })
    }

    /// Check if the buffer is full
    pub fn is_full(&self) -> bool {
        cortex_m::interrupt::free(|_| unsafe { *self.count.get() == RING_BUFFER_SIZE })
    }

    /// Get the number of bytes in the buffer
    pub fn len(&self) -> usize {
        cortex_m::interrupt::free(|_| unsafe { *self.count.get() })
    }

    /// Clear the buffer
    pub fn clear(&self) {
        cortex_m::interrupt::free(|_| {
            unsafe {
                *self.head.get() = 0;
                *self.tail.get() = 0;
                *self.count.get() = 0;
            }
        })
    }

    /// Set the TX in progress flag
    pub fn set_tx_in_progress(&self, in_progress: bool) {
        self.tx_in_progress.store(in_progress, Ordering::SeqCst);
    }

    /// Check if TX is in progress
    pub fn is_tx_in_progress(&self) -> bool {
        self.tx_in_progress.load(Ordering::SeqCst)
    }
}