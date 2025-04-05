#![no_std]

// Export modules
pub mod ring_buffer;
pub mod image;
pub mod flash;
pub mod systick;
pub mod firmware_update;

pub use ring_buffer::RingBuffer;
pub use systick::{get_tick_ms, wait_ms, setup_systick, increment_tick};
pub use firmware_update::{
    process_xmodem, 
    start_update, 
    get_state, 
    set_state, 
    is_update_in_progress, 
    queue_string,
    XmodemState
};