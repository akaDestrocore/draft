#![no_std]

// Export modules
pub mod ring_buffer;
pub mod xmodem;
pub mod image;
pub mod vec;

pub use ring_buffer::RingBuffer;
pub use xmodem::{
    XmodemReceiver, XmodemError, XmodemState, UpdateType, FlashOperations, CryptoOperations,
    X_SOH, X_STX, X_EOT, X_ACK, X_NAK, X_CAN, X_C,
    SLOT_2_APP_ADDR, SLOT_2_VER_ADDR, UPDATER_ADDR, PATCH_ADDR, BACKUP_ADDR
};