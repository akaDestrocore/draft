use core::{
    convert::TryFrom,
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};
use cortex_m::asm;
use rmodem::{Control, OneKData, Sequence, XmodemOneKPacket};
use stm32f4 as pac;

use crate::{
    flash,
    image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER},
    ring_buffer::RingBuffer,
};

use core::sync::atomic::AtomicU32;

// Constants for flash addresses (these should match your memory.x file)
pub const UPDATER_ADDR: u32 = 0x08008000;
pub const APP_ADDR: u32 = 0x08020000;

// State machine states
const XMODEM_STATE_IDLE: u8 = 0;
const XMODEM_STATE_INIT: u8 = 1;
const XMODEM_STATE_RECEIVING: u8 = 2;
const XMODEM_STATE_COMPLETE: u8 = 3;
const XMODEM_STATE_ERROR: u8 = 4;

// Error codes
const ERROR_NONE: u8 = 0;
const ERROR_INVALID_HEADER: u8 = 1;
const ERROR_NOT_NEWER_VERSION: u8 = 2;
const ERROR_FLASH_ERASE_FAILURE: u8 = 3;
const ERROR_FLASH_WRITE_FAILURE: u8 = 4;

// Global state variables
static UPDATE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static UPDATE_TARGET_APP: AtomicBool = AtomicBool::new(false);
static XMODEM_RECEIVE_STATE: AtomicU8 = AtomicU8::new(XMODEM_STATE_IDLE);
static CURRENT_ERROR: AtomicU8 = AtomicU8::new(ERROR_NONE);
static LAST_C_SENT_TIME: AtomicU32 = AtomicU32::new(0);

// Constants
const C_SEND_INTERVAL_MS: u32 = 1000; // Интервал отправки символа 'C' в миллисекундах

// Function to get error message from error code
fn get_error_message(error_code: u8) -> &'static str {
    match error_code {
        ERROR_INVALID_HEADER => "Invalid image header",
        ERROR_NOT_NEWER_VERSION => "New firmware is not newer than current version",
        ERROR_FLASH_ERASE_FAILURE => "Failed to erase flash sector",
        ERROR_FLASH_WRITE_FAILURE => "Failed to write to flash",
        _ => "Unknown error",
    }
}

// Function to queue a string to be transmitted
pub fn queue_string(tx_buffer: &RingBuffer, s: &str) {
    for byte in s.bytes() {
        tx_buffer.write(byte);
    }
}

// Function to start the firmware update process
pub fn start_firmware_update(tx_buffer: &RingBuffer) {
    // Clear screen and show update options
    queue_string(tx_buffer, "\r\n\r\nPlease select an update method:\r\n\r\n");
    queue_string(tx_buffer, " > 'A' - Update Application image\r\n\r\n");
    queue_string(tx_buffer, " > 'U' - Update Updater image\r\n\r\n");
    
    // Set update flag
    UPDATE_IN_PROGRESS.store(true, Ordering::SeqCst);
}

// Function to handle firmware update selection
pub fn handle_firmware_update(tx_buffer: &RingBuffer, rx_byte: u8) -> bool {
    if !UPDATE_IN_PROGRESS.load(Ordering::SeqCst) {
        return false;
    }

    match rx_byte {
        b'A' | b'a' => {
            queue_string(tx_buffer, "\r\nPreparing to update application. Send firmware via XMODEM-1K...\r\n");
            UPDATE_TARGET_APP.store(true, Ordering::SeqCst);
            
            // Start XMODEM receive
            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_INIT, Ordering::SeqCst);
            return true;
        },
        b'U' | b'u' => {
            queue_string(tx_buffer, "\r\nPreparing to update updater. Send firmware via XMODEM-1K...\r\n");
            UPDATE_TARGET_APP.store(false, Ordering::SeqCst);
            
            // Start XMODEM receive
            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_INIT, Ordering::SeqCst);
            return true;
        },
        _ => {
            queue_string(tx_buffer, "\r\nInvalid selection. Please choose 'A' or 'U'.\r\n");
            return true;
        }
    }
}

// Check if update is in progress
pub fn is_update_in_progress() -> bool {
    UPDATE_IN_PROGRESS.load(Ordering::SeqCst)
}

// Send 'C' character to request XMODEM-CRC transfer
fn send_c_character(tx_buffer: &RingBuffer, current_ms: u32) {
    let last_sent = LAST_C_SENT_TIME.load(Ordering::Relaxed);
    
    // Check if it's time to send another 'C'
    if current_ms - last_sent >= C_SEND_INTERVAL_MS {
        // Send 'C' character
        tx_buffer.write(Control::Idle.into_u8());
        
        // Update the last sent time
        LAST_C_SENT_TIME.store(current_ms, Ordering::Relaxed);
    }
}

// Function to process XMODEM receive state machine
pub fn process_xmodem_receive(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) -> bool {
    static mut RECEIVE_BUFFER: [u8; XmodemOneKPacket::LEN] = [0; XmodemOneKPacket::LEN];
    static mut CURRENT_ADDR: u32 = 0;
    static mut SEQUENCE: u8 = 1;
    static mut FIRST_PACKET: bool = true;
    static mut PACKET_RECEIVED: bool = false;
    
    let state = XMODEM_RECEIVE_STATE.load(Ordering::SeqCst);
    
    if state == XMODEM_STATE_IDLE {
        return false;
    }
    
    let current_ms = crate::systick::get_tick_ms();
    
    match state {
        XMODEM_STATE_INIT => {
            // Initialize variables
            unsafe {
                RECEIVE_BUFFER = [0; XmodemOneKPacket::LEN];
                CURRENT_ADDR = if UPDATE_TARGET_APP.load(Ordering::SeqCst) {
                    APP_ADDR
                } else {
                    UPDATER_ADDR
                };
                SEQUENCE = 1;
                FIRST_PACKET = true;
                PACKET_RECEIVED = false;
            }
            
            // Reset error code
            CURRENT_ERROR.store(ERROR_NONE, Ordering::SeqCst);
            
            // Initialize the last C sent time
            LAST_C_SENT_TIME.store(current_ms, Ordering::Relaxed);
            
            // Send first 'C' character to initiate XMODEM-1K transfer
            tx_buffer.write(Control::Idle.into_u8());
            
            // Move to receiving state
            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_RECEIVING, Ordering::SeqCst);
        },
        XMODEM_STATE_RECEIVING => {
            unsafe {
                // If first packet hasn't been received yet, send 'C' periodically
                if FIRST_PACKET && !PACKET_RECEIVED {
                    send_c_character(tx_buffer, current_ms);
                }
                
                // Check for EOT (End of Transmission)
                if let Some(byte) = rx_buffer.read() {
                    if byte == Control::Eot.into_u8() {
                        // Acknowledge EOT
                        tx_buffer.write(Control::Ack.into_u8());
                        
                        // Update complete
                        XMODEM_RECEIVE_STATE.store(XMODEM_STATE_COMPLETE, Ordering::SeqCst);
                        return true;
                    }
                }
                
                // If we have enough data, try to process a packet
                let buffer_len = rx_buffer.len();
                if buffer_len >= XmodemOneKPacket::LEN {
                    // Read the packet from the buffer
                    for i in 0..XmodemOneKPacket::LEN {
                        if let Some(byte) = rx_buffer.read() {
                            RECEIVE_BUFFER[i] = byte;
                        }
                    }
                    
                    // Reset timeout counter since we received data
                    PACKET_RECEIVED = true;
                    
                    // Try to parse and validate the packet
                    match XmodemOneKPacket::try_from(&RECEIVE_BUFFER[..]) {
                        Ok(packet) => {
                            // Validate sequence number
                            if packet.sequence().into_u8() != SEQUENCE {
                                // Wrong sequence number
                                tx_buffer.write(Control::Nak.into_u8());
                                return true;
                            }
                            
                            // If this is the first packet, validate the header
                            if FIRST_PACKET {
                                let data = packet.data().inner();
                                let header = &*(data.as_ptr() as *const ImageHeader);
                                
                                // Validate magic number and type
                                let (expected_magic, expected_type) = if UPDATE_TARGET_APP.load(Ordering::SeqCst) {
                                    (IMAGE_MAGIC_APP, IMAGE_TYPE_APP)
                                } else {
                                    (IMAGE_MAGIC_UPDATER, IMAGE_TYPE_UPDATER)
                                };
                                
                                if header.image_magic != expected_magic || header.image_type != expected_type {
                                    // Invalid header
                                    tx_buffer.write(Control::Nak.into_u8());
                                    
                                    CURRENT_ERROR.store(ERROR_INVALID_HEADER, Ordering::SeqCst);
                                    XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
                                    return true;
                                }
                                
                                // Check if there's an existing firmware
                                let current_header_ptr = CURRENT_ADDR as *const ImageHeader;
                                let current_magic = (*current_header_ptr).image_magic;
                                
                                if current_magic != 0xFFFFFFFF {
                                    // There is existing firmware, validate version
                                    let current_header = &*current_header_ptr;
                                    
                                    if !header.is_newer_than(current_header) {
                                        // Not newer version
                                        tx_buffer.write(Control::Nak.into_u8());
                                        
                                        CURRENT_ERROR.store(ERROR_NOT_NEWER_VERSION, Ordering::SeqCst);
                                        XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
                                        return true;
                                    }
                                }
                                
                                // Erase the target flash sector
                                if flash::erase_sector(p, CURRENT_ADDR) == 0 {
                                    tx_buffer.write(Control::Nak.into_u8());
                                    
                                    CURRENT_ERROR.store(ERROR_FLASH_ERASE_FAILURE, Ordering::SeqCst);
                                    XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
                                    return true;
                                }
                                
                                FIRST_PACKET = false;
                            }
                            
                            // Write the data to flash - get the data from the packet
                            let packet_data = packet.data().inner();
                            if flash::write(p, packet_data, CURRENT_ADDR) != 0 {
                                tx_buffer.write(Control::Nak.into_u8());
                                
                                CURRENT_ERROR.store(ERROR_FLASH_WRITE_FAILURE, Ordering::SeqCst);
                                XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
                                return true;
                            }
                            
                            // Update address for next packet
                            CURRENT_ADDR += OneKData::LEN as u32;
                            
                            // Acknowledge the packet
                            tx_buffer.write(Control::Ack.into_u8());
                            
                            // Increment sequence number for next packet
                            SEQUENCE = SEQUENCE.wrapping_add(1);
                        },
                        Err(_) => {
                            // Invalid packet
                            tx_buffer.write(Control::Nak.into_u8());
                        }
                    }
                }
            }
        },
        XMODEM_STATE_COMPLETE => {
            // Update completed successfully
            queue_string(tx_buffer, "\r\nFirmware update complete!\r\n");
            
            // Reset flags and state
            UPDATE_IN_PROGRESS.store(false, Ordering::SeqCst);
            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_IDLE, Ordering::SeqCst);
            
            // Return true to indicate state machine is done
            return true;
        },
        XMODEM_STATE_ERROR => {
            // Update failed - get the error message
            let error_code = CURRENT_ERROR.load(Ordering::SeqCst);
            let error_message = get_error_message(error_code);
            
            // Print the error message
            queue_string(tx_buffer, "\r\nFirmware update failed: ");
            queue_string(tx_buffer, error_message);
            queue_string(tx_buffer, "\r\n");
            
            // Reset flags and state
            UPDATE_IN_PROGRESS.store(false, Ordering::SeqCst);
            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_IDLE, Ordering::SeqCst);
            
            // Return true to indicate state machine is done
            return true;
        },
        _ => {
            // Invalid state, reset
            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_IDLE, Ordering::SeqCst);
            UPDATE_IN_PROGRESS.store(false, Ordering::SeqCst);
        }
    }
    
    // Return true to indicate state machine is still running
    true
}

// Delay function
pub fn delay_ms(ms: u32) {
    let mut count: u32 = ms * 8000; 
    while count > 0 {
        count -= 1;
        asm::nop();
    }
}