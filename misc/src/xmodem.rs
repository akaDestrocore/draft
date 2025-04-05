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
    systick,
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
const ERROR_INVALID_PACKET: u8 = 5;
const ERROR_TIMEOUT: u8 = 6;

// Global state variables
static UPDATE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static UPDATE_TARGET_APP: AtomicBool = AtomicBool::new(false);
static XMODEM_RECEIVE_STATE: AtomicU8 = AtomicU8::new(XMODEM_STATE_IDLE);
static CURRENT_ERROR: AtomicU8 = AtomicU8::new(ERROR_NONE);
static LAST_C_SENT_TIME: AtomicU32 = AtomicU32::new(0);
static LAST_ACTIVITY_TIME: AtomicU32 = AtomicU32::new(0);

// Constants
const C_SEND_INTERVAL_MS: u32 = 1000; // Interval to send 'C' character in milliseconds
const XMODEM_TIMEOUT_MS: u32 = 10000; // 10 second timeout for XMODEM transfer

// Function to get error message from error code
fn get_error_message(error_code: u8) -> &'static str {
    match error_code {
        ERROR_INVALID_HEADER => "Invalid image header",
        ERROR_NOT_NEWER_VERSION => "New firmware is not newer than current version",
        ERROR_FLASH_ERASE_FAILURE => "Failed to erase flash sector",
        ERROR_FLASH_WRITE_FAILURE => "Failed to write to flash",
        ERROR_INVALID_PACKET => "Invalid XMODEM packet format",
        ERROR_TIMEOUT => "XMODEM transfer timed out",
        _ => "Unknown error",
    }
}

// Helper function to format a byte as hex string
fn format_byte(byte: u8) -> [u8; 2] {
    const HEX_DIGITS: [u8; 16] = *b"0123456789ABCDEF";
    [
        HEX_DIGITS[(byte >> 4) as usize],
        HEX_DIGITS[(byte & 0xF) as usize],
    ]
}

// Helper function to format a u16 as hex string - returns the hex bytes
fn format_u16(value: u16) -> [u8; 4] {
    let b1 = ((value >> 8) & 0xFF) as u8;
    let b2 = (value & 0xFF) as u8;
    let upper = format_byte(b1);
    let lower = format_byte(b2);
    [upper[0], upper[1], lower[0], lower[1]]
}

// Helper function to format a u32 as hex string - returns the hex bytes
fn format_u32(value: u32) -> [u8; 8] {
    let b1 = ((value >> 24) & 0xFF) as u8;
    let b2 = ((value >> 16) & 0xFF) as u8;
    let b3 = ((value >> 8) & 0xFF) as u8;
    let b4 = (value & 0xFF) as u8;
    
    let bytes1 = format_byte(b1);
    let bytes2 = format_byte(b2);
    let bytes3 = format_byte(b3);
    let bytes4 = format_byte(b4);
    
    [
        bytes1[0], bytes1[1], 
        bytes2[0], bytes2[1], 
        bytes3[0], bytes3[1], 
        bytes4[0], bytes4[1]
    ]
}

// Converts a u8 to ASCII decimal digits (returns the bytes and length)
fn u8_to_dec_bytes(value: u8, buf: &mut [u8; 3]) -> usize {
    if value < 10 {
        buf[0] = b'0' + value;
        return 1;
    } else if value < 100 {
        buf[0] = b'0' + (value / 10);
        buf[1] = b'0' + (value % 10);
        return 2;
    } else {
        buf[0] = b'0' + (value / 100);
        buf[1] = b'0' + ((value / 10) % 10);
        buf[2] = b'0' + (value % 10);
        return 3;
    }
}

// Converts a u16 to ASCII decimal digits (returns the bytes and length)
fn u16_to_dec_bytes(mut value: u16, buf: &mut [u8; 5]) -> usize {
    if value == 0 {
        buf[0] = b'0';
        return 1;
    }
    
    let mut idx = 0;
    let mut digits = [0u8; 5];
    let mut digit_count = 0;
    
    while value > 0 {
        digits[digit_count] = (value % 10) as u8;
        value /= 10;
        digit_count += 1;
    }
    
    for i in (0..digit_count).rev() {
        buf[idx] = b'0' + digits[i];
        idx += 1;
    }
    
    idx
}

// Converts a usize to ASCII decimal digits (returns the bytes and length)
fn usize_to_dec_bytes(mut value: usize, buf: &mut [u8; 10]) -> usize {
    if value == 0 {
        buf[0] = b'0';
        return 1;
    }
    
    let mut idx = 0;
    let mut digits = [0u8; 10];
    let mut digit_count = 0;
    
    while value > 0 {
        digits[digit_count] = (value % 10) as u8;
        value /= 10;
        digit_count += 1;
    }
    
    for i in (0..digit_count).rev() {
        buf[idx] = b'0' + digits[i];
        idx += 1;
    }
    
    idx
}

// Function to queue a string to be transmitted
pub fn queue_string(tx_buffer: &RingBuffer, s: &str) {
    for byte in s.bytes() {
        tx_buffer.write(byte);
    }
}

// Function to queue a hex byte to the TX buffer
fn queue_hex_byte(tx_buffer: &RingBuffer, byte: u8) {
    let hex = format_byte(byte);
    tx_buffer.write(hex[0]);
    tx_buffer.write(hex[1]);
}

// Function to queue a hex u16 to the TX buffer
fn queue_hex_u16(tx_buffer: &RingBuffer, value: u16) {
    let hex = format_u16(value);
    for &b in hex.iter() {
        tx_buffer.write(b);
    }
}

// Function to queue a hex u32 to the TX buffer
fn queue_hex_u32(tx_buffer: &RingBuffer, value: u32) {
    let hex = format_u32(value);
    for &b in hex.iter() {
        tx_buffer.write(b);
    }
}

// Queue a decimal u8 to the TX buffer
fn queue_dec_u8(tx_buffer: &RingBuffer, value: u8) {
    let mut buf = [0u8; 3];
    let len = u8_to_dec_bytes(value, &mut buf);
    
    for i in 0..len {
        tx_buffer.write(buf[i]);
    }
}

// Queue a decimal u16 to the TX buffer
fn queue_dec_u16(tx_buffer: &RingBuffer, value: u16) {
    let mut buf = [0u8; 5];
    let len = u16_to_dec_bytes(value, &mut buf);
    
    for i in 0..len {
        tx_buffer.write(buf[i]);
    }
}

// Queue a decimal u32 to the TX buffer
fn queue_dec_u32(tx_buffer: &RingBuffer, value: u32) {
    // For simplicity, just convert to decimal step by step
    // This isn't the most efficient way but works for our logging purposes
    let mut buf = [0u8; 10];
    let mut idx = 0;
    
    if value == 0 {
        tx_buffer.write(b'0');
        return;
    }
    
    let mut val = value;
    let mut digits = [0u8; 10];
    let mut digit_count = 0;
    
    while val > 0 {
        digits[digit_count] = (val % 10) as u8;
        val /= 10;
        digit_count += 1;
    }
    
    for i in (0..digit_count).rev() {
        tx_buffer.write(b'0' + digits[i]);
    }
}

// Queue a decimal usize to the TX buffer
fn queue_dec_usize(tx_buffer: &RingBuffer, value: usize) {
    let mut buf = [0u8; 10];
    let len = usize_to_dec_bytes(value, &mut buf);
    
    for i in 0..len {
        tx_buffer.write(buf[i]);
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

// Check for timeout in XMODEM transfer
fn check_timeout(tx_buffer: &RingBuffer, current_ms: u32) -> bool {
    let last_activity = LAST_ACTIVITY_TIME.load(Ordering::Relaxed);
    
    if current_ms - last_activity >= XMODEM_TIMEOUT_MS {
        queue_string(tx_buffer, "\r\nXMODEM transfer timed out\r\n");
        CURRENT_ERROR.store(ERROR_TIMEOUT, Ordering::SeqCst);
        XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
        return true;
    }
    
    return false;
}

// Function to process XMODEM receive state machine
pub fn process_xmodem_receive(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) -> bool {
    static mut RECEIVE_BUFFER: [u8; XmodemOneKPacket::LEN] = [0; XmodemOneKPacket::LEN];
    static mut RECEIVE_INDEX: usize = 0;
    static mut CURRENT_ADDR: u32 = 0;
    static mut SEQUENCE: u8 = 1;
    static mut FIRST_PACKET: bool = true;
    static mut HEADER_VALIDATED: bool = false;
    
    let state = XMODEM_RECEIVE_STATE.load(Ordering::SeqCst);
    
    if state == XMODEM_STATE_IDLE {
        return false;
    }
    
    let current_ms = systick::get_tick_ms();
    
    match state {
        XMODEM_STATE_INIT => {
            // Initialize variables
            unsafe {
                RECEIVE_BUFFER = [0; XmodemOneKPacket::LEN];
                RECEIVE_INDEX = 0;
                CURRENT_ADDR = if UPDATE_TARGET_APP.load(Ordering::SeqCst) {
                    APP_ADDR
                } else {
                    UPDATER_ADDR
                };
                SEQUENCE = 1;
                FIRST_PACKET = true;
                HEADER_VALIDATED = false;
            }
            
            // Reset error code
            CURRENT_ERROR.store(ERROR_NONE, Ordering::SeqCst);
            
            // Initialize the last C sent time and activity time
            LAST_C_SENT_TIME.store(current_ms, Ordering::Relaxed);
            LAST_ACTIVITY_TIME.store(current_ms, Ordering::Relaxed);
            
            // Send first 'C' character to initiate XMODEM-1K transfer
            tx_buffer.write(Control::Idle.into_u8());
            queue_string(tx_buffer, "\r\nSent 'C' to start XMODEM-1K transfer\r\n");
            
            // Move to receiving state
            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_RECEIVING, Ordering::SeqCst);
        },
        XMODEM_STATE_RECEIVING => {
            unsafe {
                // Check for timeout
                if check_timeout(tx_buffer, current_ms) {
                    return true;
                }
                
                // If first packet hasn't been received yet, send 'C' periodically
                if FIRST_PACKET && RECEIVE_INDEX == 0 {
                    send_c_character(tx_buffer, current_ms);
                }
                
                // Check for EOT (End of Transmission) at the beginning of a packet
                if RECEIVE_INDEX == 0 {
                    if let Some(byte) = rx_buffer.peek() {
                        if byte == Control::Eot.into_u8() {
                            // Consume the EOT byte
                            let _ = rx_buffer.read();
                            
                            // Acknowledge EOT
                            tx_buffer.write(Control::Ack.into_u8());
                            queue_string(tx_buffer, "\r\nReceived EOT, sending ACK\r\n");
                            
                            // Update complete
                            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_COMPLETE, Ordering::SeqCst);
                            return true;
                        }
                    }
                }
                
                // Process incoming bytes
                while RECEIVE_INDEX < XmodemOneKPacket::LEN {
                    if let Some(byte) = rx_buffer.read() {
                        // Update activity timestamp
                        LAST_ACTIVITY_TIME.store(current_ms, Ordering::Relaxed);
                        
                        // Add byte to receive buffer
                        RECEIVE_BUFFER[RECEIVE_INDEX] = byte;
                        RECEIVE_INDEX += 1;
                        
                        // For first few bytes, print them for debugging
                        if RECEIVE_INDEX <= 4 {
                            queue_string(tx_buffer, "\r\nReceived byte: 0x");
                            queue_hex_byte(tx_buffer, byte);
                            if RECEIVE_INDEX == 1 && byte != Control::Stx.into_u8() {
                                queue_string(tx_buffer, " (expected STX: 0x02)\r\n");
                            } else {
                                queue_string(tx_buffer, "\r\n");
                            }
                        }
                    } else {
                        // No more bytes to read now
                        break;
                    }
                }
                
                // If we have a full packet, process it
                if RECEIVE_INDEX == XmodemOneKPacket::LEN {
                    queue_string(tx_buffer, "\r\nReceived full packet (");
                    queue_dec_usize(tx_buffer, RECEIVE_INDEX);
                    queue_string(tx_buffer, " bytes)\r\n");
                    
                    // Try to parse and validate the packet
                    match XmodemOneKPacket::try_from(&RECEIVE_BUFFER[..]) {
                        Ok(packet) => {
                            // Validate sequence number
                            if packet.sequence().into_u8() != SEQUENCE {
                                queue_string(tx_buffer, "\r\nSequence mismatch, expected: ");
                                queue_dec_u8(tx_buffer, SEQUENCE);
                                queue_string(tx_buffer, " got: ");
                                queue_dec_u8(tx_buffer, packet.sequence().into_u8());
                                queue_string(tx_buffer, "\r\n");
                                
                                // Send NAK
                                tx_buffer.write(Control::Nak.into_u8());
                                queue_string(tx_buffer, "Sending NAK due to sequence mismatch\r\n");
                                
                                // Reset buffer index to receive another packet
                                RECEIVE_INDEX = 0;
                            }
                            
                            // Handle first packet special case (contains header)
                            if FIRST_PACKET && !HEADER_VALIDATED {
                                let data = packet.data().inner();
                                let header = &*(data.as_ptr() as *const ImageHeader);
                                
                                // Validate magic number and type
                                let (expected_magic, expected_type) = if UPDATE_TARGET_APP.load(Ordering::SeqCst) {
                                    (IMAGE_MAGIC_APP, IMAGE_TYPE_APP)
                                } else {
                                    (IMAGE_MAGIC_UPDATER, IMAGE_TYPE_UPDATER)
                                };
                                
                                // Debug output for header validation
                                queue_string(tx_buffer, "\r\nValidating image header:\r\n");
                                queue_string(tx_buffer, "  Magic: 0x");
                                queue_hex_u32(tx_buffer, header.image_magic);
                                queue_string(tx_buffer, " (expected: 0x");
                                queue_hex_u32(tx_buffer, expected_magic);
                                queue_string(tx_buffer, ")\r\n");
                                
                                queue_string(tx_buffer, "  Type: ");
                                queue_dec_u8(tx_buffer, header.image_type);
                                queue_string(tx_buffer, " (expected: ");
                                queue_dec_u8(tx_buffer, expected_type);
                                queue_string(tx_buffer, ")\r\n");
                                
                                // Validate header
                                if header.image_magic != expected_magic || header.image_type != expected_type {
                                    queue_string(tx_buffer, "\r\nInvalid image header detected!\r\n");
                                    tx_buffer.write(Control::Nak.into_u8());
                                    
                                    CURRENT_ERROR.store(ERROR_INVALID_HEADER, Ordering::SeqCst);
                                    XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
                                    return true;
                                }
                                
                                // Check if there's an existing firmware
                                let dest_addr = CURRENT_ADDR;
                                let current_header_ptr = dest_addr as *const ImageHeader;
                                let current_magic = (*current_header_ptr).image_magic;
                                
                                if current_magic == expected_magic {
                                    // There is existing firmware, validate version
                                    let current_header = &*current_header_ptr;
                                    
                                    queue_string(tx_buffer, "  Version: ");
                                    queue_dec_u8(tx_buffer, header.version_major);
                                    tx_buffer.write(b'.');
                                    queue_dec_u8(tx_buffer, header.version_minor);
                                    tx_buffer.write(b'.');
                                    queue_dec_u8(tx_buffer, header.version_patch);
                                    queue_string(tx_buffer, " (current: ");
                                    queue_dec_u8(tx_buffer, current_header.version_major);
                                    tx_buffer.write(b'.');
                                    queue_dec_u8(tx_buffer, current_header.version_minor);
                                    tx_buffer.write(b'.');
                                    queue_dec_u8(tx_buffer, current_header.version_patch);
                                    queue_string(tx_buffer, ")\r\n");
                                    
                                    if !header.is_newer_than(current_header) {
                                        queue_string(tx_buffer, "\r\nNew firmware is not newer than current version!\r\n");
                                        tx_buffer.write(Control::Nak.into_u8());
                                        
                                        CURRENT_ERROR.store(ERROR_NOT_NEWER_VERSION, Ordering::SeqCst);
                                        XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
                                        return true;
                                    }
                                } else {
                                    queue_string(tx_buffer, "  No existing firmware detected or different type\r\n");
                                }
                                
                                // Erase the target flash sector
                                queue_string(tx_buffer, "\r\nErasing flash sector at address 0x");
                                queue_hex_u32(tx_buffer, CURRENT_ADDR);
                                queue_string(tx_buffer, "...\r\n");
                                
                                let erased_size = flash::erase_sector(p, CURRENT_ADDR);
                                if erased_size == 0 {
                                    queue_string(tx_buffer, "Flash erase failed!\r\n");
                                    tx_buffer.write(Control::Nak.into_u8());
                                    
                                    CURRENT_ERROR.store(ERROR_FLASH_ERASE_FAILURE, Ordering::SeqCst);
                                    XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
                                    return true;
                                }
                                
                                queue_string(tx_buffer, "Flash sector erased successfully (");
                                queue_dec_u32(tx_buffer, erased_size);
                                queue_string(tx_buffer, " bytes)\r\n");
                                
                                HEADER_VALIDATED = true;
                            }
                            
                            // Write the packet data to flash
                            queue_string(tx_buffer, "\r\nWriting data to flash at 0x");
                            queue_hex_u32(tx_buffer, CURRENT_ADDR);
                            queue_string(tx_buffer, "...\r\n");
                            
                            let packet_data = packet.data().inner();
                            let result = flash::write(p, packet_data, CURRENT_ADDR);
                            if result != 0 {
                                queue_string(tx_buffer, "Flash write failed! Error code: ");
                                queue_dec_u8(tx_buffer, result);
                                queue_string(tx_buffer, "\r\n");
                                
                                tx_buffer.write(Control::Nak.into_u8());
                                
                                CURRENT_ERROR.store(ERROR_FLASH_WRITE_FAILURE, Ordering::SeqCst);
                                XMODEM_RECEIVE_STATE.store(XMODEM_STATE_ERROR, Ordering::SeqCst);
                                return true;
                            }
                            
                            queue_string(tx_buffer, "Flash write successful\r\n");
                            
                            // Update address for next packet
                            CURRENT_ADDR += OneKData::LEN as u32;
                            
                            // Acknowledge the packet
                            tx_buffer.write(Control::Ack.into_u8());
                            queue_string(tx_buffer, "Sending ACK for packet sequence ");
                            queue_dec_u8(tx_buffer, SEQUENCE);
                            queue_string(tx_buffer, "\r\n");
                            
                            // Increment sequence number for next packet
                            SEQUENCE = SEQUENCE.wrapping_add(1);
                            
                            // Clear first packet flag after successful processing
                            if FIRST_PACKET {
                                FIRST_PACKET = false;
                            }
                            
                            // Reset buffer index for next packet
                            RECEIVE_INDEX = 0;
                        },
                        Err(err) => {
                            // Log detailed error information
                            queue_string(tx_buffer, "\r\nPacket validation error: ");
                            match err {
                                rmodem::Error::InvalidControl(val) => {
                                    queue_string(tx_buffer, "Invalid control byte: 0x");
                                    queue_hex_byte(tx_buffer, val);
                                },
                                rmodem::Error::InvalidStart((have, exp)) => {
                                    queue_string(tx_buffer, "Invalid start byte, have: 0x");
                                    queue_hex_byte(tx_buffer, have);
                                    queue_string(tx_buffer, " expected: 0x");
                                    queue_hex_byte(tx_buffer, exp);
                                },
                                rmodem::Error::InvalidSequence((have, exp)) => {
                                    queue_string(tx_buffer, "Invalid sequence, have: ");
                                    queue_dec_u8(tx_buffer, have);
                                    queue_string(tx_buffer, " expected: ");
                                    queue_dec_u8(tx_buffer, exp);
                                },
                                rmodem::Error::InvalidComplementSequence((have, exp)) => {
                                    queue_string(tx_buffer, "Invalid complement, have: 0x");
                                    queue_hex_byte(tx_buffer, have);
                                    queue_string(tx_buffer, " expected: 0x");
                                    queue_hex_byte(tx_buffer, exp);
                                },
                                rmodem::Error::InvalidCrc16((have, exp)) => {
                                    queue_string(tx_buffer, "Invalid CRC, have: 0x");
                                    queue_hex_u16(tx_buffer, have);
                                    queue_string(tx_buffer, " expected: 0x");
                                    queue_hex_u16(tx_buffer, exp);
                                },
                                _ => {
                                    queue_string(tx_buffer, "Other validation error");
                                }
                            }
                            queue_string(tx_buffer, "\r\n");
                            
                            // Print hex dump of the first few bytes for debugging
                            queue_string(tx_buffer, "First bytes: ");
                            for i in 0..8 {
                                queue_hex_byte(tx_buffer, RECEIVE_BUFFER[i]);
                                tx_buffer.write(b' ');
                            }
                            queue_string(tx_buffer, "...\r\n");
                            
                            // Invalid packet - send NAK
                            queue_string(tx_buffer, "Sending NAK due to invalid packet\r\n");
                            tx_buffer.write(Control::Nak.into_u8());
                            
                            // Reset buffer index to receive another packet
                            RECEIVE_INDEX = 0;
                        }
                    }
                }
            }
        },
        XMODEM_STATE_COMPLETE => {
            // Update completed successfully
            queue_string(tx_buffer, "\r\n=====================================\r\n");
            queue_string(tx_buffer, "Firmware update completed successfully!\r\n");
            queue_string(tx_buffer, "=====================================\r\n");
            
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
            queue_string(tx_buffer, "\r\n=====================================\r\n");
            queue_string(tx_buffer, "Firmware update failed: ");
            queue_string(tx_buffer, error_message);
            queue_string(tx_buffer, "\r\n=====================================\r\n");
            
            // Reset flags and state
            UPDATE_IN_PROGRESS.store(false, Ordering::SeqCst);
            XMODEM_RECEIVE_STATE.store(XMODEM_STATE_IDLE, Ordering::SeqCst);
            
            // Return true to indicate state machine is done
            return true;
        },
        _ => {
            // Invalid state, reset
            queue_string(tx_buffer, "\r\nInvalid XMODEM state, resetting\r\n");
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