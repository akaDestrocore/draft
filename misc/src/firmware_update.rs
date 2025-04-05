use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use stm32f4 as pac;

use crate::{
    flash,
    image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER},
    ring_buffer::RingBuffer,
    systick,
};

use rmodem::{Control, Sequence, XmodemData, XmodemCrcPacket};

// State machine states
#[derive(Clone, Copy, PartialEq)]
pub enum XmodemState {
    Idle,          // Not receiving data
    Init,          // Initializing data reception
    Receiving,     // Receiving data
    Complete,      // Transfer complete
    Error          // Error occurred
}

// Global state variables for XMODEM
static XMODEM_STATE: AtomicU8 = AtomicU8::new(0); // Maps to XmodemState
static SEQUENCE: AtomicU8 = AtomicU8::new(1);
static FIRST_PACKET: AtomicBool = AtomicBool::new(true);
static TARGET_APP: AtomicBool = AtomicBool::new(true);
static UPDATE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

static mut CURRENT_ADDRESS: u32 = 0;
static mut LAST_C_TIME: u32 = 0;
static mut C_SENT_COUNT: u8 = 0;

// Buffer for packet reception
static mut PARTIAL_PACKET: [u8; 1029] = [0; 1029]; // Increased to handle 1K packets
static mut BUFFER_INDEX: usize = 0;

// Get current XMODEM state
pub fn get_state() -> XmodemState {
    match XMODEM_STATE.load(Ordering::Relaxed) {
        0 => XmodemState::Idle,
        1 => XmodemState::Init,
        2 => XmodemState::Receiving,
        3 => XmodemState::Complete,
        4 => XmodemState::Error,
        _ => XmodemState::Idle,
    }
}

// Set XMODEM state
pub fn set_state(state: XmodemState) {
    let value = match state {
        XmodemState::Idle => 0,
        XmodemState::Init => 1,
        XmodemState::Receiving => 2,
        XmodemState::Complete => 3,
        XmodemState::Error => 4,
    };
    XMODEM_STATE.store(value, Ordering::Relaxed);
}

// Check if update is in progress
pub fn is_update_in_progress() -> bool {
    UPDATE_IN_PROGRESS.load(Ordering::Relaxed)
}

// Function to queue a string to the TX buffer
pub fn queue_string(tx_buffer: &RingBuffer, s: &str) {
    for byte in s.bytes() {
        tx_buffer.write(byte);
    }
}

// Convert a u8 value to ASCII digits and send to buffer
fn write_u8_decimal(tx_buffer: &RingBuffer, value: u8) {
    if value >= 100 {
        tx_buffer.write(b'0' + (value / 100));
        tx_buffer.write(b'0' + ((value / 10) % 10));
        tx_buffer.write(b'0' + (value % 10));
    } else if value >= 10 {
        tx_buffer.write(b'0' + (value / 10));
        tx_buffer.write(b'0' + (value % 10));
    } else {
        tx_buffer.write(b'0' + value);
    }
}

// Convert a u32 value to hex string and send to buffer
fn write_u32_hex(tx_buffer: &RingBuffer, value: u32) {
    static HEX_DIGITS: [u8; 16] = *b"0123456789ABCDEF";
    
    tx_buffer.write(HEX_DIGITS[((value >> 28) & 0xF) as usize]);
    tx_buffer.write(HEX_DIGITS[((value >> 24) & 0xF) as usize]);
    tx_buffer.write(HEX_DIGITS[((value >> 20) & 0xF) as usize]);
    tx_buffer.write(HEX_DIGITS[((value >> 16) & 0xF) as usize]);
    tx_buffer.write(HEX_DIGITS[((value >> 12) & 0xF) as usize]);
    tx_buffer.write(HEX_DIGITS[((value >> 8) & 0xF) as usize]);
    tx_buffer.write(HEX_DIGITS[((value >> 4) & 0xF) as usize]);
    tx_buffer.write(HEX_DIGITS[(value & 0xF) as usize]);
}

// Helper function to start the firmware update process
pub fn start_update(tx_buffer: &RingBuffer, target_addr: u32, is_app_update: bool) {
    // Reset state
    set_state(XmodemState::Init);
    SEQUENCE.store(1, Ordering::Relaxed);
    FIRST_PACKET.store(true, Ordering::Relaxed);
    TARGET_APP.store(is_app_update, Ordering::Relaxed);
    UPDATE_IN_PROGRESS.store(true, Ordering::Relaxed);
    
    unsafe {
        CURRENT_ADDRESS = target_addr;
        BUFFER_INDEX = 0;
        PARTIAL_PACKET = [0; 1029];
        LAST_C_TIME = systick::get_tick_ms();
        C_SENT_COUNT = 0; // Reset the counter
    }
    
    // Only send message before starting the protocol
    queue_string(tx_buffer, "\r\nStarting firmware update. Send file using XMODEM-CRC protocol...\r\n");
    
    // Send 'C' to request XMODEM-CRC transfer - first C character
    tx_buffer.write(Control::Idle.into_u8());
}

// Process XMODEM state machine using rmodem library
pub fn process_xmodem(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) -> bool {
    let state = get_state();
    if state == XmodemState::Idle {
        return false;
    }
    
    // Periodically send 'C' in Init state - more aggressively
    if state == XmodemState::Init {
        let current_time = systick::get_tick_ms();
        
        unsafe {
            // Send 'C' more frequently (every 300ms) for the first 10 attempts
            if current_time.wrapping_sub(LAST_C_TIME) >= 300 {
                tx_buffer.write(Control::Idle.into_u8());
                LAST_C_TIME = current_time;
                C_SENT_COUNT += 1;
                
                // Only move to Receiving state after a certain number of attempts
                if C_SENT_COUNT >= 10 {
                    // Move to Receiving state after sending enough 'C' characters
                    set_state(XmodemState::Receiving);
                }
            }
        }
    }
    
    // Process available bytes in receiving state
    if state == XmodemState::Receiving || state == XmodemState::Init {
        while let Some(byte) = rx_buffer.read() {
            match process_byte(p, tx_buffer, byte) {
                true => return true, // Transfer completed or error
                false => continue,   // Continue processing
            }
        }
    }
    
    false
}

// Process a single byte in the XMODEM protocol
fn process_byte(p: &pac::Peripherals, tx_buffer: &RingBuffer, byte: u8) -> bool {
    unsafe {
        // Check for EOT (End of Transmission)
        if BUFFER_INDEX == 0 && byte == Control::Eot.into_u8() {
            tx_buffer.write(Control::Ack.into_u8());
            
            // Only send a message after completing the protocol
            queue_string(tx_buffer, "\r\nFirmware update completed successfully!\r\n");
            
            set_state(XmodemState::Complete);
            UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
            return true;
        }
        
        // Add byte to buffer
        PARTIAL_PACKET[BUFFER_INDEX] = byte;
        BUFFER_INDEX += 1;
        
        // Check if we have a complete packet (handle both 128 byte and 1K packets)
        let packet_size = if BUFFER_INDEX > 0 && PARTIAL_PACKET[0] == Control::Stx.into_u8() {
            1029 // STX + SEQ + COMPLEMENTED_SEQ + DATA[1024] + CRC[2]
        } else {
            133  // SOH + SEQ + COMPLEMENTED_SEQ + DATA[128] + CRC[2]
        };
        
        if BUFFER_INDEX == packet_size {
            BUFFER_INDEX = 0; // Reset for next packet
            
            // Try to parse packet using rmodem
            match rmodem::XmodemCrcPacket::try_from(&PARTIAL_PACKET[..]) {
                Ok(packet) => {
                    // Verify sequence number
                    let expected_seq = SEQUENCE.load(Ordering::Relaxed);
                    if packet.sequence().into_u8() != expected_seq {
                        // Just send NAK without debug text
                        tx_buffer.write(Control::Nak.into_u8());
                        return false;
                    }
                    
                    // For first packet, verify header
                    if FIRST_PACKET.load(Ordering::Relaxed) {
                        let valid = validate_header(packet.data().inner());
                        if !valid {
                            // Cancel on invalid header - no debug output
                            tx_buffer.write(Control::Can.into_u8());
                            tx_buffer.write(Control::Can.into_u8());
                            tx_buffer.write(Control::Can.into_u8());
                            set_state(XmodemState::Error);
                            UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
                            return true;
                        }
                        
                        // Erase flash sector for first packet - no debug output
                        flash::erase_sector(p, CURRENT_ADDRESS);
                        
                        FIRST_PACKET.store(false, Ordering::Relaxed);
                    }
                    
                    // Write data to flash
                    let result = flash::write(p, packet.data().inner(), CURRENT_ADDRESS);
                    if result != 0 {
                        // Just cancel without debug output
                        tx_buffer.write(Control::Can.into_u8());
                        tx_buffer.write(Control::Can.into_u8());
                        tx_buffer.write(Control::Can.into_u8());
                        set_state(XmodemState::Error);
                        UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
                        return true;
                    }
                    
                    // Update address and acknowledge
                    CURRENT_ADDRESS += XmodemData::LEN as u32;
                    tx_buffer.write(Control::Ack.into_u8());
                    
                    // Increment sequence for next packet
                    let next_seq = SEQUENCE.load(Ordering::Relaxed).wrapping_add(1);
                    SEQUENCE.store(next_seq, Ordering::Relaxed);
                },
                Err(_) => {
                    // Send NAK for invalid packet - no debug output
                    tx_buffer.write(Control::Nak.into_u8());
                }
            }
        } else if BUFFER_INDEX > packet_size {
            // Buffer overflow - reset without debug output
            BUFFER_INDEX = 0;
            tx_buffer.write(Control::Nak.into_u8());
        }
    }
    
    false
}

// Function to validate the image header - removed all debug outputs
fn validate_header(data: &[u8]) -> bool {
    // Cast the first bytes to an ImageHeader
    let header = unsafe { &*(data.as_ptr() as *const ImageHeader) };
    
    // Check if target is application or updater
    let (expected_magic, expected_type) = if TARGET_APP.load(Ordering::Relaxed) {
        (IMAGE_MAGIC_APP, IMAGE_TYPE_APP)
    } else {
        (IMAGE_MAGIC_UPDATER, IMAGE_TYPE_UPDATER)
    };
    
    // Check magic number and type
    if header.image_magic != expected_magic || header.image_type != expected_type {
        return false;
    }
    
    // Check version if there's existing firmware
    unsafe {
        let dest_addr = CURRENT_ADDRESS;
        if *(dest_addr as *const u32) != 0xFFFFFFFF {
            let existing_header = &*(dest_addr as *const ImageHeader);
            
            if expected_magic == existing_header.image_magic {
                if !header.is_newer_than(existing_header) {
                    return false;
                }
            }
        }
    }
    
    true
}