use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use stm32f4 as pac;

use crate::{
    flash,
    image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER},
    ring_buffer::RingBuffer,
    systick,
};

use rmodem::{Control, XmodemCrcPacket};

// State machine states
#[derive(Clone, Copy, PartialEq)]
pub enum XmodemState {
    Idle,          // Not receiving data
    Waiting,       // Waiting for first packet
    Receiving,     // Receiving data
    Complete,      // Transfer complete
    Error          // Error occurred
}

// Global state variables for XMODEM
static STATE: AtomicU8 = AtomicU8::new(0); // 0=Idle, 1=Waiting, 2=Receiving, 3=Complete, 4=Error
static SEQUENCE: AtomicU8 = AtomicU8::new(1);
static FIRST_PACKET: AtomicBool = AtomicBool::new(true);
static TARGET_APP: AtomicBool = AtomicBool::new(true);
static UPDATE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static ERROR_CODE: AtomicU8 = AtomicU8::new(0);

// Diagnostic counters
static BYTES_RECEIVED: AtomicU8 = AtomicU8::new(0);
static PACKETS_RECEIVED: AtomicU8 = AtomicU8::new(0);

// Buffer for XMODEM packet
static mut BUFFER: [u8; 133] = [0; 133]; // Simple buffer for 128-byte packets
static mut BUFFER_INDEX: usize = 0;
static mut CURRENT_ADDRESS: u32 = 0;
static mut LAST_C_TIME: u32 = 0;

// Get current XMODEM state
pub fn get_state() -> XmodemState {
    match STATE.load(Ordering::Relaxed) {
        0 => XmodemState::Idle,
        1 => XmodemState::Waiting,
        2 => XmodemState::Receiving,
        3 => XmodemState::Complete,
        _ => XmodemState::Error,
    }
}

// Set XMODEM state
pub fn set_state(state: XmodemState) {
    let value = match state {
        XmodemState::Idle => 0,
        XmodemState::Waiting => 1,
        XmodemState::Receiving => 2,
        XmodemState::Complete => 3,
        XmodemState::Error => 4,
    };
    STATE.store(value, Ordering::Relaxed);
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

// Simple helper to write a u8 as decimal string
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

// Helper function to start the firmware update process
pub fn start_update(tx_buffer: &RingBuffer, target_addr: u32, is_app_update: bool) {
    // Reset state
    set_state(XmodemState::Waiting);
    SEQUENCE.store(1, Ordering::Relaxed);
    FIRST_PACKET.store(true, Ordering::Relaxed);
    TARGET_APP.store(is_app_update, Ordering::Relaxed);
    UPDATE_IN_PROGRESS.store(true, Ordering::Relaxed);
    ERROR_CODE.store(0, Ordering::Relaxed);
    BYTES_RECEIVED.store(0, Ordering::Relaxed);
    PACKETS_RECEIVED.store(0, Ordering::Relaxed);
    
    unsafe {
        CURRENT_ADDRESS = target_addr;
        BUFFER_INDEX = 0;
        LAST_C_TIME = systick::get_tick_ms();
    }
    
    // Send message
    queue_string(tx_buffer, "\r\n--- DIAGNOSTIC MODE ---\r\n");
    queue_string(tx_buffer, "Starting firmware update. Send file using XMODEM-CRC...\r\n");
    
    // Send 'C' for CRC mode
    tx_buffer.write(Control::Idle.into_u8());
}

// Process XMODEM protocol
pub fn process_xmodem(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) -> bool {
    // Clear all LEDs
    p.gpiod.bsrr().write(|w| w
        .br12().set_bit()
        .br13().set_bit()
        .br14().set_bit()
        .br15().set_bit()
    );
    
    // Update LEDs based on state (not using a separate function to avoid issues)
    match get_state() {
        XmodemState::Idle => {
            p.gpiod.bsrr().write(|w| w.bs12().set_bit()); // Red LED
            return false; // Early return if idle
        },
        XmodemState::Waiting => {
            p.gpiod.bsrr().write(|w| w.bs13().set_bit()); // Orange LED
        },
        XmodemState::Receiving => {
            p.gpiod.bsrr().write(|w| w.bs14().set_bit()); // Green LED
            
            // Blink LED 15 based on packet count for visual feedback
            let packets = PACKETS_RECEIVED.load(Ordering::Relaxed);
            if (packets & 0x01) != 0 {
                p.gpiod.bsrr().write(|w| w.bs15().set_bit());
            }
        },
        XmodemState::Complete => {
            p.gpiod.bsrr().write(|w| w.bs15().set_bit()); // Blue LED
        },
        XmodemState::Error => {
            // Blink error code
            let time = systick::get_tick_ms();
            if (time % 500) < 250 {
                p.gpiod.bsrr().write(|w| w.bs12().set_bit()); // Red = error
                
                // Also show the error code
                let error = ERROR_CODE.load(Ordering::Relaxed);
                if (error & 0x02) != 0 { p.gpiod.bsrr().write(|w| w.bs13().set_bit()); }
                if (error & 0x04) != 0 { p.gpiod.bsrr().write(|w| w.bs14().set_bit()); }
                if (error & 0x08) != 0 { p.gpiod.bsrr().write(|w| w.bs15().set_bit()); }
            }
        },
    }
    
    // Periodically send 'C' while waiting for first packet
    if get_state() == XmodemState::Waiting {
        let current_time = systick::get_tick_ms();
        unsafe {
            if current_time.wrapping_sub(LAST_C_TIME) >= 2000 {
                // Send 'C' every 2 seconds
                tx_buffer.write(Control::Idle.into_u8());
                LAST_C_TIME = current_time;
                
                // Toggle orange/blue LEDs as a "breathing" effect
                let cycle = current_time / 2000;
                if (cycle & 0x01) != 0 {
                    p.gpiod.bsrr().write(|w| w.bs15().set_bit());
                }
            }
        }
    }
    
    // Process any received bytes
    while let Some(byte) = rx_buffer.read() {
        // Update bytes received counter
        let count = BYTES_RECEIVED.load(Ordering::Relaxed);
        BYTES_RECEIVED.store(count.wrapping_add(1), Ordering::Relaxed);
        
        // As soon as we get a byte, transition to receiving state
        if get_state() == XmodemState::Waiting {
            // Show transition in UART log
            queue_string(tx_buffer, "First byte received: 0x");
            
            // Convert byte to hex
            const HEX_CHARS: &[u8] = b"0123456789ABCDEF";
            tx_buffer.write(HEX_CHARS[(byte >> 4) as usize]);
            tx_buffer.write(HEX_CHARS[(byte & 0x0F) as usize]);
            
            if byte == Control::Soh.into_u8() {
                queue_string(tx_buffer, " (SOH)\r\n");
            } else if byte == Control::Stx.into_u8() {
                queue_string(tx_buffer, " (STX)\r\n");
            } else if byte == Control::Eot.into_u8() {
                queue_string(tx_buffer, " (EOT)\r\n");
            } else {
                queue_string(tx_buffer, "\r\n");
            }
            
            set_state(XmodemState::Receiving);
        }
        
        // Process the byte
        if process_byte(p, tx_buffer, byte) {
            return true;
        }
    }
    
    false
}

// Process a single byte in the XMODEM protocol
fn process_byte(p: &pac::Peripherals, tx_buffer: &RingBuffer, byte: u8) -> bool {
    unsafe {
        // Check for EOT (End of Transmission)
        if BUFFER_INDEX == 0 && byte == Control::Eot.into_u8() {
            // Send ACK
            tx_buffer.write(Control::Ack.into_u8());
            
            // Complete the transfer
            set_state(XmodemState::Complete);
            UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
            
            // Show stats
            queue_string(tx_buffer, "\r\nFirmware update completed!\r\n");
            queue_string(tx_buffer, "Bytes received: ");
            write_u8_decimal(tx_buffer, BYTES_RECEIVED.load(Ordering::Relaxed));
            queue_string(tx_buffer, "\r\nPackets received: ");
            write_u8_decimal(tx_buffer, PACKETS_RECEIVED.load(Ordering::Relaxed));
            queue_string(tx_buffer, "\r\n");
            
            return true;
        }
        
        // Add byte to buffer
        BUFFER[BUFFER_INDEX] = byte;
        BUFFER_INDEX += 1;
        
        // For diagnostic purposes, accept only SOH packets
        if BUFFER_INDEX == 1 && byte != Control::Soh.into_u8() {
            // Not a standard XMODEM packet, reset buffer
            BUFFER_INDEX = 0;
            ERROR_CODE.store(1, Ordering::Relaxed);
            tx_buffer.write(Control::Nak.into_u8());
            return false;
        }
        
        // Check if we have a complete packet
        if BUFFER_INDEX == 132 {
            // We have a complete standard XMODEM packet
            match process_packet(p, tx_buffer) {
                true => {
                    // Packet processed successfully
                    // Get current count and increment
                    let count = PACKETS_RECEIVED.load(Ordering::Relaxed);
                    PACKETS_RECEIVED.store(count.wrapping_add(1), Ordering::Relaxed);
                },
                false => {
                    // Error processing packet
                }
            }
            
            // Reset buffer for next packet
            BUFFER_INDEX = 0;
        }
        
        // Buffer overflow check
        if BUFFER_INDEX >= BUFFER.len() {
            BUFFER_INDEX = 0;
            ERROR_CODE.store(2, Ordering::Relaxed);
            tx_buffer.write(Control::Nak.into_u8());
        }
    }
    
    false
}

// Process a complete packet
fn process_packet(p: &pac::Peripherals, tx_buffer: &RingBuffer) -> bool {
    unsafe {
        // Log basic packet info
        queue_string(tx_buffer, "Packet received: seq=");
        write_u8_decimal(tx_buffer, BUFFER[1]);
        queue_string(tx_buffer, "\r\n");
        
        // Simple validation of packet format
        let seq = BUFFER[1];
        let seq_complement = BUFFER[2];
        
        // Sequence number should be complemented correctly
        if seq + seq_complement != 255 {
            queue_string(tx_buffer, "Invalid sequence complement!\r\n");
            ERROR_CODE.store(3, Ordering::Relaxed);
            tx_buffer.write(Control::Nak.into_u8());
            return false;
        }
        
        // Check sequence number
        let expected_seq = SEQUENCE.load(Ordering::Relaxed);
        if seq != expected_seq {
            queue_string(tx_buffer, "Unexpected sequence number! Got ");
            write_u8_decimal(tx_buffer, seq);
            queue_string(tx_buffer, ", expected ");
            write_u8_decimal(tx_buffer, expected_seq);
            queue_string(tx_buffer, "\r\n");
            
            ERROR_CODE.store(4, Ordering::Relaxed);
            tx_buffer.write(Control::Nak.into_u8());
            return false;
        }
        
        // Get a few bytes for logging
        queue_string(tx_buffer, "Data begins with: ");
        for i in 0..min(4, 128) {
            // Convert byte to hex
            const HEX_CHARS: &[u8] = b"0123456789ABCDEF";
            let b = BUFFER[3 + i];
            tx_buffer.write(HEX_CHARS[(b >> 4) as usize]);
            tx_buffer.write(HEX_CHARS[(b & 0x0F) as usize]);
            tx_buffer.write(b' ');
        }
        queue_string(tx_buffer, "...\r\n");
        
        // Special handling for first packet
        if FIRST_PACKET.load(Ordering::Relaxed) {
            // Print the header
            let header = unsafe { &*(BUFFER.as_ptr().add(3) as *const ImageHeader) };
            
            queue_string(tx_buffer, "Image magic: 0x");
            // Convert u32 to hex
            let magic = header.image_magic;
            for shift in [24, 16, 8, 0] {
                let nibble = ((magic >> shift) & 0xF) as usize;
                const HEX_CHARS: &[u8] = b"0123456789ABCDEF";
                tx_buffer.write(HEX_CHARS[nibble]);
            }
            queue_string(tx_buffer, "\r\n");
            
            queue_string(tx_buffer, "Image type: ");
            write_u8_decimal(tx_buffer, header.image_type);
            queue_string(tx_buffer, "\r\n");
            
            // Validate header (no flash operations)
            let (expected_magic, expected_type) = if TARGET_APP.load(Ordering::Relaxed) {
                (IMAGE_MAGIC_APP, IMAGE_TYPE_APP)
            } else {
                (IMAGE_MAGIC_UPDATER, IMAGE_TYPE_UPDATER)
            };
            
            if header.image_magic != expected_magic {
                queue_string(tx_buffer, "Invalid image magic!\r\n");
                ERROR_CODE.store(5, Ordering::Relaxed);
                tx_buffer.write(Control::Can.into_u8());
                tx_buffer.write(Control::Can.into_u8());
                tx_buffer.write(Control::Can.into_u8());
                set_state(XmodemState::Error);
                UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
                return false;
            }
            
            if header.image_type != expected_type {
                queue_string(tx_buffer, "Invalid image type!\r\n");
                ERROR_CODE.store(6, Ordering::Relaxed);
                tx_buffer.write(Control::Can.into_u8());
                tx_buffer.write(Control::Can.into_u8());
                tx_buffer.write(Control::Can.into_u8());
                set_state(XmodemState::Error);
                UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
                return false;
            }
            
            // In diagnostic mode - no flash erase, just log
            queue_string(tx_buffer, "[DIAGNOSTIC] Flash erase would happen here\r\n");
            
            // No longer first packet
            FIRST_PACKET.store(false, Ordering::Relaxed);
        }
        
        // In diagnostic mode - no flash write, just log
        queue_string(tx_buffer, "[DIAGNOSTIC] Flash write would happen here\r\n");
        
        // ACK the packet
        tx_buffer.write(Control::Ack.into_u8());
        
        // Next sequence
        let next_seq = (expected_seq + 1) & 0xFF;
        SEQUENCE.store(next_seq, Ordering::Relaxed);
    }
    
    true
}

// Simple min function since we don't have std
fn min(a: usize, b: usize) -> usize {
    if a < b { a } else { b }
}