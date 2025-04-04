#![no_std]

use core::{array, slice::from_raw_parts};
use rmodem::{
    Control,
    Error as RmodemError,
    OneKData,
    Sequence,
    XmodemOneKPacket,
    XmodemPacket
};
use crate::{
    systick,
    flash::{self, read, write},
    image::{ImageHeader}
};
use crate::ring_buffer::RingBuffer;

// Define custom error types for XMODEM communication
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XmodemError {
    Crc,              // CRC error
    PacketNumber,     // Packet number error
    Canceled,         // Transfer was canceled
    InvalidMagic,     // Invalid magic number in firmware
    OlderVersion,     // New firmware is not newer
    Timeout,          // Timeout during transfer
    FlashError,       // Flash writing error
    InvalidPacket,    // Invalid packet received
    EOT,              // Unexpected EOT
}

// Constants for XMODEM protocol
const SOH: u8 = 0x01;  // Start of header for 128-byte packet
const STX: u8 = 0x02;  // Start of header for 1024-byte packet
const EOT: u8 = 0x04;  // End of transmission
const ACK: u8 = 0x06;  // Acknowledge
const NAK: u8 = 0x15;  // Negative acknowledge
const CAN: u8 = 0x18;  // Cancel transmission
const CTRL_Z: u8 = 0x1A; // EOF marker
const IDLE: u8 = 0x43;  // ASCII 'C' character to request XMODEM-1K

const RECV_TIMEOUT_MS: u32 = 10000; // 10 seconds
const IDLE_SEND_INTERVAL_MS: u32 = 1000; // Send 'C' every second

// Convert rmodem error to our custom error type
fn map_error(err: RmodemError) -> XmodemError {
    match err {
        RmodemError::InvalidChecksum(..) => XmodemError::Crc,
        RmodemError::InvalidCrc16(..) => XmodemError::Crc,
        RmodemError::InvalidSequence(..) => XmodemError::PacketNumber,
        RmodemError::InvalidComplementSequence(..) => XmodemError::PacketNumber,
        _ => XmodemError::InvalidPacket,
    }
}

// Receive firmware via XMODEM-1K protocol
pub fn receive_firmware(
    rx_buffer: &RingBuffer,
    tx_buffer: &RingBuffer,
    target_addr: u32,
    expected_magic: u32,
    max_size: u32,
    mut transmit_fn: impl FnMut(&RingBuffer),
) -> Result<u32, XmodemError> {
    let mut sequence_num: u8 = 1;
    let mut received_bytes: u32 = 0;
    let mut buffer: [u8; 1024] = [0; 1024];
    
    // Send initial 'C' to start transfer (for XMODEM-1K)
    tx_buffer.write(IDLE);
    transmit_fn(tx_buffer);
    
    let mut header_buffer: [u8; 512] = [0; 512];
    
    // Get peripherals to access flash
    let peripherals = unsafe { stm32f4::Peripherals::steal() };
    
    // Temporary buffer for flash writes
    let mut flash_buffer: [u8; 1024] = [0; 1024];
    
    // Main receive loop
    let start_time = systick::get_tick_ms();
    let mut last_idle_time = start_time;
    
    loop {
        // Check for overall timeout
        if systick::wait_ms(start_time, RECV_TIMEOUT_MS) {
            return Err(XmodemError::Timeout);
        }
        
        // Send 'C' character periodically until we receive data
        let current_time = systick::get_tick_ms();
        if rx_buffer.is_empty() && systick::wait_ms(last_idle_time, IDLE_SEND_INTERVAL_MS) {
            tx_buffer.write(IDLE);
            transmit_fn(tx_buffer);
            last_idle_time = current_time;
            continue;
        }
        
        // Wait for byte in rx_buffer
        if rx_buffer.is_empty() {
            continue;
        }
        
        // Get the byte
        let byte = rx_buffer.read().unwrap_or(0);
        
        match byte {
            SOH => {
                // Standard 128-byte XMODEM packet - not supported
                tx_buffer.write(NAK);
                transmit_fn(tx_buffer);
            },
            
            STX => {
                // XMODEM-1K packet (1024 bytes)
                
                // Read the rest of the packet header (seq, ~seq)
                let header_start = systick::get_tick_ms();
                let mut header_bytes: [u8; 3] = [STX, 0, 0];
                
                // Already read STX
                let mut idx = 1;
                while idx < 3 {
                    if systick::wait_ms(header_start, RECV_TIMEOUT_MS) {
                        return Err(XmodemError::Timeout);
                    }
                    
                    if !rx_buffer.is_empty() {
                        header_bytes[idx] = rx_buffer.read().unwrap_or(0);
                        idx += 1;
                    }
                }
                
                // Read the 1024 bytes of data and 2 bytes of CRC
                let data_start = systick::get_tick_ms();
                let mut packet_bytes = [0u8; XmodemOneKPacket::LEN];
                packet_bytes[0..3].copy_from_slice(&header_bytes);
                
                let mut idx = 3;
                while idx < XmodemOneKPacket::LEN {
                    if systick::wait_ms(data_start, RECV_TIMEOUT_MS) {
                        return Err(XmodemError::Timeout);
                    }
                    
                    if !rx_buffer.is_empty() {
                        packet_bytes[idx] = rx_buffer.read().unwrap_or(0);
                        idx += 1;
                    }
                }
                
                // Try to parse the packet
                match XmodemOneKPacket::try_from(packet_bytes.as_ref()) {
                    Ok(packet) => {
                        // Valid packet
                        
                        // Check sequence number
                        let packet_seq = packet.sequence().into_u8();
                        if packet_seq != sequence_num {
                            // Wrong sequence number
                            tx_buffer.write(NAK);
                            transmit_fn(tx_buffer);
                            continue;
                        }
                        
                        // Process the data
                        let data = packet.data().as_ref();
                        
                        // First packet contains the header
                        if sequence_num == 1 {
                            // Copy header to buffer
                            for i in 0..core::mem::size_of::<ImageHeader>() {
                                if i < data.len() {
                                    header_buffer[i] = data[i];
                                }
                            }
                            
                            // Validate the header
                            let header = unsafe {
                                &*(header_buffer.as_ptr() as *const ImageHeader)
                            };
                            
                            // Check magic number
                            if header.image_magic != expected_magic {
                                return Err(XmodemError::InvalidMagic);
                            }
                            
                            // Check if this is a newer version
                            let current_header = unsafe {
                                &*(target_addr as *const ImageHeader)
                            };
                            
                            // Only check version if there's an existing firmware
                            if current_header.image_magic == expected_magic {
                                if !header.is_newer_than(current_header) {
                                    return Err(XmodemError::OlderVersion);
                                }
                            }
                            
                            // Erase the sector
                            let erased_size: u32 = flash::erase_sector(&peripherals, target_addr);
                            if erased_size == 0 {
                                return Err(XmodemError::FlashError);
                            }
                        }
                        
                        // Calculate flash write address
                        let write_addr = target_addr + ((sequence_num - 1) as u32 * 1024);
                        
                        // Check if we'll exceed the max size
                        if received_bytes + 1024 > max_size {
                            return Err(XmodemError::FlashError);
                        }
                        
                        // Write data to flash
                        // Copy data to buffer
                        flash_buffer[..data.len()].copy_from_slice(data);
                        
                        let result = flash::write(&peripherals, &flash_buffer, write_addr);
                        if result != 0 {
                            return Err(XmodemError::FlashError);
                        }
                        
                        // Update received byte count
                        received_bytes += 1024;
                        
                        // Send ACK
                        tx_buffer.write(ACK);
                        transmit_fn(tx_buffer);
                        
                        // Increment sequence number (it wraps automatically at 255)
                        sequence_num = sequence_num.wrapping_add(1);
                    },
                    Err(e) => {
                        // Invalid packet
                        tx_buffer.write(NAK);
                        transmit_fn(tx_buffer);
                    }
                }
            },
            
            EOT => {
                // End of transmission
                tx_buffer.write(ACK);
                transmit_fn(tx_buffer);
                
                // Return total received bytes
                return Ok(received_bytes);
            },
            
            CAN => {
                // Canceled by sender
                return Err(XmodemError::Canceled);
            },
            
            _ => {
                // Unknown byte, send NAK
                tx_buffer.write(NAK);
                transmit_fn(tx_buffer);
            }
        }
    }
}