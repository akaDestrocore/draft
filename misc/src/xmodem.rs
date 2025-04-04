//! XMODEM-1K protocol implementation for firmware updates
//!
//! This module provides functionality for receiving firmware updates using the XMODEM-1K protocol.
//! It handles both 128-byte and 1K packet reception, CRC validation, and magic number/version validation.

#![no_std]

use crate::ring_buffer::RingBuffer;
use crate::flash;
use crate::systick;
use core::sync::atomic::{AtomicBool, Ordering};

/// XMODEM protocol constants
pub const SOH: u8 = 0x01; // Start of 128-byte data packet
pub const STX: u8 = 0x02; // Start of 1K data packet
pub const EOT: u8 = 0x04; // End of transmission
pub const ACK: u8 = 0x06; // Acknowledge
pub const NAK: u8 = 0x15; // Not acknowledge
pub const CAN: u8 = 0x18; // Cancel
pub const C: u8 = 0x43;   // ASCII 'C' - used to initiate XMODEM-CRC transfer

/// XMODEM packet sizes
pub const SMALL_PACKET_SIZE: usize = 128;     // Standard XMODEM packet data size
pub const LARGE_PACKET_SIZE: usize = 1024;    // XMODEM-1K packet data size
pub const PACKET_OVERHEAD: usize = 5;         // Header + packet# + ~packet# + CRC16(2 bytes)
pub const SMALL_TOTAL_SIZE: usize = SMALL_PACKET_SIZE + PACKET_OVERHEAD; // 133 bytes
pub const LARGE_TOTAL_SIZE: usize = LARGE_PACKET_SIZE + PACKET_OVERHEAD; // 1029 bytes

/// XMODEM state machine states
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XmodemState {
    WaitHeader,  // Waiting for SOH or STX byte
    WaitIndex1,  // Waiting for packet number
    WaitIndex2,  // Waiting for packet number complement
    ReadData,    // Reading data bytes
    WaitCrc,     // Waiting for CRC bytes
}

/// Error types that can occur during XMODEM transfer
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XmodemError {
    Crc,            // CRC mismatch
    PacketNumber,   // Packet number error
    Canceled,       // Transfer was canceled
    InvalidMagic,   // Invalid magic number in image header
    OlderVersion,   // New version is not newer than current version
    Timeout,        // Timeout during transfer
    FlashError,     // Error writing to flash
    InvalidPacket,  // Invalid packet structure
}

/// Calculate CRC16 for the given data
///
/// Implements the CCITT CRC-16 algorithm with polynomial 0x1021
pub fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0u16;
    
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    crc
}

/// Send a CAN signal to abort the transfer
pub fn send_cancel(tx_buffer: &mut RingBuffer) -> Result<(), XmodemError> {
    // Send CAN bytes (standard practice is to send multiple CAN bytes)
    tx_buffer.write(CAN);
    tx_buffer.write(CAN);
    tx_buffer.write(CAN);
    Ok(())
}

/// Validate the first packet of the firmware image
///
/// Checks if:
/// 1. The magic number matches the expected value
/// 2. The version is newer than any existing image
pub fn validate_first_packet(data: &[u8], expected_magic: u32, target_address: u32) -> Result<(), XmodemError> {
    // Extract magic number from the first 4 bytes
    let magic: u32 = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    
    // Check if magic number matches
    if magic != expected_magic {
        return Err(XmodemError::InvalidMagic);
    }
    
    // Extract version fields (version_major at offset 5, minor at 6, patch at 7)
    let new_version_major = data[5];
    let new_version_minor = data[6];
    let new_version_patch = data[7];
    
    // Check if there's already an image at target address
    if unsafe { *(target_address as *const u32) } != 0xFFFFFFFF {
        // Read current version
        let current_version_major = unsafe { *((target_address + 5) as *const u8) };
        let current_version_minor = unsafe { *((target_address + 6) as *const u8) };
        let current_version_patch = unsafe { *((target_address + 7) as *const u8) };
        
        // Compare versions (major first, then minor, then patch)
        if new_version_major < current_version_major {
            return Err(XmodemError::OlderVersion);
        } else if new_version_major == current_version_major {
            if new_version_minor < current_version_minor {
                return Err(XmodemError::OlderVersion);
            } else if new_version_minor == current_version_minor {
                if new_version_patch <= current_version_patch {
                    return Err(XmodemError::OlderVersion);
                }
            }
        }
    }
    
    Ok(())
}

/// Process a received XMODEM packet 
///
/// Handles both 128-byte (SOH) and 1K (STX) packets
pub fn process_packet(
    state: &mut XmodemState,
    packet_data: &[u8],
    data_buffer: &mut [u8; LARGE_PACKET_SIZE],
    current_packet_size: &mut usize,
    prev_index1: &mut u8,
    prev_index2: &mut u8
) -> Result<bool, XmodemError> {
    match *state {
        XmodemState::WaitHeader => {
            if packet_data[0] == SOH {
                *current_packet_size = SMALL_PACKET_SIZE;
                *state = XmodemState::WaitIndex1;
                Ok(false) // Continue processing
            } else if packet_data[0] == STX {
                *current_packet_size = LARGE_PACKET_SIZE;
                *state = XmodemState::WaitIndex1;
                Ok(false) // Continue processing
            } else if packet_data[0] == EOT {
                Ok(true) // Transfer complete
            } else {
                Err(XmodemError::InvalidPacket)
            }
        },
        XmodemState::WaitIndex1 => {
            // Check if packet number is sequential (or wrapped around)
            if packet_data[1] == (*prev_index1).wrapping_add(1) {
                *prev_index1 = packet_data[1];
                *state = XmodemState::WaitIndex2;
                Ok(false) // Continue processing
            } else {
                Err(XmodemError::PacketNumber)
            }
        },
        XmodemState::WaitIndex2 => {
            // Check if packet number complement is correct (~packet_number)
            if packet_data[2] == !(*prev_index1) {
                *prev_index2 = packet_data[2];
                *state = XmodemState::ReadData;
                Ok(false) // Continue processing
            } else {
                Err(XmodemError::PacketNumber)
            }
        },
        XmodemState::ReadData => {
            // Copy data portion of the packet
            for i in 0..*current_packet_size {
                data_buffer[i] = packet_data[i + 3]; // Skip header byte, index1, index2
            }
            *state = XmodemState::WaitCrc;
            Ok(false) // Continue processing
        },
        XmodemState::WaitCrc => {
            // Extract the CRC from the packet
            let received_crc: u16 = ((packet_data[*current_packet_size + 3] as u16) << 8) | 
                                (packet_data[*current_packet_size + 4] as u16);
            
            // Calculate CRC for the data portion
            let calculated_crc: u16 = calculate_crc16(&data_buffer[0..*current_packet_size]);
            
            // Verify CRC matches
            if received_crc != calculated_crc {
                Err(XmodemError::Crc)
            } else {
                // Reset state for next packet
                *state = XmodemState::WaitHeader;
                Ok(false) // Continue processing, CRC is valid
            }
        }
    }
}

/// Extract a complete XMODEM packet from the receive buffer
pub fn extract_packet(
    rx_buffer: &mut RingBuffer, 
    packet: &mut [u8], 
    is_first_byte: bool
) -> Result<usize, XmodemError> {
    // For the very first byte, we need to determine the packet type
    if is_first_byte {
        if rx_buffer.len() < 1 {
            return Err(XmodemError::Timeout);
        }
        
        let first_byte: u8 = match rx_buffer.read() {
            Some(byte) => byte,
            None => return Err(XmodemError::Timeout),
        };
        
        packet[0] = first_byte;
        
        if first_byte == EOT {
            return Ok(1); // End of transmission, only 1 byte needed
        } else if first_byte != SOH && first_byte != STX {
            return Err(XmodemError::InvalidPacket);
        }
        
        // Determine the packet size based on the first byte
        let expected_packet_size: usize = if first_byte == SOH {
            SMALL_TOTAL_SIZE
        } else {
            LARGE_TOTAL_SIZE
        };
        
        // Wait until we have enough data for the complete packet
        while rx_buffer.len() < expected_packet_size - 1 {
            // Wait for more data (handled by caller or in a timeout loop)
            return Err(XmodemError::Timeout);
        }
        
        // Read the rest of the packet
        for i in 1..expected_packet_size {
            if let Some(byte) = rx_buffer.read() {
                packet[i] = byte;
            } else {
                return Err(XmodemError::Timeout);
            }
        }
        
        Ok(expected_packet_size)
    } else {
        // For subsequent packets, we can directly check SOH or STX
        if rx_buffer.len() < 1 {
            return Err(XmodemError::Timeout);
        }
        
        // Peek at first byte to determine packet type
        let first_byte: u8 = match rx_buffer.peek() {
            Some(byte) => byte,
            None => return Err(XmodemError::Timeout),
        };
        
        if first_byte == EOT {
            // Read the EOT byte and return it
            if let Some(byte) = rx_buffer.read() {
                packet[0] = byte;
                return Ok(1);
            }
        }
        
        // Determine expected packet size
        let expected_packet_size: usize = if first_byte == SOH {
            SMALL_TOTAL_SIZE
        } else if first_byte == STX {
            LARGE_TOTAL_SIZE
        } else {
            return Err(XmodemError::InvalidPacket);
        };
        
        // Check if we have a complete packet
        if rx_buffer.len() < expected_packet_size {
            return Err(XmodemError::Timeout);
        }
        
        // Read the packet
        for i in 0..expected_packet_size {
            if let Some(byte) = rx_buffer.read() {
                packet[i] = byte;
            } else {
                return Err(XmodemError::Timeout);
            }
        }
        
        Ok(expected_packet_size)
    }
}

/// Send initial 'C' character to start XMODEM-CRC transfer
pub fn send_initial_c(tx_buffer: &mut RingBuffer) {
    tx_buffer.write(C);
}

/// Send ACK response
pub fn send_ack(tx_buffer: &mut RingBuffer) {
    tx_buffer.write(ACK);
}

/// Send NAK response
pub fn send_nak(tx_buffer: &mut RingBuffer) {
    tx_buffer.write(NAK);
}

/// Receive a firmware update using XMODEM-1K protocol
///
/// This function implements the XMODEM-1K protocol to receive firmware updates.
/// It validates the magic number and version of the incoming firmware before
/// writing it to flash memory.
pub fn receive_firmware(
    rx_buffer: &mut RingBuffer,
    tx_buffer: &mut RingBuffer,
    target_address: u32,
    expected_magic: u32,
    slot_size: usize,
    transmit_fn: fn(&mut RingBuffer),
) -> Result<(), XmodemError> {
    // XMODEM state variables
    let mut state = XmodemState::WaitHeader;
    let mut prev_index1: u8 = 0;
    let mut prev_index2: u8 = 0xFF;
    
    // Buffer to hold the largest possible packet
    let mut packet_buffer = [0u8; LARGE_TOTAL_SIZE];
    let mut data_buffer = [0u8; LARGE_PACKET_SIZE];
    let mut current_packet_size = SMALL_PACKET_SIZE;
    
    let mut current_address = target_address;
    let mut first_packet = true;
    let mut first_byte_processed = false;
    let mut transfer_complete = false;
    
    // Initialize timeout tracking
    let transfer_start = systick::get_tick_ms();
    let timeout_ms = 60000; // 60 seconds timeout
    
    // Start by sending 'C' to initiate XMODEM-CRC transfer
    send_initial_c(tx_buffer);
    transmit_fn(tx_buffer);
    
    // Send 'C' periodically until we receive a response
    let mut last_c_sent = transfer_start;
    let c_interval_ms = 3000; // Send 'C' every 3 seconds
    
    while !transfer_complete {
        // Check for timeout
        let current_time = systick::get_tick_ms();
        if current_time.wrapping_sub(transfer_start) > timeout_ms {
            return Err(XmodemError::Timeout);
        }
        
        // Send 'C' periodically until first byte is received
        if !first_byte_processed && current_time.wrapping_sub(last_c_sent) > c_interval_ms {
            send_initial_c(tx_buffer);
            transmit_fn(tx_buffer);
            last_c_sent = current_time;
        }
        
        // Try to extract a packet
        match extract_packet(rx_buffer, &mut packet_buffer, !first_byte_processed) {
            Ok(packet_size) => {
                first_byte_processed = true;
                
                // Handle EOT (End of Transmission)
                if packet_size == 1 && packet_buffer[0] == EOT {
                    send_ack(tx_buffer);
                    transmit_fn(tx_buffer);
                    transfer_complete = true;
                    break;
                }
                
                // Process the packet
                match process_packet(
                    &mut state, 
                    &packet_buffer, 
                    &mut data_buffer, 
                    &mut current_packet_size, 
                    &mut prev_index1, 
                    &mut prev_index2
                ) {
                    Ok(completed) => {
                        if completed {
                            transfer_complete = true;
                            break;
                        }
                        
                        // If we've processed the data, we need to write it to flash
                        if state == XmodemState::WaitHeader {
                            // For the first packet, validate magic number and version
                            if first_packet {
                                match validate_first_packet(&data_buffer, expected_magic, target_address) {
                                    Ok(_) => {
                                        // Erase the flash sector before writing first packet
                                        let p = unsafe { stm32f4::Peripherals::steal() };
                                        flash::erase_sector(&p, current_address);
                                        
                                        // First packet is valid, continue
                                        first_packet = false;
                                    },
                                    Err(e) => {
                                        // Cancel the transfer if validation fails
                                        send_cancel(tx_buffer)?;
                                        transmit_fn(tx_buffer);
                                        return Err(e);
                                    }
                                }
                            }
                            
                            // Write data to flash
                            let p = unsafe { stm32f4::Peripherals::steal() };
                            let result = flash::write(&p, &data_buffer[0..current_packet_size], current_address);
                            
                            if result != 0 {
                                // Flash write failed
                                send_cancel(tx_buffer)?;
                                transmit_fn(tx_buffer);
                                return Err(XmodemError::FlashError);
                            }
                            
                            // Update address for next packet
                            current_address += current_packet_size as u32;
                            
                            // Ensure we don't exceed the slot size
                            if (current_address - target_address) as usize > slot_size {
                                send_cancel(tx_buffer)?;
                                transmit_fn(tx_buffer);
                                return Err(XmodemError::FlashError);
                            }
                            
                            // Send ACK
                            send_ack(tx_buffer);
                            transmit_fn(tx_buffer);
                        }
                    },
                    Err(e) => {
                        // Send NAK for recoverable errors, CAN for fatal errors
                        match e {
                            XmodemError::Crc | XmodemError::PacketNumber => {
                                send_nak(tx_buffer);
                                transmit_fn(tx_buffer);
                                // Reset state to wait for packet retransmission
                                state = XmodemState::WaitHeader;
                            },
                            _ => {
                                send_cancel(tx_buffer)?;
                                transmit_fn(tx_buffer);
                                return Err(e);
                            }
                        }
                    }
                }
            },
            Err(XmodemError::Timeout) => {
                // Not enough data yet, continue waiting
                continue;
            },
            Err(e) => {
                // Handle other errors
                send_cancel(tx_buffer)?;
                transmit_fn(tx_buffer);
                return Err(e);
            }
        }
    }
    
    Ok(())
}

/// Helper function to print transfer status
pub fn print_error_message(err: XmodemError, tx_buffer: &mut RingBuffer, transmit_fn: fn(&mut RingBuffer)) {
    let error_msg = match err {
        XmodemError::Crc => "CRC error in transfer!\r\n",
        XmodemError::PacketNumber => "Packet number error in transfer!\r\n",
        XmodemError::Canceled => "Transfer was canceled!\r\n",
        XmodemError::InvalidMagic => "Invalid magic number in firmware!\r\n",
        XmodemError::OlderVersion => "New firmware is not newer than current version!\r\n",
        XmodemError::Timeout => "Timeout during transfer!\r\n",
        XmodemError::FlashError => "Error writing to flash memory!\r\n",
        XmodemError::InvalidPacket => "Invalid packet received!\r\n",
    };
    
    // Send the error message through UART
    for &byte in error_msg.as_bytes() {
        tx_buffer.write(byte);
    }
    transmit_fn(tx_buffer);
}

/// Extension method for RingBuffer to peek at first byte without removing it
pub trait RingBufferExt {
    fn peek(&self) -> Option<u8>;
}

impl RingBufferExt for RingBuffer {
    fn peek(&self) -> Option<u8> {
        if self.is_empty() {
            None
        } else {
            unsafe {
                // RingBuffer implementation should store head, tail and buffer as fields
                let tail = *self.tail.get();
                let buffer = &*self.buffer.get();
                Some(buffer[tail])
            }
        }
    }
}