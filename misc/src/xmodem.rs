#![no_std]

use core::convert::TryInto;
use stm32f4::Peripherals;
use crate::ring_buffer::RingBuffer;
use crate::flash;
use crate::image::{ImageHeader, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER};
use crate::systick;

// XMODEM Protocol Constants
pub const SOH: u8 = 0x01;  // Start of Header (128-byte packets)
pub const STX: u8 = 0x02;  // Start of Text (1024-byte packets)
pub const EOT: u8 = 0x04;  // End of Transmission
pub const ACK: u8 = 0x06;  // Acknowledge
pub const NAK: u8 = 0x15;  // Negative Acknowledge
pub const CAN: u8 = 0x18;  // Cancel
pub const C: u8 = 0x43;    // ASCII 'C' - Used for CRC-16 mode

// Timeout values (in milliseconds)
const INITIAL_TIMEOUT_MS: u32 = 10000;    // 10 seconds for initial connection
const PACKET_TIMEOUT_MS: u32 = 3000;      // 3 seconds between packets
const RETRY_DELAY_MS: u32 = 3000;         // 3 seconds between retry attempts
const MAX_RETRIES: usize = 10;            // Maximum number of retry attempts

// Buffer sizes
const STD_PACKET_SIZE: usize = 128;      // Standard XMODEM packet size
const CRC_SIZE: usize = 2;               // Size of CRC-16 in bytes
const PACKET_OVERHEAD: usize = 4;        // SOH/STX + PacketNum + ~PacketNum + CRC16
const MAX_PACKET_SIZE: usize = 1024;     // XMODEM-1K max data size
const MAX_BUFFER_SIZE: usize = MAX_PACKET_SIZE + PACKET_OVERHEAD; // Total buffer size needed

// XMODEM State Machine States
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XmodemState {
    Idle,
    WaitForC,
    WaitForPacketAck,
    WaitForSOH,
    WaitForPacketNumber,
    WaitForComplementPacketNumber,
    ReceivingData,
    WaitForCRC,
    Complete,
    Error,
}

// Error types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XmodemError {
    Timeout,
    Canceled,
    PacketNumber,
    Crc,
    InvalidPacket,
    FlashError,
    InvalidMagic,
    OlderVersion,
    EOT,
    BufferOverflow,
    InvalidState,
}

/// Calculate CRC-16 CCITT for XMODEM
pub fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Send a firmware image using XMODEM-1K protocol
pub fn send_firmware<F, G>(
    tx_buffer: &mut RingBuffer,
    rx_buffer: &mut RingBuffer,
    firmware_addr: u32,
    firmware_size: u32,
    transmit_fn: F,
    check_abort: G,
) -> Result<(), XmodemError>
where
    F: Fn(&mut RingBuffer),
    G: Fn() -> bool,
{
    // Local variables
    let mut state = XmodemState::Idle;
    let mut packet_buffer: [u8; MAX_BUFFER_SIZE] = [0; MAX_BUFFER_SIZE];
    let mut packet_number: u8 = 1;
    let mut retries: usize = 0;
    let mut current_offset: u32 = 0;
    let mut received_byte: u8;
    let mut start_time = systick::get_tick_ms();
    
    // Wait for receiver to initiate transfer with 'C'
    state = XmodemState::WaitForC;
    start_time = systick::get_tick_ms();
    
    while current_offset < firmware_size {
        // Check for abort request
        if check_abort() {
            // Send cancel and abort
            tx_buffer.write(CAN);
            transmit_fn(tx_buffer);
            tx_buffer.write(CAN);
            transmit_fn(tx_buffer);
            return Err(XmodemError::Canceled);
        }
        
        // Process state machine
        match state {
            XmodemState::WaitForC => {
                // Wait for receiver to send 'C' to start
                if rx_buffer.is_empty() {
                    // Check for timeout
                    if systick::wait_ms(start_time, INITIAL_TIMEOUT_MS) {
                        return Err(XmodemError::Timeout);
                    }
                    continue;
                }
                
                if let Some(byte) = rx_buffer.read() {
                    if byte == C {
                        // Got 'C', start sending first packet
                        state = XmodemState::Idle;
                    } else if byte == CAN {
                        return Err(XmodemError::Canceled);
                    } else {
                        // Ignore other bytes
                    }
                }
            },
            
            XmodemState::Idle => {
                // Prepare next packet
                let remaining_bytes = firmware_size - current_offset;
                let use_1k_packet = remaining_bytes >= MAX_PACKET_SIZE as u32;
                let packet_size = if use_1k_packet { MAX_PACKET_SIZE } else { STD_PACKET_SIZE };
                
                // Don't exceed remaining firmware size
                let actual_packet_size = core::cmp::min(packet_size, remaining_bytes as usize);
                
                // Prepare packet header
                if use_1k_packet {
                    packet_buffer[0] = STX;
                } else {
                    packet_buffer[0] = SOH;
                }
                packet_buffer[1] = packet_number;
                packet_buffer[2] = !packet_number;
                
                // Copy firmware data
                let mut i = 0;
                while i < packet_size {
                    if i < actual_packet_size {
                        // Read actual data from firmware
                        unsafe {
                            packet_buffer[3 + i] = *(firmware_addr.wrapping_add(current_offset) as *const u8).add(i);
                        }
                    } else {
                        // Pad with SUB character (0x1A)
                        packet_buffer[3 + i] = 0x1A;
                    }
                    i += 1;
                }
                
                // Calculate CRC-16
                let crc = calculate_crc16(&packet_buffer[3..(3 + packet_size)]);
                packet_buffer[3 + packet_size] = (crc >> 8) as u8;     // CRC high byte
                packet_buffer[3 + packet_size + 1] = crc as u8;        // CRC low byte
                
                // Send the packet
                for i in 0..(3 + packet_size + 2) {
                    tx_buffer.write(packet_buffer[i]);
                }
                transmit_fn(tx_buffer);
                
                // Move to wait for ACK state
                state = XmodemState::WaitForPacketAck;
                start_time = systick::get_tick_ms();
                retries = 0;
            },
            
            XmodemState::WaitForPacketAck => {
                // Wait for ACK or NAK
                if rx_buffer.is_empty() {
                    // Check for timeout
                    if systick::wait_ms(start_time, PACKET_TIMEOUT_MS) {
                        retries += 1;
                        if retries >= MAX_RETRIES {
                            return Err(XmodemError::Timeout);
                        }
                        
                        // Resend the packet
                        state = XmodemState::Idle;
                    }
                    continue;
                }
                
                if let Some(byte) = rx_buffer.read() {
                    if byte == ACK {
                        // Packet acknowledged, move to next packet
                        current_offset += packet_buffer[0] as u32;
                        packet_number = packet_number.wrapping_add(1);
                        state = XmodemState::Idle;
                    } else if byte == NAK {
                        // Packet rejected, retry
                        retries += 1;
                        if retries >= MAX_RETRIES {
                            return Err(XmodemError::Timeout);
                        }
                        state = XmodemState::Idle;
                    } else if byte == CAN {
                        // Transfer canceled by receiver
                        return Err(XmodemError::Canceled);
                    } else {
                        // Ignore unexpected bytes
                    }
                }
            },
            
            _ => {
                // Invalid state for sender
                return Err(XmodemError::InvalidState);
            }
        }
    }
    
    // All packets sent, send EOT
    tx_buffer.write(EOT);
    transmit_fn(tx_buffer);
    
    // Wait for ACK to EOT
    start_time = systick::get_tick_ms();
    while !rx_buffer.is_empty() {
        if let Some(byte) = rx_buffer.read() {
            if byte == ACK {
                return Ok(());
            } else if byte == NAK {
                // Resend EOT
                tx_buffer.write(EOT);
                transmit_fn(tx_buffer);
            } else if byte == CAN {
                return Err(XmodemError::Canceled);
            }
        }
        
        // Check for timeout
        if systick::wait_ms(start_time, PACKET_TIMEOUT_MS) {
            retries += 1;
            if retries >= MAX_RETRIES {
                return Err(XmodemError::Timeout);
            }
            
            // Resend EOT
            tx_buffer.write(EOT);
            transmit_fn(tx_buffer);
            start_time = systick::get_tick_ms();
        }
    }
    
    Ok(())
}

/// Receive a firmware image using XMODEM-1K protocol
pub fn receive_firmware<F>(
    rx_buffer: &mut RingBuffer,
    tx_buffer: &mut RingBuffer,
    target_address: u32,
    expected_magic: u32,
    slot_size: u32,
    transmit_fn: F,
) -> Result<(), XmodemError>
where
    F: Fn(&mut RingBuffer),
{
    // Local variables
    let mut state = XmodemState::WaitForSOH;
    let mut packet_number: u8 = 1;
    let mut packet_buffer: [u8; MAX_BUFFER_SIZE] = [0; MAX_BUFFER_SIZE];
    let mut buffer_index: usize = 0;
    let mut current_address = target_address;
    let mut received_byte: u8;
    let mut packet_size: usize = STD_PACKET_SIZE;
    let mut bytes_received: usize = 0;
    let mut crc_received: u16 = 0;
    let mut retry_count: usize = 0;
    let mut initial_transfer: bool = true;
    let mut last_c_time = systick::get_tick_ms();
    
    // Initialize peripherals for Flash operations
    let p = unsafe { Peripherals::steal() };
    
    // Send initial 'C' to request transfer in CRC mode
    tx_buffer.write(C);
    transmit_fn(tx_buffer);
    
    // Main state machine loop
    loop {
        // Check if we need to send another 'C' (every 3 seconds if still waiting for SOH)
        if state == XmodemState::WaitForSOH && initial_transfer {
            let current_time = systick::get_tick_ms();
            if systick::wait_ms(last_c_time, RETRY_DELAY_MS) {
                retry_count += 1;
                if retry_count >= MAX_RETRIES {
                    return Err(XmodemError::Timeout);
                }
                
                // Send another 'C'
                tx_buffer.write(C);
                transmit_fn(tx_buffer);
                last_c_time = current_time;
            }
        }
        
        // Try to read a byte
        if rx_buffer.is_empty() {
            // No data yet, continue waiting
            continue;
        }
        
        // We have data, read a byte
        if let Some(byte) = rx_buffer.read() {
            received_byte = byte;
            
            // Check for cancel
            if received_byte == CAN {
                // Send CAN to acknowledge
                tx_buffer.write(CAN);
                transmit_fn(tx_buffer);
                
                return Err(XmodemError::Canceled);
            }
            
            // Check for EOT (end of transmission)
            if received_byte == EOT {
                if state != XmodemState::WaitForSOH {
                    // Unexpected EOT
                    return Err(XmodemError::EOT);
                }
                
                // Acknowledge the EOT
                tx_buffer.write(ACK);
                transmit_fn(tx_buffer);
                
                // Check the firmware header
                let header = unsafe { &*(target_address as *const ImageHeader) };
                
                // Validate the magic number
                if header.image_magic != expected_magic {
                    return Err(XmodemError::InvalidMagic);
                }
                
                return Ok(());
            }
            
            // Process state machine
            match state {
                XmodemState::WaitForSOH => {
                    if received_byte == SOH {
                        // Standard 128-byte packet
                        packet_size = STD_PACKET_SIZE;
                        state = XmodemState::WaitForPacketNumber;
                        buffer_index = 0;
                    } else if received_byte == STX {
                        // 1K packet (1024 bytes)
                        packet_size = MAX_PACKET_SIZE;
                        state = XmodemState::WaitForPacketNumber;
                        buffer_index = 0;
                    }
                    // Ignore other bytes when waiting for SOH/STX
                },
                
                XmodemState::WaitForPacketNumber => {
                    packet_buffer[buffer_index] = received_byte;
                    buffer_index += 1;
                    
                    // Mark that we've started receiving packets
                    initial_transfer = false;
                    
                    // Verify packet number
                    if received_byte != packet_number {
                        // Send NAK to request retransmission
                        tx_buffer.write(NAK);
                        transmit_fn(tx_buffer);
                        
                        // Go back to waiting for SOH
                        state = XmodemState::WaitForSOH;
                        continue;
                    }
                    
                    state = XmodemState::WaitForComplementPacketNumber;
                },
                
                XmodemState::WaitForComplementPacketNumber => {
                    packet_buffer[buffer_index] = received_byte;
                    buffer_index += 1;
                    
                    // Verify complement of packet number
                    if received_byte != (255 - packet_buffer[0]) {
                        // Send NAK to request retransmission
                        tx_buffer.write(NAK);
                        transmit_fn(tx_buffer);
                        
                        // Go back to waiting for SOH
                        state = XmodemState::WaitForSOH;
                        continue;
                    }
                    
                    // Move to data reception state
                    state = XmodemState::ReceivingData;
                    bytes_received = 0;
                },
                
                XmodemState::ReceivingData => {
                    if buffer_index >= MAX_BUFFER_SIZE {
                        // Buffer overflow protection
                        return Err(XmodemError::BufferOverflow);
                    }
                    
                    packet_buffer[buffer_index] = received_byte;
                    buffer_index += 1;
                    bytes_received += 1;
                    
                    if bytes_received >= packet_size {
                        state = XmodemState::WaitForCRC;
                        bytes_received = 0;
                    }
                },
                
                XmodemState::WaitForCRC => {
                    if buffer_index >= MAX_BUFFER_SIZE {
                        // Buffer overflow protection
                        return Err(XmodemError::BufferOverflow);
                    }
                    
                    packet_buffer[buffer_index] = received_byte;
                    buffer_index += 1;
                    
                    if bytes_received == 0 {
                        // First CRC byte (high byte)
                        crc_received = (received_byte as u16) << 8;
                        bytes_received += 1;
                    } else {
                        // Second CRC byte (low byte)
                        crc_received |= received_byte as u16;
                        
                        // Calculate CRC-16 for the received data
                        let calculated_crc = calculate_crc16(&packet_buffer[2..(2 + packet_size)]);
                        
                        if calculated_crc != crc_received {
                            // CRC error, request retransmission
                            tx_buffer.write(NAK);
                            transmit_fn(tx_buffer);
                            
                            // Go back to waiting for SOH
                            state = XmodemState::WaitForSOH;
                            continue;
                        }
                        
                        // Data is valid, write to flash memory
                        // First check if we have space
                        if current_address - target_address + (packet_size as u32) > slot_size {
                            // Not enough space
                            tx_buffer.write(CAN);
                            transmit_fn(tx_buffer);
                            tx_buffer.write(CAN);
                            transmit_fn(tx_buffer);
                            
                            return Err(XmodemError::FlashError);
                        }
                        
                        // For the first packet, erase the sector
                        if packet_number == 1 {
                            // Erase the first sector
                            if flash::erase_sector(&p, current_address) == 0 {
                                return Err(XmodemError::FlashError);
                            }
                        }
                        
                        // Check if we need to erase more sectors
                        let next_addr = current_address + packet_size as u32;
                        let current_sector = flash::get_sector_number(current_address);
                        let next_sector = flash::get_sector_number(next_addr);
                        
                        if current_sector != next_sector && next_sector.is_some() {
                            // We're crossing a sector boundary, erase the next sector
                            if flash::erase_sector(&p, next_addr) == 0 {
                                return Err(XmodemError::FlashError);
                            }
                        }
                        
                        // Write data to flash
                        let flash_result = flash::write(&p, &packet_buffer[2..(2 + packet_size)], current_address);
                        
                        if flash_result != 0 {
                            // Flash write error
                            tx_buffer.write(CAN);
                            transmit_fn(tx_buffer);
                            tx_buffer.write(CAN);
                            transmit_fn(tx_buffer);
                            
                            return Err(XmodemError::FlashError);
                        }
                        
                        // Advance address pointer
                        current_address += packet_size as u32;
                        
                        // Acknowledge successful reception
                        tx_buffer.write(ACK);
                        transmit_fn(tx_buffer);
                        
                        // Increment packet number for next packet
                        packet_number = packet_number.wrapping_add(1);
                        
                        // Reset state machine for next packet
                        state = XmodemState::WaitForSOH;
                    }
                },
                
                _ => {
                    // Invalid state
                    return Err(XmodemError::InvalidState);
                }
            }
        }
    }
}

/// Helper function to send CAN (cancel) sequence
pub fn send_cancel(tx_buffer: &mut RingBuffer, transmit_fn: impl Fn(&mut RingBuffer)) {
    // Send CAN characters to abort transfer
    tx_buffer.write(CAN);
    transmit_fn(tx_buffer);
    tx_buffer.write(CAN);
    transmit_fn(tx_buffer);
}

/// Check if the new firmware is a valid update compared to the current one
pub fn validate_firmware_update(current_addr: u32, new_addr: u32) -> Result<bool, XmodemError> {
    let current_header = unsafe { &*(current_addr as *const ImageHeader) };
    let new_header = unsafe { &*(new_addr as *const ImageHeader) };
    
    // Check if new firmware has valid magic
    if !new_header.is_valid() {
        return Err(XmodemError::InvalidMagic);
    }
    
    // If current firmware is invalid/missing, any valid firmware is acceptable
    if !current_header.is_valid() {
        return Ok(true);
    }
    
    // Check if new firmware is newer than current one
    Ok(new_header.is_newer_than(current_header))
}

/// Verify CRC of downloaded firmware
pub fn verify_firmware_crc(addr: u32, size: u32) -> bool {
    // This function would verify the CRC of the firmware
    // We'd need to know how the CRC is calculated and stored
    // For now, just return true
    true
}