#![no_std]

use crate::{flash, systick};
use crate::ring_buffer::RingBuffer;
use core::convert::TryFrom;

use rmodem::{
    Control, Error as RmodemError, Result as RmodemResult, Sequence, 
    XmodemData, XmodemPacket, OneKData, XmodemOneKPacket, SOH, STX, 
    EOT, ACK, NAK, CAN, IDLE
};

// Timeouts (in milliseconds)
const INITIAL_TIMEOUT_MS: u32 = 10000; // 10 seconds for initial connection
const PACKET_TIMEOUT_MS: u32 = 1000;   // 1 second for packet operations

/// Custom error type for XMODEM operations
#[derive(Debug, PartialEq)]
pub enum XmodemError {
    Crc,
    PacketNumber,
    Canceled,
    InvalidMagic,
    OlderVersion,
    Timeout,
    FlashError,
    InvalidPacket,
    EOT,
}

// Convert rmodem Error to our XmodemError
impl From<RmodemError> for XmodemError {
    fn from(err: RmodemError) -> Self {
        match err {
            RmodemError::Crc => XmodemError::Crc,
            RmodemError::SequenceNumber => XmodemError::PacketNumber,
            RmodemError::Cancel => XmodemError::Canceled,
            RmodemError::InvalidControl(_) => XmodemError::InvalidPacket,
            RmodemError::InvalidHeader(_) => XmodemError::InvalidPacket,
            _ => XmodemError::InvalidPacket,
        }
    }
}

/// Read a byte with timeout from the ring buffer
fn read_byte_timeout(rx_buffer: &mut RingBuffer, timeout_ms: u32) -> Option<u8> {
    let start_ms = systick::get_tick_ms();
    
    while !systick::wait_ms(start_ms, timeout_ms) {
        if let Some(byte) = rx_buffer.read() {
            return Some(byte);
        }
        cortex_m::asm::wfi();
    }
    
    None // Timeout occurred
}

/// Write a byte to the ring buffer
fn write_byte(tx_buffer: &mut RingBuffer, byte: u8) {
    tx_buffer.write(byte);
}

/// Receive firmware via XMODEM protocol (supports both standard and 1K variants)
pub fn receive_firmware<F>(
    rx_buffer: &mut RingBuffer,
    tx_buffer: &mut RingBuffer,
    target_address: u32,
    expected_magic: u32,
    max_size: u32,
    mut ensure_tx: F
) -> Result<usize, XmodemError>
where
    F: FnMut() -> (),
{
    // Clear receive buffer before starting
    while rx_buffer.read().is_some() {}
    
    // Send 'C' to indicate we want CRC mode (IDLE = 'C')
    write_byte(tx_buffer, IDLE);
    ensure_tx();
    
    let mut total_bytes: usize = 0;
    let mut write_address: u32 = target_address;
    let mut sequence = Sequence::default();
    
    // Start timer for initial timeout
    let mut start_ms = systick::get_tick_ms();
    
    let mut one_k_mode = false;
    
    loop {
        // Check for timeout
        if systick::wait_ms(start_ms, INITIAL_TIMEOUT_MS) {
            // Send cancel bytes
            write_byte(tx_buffer, CAN);
            write_byte(tx_buffer, CAN);
            ensure_tx();
            return Err(XmodemError::Timeout);
        }
        
        // Wait for packet header
        let header_byte = match read_byte_timeout(rx_buffer, PACKET_TIMEOUT_MS) {
            Some(byte) => byte,
            None => {
                // Timeout on read, send another 'C'
                write_byte(tx_buffer, IDLE);
                ensure_tx();
                start_ms = systick::get_tick_ms();
                continue;
            }
        };
        
        // Process different headers
        match header_byte {
            SOH => {
                // Standard 128-byte packet
                one_k_mode = false;
            },
            STX => {
                // 1K 1024-byte packet
                one_k_mode = true;
            },
            EOT => {
                // End of transmission
                write_byte(tx_buffer, ACK);
                ensure_tx();
                
                if total_bytes == 0 {
                    return Err(XmodemError::EOT);
                }
                
                // Verify firmware
                let mut header_bytes = [0u8; core::mem::size_of::<crate::image::ImageHeader>()];
                flash::read(target_address, &mut header_bytes);
                
                let header_ptr: *const crate::image::ImageHeader = header_bytes.as_ptr() as *const _;
                let header: &crate::image::ImageHeader = unsafe { &*header_ptr };
                
                if header.image_magic != expected_magic {
                    return Err(XmodemError::InvalidMagic);
                }
                
                // Firmware valid!
                return Ok(total_bytes);
            },
            CAN => {
                // Read another CAN to confirm
                if read_byte_timeout(rx_buffer, PACKET_TIMEOUT_MS) == Some(CAN) {
                    return Err(XmodemError::Canceled);
                }
                continue;
            },
            _ => {
                // Unknown character, ignore
                continue;
            }
        };
        
        if one_k_mode {
            // Handle 1K packet
            // Buffer for 1K packet (1024 bytes) + sequence (1) + complement (1) + CRC (2) = 1028 bytes
            let mut packet_buffer = [0u8; 1028];
            
            // Store header byte
            packet_buffer[0] = header_byte;
            
            // Read the rest of the packet
            let mut bytes_read = 1;
            
            while bytes_read < 1028 {
                match read_byte_timeout(rx_buffer, PACKET_TIMEOUT_MS) {
                    Some(byte) => {
                        packet_buffer[bytes_read] = byte;
                        bytes_read += 1;
                    },
                    None => {
                        // Timeout reading packet
                        write_byte(tx_buffer, NAK);
                        ensure_tx();
                        break;
                    }
                }
            }
            
            if bytes_read < 1028 {
                // Didn't get complete packet
                continue;
            }
            
            // Try to parse the packet
            let packet_result = XmodemOneKPacket::try_from(&packet_buffer[..]);
            
            match packet_result {
                Ok(packet) => {
                    // Verify sequence number
                    if packet.sequence() != sequence {
                        // If it's a repeat of the previous packet, ACK it
                        if packet.sequence().value() == sequence.value().wrapping_sub(1) {
                            write_byte(tx_buffer, ACK);
                            ensure_tx();
                        } else {
                            // Otherwise NAK it
                            write_byte(tx_buffer, NAK);
                            ensure_tx();
                        }
                        continue;
                    }
                    
                    // Get the data from the packet
                    let data = packet.data();
                    let data_slice = data.as_ref();
                    
                    // Process first packet specially to check firmware header
                    if total_bytes == 0 {
                        // First packet contains the header
                        let header_ptr: *const crate::image::ImageHeader = data_slice.as_ptr() as *const _;
                        let header: &crate::image::ImageHeader = unsafe { &*header_ptr };
                        
                        if header.image_magic != expected_magic {
                            write_byte(tx_buffer, CAN);
                            write_byte(tx_buffer, CAN);
                            ensure_tx();
                            return Err(XmodemError::InvalidMagic);
                        }
                        
                        // Erase the flash sector for this firmware
                        let peripherals = unsafe { stm32f4::Peripherals::steal() };
                        flash::erase_sector(&peripherals, write_address);
                    }
                    
                    // Calculate padding for aligned writes
                    let data_len = data_slice.len();
                    let padding = data_len % 4;
                    let aligned_size = if padding == 0 { data_len } else { data_len + (4 - padding) };
                    
                    // Make sure we don't exceed maximum firmware size
                    if total_bytes + aligned_size > max_size as usize {
                        write_byte(tx_buffer, CAN);
                        write_byte(tx_buffer, CAN);
                        ensure_tx();
                        return Err(XmodemError::Timeout); // Using timeout as a generic error
                    }
                    
                    // Write data to flash
                    let peripherals = unsafe { stm32f4::Peripherals::steal() };
                    let result = flash::write(&peripherals, data_slice, write_address);
                    if result != 0 {
                        write_byte(tx_buffer, CAN);
                        write_byte(tx_buffer, CAN);
                        ensure_tx();
                        return Err(XmodemError::FlashError);
                    }
                    
                    // Update counters
                    write_address += data_len as u32;
                    total_bytes += data_len;
                    sequence = sequence.next();
                    
                    // ACK the packet
                    write_byte(tx_buffer, ACK);
                    ensure_tx();
                }
                Err(_) => {
                    // Invalid packet
                    write_byte(tx_buffer, NAK);
                    ensure_tx();
                    continue;
                }
            }
        } else {
            // Handle standard packet
            // Buffer for standard packet (128 bytes) + sequence (1) + complement (1) + CRC (2) = 132 bytes
            let mut packet_buffer = [0u8; 132];
            
            // Store header byte
            packet_buffer[0] = header_byte;
            
            // Read the rest of the packet
            let mut bytes_read = 1;
            
            while bytes_read < 132 {
                match read_byte_timeout(rx_buffer, PACKET_TIMEOUT_MS) {
                    Some(byte) => {
                        packet_buffer[bytes_read] = byte;
                        bytes_read += 1;
                    },
                    None => {
                        // Timeout reading packet
                        write_byte(tx_buffer, NAK);
                        ensure_tx();
                        break;
                    }
                }
            }
            
            if bytes_read < 132 {
                // Didn't get complete packet
                continue;
            }
            
            // Try to parse the packet
            let packet_result = XmodemPacket::try_from(&packet_buffer[..]);
            
            match packet_result {
                Ok(packet) => {
                    // Verify sequence number
                    if packet.sequence() != sequence {
                        // If it's a repeat of the previous packet, ACK it
                        if packet.sequence().value() == sequence.value().wrapping_sub(1) {
                            write_byte(tx_buffer, ACK);
                            ensure_tx();
                        } else {
                            // Otherwise NAK it
                            write_byte(tx_buffer, NAK);
                            ensure_tx();
                        }
                        continue;
                    }
                    
                    // Get the data from the packet
                    let data = packet.data();
                    let data_slice = data.as_ref();
                    
                    // Process first packet specially to check firmware header
                    if total_bytes == 0 {
                        // First packet contains the header
                        let header_ptr: *const crate::image::ImageHeader = data_slice.as_ptr() as *const _;
                        let header: &crate::image::ImageHeader = unsafe { &*header_ptr };
                        
                        if header.image_magic != expected_magic {
                            write_byte(tx_buffer, CAN);
                            write_byte(tx_buffer, CAN);
                            ensure_tx();
                            return Err(XmodemError::InvalidMagic);
                        }
                        
                        // Erase the flash sector for this firmware
                        let peripherals = unsafe { stm32f4::Peripherals::steal() };
                        flash::erase_sector(&peripherals, write_address);
                    }
                    
                    // Calculate padding for aligned writes
                    let data_len = data_slice.len();
                    let padding = data_len % 4;
                    let aligned_size = if padding == 0 { data_len } else { data_len + (4 - padding) };
                    
                    // Make sure we don't exceed maximum firmware size
                    if total_bytes + aligned_size > max_size as usize {
                        write_byte(tx_buffer, CAN);
                        write_byte(tx_buffer, CAN);
                        ensure_tx();
                        return Err(XmodemError::Timeout); // Using timeout as a generic error
                    }
                    
                    // Write data to flash
                    let peripherals = unsafe { stm32f4::Peripherals::steal() };
                    let result = flash::write(&peripherals, data_slice, write_address);
                    if result != 0 {
                        write_byte(tx_buffer, CAN);
                        write_byte(tx_buffer, CAN);
                        ensure_tx();
                        return Err(XmodemError::FlashError);
                    }
                    
                    // Update counters
                    write_address += data_len as u32;
                    total_bytes += data_len;
                    sequence = sequence.next();
                    
                    // ACK the packet
                    write_byte(tx_buffer, ACK);
                    ensure_tx();
                }
                Err(_) => {
                    // Invalid packet
                    write_byte(tx_buffer, NAK);
                    ensure_tx();
                    continue;
                }
            }
        }
        
        // Reset timeout timer
        start_ms = systick::get_tick_ms();
    }
}