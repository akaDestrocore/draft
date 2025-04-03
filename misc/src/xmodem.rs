#![no_std]

use crate::ring_buffer::RingBuffer;
use crate::image::{
    ImageHeader, 
    IMAGE_MAGIC_APP, IMAGE_MAGIC_LOADER, IMAGE_MAGIC_UPDATER,
    IMAGE_TYPE_APP, IMAGE_TYPE_LOADER, IMAGE_TYPE_UPDATER
};
use core::mem::size_of;

// XMODEM constants
pub const X_SOH: u8 = 0x01;
pub const X_STX: u8 = 0x02;
pub const X_EOT: u8 = 0x04;
pub const X_ACK: u8 = 0x06;
pub const X_NAK: u8 = 0x15;
pub const X_CAN: u8 = 0x18;
pub const X_C: u8 = 0x43;

pub const BUFFER_SIZE: usize = 2048;
pub const PACKET_128_SIZE: usize = 128;
pub const PACKET_1K_SIZE: usize = 1024;
pub const PACKET_OVERHEAD: usize = 5; // SOH/STX + SEQ + ~SEQ + CRC16
pub const MAX_PACKET_SIZE: usize = PACKET_1K_SIZE + PACKET_OVERHEAD;
pub const AUTH_TAG_SIZE: usize = 16;
pub const NONCE_SIZE: usize = 12;
pub const FILE_SIZE_FIELD: usize = 4;
pub const HEADER_SIZE: usize = NONCE_SIZE + FILE_SIZE_FIELD;
pub const IMAGE_HEADER_SIZE: usize = size_of::<ImageHeader>();

// Memory addresses
pub const SLOT_2_APP_ADDR: u32 = 0x08020000;
pub const SLOT_2_VER_ADDR: u32 = 0x08020000;
pub const UPDATER_ADDR: u32 = 0x08008000;
pub const PATCH_ADDR: u32 = 0x08040000;
pub const BACKUP_ADDR: u32 = 0x08060000;

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum XmodemState {
    WaitHeaderByte,
    WaitIndex1, 
    WaitIndex2,
    ReadData,
    WaitCRC1,
    WaitCRC2,
    ProcessPacket,
}

#[derive(Debug)]
pub enum XmodemError {
    IndexError,             // Packet index error
    CrcError,               // CRC verification failed
    DecryptionError,        // Error during decryption
    AuthenticationError,    // Authentication tag verification failed
    FlashError,             // Error writing to flash
    FileSizeTooLarge,       // File size exceeds available space
    BufferOverflow,         // Buffer overflow
    Timeout,                // Communication timeout
    UnexpectedEOT,          // Unexpected end of transmission
    InvalidPacketType,      // Invalid packet type received
    
    InvalidImageType,       // Image type doesn't match expected
    InvalidMagic,           // Invalid magic number
    OlderVersion,           // Received version is not newer than installed
    HeaderCheckFailed,      // General header check failed
}

pub trait FlashOperations {
    fn erase(&self, address: u32) -> Result<(), XmodemError>;
    fn write(&self, address: u32, data: &[u8]) -> Result<(), XmodemError>;
}

/// Cryptographic operations trait for abstraction
pub trait CryptoOperations {
    fn init_decryption(&mut self, key: &[u8], nonce: &[u8], header: &[u8]) -> Result<(), XmodemError>;
    fn decrypt_chunk(&mut self, data: &[u8], output: &mut [u8]) -> Result<usize, XmodemError>;
    fn verify_tag(&mut self, expected_tag: &[u8]) -> Result<(), XmodemError>;
    fn finish_decryption(&mut self) -> Result<(), XmodemError>;
}

pub struct XmodemReceiver<F: FlashOperations, C: CryptoOperations> {
    pub state: XmodemState,
    pub packet_index: u8,
    pub packet_size: usize,
    pub buffer: [u8; MAX_PACKET_SIZE],
    pub data_buffer: [u8; PACKET_1K_SIZE],
    pub packet_received: bool,
    pub first_packet: bool,
    pub crc_received: u16,
    
    // Firmware state
    pub target_address: u32,
    pub current_address: u32,
    pub file_size: u32,
    pub bytes_received: u32,
    pub first_packet_complete: bool,
    
    // Image header verification
    pub expected_image_type: u8,
    pub header_verified: bool,
    
    // Encryption state
    pub key: [u8; 16],
    pub nonce: [u8; 12],
    pub auth_tag: [u8; AUTH_TAG_SIZE],
    pub aad_header: [u8; 16],
    
    // Tracking variables
    data_counter: usize,
    total_packet_count: u32,
    
    // External dependencies
    flash: F,
    crypto: C,
}

impl<F: FlashOperations, C: CryptoOperations> XmodemReceiver<F, C> {
    pub fn new(target_address: u32, expected_image_type: u8, flash: F, crypto: C) -> Self {
        Self {
            state: XmodemState::WaitHeaderByte,
            packet_index: 0,
            packet_size: 0,
            buffer: [0; MAX_PACKET_SIZE],
            data_buffer: [0; PACKET_1K_SIZE],
            packet_received: false,
            first_packet: false,
            crc_received: 0,
            
            target_address,
            current_address: target_address,
            file_size: 0,
            bytes_received: 0,
            first_packet_complete: false,
            
            expected_image_type,
            header_verified: false,
            
            key: [
                0xAC, 0x00, 0xD6, 0x7F, 0x21, 0xD2, 0x94, 0x46,
                0x2F, 0x2A, 0xB6, 0x84, 0xE2, 0xE7, 0x83, 0xF7
            ],
            nonce: [0; 12],
            auth_tag: [0; AUTH_TAG_SIZE],
            aad_header: [0; 16],
            
            data_counter: 0,
            total_packet_count: 0,
            
            flash,
            crypto,
        }
    }
    
    // CRC16 for XMODEM
    pub fn calculate_crc16(&self, data: &[u8]) -> u16 {
        let mut crc: u16 = 0;
        for &byte in data.iter() {
            crc ^= (byte as u16) << 8;
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

    // Prepare AAD
    pub fn prepare_header(&mut self) -> Result<(), XmodemError> {
        self.aad_header = [
            0x02, 0x00, 0x07, 0x07, 0x60, 0x61, 0x5F, 0x6B,
            0x65, 0x79, 0x01, 0x00, 0x00, 0x01, 0xFF, 0xFF
        ];
        
        if let Some(backup_header) = self.get_backup_version() {
            // Update AAD header with backup version
            self.aad_header[12] = backup_header.version_major;
            self.aad_header[13] = backup_header.version_minor;
            self.aad_header[14] = backup_header.version_patch;
        }
        
        Ok(())
    }
    
    /// Extract firmware size from header
    pub fn extract_size(&self, offset: usize) -> u32 {
        ((self.buffer[offset] as u32) << 24) |
        ((self.buffer[offset + 1] as u32) << 16) |
        ((self.buffer[offset + 2] as u32) << 8) |
        (self.buffer[offset + 3] as u32)
    }
    
    /// Get backup firmware version info
    pub fn get_backup_version(&self) -> Option<ImageHeader> {
        // Read version info from backup location
        let backup_ptr = BACKUP_ADDR as *const ImageHeader;
        let magic: u32 = unsafe { (*backup_ptr).image_magic };
        
        if magic == 0xFFFFFFFF {
            None
        } else {
            unsafe { Some(*backup_ptr) }
        }
    }
    
    /// Verify image header before accepting firmware
    fn verify_image_header(&self, header_data: &[u8]) -> Result<(), XmodemError> {
        if header_data.len() < size_of::<ImageHeader>() {
            return Err(XmodemError::HeaderCheckFailed);
        }
        
        // Parse the header
        let header: ImageHeader = unsafe { 
            core::ptr::read_unaligned(header_data.as_ptr() as *const ImageHeader) 
        };
        
        // Check the magic number based on expected type
        let expected_magic = match self.expected_image_type {
            IMAGE_TYPE_LOADER => IMAGE_MAGIC_LOADER,
            IMAGE_TYPE_UPDATER => IMAGE_MAGIC_UPDATER,
            IMAGE_TYPE_APP => IMAGE_MAGIC_APP,
            _ => return Err(XmodemError::InvalidImageType),
        };
        
        if header.image_magic != expected_magic {
            return Err(XmodemError::InvalidMagic);
        }
        
        // Check image type
        if header.image_type != self.expected_image_type {
            return Err(XmodemError::InvalidImageType);
        }
        
        // Check if a valid header exists at the target address
        let current_header = unsafe {
            let magic = *(self.target_address as *const u32);
            if magic == 0xFFFFFFFF {
                // No valid header
                None
            } else {
                // Possibly valid header exists
                let header_ptr = self.target_address as *const ImageHeader;
                Some(*header_ptr)
            }
        };
        
        // If a valid header exists, check that the new version is newer
        if let Some(current_header) = current_header {
            if current_header.image_magic == expected_magic {
                // Check version
                if !header.is_newer_than(&current_header) {
                    return Err(XmodemError::OlderVersion);
                }
            }
        }
        
        Ok(())
    }
    
    /// Process one received byte according to the XMODEM protocol
    pub fn process_byte(&mut self, rx_byte: u8) -> Result<bool, XmodemError> {
        match self.state {
            XmodemState::WaitHeaderByte => {
                match rx_byte {
                    X_SOH => {
                        // 128-byte packet
                        self.packet_size = PACKET_128_SIZE;
                        self.state = XmodemState::WaitIndex1;
                        self.buffer[0] = rx_byte;
                    },
                    X_STX => {
                        // 1024-byte packet
                        self.packet_size = PACKET_1K_SIZE;
                        self.state = XmodemState::WaitIndex1;
                        self.buffer[0] = rx_byte;
                    },
                    X_EOT => {
                        // End of transmission
                        return Ok(true);
                    },
                    _ => {
                        // Ignore other bytes while waiting for header
                    }
                }
            },
            XmodemState::WaitIndex1 => {
                self.packet_index = rx_byte;
                self.buffer[1] = rx_byte;
                self.state = XmodemState::WaitIndex2;
            },
            XmodemState::WaitIndex2 => {
                // The second byte should be the complement of the first
                if rx_byte != (255 - self.packet_index) {
                    return Err(XmodemError::IndexError);
                }
                self.buffer[2] = rx_byte;
                self.state = XmodemState::ReadData;
                self.data_counter = 0;
            },
            XmodemState::ReadData => {
                if self.data_counter < self.packet_size {
                    self.buffer[3 + self.data_counter] = rx_byte;
                    self.data_counter += 1;
                    
                    if self.data_counter == self.packet_size {
                        self.state = XmodemState::WaitCRC1;
                    }
                }
            },
            XmodemState::WaitCRC1 => {
                self.crc_received = (rx_byte as u16) << 8;
                self.buffer[3 + self.packet_size] = rx_byte;
                self.state = XmodemState::WaitCRC2;
            },
            XmodemState::WaitCRC2 => {
                self.crc_received |= rx_byte as u16;
                self.buffer[4 + self.packet_size] = rx_byte;
                
                // Calculate CRC
                let calculated_crc = self.calculate_crc16(&self.buffer[3..(3 + self.packet_size)]);
                
                if calculated_crc != self.crc_received {
                    return Err(XmodemError::CrcError);
                }
                
                self.state = XmodemState::ProcessPacket;
                self.packet_received = true;
                
                // Automatically transition to processing
                return self.process_packet();
            },
            XmodemState::ProcessPacket => {
                // This shouldn't be reached through process_byte
                // It's handled in process_packet
            }
        }
        
        Ok(false) // Not finished yet
    }
    
    /// Process a complete packet
    pub fn process_packet(&mut self) -> Result<bool, XmodemError> {
        if !self.packet_received {
            return Ok(false);
        }
        
        // Reset for next packet
        self.packet_received = false;
        
        // Process based on packet index
        if self.packet_index == 1 && !self.first_packet_complete {
            // This is the first packet, which contains the header
            let header_offset = 3; // SOH/STX + SEQ + ~SEQ
            
            // Make sure the packet is large enough to contain a header
            if self.packet_size < IMAGE_HEADER_SIZE {
                return Err(XmodemError::BufferOverflow);
            }
            
            // Create a copy of header data to avoid borrow conflicts
            let mut header_data_copy: [u8; IMAGE_HEADER_SIZE] = [0; IMAGE_HEADER_SIZE];
            header_data_copy.copy_from_slice(&self.buffer[header_offset..(header_offset + IMAGE_HEADER_SIZE)]);
            
            // Verify the header before erasing anything
            if !self.header_verified {
                match self.verify_image_header(&header_data_copy) {
                    Ok(()) => {
                        // Header verification passed
                        self.header_verified = true;
                    },
                    Err(e) => {
                        // Cancel the transfer
                        return Err(e);
                    }
                }
            }
            
            // At this point, header is verified - we can erase the target flash
            self.flash.erase(self.target_address)?;
            
            // Extract nonce from the first packet (immediately after header)
            let nonce_offset = header_offset + IMAGE_HEADER_SIZE;
            if nonce_offset + NONCE_SIZE > self.packet_size + 3 {
                return Err(XmodemError::BufferOverflow);
            }
            self.nonce.copy_from_slice(&self.buffer[nonce_offset..(nonce_offset + NONCE_SIZE)]);
            
            // Extract file size (after nonce)
            let size_offset = nonce_offset + NONCE_SIZE;
            if size_offset + FILE_SIZE_FIELD > self.packet_size + 3 {
                return Err(XmodemError::BufferOverflow);
            }
            self.file_size = self.extract_size(size_offset);
            
            // Check file size limit
            if self.file_size > 262144 { // 256KB
                return Err(XmodemError::FileSizeTooLarge);
            }
            
            // Update AAD header with current firmware version
            self.prepare_header()?;
            
            // Initialize decryption
            self.crypto.init_decryption(&self.key, &self.nonce, &self.aad_header)?;
            
            // Write the header to flash
            self.flash.write(self.target_address, &header_data_copy)?;
            
            // Process the data part in the first packet
            let data_offset = nonce_offset + NONCE_SIZE + FILE_SIZE_FIELD;
            let data_len = if data_offset < self.packet_size + 3 {
                self.packet_size + 3 - data_offset
            } else {
                0
            };
            
            if data_len > 0 {
                // Decrypt first packet data
                let decrypted_len = self.crypto.decrypt_chunk(
                    &self.buffer[data_offset..(data_offset + data_len)],
                    &mut self.data_buffer
                )?;
                
                // Write the decrypted data to flash
                if decrypted_len > 0 {
                    let write_address = self.target_address + (IMAGE_HEADER_SIZE as u32);
                    self.flash.write(write_address, &self.data_buffer[..decrypted_len])?;
                    
                    self.current_address = write_address + (decrypted_len as u32);
                    self.bytes_received = decrypted_len as u32;
                } else {
                    self.current_address = self.target_address + (IMAGE_HEADER_SIZE as u32);
                    self.bytes_received = 0;
                }
            } else {
                self.current_address = self.target_address + (IMAGE_HEADER_SIZE as u32);
                self.bytes_received = 0;
            }
            
            self.first_packet_complete = true;
        } else {
            // For subsequent packets
            if self.first_packet_complete {
                // Check if flash needs erasing (sector boundaries)
                if (self.current_address & 0x1FFFF) == 0 {
                    self.flash.erase(self.current_address)?;
                }
                
                // Regular data packet
                let data_offset = 3; // SOH/STX + SEQ + ~SEQ
                let data_len = self.packet_size;
                
                // Check if this is the last packet (contains auth tag)
                let remaining_bytes = self.file_size as usize - self.bytes_received as usize;
                
                if remaining_bytes <= self.packet_size {
                    // Last packet - contains authentication tag
                    let auth_offset = data_offset + remaining_bytes - AUTH_TAG_SIZE;
                    
                    // Extract authentication tag
                    if auth_offset + AUTH_TAG_SIZE <= data_offset + data_len {
                        self.auth_tag.copy_from_slice(&self.buffer[auth_offset..(auth_offset + AUTH_TAG_SIZE)]);
                        
                        // Decrypt data part (excluding auth tag)
                        if auth_offset > data_offset {
                            let data_part_len = auth_offset - data_offset;
                            let decrypted_len = self.crypto.decrypt_chunk(
                                &self.buffer[data_offset..auth_offset],
                                &mut self.data_buffer
                            )?;
                            
                            // Write to flash
                            if decrypted_len > 0 {
                                self.flash.write(self.current_address, &self.data_buffer[..decrypted_len])?;
                                self.current_address += decrypted_len as u32;
                                self.bytes_received += decrypted_len as u32;
                            }
                        }
                        
                        // Verify authentication tag
                        self.crypto.verify_tag(&self.auth_tag)?;
                        
                        // Finish decryption
                        self.crypto.finish_decryption()?;
                    } else {
                        return Err(XmodemError::BufferOverflow);
                    }
                } else {
                    // Regular packet - decrypt and write
                    let decrypted_len = self.crypto.decrypt_chunk(
                        &self.buffer[data_offset..(data_offset + data_len)],
                        &mut self.data_buffer
                    )?;
                    
                    // Write the data to flash
                    if decrypted_len > 0 {
                        self.flash.write(self.current_address, &self.data_buffer[..decrypted_len])?;
                        self.current_address += decrypted_len as u32;
                        self.bytes_received += decrypted_len as u32;
                    }
                }
            } else {
                return Err(XmodemError::InvalidPacketType);
            }
        }
        
        // Update total packet count and reset for next packet
        self.total_packet_count += 1;
        self.state = XmodemState::WaitHeaderByte;
        
        Ok(false) // Not finished yet
    }
    
    /// Send cancel command
    pub fn send_cancel(&self, tx_buffer: &mut RingBuffer) {
        for _ in 0..3 {
            tx_buffer.write(X_CAN);
        }
    }
    
    /// Send request for CRC mode
    pub fn send_crc_request(&self, tx_buffer: &mut RingBuffer) {
        tx_buffer.write(X_C);
    }
    
    /// Send acknowledgment
    pub fn send_ack(&self, tx_buffer: &mut RingBuffer) {
        tx_buffer.write(X_ACK);
    }
    
    /// Send negative acknowledgment 
    pub fn send_nak(&self, tx_buffer: &mut RingBuffer) {
        tx_buffer.write(X_NAK);
    }
    
    /// Main method to process a firmware download
    pub fn process_download(
        &mut self, 
        rx_buffer: &mut RingBuffer, 
        tx_buffer: &mut RingBuffer
    ) -> Result<bool, XmodemError> {
        // If first packet was not received, send 'C' to request CRC mode
        if !self.first_packet && !self.packet_received {
            self.send_crc_request(tx_buffer);
            self.first_packet = true;
        }
        
        // If packet was already received, process it and acknowledge
        if self.packet_received {
            let result = self.process_packet();
            
            match result {
                Ok(complete) => {
                    self.send_ack(tx_buffer);
                    return Ok(complete);
                },
                Err(e) => {
                    self.send_cancel(tx_buffer);
                    return Err(e);
                }
            }
        }
        
        // Process any available bytes in the rx buffer
        while let Some(byte) = rx_buffer.read() {
            let result = self.process_byte(byte);
            
            match result {
                Ok(true) => {
                    // EOT received, send ACK and signal completion
                    self.send_ack(tx_buffer);
                    return Ok(true);
                },
                Ok(false) => {
                    // Continue processing
                },
                Err(e) => {
                    // Error occurred, send cancel
                    self.send_cancel(tx_buffer);
                    return Err(e);
                }
            }
        }
        
        Ok(false) // Not finished yet
    }
    
    /// Method to set the expected image type
    pub fn set_expected_image_type(&mut self, image_type: u8) {
        self.expected_image_type = image_type;
        self.header_verified = false; // Reset verification status when changing type
    }
}