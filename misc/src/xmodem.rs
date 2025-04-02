#![no_std]

use crate::ring_buffer::RingBuffer;
use core::{cmp, convert::TryFrom, mem::size_of};
use crate::image::{
    ImageHeader, 
    IMAGE_MAGIC_LOADER, IMAGE_MAGIC_UPDATER, IMAGE_MAGIC_APP,
    IMAGE_TYPE_LOADER, IMAGE_TYPE_UPDATER, IMAGE_TYPE_APP
};

use aes_gcm::{
    aead::{AeadInPlace, KeyInit, Payload},
    Aes128Gcm, Key, Nonce, Tag,
};

// XMODEM constants
pub const X_SOH: u8 = 0x01; // Start of 128-byte packet
pub const X_STX: u8 = 0x02; // Start of 1024-byte packet
pub const X_EOT: u8 = 0x04; // End of transmission
pub const X_ACK: u8 = 0x06; // Acknowledge
pub const X_NAK: u8 = 0x15; // Not acknowledge
pub const X_CAN: u8 = 0x18; // Cancel
pub const X_C: u8 = 0x43;   // ASCII 'C' - Request with CRC

// Memory constants
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
pub const SLOT_1_APP_LOADER_ADDR: u32 = 0x08004000;

/// XMODEM state machine states
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum XmodemState {
    WaitHeaderByte,  // Waiting for SOH/STX
    WaitIndex1,      // Waiting for packet index 
    WaitIndex2,      // Waiting for complement of packet index
    ReadData,        // Reading data bytes
    WaitCRC1,        // Waiting for first CRC byte
    WaitCRC2,        // Waiting for second CRC byte
    ProcessPacket,   // Processing the complete packet
}

/// Error types for XMODEM operations
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
    
    // New error types for header verification
    InvalidImageType,       // Image type doesn't match expected
    InvalidMagic,           // Invalid magic number
    OlderVersion,           // Received version is not newer than installed
    HeaderCheckFailed,      // General header check failed
}

/// Update type (full or patch)
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum UpdateType {
    FullUpdate,
    PatchUpdate,
}

/// Flash operation trait for abstraction
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
    
    fn init_encryption(&mut self, key: &[u8], nonce: &[u8], header: &[u8]) -> Result<(), XmodemError>;
    fn encrypt_chunk(&mut self, data: &[u8], output: &mut [u8]) -> Result<usize, XmodemError>;
    fn get_tag(&mut self, tag: &mut [u8]) -> Result<(), XmodemError>;
    fn finish_encryption(&mut self) -> Result<(), XmodemError>;
}

/// AES-GCM Crypto implementation
pub struct AesGcmCrypto {
    cipher: Option<Aes128Gcm>,
    buffer: Vec<u8, BUFFER_SIZE>,
    nonce: [u8; NONCE_SIZE],
    tag: [u8; AUTH_TAG_SIZE],
    tag_calculated: bool,
    encrypt_mode: bool,
}

impl AesGcmCrypto {
    pub fn new() -> Self {
        Self {
            cipher: None,
            buffer: Vec::new(),
            nonce: [0; NONCE_SIZE],
            tag: [0; AUTH_TAG_SIZE],
            tag_calculated: false,
            encrypt_mode: false,
        }
    }
}

impl CryptoOperations for AesGcmCrypto {
    fn init_decryption(&mut self, key: &[u8], nonce: &[u8], header: &[u8]) -> Result<(), XmodemError> {
        // Create a new cipher instance
        let key = Key::<Aes128Gcm>::from_slice(key);
        self.cipher = Some(Aes128Gcm::new(key));
        
        // Copy the nonce
        self.nonce.copy_from_slice(nonce);
        
        // Clear the internal buffer
        self.buffer.clear();
        
        self.tag_calculated = false;
        self.encrypt_mode = false;
        
        Ok(())
    }
    
    fn decrypt_chunk(&mut self, data: &[u8], output: &mut [u8]) -> Result<usize, XmodemError> {
        if self.cipher.is_none() {
            return Err(XmodemError::DecryptionError);
        }
        
        // Instead of incremental decryption, we buffer the data
        // This is because AES-GCM requires the entire ciphertext before decryption
        let current_len = self.buffer.len();
        let bytes_to_add = cmp::min(data.len(), self.buffer.capacity() - current_len);
        
        if bytes_to_add == 0 {
            return Err(XmodemError::BufferOverflow);
        }
        
        // Add data to buffer
        for i in 0..bytes_to_add {
            self.buffer.push(data[i]);
        }
        
        // If we have enough data to fill the output buffer, decrypt it
        let bytes_to_decrypt = cmp::min(self.buffer.len(), output.len());
        if bytes_to_decrypt > 0 {
            let nonce = Nonce::<Aes128Gcm>::from_slice(&self.nonce);
            
            // Prepare a copy of the data to decrypt
            let mut to_decrypt = [0u8; PACKET_1K_SIZE];
            to_decrypt[..bytes_to_decrypt].copy_from_slice(&self.buffer[..bytes_to_decrypt]);
            
            // Decrypt in-place
            let aad = [];
            let payload = Payload { msg: &mut to_decrypt[..bytes_to_decrypt], aad: &aad };
            if let Err(_) = self.cipher.as_ref().unwrap().decrypt_in_place(nonce, payload) {
                return Err(XmodemError::DecryptionError);
            }
            
            // Copy the result to the output buffer
            output[..bytes_to_decrypt].copy_from_slice(&to_decrypt[..bytes_to_decrypt]);
            
            // Remove the processed data from the buffer
            for i in 0..self.buffer.len() - bytes_to_decrypt {
                self.buffer[i] = self.buffer[i + bytes_to_decrypt];
            }
            self.buffer.truncate(self.buffer.len() - bytes_to_decrypt);
            
            Ok(bytes_to_decrypt)
        } else {
            Ok(0)
        }
    }
    
    fn verify_tag(&mut self, expected_tag: &[u8]) -> Result<(), XmodemError> {
        if expected_tag.len() != AUTH_TAG_SIZE {
            return Err(XmodemError::AuthenticationError);
        }
        
        // Compare expected tag with calculated tag
        for i in 0..AUTH_TAG_SIZE {
            if expected_tag[i] != self.tag[i] {
                return Err(XmodemError::AuthenticationError);
            }
        }
        
        Ok(())
    }
    
    fn finish_decryption(&mut self) -> Result<(), XmodemError> {
        self.cipher = None;
        self.buffer.clear();
        self.tag_calculated = false;
        
        Ok(())
    }
    
    fn init_encryption(&mut self, key: &[u8], nonce: &[u8], header: &[u8]) -> Result<(), XmodemError> {
        // Create a new cipher instance
        let key = Key::<Aes128Gcm>::from_slice(key);
        self.cipher = Some(Aes128Gcm::new(key));
        
        // Copy the nonce
        self.nonce.copy_from_slice(nonce);
        
        // Clear the internal buffer
        self.buffer.clear();
        
        self.tag_calculated = false;
        self.encrypt_mode = true;
        
        Ok(())
    }
    
    fn encrypt_chunk(&mut self, data: &[u8], output: &mut [u8]) -> Result<usize, XmodemError> {
        if self.cipher.is_none() {
            return Err(XmodemError::DecryptionError);
        }
        
        // Similar to decryption, we buffer the data
        let current_len = self.buffer.len();
        let bytes_to_add = cmp::min(data.len(), self.buffer.capacity() - current_len);
        
        if bytes_to_add == 0 {
            return Err(XmodemError::BufferOverflow);
        }
        
        // Add data to buffer
        for i in 0..bytes_to_add {
            self.buffer.push(data[i]);
        }
        
        // If we have enough data to fill the output buffer, encrypt it
        let bytes_to_encrypt = cmp::min(self.buffer.len(), output.len());
        if bytes_to_encrypt > 0 {
            let nonce = Nonce::<Aes128Gcm>::from_slice(&self.nonce);
            
            // Prepare a copy of the data to encrypt
            let mut to_encrypt = [0u8; PACKET_1K_SIZE];
            to_encrypt[..bytes_to_encrypt].copy_from_slice(&self.buffer[..bytes_to_encrypt]);
            
            // Encrypt in-place
            let aad = [];
            let payload = Payload { msg: &mut to_encrypt[..bytes_to_encrypt], aad: &aad };
            if let Err(_) = self.cipher.as_ref().unwrap().encrypt_in_place(nonce, payload) {
                return Err(XmodemError::DecryptionError);
            }
            
            // Copy the result to the output buffer
            output[..bytes_to_encrypt].copy_from_slice(&to_encrypt[..bytes_to_encrypt]);
            
            // Remove the processed data from the buffer
            for i in 0..self.buffer.len() - bytes_to_encrypt {
                self.buffer[i] = self.buffer[i + bytes_to_encrypt];
            }
            self.buffer.truncate(self.buffer.len() - bytes_to_encrypt);
            
            Ok(bytes_to_encrypt)
        } else {
            Ok(0)
        }
    }
    
    fn get_tag(&mut self, tag: &mut [u8]) -> Result<(), XmodemError> {
        if !self.tag_calculated {
            return Err(XmodemError::DecryptionError);
        }
        
        // Copy the calculated tag
        tag[..AUTH_TAG_SIZE].copy_from_slice(&self.tag);
        
        Ok(())
    }
    
    fn finish_encryption(&mut self) -> Result<(), XmodemError> {
        self.cipher = None;
        self.buffer.clear();
        self.tag_calculated = false;
        
        Ok(())
    }
}

/// XMODEM receiver for handling firmware updates
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
    pub update_type: UpdateType,
    pub first_packet_complete: bool,
    
    // Image header verification
    pub expected_image_type: u8,
    pub header_verified: bool,
    
    // Encryption state
    pub key: [u8; 16],
    pub nonce: [u8; NONCE_SIZE],
    pub auth_tag: [u8; AUTH_TAG_SIZE],
    pub calculated_tag: [u8; AUTH_TAG_SIZE],
    pub aad_header: [u8; 16],
    
    // Tracking variables
    data_counter: usize,
    total_packet_count: u32,
    remaining_encrypted_length: u32,
    remaining_packets: u32,
    remaining_bytes_in_last_packet: u32,
    
    // External dependencies
    flash: F,
    crypto: C,
}

impl<F: FlashOperations, C: CryptoOperations> XmodemReceiver<F, C> {
    /// Create a new XmodemReceiver
    pub fn new(target_address: u32, update_type: UpdateType, flash: F, crypto: C) -> Self {
        // Define expected image type based on update type
        let expected_image_type = match update_type {
            UpdateType::FullUpdate => IMAGE_TYPE_APP,
            UpdateType::PatchUpdate => IMAGE_TYPE_UPDATER,
        };
        
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
            update_type,
            first_packet_complete: false,
            
            // Fields for header verification
            expected_image_type,
            header_verified: false,
            
            key: [
                0xAC, 0x00, 0xD6, 0x7F, 0x21, 0xD2, 0x94, 0x46,
                0x2F, 0x2A, 0xB6, 0x84, 0xE2, 0xE7, 0x83, 0xF7
            ],
            nonce: [0x76, 0x4c, 0x10, 0x12, 0x61, 0x75, 0xc3, 0xa1, 0xc3, 0x94, 0x5f, 0x06],
            auth_tag: [0; AUTH_TAG_SIZE],
            calculated_tag: [0; AUTH_TAG_SIZE],
            aad_header: [
                0x02, 0x00, 0x07, 0x07, 0x60, 0x61, 0x5F, 0x6B,
                0x65, 0x73, 0x01, 0x00, 0x00, 0x01, 0xFF, 0xFF
            ],
            
            data_counter: 0,
            total_packet_count: 0,
            remaining_encrypted_length: 0,
            remaining_packets: 0,
            remaining_bytes_in_last_packet: 0,
            
            flash,
            crypto,
        }
    }
    
    /// Calculate CRC16 for XMODEM (CCITT polynomial)
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

    /// Prepare the AAD header with firmware version information
    pub fn prepare_header(&mut self) -> Result<(), XmodemError> {
        // In a real implementation, update the AAD header with version info
        // For now, we'll use the predefined header from initialization
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
    
    /// Verify image header before erasing flash
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
                if header.version_major < current_header.version_major {
                    return Err(XmodemError::OlderVersion);
                }
                
                if header.version_major == current_header.version_major && 
                   header.version_minor < current_header.version_minor {
                    return Err(XmodemError::OlderVersion);
                }
                
                if header.version_major == current_header.version_major && 
                   header.version_minor == current_header.version_minor &&
                   header.version_patch <= current_header.version_patch {
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
            
            let header_data = &self.buffer[header_offset..(header_offset + IMAGE_HEADER_SIZE)];
            
            // Verify the header before erasing anything
            if !self.header_verified {
                match self.verify_image_header(header_data) {
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
            
            // Extract nonce and file size from the first packet
            self.nonce.copy_from_slice(&self.buffer[header_offset..(header_offset + NONCE_SIZE)]);
            
            // Extract file size (encrypted data + tag)
            self.file_size = self.extract_size(header_offset + NONCE_SIZE);
            
            // Check file size limit
            if self.file_size > 262144 {
                return Err(XmodemError::FileSizeTooLarge);
            }
            
            // Calculate total encrypted length (excluding authentication tag)
            self.remaining_encrypted_length = self.file_size - AUTH_TAG_SIZE as u32;
            
            // Calculate total packets and remaining bytes
            self.remaining_packets = (self.file_size + NONCE_SIZE as u32 + FILE_SIZE_FIELD as u32) / self.packet_size as u32;
            self.remaining_bytes_in_last_packet = (self.file_size + NONCE_SIZE as u32 + FILE_SIZE_FIELD as u32) % self.packet_size as u32;
            
            if self.remaining_bytes_in_last_packet != 0 {
                self.remaining_packets += 1;
            }
            
            // Update AAD header with current firmware version
            self.prepare_header()?;
            
            // Initialize decryption
            self.crypto.init_decryption(&self.key, &self.nonce, &self.aad_header)?;
            
            // Write the header to flash
            self.flash.write(self.target_address, header_data)?;
            
            // Process the data part in the first packet
            let data_offset = header_offset + IMAGE_HEADER_SIZE;
            let data_len = self.packet_size - IMAGE_HEADER_SIZE;
            
            if data_len > 0 {
                // Decrypt first packet data
                let decrypted_len = self.crypto.decrypt_chunk(
                    &self.buffer[data_offset..(data_offset + data_len)],
                    &mut self.data_buffer
                )?;
                
                // Write the decrypted data to flash
                self.flash.write(
                    self.target_address + IMAGE_HEADER_SIZE as u32,
                    &self.data_buffer[..decrypted_len]
                )?;
                
                self.current_address = self.target_address + IMAGE_HEADER_SIZE as u32 + decrypted_len as u32;
                self.bytes_received = decrypted_len as u32;
            } else {
                self.current_address = self.target_address + IMAGE_HEADER_SIZE as u32;
                self.bytes_received = 0;
            }
            
            self.first_packet_complete = true;
        } else {
            // For subsequent packets
            if self.first_packet_complete {
                // Check if flash needs erasing (every 4K bytes or so)
                if self.total_packet_count % 4 == 0 {
                    self.flash.erase(self.current_address)?;
                }
                
                // Process the last packet differently if it contains authentication tag
                if self.remaining_packets == 1 && self.remaining_bytes_in_last_packet > 0 {
                    let data_len = self.remaining_bytes_in_last_packet as usize - AUTH_TAG_SIZE;
                    
                    // Decrypt data part
                    if data_len > 0 {
                        let decrypted_len = self.crypto.decrypt_chunk(
                            &self.buffer[3..(3 + data_len)],
                            &mut self.data_buffer
                        )?;
                        
                        // Write to flash
                        self.flash.write(self.current_address, &self.data_buffer[..decrypted_len])?;
                        self.current_address += decrypted_len as u32;
                    }
                    
                    // Extract authentication tag
                    self.auth_tag.copy_from_slice(&self.buffer[3 + data_len..(3 + data_len + AUTH_TAG_SIZE)]);
                    
                    // Verify authentication tag
                    self.crypto.verify_tag(&self.auth_tag)?;
                    
                    // Finish decryption
                    self.crypto.finish_decryption()?;
                } else {
                    // Regular packet processing
                    let data_len = self.packet_size;
                    let decrypted_len = self.crypto.decrypt_chunk(
                        &self.buffer[3..(3 + data_len)],
                        &mut self.data_buffer
                    )?;
                    
                    // Write the data to flash
                    self.flash.write(self.current_address, &self.data_buffer[..decrypted_len])?;
                    self.current_address += decrypted_len as u32;
                    self.bytes_received += decrypted_len as u32;
                    
                    // Update remaining counters
                    if self.remaining_encrypted_length >= decrypted_len as u32 {
                        self.remaining_encrypted_length -= decrypted_len as u32;
                    }
                    
                    if self.remaining_packets > 0 {
                        self.remaining_packets -= 1;
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
    
    /// Method to upload firmware to a host
    pub fn upload_firmware(
        &mut self,
        rx_buffer: &mut RingBuffer,
        tx_buffer: &mut RingBuffer,
        source_address: u32,
        max_size: usize
    ) -> Result<bool, XmodemError> {
        // Wait for 'C' from receiver to start
        if !self.first_packet {
            if let Some(byte) = rx_buffer.read() {
                if byte == X_C {
                    self.first_packet = true;
                } else {
                    return Ok(false); // Keep waiting
                }
            } else {
                return Ok(false); // Keep waiting
            }
        }
        
        // Read data from source
        let src_ptr = source_address as *const u8;
        let mut buffer = [0u8; PACKET_128_SIZE + PACKET_OVERHEAD];
        
        // Set up the packet
        buffer[0] = X_SOH; // Use 128-byte packets for simplicity
        
        let packet_idx = (self.total_packet_count % 255) as u8 + 1;
        buffer[1] = packet_idx;
        buffer[2] = 255 - packet_idx;
        
        // Read and encrypt data
        let offset = self.total_packet_count as usize * PACKET_128_SIZE;
        if offset >= max_size {
            // All data sent, send EOT
            tx_buffer.write(X_EOT);
            
            // Wait for ACK
            while let Some(byte) = rx_buffer.read() {
                if byte == X_ACK {
                    return Ok(true); // Completed
                }
            }
            
            return Ok(false); // Wait for ACK
        }
        
        let bytes_to_read = cmp::min(PACKET_128_SIZE, max_size - offset);
        
        if !self.first_packet_complete {
            // First packet needs special handling for the header
            // Add nonce and file size
            for i in 0..NONCE_SIZE {
                buffer[3 + i] = self.nonce[i];
            }
            
            // File size (big endian)
            let file_size = max_size as u32;
            buffer[3 + NONCE_SIZE] = (file_size >> 24) as u8;
            buffer[3 + NONCE_SIZE + 1] = (file_size >> 16) as u8;
            buffer[3 + NONCE_SIZE + 2] = (file_size >> 8) as u8;
            buffer[3 + NONCE_SIZE + 3] = file_size as u8;
            
            // Initialize encryption
            self.prepare_header()?;
            self.crypto.init_encryption(&self.key, &self.nonce, &self.aad_header)?;
            
            // Read and encrypt actual data
            let data_offset = HEADER_SIZE;
            let data_to_read = bytes_to_read - data_offset;
            
            for i in 0..data_to_read {
                unsafe {
                    self.data_buffer[i] = *src_ptr.add(i);
                }
            }
            
            // Encrypt data
            let encrypted_len = self.crypto.encrypt_chunk(
                &self.data_buffer[..data_to_read],
                &mut buffer[3 + data_offset..]
            )?;
            
            // Fill the rest with 0x1A (^Z) padding
            for i in (3 + data_offset + encrypted_len)..buffer.len() {
                buffer[i] = 0x1A;
            }
            
            self.first_packet_complete = true;
        } else {
            // Regular data packet
            
            // Read data from source
            for i in 0..bytes_to_read {
                unsafe {
                    self.data_buffer[i] = *src_ptr.add(offset + i);
                }
            }
            
            // Check if this is the last packet
            let remaining = max_size - offset;
            if remaining <= PACKET_128_SIZE {
                // Last packet - leave space for the auth tag
                let payload_size = remaining - AUTH_TAG_SIZE;
                
                // Encrypt data
                let encrypted_len = self.crypto.encrypt_chunk(
                    &self.data_buffer[..payload_size],
                    &mut buffer[3..]
                )?;
                
                // Get authentication tag and append it
                let mut tag = [0u8; AUTH_TAG_SIZE];
                self.crypto.get_tag(&mut tag)?;
                
                // Copy tag to buffer
                for i in 0..AUTH_TAG_SIZE {
                    buffer[3 + encrypted_len + i] = tag[i];
                }
                
                // Finish encryption
                self.crypto.finish_encryption()?;
                
                // Fill the rest with 0x1A (^Z) padding
                for i in (3 + encrypted_len + AUTH_TAG_SIZE)..buffer.len() {
                    buffer[i] = 0x1A;
                }
            } else {
                // Regular packet
                let encrypted_len = self.crypto.encrypt_chunk(
                    &self.data_buffer[..bytes_to_read],
                    &mut buffer[3..]
                )?;
                
                // Fill the rest with 0x1A (^Z) padding
                for i in (3 + encrypted_len)..buffer.len() {
                    buffer[i] = 0x1A;
                }
            }
        }
        
        // Calculate and set CRC
        let crc = self.calculate_crc16(&buffer[3..(3 + PACKET_128_SIZE)]);
        buffer[3 + PACKET_128_SIZE] = (crc >> 8) as u8;
        buffer[3 + PACKET_128_SIZE + 1] = crc as u8;
        
        // Send the packet
        for b in buffer.iter() {
            tx_buffer.write(*b);
        }
        
        // Wait for ACK/NAK
        while let Some(byte) = rx_buffer.read() {
            if byte == X_ACK {
                // Packet acknowledged, move to next
                self.total_packet_count += 1;
                return Ok(false); // Continue with next packet
            } else if byte == X_NAK {
                // Resend the packet
                for b in buffer.iter() {
                    tx_buffer.write(*b);
                }
            }
        }
        
        Ok(false) // Continue
    }
    
    /// Method to set the expected image type
    pub fn set_expected_image_type(&mut self, image_type: u8) {
        self.expected_image_type = image_type;
        self.header_verified = false; // Reset verification status when changing type
    }
}