use aes_gcm::{
    aead::{Aead, KeyInit, Payload},
    Aes128Gcm, Key, Nonce, Tag,
};
use core::convert::TryInto;

// XMODEM constants
pub const X_SOH: u8 = 0x01; // Start of header
pub const X_EOT: u8 = 0x04; // End of transmission
pub const X_ACK: u8 = 0x06; // Acknowledge
pub const X_NAK: u8 = 0x15; // Negative acknowledge
pub const X_CAN: u8 = 0x18; // Cancel
pub const X_C: u8 = 0x43;   // ASCII 'C' - request CRC mode

// Memory addresses - update these based on your actual memory layout
pub const SLOT_2_APP_ADDR: u32 = 0x08020200;
pub const SLOT_2_VER_ADDR: u32 = 0x08020000;
pub const UPDATER_ADDR: u32 = 0x08008000;
pub const PATCH_ADDR: u32 = 0x08040000;
pub const BACKUP_ADDR: u32 = 0x08060000;
pub const BUFFER_SIZE: usize = 2048;

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum XmodemState {
    WaitSOH,
    WaitIndex1,
    WaitIndex2,
    ReadData,
    WaitCRC,
}

#[derive(Debug)]
pub enum XmodemError {
    IndexError,
    CrcError,
    DecryptionError,
    AuthenticationError,
    FlashError,
    FileSizeTooLarge,
}

pub struct XmodemReceiver {
    // XMODEM state
    pub state: XmodemState,
    pub prev_index1: u8,
    pub prev_index2: u8,
    pub data_counter: u8,
    pub a_read_rx_data: [u8; 133],
    pub a_encrypted_data: [u8; 128],
    pub a_decrypted_data: [u8; 128],
    pub packet_received: bool,
    pub copy_data: bool,
    pub first_packet: bool,
    pub crc_received: u16,
    pub crc_calculated: u16,
    pub total_packet_count: u32,
    
    // AES-GCM state
    pub key: [u8; 16],
    pub nonce_counter: [u8; 12],
    pub header_aes: [u8; 16],
    pub received_tag: [u8; 16],
    pub calculated_tag: [u8; 16],
    pub cipher: Option<Aes128Gcm>,
    
    // Firmware state
    pub current_address: u32,
    pub file_size: u32,
    pub total_encrypted_length: u32,
    pub total_packets: u32,
    pub remaining_encrypted_length: u32,
    pub remaining_packets: u32,
    pub remaining_bytes_in_last_packet: u32,
    pub first_packet_complete: bool,
    pub update_type: UpdateType,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum UpdateType {
    FullUpdate,    // Update entire application
    PatchUpdate,   // Apply a patch to existing application
}

impl XmodemReceiver {
    pub fn new(target_address: u32, update_type: UpdateType) -> Self {
        Self {
            // XMODEM state
            state: XmodemState::WaitSOH,
            prev_index1: 0,
            prev_index2: 0xFF,
            data_counter: 0,
            a_read_rx_data: [0; 133],
            a_encrypted_data: [0; 128],
            a_decrypted_data: [0; 128],
            packet_received: false,
            copy_data: true,
            first_packet: false,
            crc_received: 0,
            crc_calculated: 0,
            total_packet_count: 0,
            
            // AES-GCM state
            key: [0xAC, 0x00, 0xD6, 0x7F, 0x21, 0xD2, 0x94, 0x46, 0x2F, 0x2A, 0xB6, 0x84, 0xE2, 0xE7, 0x83, 0xF7],
            nonce_counter: [0x76, 0x4C, 0x10, 0x12, 0x61, 0x75, 0xC3, 0xA1, 0xC3, 0x94, 0x5F, 0x06],
            header_aes: [0x02, 0x00, 0x07, 0x07, 0x60, 0x61, 0x5F, 0x6B, 0x65, 0x73, 0x01, 0x00, 0x00, 0x01, 0xFF, 0xFF],
            received_tag: [0; 16],
            calculated_tag: [0; 16],
            cipher: None,
            
            // Firmware state
            current_address: target_address,
            file_size: 0,
            total_encrypted_length: 0,
            total_packets: 0,
            remaining_encrypted_length: 0,
            remaining_packets: 0,
            remaining_bytes_in_last_packet: 0,
            first_packet_complete: false,
            update_type,
        }
    }
    
    pub fn init_cipher(&mut self) -> Result<(), XmodemError> {
        let key = Key::<Aes128Gcm>::from_slice(&self.key);
        self.cipher = Some(Aes128Gcm::new(key));
        Ok(())
    }
    
    // Prepare AES-GCM header based on version info
    pub fn prepare_header(&mut self) {
        // In C this reads hardware ID and version info
        // Simplified for Rust implementation
        self.header_aes = [0x02, 0x00, 0x07, 0x07, 0x60, 0x61, 0x5F, 0x6B, 0x65, 0x73, 0x01, 0x00, 0x00, 0x01, 0xFF, 0xFF];
    }
    
    // Extract size from received data
    pub fn extract_size(&self, data: &[u8]) -> u32 {
        let size_bytes = &data[12..16];
        ((size_bytes[0] as u32) << 24) | 
        ((size_bytes[1] as u32) << 16) | 
        ((size_bytes[2] as u32) << 8) | 
        (size_bytes[3] as u32)
    }
    
    // Calculate CRC16 for XMODEM
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
        crc & 0xFFFF
    }
    
    // Decrypt firmware data in real-time
    pub fn decrypt_firmware(&mut self, encrypted_data: &[u8], length: usize) -> Result<Vec<u8>, XmodemError> {
        if self.cipher.is_none() {
            self.init_cipher()?;
        }
        
        if let Some(ref cipher) = self.cipher {
            let nonce = Nonce::from_slice(&self.nonce_counter);
            let payload = Payload {
                msg: encrypted_data,
                aad: &self.header_aes,
            };
            
            match cipher.decrypt(nonce, payload) {
                Ok(decrypted) => Ok(decrypted),
                Err(_) => Err(XmodemError::DecryptionError),
            }
        } else {
            Err(XmodemError::DecryptionError)
        }
    }
    
    // Process one byte of incoming data
    pub fn process_byte(&mut self, rx_byte: u8, rx_buffer: &mut RingBuffer, tx_buffer: &mut RingBuffer, 
                        flash_erase: impl Fn(u32) -> Result<(), XmodemError>,
                        flash_write: impl Fn(&[u8], u32) -> Result<(), XmodemError>) -> Result<bool, XmodemError> {
        
        // Check for end of transmission
        if rx_byte == X_EOT && self.state == XmodemState::WaitSOH {
            // Send ACK to acknowledge EOT
            tx_buffer.write(X_ACK);
            tx_buffer.transmit();
            
            // Verify authentication tag
            let auth_result = self.verify_authentication();
            
            // Reset state
            self.reset_state();
            
            // Return result based on authentication
            if auth_result.is_ok() {
                // Handle update completion based on update type
                match self.update_type {
                    UpdateType::FullUpdate => {
                        // For full update, we just return success
                        Ok(true)
                    },
                    UpdateType::PatchUpdate => {
                        // For patch update, we need to apply the patch
                        // This would be handled externally - just signal completion
                        Ok(true)
                    }
                }
            } else {
                // Authentication failed, erase the received data
                self.handle_auth_failure(flash_erase)?;
                Err(XmodemError::AuthenticationError)
            }
        }
        
        // If we're copying data from the buffer, do that
        if self.copy_data {
            // Copy data from ring buffer to our processing buffer
            for i in 0..133 {
                if let Some(byte) = rx_buffer.read() {
                    self.a_read_rx_data[i] = byte;
                }
            }
            self.packet_received = false;
            // Empty the ring buffer
            rx_buffer.clear();
        }
        
        // Process based on current state
        match self.state {
            XmodemState::WaitSOH => {
                if self.a_read_rx_data[0] == X_SOH {
                    self.state = XmodemState::WaitIndex1;
                    self.packet_received = true;
                    self.copy_data = false;
                } else {
                    self.packet_received = false;
                }
            },
            XmodemState::WaitIndex1 => {
                // Check if index is correct
                if (self.prev_index1 + 1) == self.a_read_rx_data[1] {
                    self.state = XmodemState::WaitIndex2;
                    self.prev_index1 += 1;
                } else if (self.prev_index1 + 1) == 256 && self.a_read_rx_data[1] == 0 {
                    self.state = XmodemState::WaitIndex2;
                    self.prev_index1 = 0;
                } else {
                    // Send cancel
                    self.send_cancel(tx_buffer);
                    return Err(XmodemError::IndexError);
                }
            },
            XmodemState::WaitIndex2 => {
                // Check if inverted index is correct
                if (self.prev_index2 as i16 - 1) == (self.a_read_rx_data[2] as i16) {
                    self.state = XmodemState::ReadData;
                    self.prev_index2 = self.prev_index2.wrapping_sub(1);
                } else if (self.prev_index2 as i16 - 1) == -1 && self.a_read_rx_data[2] == 255 {
                    self.state = XmodemState::ReadData;
                    self.prev_index2 = 255;
                } else {
                    // Send cancel
                    self.send_cancel(tx_buffer);
                    return Err(XmodemError::IndexError);
                }
            },
            XmodemState::ReadData => {
                // Copy data from packet
                for i in 0..128 {
                    self.a_encrypted_data[i] = self.a_read_rx_data[i + 3];
                }
                self.state = XmodemState::WaitCRC;
            },
            XmodemState::WaitCRC => {
                // Verify CRC
                self.crc_received = ((self.a_read_rx_data[131] as u16) << 8) | (self.a_read_rx_data[132] as u16);
                self.crc_calculated = self.calculate_crc16(&self.a_encrypted_data);
                
                if self.crc_received != self.crc_calculated {
                    // Send cancel
                    self.send_cancel(tx_buffer);
                    return Err(XmodemError::CrcError);
                }
                
                // Process the first packet specially
                if self.a_read_rx_data[1] == 0x1 {
                    // First packet needs special handling to extract nonce and size
                    self.process_first_packet(flash_erase, flash_write)?;
                } else {
                    // Process subsequent packets
                    self.process_subsequent_packet(flash_write)?;
                }
                
                // Reset for next packet
                self.state = XmodemState::WaitSOH;
                self.packet_received = true;
                self.copy_data = true;
                self.total_packet_count += 1;
            }
        }
        
        Ok(false) // Not finished yet
    }
    
    // Process the first packet which contains nonce and size
    fn process_first_packet<F, W>(&mut self, flash_erase: F, flash_write: W) -> Result<(), XmodemError>
    where
        F: Fn(u32) -> Result<(), XmodemError>,
        W: Fn(&[u8], u32) -> Result<(), XmodemError>,
    {
        // Check if flash needs to be erased
        for i in 0..BUFFER_SIZE {
            if unsafe { *((self.current_address + i as u32) as *const u8) } != 0xFF {
                flash_erase(self.current_address)?;
                break;
            }
        }
        
        // If this is the very first packet (firmware image start)
        if unsafe { *(self.current_address as *const u32) } == 0xFFFFFFFF {
            // Extract nonce from the first 12 bytes
            self.nonce_counter.copy_from_slice(&self.a_encrypted_data[0..12]);
            
            // Extract file size (encrypted data + tag)
            self.file_size = self.extract_size(&self.a_encrypted_data);
            
            // Verify file size isn't too large (for example > 256KB)
            if self.file_size > 262144 {
                self.send_cancel_with_message("File size too large");
                return Err(XmodemError::FileSizeTooLarge);
            }
            
            // Calculate total encrypted length and packets
            self.total_encrypted_length = self.file_size - 16; // Subtract tag length
            self.total_packets = (self.file_size + 12 + 4) / 128; // Add nonce and size
            
            // Calculate remaining bytes in last packet
            self.remaining_bytes_in_last_packet = (self.file_size + 12 + 4) % 128;
            if self.remaining_bytes_in_last_packet != 0 && self.total_packets != 0 {
                self.total_packets += 1;
            }
            
            // Calculate length to decrypt from first packet
            let mut length_to_decrypt = 0;
            for i in (12 + 4)..128 {
                if i + 3 >= self.a_encrypted_data.len() {
                    break;
                }
                if self.a_encrypted_data[i] == 0x1A && 
                   self.a_encrypted_data[i + 1] == 0x1A && 
                   self.a_encrypted_data[i + 2] == 0x1A && 
                   self.a_encrypted_data[i + 3] == 0x1A {
                    break;
                }
                length_to_decrypt += 1;
            }
            
            // Adjust for only one packet
            if self.file_size <= (128 - 16 - 4) {
                length_to_decrypt -= 16;
            }
            
            // Prepare AES header with current values
            self.prepare_header();
            
            // Initialize cipher and decrypt first chunk
            self.init_cipher()?;
            
            if let Some(decrypted) = self.decrypt_chunk(&self.a_encrypted_data[16..16+length_to_decrypt])? {
                // Write decrypted data to flash
                flash_write(&decrypted, self.current_address)?;
                self.current_address += length_to_decrypt as u32;
                
                // Update remaining values
                self.remaining_encrypted_length = self.total_encrypted_length - length_to_decrypt as u32;
                self.remaining_packets = self.total_packets - 1;
            }
        } else {
            // This is not the first packet, decrypt rest of firmware
            self.decrypt_rest_of_firmware(flash_write)?;
        }
        
        Ok(())
    }
    
    // Process subsequent packets
    fn process_subsequent_packet<W>(&mut self, flash_write: W) -> Result<(), XmodemError>
    where
        W: Fn(&[u8], u32) -> Result<(), XmodemError>,
    {
        // Decrypt and write subsequent data
        self.decrypt_rest_of_firmware(flash_write)
    }
    
    // Decrypt rest of firmware
    fn decrypt_rest_of_firmware<W>(&mut self, flash_write: W) -> Result<(), XmodemError>
    where
        W: Fn(&[u8], u32) -> Result<(), XmodemError>,
    {
        // Initialize length to decrypt
        let length_to_decrypt = if self.remaining_packets > 0 {
            128
        } else {
            self.remaining_bytes_in_last_packet
        };
        
        if length_to_decrypt == 0 {
            return Ok(());
        }
        
        // Check if this is the last packet with tag
        if self.remaining_packets == 0 && self.remaining_bytes_in_last_packet <= 16 {
            // Save authentication tag
            let tag_position = self.remaining_bytes_in_last_packet;
            if tag_position != 0 {
                self.received_tag.copy_from_slice(&self.a_encrypted_data[0..16]);
            } else {
                self.received_tag.copy_from_slice(&self.a_encrypted_data[length_to_decrypt as usize - 16..length_to_decrypt as usize]);
            }
            
            // Finalize decryption and get auth tag
            self.finish_decryption()?;
        } else if self.remaining_packets == 1 && self.remaining_bytes_in_last_packet < 128 {
            // Last packet with data and tag
            let encrypted_data_length = self.remaining_bytes_in_last_packet - 16;
            if encrypted_data_length > 0 {
                if let Some(decrypted) = self.decrypt_chunk(&self.a_encrypted_data[0..encrypted_data_length as usize])? {
                    flash_write(&decrypted, self.current_address)?;
                    self.current_address += encrypted_data_length as u32;
                }
            }
            
            // Save authentication tag
            self.received_tag.copy_from_slice(&self.a_encrypted_data[encrypted_data_length as usize..encrypted_data_length as usize + 16]);
            
            // Finalize decryption
            self.finish_decryption()?;
        } else {
            // Regular packet
            if let Some(decrypted) = self.decrypt_chunk(&self.a_encrypted_data[0..length_to_decrypt as usize])? {
                flash_write(&decrypted, self.current_address)?;
                self.current_address += length_to_decrypt as u32;
                
                self.remaining_encrypted_length -= length_to_decrypt as u32;
                self.remaining_packets -= 1;
                
                // Handle last packet with remaining bytes
                if self.remaining_packets == 0 && self.remaining_bytes_in_last_packet > 16 {
                    let last_chunk_size = self.remaining_bytes_in_last_packet - 16;
                    if last_chunk_size > 0 {
                        if let Some(decrypted) = self.decrypt_chunk(&self.a_encrypted_data[0..last_chunk_size as usize])? {
                            flash_write(&decrypted, self.current_address)?;
                            self.current_address += last_chunk_size as u32;
                            
                            // Save tag
                            self.received_tag.copy_from_slice(&self.a_encrypted_data[last_chunk_size as usize..last_chunk_size as usize + 16]);
                        }
                    }
                    
                    // Finalize decryption
                    self.finish_decryption()?;
                }
            }
        }
        
        Ok(())
    }
    
    // Decrypt a chunk of data
    fn decrypt_chunk(&mut self, data: &[u8]) -> Result<Option<Vec<u8>>, XmodemError> {
        if let Some(ref cipher) = self.cipher {
            let nonce = Nonce::from_slice(&self.nonce_counter);
            let payload = Payload {
                msg: data,
                aad: &self.header_aes,
            };
            
            match cipher.decrypt(nonce, payload) {
                Ok(decrypted) => Ok(Some(decrypted)),
                Err(_) => Err(XmodemError::DecryptionError),
            }
        } else {
            Err(XmodemError::DecryptionError)
        }
    }
    
    // Finalize decryption and get authentication tag
    fn finish_decryption(&mut self) -> Result<(), XmodemError> {
        // In aes-gcm crate, we would verify tag during decrypt operations
        // This is a placeholder for that functionality
        Ok(())
    }
    
    // Verify authentication tag
    fn verify_authentication(&self) -> Result<(), XmodemError> {
        // In real implementation, we would compare received tag with calculated tag
        // For now, we'll just return success
        Ok(())
    }
    
    // Handle authentication failure
    fn handle_auth_failure<F>(&self, flash_erase: F) -> Result<(), XmodemError>
    where
        F: Fn(u32) -> Result<(), XmodemError>,
    {
        // Based on update type, erase the appropriate sections
        match self.update_type {
            UpdateType::FullUpdate => {
                // Erase app sections
                flash_erase(SLOT_2_APP_ADDR - 0x20000)?;
                flash_erase(SLOT_2_APP_ADDR)?;
                flash_erase(SLOT_2_APP_ADDR + 0x20000)?;
                
                // Load backup if available
                self.load_backup()?;
            },
            UpdateType::PatchUpdate => {
                // Erase only patch section
                flash_erase(PATCH_ADDR)?;
            }
        }
        
        Ok(())
    }
    
    // Load backup firmware
    fn load_backup(&self) -> Result<(), XmodemError> {
        // In real implementation, this would copy from backup to app
        // For now, it's just a placeholder
        Ok(())
    }
    
    // Send cancel command
    fn send_cancel(&self, tx_buffer: &mut RingBuffer) {
        tx_buffer.write(X_CAN);
        tx_buffer.write(X_CAN);
        tx_buffer.write(X_CAN);
        tx_buffer.transmit();
    }
    
    // Send cancel with message
    fn send_cancel_with_message(&self, message: &str) {
        // In real implementation, this would send cancel and message
        // For now, it's just a placeholder
    }
    
    // Reset state after completion
    fn reset_state(&mut self) {
        self.first_packet = false;
        self.copy_data = true;
        self.packet_received = false;
        self.prev_index1 = 0;
        self.prev_index2 = 0xFF;
        self.data_counter = 0;
        self.total_packets = 0;
        self.cipher = None;
        self.calculated_tag = [0; 16];
        self.a_encrypted_data = [0; 128];
    }
}

// Ring buffer implementation for UART communication
pub struct RingBuffer {
    buffer: [u8; 256],
    head: usize,
    tail: usize,
    count: usize,
}

impl RingBuffer {
    pub fn new() -> Self {
        Self {
            buffer: [0; 256],
            head: 0,
            tail: 0,
            count: 0,
        }
    }
    
    pub fn write(&mut self, byte: u8) -> bool {
        if self.count < 256 {
            self.buffer[self.head] = byte;
            self.head = (self.head + 1) % 256;
            self.count += 1;
            true
        } else {
            false
        }
    }
    
    pub fn read(&mut self) -> Option<u8> {
        if self.count > 0 {
            let byte = self.buffer[self.tail];
            self.tail = (self.tail + 1) % 256;
            self.count -= 1;
            Some(byte)
        } else {
            None
        }
    }
    
    pub fn len(&self) -> usize {
        self.count
    }
    
    pub fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.count = 0;
    }
    
    pub fn transmit(&mut self) {
        // In real implementation, this would trigger UART transmission
        // For now, it's just a placeholder
    }
}