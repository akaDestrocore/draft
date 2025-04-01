#![no_std]

use crate::ring_buffer::RingBuffer;
use core::cmp;

// XMODEM constants
pub const X_SOH: u8 = 0x01;
pub const X_STX: u8 = 0x02;
pub const X_EOT: u8 = 0x04;
pub const X_ACK: u8 = 0x06;
pub const X_NAK: u8 = 0x15;
pub const X_CAN: u8 = 0x18;
pub const X_C: u8 = 0x43;

pub const BUFFER_SIZE: usize = 2048;
pub const SLOT_2_APP_ADDR: u32 = 0x08020200;
pub const SLOT_2_VER_ADDR: u32 = 0x08020000;
pub const UPDATER_ADDR: u32 = 0x08008000;
pub const PATCH_ADDR: u32 = 0x08040000;
pub const BACKUP_ADDR: u32 = 0x08060000;

pub const PACKET_128_SIZE: usize = 128;
pub const PACKET_1K_SIZE: usize = 1024;
pub const PACKET_OVERHEAD: usize = 5; // SOH/STX + SEQ + ~SEQ + CRC16
pub const MAX_PACKET_SIZE: usize = PACKET_1K_SIZE + PACKET_OVERHEAD;
pub const AUTH_TAG_SIZE: usize = 16;
pub const NONCE_SIZE: usize = 12;
pub const FILE_SIZE_FIELD: usize = 4;
pub const HEADER_SIZE: usize = NONCE_SIZE + FILE_SIZE_FIELD;

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
    IndexError,
    CrcError,
    DecryptionError,
    AuthenticationError,
    FlashError,
    FileSizeTooLarge,
    BufferOverflow,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum UpdateType {
    FullUpdate,
    PatchUpdate,
}

pub struct XmodemReceiver {
    pub state: XmodemState,
    pub packet_index: u8,
    pub data_counter: usize,
    pub packet_size: usize,
    pub buffer: [u8; MAX_PACKET_SIZE],
    pub packet_received: bool,
    pub first_packet: bool,
    pub crc_received: u16,
    pub total_packet_count: u32,
    
    // Encryption state
    pub key: [u8; 16],
    pub nonce_counter: [u8; NONCE_SIZE],
    pub auth_tag: [u8; AUTH_TAG_SIZE],
    pub calculated_tag: [u8; AUTH_TAG_SIZE],
    pub header_aes: [u8; 16],
    
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
    
    // Decryption buffer
    pub decrypted_data: [u8; PACKET_1K_SIZE],
}

impl XmodemReceiver {
    pub fn new(target_address: u32, update_type: UpdateType) -> Self {
        Self {
            state: XmodemState::WaitHeaderByte,
            packet_index: 0,
            data_counter: 0,
            packet_size: 0,
            buffer: [0; MAX_PACKET_SIZE],
            packet_received: false,
            first_packet: false,
            crc_received: 0,
            total_packet_count: 0,
            
            // Encryption state
            key: [0x08, 0xbc, 0x87, 0xcf, 0x0d, 0xd4, 0x57, 0x33, 0xb9, 0x74, 0xB5, 0x7b, 0xb0, 0x8b, 0xc5, 0x2e],
            nonce_counter: [0x30, 0x00, 0xd4, 0x37, 0xca, 0x8e, 0x89, 0x5e, 0x03, 0x7b, 0x7a, 0xf5],
            auth_tag: [0; AUTH_TAG_SIZE],
            calculated_tag: [0; AUTH_TAG_SIZE],
            header_aes: [0x12, 0xe5, 0x65, 0x3c, 0xb1, 0xc4, 0xdc, 0xfe, 0x43, 0x93, 0x21, 0x0d, 0x65, 0x5b, 0x1d, 0x66],
            
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
            
            // Decryption buffer
            decrypted_data: [0; PACKET_1K_SIZE],
        }
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
        crc
    }
    
    // Extract file size from the header
    pub fn extract_size(&self, data: &[u8]) -> u32 {
        // Assuming the size is stored as a big-endian u32 at offset NONCE_SIZE
        ((data[NONCE_SIZE] as u32) << 24) |
        ((data[NONCE_SIZE + 1] as u32) << 16) |
        ((data[NONCE_SIZE + 2] as u32) << 8) |
        (data[NONCE_SIZE + 3] as u32)
    }
    
    pub fn prepare_header(&mut self) {
        // In a real implementation, this would update the header data
        // based on backed-up firmware version as in the C code
        // For now, we'll just use the predefined header from initialization
    }
    
    pub fn init_gcm_decryption(&mut self) -> Result<(), XmodemError> {
        // This would initialize the AES-GCM context using the nonce and key
        // It would use the aes-gcm crate in a real implementation
        // For example:
        // 
        // let cipher = Aes128Gcm::new_from_slice(&self.key)
        //     .map_err(|_| XmodemError::DecryptionError)?;
        // self.cipher = Some(cipher);
        
        // Update header with appropriate values
        self.prepare_header();
        
        Ok(())
    }
    
    // Decrypt a chunk of data using AES-GCM
    pub fn decrypt_data(&mut self, data: &[u8], out: &mut [u8]) -> Result<usize, XmodemError> {
        // This would decrypt the data using the AES-GCM context
        // It would use the aes-gcm crate in a real implementation
        // For example:
        //
        // let nonce = GenericArray::from_slice(&self.nonce_counter);
        // let cipher = self.cipher.as_ref().ok_or(XmodemError::DecryptionError)?;
        // let plaintext = cipher.decrypt(nonce, data)
        //     .map_err(|_| XmodemError::DecryptionError)?;
        // 
        // out[..plaintext.len()].copy_from_slice(&plaintext);
        // Ok(plaintext.len())
        
        // For now, we'll just copy the data (as if it was already decrypted)
        let len = data.len().min(out.len());
        out[..len].copy_from_slice(&data[..len]);
        
        Ok(len)
    }
    
    // Finish the AES-GCM decryption and verify the tag
    pub fn finish_gcm_decryption(&mut self) -> Result<(), XmodemError> {
        // This would finalize the GCM context and verify the authentication tag
        // It would use the aes-gcm crate in a real implementation
        // For example:
        //
        // let tag = self.auth_tag;
        // let calculated = self.calculated_tag;
        // 
        // // In a real implementation, we would verify that the calculated tag
        // // matches the received tag
        // if tag != calculated {
        //     return Err(XmodemError::AuthenticationError);
        // }
        
        Ok(())
    }
    
    // Process one received byte according to the XMODEM-1K protocol
    pub fn process_byte(&mut self, rx_byte: u8) -> Result<bool, XmodemError> {
        match self.state {
            XmodemState::WaitHeaderByte => {
                match rx_byte {
                    X_SOH => {
                        // 128-byte packet
                        self.packet_size = PACKET_128_SIZE;
                        self.state = XmodemState::WaitIndex1;
                        self.data_counter = 0;
                        self.buffer[0] = rx_byte;
                    },
                    X_STX => {
                        // 1024-byte packet
                        self.packet_size = PACKET_1K_SIZE;
                        self.state = XmodemState::WaitIndex1;
                        self.data_counter = 0;
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
    
    // Process a complete packet
    pub fn process_packet(&mut self) -> Result<bool, XmodemError> {
        if !self.packet_received {
            return Ok(false);
        }
        
        // Reset for next packet
        self.packet_received = false;
        
        // Process based on packet index
        if self.packet_index == 1 && !self.first_packet_complete {
            // This is the first packet, extract nonce and file size
            self.first_packet = true;
            
            // Copy nonce from packet data
            self.nonce_counter.copy_from_slice(&self.buffer[3..3+NONCE_SIZE]);
            
            // Extract file size (after nonce)
            self.file_size = self.extract_size(&self.buffer[3..]);
            
            if self.file_size > 262144 {
                // File too large
                return Err(XmodemError::FileSizeTooLarge);
            }
            
            // Calculate total encrypted length (excluding auth tag)
            self.total_encrypted_length = self.file_size - AUTH_TAG_SIZE as u32;
            
            // Calculate packet counts
            self.total_packets = ((self.file_size as usize + HEADER_SIZE) / self.packet_size) as u32;
            if (self.file_size as usize + HEADER_SIZE) % self.packet_size != 0 {
                self.total_packets += 1;
            }
            
            // Calculate remaining data
            self.remaining_packets = self.total_packets - 1;
            let first_packet_data_len = self.packet_size - HEADER_SIZE;
            self.remaining_encrypted_length = self.total_encrypted_length - first_packet_data_len as u32;
            
            // Calculate bytes in last packet
            self.remaining_bytes_in_last_packet = 
                (self.file_size as usize + HEADER_SIZE) as u32 % self.packet_size as u32;
            if self.remaining_bytes_in_last_packet == 0 && self.total_packets > 0 {
                self.remaining_bytes_in_last_packet = self.packet_size as u32;
            }
            
            // Initialize AES-GCM decryption
            self.init_gcm_decryption()?;
            
            // Decrypt first packet data (after header)
            let data_offset = HEADER_SIZE;
            let data_len = self.packet_size - data_offset;
            
            if data_len > 0 {
                self.decrypt_data(
                    &self.buffer[3 + data_offset..3 + self.packet_size],
                    &mut self.decrypted_data
                )?;
                
                // In a real implementation, this would write to flash
                // flash_write(&self.decrypted_data[..data_len], self.current_address);
                self.current_address += data_len as u32;
            }
            
            self.first_packet_complete = true;
        } else {
            // Regular data packet or continuation of first packet
            
            // In a real implementation, check if flash needs erasing
            // if need_erase(self.current_address) {
            //     flash_erase(self.current_address);
            // }
            
            // Decrypt the packet data
            let data_len = self.packet_size;
            let decrypted_len = self.decrypt_data(
                &self.buffer[3..3 + data_len],
                &mut self.decrypted_data
            )?;
            
            // Check if this is the last packet
            if self.total_packet_count + 1 >= self.total_packets {
                // Extract authentication tag from the end of the last packet
                let actual_data_len = if self.remaining_bytes_in_last_packet > 0 {
                    self.remaining_bytes_in_last_packet as usize
                } else {
                    data_len
                };
                
                let tag_offset = actual_data_len - AUTH_TAG_SIZE;
                
                if tag_offset < decrypted_len {
                    // Last packet contains the authentication tag
                    // Save it for verification
                    self.auth_tag.copy_from_slice(&self.buffer[3 + tag_offset..3 + tag_offset + AUTH_TAG_SIZE]);
                    
                    // In a real implementation, write data before the tag
                    // flash_write(&self.decrypted_data[..tag_offset], self.current_address);
                    self.current_address += tag_offset as u32;
                    
                    // Verify the authentication tag
                    self.finish_gcm_decryption()?;
                } else {
                    // In a real implementation, write all data
                    // flash_write(&self.decrypted_data[..decrypted_len], self.current_address);
                    self.current_address += decrypted_len as u32;
                }
            } else {
                // Regular packet - write all data
                // flash_write(&self.decrypted_data[..decrypted_len], self.current_address);
                self.current_address += decrypted_len as u32;
            }
            
            // Update tracking variables
            self.remaining_encrypted_length = self.remaining_encrypted_length.saturating_sub(data_len as u32);
            if self.remaining_packets > 0 {
                self.remaining_packets -= 1;
            }
        }
        
        // Increment packet counter and reset for next packet
        self.total_packet_count += 1;
        self.state = XmodemState::WaitHeaderByte;
        
        Ok(false) // Not finished yet
    }
    
    // Send cancel command
    pub fn send_cancel(&self, tx_buffer: &mut RingBuffer) {
        for _ in 0..3 {
            tx_buffer.write(X_CAN);
        }
    }
    
    // Download firmware using XMODEM-1K
    pub fn download_firmware(
        &mut self, 
        rx_buffer: &mut RingBuffer, 
        tx_buffer: &mut RingBuffer
    ) -> Result<bool, XmodemError> {
        // If first packet was not received, send 'C' to request CRC mode
        if !self.first_packet && !self.packet_received {
            tx_buffer.write(X_C);
            // In a real implementation, wait a bit before retrying
        }
        
        // If packet was already received, process it
        if self.packet_received {
            let result = self.process_packet();
            
            // Send ACK if packet was processed successfully
            if result.is_ok() {
                tx_buffer.write(X_ACK);
            } else {
                self.send_cancel(tx_buffer);
            }
            
            return result;
        }
        
        // Process any available bytes in the rx buffer
        while let Some(byte) = rx_buffer.read() {
            let result = self.process_byte(byte);
            
            match result {
                Ok(true) => {
                    // EOT received, send ACK and signal completion
                    tx_buffer.write(X_ACK);
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
}