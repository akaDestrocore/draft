#![no_std]

use crate::xmodem::XmodemError;

pub struct SimpleCrypto {
    key: [u8; 16],
    nonce: [u8; 12],
    buffer: [u8; 2048],
    buffer_len: usize,
    tag: [u8; 16],
    is_active: bool,
}

impl SimpleCrypto {
    pub const fn new() -> Self {
        Self {
            key: [0; 16],
            nonce: [0; 12],
            buffer: [0; 2048],
            buffer_len: 0,
            tag: [0; 16],
            is_active: false,
        }
    }

    // Very simple tag generation algorithm
    fn calculate_tag(&self, data: &[u8]) -> [u8; 16] {
        let mut tag = [0u8; 16];
        
        // Simple hash function based on XOR-ing all data with a rolling offset
        for (i, &byte) in data.iter().enumerate() {
            tag[i % 16] ^= byte.wrapping_add((i % 256) as u8);
        }
        
        // Mix in the key
        for i in 0..16 {
            tag[i] ^= self.key[i];
        }
        
        // Mix in the nonce
        for i in 0..12 {
            tag[i % 16] ^= self.nonce[i];
        }
        
        tag
    }
}

impl crate::xmodem::CryptoOperations for SimpleCrypto {
    fn init_decryption(&mut self, key: &[u8], nonce: &[u8], header: &[u8]) -> Result<(), XmodemError> {
        self.key.copy_from_slice(key);
        self.nonce.copy_from_slice(nonce);
        self.buffer_len = 0;
        self.is_active = true;
        Ok(())
    }
    
    fn decrypt_chunk(&mut self, data: &[u8], output: &mut [u8]) -> Result<usize, XmodemError> {
        if !self.is_active {
            return Err(XmodemError::DecryptionError);
        }
        
        let mut output_len = 0;
        
        // Simple XOR decryption with key
        for (i, &byte) in data.iter().enumerate() {
            if i >= output.len() {
                break;
            }
            
            // XOR with key byte and nonce byte for a simple stream cipher
            let key_byte = self.key[i % 16];
            let nonce_byte = if i < 12 { self.nonce[i] } else { 0 };
            
            output[i] = byte ^ key_byte ^ nonce_byte;
            output_len += 1;
            
            // Save data for tag calculation
            if self.buffer_len < self.buffer.len() {
                self.buffer[self.buffer_len] = output[i];
                self.buffer_len += 1;
            }
        }
        
        Ok(output_len)
    }
    
    fn verify_tag(&mut self, expected_tag: &[u8]) -> Result<(), XmodemError> {
        if !self.is_active {
            return Err(XmodemError::DecryptionError);
        }
        
        // Calculate the tag
        let calculated_tag = self.calculate_tag(&self.buffer[0..self.buffer_len]);
        
        // Compare with expected tag
        for i in 0..16 {
            if calculated_tag[i] != expected_tag[i] {
                return Err(XmodemError::AuthenticationError);
            }
        }
        
        Ok(())
    }
    
    fn finish_decryption(&mut self) -> Result<(), XmodemError> {
        self.is_active = false;
        self.buffer_len = 0;
        Ok(())
    }
}