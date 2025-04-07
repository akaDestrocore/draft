use core::{mem, ptr::addr_eq};
use cortex_m::peripheral;
use misc::{
    flash,
    systick,
    image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER },
};
use stm32f4 as pac;

// XMODEM constants
pub const SOH: u8 = 0x01;
pub const EOT: u8 = 0x04;
pub const ACK: u8 = 0x06;
pub const NAK: u8 = 0x15;
pub const CAN: u8 = 0x18;
pub const X_C: u8 = 0x43;

// Timeout values in millis
const PACKET_TIMEOUT_MS: u32 = 5000; // 5 sec for each packet
const C_RETRY_INTERVAL_MS: u32 = 3000;
const MAX_RETRIES: u8 = 10;
const PACKET_SIZE: usize = 133;
const DATA_SIZE: usize = 128;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum XmodemState {
    Idle,
    SendingInitialC,
    WaitingForData,
    ReceivingData,
    ProcessingPacket,
    Error,
    Complete,
}

#[derive(Debug)]
pub enum XmodemError {
    InvalidPacket,         // Invalid packet structure
    SequenceError,         // Sequence number mismatch
    CrcError,              // CRC checksum mismatch
    Cancelled,             // Transfer cancelled by sender
    Timeout,               // Timeout waiting for data
    FlashWriteError,       // Error writing to flash
    InvalidMagic,          // Invalid magic number in firmware
    OlderVersion,          // Firmware version is older than current
    TransferComplete,      // Transfer completed successfully
}

pub struct XmodemManager {
    state: XmodemState,
    target_addr: u32,
    current_addr: u32,
    expected_packet_num: u8,
    last_poll_time: u32,
    buffer: [u8; PACKET_SIZE],
    buffer_index: usize,
    expected_magic: u32,
    expected_img_type: u8,
    packet_count: u16,
    next_byte_to_send: Option<u8>,
    retries: u8,
    first_packet_processed: bool,
    current_sector_base: u32,
}

impl XmodemManager {
    pub fn new() -> Self {
        Self {
            state: XmodemState::Idle,
            target_addr: 0,
            current_addr: 0,
            expected_packet_num: 1,
            last_poll_time: 0,
            buffer: [0; PACKET_SIZE],
            buffer_index: 0,
            expected_magic: 0,
            expected_img_type: 0,
            packet_count: 0,
            next_byte_to_send: None,
            retries: 0,
            first_packet_processed: false,
            current_sector_base: 0,
        }
    }

    pub fn start(&mut self, addr: u32) {
        self.state = XmodemState::SendingInitialC;
        self.target_addr = addr;
        self.current_addr = addr;
        self.expected_packet_num = 1;
        self.buffer_index = 0;
        self.packet_count = 0;
        self.retries = 0;
        self.first_packet_processed = false;
        self.next_byte_to_send = Some(X_C); 
        self.last_poll_time = systick::get_tick_ms();

        if addr == crate::APP_ADDR {
            self.expected_magic = IMAGE_MAGIC_APP;
            self.expected_img_type = IMAGE_TYPE_APP;
        } else if addr == crate::UPDATER_ADDR {
            self.expected_magic = IMAGE_MAGIC_UPDATER;
            self.expected_img_type = IMAGE_TYPE_UPDATER;
        } else {
            self.state = XmodemState::Error;
            return;
        }

        self.current_sector_base = addr;
        // ersae first sector at target address
        let peripherals: stm32f4::Peripherals = unsafe {
            pac::Peripherals::steal()
        };
        match flash::erase_sector(&peripherals, addr) {
            0 => { self.state = XmodemState::Error;},
            _ => {
                // success
            }
        }
    }

    pub fn process_byte(&mut self, byte: u8) -> Result<bool, XmodemError> {
        let current_time: u32 = systick::get_tick_ms();

        match self.state {
            XmodemState::Idle => {
                Ok(false)
            },

            XmodemState::SendingInitialC => {
                // keep checkign for SOH to start firmware reception
                if byte == SOH {
                    self.buffer[0] = byte;
                    self.buffer_index = 1;
                    self.state = XmodemState::ReceivingData;
                    self.last_poll_time = current_time;
                    Ok(false)
                } else if byte == CAN {
                    self.state = XmodemState::Error;
                    Err(XmodemError::Cancelled)
                } else {
                    // 3 sec timeout for sending 'C' again
                    if current_time.wrapping_sub(self.last_poll_time) >= C_RETRY_INTERVAL_MS {
                        self.next_byte_to_send = Some(X_C);
                        self.last_poll_time = current_time;
                        self.retries += 1;

                        if self.retries >= MAX_RETRIES {
                            self.state = XmodemState:: Error;
                            return Err(XmodemError::Timeout);
                        }

                        Ok(true)
                    } else {
                        Ok(false)
                    }
                }
            },

            XmodemState::WaitingForData => {
                // timeout check first
                if current_time.wrapping_sub(self.last_poll_time) >= PACKET_TIMEOUT_MS {
                    self.state = XmodemState::Error;
                    return Err(XmodemError::Timeout);
                }

                match byte {
                    SOH => {
                        self.buffer[0] = byte;
                        self.buffer_index = 1;
                        self.state = XmodemState::ReceivingData;
                        self.last_poll_time = current_time;
                        Ok(false)
                    },
                    EOT => {
                        self.state = XmodemState::Complete;
                        self.next_byte_to_send = Some(ACK);
                        Err(XmodemError::TransferComplete)
                    },
                    CAN => {
                        self.state = XmodemState::Error;
                        Err(XmodemError::Cancelled)
                    },
                    _ => Ok(false)
                }
            },

            XmodemState::ReceivingData => {
                // timeout check first
                if current_time.wrapping_sub(self.last_poll_time) >= PACKET_TIMEOUT_MS {
                    self.state = XmodemState::Error;
                    return Err(XmodemError::Timeout);
                }
                
                // Add byte to buffer
                self.buffer[self.buffer_index] = byte;
                self.buffer_index += 1;
                
                // Check if we have a complete packet
                if self.buffer_index == PACKET_SIZE {
                    self.state = XmodemState::ProcessingPacket;
                    self.process_packet()
                } else {
                    Ok(false)
                }
            },

            XmodemState::ProcessingPacket => {
                // will do it in other function
                Ok(false)
            },
            
            XmodemState::Error | XmodemState::Complete => {
                // TODO:
                Ok(false)
            }
        }
    }

    fn process_packet(&mut self) -> Result<bool, XmodemError> {
        // dissassemply the packet
        let packet_num: u8 = self.buffer[1];
        let packet_num_complement: u8 = self.buffer[2];
        
        if packet_num.wrapping_add(packet_num_complement) != 0xFF {
            self.state = XmodemState::WaitingForData;
            self.buffer_index = 0;
            self.next_byte_to_send = Some(NAK);
            return Err(XmodemError::SequenceError);
        }
        
        if packet_num != self.expected_packet_num {
            self.state = XmodemState::WaitingForData;
            self.buffer_index = 0;
            self.next_byte_to_send = Some(NAK);
            return Err(XmodemError::SequenceError);
        }
        
        // CRC verification
        let received_crc: u16 = ((self.buffer[PACKET_SIZE-2] as u16) << 8) | (self.buffer[PACKET_SIZE-1] as u16);
        let calculated_crc: u16 = self.calculate_crc16(&self.buffer[3..3+DATA_SIZE]);
        
        if received_crc != calculated_crc {
            self.state = XmodemState::WaitingForData;
            self.buffer_index = 0;
            self.next_byte_to_send = Some(NAK);
            return Err(XmodemError::CrcError);
        }
        
        // For the very first packet we need to check magic and version first
        if packet_num == 1 && !self.first_packet_processed {
            let mut data_copy = [0u8; DATA_SIZE];
            data_copy.copy_from_slice(&self.buffer[3..3+DATA_SIZE]);

            let result = self.process_first_packet(&data_copy);
            if result.is_err() {
                return result;
            }
        } else {
            // check if we need to erase the next sector
            let next_addr = self.current_addr + DATA_SIZE as u32;
            let current_sector_end: u32 = self.current_sector_base + 0x20000;
            
            if next_addr > current_sector_end {
                let peripherals: stm32f4::Peripherals = unsafe { pac::Peripherals::steal() };
                let next_sector_base: u32 = self.current_sector_base + 0x20000;
                
                match flash::erase_sector(&peripherals, next_sector_base) {
                    0 => {
                        self.state = XmodemState::Error;
                        return Err(XmodemError::FlashWriteError);
                    },
                    _ => {
                        self.current_sector_base = next_sector_base;
                    }
                }
            }
            
            // write data to flash
            let mut data_copy = [0u8; DATA_SIZE];
            data_copy.copy_from_slice(&self.buffer[3..3+DATA_SIZE]);
            
            let peripherals: stm32f4::Peripherals = unsafe { pac::Peripherals::steal() };
            let result: u8 = flash::write(&peripherals, &data_copy, self.current_addr);
            
            if result != 0 {
                self.state = XmodemState::Error;
                return Err(XmodemError::FlashWriteError);
            }
            
            // Update current address
            self.current_addr += DATA_SIZE as u32;
        }
        
        // success
        self.state = XmodemState::WaitingForData;
        self.buffer_index = 0;
        self.expected_packet_num = self.expected_packet_num.wrapping_add(1);
        self.packet_count += 1;
        self.next_byte_to_send = Some(ACK);
        self.last_poll_time = systick::get_tick_ms();
        
        Ok(true) // Need to send ACK
    }

    fn process_first_packet(&mut self, data: &[u8]) -> Result<bool, XmodemError> {
        // Check if we have enough data for a header
        if data.len() < mem::size_of::<ImageHeader>() {
            self.state = XmodemState::Error;
            return Err(XmodemError::InvalidPacket);
        }
        
        let header: &ImageHeader = unsafe {
            let header_ptr: *const ImageHeader = data.as_ptr() as *const ImageHeader;
            &*header_ptr
        };
        
        // Check magic number
        if header.image_magic != self.expected_magic {
            self.state = XmodemState::Error;
            return Err(XmodemError::InvalidMagic);
        }
        
        // Check image type
        if header.image_type != self.expected_img_type {
            self.state = XmodemState::Error;
            return Err(XmodemError::InvalidMagic);
        }
        
        // Compare versions
        let current_header_addr: *const ImageHeader = self.target_addr as *const ImageHeader;
        let current_header: Option<&ImageHeader> = unsafe { 
            // Only read if the address is not all 0xFF (erased flash)
            if *(self.target_addr as *const u32) != 0xFFFFFFFF {
                Some(&*current_header_addr)
            } else {
                None
            }
        };
        
        if let Some(existing_header) = current_header {
            if existing_header.is_valid() && !header.is_newer_than(existing_header) {
                self.state = XmodemState::Error;
                return Err(XmodemError::OlderVersion);
            }
        }
        
        // Write the first packet data to flash
        let peripherals: stm32f4::Peripherals = unsafe { pac::Peripherals::steal() };
        let result: u8 = flash::write(&peripherals, data, self.current_addr);
        
        if result != 0 {
            self.state = XmodemState::Error;
            return Err(XmodemError::FlashWriteError);
        }
        
        // Update current address for next packet
        self.current_addr += DATA_SIZE as u32;
        self.first_packet_processed = true;
        
        Ok(true)
    }

    fn calculate_crc16(&self, data: &[u8]) -> u16 {
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

    pub fn should_send_byte(&mut self) -> bool {
        match self.state {
            XmodemState::SendingInitialC => {
                let current_time: u32 = systick::get_tick_ms();
                if current_time.wrapping_sub(self.last_poll_time) >= C_RETRY_INTERVAL_MS || self.next_byte_to_send.is_some() {
                    if self.next_byte_to_send.is_none() {
                        self.next_byte_to_send = Some(X_C);
                    }
                    self.last_poll_time = current_time;
                    self.retries += 1;
                    
                    if self.retries >= MAX_RETRIES {
                        self.state = XmodemState::Error;
                    }
                    
                    true
                } else {
                    false
                }
            },
            _ => self.next_byte_to_send.is_some()
        }
    }

    pub fn get_response(&mut self) -> Option<u8> {
        let response: Option<u8> = self.next_byte_to_send;
        self.next_byte_to_send = None;
        response
    }

    pub fn get_state(&self) -> XmodemState {
        self.state
    }

    pub fn get_packet_count(&self) -> u16 {
        self.packet_count
    }
}