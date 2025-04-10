#![no_std]

use core::{mem, ptr::addr_eq};
use cortex_m::{peripheral, asm};
use crate::{
    flash,
    systick,
    image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_MAGIC_LOADER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER, IMAGE_TYPE_LOADER},
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

// Config structure to handle addresses
#[derive(Clone, Copy)]
pub struct XmodemConfig {
    pub app_addr: u32,
    pub updater_addr: u32,
    pub loader_addr: u32, 
    pub image_hdr_size: u32,
}

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
    total_data_received: u32,
    actual_firmware_size: Option<u32>,
    header_size: u32,
    received_eot: bool,
    first_sector_erased: bool,
    expected_total_packets: Option<u16>,
    last_packet_useful_bytes: Option<usize>,
    config: XmodemConfig,
}

impl XmodemManager {
    pub fn new(config: XmodemConfig) -> Self {
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
            total_data_received: 0,
            actual_firmware_size: None,
            header_size: config.image_hdr_size,
            received_eot: false,
            first_sector_erased: false,
            expected_total_packets: None,
            last_packet_useful_bytes: None,
            config,
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
        self.total_data_received = 0;
        self.actual_firmware_size = None;
        self.received_eot = false;
        self.first_sector_erased = false;
        self.expected_total_packets = None;
        self.last_packet_useful_bytes = None;
        self.header_size = self.config.image_hdr_size;

        if addr == self.config.app_addr {
            self.expected_magic = IMAGE_MAGIC_APP;
            self.expected_img_type = IMAGE_TYPE_APP;
        } else if addr == self.config.updater_addr {
            self.expected_magic = IMAGE_MAGIC_UPDATER;
            self.expected_img_type = IMAGE_TYPE_UPDATER;
        } else if addr == self.config.loader_addr {
            self.expected_magic = IMAGE_MAGIC_LOADER;
            self.expected_img_type = IMAGE_TYPE_LOADER;
        } else {
            self.state = XmodemState::Error;
            return;
        }

        self.current_sector_base = addr;
    }

    pub fn process_byte(&mut self, byte: u8) -> Result<bool, XmodemError> {
        let current_time: u32 = systick::get_tick_ms();

        match self.state {
            XmodemState::Idle => {
                Ok(false)
            },

            XmodemState::SendingInitialC => {
                // Keep checking for SOH to start firmware reception
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
                // Timeout check first
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
                        // Set EOT received because this is the last packet
                        self.received_eot = true;
                        self.state = XmodemState::Complete;
                        self.next_byte_to_send = Some(ACK);
                        
                        // If we have size info from header then we just cut unnecessary stuff
                        if let Some(firmware_size) = self.actual_firmware_size {
                            // If we got less than expected then it's an error
                            if self.total_data_received < firmware_size {
                                self.state = XmodemState::Error;
                                return Err(XmodemError::InvalidPacket);
                            }
                            // If we got as much as we expected then just check that current_addr is pointing to the actual end of data
                            if self.current_addr > self.target_addr + firmware_size {
                                self.current_addr = self.target_addr + firmware_size;
                            }
                        }
                        
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
                // Timeout check first
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
                // Will do it in other function
                Ok(false)
            },
            
            XmodemState::Error | XmodemState::Complete => {
                Ok(false)
            }
        }
    }

    fn process_packet(&mut self) -> Result<bool, XmodemError> {
        // Disassemble the packet
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

        // Check header and parse size out of the first packet
        if packet_num == 1 && !self.first_packet_processed {
            let mut data_copy = [0u8; DATA_SIZE];
            data_copy.copy_from_slice(&self.buffer[3..3+DATA_SIZE]);

            let result = self.process_first_packet(&data_copy);
            if result.is_err() {
                return result;
            }
        } else {
            if packet_num == 2 && !self.first_packet_processed {
                self.state = XmodemState::Error;
                return Err(XmodemError::InvalidPacket);
            }
            
            // Calculate useful bytes in the packet
            let data = &self.buffer[3..3+DATA_SIZE];
            let mut useful_bytes: usize = self.find_useful_bytes_in_packet(data, packet_num);
            
            if useful_bytes == 0 {
                self.state = XmodemState::WaitingForData;
                self.buffer_index = 0;
                self.expected_packet_num = self.expected_packet_num.wrapping_add(1);
                self.packet_count += 1;
                self.next_byte_to_send = Some(ACK);
                self.last_poll_time = systick::get_tick_ms();
                return Ok(true);
            }
            
            self.total_data_received += useful_bytes as u32;
            
            // Check for overflow if the size is not mentioned
            if let Some(firmware_size) = self.actual_firmware_size {
                // If we already got as much data (or even more)
                if self.total_data_received > firmware_size {
                    // Calculate overflow
                    let excess: u32 = self.total_data_received - firmware_size;
                    if excess < useful_bytes as u32 {
                        // Fix the size of useful data of the last packet
                        let adjusted_bytes: u32 = useful_bytes as u32 - excess;
                        useful_bytes = adjusted_bytes as usize;
                        self.total_data_received = firmware_size;
                    } else {
                        // If all of the bytes in the packet were useless (not possible but still)
                        useful_bytes = 0;
                    }
                }
            }
            
            // Check if we are not getting out of the sector
            let next_addr: u32 = self.current_addr + useful_bytes as u32;
            let current_sector_end: u32 = self.current_sector_base + 0x20000;
            
            if next_addr > current_sector_end && useful_bytes > 0 {
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

            // Write only useful data to flash
            if useful_bytes > 0 {
                let peripherals: stm32f4::Peripherals = unsafe { pac::Peripherals::steal() };

                let mut data_to_write = [0u8; DATA_SIZE];
                data_to_write[..useful_bytes].copy_from_slice(&self.buffer[3..3 + useful_bytes]);
                
                // Write
                let result: u8 = flash::write(&peripherals, &data_to_write[..useful_bytes], self.current_addr);
                
                if result != 0 {
                    self.state = XmodemState::Error;
                    return Err(XmodemError::FlashWriteError);
                }
                
                self.current_addr += useful_bytes as u32;
            }
        }

        // Success
        self.state = XmodemState::WaitingForData;
        self.buffer_index = 0;
        self.expected_packet_num = self.expected_packet_num.wrapping_add(1);
        self.packet_count += 1;
        self.next_byte_to_send = Some(ACK);
        self.last_poll_time = systick::get_tick_ms();
        Ok(true)
    }

    fn find_useful_bytes_in_packet(&self, data: &[u8], packet_num: u8) -> usize {
        if let (Some(total_packets), Some(last_bytes)) = (self.expected_total_packets, self.last_packet_useful_bytes) {
            if packet_num as u16 == total_packets {
                return last_bytes;
            }
        }
        DATA_SIZE
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
        
        // Calculate total data of the image
        if header.data_size > 0 {
            // header_size + data_size from header
            self.actual_firmware_size = Some(header.data_size + self.header_size);
            
            // Calculate total packets count
            let total_bytes: u32 = header.data_size + self.header_size;
            let full_packets: u32 = total_bytes / DATA_SIZE as u32;
            let remainder: u32 = total_bytes % DATA_SIZE as u32;
            
            self.expected_total_packets = Some((full_packets + if remainder > 0 { 1 } else { 0 }) as u16);
            
            if remainder > 0 {
                self.last_packet_useful_bytes = Some(remainder as usize);
            } else {
                self.last_packet_useful_bytes = Some(DATA_SIZE); // last 128 packet
            }
        } else {
            // If no size mentioned then use hardcoded size
            self.actual_firmware_size = Some(0x40000); // 256 kB
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
        
        // After checking the header we can erase sector
        if !self.first_sector_erased {
            let peripherals: stm32f4::Peripherals = unsafe { pac::Peripherals::steal() };
            match flash::erase_sector(&peripherals, self.target_addr) {
                0 => { 
                    self.state = XmodemState::Error;
                    return Err(XmodemError::FlashWriteError);
                },
                _ => {
                    self.first_sector_erased = true;
                }
            }
        }
        
        // Write the first packet data to flash
        let peripherals: stm32f4::Peripherals = unsafe { pac::Peripherals::steal() };
        
        // First packet should have only useful data 
        let useful_bytes: usize = DATA_SIZE;
        
        // Write
        let result: u8 = flash::write(&peripherals, &data[..useful_bytes], self.current_addr);
        
        if result != 0 {
            self.state = XmodemState::Error;
            return Err(XmodemError::FlashWriteError);
        }
        
        self.total_data_received = useful_bytes as u32;
        
        // Update current address for next packet
        self.current_addr += useful_bytes as u32;
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
    
    pub fn cancel_transfer(&mut self) {
        self.state = XmodemState::Error;
        self.next_byte_to_send = None;
    }
}