use misc::{flash, systick};
use misc::image::{ImageHeader, SharedMemory, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER};
use stm32f4 as pac;
use rmodem::{
    Control, 
    XmodemPacket, 
    SOH, STX, EOT, ACK, NAK, CAN,
};

// XMODEM constants from rmodem
pub const PACKET_SIZE: usize = 132;  // SOH + seq + ~seq + 128 data + checksum

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum XmodemState {
    Idle,           // Not receiving data
    WaitingForData, // Waiting for data (sent NAK/C)
    ReceivingData,  // Currently receiving a packet
    Error,          // Error encountered; transfer aborted
    Complete,       // Transfer complete
}

#[derive(Debug)]
pub enum XmodemError {
    InvalidPacket,
    Cancelled,
    Timeout,
    TransferComplete,
    FlashWriteError,
    InvalidMagic,     // Magic number in header did not match
    OlderVersion,     // Firmware version is not newer than current
}

pub struct XmodemManager {
    state: XmodemState,        
    target_address: u32,       
    current_address: u32,      
    packet_number: u8,         
    last_poll_time: u32,       
    buffer: [u8; PACKET_SIZE], 
    buffer_index: usize,       
    last_sector_erased: u32,   
    next_response: Option<u8>, 
    packet_count: u16,         
    use_crc: bool,             
    expected_magic: u32,       
    first_packet_validated: bool,
}

impl XmodemManager {
    pub fn new() -> Self {
        Self {
            state: XmodemState::Idle,
            target_address: 0,
            current_address: 0,
            packet_number: 1,
            last_poll_time: 0,
            buffer: [0; PACKET_SIZE],
            buffer_index: 0,
            last_sector_erased: 0,
            next_response: None,
            packet_count: 0,
            use_crc: false,
            expected_magic: 0,
            first_packet_validated: false,
        }
    }

    pub fn start(&mut self, address: u32) {
        self.state = XmodemState::WaitingForData;
        self.target_address = address;
        self.current_address = address;
        self.packet_number = 1;
        self.last_poll_time = systick::get_tick_ms();
        self.next_response = Some(b'C');
        self.buffer_index = 0;
        self.packet_count = 0;
        self.use_crc = true;  // Включаем CRC-режим
        self.first_packet_validated = false;
        
        // Set expected magic based on address
        self.expected_magic = if address == super::APP_ADDR {
            IMAGE_MAGIC_APP
        } else if address == super::UPDATER_ADDR {
            IMAGE_MAGIC_UPDATER
        } else {
            // Invalid address
            self.state = XmodemState::Error;
            return;
        };
        
        // Erase first sector
        let result = unsafe { 
            flash::erase_sector(&pac::Peripherals::steal(), address) 
        };
        
        if result == 0 {
            self.state = XmodemState::Error;
        } else {
            self.last_sector_erased = address;
        }
    }

    pub fn get_state(&self) -> XmodemState {
        self.state
    }

    pub fn should_send_c(&mut self) -> bool {
        if self.state == XmodemState::WaitingForData {
            let current_time = systick::get_tick_ms();
            if current_time.wrapping_sub(self.last_poll_time) >= 1000 {  // Чаще опрашиваем
                self.last_poll_time = current_time;
                
                // Всегда отправляем 'C' в режиме ожидания
                self.next_response = Some(b'C');
                return true;
            }
        }
        false
    }

    pub fn get_response(&mut self) -> Option<u8> {
        self.next_response.take()
    }

    pub fn get_packet_count(&self) -> u16 {
        self.packet_count
    }

    fn validate_header(&mut self, packet_data: &[u8]) -> Result<(), XmodemError> {
        if packet_data.len() < core::mem::size_of::<ImageHeader>() {
            return Err(XmodemError::InvalidPacket);
        }

        let header = unsafe {
            let header_ptr = packet_data.as_ptr() as *const ImageHeader;
            *header_ptr
        };

        if header.image_magic != self.expected_magic {
            return Err(XmodemError::InvalidMagic);
        }

        Ok(())
    }

    pub fn process_byte(&mut self, byte: u8) -> Result<bool, XmodemError> {
        match self.state {
            XmodemState::Idle => Ok(false),
            
            XmodemState::WaitingForData => {
                match byte {
                    SOH => {
                        self.state = XmodemState::ReceivingData;
                        self.buffer[0] = byte;
                        self.buffer_index = 1;
                        Ok(false)
                    },
                    EOT => {
                        self.next_response = Some(ACK);
                        self.state = XmodemState::Complete;
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
                // Accumulate packet bytes
                if self.buffer_index < self.buffer.len() {
                    self.buffer[self.buffer_index] = byte;
                    self.buffer_index += 1;
                }

                // Process complete packet
                if self.buffer_index == PACKET_SIZE {
                    match XmodemPacket::try_from(&self.buffer[..]) {
                        Ok(packet) => {
                            // Validate packet sequence
                            let packet_num = packet.sequence().into_u8();
                            if packet_num != self.packet_number {
                                self.state = XmodemState::WaitingForData;
                                self.buffer_index = 0;
                                return Err(XmodemError::InvalidPacket);
                            }

                            // First packet validation
                            if self.packet_count == 0 && !self.first_packet_validated {
                                match self.validate_header(packet.data().inner()) {
                                    Ok(_) => self.first_packet_validated = true,
                                    Err(e) => {
                                        self.state = XmodemState::Error;
                                        return Err(e);
                                    }
                                }
                            }

                            // Manage flash sectors
                            let next_address = self.current_address + 128;
                            let current_sector = self.current_address & 0xFFFF0000;
                            let next_sector = next_address & 0xFFFF0000;
                            
                            if current_sector != next_sector && next_sector != self.last_sector_erased {
                                let result = unsafe { 
                                    flash::erase_sector(&pac::Peripherals::steal(), next_sector) 
                                };
                                
                                if result == 0 {
                                    self.state = XmodemState::Error;
                                    return Err(XmodemError::FlashWriteError);
                                }
                                
                                self.last_sector_erased = next_sector;
                            }

                            // Write packet to flash
                            let result = unsafe { 
                                flash::write(
                                    &pac::Peripherals::steal(), 
                                    packet.data().inner(),
                                    self.current_address
                                )
                            };

                            if result != 0 {
                                self.state = XmodemState::Error;
                                return Err(XmodemError::FlashWriteError);
                            }

                            // Update state
                            self.current_address += 128;
                            self.packet_number = self.packet_number.wrapping_add(1);
                            self.packet_count += 1;
                            
                            self.next_response = Some(ACK);
                            self.state = XmodemState::WaitingForData;
                            self.buffer_index = 0;

                            Ok(true)
                        },
                        Err(_) => {
                            self.state = XmodemState::WaitingForData;
                            self.buffer_index = 0;
                            Err(XmodemError::InvalidPacket)
                        }
                    }
                } else {
                    Ok(false)
                }
            },
            
            XmodemState::Error => Ok(false),
            XmodemState::Complete => Ok(false),
        }
    }
}