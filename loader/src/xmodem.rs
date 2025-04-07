use misc::{flash, systick};
use stm32f4 as pac;

// XMODEM protocol constants
pub const SOH: u8 = 0x01;  // Start of header (128 bytes)
pub const STX: u8 = 0x02;  // Start of header (1K bytes)
pub const EOT: u8 = 0x04;  // End of transmission
pub const ACK: u8 = 0x06;  // Acknowledge
pub const NAK: u8 = 0x15;  // Negative acknowledge
pub const CAN: u8 = 0x18;  // Cancel
pub const C: u8 = 0x43;    // ASCII 'C' for CRC mode

// XMODEM state machine
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum XmodemState {
    Idle,           // Not receiving
    Start,          // Starting a transmission
    WaitForHeader,  // Waiting for packet header (SOH/STX)
    ReceiveData,    // Receiving data
    CheckCrc,       // Verifying CRC
    FinishTransfer, // Finishing transfer
    Error,          // Error state
}

#[derive(Debug)]
pub enum XmodemError {
    InvalidPacket,
    Cancelled,
    Timeout,
    TransferComplete,
    FlashWriteError,
}

/// CRC16 calculation for XMODEM protocol (CCITT)
fn calculate_crc16(data: &[u8]) -> u16 {
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

pub struct XmodemManager {
    state: XmodemState,
    target_address: u32,
    current_address: u32,
    buffer: [u8; 1024+5], // Data buffer (1K max packet + header + CRC)
    buffer_index: usize,
    packet_size: usize,
    packet_number: u8,
    last_c_time: u32,
    response: Option<u8>,
    retries: u8,
    transfer_started: bool,
    last_erase_sector: u32,
}

impl XmodemManager {
    pub fn new() -> Self {
        Self {
            state: XmodemState::Idle,
            target_address: 0,
            current_address: 0,
            buffer: [0; 1024+5],
            buffer_index: 0,
            packet_size: 0,
            packet_number: 0,
            last_c_time: 0,
            response: None,
            retries: 0,
            transfer_started: false,
            last_erase_sector: 0,
        }
    }

    /// Start an XMODEM transfer
    pub fn start(&mut self, address: u32) {
        self.state = XmodemState::Start;
        self.target_address = address;
        self.current_address = address;
        self.buffer_index = 0;
        self.packet_number = 1;
        self.last_c_time = systick::get_tick_ms();
        self.response = Some(C); // Send initial 'C'
        self.retries = 0;
        self.transfer_started = false;
        self.last_erase_sector = 0;
        
        // We're starting a transfer - let's erase the first sector
        let sector_erased = unsafe { 
            flash::erase_sector(&pac::Peripherals::steal(), address) 
        };
        
        if sector_erased == 0 {
            self.state = XmodemState::Error;
        } else {
            self.last_erase_sector = address;
        }
    }

    /// Get the current state
    pub fn get_state(&self) -> XmodemState {
        self.state
    }

    /// Check if we should send a 'C' character to initiate transfer
    pub fn should_send_c(&mut self) -> bool {
        if self.state == XmodemState::Start {
            let current_time = systick::get_tick_ms();
            if current_time.wrapping_sub(self.last_c_time) >= 3000 {
                self.last_c_time = current_time;
                return true;
            }
        }
        false
    }

    /// Get response byte to send
    pub fn get_response(&mut self) -> Option<u8> {
        let response = self.response;
        self.response = None;
        response
    }

    /// Process a received byte
    pub fn process_byte(&mut self, byte: u8) -> Result<bool, XmodemError> {
        match self.state {
            XmodemState::Idle => {
                // Not receiving
                Ok(false)
            },
            
            XmodemState::Start => {
                if byte == SOH {
                    // Start of 128-byte packet
                    self.buffer[0] = byte;
                    self.buffer_index = 1;
                    self.packet_size = 128;
                    self.state = XmodemState::ReceiveData;
                    self.transfer_started = true;
                    Ok(true)
                } else if byte == STX {
                    // Start of 1K packet
                    self.buffer[0] = byte;
                    self.buffer_index = 1;
                    self.packet_size = 1024;
                    self.state = XmodemState::ReceiveData;
                    self.transfer_started = true;
                    Ok(true)
                } else if byte == EOT {
                    // End of transmission
                    self.state = XmodemState::FinishTransfer;
                    self.response = Some(ACK);
                    Err(XmodemError::TransferComplete)
                } else if byte == CAN {
                    // Cancelled by sender
                    self.state = XmodemState::Idle;
                    Err(XmodemError::Cancelled)
                } else {
                    // Ignore other bytes
                    Ok(false)
                }
            },
            
            XmodemState::WaitForHeader => {
                    Ok(false)
            },

            XmodemState::ReceiveData => {
                if byte == CAN {
                    // Cancelled by sender
                    self.state = XmodemState::Idle;
                    return Err(XmodemError::Cancelled);
                }
                
                // Store the byte
                self.buffer[self.buffer_index] = byte;
                self.buffer_index += 1;
                
                // Check if we have received the whole packet
                if self.buffer_index == 3 + self.packet_size + 2 {
                    self.state = XmodemState::CheckCrc;
                    
                    // Verify packet number
                    let packet_num = self.buffer[1];
                    let packet_num_complement = self.buffer[2];
                    
                    if packet_num + packet_num_complement != 255 {
                        // Invalid packet numbering
                        self.response = Some(NAK);
                        self.state = XmodemState::Start;
                        self.retries += 1;
                        if self.retries > 10 {
                            self.state = XmodemState::Error;
                            return Err(XmodemError::InvalidPacket);
                        }
                        return Ok(true);
                    }
                    
                    // If it's the first packet and packet number is not 1,
                    // it might be a retransmission - accept it
                    if !self.transfer_started && packet_num != 1 {
                        self.packet_number = packet_num;
                    }
                    
                    // Verify packet number sequence
                    if packet_num != self.packet_number {
                        // Wrong packet number, request retransmission
                        self.response = Some(NAK);
                        self.state = XmodemState::Start;
                        self.retries += 1;
                        if self.retries > 10 {
                            self.state = XmodemState::Error;
                            return Err(XmodemError::InvalidPacket);
                        }
                        return Ok(true);
                    }
                    
                    // Verify CRC
                    let crc_received = ((self.buffer[3 + self.packet_size] as u16) << 8) 
                                    | (self.buffer[3 + self.packet_size + 1] as u16);
                    
                    let crc_calculated = calculate_crc16(&self.buffer[3..3 + self.packet_size]);
                    
                    if crc_received != crc_calculated {
                        // CRC error, request retransmission
                        self.response = Some(NAK);
                        self.state = XmodemState::Start;
                        self.retries += 1;
                        if self.retries > 10 {
                            self.state = XmodemState::Error;
                            return Err(XmodemError::InvalidPacket);
                        }
                        return Ok(true);
                    }
                    
                    // Check if we need to erase next flash sector
                    let next_address = self.current_address + self.packet_size as u32;
                    let current_sector = self.current_address & 0xFF000000 | (self.current_address & 0x00FF0000);
                    let next_sector = next_address & 0xFF000000 | (next_address & 0x00FF0000);
                    
                    if current_sector != next_sector && next_sector != self.last_erase_sector {
                        // Need to erase a new sector
                        let sector_erased = unsafe { 
                            flash::erase_sector(&pac::Peripherals::steal(), next_sector) 
                        };
                        
                        if sector_erased == 0 {
                            self.state = XmodemState::Error;
                            return Err(XmodemError::FlashWriteError);
                        }
                        
                        self.last_erase_sector = next_sector;
                    }
                    
                    // Write to flash
                    let write_result = unsafe { 
                        flash::write(&pac::Peripherals::steal(), 
                                    &self.buffer[3..3 + self.packet_size], 
                                    self.current_address)
                    };
                    
                    if write_result != 0 {
                        // Flash write error
                        self.state = XmodemState::Error;
                        return Err(XmodemError::FlashWriteError);
                    }
                    
                    // Update address and packet number
                    self.current_address += self.packet_size as u32;
                    self.packet_number = self.packet_number.wrapping_add(1);
                    
                    // Acknowledge the packet
                    self.response = Some(ACK);
                    self.state = XmodemState::Start;
                    self.buffer_index = 0;
                    self.retries = 0;
                    
                    return Ok(true);
                }
                
                Ok(false)
            },
            
            XmodemState::CheckCrc => {
                // This state is handled directly in ReceiveData
                // Should never reach here
                self.state = XmodemState::Start;
                Ok(false)
            },
            
            XmodemState::FinishTransfer => {
                // End of transfer
                self.state = XmodemState::Idle;
                self.response = Some(ACK);
                Err(XmodemError::TransferComplete)
            },
            
            XmodemState::Error => {
                // Error state
                self.state = XmodemState::Idle;
                Ok(false)
            }
        }
    }
}