use core::mem;
use misc::flash;
use misc::systick;
use misc::image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER};
use stm32f4 as pac;
use cortex_m::asm;

// XMODEM constants
pub const SOH: u8 = 0x01;   // Start of 128-byte Header
pub const STX: u8 = 0x02;   // Start of 1024-byte Header
pub const EOT: u8 = 0x04;   // End of Transmission
pub const ACK: u8 = 0x06;   // Acknowledge
pub const NAK: u8 = 0x15;   // Negative Acknowledge
pub const CAN: u8 = 0x18;   // Cancel
pub const C: u8   = 0x43;   // Request CRC

// XMODEM state machine
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum XmodemState {
    Idle,
    SendingInitialC,    // Sending initial 'C' before file transfer
    WaitingForData,     // Ready to receive first packet
    ReceivingData,      // Receiving data packets
    Error,
    Complete,
}

#[derive(Debug)]
pub enum XmodemError {
    InvalidPacket,
    Cancelled,
    Timeout,
    TransferComplete,
    FlashWriteError,
    InvalidMagic,     // Invalid magic number
    OlderVersion,     // Firmware version is older than current
}

pub struct XmodemManager {
    state: XmodemState,
    target_address: u32,
    current_address: u32,
    packet_number: u8,
    last_poll_time: u32,
    buffer: [u8; 1024],  // Max 1024-byte buffer
    buffer_index: usize,
    last_sector_erased: u32,
    block_size: usize,
    use_crc: bool,
    expected_magic: u32,
    first_packet_validated: bool,
    packet_count: u16,
    next_byte_to_send: Option<u8>,
    initial_c_retries: u8,
}

impl XmodemManager {
    pub fn new() -> Self {
        Self {
            state: XmodemState::Idle,
            target_address: 0,
            current_address: 0,
            packet_number: 1,
            last_poll_time: 0,
            buffer: [0; 1024],
            buffer_index: 0,
            last_sector_erased: 0,
            block_size: 128,  // Default XMODEM block size
            use_crc: true,    // Default to CRC mode
            expected_magic: 0,
            first_packet_validated: false,
            packet_count: 0,
            next_byte_to_send: None,
            initial_c_retries: 0,
        }
    }

    fn validate_image_header(&self, data: &[u8]) -> Result<(), XmodemError> {
        if data.len() < mem::size_of::<ImageHeader>() {
            return Err(XmodemError::InvalidPacket);
        }

        let header = unsafe {
            let header_ptr = data.as_ptr() as *const ImageHeader;
            *header_ptr
        };

        // Check magic number
        if header.image_magic != self.expected_magic {
            return Err(XmodemError::InvalidMagic);
        }

        // Here you could add version checking logic
        // For example, compare with currently installed version

        Ok(())
    }

    pub fn start(&mut self, address: u32) {
        self.state = XmodemState::SendingInitialC;
        self.target_address = address;
        self.current_address = address;
        self.packet_number = 1;
        self.buffer_index = 0;
        self.packet_count = 0;
        self.initial_c_retries = 0;
        self.use_crc = true;
        self.next_byte_to_send = Some(C);  // Start with 'C' in CRC mode
        self.last_poll_time = systick::get_tick_ms();
        
        // Determine block size and expected magic
        if address == crate::APP_ADDR {
            self.block_size = 128;
            self.expected_magic = IMAGE_MAGIC_APP;
        } else if address == crate::UPDATER_ADDR {
            self.block_size = 128;
            self.expected_magic = IMAGE_MAGIC_UPDATER;
        } else {
            self.state = XmodemState::Error;
            return;
        }

        // Erase first sector
        let peripherals = unsafe { pac::Peripherals::steal() };
        let result = flash::erase_sector(&peripherals, address);
        
        if result > 0 {
            self.last_sector_erased = address;
        } else {
            self.state = XmodemState::Error;
        }
    }

    pub fn process_byte(&mut self, byte: u8) -> Result<bool, XmodemError> {
        match self.state {
            XmodemState::Idle => Ok(false),
            
            XmodemState::SendingInitialC => {
                // Handling timeout for initial 'C'
                let current_time = systick::get_tick_ms();
                if current_time.wrapping_sub(self.last_poll_time) >= 3000 {
                    self.initial_c_retries += 1;
                    
                    // Max 20 retries before switching to NAK mode
                    if self.initial_c_retries >= 20 {
                        self.use_crc = false;
                        self.next_byte_to_send = Some(NAK);
                        self.state = XmodemState::WaitingForData;
                    } else {
                        self.next_byte_to_send = Some(C);
                    }
                    
                    self.last_poll_time = current_time;
                    return Ok(true);
                }

                // Switch to waiting state when first data arrives
                match byte {
                    SOH => {
                        self.state = XmodemState::ReceivingData;
                        self.block_size = 128;
                        self.buffer[0] = byte;
                        self.buffer_index = 1;
                        Ok(false)
                    },
                    STX => {
                        // 1K blocks not supported
                        Ok(true)
                    },
                    _ => Ok(false)
                }
            },
            
            XmodemState::WaitingForData => {
                match byte {
                    SOH => {
                        self.block_size = 128;
                        self.state = XmodemState::ReceivingData;
                        self.buffer[0] = byte;
                        self.buffer_index = 1;
                        Ok(false)
                    },
                    STX => {
                        // 1K blocks not supported
                        Ok(true)
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
                self.buffer[self.buffer_index] = byte;
                self.buffer_index += 1;

                if self.buffer_index == self.block_size + 5 {  // Header + Data + Checksum
                    let packet_num = self.buffer[1];
                    let packet_num_comp = self.buffer[2];

                    // Validate packet number
                    if packet_num + packet_num_comp != 0xFF {
                        self.state = XmodemState::WaitingForData;
                        self.buffer_index = 0;
                        self.next_byte_to_send = Some(NAK);
                        return Ok(true);
                    }

                    // First packet validation
                    if self.packet_number == 1 && !self.first_packet_validated {
                        match self.validate_image_header(&self.buffer[3..self.block_size+3]) {
                            Ok(_) => {
                                self.first_packet_validated = true;
                            },
                            Err(e) => {
                                self.state = XmodemState::Error;
                                return Err(e);
                            }
                        }
                    }

                    // Write to flash
                    let peripherals = unsafe { pac::Peripherals::steal() };
                    let result = flash::write(
                        &peripherals, 
                        &self.buffer[3..self.block_size+3], 
                        self.current_address
                    );

                    if result != 0 {
                        self.state = XmodemState::Error;
                        return Err(XmodemError::FlashWriteError);
                    }

                    self.current_address += self.block_size as u32;
                    self.packet_number = self.packet_number.wrapping_add(1);
                    self.packet_count += 1;
                    
                    self.state = XmodemState::WaitingForData;
                    self.buffer_index = 0;
                    self.next_byte_to_send = Some(ACK);
                    
                    return Ok(true);
                }

                Ok(false)
            },
            
            XmodemState::Error => Ok(false),
            XmodemState::Complete => Ok(false),
        }
    }

    pub fn get_state(&self) -> XmodemState {
        self.state
    }

    pub fn get_packet_count(&self) -> u16 {
        self.packet_count
    }

    pub fn should_send_c(&mut self) -> bool {
        match self.state {
            XmodemState::SendingInitialC => {
                let current_time = systick::get_tick_ms();
                if current_time.wrapping_sub(self.last_poll_time) >= 3000 {
                    self.initial_c_retries += 1;
                    
                    // Max 20 retries before switching to NAK mode
                    if self.initial_c_retries >= 20 {
                        self.use_crc = false;
                        self.next_byte_to_send = Some(NAK);
                        self.state = XmodemState::WaitingForData;
                    } else {
                        self.next_byte_to_send = Some(C);
                    }
                    
                    self.last_poll_time = current_time;
                    return true;
                }
                false
            },
            XmodemState::WaitingForData => {
                let current_time = systick::get_tick_ms();
                if current_time.wrapping_sub(self.last_poll_time) >= 3000 {
                    self.last_poll_time = current_time;
                    
                    // In standard mode send NAK, in CRC mode send 'C'
                    if self.use_crc {
                        self.next_byte_to_send = Some(C);
                    } else {
                        self.next_byte_to_send = Some(NAK);
                    }
                    
                    return true;
                }
                false
            },
            _ => false
        }
    }

    pub fn get_response(&mut self) -> Option<u8> {
        let response = self.next_byte_to_send;
        self.next_byte_to_send = None;
        response
    }
}