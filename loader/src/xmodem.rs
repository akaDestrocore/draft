use misc::{flash, systick};
use misc::image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER};
use stm32f4 as pac;
use cortex_m::asm;
use core::mem;

// Import rmodem functionality
use rmodem::{
    Control, 
    Error as RmodemError, 
    XmodemPacket, 
    XmodemData,
    Sequence,
    Checksum,
    SOH, STX, EOT, ACK, NAK, CAN, IDLE
};

// XMODEM constants from rmodem
pub const SOH: u8 = rmodem::SOH;
pub const STX: u8 = rmodem::STX;
pub const EOT: u8 = rmodem::EOT;
pub const ACK: u8 = rmodem::ACK;
pub const NAK: u8 = rmodem::NAK;
pub const CAN: u8 = rmodem::CAN;
pub const C: u8 = rmodem::IDLE; // ASCII 'C' (0x43)

// XMODEM state machine
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum XmodemState {
    Idle,           // Not receiving data
    WaitingForData, // Waiting for data (sent 'C' or NAK)
    ReceivingData,  // Receiving data
    Error,          // Error state
    Complete,       // Transfer complete
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

// For debugging - expose important values
static mut DEBUG_PACKET_NUM: u8 = 0;
static mut DEBUG_COMPLEMENT: u8 = 0;
static mut DEBUG_CHECKSUM_RECV: u8 = 0; 
static mut DEBUG_CHECKSUM_CALC: u8 = 0;
static mut DEBUG_FLASH_RESULT: u8 = 0;
static mut DEBUG_CURRENT_STATE: u8 = 0;
static mut DEBUG_LAST_BYTE: u8 = 0;
static mut DEBUG_ERROR_CODE: u8 = 0;
static mut DEBUG_MAGIC: u32 = 0;
static mut DEBUG_VERSION: [u8; 3] = [0, 0, 0];

#[inline(never)]
fn breakpoint_helper(val: u8) -> u8 {
    // Needed for debugging - creates a breakpoint the compiler won't optimize out
    asm::nop();
    val
}

// Validate image header
fn validate_image_header(data: &[u8], expected_magic: u32) -> Result<(), XmodemError> {
    if data.len() < mem::size_of::<ImageHeader>() {
        return Err(XmodemError::InvalidPacket);
    }

    // Read header from data
    let header = unsafe {
        let header_ptr = data.as_ptr() as *const ImageHeader;
        *header_ptr
    };

    // Save for debugging
    unsafe {
        DEBUG_MAGIC = header.image_magic;
        DEBUG_VERSION[0] = header.version_major;
        DEBUG_VERSION[1] = header.version_minor;
        DEBUG_VERSION[2] = header.version_patch;
    }

    // Check magic number
    if header.image_magic != expected_magic {
        return Err(XmodemError::InvalidMagic);
    }

    // Here you could add version checking
    // For example, read current installed version from flash
    // and compare with header.version_*

    Ok(())
}

pub struct XmodemManager {
    state: XmodemState,        // Current state
    target_address: u32,       // Target address for firmware
    current_address: u32,      // Current address for writing
    packet_number: u8,         // Expected packet number
    last_poll_time: u32,       // Time of last NAK/C send
    buffer: [u8; 132],         // Buffer for packet data (1+1+1+128+1)
    buffer_index: usize,       // Buffer index
    last_sector_erased: u32,   // Last erased sector
    next_byte_to_send: Option<u8>, // Next byte to send
    packet_count: u16,         // Counter of received packets
    use_crc: bool,             // Use CRC instead of simple checksum
    expected_magic: u32,       // Expected magic number
    first_packet_validated: bool, // Flag that first packet was validated
}

impl XmodemManager {
    pub fn new() -> Self {
        Self {
            state: XmodemState::Idle,
            target_address: 0,
            current_address: 0,
            packet_number: 1,
            last_poll_time: 0,
            buffer: [0; 132],
            buffer_index: 0,
            last_sector_erased: 0,
            next_byte_to_send: None,
            packet_count: 0,
            use_crc: false,
            expected_magic: 0,
            first_packet_validated: false,
        }
    }

    /// Start XMODEM receive
    pub fn start(&mut self, address: u32) {
        let _ = breakpoint_helper(1); // Breakpoint: XMODEM Start
        
        self.state = XmodemState::WaitingForData;
        self.target_address = address;
        self.current_address = address;
        self.packet_number = 1;
        self.last_poll_time = systick::get_tick_ms();
        self.next_byte_to_send = Some(NAK);  // Standard XMODEM starts with NAK
        self.buffer_index = 0;
        self.packet_count = 0;
        self.use_crc = false;
        self.first_packet_validated = false;
        
        // Set expected magic number based on address
        if address == crate::APP_ADDR {
            self.expected_magic = IMAGE_MAGIC_APP;
        } else if address == crate::UPDATER_ADDR {
            self.expected_magic = IMAGE_MAGIC_UPDATER;
        } else {
            // Unknown address - error
            unsafe { DEBUG_ERROR_CODE = 10; }
            self.state = XmodemState::Error;
            return;
        }
        
        // Erase first sector before starting write
        let result = unsafe { 
            flash::erase_sector(&pac::Peripherals::steal(), address) 
        };
        
        if result > 0 {
            self.last_sector_erased = address;
        } else {
            let _ = breakpoint_helper(2); // Breakpoint: Erase Error
            self.state = XmodemState::Error;
        }
    }

    /// Get current state
    pub fn get_state(&self) -> XmodemState {
        self.state
    }

    /// Check if NAK/C needs to be sent
    pub fn should_send_c(&mut self) -> bool {
        if self.state == XmodemState::WaitingForData {
            let current_time = systick::get_tick_ms();
            if current_time.wrapping_sub(self.last_poll_time) >= 3000 {
                self.last_poll_time = current_time;
                
                // In standard mode send NAK, in CRC mode send 'C'
                if self.use_crc {
                    self.next_byte_to_send = Some(C);
                    let _ = breakpoint_helper(3); // Breakpoint: Sending 'C'
                } else {
                    self.next_byte_to_send = Some(NAK);
                    let _ = breakpoint_helper(4); // Breakpoint: Sending NAK
                }
                
                return true;
            }
        }
        false
    }

    /// Get byte to send
    pub fn get_response(&mut self) -> Option<u8> {
        let response = self.next_byte_to_send;
        if let Some(byte) = response {
            unsafe { DEBUG_LAST_BYTE = byte; }
            let _ = breakpoint_helper(5); // Breakpoint: Sending byte
        }
        self.next_byte_to_send = None;
        response
    }

    /// Number of successfully received packets
    pub fn get_packet_count(&self) -> u16 {
        self.packet_count
    }

    /// Process received byte using rmodem
    pub fn process_byte(&mut self, byte: u8) -> Result<bool, XmodemError> {
        unsafe { 
            DEBUG_LAST_BYTE = byte;
            DEBUG_CURRENT_STATE = match self.state {
                XmodemState::Idle => 0,
                XmodemState::WaitingForData => 1,
                XmodemState::ReceivingData => 2,
                XmodemState::Error => 3,
                XmodemState::Complete => 4,
            };
        }
        
        match self.state {
            XmodemState::Idle => {
                // No active transfer
                Ok(false)
            },
            
            XmodemState::WaitingForData => {
                // Waiting for transfer to start (SOH, STX or EOT)
                match byte {
                    SOH => {
                        // Save for debugging
                        unsafe { 
                            DEBUG_PACKET_NUM = self.packet_count as u8;
                        }
                        let _ = breakpoint_helper(10); // Breakpoint: SOH received
                        
                        // Packet start - 128 bytes
                        self.state = XmodemState::ReceivingData;
                        self.buffer[0] = byte;
                        self.buffer_index = 1;
                        Ok(false)
                    },
                    STX => {
                        let _ = breakpoint_helper(11); // Breakpoint: STX received
                        
                        // 1K packet start - not supported in this version
                        self.next_byte_to_send = Some(NAK);
                        Ok(true)
                    },
                    EOT => {
                        let _ = breakpoint_helper(12); // Breakpoint: EOT received
                        
                        // End of transmission
                        self.next_byte_to_send = Some(ACK);
                        self.state = XmodemState::Complete;
                        Err(XmodemError::TransferComplete)
                    },
                    CAN => {
                        let _ = breakpoint_helper(13); // Breakpoint: CAN received
                        
                        // Transmission cancelled
                        self.state = XmodemState::Error;
                        Err(XmodemError::Cancelled)
                    },
                    _ => {
                        // Ignore other bytes in this state
                        Ok(false)
                    }
                }
            },
            
            XmodemState::ReceivingData => {
                // Save byte to buffer
                self.buffer[self.buffer_index] = byte;
                self.buffer_index += 1;
                
                // Check if we received the entire packet
                // SOH(1) + packet number(1) + ~number(1) + data(128) + checksum(1) = 132 bytes
                if self.buffer_index == 132 {
                    let _ = breakpoint_helper(20); // Breakpoint: Full packet received
                    
                    // Try parsing as XmodemPacket using rmodem
                    let packet_result = XmodemPacket::try_from(&self.buffer[..]);
                    
                    match packet_result {
                        Ok(packet) => {
                            let packet_num = packet.sequence().into_u8();
                            
                            // Save for debugging via watch window
                            unsafe {
                                DEBUG_PACKET_NUM = packet_num;
                                DEBUG_COMPLEMENT = !packet_num; // complement
                            }
                            
                            // Check expected packet number
                            if packet_num != self.packet_number {
                                // Save for debugging
                                unsafe {
                                    DEBUG_PACKET_NUM = packet_num;
                                    DEBUG_COMPLEMENT = self.packet_number;
                                }
                                let _ = breakpoint_helper(22); // Breakpoint: Unexpected packet number
                                
                                // Wrong packet number
                                self.next_byte_to_send = Some(NAK);
                                self.state = XmodemState::WaitingForData;
                                self.buffer_index = 0;
                                return Ok(true);
                            }
                            
                            // Breakpoint for packet #7
                            if packet_num == 7 {
                                let _ = breakpoint_helper(23); // Breakpoint: Processing packet #7
                            }
                            
                            // For first packet check magic number and version
                            if packet_num == 1 && !self.first_packet_validated {
                                let _ = breakpoint_helper(40); // Breakpoint: Validating first packet
                                
                                // Get the data from the packet
                                let packet_data = packet.data().inner();
                                
                                // Validate magic number and version
                                match validate_image_header(packet_data, self.expected_magic) {
                                    Ok(_) => {
                                        // Validation successful
                                        self.first_packet_validated = true;
                                        let _ = breakpoint_helper(41); // Breakpoint: Magic valid
                                    },
                                    Err(e) => {
                                        // Image validation error
                                        unsafe { 
                                            match e {
                                                XmodemError::InvalidMagic => DEBUG_ERROR_CODE = 50,
                                                XmodemError::OlderVersion => DEBUG_ERROR_CODE = 51,
                                                _ => DEBUG_ERROR_CODE = 52,
                                            }
                                        }
                                        let _ = breakpoint_helper(42); // Breakpoint: Magic invalid
                                        
                                        self.state = XmodemState::Error;
                                        return Err(e);
                                    }
                                }
                            }
                            
                            // Check if next sector needs to be erased
                            let next_address = self.current_address + 128;
                            let current_sector = self.current_address & 0xFFFF0000;
                            let next_sector = next_address & 0xFFFF0000;
                            
                            if current_sector != next_sector && next_sector != self.last_sector_erased {
                                let _ = breakpoint_helper(25); // Breakpoint: Erasing new sector
                                
                                // Erase new sector
                                let result = unsafe { 
                                    flash::erase_sector(&pac::Peripherals::steal(), next_sector) 
                                };
                                
                                if result == 0 {
                                    unsafe { DEBUG_ERROR_CODE = 1; } // Code for flash erase error
                                    let _ = breakpoint_helper(26); // Breakpoint: Flash erase error
                                    
                                    self.state = XmodemState::Error;
                                    return Err(XmodemError::FlashWriteError);
                                }
                                
                                self.last_sector_erased = next_sector;
                            }
                            
                            // Write data to flash
                            let _ = breakpoint_helper(27); // Breakpoint: Before flash write
                            
                            let result = unsafe { 
                                // Save address for debugging via watch window
                                let addr = self.current_address;
                                let flash_result = flash::write(
                                    &pac::Peripherals::steal(), 
                                    packet.data().inner(),
                                    addr
                                );
                                DEBUG_FLASH_RESULT = flash_result;
                                flash_result
                            };
                            
                            if result != 0 {
                                unsafe { DEBUG_ERROR_CODE = 2; } // Code for flash write error
                                let _ = breakpoint_helper(28); // Breakpoint: Flash write error
                                
                                // Write error
                                self.state = XmodemState::Error;
                                return Err(XmodemError::FlashWriteError);
                            }
                            
                            let _ = breakpoint_helper(29); // Breakpoint: Successful write
                            
                            // Increment address and packet number
                            self.current_address += 128;
                            self.packet_number = self.packet_number.wrapping_add(1);
                            self.packet_count += 1;
                            
                            // Send ACK
                            self.next_byte_to_send = Some(ACK);
                            self.state = XmodemState::WaitingForData;
                            self.buffer_index = 0;
                            
                            return Ok(true);
                        },
                        Err(_) => {
                            // Packet parsing failed
                            let _ = breakpoint_helper(24); // Breakpoint: Packet parse error
                            
                            self.next_byte_to_send = Some(NAK);
                            self.state = XmodemState::WaitingForData;
                            self.buffer_index = 0;
                            return Ok(true);
                        }
                    }
                }
                
                Ok(false)
            },
            
            XmodemState::Error => {
                unsafe { 
                    DEBUG_LAST_BYTE = byte;
                    DEBUG_ERROR_CODE = 99; // Generic error
                }
                let _ = breakpoint_helper(30); // Breakpoint: Error state
                Ok(false)
            },
            
            XmodemState::Complete => {
                unsafe { DEBUG_LAST_BYTE = byte; }
                let _ = breakpoint_helper(31); // Breakpoint: Complete state
                Ok(false)
            }
        }
    }
}