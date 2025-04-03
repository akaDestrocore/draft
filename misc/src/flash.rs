#![no_std]

use core::ptr;
use stm32f4::Flash;

// Flash sector sizes
const FLASH_SECTORS: [u32; 12] = [
    16,   // sector 0
    16,   // sector 1
    16,   // sector 2
    16,   // sector 3
    64,   // sector 4
    128,  // sector 5
    128,  // sector 6
    128,  // sector 7
    128,  // sector 8
    128,  // sector 9
    128,  // sector 10
    128,  // sector 11
];

pub const FLASH_BASE: u32 = 0x08000000;
pub const FLASH_SECTOR_TOTAL: u8 = 12;

pub struct UpdaterFlash {
    flash: Flash,
}

impl UpdaterFlash {
    pub fn new(flash: Flash) -> Self {
        Self { flash }
    }

    pub fn unlock(&self) -> bool {
        // Check if already unlocked
        if self.flash.cr().read().lock().is_unlocked() {
            return true;
        }

        // Write the key sequence
        unsafe {
            self.flash.keyr().write(|w| w.key().set(0x45670123));
            self.flash.keyr().write(|w| w.key().set(0xCDEF89AB));
        }

        // Verify unlock was successful
        self.flash.cr().read().lock().is_unlocked()
    }

    pub fn lock(&self) {
        self.flash.cr().modify(|_, w| w.lock().locked());
    }

    pub fn wait_for_last_operation(&self) -> bool {
        // Wait for busy flag to clear
        let mut timeout: i32 = 50000;
        while self.flash.sr().read().bsy().is_busy() {
            timeout -= 1;
            if timeout == 0 {
                return false;
            }
        }

        // Check for errors
        let sr = self.flash.sr().read();
        
        if sr.pgserr().is_active() || 
           sr.pgperr().is_active() || 
           sr.pgaerr().is_active() || 
           sr.wrperr().is_active() || 
           sr.operr().is_active() {
            
            // Clear error flags
            self.flash.sr().write(|w| w
                .pgserr().clear()
                .pgperr().clear()
                .pgaerr().clear()
                .wrperr().clear()
                .operr().clear()
            );
            
            return false;
        }

        true
    }

    // Find the sector number corresponding to the given address
    pub fn get_sector_number(&self, address: u32) -> Option<u8> {
        let mut addr: u32 = FLASH_BASE;
        
        for (i, &size) in FLASH_SECTORS.iter().enumerate() {
            if addr == address {
                return Some(i as u8);
            } else if addr > address {
                return None;
            }
            addr += size * 1024;
        }
        
        None
    }

    // Erase a single flash sector at the given address and return erased size
    pub fn erase_sector(&self, destination: u32) -> u32 {
        // Check for existing flash errors
        if self.flash.sr().read().bsy().is_busy() || !self.wait_for_last_operation() {
            return 0;
        }

        // Find sector number for the address
        let sector: u8 = match self.get_sector_number(destination) {
            Some(s) => s,
            None => return 0,
        };

        // Unlock flash
        if !self.unlock() {
            return 0;
        }

        // Configure sector erase
        unsafe {
            self.flash.cr().modify(|_, w| w
                .ser().sector_erase()
                .snb().bits(sector)
            );

            // Start the erase operation
            self.flash.cr().modify(|_, w| w.strt().start());
        }

        // Wait for operation to complete
        if !self.wait_for_last_operation() {
            // Clear SER bit
            self.flash.cr().modify(|_, w| w.ser().clear_bit());
            self.lock();
            return 0;
        }

        // Clear SER bit
        self.flash.cr().modify(|_, w| w.ser().clear_bit());
        
        // Lock flash
        self.lock();

        // Return the size of the erased sector
        FLASH_SECTORS[sector as usize] * 1024
    }

    /// Erases all flash sectors starting from the given address
    pub fn erase(&self, destination: u32) {
        // Check for existing flash errors
        if !self.wait_for_last_operation() {
            return;
        }

        // Find sector number for the address
        let start_sector: u8 = match self.get_sector_number(destination) {
            Some(s) => s,
            None => return,
        };

        // Unlock flash
        if !self.unlock() {
            return;
        }

        // Erase each sector from start_sector to the end
        for sector in start_sector..FLASH_SECTOR_TOTAL {
            // Configure sector erase
            unsafe {
                self.flash.cr().modify(|_, w| w
                    .ser().sector_erase()
                    .snb().bits(sector)
                );

                // Start the erase operation
                self.flash.cr().modify(|_, w| w.strt().start());
            }

            // Wait for operation to complete
            if !self.wait_for_last_operation() {
                // Clear SER bit and exit on error
                self.flash.cr().modify(|_, w| w.ser().clear_bit());
                self.lock();
                return;
            }

            // Clear SER bit
            self.flash.cr().modify(|_, w| w.ser().clear_bit());
        }

        // Lock flash
        self.lock();
    }

    // Write data to flash at the given address
    pub fn write(&self, source_data: &[u8], destination: u32) -> u8 {
        if source_data.is_empty() {
            return 0;
        }

        // Determine program size from CR register
        let psize_bits: u8 = self.flash.cr().read().psize().bits();
        let block_size: i32 = match psize_bits {
            0 => 1, // 8-bit
            1 => 2, // 16-bit
            2 => 4, // 32-bit
            3 => 8, // 64-bit
            _ => return 1,
        };

        // Check if data length is a multiple of block size
        if source_data.len() % block_size as usize != 0 {
            return 2;
        }

        // Unlock flash
        if !self.unlock() {
            return 1;
        }

        // Program data in block_size chunks
        for i in (0..source_data.len()).step_by(block_size as usize) {
            let addr: u32 = destination + i as u32;
            
            // Construct data word from bytes
            let mut data: u32 = 0;
            for j in 0..block_size {
                if i + (j as usize) < source_data.len() {
                    data |= (source_data[i + j as usize] as u32) << (j * 8);
                }
            }

            // Set programming mode
            self.flash.cr().modify(|_, w| w.pg().program());

            // Write data to flash
            match block_size {
                1 => unsafe { ptr::write_volatile(addr as *mut u8, data as u8) },
                2 => unsafe { ptr::write_volatile(addr as *mut u16, data as u16) },
                4 => unsafe { ptr::write_volatile(addr as *mut u32, data) },
                8 => {
                    // 64-bit write would be split into two 32-bit writes
                    unsafe { 
                        ptr::write_volatile(addr as *mut u32, data);
                        ptr::write_volatile((addr + 4) as *mut u32, 0);
                    }
                },
                _ => {}
            }

            // Wait for programming to complete
            if !self.wait_for_last_operation() {
                self.flash.cr().modify(|_, w| w.pg().clear_bit());
                self.lock();
                return 1;
            }
        }

        // Clear programming bit
        self.flash.cr().modify(|_, w| w.pg().clear_bit());
        
        // Lock flash
        self.lock();
        
        0 // Success
    }

    // Read data from flash into the provided buffer
    pub fn read(&self, source: u32, destination: &mut [u8]) {
        for (i, byte) in destination.iter_mut().enumerate() {
            unsafe {
                *byte = ptr::read_volatile((source + i as u32) as *const u8);
            }
        }
    }
}

impl crate::xmodem::FlashOperations for UpdaterFlash {
    fn erase(&self, address: u32) -> Result<(), crate::xmodem::XmodemError> {
        let result: u32 = self.erase_sector(address);
        if result == 0 {
            Err(crate::xmodem::XmodemError::FlashError)
        } else {
            Ok(())
        }
    }

    fn write(&self, address: u32, data: &[u8]) -> Result<(), crate::xmodem::XmodemError> {
        let result: u8 = self.write(data, address);
        if result != 0 {
            Err(crate::xmodem::XmodemError::FlashError)
        } else {
            Ok(())
        }
    }
}