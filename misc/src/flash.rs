#![no_std]

use crate::xmodem::XmodemError;
use stm32f4::{self as pac, Flash};

pub struct Stm32f4Flash {
    flash: Flash,
}

impl Stm32f4Flash {
    pub fn new(flash: Flash) -> Self {
        Self { flash }
    }

    fn unlock(&self) -> Result<(), XmodemError> {
        // Check if already unlocked
        if self.flash.cr().read().lock().is_unlocked() {
            return Ok(());
        }

        // Write unlock keys
        unsafe {
            self.flash.keyr().write(|w| w.key().set(0x45670123));
            self.flash.keyr().write(|w| w.key().set(0xCDEF89AB));
        }

        // Verify unlock
        if self.flash.cr().read().lock().is_locked() {
            return Err(XmodemError::FlashError);
        }

        Ok(())
    }

    fn lock(&self) {
        self.flash.cr().modify(|_, w| w.lock().locked());
    }

    fn wait_for_last_operation(&self) -> Result<(), XmodemError> {
        // Wait for operation to complete
        let mut retries = 50000;
        while self.flash.sr().read().bsy().is_busy() && retries > 0 {
            retries -= 1;
        }

        if retries == 0 {
            return Err(XmodemError::FlashError);
        }

        // Check for errors
        let sr = self.flash.sr().read();
        
        // Check each error flag
        if sr.pgserr().is_active() || sr.pgperr().is_active() || 
           sr.pgaerr().is_active() || sr.wrperr().is_active() || 
           sr.operr().is_active() {
            
            // Clear error flags
            self.flash.sr().write(|w| {
                w.pgserr().clear()
                 .pgperr().clear()
                 .pgaerr().clear()
                 .wrperr().clear()
                 .operr().clear()
            });
            
            return Err(XmodemError::FlashError);
        }

        Ok(())
    }

    fn get_sector_number(&self, address: u32) -> Option<u8> {
        match address {
            addr if addr >= 0x08000000 && addr < 0x08004000 => Some(0),  // 16 KB
            addr if addr >= 0x08004000 && addr < 0x08008000 => Some(1),  // 16 KB
            addr if addr >= 0x08008000 && addr < 0x0800C000 => Some(2),  // 16 KB
            addr if addr >= 0x0800C000 && addr < 0x08010000 => Some(3),  // 16 KB
            addr if addr >= 0x08010000 && addr < 0x08020000 => Some(4),  // 64 KB
            addr if addr >= 0x08020000 && addr < 0x08040000 => Some(5),  // 128 KB
            addr if addr >= 0x08040000 && addr < 0x08060000 => Some(6),  // 128 KB
            addr if addr >= 0x08060000 && addr < 0x08080000 => Some(7),  // 128 KB
            addr if addr >= 0x08080000 && addr < 0x080A0000 => Some(8),  // 128 KB
            addr if addr >= 0x080A0000 && addr < 0x080C0000 => Some(9),  // 128 KB
            addr if addr >= 0x080C0000 && addr < 0x080E0000 => Some(10), // 128 KB
            addr if addr >= 0x080E0000 && addr < 0x08100000 => Some(11), // 128 KB
            _ => None,
        }
    }
}

impl crate::xmodem::FlashOperations for Stm32f4Flash {
    fn erase(&self, address: u32) -> Result<(), XmodemError> {
        // Get the sector number from the address
        let sector = match self.get_sector_number(address) {
            Some(s) => s,
            None => return Err(XmodemError::FlashError),
        };

        // Unlock flash
        self.unlock()?;
        
        // Wait for any previous operations to complete
        self.wait_for_last_operation()?;

        // Set up sector erase
        self.flash.cr().modify(|_, w| unsafe {
            w.ser().sector_erase()
             .snb().bits(sector)
        });

        // Start erase
        self.flash.cr().modify(|_, w| w.strt().start());

        // Wait for the erase to complete
        self.wait_for_last_operation()?;

        // Clear SER bit
        self.flash.cr().modify(|_, w| w.ser().clear_bit());

        // Lock flash
        self.lock();
        
        Ok(())
    }

    fn write(&self, address: u32, data: &[u8]) -> Result<(), XmodemError> {
        if data.is_empty() {
            return Ok(());
        }

        // Ensure address is aligned to 4 bytes (32-bit)
        if address % 4 != 0 {
            return Err(XmodemError::FlashError);
        }

        // Unlock flash
        self.unlock()?;
        
        // Wait for any previous operations to complete
        self.wait_for_last_operation()?;

        // Set programming mode (32-bit)
        self.flash.cr().modify(|_, w| {
            w.pg().program()
             .psize().psize32()
        });

        // Write data in 32-bit chunks
        for (i, chunk) in data.chunks(4).enumerate() {
            let mut word = 0u32;
            
            // Convert chunk to u32
            for (j, &byte) in chunk.iter().enumerate() {
                word |= (byte as u32) << (j * 8);
            }

            // Fill remaining bytes with 0xFF if chunk is smaller than 4 bytes
            for j in chunk.len()..4 {
                word |= 0xFF_u32 << (j * 8);
            }

            // Write word to flash
            let addr = address + (i * 4) as u32;
            unsafe {
                core::ptr::write_volatile(addr as *mut u32, word);
            }

            // Wait for the write to complete
            self.wait_for_last_operation()?;
        }

        // Clear PG bit
        self.flash.cr().modify(|_, w| w.pg().clear_bit());

        // Lock flash
        self.lock();
        
        Ok(())
    }
}