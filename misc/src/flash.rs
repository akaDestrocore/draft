#![no_std]

use crate::xmodem::XmodemError;
use core::ptr::{read_volatile, write_volatile};
use stm32f4::{self as pac, Peripherals};

pub const FLASH_KEY1: u32 = 0x45670123;
pub const FLASH_KEY2: u32 = 0xCDEF89AB;

// STM32F4 flash sector layout
pub struct FlashSector {
    pub address: u32,
    pub size: u32,
}

const FLASH_SECTORS: [FlashSector; 12] = [
    FlashSector { address: 0x08000000, size: 16 * 1024 },    // Sector 0
    FlashSector { address: 0x08004000, size: 16 * 1024 },    // Sector 1
    FlashSector { address: 0x08008000, size: 16 * 1024 },    // Sector 2
    FlashSector { address: 0x0800C000, size: 16 * 1024 },    // Sector 3
    FlashSector { address: 0x08010000, size: 64 * 1024 },    // Sector 4
    FlashSector { address: 0x08020000, size: 128 * 1024 },   // Sector 5
    FlashSector { address: 0x08040000, size: 128 * 1024 },   // Sector 6
    FlashSector { address: 0x08060000, size: 128 * 1024 },   // Sector 7
    FlashSector { address: 0x08080000, size: 128 * 1024 },   // Sector 8
    FlashSector { address: 0x080A0000, size: 128 * 1024 },   // Sector 9
    FlashSector { address: 0x080C0000, size: 128 * 1024 },   // Sector 10
    FlashSector { address: 0x080E0000, size: 128 * 1024 },   // Sector 11
];

fn wait_for_last_operation(&self) -> Result<(), XmodemError> {
    let max_retries: i32 = 50000;
    let mut retries: i32 = 0;
    
    while 

    while self.is_busy() && retries < max_retries {
        retries += 1;
    }
    
    if retries >= max_retries {
        return Err(XmodemError::FlashError);
    }
    
    // Check for errors
    let sr = unsafe { read_volatile(&self.get_registers().sr) };
    if sr & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR) != 0 {
        return Err(XmodemError::FlashError);
    }
    
    Ok(())
}

fn is_busy(&self) -> bool {
    let sr = unsafe { read_volatile(&self.get_registers().sr) };
    (sr & FLASH_SR_BSY) != 0
}

fn unlock(&self) -> Result<(), XmodemError> {
    // Check if already unlocked
    let cr = unsafe { read_volatile(&self.get_registers().cr) };
    if (cr & FLASH_CR_LOCK) == 0 {
        return Ok(());
    }
    
    // Write unlock keys
    unsafe {
        write_volatile(&mut self.get_registers().keyr, FLASH_KEY1);
        write_volatile(&mut self.get_registers().keyr, FLASH_KEY2);
    }
    
    // Verify unlock
    let cr = unsafe { read_volatile(&self.get_registers().cr) };
    if (cr & FLASH_CR_LOCK) != 0 {
        return Err(XmodemError::FlashError);
    }
    
    Ok(())
}

fn lock(&self) {
    unsafe {
        let mut cr = read_volatile(&self.get_registers().cr);
        cr |= FLASH_CR_LOCK;
        write_volatile(&mut self.get_registers().cr, cr);
    }
}

fn find_sector(&self, address: u32) -> Option<usize> {
    FLASH_SECTORS.iter().position(|sector| 
        address >= sector.address && 
        address < (sector.address + sector.size)
    )
}

fn erase_sector(&self, sector_number: usize) -> Result<(), XmodemError> {
    self.wait_for_last_operation()?;
    
    unsafe {
        // Set sector erase mode
        let mut cr = read_volatile(&self.get_registers().cr);
        cr &= !(FLASH_CR_SNB_MASK << FLASH_CR_SNB_SHIFT);
        cr |= (sector_number as u32 & 0xF) << FLASH_CR_SNB_SHIFT;
        cr |= FLASH_CR_SER;
        write_volatile(&mut self.get_registers().cr, cr);
        
        // Start the erase operation
        cr |= FLASH_CR_STRT;
        write_volatile(&mut self.get_registers().cr, cr);
    }
    
    self.wait_for_last_operation()?;
    
    // Clear sector erase bit
    unsafe {
        let mut cr = read_volatile(&self.get_registers().cr);
        cr &= !FLASH_CR_SER;
        write_volatile(&mut self.get_registers().cr, cr);
    }
    
    Ok(())
}

fn program_word(&self, address: u32, data: u32) -> Result<(), XmodemError> {
    self.wait_for_last_operation()?;
    
    unsafe {
        // Set programming mode
        let mut cr = read_volatile(&self.get_registers().cr);
        cr &= !(FLASH_CR_PSIZE_64); // Clear PSIZE bits
        cr |= FLASH_CR_PSIZE_32;    // Set 32-bit programming
        cr |= FLASH_CR_PG;          // Set programming bit
        write_volatile(&mut self.get_registers().cr, cr);
        
        // Write data
        write_volatile(address as *mut u32, data);
    }
    
    self.wait_for_last_operation()?;
    
    // Verify written data
    let read_data = unsafe { read_volatile(address as *const u32) };
    if read_data != data {
        return Err(XmodemError::FlashError);
    }
    
    // Clear programming bit
    unsafe {
        let mut cr = read_volatile(&self.get_registers().cr);
        cr &= !FLASH_CR_PG;
        write_volatile(&mut self.get_registers().cr, cr);
    }
    
    Ok(())
}

impl crate::xmodem::FlashOperations for Stm32f4Flash {
    fn erase(&self, address: u32) -> Result<(), XmodemError> {
        // Find sector to erase
        let sector_number = match self.find_sector(address) {
            Some(sector) => sector,
            None => return Err(XmodemError::FlashError),
        };
        
        self.unlock()?;
        let result = self.erase_sector(sector_number);
        self.lock();
        
        result
    }
    
    fn write(&self, address: u32, data: &[u8]) -> Result<(), XmodemError> {
        if data.is_empty() {
            return Ok(());
        }
        
        // Ensure alignment
        if address % 4 != 0 {
            return Err(XmodemError::FlashError);
        }
        
        self.unlock()?;
        
        // Write data in 32-bit chunks
        for (i, chunk) in data.chunks(4).enumerate() {
            let mut word: u32 = 0;
            
            // Convert chunk to u32
            for j in 0..chunk.len() {
                word |= (chunk[j] as u32) << (8 * j);
            }
            
            // If chunk is less than 4 bytes, fill remaining with 0xFF
            for j in chunk.len()..4 {
                word |= 0xFF_u32 << (8 * j);
            }
            
            let offset = i * 4;
            self.program_word(address + (offset as u32), word)?;
        }
        
        self.lock();
        Ok(())
    }
}