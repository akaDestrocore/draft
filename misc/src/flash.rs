#![no_std]

use core::ptr;
use stm32f4 as pac;
use crate::systick;

// Flash sector sizes in kilobytes
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

// Base address of flash memory
pub const FLASH_BASE: u32 = 0x08000000;

pub const FLASH_SECTOR_TOTAL: u8 = 12;

// Unlocks the flash control register
pub fn unlock(p: &pac::Peripherals) -> bool {
    // Check if already unlocked
    if p.flash.cr().read().lock().is_unlocked() {
        return true;
    }

    // Write the key sequence
    unsafe {
        p.flash.keyr().write(|w| w.key().set(0x45670123));
        p.flash.keyr().write(|w| w.key().set(0xCDEF89AB));
    }

    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 1) {
        cortex_m::asm::nop();
    }

    // Verify unlock was successful
    p.flash.cr().read().lock().is_unlocked()
}

// Locks the flash control register
pub fn lock(p: &pac::Peripherals) {
    p.flash.cr().modify(|_, w| w.lock().locked());
    
    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 1) {
        cortex_m::asm::nop();
    }
}

// Wait for the last flash operation to complete
pub fn wait_for_last_operation(p: &pac::Peripherals) -> bool {
    // Clear any existing error flags first
    p.flash.sr().modify(|_, w| w
        .pgserr().clear()
        .pgperr().clear()
        .pgaerr().clear()
        .wrperr().clear()
        .operr().clear()
    );

    // Wait for busy flag to clear with timeout
    let start_ms: u32 = systick::get_tick_ms();
    let timeout_ms: u32 = 1000;
    
    while p.flash.sr().read().bsy().is_busy() {
        if systick::wait_ms(start_ms, timeout_ms) {
            // timeout
            return false;
        }
        cortex_m::asm::wfi();
    }

    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 5) {
        cortex_m::asm::nop();
    }

    let sr = p.flash.sr().read();
    
    // Check all possible error flags
    if sr.pgserr().is_active() || 
       sr.pgperr().is_active() || 
       sr.pgaerr().is_active() || 
       sr.wrperr().is_active() || 
       sr.operr().is_active() {
        
        // Clear error flags by writing 1 to them
        p.flash.sr().modify(|_, w| w
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
pub fn get_sector_number(address: u32) -> Option<u8> {
    if address < FLASH_BASE {
        return None;
    }
    
    let offset: u32 = address - FLASH_BASE;
    let mut current_offset: u32 = 0;
    
    for (i, &size) in FLASH_SECTORS.iter().enumerate() {
        let sector_size = size * 1024;
        if offset >= current_offset && offset < current_offset + sector_size {
            return Some(i as u8);
        }
        current_offset += sector_size;
    }
    
    None
}

pub fn erase_sector(p: &pac::Peripherals, destination: u32) -> u32 {
    // Clear any existing errors and check if Flash is ready
    if !wait_for_last_operation(p) {
        return 0;
    }

    // Find sector number for the address
    let sector: u8 = match get_sector_number(destination) {
        Some(s) => s,
        None => return 0,
    };

    // Unlock flash
    if !unlock(p) {
        return 0;
    }

    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 5) {
        cortex_m::asm::nop();
    }

    // Configure sector erase with correct PSIZE
    unsafe {
        // First reset all bits we're going to modify
        p.flash.cr().modify(|_, w| w
            .psize().bits(0)
            .ser().clear_bit()
            .snb().bits(0)
        );
        
        let start_ms: u32 = systick::get_tick_ms();
        while !systick::wait_ms(start_ms, 1) {
            cortex_m::asm::nop();
        }
        
        // Set all the parameters properly
        p.flash.cr().modify(|_, w| w
            .psize().psize32()  // Set 32-bit PSIZE for 2.7V-3.6V
            .ser().sector_erase()
            .snb().bits(sector)
        );

        let start_ms: u32 = systick::get_tick_ms();
        while !systick::wait_ms(start_ms, 1) {
            cortex_m::asm::nop();
        }

        // Start the erase operation
        p.flash.cr().modify(|_, w| w.strt().start());
    }

    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 10) {
        cortex_m::asm::nop();
    }

    // Wait for operation to complete with longer timeout
    if !wait_for_last_operation(p) {
        // Clear SER bit
        p.flash.cr().modify(|_, w| w.ser().clear_bit());
        lock(p);
        return 0;
    }

    // Clear SER bit
    p.flash.cr().modify(|_, w| w.ser().clear_bit());
    
    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 5) {
        cortex_m::asm::nop();
    }
    
    // Lock flash
    lock(p);

    // Return the size of the erased sector
    FLASH_SECTORS[sector as usize] * 1024
}

// Erase all flash sectors starting from the given address
pub fn erase(p: &pac::Peripherals, destination: u32) -> bool {
    // Check for existing flash errors
    if !wait_for_last_operation(p) {
        return false;
    }

    // Find sector number for the address
    let start_sector: u8 = match get_sector_number(destination) {
        Some(s) => s,
        None => return false,
    };

    // Unlock flash
    if !unlock(p) {
        return false;
    }

    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 5) {
        cortex_m::asm::nop();
    }

    for sector in start_sector..FLASH_SECTOR_TOTAL {
        // Configure sector erase
        unsafe {
            // Reset CR fields first
            p.flash.cr().modify(|_, w| w
                .psize().bits(0)
                .ser().clear_bit()
                .snb().bits(0)
            );
            
            let start_ms: u32 = systick::get_tick_ms();
            while !systick::wait_ms(start_ms, 1) {
                cortex_m::asm::nop();
            }
            
            // Set proper values
            p.flash.cr().modify(|_, w| w
                .psize().psize32()  // Set 32-bit PSIZE
                .ser().sector_erase()
                .snb().bits(sector)
            );

            let start_ms: u32 = systick::get_tick_ms();
            while !systick::wait_ms(start_ms, 1) {
                cortex_m::asm::nop();
            }

            // Start the erase operation
            p.flash.cr().modify(|_, w| w.strt().start());
        }

        let start_ms: u32 = systick::get_tick_ms();
        while !systick::wait_ms(start_ms, 10) {
            cortex_m::asm::nop();
        }

        // Wait for operation to complete
        if !wait_for_last_operation(p) {
            // Clear SER bit and exit on error
            p.flash.cr().modify(|_, w| w.ser().clear_bit());
            lock(p);
            return false;
        }

        // Clear SER bit
        p.flash.cr().modify(|_, w| w.ser().clear_bit());
        
        let start_ms: u32 = systick::get_tick_ms();
        while !systick::wait_ms(start_ms, 5) {
            cortex_m::asm::nop();
        }
    }

    // Lock flash
    lock(p);
    
    true
}

pub fn write(p: &pac::Peripherals, source_data: &[u8], destination: u32) -> u8 {
    if source_data.is_empty() {
        return 0;
    }

    let block_size: i32 = 4; // 32-bit

    if source_data.len() % block_size as usize != 0 {
        return 2;
    }

    // Check for existing errors
    if !wait_for_last_operation(p) {
        return 3;
    }

    // Unlock flash
    if !unlock(p) {
        return 1;
    }

    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 5) {
        cortex_m::asm::nop();
    }

    // Reset CR register first
    unsafe {
        p.flash.cr().modify(|_, w| w
            .pg().clear_bit()
            .psize().bits(0)
        );
        
        let start_ms: u32 = systick::get_tick_ms();
        while !systick::wait_ms(start_ms, 1) {
            cortex_m::asm::nop();
        }
    }

    p.flash.cr().modify(|_, w| w
        .psize().psize32() // 2 = 32-bit for 2.7V-3.6V
    );

    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 1) {
        cortex_m::asm::nop();
    }

    for i in (0..source_data.len()).step_by(block_size as usize) {
        let addr: u32 = destination + i as u32;
        
        let mut data: u32 = 0;
        for j in 0..block_size {
            if i + (j as usize) < source_data.len() {
                data |= (source_data[i + j as usize] as u32) << (j * 8);
            }
        }

        p.flash.cr().modify(|_, w| w.pg().program());

        let start_ms: u32 = systick::get_tick_ms();
        while !systick::wait_ms(start_ms, 1) {
            cortex_m::asm::nop();
        }

        unsafe { 
            ptr::write_volatile(addr as *mut u32, data);
        }
        
        let start_ms: u32 = systick::get_tick_ms();
        while !systick::wait_ms(start_ms, 1) {
            cortex_m::asm::nop();
        }

        if !wait_for_last_operation(p) {
            p.flash.cr().modify(|_, w| w.pg().clear_bit());
            lock(p);
            return 4;
        }
        
        // Clear PG bit between writes
        p.flash.cr().modify(|_, w| w.pg().clear_bit());

        let start_ms: u32 = systick::get_tick_ms();
        while !systick::wait_ms(start_ms, 1) {
            cortex_m::asm::nop();
        }
    }
    
    // Lock flash
    lock(p);
    
    0 // Успех
}

// Read data from flash into the provided buffer
pub fn read(source: u32, destination: &mut [u8]) {

    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 1) {
        cortex_m::asm::nop();
    }
    
    for (i, byte) in destination.iter_mut().enumerate() {
        unsafe {
            *byte = ptr::read_volatile((source + i as u32) as *const u8);
        }
    }
    
    let start_ms: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_ms, 1) {
        cortex_m::asm::nop();
    }
}