#![no_std]

use core::ptr;
use stm32f4 as pac;

/// Flash sector sizes in kilobytes
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

/// Base address of flash memory
pub const FLASH_BASE: u32 = 0x08000000;

/// Total number of flash sectors
pub const FLASH_SECTOR_TOTAL: u8 = 12;

/// Unlocks the flash control register
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

    // Verify unlock was successful
    p.flash.cr().read().lock().is_unlocked()
}

/// Locks the flash control register
pub fn lock(p: &pac::Peripherals) {
    p.flash.cr().modify(|_, w| w.lock().locked());
}

/// Waits for the last flash operation to complete and checks for errors
pub fn wait_for_last_operation(p: &pac::Peripherals) -> bool {
    // Wait for busy flag to clear
    let mut timeout = 50000;
    while p.flash.sr().read().bsy().is_busy() {
        timeout -= 1;
        if timeout == 0 {
            return false;
        }
    }

    // Check for errors
    let sr = p.flash.sr().read();
    
    if sr.pgserr().is_active() || 
       sr.pgperr().is_active() || 
       sr.pgaerr().is_active() || 
       sr.wrperr().is_active() || 
       sr.operr().is_active() {
        
        // Clear error flags
        p.flash.sr().write(|w| w
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

/// Finds the sector number corresponding to the given address
pub fn get_sector_number(address: u32) -> Option<u8> {
    let mut addr = FLASH_BASE;
    
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

/// Erases a single flash sector at the given address
/// Returns the size of the erased sector in bytes (or 0 on error)
pub fn erase_sector(p: &pac::Peripherals, destination: u32) -> u32 {
    // Check for existing flash errors
    if p.flash.sr().read().bsy().is_busy() || !wait_for_last_operation(p) {
        return 0;
    }

    // Find sector number for the address
    let sector = match get_sector_number(destination) {
        Some(s) => s,
        None => return 0,
    };

    // Unlock flash
    if !unlock(p) {
        return 0;
    }

    // Configure sector erase
    unsafe {
        p.flash.cr().modify(|_, w| w
            .ser().sector_erase()
            .snb().bits(sector)
        );

        // Start the erase operation
        p.flash.cr().modify(|_, w| w.strt().start());
    }

    // Wait for operation to complete
    if !wait_for_last_operation(p) {
        // Clear SER bit
        p.flash.cr().modify(|_, w| w.ser().clear_bit());
        lock(p);
        return 0;
    }

    // Clear SER bit
    p.flash.cr().modify(|_, w| w.ser().clear_bit());
    
    // Lock flash
    lock(p);

    // Return the size of the erased sector
    FLASH_SECTORS[sector as usize] * 1024
}

/// Erases all flash sectors starting from the given address
pub fn erase(p: &pac::Peripherals, destination: u32) {
    // Check for existing flash errors
    if !wait_for_last_operation(p) {
        return;
    }

    // Find sector number for the address
    let start_sector = match get_sector_number(destination) {
        Some(s) => s,
        None => return,
    };

    // Unlock flash
    if !unlock(p) {
        return;
    }

    // Erase each sector from start_sector to the end
    for sector in start_sector..FLASH_SECTOR_TOTAL {
        // Configure sector erase
        unsafe {
            p.flash.cr().modify(|_, w| w
                .ser().sector_erase()
                .snb().bits(sector)
            );

            // Start the erase operation
            p.flash.cr().modify(|_, w| w.strt().start());
        }

        // Wait for operation to complete
        if !wait_for_last_operation(p) {
            // Clear SER bit and exit on error
            p.flash.cr().modify(|_, w| w.ser().clear_bit());
            lock(p);
            return;
        }

        // Clear SER bit
        p.flash.cr().modify(|_, w| w.ser().clear_bit());
    }

    // Lock flash
    lock(p);
}

/// Writes data to flash at the given address
/// Returns 0 on success, error code otherwise
pub fn write(p: &pac::Peripherals, source_data: &[u8], destination: u32) -> u8 {
    if source_data.is_empty() {
        return 0;
    }

    // Determine program size from CR register
    let psize_bits = p.flash.cr().read().psize().bits();
    let block_size = match psize_bits {
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
    if !unlock(p) {
        return 1;
    }

    // Program data in block_size chunks
    for i in (0..source_data.len()).step_by(block_size as usize) {
        let addr = destination + i as u32;
        
        // Construct data word from bytes
        let mut data: u32 = 0;
        for j in 0..block_size {
            if i + (j as usize) < source_data.len() {
                data |= (source_data[i + j as usize] as u32) << (j * 8);
            }
        }

        // Set programming mode
        p.flash.cr().modify(|_, w| w.pg().program());

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
        if !wait_for_last_operation(p) {
            p.flash.cr().modify(|_, w| w.pg().clear_bit());
            lock(p);
            return 1;
        }
    }

    // Clear programming bit
    p.flash.cr().modify(|_, w| w.pg().clear_bit());
    
    // Lock flash
    lock(p);
    
    0 // Success
}

/// Reads data from flash into the provided buffer
pub fn read(source: u32, destination: &mut [u8]) {
    for (i, byte) in destination.iter_mut().enumerate() {
        unsafe {
            *byte = ptr::read_volatile((source + i as u32) as *const u8);
        }
    }
}