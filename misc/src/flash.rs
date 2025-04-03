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

/// Finds the sector number corresponding to the given address
pub fn get_sector_number(address: u32) -> Option<u8> {
    if address < FLASH_BASE {
        return None;
    }
    
    let offset = address - FLASH_BASE;
    let mut current_offset = 0;
    
    for (i, &size) in FLASH_SECTORS.iter().enumerate() {
        let sector_size = size * 1024;
        if offset >= current_offset && offset < current_offset + sector_size {
            return Some(i as u8);
        }
        current_offset += sector_size;
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
pub fn erase(p: &pac::Peripherals, destination: u32) -> bool {
    // Check for existing flash errors
    if !wait_for_last_operation(p) {
        return false;
    }

    // Find sector number for the address
    let start_sector = match get_sector_number(destination) {
        Some(s) => s,
        None => return false,
    };

    // Unlock flash
    if !unlock(p) {
        return false;
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
            return false;
        }

        // Clear SER bit
        p.flash.cr().modify(|_, w| w.ser().clear_bit());
    }

    // Lock flash
    lock(p);
    
    true
}

/// Writes data to flash at the given address
/// Returns 0 on success, error code otherwise
pub fn write(p: &pac::Peripherals, source_data: &[u8], destination: u32) -> u8 {
    if source_data.is_empty() {
        return 0;
    }

    // Для STM32F4 при напряжении 2.7V-3.6V используем 32-bit доступ
    let block_size = 4; // 32-bit

    // Проверяем, что длина данных кратна размеру блока
    if source_data.len() % block_size as usize != 0 {
        return 2;
    }

    // Unlock flash
    if !unlock(p) {
        return 1;
    }

    // Используем программирование 32-bit словами
    unsafe {
        p.flash.cr().modify(|_, w| w
            .psize().bits(2) // 2 = 32-bit для напряжения 2.7V-3.6V
        );
    }

    // Программируем данные блоками
    for i in (0..source_data.len()).step_by(block_size as usize) {
        let addr = destination + i as u32;
        
        // Конструируем 32-bit слово из байтов
        let mut data: u32 = 0;
        for j in 0..block_size {
            if i + (j as usize) < source_data.len() {
                data |= (source_data[i + j as usize] as u32) << (j * 8);
            }
        }

        // Активируем режим программирования
        p.flash.cr().modify(|_, w| w.pg().program());

        // Запись данных во флеш
        unsafe { 
            ptr::write_volatile(addr as *mut u32, data);
        }

        // Ждем завершения операции
        if !wait_for_last_operation(p) {
            p.flash.cr().modify(|_, w| w.pg().clear_bit());
            lock(p);
            return 1;
        }
    }

    // Сбрасываем бит программирования
    p.flash.cr().modify(|_, w| w.pg().clear_bit());
    
    // Блокируем флеш-контроллер
    lock(p);
    
    0 // Успех
}

/// Reads data from flash into the provided buffer
pub fn read(source: u32, destination: &mut [u8]) {
    for (i, byte) in destination.iter_mut().enumerate() {
        unsafe {
            *byte = ptr::read_volatile((source + i as u32) as *const u8);
        }
    }
}