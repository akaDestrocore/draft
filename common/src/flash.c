#include "flash.h"

static const uint32_t FLASH_SECTORS_KB[] = {
    16,  // sector 0
    16,  // sector 1
    16,  // sector 2
    16,  // sector 3
    64,  // sector 4
    128, // sector 5
    128, // sector 6
    128, // sector 7
    128, // sector 8
    128, // sector 9
    128, // sector 10
    128  // sector 11
};

static const uint8_t FLASH_SECTOR_COUNT = sizeof(FLASH_SECTORS_KB) / sizeof(FLASH_SECTORS_KB[0]);

int flash_unlock(void) {
    // Check if already unlocked
    if ((FLASH->CR & FLASH_CR_LOCK) == 0) {
        return 1;
    }
    
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    return (status == HAL_OK) ? 1 : 0;
}

void flash_lock(void) {
    HAL_FLASH_Lock();
}

int flash_wait_for_last_operation(void) {
    uint32_t timeout = HAL_GetTick() + 100; // 100ms
    
    while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
        if (HAL_GetTick() > timeout) {
            return 0;
        }
    }
    
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR) || 
        __HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) || 
        __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR) || 
        __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGPERR) || 
        __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR)) {
        
        // Clear error flags
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                             FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | 
                             FLASH_FLAG_PGSERR);
        return 0;
    }
    
    return 1;
}

uint8_t flash_get_sector(uint32_t addr) {
    if (addr < FLASH_BASE) {
        return 0xFF; // Invalid addr
    }
    
    uint32_t offset = addr - FLASH_BASE;
    uint32_t current_offset = 0;
    
    for (uint8_t i = 0; i < FLASH_SECTOR_COUNT; i++) {
        uint32_t sector_size_bytes = FLASH_SECTORS_KB[i] * 1024;
        if (offset >= current_offset && offset < current_offset + sector_size_bytes) {
            return i;
        }
        current_offset += sector_size_bytes;
    }
    
    return 0xFF; // addr
}

int flash_erase_sector(uint32_t sector_addr) {
    // Check if any pending operations
    if (HAL_FLASH_GetError() != HAL_FLASH_ERROR_NONE) {
        // Clear error flags
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                             FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | 
                             FLASH_FLAG_PGSERR);
        return 0;
    }
    
    // Get sector number
    uint8_t sector = flash_get_sector(sector_addr);
    if (sector == 0xFF) {
        return 0;
    }
    
    // Prepare for erase
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t SectorError = 0;
    
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V to 3.6V
    EraseInitStruct.Sector = sector;
    EraseInitStruct.NbSectors = 1;
    
    // Unlock flash
    if (!flash_unlock()) {
        return 0;
    }
    
    // Erase the sector
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    
    // Lock flash again
    flash_lock();
    
    if (status != HAL_OK) {
        return 0;
    }
    
    // Return the size of the erased sector in bytes
    return FLASH_SECTORS_KB[sector] * 1024;
}

int flash_erase(uint32_t destination) {
    // Check if any pending operations
    if (HAL_FLASH_GetError() != HAL_FLASH_ERROR_NONE) {
        return 0;
    }
    
    // Get starting sector
    uint8_t start_sector = flash_get_sector(destination);
    if (start_sector == 0xFF) {
        return 0;
    }
    
    // Unlock flash
    if (!flash_unlock()) {
        return 0;
    }
    
    // Erase all sectors from start_sector to the end
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t SectorError = 0;
    
    for (uint8_t i = start_sector; i < FLASH_SECTOR_COUNT; i++) {
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        EraseInitStruct.Sector = i;
        EraseInitStruct.NbSectors = 1;
        
        if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
            flash_lock();
            return 0;
        }
    }
    
    // Lock flash
    flash_lock();
    
    return 1;
}

int flash_write(uint32_t addr, const uint8_t* data, size_t len) {
    if (len == 0) {
        return 1; // Nothing to do
    }
    
    // Determine the programming word size
    uint8_t program_size;
    uint32_t program_type;
    
    uint32_t psize = (FLASH->CR & FLASH_CR_PSIZE) >> FLASH_CR_PSIZE_Pos;
    switch (psize) {
        case 0: // 8-bit
            program_size = 1;
            program_type = FLASH_TYPEPROGRAM_BYTE;
            break;
        case 1: // 16-bit
            program_size = 2;
            program_type = FLASH_TYPEPROGRAM_HALFWORD;
            break;
        case 2: // 32-bit (default)
            program_size = 4;
            program_type = FLASH_TYPEPROGRAM_WORD;
            break;
        case 3: // 64-bit
            program_size = 8;
            program_type = FLASH_TYPEPROGRAM_DOUBLEWORD;
            break;
        default:
            return 0; // Should never happen
    }
    
    // Check alignment
    if (len % program_size != 0) {
        return 0; // Data must be aligned to the programming size
    }
    
    // Wait for any previous operations
    if (!flash_wait_for_last_operation()) {
        return 0;
    }
    
    // Unlock
    if (!flash_unlock()) {
        return 0;
    }
    
    // Program flash
    for (size_t offset = 0; offset < len; offset += program_size) {
        uint32_t data_word = 0;
        memcpy(&data_word, data + offset, program_size);
        
        if (HAL_FLASH_Program(program_type, addr + offset, data_word) != HAL_OK) {
            flash_lock();
            return 0;
        }
        
        // Wait with timeout
        uint32_t timeout = HAL_GetTick() + 50; // 50ms
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
            if (HAL_GetTick() > timeout) {
                flash_lock();
                return 0;
            }
        }
        
        // Verify written data
        uint32_t read_data = 0;
        memcpy(&read_data, (void*)(addr + offset), program_size);
        if (read_data != data_word) {
            flash_lock();
            return 0;
        }
    }
    
    // Lock flash
    flash_lock();
    
    return 1;
}

void flash_read(uint32_t addr, uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        data[i] = *(__IO uint8_t*)(addr + i);
    }
}