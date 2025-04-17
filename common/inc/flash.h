#ifndef _FLASH_H
#define _FLASH_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>

// Flash base address
#define FLASH_BASE 0x08000000

int flash_unlock(void);
void flash_lock(void);
int flash_wait_for_last_operation(void);
uint8_t flash_get_sector(uint32_t address);
int flash_erase_sector(uint32_t sector_addr);
int flash_write(uint32_t address, const uint8_t* data, size_t len);
void flash_read(uint32_t address, uint8_t* data, size_t len);

#endif /* _FLASH_H */