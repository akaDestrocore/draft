#ifndef _CRC_H
#define _CRC_H

#include <stdint.h>
#include <stddef.h>

// Initialize CRC hardware unit
void crc_init(void);

// Reset CRC calculation
void crc_reset(void);

// Calculate CRC for a buffer
uint32_t crc_calculate(const uint8_t* data, size_t len);

// Calculate CRC for data of selected size in flash memory
uint32_t crc_calculate_memory(uint32_t addr, uint32_t size);

// Verify image CRC
int verify_firmware_crc(uint32_t addr, uint32_t header_size);

// Invalidate image by corrupting the header
int invalidate_firmware(uint32_t addr);

#endif /* _CRC_H */