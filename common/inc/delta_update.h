#ifndef _DELTA_UPDATE_H
#define _DELTA_UPDATE_H

#include "flash.h"
#include "crc.h"
#include "janpatch.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Apply a delta patch to a firmware image
 * 
 * @param source_addr Address of the source firmware
 * @param patch_addr Address of the patch data
 * @param target_addr Address to write the patched firmware
 * @param source_size Size of the source firmware in bytes
 * @param patch_size Size of the patch data in bytes
 * @return int 0 on success, non-zero on failure
 */
int apply_delta_patch(uint32_t source_addr, uint32_t patch_addr, uint32_t target_addr,
                      uint32_t source_size, uint32_t patch_size);

/**
 * @brief Verify the patched firmware
 * 
 * @param target_addr Address of the patched firmware
 * @param header_size Size of the firmware header
 * @return int 1 if valid, 0 if invalid
 */
int verify_patched_firmware(uint32_t target_addr, uint32_t header_size);

#endif /* _DELTA_UPDATE_H */