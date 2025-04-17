#ifndef _BOOTLOADER_H
#define _BOOTLOADER_H

#include "image.h"
#include "flash.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

// Boot configuration struct
typedef struct {
    uint32_t app_addr;        // Application address
    uint32_t updater_addr;    // Updater address
    uint32_t loader_addr;     // Loader address
    uint32_t image_hdr_size;  // Size of image header
} BootConfig_t;

// Boot options
typedef enum {
    BOOT_OPTION_NONE,
    BOOT_OPTION_APPLICATION,
    BOOT_OPTION_UPDATER,
    BOOT_OPTION_LOADER,
    BOOT_OPTION_SELECT_UPDATE_TARGET
} BootOption_t;

// Check if firmware at given address is valid
int is_firmware_valid(uint32_t addr, const BootConfig_t* config);

// Boot to app
void boot_application(const BootConfig_t* config);

// Boot to updater
void boot_updater(const BootConfig_t* config);

// Boot to loader
void boot_loader(const BootConfig_t* config);

// Get image header
int get_firmware_header(uint32_t addr, const BootConfig_t* config, ImageHeader* header);

// Initialize system clock and peripherals before booting
void prepare_for_boot(uint32_t addr, uint32_t header_size);

#endif /* _BOOTLOADER_H */