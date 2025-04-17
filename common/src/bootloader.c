#include "bootloader.h"

int is_firmware_valid(uint32_t addr, const BootConfig_t* config) {
    // Read header from flash memory
    ImageHeader_t header;
    memcpy(&header, (void*)addr, sizeof(ImageHeader_t));
    
    // Check if the magic number matches
    if (addr == config->app_addr) {
        return header.image_magic == IMAGE_MAGIC_APP;
    } else if (addr == config->updater_addr) {
        return header.image_magic == IMAGE_MAGIC_UPDATER;
    } else if (addr == config->loader_addr) {
        return header.image_magic == IMAGE_MAGIC_LOADER;
    }
    
    return 0; // Invalid addr
}

static void deinit_peripherals(void) {
    // Reset all peripheral clocks
    RCC->APB1RSTR = 0xF6FEC9FF;
    RCC->APB1RSTR = 0x0;
    
    RCC->APB2RSTR = 0x04777933;
    RCC->APB2RSTR = 0x0;
    
    RCC->AHB1RSTR = 0x226011FF;
    RCC->AHB1RSTR = 0x0;
    
    RCC->AHB2RSTR = 0x000000C1;
    RCC->AHB2RSTR = 0x0;
    
    RCC->AHB3RSTR = 0x00000001;
    RCC->AHB3RSTR = 0x0;
}

static void reset_system_clock(void) {
    // Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY)) {
        // Wait
    }
    
    // Set HSITRIM to default
    RCC->CR = (RCC->CR & ~RCC_CR_HSITRIM) | (0x10 << RCC_CR_HSITRIM_Pos);
    
    // Reset CFGR
    RCC->CFGR = 0;
    while((RCC->CFGR & RCC_CFGR_SWS) != 0) {
        // Wait
    }
    
    // Disable HSE, CSS, HSEBYP
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSON);
    while(RCC->CR & RCC_CR_HSERDY) {
        // Wait
    }
    
    // Disable PLL
    RCC->CR &= ~RCC_CR_PLLON;
    while(RCC->CR & RCC_CR_PLLRDY) {
        // Wait
    }
    
    // Reset PLL configuration
    RCC->PLLCFGR = 0x24003010;

    // Disable clock interrupts
    RCC->CIR = 0;
}

void prepare_for_boot(uint32_t addr, uint32_t header_size) {
    // Reset clock and peripherals
    reset_system_clock();
    deinit_peripherals();
    
    // Memory remap
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->MEMRMP = 0x01;
    
    // Disable SysTick
    SysTick->CTRL = 0;
    
    // Clear pending exceptions
    SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;
    
    // Disable fault handlers
    SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | 
                    SCB_SHCSR_BUSFAULTENA_Msk | 
                    SCB_SHCSR_MEMFAULTENA_Msk);
    
    // Set vector table
    SCB->VTOR = addr + header_size;
}

void boot_application(const BootConfig_t* config) {
    if (!is_firmware_valid(config->app_addr, config)) {
        return; // don't boot
    }
    
    prepare_for_boot(config->app_addr, config->image_hdr_size);
    
    // Read SP and reset vector from vector table
    uint32_t app_vector_table = config->app_addr + config->image_hdr_size;
    uint32_t sp = *((uint32_t*)app_vector_table);
    uint32_t reset_handler = *((uint32_t*)(app_vector_table + 4));
    
    // Disable all interrupts
    __disable_irq();
    
    // Set MSP
    __set_MSP(sp);
    
    // Jump to reset handler
    void (*jump_to_app)(void) = (void (*)(void))reset_handler;
    jump_to_app();

}

void boot_updater(const BootConfig_t* config) {
    if (!is_firmware_valid(config->updater_addr, config)) {
        return; // Invalid updater - don't boot
    }
    
    prepare_for_boot(config->updater_addr, config->image_hdr_size);
    
    // Read SP and reset vector from vector table
    uint32_t updater_vector_table = config->updater_addr + config->image_hdr_size;
    uint32_t sp = *((uint32_t*)updater_vector_table);
    uint32_t reset_handler = *((uint32_t*)(updater_vector_table + 4));
    
    // Disable all interrupts
    __disable_irq();
    
    // Set MSP
    __set_MSP(sp);
    
    // Jump to reset handler
    void (*jump_to_updater)(void) = (void (*)(void))reset_handler;
    jump_to_updater();
    
}

void boot_loader(const BootConfig_t* config) {
    if (!is_firmware_valid(config->loader_addr, config)) {
        return; // don't boot
    }
    
    prepare_for_boot(config->loader_addr, config->image_hdr_size);
    
    // Read SP and reset vector from vector table
    uint32_t loader_vector_table = config->loader_addr + config->image_hdr_size;
    uint32_t sp = *((uint32_t*)loader_vector_table);
    uint32_t reset_handler = *((uint32_t*)(loader_vector_table + 4));
    
    // Disable all interrupts
    __disable_irq();
    
    // Set MSP
    __set_MSP(sp);
    
    // Jump to reset handler
    void (*jump_to_loader)(void) = (void (*)(void))reset_handler;
    jump_to_loader();

}

int get_firmware_header(uint32_t addr, const BootConfig_t* config, ImageHeader_t* header) {
    // Read header from flash
    memcpy(header, (void*)addr, sizeof(ImageHeader_t));
    
    // Validate
    if (addr == config->app_addr && header->image_magic == IMAGE_MAGIC_APP) {
        return 1;
    } else if (addr == config->updater_addr && header->image_magic == IMAGE_MAGIC_UPDATER) {
        return 1;
    } else if (addr == config->loader_addr && header->image_magic == IMAGE_MAGIC_LOADER) {
        return 1;
    }
    
    return 0; // Invalid header
}