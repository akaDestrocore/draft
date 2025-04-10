#![no_std]

use core::ptr;
use cortex_m::asm;
use cortex_m::peripheral::SCB;
use crate::image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_MAGIC_LOADER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER, IMAGE_TYPE_LOADER};
use stm32f4 as pac;

// Configuration structure for boot addresses
#[derive(Clone, Copy)]
pub struct BootConfig {
    pub app_addr: u32,
    pub updater_addr: u32,
    pub loader_addr: u32,
    pub image_hdr_size: u32,
}

#[derive(PartialEq, Copy, Clone)]
pub enum BootOption {
    None,
    Application,
    Updater,
    Loader,
    SelectUpdateTarget,
}

pub fn is_firmware_valid(addr: u32, config: &BootConfig) -> bool {
    let header: ImageHeader = unsafe { ptr::read_volatile(addr as *const ImageHeader) };
    
    match addr {
        addr if addr == config.app_addr => header.image_magic == IMAGE_MAGIC_APP,
        addr if addr == config.updater_addr => header.image_magic == IMAGE_MAGIC_UPDATER,
        addr if addr == config.loader_addr => header.image_magic == IMAGE_MAGIC_LOADER,
        _ => false,
    }
}

fn deinit_peripherals(p: &pac::Peripherals) {
    // Reset all peripheral clocks
    p.rcc.apb1rstr().write(|w| unsafe { w.bits(0xF6FEC9FF) });
    p.rcc.apb1rstr().write(|w| unsafe { w.bits(0x0) });

    p.rcc.apb2rstr().write(|w| unsafe { w.bits(0x04777933) });
    p.rcc.apb2rstr().write(|w| unsafe { w.bits(0x0) });

    p.rcc.ahb1rstr().write(|w| unsafe { w.bits(0x226011FF) });
    p.rcc.ahb1rstr().write(|w| unsafe { w.bits(0x0) });

    p.rcc.ahb2rstr().write(|w| unsafe { w.bits(0x000000C1) });
    p.rcc.ahb2rstr().write(|w| unsafe { w.bits(0x0) });

    p.rcc.ahb3rstr().write(|w| unsafe { w.bits(0x00000001) });
    p.rcc.ahb3rstr().write(|w| unsafe { w.bits(0x0) });
}

fn reset_system_clock(p: &pac::Peripherals) {
    // Enable HSI
    p.rcc.cr().modify(|_, w| w.hsion().set_bit());
    while p.rcc.cr().read().hsirdy().bit_is_clear() {
        // wait
    }

    // Set HSITRIM to default
    p.rcc.cr().modify(|_, w| unsafe {
        w.hsitrim().bits(0x10)
    });

    // Reset CFGR
    p.rcc.cfgr().reset();
    while !p.rcc.cfgr().read().sws().is_hsi() {
        // wait
    }

    // Disable HSE, CSS, HSEBYP
    p.rcc.cr().modify(|_, w| w
        .hseon().clear_bit()
        .hsebyp().clear_bit()
        .csson().clear_bit()
    );
    while p.rcc.cr().read().hserdy().bit_is_set() {
        // wait
    }

    // Disable PLL
    p.rcc.cr().modify(|_, w| w.pllon().clear_bit());
    while p.rcc.cr().read().pllrdy().bit_is_set() {
        // wait
    }

    // Reset PLL configuration
    p.rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllm().bits(0x10)
        .plln().bits(0x040)
        .pllp().bits(0x080)
        .pllq().bits(0x4)
    });

    // Disable clock interrupts
    p.rcc.cir().modify(|_, w| w
        .lsirdyie().clear_bit()
        .lserdyie().clear_bit()
        .hsirdyie().clear_bit()
        .hserdyie().clear_bit()
        .pllrdyie().clear_bit()
    );

    // Clear clock interrupt flags
    p.rcc.cir().modify(|_, w| w
        .lsirdyc().set_bit()
        .lserdyc().set_bit()
        .hsirdyc().set_bit()
        .hserdyc().set_bit()
        .pllrdyc().set_bit()
        .cssc().set_bit()
    );

    // Reset CSR flags
    p.rcc.csr().modify(|_, w| w.rmvf().set_bit());
}

/// Common boot code preparation
fn prepare_boot(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals, addr: u32, image_hdr_size: u32) {
    reset_system_clock(p);
    deinit_peripherals(p);

    // Memory remap
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());
    p.syscfg.memrmp().write(|w| unsafe {
        w.bits(0x01)
    });

    // Disable SysTick
    cp.SYST.disable_counter();
    cp.SYST.disable_interrupt();

    unsafe {
        // Clear pending exceptions
        let scb = SCB::ptr();
        
        let icsr: u32 = (*scb).icsr.read();
        (*scb).icsr.write(icsr | (1 << 25)); // Clear PENDSTCLR

        // Disable fault handlers
        (*scb).shcsr.modify(|v: u32| v & !( (1 << 18) | (1 << 17) | (1 << 16) ));

        // Set vector table offset
        (*scb).vtor.write(addr + image_hdr_size);
    }
}

pub fn boot_application(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals, config: &BootConfig) -> ! {
    if !is_firmware_valid(config.app_addr, config) {
        loop {
            asm::nop();
        }
    }

    prepare_boot(p, cp, config.app_addr, config.image_hdr_size);

    let stack_addr: u32 = unsafe {
        ptr::read_volatile((config.app_addr + config.image_hdr_size) as *const u32)
    };
    
    let reset_vector: u32 = unsafe {
        ptr::read_volatile((config.app_addr + config.image_hdr_size + 4) as *const u32)
    };

    unsafe {
        // Set SP
        core::arch::asm!("MSR msp, {0}", in(reg) stack_addr);

        // Jump to reset handler
        let jump_fn: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
        jump_fn();
    }
}

pub fn boot_updater(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals, config: &BootConfig) -> ! {
    if !is_firmware_valid(config.updater_addr, config) {
        loop {
            asm::nop();
        }
    }

    prepare_boot(p, cp, config.updater_addr, config.image_hdr_size);

    let stack_addr: u32 = unsafe {
        ptr::read_volatile((config.updater_addr + config.image_hdr_size) as *const u32)
    };
    
    let reset_vector: u32 = unsafe {
        ptr::read_volatile((config.updater_addr + config.image_hdr_size + 4) as *const u32)
    };

    unsafe {
        // Set SP
        core::arch::asm!("MSR msp, {0}", in(reg) stack_addr);

        // Jump to reset handler
        let jump_fn: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
        jump_fn();
    }
}

pub fn boot_loader(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals, config: &BootConfig) -> ! {
    if !is_firmware_valid(config.loader_addr, config) {
        loop {
            asm::nop();
        }
    }

    prepare_boot(p, cp, config.loader_addr, config.image_hdr_size);

    let stack_addr: u32 = unsafe {
        ptr::read_volatile((config.loader_addr + config.image_hdr_size) as *const u32)
    };
    
    let reset_vector: u32 = unsafe {
        ptr::read_volatile((config.loader_addr + config.image_hdr_size + 4) as *const u32)
    };

    unsafe {
        // Set SP
        core::arch::asm!("MSR msp, {0}", in(reg) stack_addr);

        // Jump to reset handler
        let jump_fn: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
        jump_fn();
    }
}

pub fn get_firmware_header(addr: u32, config: &BootConfig) -> Option<ImageHeader> {
    // Read header from flash memory
    let header: ImageHeader = unsafe { ptr::read_volatile(addr as *const ImageHeader) };
    
    match addr {
        addr if addr == config.app_addr && header.image_magic == IMAGE_MAGIC_APP => Some(header),
        addr if addr == config.updater_addr && header.image_magic == IMAGE_MAGIC_UPDATER => Some(header),
        addr if addr == config.loader_addr && header.image_magic == IMAGE_MAGIC_LOADER => Some(header),
        _ => None,
    }
}