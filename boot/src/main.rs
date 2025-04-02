#![no_std]
#![no_main]

use core::{f32::consts, panic::PanicInfo};
use cortex_m::{asm, peripheral::{self, scb, SCB, SYST}};
use cortex_m_rt::entry;
use stm32f4::{self as pac, Peripherals};
use misc::image::{ImageHeader, IMAGE_MAGIC_LOADER, IMAGE_MAGIC_UPDATER};

const LOADER_ADDR: u32 = 0x08004000;
const UPDATER_ADDR: u32 = 0x08008000;
const IMAGE_HEADER_SIZE: usize = core::mem::size_of::<ImageHeader>();

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take();

    if let Some(p) = peripherals {
        setup_system_clock(&p);

        let loader_is_valid: bool = check_loader_valid();
        
        prepare_for_jump(&p);

        if loader_is_valid {
            boot_to_image(LOADER_ADDR);
        } else {
            boot_to_image(UPDATER_ADDR);
        }
    }

    loop {
        asm::nop();
    }
}

fn setup_system_clock(p: &Peripherals) {
    // PWR clock
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());

    // Scale 1
    p.pwr.cr().modify(|_, w| w.vos().scale1());

    // flash latency
    p.flash.acr().modify(|_, w| w
        .latency().ws5()
        .prften().set_bit()
        .icen().set_bit()
        .dcen().set_bit()
    );

    // Enable HSE
    p.rcc.cr().modify(|_, w| w.hseon().set_bit());
    while p.rcc.cr().read().hserdy().bit_is_clear() {
        // wait
    }

    // PLL configuration
    p.rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllsrc().hse()
        .pllm().bits(4)
        .plln().bits(90)
        .pllp().div2()
        .pllq().bits(4)
    });

    // Enable PLL
    p.rcc.cr().modify(|_, w| w.pllon().set_bit());
    while p.rcc.cr().read().pllrdy().bit_is_clear() {
        // wait
    }

    // bus dividers
    p.rcc.cfgr().modify(|_, w| {
        w.hpre().div1()
        .ppre1().div4()
        .ppre2().div2()
    });

    // PLL as sys clock
    p.rcc.cfgr().modify(|_, w| w.sw().pll());
    while !p.rcc.cfgr().read().sws().is_pll() {
        // wait
    }
}

fn check_loader_valid() -> bool {
    unsafe {
        let header_ptr: *const ImageHeader = LOADER_ADDR as *const ImageHeader;
        
        // check if magic is correct
        if (*header_ptr).image_magic == IMAGE_MAGIC_LOADER {
            return true;
        }
        
        // if magic is incorrect then loader is corrupted
        false
    }
}

fn rcc_deinit(p: &Peripherals) {
    // Reset clock
    p.rcc.cr().modify(|_, w| w.hsion().set_bit());
    while p.rcc.cr().read().hsirdy().bit_is_clear() {
        // wait
    }

    // Set HSITRIM[4:0] bits to the reset value
    p.rcc.cr().modify(|_, w| unsafe {
        w.hsitrim().bits(0x10)
    });

    p.rcc.cfgr().reset();
    while !p.rcc.cfgr().read().sws().is_hsi() {
        // wait
    }

    p.rcc.cr().modify(|_, w| w
        .hseon().clear_bit()
        .hsebyp().clear_bit()
        .csson().clear_bit()
    );
    while p.rcc.cr().read().hserdy().bit_is_set() {
        // wait
    }

    //reset PLL
    p.rcc.cr().modify(|_, w| w.pllon().clear_bit());
    while p.rcc.cr().read().pllrdy().bit_is_set() {
        // wait
    }

    // reset PLL configuration
    p.rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllm().bits(0x10)
        .plln().bits(0x040)
        .pllp().bits(0x080)
        .pllq().bits(0x4)
    });

    // disable all interrupts
    p.rcc.cir().modify(|_, w| w
        .lsirdyie().clear_bit()
        .lserdyie().clear_bit()
        .hsirdyie().clear_bit()
        .pllrdyie().clear_bit()
    );
    p.rcc.cir().modify(|_, w| w
        .lsirdyc().clear_bit()
        .lserdyc().clear_bit()
        .hsirdyc().clear_bit()
        .pllrdyc().clear_bit()
    );

    // reset all CSR flags
    p.rcc.csr().modify(|_, w| w.rmvf().set_bit());
}

fn deinit(p: &Peripherals) {
    // force reset for all peripherals
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

fn prepare_for_jump(p: &Peripherals) {
    
    rcc_deinit(p);
    deinit(p);

    // remap
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());
    p.syscfg.memrmp().write(|w| unsafe {
        w.bits(0x01)
    });
}

fn boot_to_image(addr: u32) -> ! {
    let vectors_addr: u32 = addr + 0x200;
    
    // SP
    let stack_addr: u32 = unsafe {
        *(vectors_addr as *const u32)
    };
    
    // reset handler
    let reset_vector: u32 = unsafe {
        *((vectors_addr + 4) as *const u32)
    };

    // disable SysTick
    let mut cp: cortex_m::Peripherals = unsafe {
        cortex_m::Peripherals::steal()
    };
    cp.SYST.disable_counter();
    cp.SYST.disable_interrupt();

    // Clear pending SV
    unsafe {
        let scb = SCB::ptr();
        let icsr: u32 = (*scb).icsr.read();
        (*scb).icsr.write(icsr | (1 << 25));

        // disable fault handlers
        (*scb).shcsr.modify(|v: u32| v & ! (
            (1 << 18) | (1 << 17) | (1 << 16)
        ));

        // vector table
        (*scb).vtor.write(vectors_addr);

        // set msp
        core::arch::asm!("MSR msp, {0}", in(reg) stack_addr);

        let jump_fn: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
        jump_fn();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}