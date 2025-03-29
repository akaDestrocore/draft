#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::{asm, peripheral::{self, SCB, SYST}, register::{msp, psp}};
use cortex_m_rt::entry;
use stm32f4 as pac;

const LOADER_ADDR: u32 = 0x08004000;
const UPDATER_ADDR: u32 = 0x08008000;

#[entry]
fn main() -> ! {

    let p: stm32f4::Peripherals = pac::Peripherals::take().unwrap();
    let mut cp: cortex_m::Peripherals = cortex_m::Peripherals::take().unwrap();

    system_clock_config(&p);

    let loader_is_valid: bool = unsafe {
        let loader_value: u32 = *{LOADER_ADDR as *const u32};
        0xFFFFFFFF != loader_value
    };

    if !loader_is_valid {
        jump_to_updater(&p, &mut cp);
    } else {
        jump_to_loader(&p, &mut cp);
    }


    loop {
        asm::nop();
    }
}

fn system_clock_config(p: &pac::Peripherals) {
    let rcc = &p.rcc;
    let pwr = &p.pwr;
    let flash = &p.flash;
    
    // PWREN
    rcc.apb1enr().modify(|_, w| w.pwren().set_bit());
    // Enable VOS 1
    pwr.cr().modify(|_, w| w.vos().set_bit());
    // Enable HSE
    rcc.cr().modify(|_, w| w.hseon().set_bit());
    while rcc.cr().read().hserdy().bit_is_clear() {
        // wait
    }

    // PLL configuration
    rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllsrc().hse()
        .pllm().bits(4)
        .plln().bits(90)
        .pllp().div2()
        .pllq().bits(4)
    });

    // Enable PLL
    rcc.cr().modify(|_, w| w.pllon().set_bit());
    while rcc.cr().read().pllrdy().bit_is_clear() {
        // wait
    }

    // bus dividers
    rcc.cfgr().modify(|_, w| {
        w.hpre().div1()
        .ppre1().div4()
        .ppre2().div2()
    });

    // Flash latency
    flash.acr().modify(|_, w| unsafe {
        w.latency().bits(5)
        .prften().set_bit()
        .icen().set_bit()
        .dcen().set_bit()
    });

    // Make PLL system clock
    rcc.cfgr().modify(|_, w| w.sw().pll());
    while !rcc.cfgr().read().sws().is_pll() {
        // wait
    }

}

fn jump_to_updater(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) -> ! {
    
    let reset_addr: u32 = UPDATER_ADDR + 4;
    let reset_ptr: u32 = unsafe { *(reset_addr as *const u32) };
    let stack_ptr: u32 = unsafe { *(UPDATER_ADDR as *const u32) };

    p.rcc.cfgr().reset();
    p.rcc.cr().modify(|_, w| w.hsion().set_bit());
    
    // Wait for HSI to be ready
    while p.rcc.cr().read().hsirdy().bit_is_clear() {
        // wait
    }
    
    p.rcc.cr().modify(|_, w| w
        .hseon().clear_bit()
        .pllon().clear_bit()
    );
    while !p.rcc.cfgr().read().sws().is_hsi() {
        // wait
    }
    
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());

    // remap
    unsafe {
        p.syscfg.memrmp().write(|w| w.bits(0x01));
    }
    
    
    // Disable SysTick
    let systick: &mut SYST = &mut cp.SYST;
    systick.disable_counter();
    systick.disable_interrupt();
    
    // Clear pendSV
    unsafe {
        let scb: *mut peripheral::scb::RegisterBlock = SCB::PTR as *mut _;
        let icsr: u32 = (*scb).icsr.read();
        (*scb).icsr.write(icsr | (1 << 25)); // PENDSTCLR bit
    }
    
    unsafe {
        let scb: *mut peripheral::scb::RegisterBlock = SCB::PTR as *mut _;
        (*scb).shcsr.modify(|v| v & !(
            (1 << 18) | // USGFAULTENA
            (1 << 17) | // BUSFAULTENA 
            (1 << 16)   // MEMFAULTENA
        ));
        
        // Set vector table offset
        (*scb).vtor.write(UPDATER_ADDR);
        
        // SP
        core::arch::asm!("MSR msp, {0}", in(reg) stack_ptr);
        
        let jump_fn: unsafe extern "C" fn() -> ! = core::mem::transmute(reset_ptr);
        jump_fn();
    }
    
}

fn jump_to_loader(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) -> ! {
    
    let reset_addr: u32 = LOADER_ADDR + 4;
    let reset_ptr: u32 = unsafe { *(reset_addr as *const u32) };
    let stack_ptr: u32 = unsafe { *(LOADER_ADDR as *const u32) };

    p.rcc.cfgr().reset();
    p.rcc.cr().modify(|_, w| w.hsion().set_bit());
    
    // Wait for HSI to be ready
    while p.rcc.cr().read().hsirdy().bit_is_clear() {
        // wait
    }
    
    p.rcc.cr().modify(|_, w| w
        .hseon().clear_bit()
        .pllon().clear_bit()
    );
    while !p.rcc.cfgr().read().sws().is_hsi() {
        // wait
    }
    
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());

    // remap
    unsafe {
        p.syscfg.memrmp().write(|w| w.bits(0x01));
    }
    
    
    // Disable SysTick
    let systick: &mut SYST = &mut cp.SYST;
    systick.disable_counter();
    systick.disable_interrupt();
    
    // Clear pendSV
    unsafe {
        let scb: *mut peripheral::scb::RegisterBlock = SCB::PTR as *mut _;
        let icsr: u32 = (*scb).icsr.read();
        (*scb).icsr.write(icsr | (1 << 25)); // PENDSTCLR bit
    }
    
    unsafe {
        let scb: *mut peripheral::scb::RegisterBlock = SCB::PTR as *mut _;
        (*scb).shcsr.modify(|v| v & !(
            (1 << 18) | // USGFAULTENA
            (1 << 17) | // BUSFAULTENA 
            (1 << 16)   // MEMFAULTENA
        ));
        
        // Set vector table offset
        (*scb).vtor.write(LOADER_ADDR);
        
        // SP
        core::arch::asm!("MSR msp, {0}", in(reg) stack_ptr);
        
        let jump_fn: unsafe extern "C" fn() -> ! = core::mem::transmute(reset_ptr);
        jump_fn();
    }
    
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}