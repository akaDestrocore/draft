#![no_std]
#![no_main]

use core::{f32::consts, panic::PanicInfo};
use cortex_m::{asm, peripheral::{self, scb, SCB}};
use cortex_m_rt::entry;
use stm32f4::{self as pac, Peripherals};

const LOADER_ADDR: u32 = 0x08004000;
const UPDATER_ADDR: u32 = 0x08008000;

#[entry]
fn main() -> ! {

    let peripherals = pac::Peripherals::take();

    if let Some(p) = peripherals {
        setup_system_clock(&p);

        let loader_is_valid: bool = check_loader_valid();
        
        prepare_for_jump(&p);

        if loader_is_valid {
            jump_to_image(LOADER_ADDR);
        } else {
            jump_to_image(UPDATER_ADDR);
        }
    }

    loop {
        asm::nop();
    }
}

fn setup_system_clock(p: &Peripherals) {
    let rcc = &p.rcc;

    // Enable HSI
    rcc.cr().modify(|_, w| w.hsion().set_bit());
    while rcc.cr().read().hsirdy().bit_is_clear() {
        // wait
    }

    // HSI as system clock
    rcc.cfgr().modify(|_, w| w.sw().hsi());
    while !rcc.cfgr().read().sws().is_hsi() {
        // wait
    }
}

fn check_loader_valid() -> bool {
    unsafe {
        let loader_value: u32 = *(LOADER_ADDR as *const u32);
        0xFFFFFFFF != loader_value
    }
}

fn prepare_for_jump(p: &Peripherals) {
    // Reset clock
    p.rcc.cr().modify(|_, w| w 
        .hsion().set_bit()
        .hseon().clear_bit()
        .pllon().clear_bit()
    );

    p.rcc.cfgr().reset();
    while !p.rcc.cfgr().read().sws().is_hsi() {
        // wait
    }

    // Disable all clocks
    p.rcc.ahb1enr().reset();
    p.rcc.ahb2enr().reset();
    p.rcc.ahb3enr().reset();
    p.rcc.apb1enr().reset();
    p.rcc.apb2enr().reset();

    // remap
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());
    p.syscfg.memrmp().write(|w| unsafe {
        w.bits(0x01)
    });

    unsafe {
        // TODO: remove after testing
        cortex_m::peripheral::NVIC::mask(pac::Interrupt::USART2);
    }
}

fn jump_to_image(addr: u32) -> ! {
    let reset_addr: u32 = addr + 4;
    let stack_addr: u32 = unsafe {
        *(addr as *const u32)
    };
    let reset_vector: u32 = unsafe {
        *(reset_addr as *const u32)
    };

    // TODO: might be removed in the future
    cortex_m::interrupt::disable();

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
        (*scb).vtor.write(addr);

        // set SP
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