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

    sysclock_config(&p);

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

fn sysclock_config(p: &pac::Peripherals) {
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
    
    unsafe {
        let reset_addr: u32 = UPDATER_ADDR + 4;
        let reset_ptr: u32 = *(reset_addr as *const u32);

        // Deinit RCC
        p.rcc.cr().modify(|_,w| w
            .hsion().set_bit()
            .hseon().clear_bit()
            .pllon().clear_bit()
        );
        while p.rcc.cr().read().hsirdy().bit_is_clear() {
            // wait
        }

        p.rcc.cfgr().modify(|_, w | w.sw().hsi());
        while !p.rcc.cfgr().read().sws().is_hsi() {
            // wait
        }
        p.rcc.cfgr().reset();

        // remap
        p.syscfg.memrmp().modify(|_, w| w.mem_mode().bits(0x1));
        
        // Get SysTick from cortex_m directly
        let systick: &mut SYST = &mut cp.SYST;
        systick.disable_counter();
        systick.disable_interrupt();

        // clear PendSV
        let scb: *const peripheral::scb::RegisterBlock = SCB::PTR;
        (*scb).icsr.write(0);

        // disable SCB error handlers
        (*scb).shcsr.modify(|v: u32| v & !(
            (1 << 18) | (1 << 17) | (1 << 16)
        ));
        
        // set vector table
        (*scb).vtor.write(UPDATER_ADDR);

        // SP
        let stack_ptr: u32 = *(UPDATER_ADDR as *const u32);

        // Set MSP and PSP
        msp::write(stack_ptr);
        psp::write(stack_ptr);

        // do the jump
        let jump_fn: unsafe extern "C" fn() -> ! = core::mem::transmute(reset_ptr);
        jump_fn();
    }

}

fn jump_to_loader(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) -> ! {

    unsafe {
        let reset_addr: u32 = LOADER_ADDR + 4;
        let reset_ptr: u32 = *(reset_addr as *const u32);

        // Deinit RCC
        p.rcc.cr().modify(|_,w| w
            .hsion().set_bit()
            .hseon().clear_bit()
            .pllon().clear_bit()
        );
        while p.rcc.cr().read().hsirdy().bit_is_clear() {
            // wait
        }

        p.rcc.cfgr().modify(|_, w | w.sw().hsi());
        while !p.rcc.cfgr().read().sws().is_hsi() {
            // wait
        }
        p.rcc.cfgr().reset();

        // remap
        p.syscfg.memrmp().modify(|_, w| w.mem_mode().bits(0x1));
        
        // Get SysTick from cortex_m directly
        let systick: &mut SYST = &mut cp.SYST;
        systick.disable_counter();
        systick.disable_interrupt();


        // clear PendSV
        let scb: *const peripheral::scb::RegisterBlock = SCB::PTR;
        (*scb).icsr.write(0);

        // disable SCB error handlers
        (*scb).shcsr.modify(|v: u32| v & !(
            (1 << 18) | (1 << 17) | (1 << 16)
        ));
        
        // set vector table
        (*scb).vtor.write(LOADER_ADDR);

        // SP
        let stack_ptr: u32 = *(LOADER_ADDR as *const u32);

        // Set MSP and PSP
        msp::write(stack_ptr);
        psp::write(stack_ptr);

        // do the jump
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