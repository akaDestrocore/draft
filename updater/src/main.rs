#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f4 as pac;
use misc::image::{ImageHeader, SharedMemory, IMAGE_MAGIC_UPDATER, IMAGE_TYPE_UPDATER};

// symbol from linker
extern "C" {
    static __firmware_size: u32;
}

#[no_mangle]
#[link_section = ".image_hdr"]
pub static mut IMAGE_HEADER: ImageHeader = ImageHeader::new(
    IMAGE_TYPE_UPDATER,
    IMAGE_MAGIC_UPDATER,
    1, 0, 0
);

#[entry]
fn main() -> ! {
    // get firmware size from linker
    let firmware_size: u32 = unsafe {
        let size: *const u32 = &__firmware_size as *const u32;
        let size_value: u32 = *size;
        
        // update size in header
        IMAGE_HEADER.update_data_size(size_value);
        
        size_value
    };

    // PD13 init 
    let peripherals: stm32f4::Peripherals = unsafe { pac::Peripherals::steal() };
    
    // Enable GPIOD clock
    unsafe {
        peripherals.rcc.ahb1enr().modify(|_, w| w.gpioden().set_bit());

        peripherals.gpiod.moder().modify(|_, w| w.moder13().bits(0b01));
        peripherals.gpiod.otyper().modify(|_, w| w.ot13().clear_bit());
        peripherals.gpiod.ospeedr().modify(|_, w| w.ospeedr13().bits(0b00));
        
        // enable LED
        peripherals.gpiod.bsrr().write(|w| w.bs13().set_bit());
    }

    
    // just blink
    loop {
        toggle_led(&peripherals);
        delay_ms(500);
    }
}

fn toggle_led(peripherals: &pac::Peripherals) {
    unsafe {
        let current: bool = peripherals.gpiod.odr().read().odr13().bit();
        if current {
            peripherals.gpiod.bsrr().write(|w| w.br13().set_bit());
        } else {
            peripherals.gpiod.bsrr().write(|w| w.bs13().set_bit());
        }
    }
}

fn delay_ms(ms: u32) {
    let mut count: u32 = ms * 8000; 
    while count > 0 {
        count -= 1;
        asm::nop();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}