#![no_std]

use core::panic::PanicInfo;
use cortex_m::asm;

#[no_mangle]
pub extern "C" fn _rust_start() {
    // This function needed by some build systems
    // We're using the entry attribute from cortex_m_rt,
    // so this function is just a placeholder
}

// If used outside of the main crate, we need a panic handler
#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}