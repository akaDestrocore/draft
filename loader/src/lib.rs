#![no_std]

use core::panic::PanicInfo;
use cortex_m::asm;

#[no_mangle]
pub extern "C" fn _rust_start() {

}

#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}