#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};

// global ms counter
pub static TICK_MS: AtomicU32 = AtomicU32::new(0);

// this must be called from SysTick handler
pub fn increment_tick() {
    let current = TICK_MS.load(Ordering::Relaxed);
    TICK_MS.store(current + 1, Ordering::Relaxed);
}

// returns current ms count
pub fn get_tick_ms() -> u32 {
    TICK_MS.load(Ordering::Relaxed)
}

// non-blocking API that returns true when time passed
pub fn wait_ms(start_ms: u32, delay_ms: u32) -> bool {
    let current_ms = get_tick_ms();
    current_ms.wrapping_sub(start_ms) >= delay_ms
}

pub fn setup_systick(syst: &mut cortex_m::peripheral::SYST) {
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload(90_000 - 1); // 1ms period for 90 MHz
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();
}