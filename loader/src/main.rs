#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f4 as pac;
use misc::{
    flash,
    image::{ImageHeader, SharedMemory, IMAGE_MAGIC_LOADER, IMAGE_TYPE_LOADER}
};

#[no_mangle]
#[link_section = ".image_hdr"]
pub static IMAGE_HEADER: ImageHeader = ImageHeader::new(
    IMAGE_TYPE_LOADER,
    IMAGE_MAGIC_LOADER,
    1, 0, 0  // ver 1.0.0
);

#[no_mangle]
#[link_section = ".shared_memory"]
pub static mut SHARED_MEMORY: SharedMemory = SharedMemory::new();

// Test address - beginning of sector 2 (0x08008000)
const TEST_ADDR: u32 = 0x08008000;
const TEST_PATTERN1: u32 = 0xABCDABCD;
const TEST_PATTERN2: u32 = 0x12341234;
const TEST_SIZE: usize = 1024; // 1KB of test data

#[entry]
fn main() -> ! {
    // Initialize peripherals
    let p = pac::Peripherals::take();
    
    // Setup system clock
    setup_system_clock(&p);
    
    // Configure LEDs for status indication
    setup_leds(&p);
    
    // Prepare test patterns
    let mut test_pattern1 = [0u8; TEST_SIZE];
    let mut test_pattern2 = [0u8; TEST_SIZE];
    
    // Fill test patterns with repeating values
    for i in (0..TEST_SIZE).step_by(4) {
        // Pattern 1: 0xABCDABCD
        test_pattern1[i] = (TEST_PATTERN1 & 0xFF) as u8;
        test_pattern1[i+1] = ((TEST_PATTERN1 >> 8) & 0xFF) as u8;
        test_pattern1[i+2] = ((TEST_PATTERN1 >> 16) & 0xFF) as u8;
        test_pattern1[i+3] = ((TEST_PATTERN1 >> 24) & 0xFF) as u8;
        
        // Pattern 2: 0x12341234
        test_pattern2[i] = (TEST_PATTERN2 & 0xFF) as u8;
        test_pattern2[i+1] = ((TEST_PATTERN2 >> 8) & 0xFF) as u8;
        test_pattern2[i+2] = ((TEST_PATTERN2 >> 16) & 0xFF) as u8;
        test_pattern2[i+3] = ((TEST_PATTERN2 >> 24) & 0xFF) as u8;
    }
    
    // Buffer for reading back data
    let mut read_buffer = [0u8; TEST_SIZE];
    
    // STEP 1: Erase the target sector
    blink_led(&p, 12); // Green LED
    
    let erase_result = flash::erase_sector(&p, TEST_ADDR);
    if erase_result == 0 {
        error_blink(&p, 14); // Red LED
        loop {}
    }
    
    // STEP 2: Write first test pattern
    blink_led(&p, 13); // Orange LED
    
    let write_result = flash::write(&p, &test_pattern1, TEST_ADDR);
    if write_result != 0 {
        error_blink(&p, 14); // Red LED
        loop {}
    }
    
    // STEP 3: Read back and verify first pattern
    blink_led(&p, 12); // Green LED
    
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for i in 0..TEST_SIZE {
        if read_buffer[i] != test_pattern1[i] {
            error_blink(&p, 14); // Red LED
            loop {}
        }
    }
    
    // STEP 4: Erase the sector again
    blink_led(&p, 13); // Orange LED
    
    let erase_result = flash::erase_sector(&p, TEST_ADDR);
    if erase_result == 0 {
        error_blink(&p, 14); // Red LED
        loop {}
    }
    
    // STEP 5: Verify the sector is erased (all 0xFF)
    blink_led(&p, 12); // Green LED
    
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for i in 0..TEST_SIZE {
        if read_buffer[i] != 0xFF {
            error_blink(&p, 14); // Red LED
            loop {}
        }
    }
    
    // STEP 6: Write second test pattern
    blink_led(&p, 13); // Orange LED
    
    let write_result = flash::write(&p, &test_pattern2, TEST_ADDR);
    if write_result != 0 {
        error_blink(&p, 14); // Red LED
        loop {}
    }
    
    // STEP 7: Read back and verify second pattern
    blink_led(&p, 12); // Green LED
    
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for i in 0..TEST_SIZE {
        if read_buffer[i] != test_pattern2[i] {
            error_blink(&p, 14); // Red LED
            loop {}
        }
    }
    
    // All tests passed! Celebrate with LED pattern
    success_pattern(&p);
    
    loop {
        asm::nop();
    }
}

fn setup_system_clock(p: &pac::Peripherals) {
    // Enable PWR clock
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());

    // Set voltage scaling to Scale 1
    p.pwr.cr().modify(|_, w| w.vos().scale1());

    // Configure flash access control
    p.flash.acr().modify(|_, w| w
        .latency().ws5()
        .prften().enabled()
        .icen().enabled()
        .dcen().enabled()
    );

    // Enable HSE
    p.rcc.cr().modify(|_, w| w.hseon().set_bit());
    while p.rcc.cr().read().hserdy().bit_is_clear() {
        // Wait for HSE ready
    }

    // Configure PLL
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
        // Wait for PLL ready
    }

    // Set bus dividers
    p.rcc.cfgr().modify(|_, w| w
        .hpre().div1()
        .ppre1().div4()
        .ppre2().div2()
    );

    // Select PLL as system clock
    p.rcc.cfgr().modify(|_, w| w.sw().pll());
    while !p.rcc.cfgr().read().sws().is_pll() {
        // Wait for system clock switch
    }
}

fn setup_leds(p: &pac::Peripherals) {
    // Enable GPIOD clock
    p.rcc.ahb1enr().modify(|_, w| w.gpioden().set_bit());

    // Configure LEDs (PD12-Green, PD13-Orange, PD14-Red, PD15-Blue)
    p.gpiod.moder().modify(|_, w| w
        .moder12().output()
        .moder13().output()
        .moder14().output()
        .moder15().output()
    );
    
    p.gpiod.otyper().modify(|_, w| w
        .ot12().push_pull()
        .ot13().push_pull()
        .ot14().push_pull()
        .ot15().push_pull()
    );
    
    p.gpiod.ospeedr().modify(|_, w| w
        .ospeedr12().low_speed()
        .ospeedr13().low_speed()
        .ospeedr14().low_speed()
        .ospeedr15().low_speed()
    );
}

fn blink_led(p: &pac::Peripherals, pin: u8) {
    // Turn on specified LED
    p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << pin) });
    delay(200000);
    
    // Turn off specified LED
    p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << (pin + 16)) });
    delay(200000);
}

fn error_blink(p: &pac::Peripherals, pin: u8) {
    // Rapidly blink error LED 10 times
    for _ in 0..10 {
        p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << pin) });
        delay(50000);
        p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << (pin + 16)) });
        delay(50000);
    }
}

fn success_pattern(p: &pac::Peripherals) {
    // Cycle through all LEDs in sequence
    for _ in 0..3 {
        for pin in 12..=15 {
            p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << pin) });
            delay(100000);
            p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << (pin + 16)) });
        }
    }
    
    // Then turn all on simultaneously
    p.gpiod.bsrr().write(|w| unsafe { w.bits(0xF000) });
    delay(500000);
    p.gpiod.bsrr().write(|w| unsafe { w.bits(0xF0000) });
}

fn delay(count: u32) {
    for _ in 0..count {
        asm::nop();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}