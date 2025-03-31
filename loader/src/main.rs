#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    panic::PanicInfo,
    sync::atomic::{AtomicBool, Ordering},
    marker::PhantomData
};
use cortex_m::{
    asm,
    peripheral::{self, SCB, SYST}
};
use cortex_m_rt::{entry, exception};
use cortex_m_rt::interrupt;
use stm32f4::{self as pac, Peripherals, Interrupt};
use misc::RingBuffer;

// Constants for memory addresses
const SLOT_2_APP_ADDR: u32 = 0x08020200;
const UPDATER_ADDR: u32 = 0x08008000;
const SLOT_2_VER_ADDR: u32 = 0x08020000;
const BOOT_TIMEOUT_MS: u32 = 10_000; // 10 seconds

// Thread-safe peripheral pointer wrapper
struct PeripheralPtr<T>(*const T);
unsafe impl<T> Send for PeripheralPtr<T> {}
unsafe impl<T> Sync for PeripheralPtr<T> {}

// Global state with proper synchronization
static TX_BUFFER: Mutex<RefCell<RingBuffer>> = Mutex::new(RefCell::new(RingBuffer::new()));
static RX_BUFFER: Mutex<RefCell<RingBuffer>> = Mutex::new(RefCell::new(RingBuffer::new()));
static TX_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static LOAD_APPLICATION: AtomicBool = AtomicBool::new(false);
static START_TIME: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

// Global peripheral pointers using our safe wrapper
static USART2_PTR: Mutex<RefCell<Option<PeripheralPtr<pac::usart1::RegisterBlock>>>> = 
    Mutex::new(RefCell::new(None));
static GPIOD_PTR: Mutex<RefCell<Option<PeripheralPtr<pac::gpioi::RegisterBlock>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Take ownership of peripherals
    let p = match pac::Peripherals::take() {
        Some(p) => p,
        None => {
            // This shouldn't happen as we're the first to run, but handle gracefully
            loop { asm::nop(); }
        }
    };
    
    let mut cp = match cortex_m::Peripherals::take() {
        Some(cp) => cp,
        None => {
            // This shouldn't happen as we're the first to run, but handle gracefully
            loop { asm::nop(); }
        }
    };
    
    // Initialize clocks before anything else
    setup_system_clock(&p);
    
    // Save start time for timeout calculation
    let sys_tick = cortex_m::peripheral::SYST::get_current();
    interrupt::free(|cs| {
        START_TIME.borrow(cs).replace(sys_tick);
    });
    
    // Setup SysTick for millisecond timing
    setup_systick(&mut cp.SYST);
    
    // Initialize GPIO 
    setup_gpio(&p);
    
    // Initialize USART with interrupts disabled
    setup_usart(&p);
    
    // Turn on an LED to indicate we're in the loader
    p.gpiod.bsrr().write(|w| w.bs12().set_bit());
    
    // Store peripheral pointers for interrupt handler
    interrupt::free(|cs| {
        // Store pointer to USART2
        let usart2_ptr = unsafe { &*(p.usart2.sr().as_ptr() as *const _ as *const pac::usart1::RegisterBlock) };
        USART2_PTR.borrow(cs).replace(Some(PeripheralPtr(usart2_ptr)));
        
        // Store pointer to GPIOD
        let gpiod_ptr = unsafe { &*(p.gpiod.bsrr().as_ptr() as *const _ as *const pac::gpioi::RegisterBlock) };
        GPIOD_PTR.borrow(cs).replace(Some(PeripheralPtr(gpiod_ptr)));
    });
    
    // Send welcome message using polling method first
    send_welcome_message_polling(&p);
    
    // Now that everything is set up, enable USART2 interrupt
    unsafe { 
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
        
        // Enable USART2 interrupts
        p.usart2.cr1().modify(|_, w| w
            .rxneie().enabled()  // RX interrupt
        );
    }
    
    // Main loop
    loop {
        // Process any received bytes
        process_input();
        
        // Check timeout
        if should_boot_application() {
            boot_application(&p, &mut cp);
        }
        
        // Ensure transmit buffer is being processed
        ensure_transmitting();
        
        // Sleep a bit to save power
        asm::wfi();
    }
}

fn setup_system_clock(p: &Peripherals) {
    // Enable PWR clock
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());
    
    // Scale 1 voltage mode
    p.pwr.cr().modify(|_, w| w.vos().set_bit());
    
    // Configure flash latency - required for higher clock speeds
    p.flash.acr().modify(|_, w| unsafe {
        w.latency().bits(5)
         .prften().set_bit()
         .icen().set_bit()
         .dcen().set_bit()
    });
    
    // Enable HSE (external oscillator)
    p.rcc.cr().modify(|_, w| w.hseon().set_bit());
    while p.rcc.cr().read().hserdy().bit_is_clear() {
        asm::nop();
    }
    
    // Configure PLL
    p.rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllsrc().set_bit() // HSE as source
         .pllm().bits(4)     // Divide by 4
         .plln().bits(90)    // Multiply by 90
         .pllp().div2()      // Divide by 2
         .pllq().bits(4)     // Divide by 4 for USB
    });
    
    // Enable PLL
    p.rcc.cr().modify(|_, w| w.pllon().set_bit());
    while p.rcc.cr().read().pllrdy().bit_is_clear() {
        asm::nop();
    }
    
    // Set bus dividers
    p.rcc.cfgr().modify(|_, w| {
        w.hpre().div1()    // AHB not divided
         .ppre1().div4()   // APB1 divided by 4
         .ppre2().div2()   // APB2 divided by 2
    });
    
    // Switch to PLL as clock source
    p.rcc.cfgr().modify(|_, w| w.sw().pll());
    while !p.rcc.cfgr().read().sws().is_pll() {
        asm::nop();
    }
}

fn setup_systick(syst: &mut SYST) {
    // Configure SysTick to generate an interrupt every 1ms
    // Clock is 180 MHz, so we need 180,000 cycles per ms
    syst.set_reload(180_000 - 1);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();
}

fn setup_gpio(p: &Peripherals) {
    // Enable GPIO clocks
    p.rcc.ahb1enr().modify(|_, w| {
        w.gpioaen().set_bit()  // USART2 pins are on GPIOA
         .gpioden().set_bit()  // LED pins are on GPIOD
    });
    
    // Configure USART2 pins (PA2=TX, PA3=RX)
    p.gpioa.moder().modify(|_, w| {
        w.moder2().alternate()  // PA2 as alternate function
         .moder3().alternate()  // PA3 as alternate function
    });
    
    p.gpioa.ospeedr().modify(|_, w| {
        w.ospeedr2().high_speed()
         .ospeedr3().high_speed()
    });
    
    p.gpioa.afrl().modify(|_, w| {
        w.afrl2().af7()  // AF7 = USART2_TX
         .afrl3().af7()  // AF7 = USART2_RX
    });
    
    // Configure LED pins (PD12-PD15)
    p.gpiod.moder().modify(|_, w| {
        w.moder12().output()
         .moder13().output()
         .moder14().output()
         .moder15().output()
    });
    
    p.gpiod.otyper().modify(|_, w| {
        w.ot12().push_pull()
         .ot13().push_pull()
         .ot14().push_pull()
         .ot15().push_pull()
    });
    
    p.gpiod.ospeedr().modify(|_, w| {
        w.ospeedr12().low_speed()
         .ospeedr13().low_speed()
         .ospeedr14().low_speed()
         .ospeedr15().low_speed()
    });
}

fn setup_usart(p: &Peripherals) {
    // Enable USART2 clock
    p.rcc.apb1enr().modify(|_, w| w.usart2en().set_bit());
    
    // Configure USART2 (115200 baud, 8N1)
    // Assuming 90 MHz APB1 clock / 4 = 22.5 MHz
    // 22,500,000 / 115,200 = 195.3125
    // Mantissa = 195 = 0xC3, Fraction = 0.3125*16 = 5 = 0x5
    p.usart2.brr().write(|w| unsafe { 
        w.div_mantissa().bits(195)
         .div_fraction().bits(5)
    });
    
    // Enable USART, TX, and RX (but not interrupts yet)
    p.usart2.cr1().write(|w| {
        w.ue().enabled()
         .te().enabled()
         .re().enabled()
    });
}

fn send_welcome_message_polling(p: &Peripherals) {
    let message = "\r\n==== STM32F4 Bootloader ====\r\n\
                   Press 'U' to enter updater\r\n\
                   Press 'Enter' to boot application\r\n\
                   Will boot automatically in 10 seconds...\r\n";
    
    for byte in message.bytes() {
        // Wait for transmit register to be empty
        while p.usart2.sr().read().txe().bit_is_clear() {
            asm::nop();
        }
        
        // Send the byte
        p.usart2.dr().write(|w| unsafe { w.bits(byte as u16) });
    }
    
    // Wait for transmission to complete
    while p.usart2.sr().read().tc().bit_is_clear() {
        asm::nop();
    }
}

fn process_input() {
    interrupt::free(|cs| {
        if let Some(byte) = RX_BUFFER.borrow(cs).borrow_mut().read() {
            match byte {
                b'U' | b'u' => {
                    // Queue message about booting to updater
                    queue_string("\r\nBooting to updater...\r\n");
                    
                    // Boot to updater after message is sent
                    LOAD_APPLICATION.store(false, Ordering::SeqCst);
                    
                    // Ensure the message is transmitted before jumping
                    while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                        ensure_transmitting();
                    }
                    
                    // Now boot to updater
                    let p = pac::Peripherals::take().unwrap();
                    let mut cp = unsafe { cortex_m::Peripherals::steal() };
                    boot_updater(&p, &mut cp);
                },
                
                b'\r' => {
                    // Queue message about booting the application
                    queue_string("\r\nBooting application...\r\n");
                    LOAD_APPLICATION.store(true, Ordering::SeqCst);
                },
                
                _ => {
                    // Any other key, show the options
                    queue_string("\r\nPress 'U' for updater, 'Enter' for application\r\n");
                }
            }
        }
    });
}

fn queue_string(s: &str) {
    interrupt::free(|cs| {
        let mut tx_buffer = TX_BUFFER.borrow(cs).borrow_mut();
        for byte in s.bytes() {
            tx_buffer.write(byte);
        }
    });
    
    ensure_transmitting();
}

fn ensure_transmitting() {
    // If transmission is not in progress, start it
    if !TX_IN_PROGRESS.load(Ordering::SeqCst) {
        // Check if there's data to transmit
        let byte = interrupt::free(|cs| TX_BUFFER.borrow(cs).borrow_mut().read());
        
        if let Some(byte) = byte {
            interrupt::free(|cs| {
                if let Some(usart2_ptr) = USART2_PTR.borrow(cs).borrow().as_ref() {
                    unsafe {
                        // Get USART2 register block
                        let usart2 = &*(usart2_ptr.0 as *const pac::usart1::RegisterBlock);
                        
                        // Write to DR (will clear TXE)
                        usart2.dr().write(|w| w.bits(byte as u16));
                        
                        // Enable TXE interrupt
                        usart2.cr1().modify(|_, w| w.txeie().enabled());
                        
                        TX_IN_PROGRESS.store(true, Ordering::SeqCst);
                    }
                }
            });
        }
    }
}

fn should_boot_application() -> bool {
    // Check if user requested boot
    if LOAD_APPLICATION.load(Ordering::SeqCst) {
        return true;
    }
    
    // Check timeout
    interrupt::free(|cs| {
        let start = *START_TIME.borrow(cs).borrow();
        let now = cortex_m::peripheral::SYST::get_current();
        let elapsed = now.wrapping_sub(start);
        
        elapsed >= BOOT_TIMEOUT_MS
    })
}

fn boot_application(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) -> ! {
    // Check if application is valid
    let app_valid = unsafe { *(SLOT_2_VER_ADDR as *const u32) != 0xFFFFFFFF };
    
    if !app_valid {
        queue_string("\r\nNo valid application found!\r\n");
        loop {
            asm::nop();
        }
    }
    
    // Get reset and stack values from the application
    let reset_addr = SLOT_2_APP_ADDR + 4;
    let reset_vector = unsafe { *(reset_addr as *const u32) };
    let stack_addr = unsafe { *(SLOT_2_APP_ADDR as *const u32) };
    
    // Reset and disable peripherals
    p.rcc.cfgr().reset();
    p.rcc.cr().modify(|_, w| w
        .hsion().set_bit()
        .hseon().clear_bit()
        .pllon().clear_bit()
    );
    
    // Wait for HSI to be ready
    while p.rcc.cr().read().hsirdy().bit_is_clear() {
        asm::nop();
    }
    
    // Wait for switch to HSI
    while !p.rcc.cfgr().read().sws().is_hsi() {
        asm::nop();
    }
    
    // Disable all peripheral clocks
    p.rcc.ahb1enr().reset();
    p.rcc.apb1enr().reset();
    p.rcc.apb2enr().reset();
    
    // Enable SYSCFG for memory remap
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());
    
    // Memory remap
    p.syscfg.memrmp().write(|w| unsafe { w.bits(0x01) });
    
    // Disable all interrupts
    cortex_m::interrupt::disable();
    
    // Disable SysTick
    cp.SYST.disable_counter();
    cp.SYST.disable_interrupt();
    
    // Reset any pending interrupts
    unsafe {
        let scb = SCB::ptr();
        
        // Clear PendSV
        let icsr = (*scb).icsr.read();
        (*scb).icsr.write(icsr | (1 << 25));
        
        // Disable fault handlers
        (*scb).shcsr.modify(|v| v & !(
            (1 << 18) | // USGFAULTENA
            (1 << 17) | // BUSFAULTENA 
            (1 << 16)   // MEMFAULTENA
        ));
        
        // Set the vector table offset
        (*scb).vtor.write(SLOT_2_APP_ADDR);
        
        // Set the stack pointer
        core::arch::asm!("MSR msp, {0}", in(reg) stack_addr);
        
        // Jump to the application
        let jump_fn: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
        jump_fn();
    }
    
    // Should never get here
    loop {
        asm::nop();
    }
}

fn boot_updater(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) -> ! {
    // Get reset and stack values from the updater
    let reset_addr = UPDATER_ADDR + 4;
    let reset_vector = unsafe { *(reset_addr as *const u32) };
    let stack_addr = unsafe { *(UPDATER_ADDR as *const u32) };
    
    // Reset and disable peripherals
    p.rcc.cfgr().reset();
    p.rcc.cr().modify(|_, w| w
        .hsion().set_bit()
        .hseon().clear_bit()
        .pllon().clear_bit()
    );
    
    // Wait for HSI to be ready
    while p.rcc.cr().read().hsirdy().bit_is_clear() {
        asm::nop();
    }
    
    // Wait for switch to HSI
    while !p.rcc.cfgr().read().sws().is_hsi() {
        asm::nop();
    }
    
    // Disable all peripheral clocks
    p.rcc.ahb1enr().reset();
    p.rcc.apb1enr().reset();
    p.rcc.apb2enr().reset();
    
    // Enable SYSCFG for memory remap
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());
    
    // Memory remap
    p.syscfg.memrmp().write(|w| unsafe { w.bits(0x01) });
    
    // Disable all interrupts
    cortex_m::interrupt::disable();
    
    // Disable SysTick
    cp.SYST.disable_counter();
    cp.SYST.disable_interrupt();
    
    // Reset any pending interrupts
    unsafe {
        let scb = SCB::ptr();
        
        // Clear PendSV
        let icsr = (*scb).icsr.read();
        (*scb).icsr.write(icsr | (1 << 25));
        
        // Disable fault handlers
        (*scb).shcsr.modify(|v| v & !(
            (1 << 18) | // USGFAULTENA
            (1 << 17) | // BUSFAULTENA 
            (1 << 16)   // MEMFAULTENA
        ));
        
        // Set the vector table offset
        (*scb).vtor.write(UPDATER_ADDR);
        
        // Set the stack pointer
        core::arch::asm!("MSR msp, {0}", in(reg) stack_addr);
        
        // Jump to the updater
        let jump_fn: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
        jump_fn();
    }
    
    // Should never get here
    loop {
        asm::nop();
    }
}

// The proper way to define an interrupt handler with cortex-m-rt for USART2
#[interrupt]
fn USART2() {
    interrupt::free(|cs| {
        if let Some(usart2_ptr) = USART2_PTR.borrow(cs).borrow().as_ref() {
            unsafe {
                let usart2 = &*(usart2_ptr.0 as *const pac::usart1::RegisterBlock);
                
                // Check for receive
                if usart2.sr().read().rxne().bit_is_set() {
                    let data = usart2.dr().read().bits() as u8;
                    RX_BUFFER.borrow(cs).borrow_mut().write(data);
                }
                
                // Check for transmit
                if usart2.sr().read().txe().bit_is_set() && usart2.cr1().read().txeie().bit_is_set() {
                    // Mark TX as not in progress
                    TX_IN_PROGRESS.store(false, Ordering::SeqCst);
                    
                    // Check if there's more data to send
                    if let Some(byte) = TX_BUFFER.borrow(cs).borrow_mut().read() {
                        // Send next byte
                        usart2.dr().write(|w| unsafe { w.bits(byte as u16) });
                        TX_IN_PROGRESS.store(true, Ordering::SeqCst);
                    } else {
                        // No more data, disable TXE interrupt
                        usart2.cr1().modify(|_, w| w.txeie().disabled());
                    }
                }
            }
        }
    });
}

#[exception]
fn SysTick() {
    // This handler is used for timekeeping
    // Nothing to do here as we just use the counter value
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Flash LEDs in a panic pattern
    interrupt::free(|cs| {
        if let Some(gpiod_ptr) = GPIOD_PTR.borrow(cs).borrow().as_ref() {
            unsafe {
                let gpiod = &*(gpiod_ptr.0 as *const pac::gpioi::RegisterBlock);
                
                // Turn on all LEDs to indicate panic
                gpiod.bsrr().write(|w| w
                    .bs12().set_bit()
                    .bs13().set_bit()
                    .bs14().set_bit()
                    .bs15().set_bit()
                );
            }
        }
    });
    
    loop {
        // Toggle LED pattern
        delay(500_000);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        asm::nop();
    }
}