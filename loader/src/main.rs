#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::{asm, peripheral::{self, SCB, SYST}, register::{msp, psp}};
use cortex_m_rt::entry;
use stm32f4_pac as pac;

use misc::RingBuffer;

// Constants for memory addresses
const SLOT_2_APP_ADDR: u32 = 0x080C0000; // Adjust based on your memory layout
const UPDATER_ADDR: u32 = 0x08008000;
const SLOT_2_VER_ADDR: u32 = 0x080C1000; // Address for version info

// Static buffers
static RX_BUFFER: RingBuffer = RingBuffer::new();
static TX_BUFFER: RingBuffer = RingBuffer::new();
static mut RX_BYTE: u8 = 0;
static mut LOAD_APPLICATION: bool = false;
const BOOT_TIMEOUT: u32 = 10000; // 10 seconds timeout
static mut START_TICK: u32 = 0;
static mut LOADING_OPTION_SELECTED: bool = false;

// Message types for display
#[derive(Clone, Copy)]
enum Message {
    PrintHello,
    PrintOptions,
    PrintSelectionErr,
    PrintBootFirmware,
    PrintBootUpdater,
}

// Function to jump to user application
fn jump_to_user_application() -> ! {
    let p: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    
    unsafe {
        let reset_addr: u32 = SLOT_2_APP_ADDR + 4;
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

        p.rcc.cfgr().modify(|_, w| w.sw().hsi());
        
        while !p.rcc.cfgr().read().sws().is_hsi() {
            // wait
        }
        
        p.rcc.cfgr().reset();

        // remap
        p.syscfg.memrm().modify(|_, w| w.mem_mode().bits(0x1));
        
        // Get SysTick from cortex_m directly
        let mut cp: cortex_m::Peripherals = cortex_m::Peripherals::steal();
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
        (*scb).vtor.write(SLOT_2_APP_ADDR);

        // SP
        let stack_ptr: u32 = *(SLOT_2_APP_ADDR as *const u32);

        // Set MSP and PSP
        msp::write(stack_ptr);
        psp::write(stack_ptr);

        // do the jump
        let jump_fn: unsafe extern "C" fn() -> ! = core::mem::transmute(reset_ptr);
        jump_fn();
    }

    loop {
        asm::nop();
    }
}

// Function to jump to updater
fn jump_to_updater() -> ! {
    let p: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    
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

        p.rcc.cfgr().modify(|_, w| w.sw().hsi());
        
        while !p.rcc.cfgr().read().sws().is_hsi() {
            // wait
        }
        
        p.rcc.cfgr().reset();

        // remap
        p.syscfg.memrm().modify(|_, w| w.mem_mode().bits(0x1));
        
        // Get SysTick from cortex_m directly
        let mut cp: cortex_m::Peripherals = cortex_m::Peripherals::steal();
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

    loop {
        asm::nop();
    }
}

// UART initialization
fn uart_init(p: &pac::Peripherals) {
    // Enable UART2 clock
    p.rcc.apb1enr().modify(|_, w| w.usart2en().set_bit());

    // Enable GPIOA clock
    p.rcc.ahb1enr().modify(|_, w| w.gpioaen().set_bit());

    // Configure PA2 and PA3 as alternate function for UART
    p.gpioa.moder().modify(|_, w| {
        w.moder2().alternate() // TX
         .moder3().alternate() // RX
    });

    // Set alternate function AF7 (USART2)
    p.gpioa.afrl().modify(|_, w| {
        w.afrl2().af7()
         .afrl3().af7()
    });

    // Configure UART: 8N1, 115200 baud
    // BRR calculation: fCK / (16 * baud) = 45,000,000 / (16 * 115,200) = 24.41
    // Mantissa = 24, Fraction = 0.41 * 16 = 6.56 ≈ 7
    p.usart2.brr().write(|w| unsafe { w.bits(0x187) }); // 24.7 = 0x187

    // Enable UART, transmitter and receiver, and RX interrupt
    p.usart2.cr1().modify(|_, w| {
        w.ue().set_bit()
         .te().set_bit()
         .re().set_bit()
         .rxneie().set_bit()
    });

    // Enable error interrupt
    p.usart2.cr3().modify(|_, w| w.eie().set_bit());

    // Enable USART2 interrupt in NVIC
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
    }
}

// GPIO initialization
fn gpio_init(p: &pac::Peripherals) {
    // Enable GPIO clocks
    p.rcc.ahb1enr().modify(|_, w| {
        w.gpioaen().set_bit() // GPIOA (user button)
         .gpioden().set_bit() // GPIOD (LEDs)
    });

    // Configure LED pins PD12-15
    p.gpiod.moder().modify(|_, w| {
        w.moder12().output() // Green
         .moder13().output() // Orange
         .moder14().output() // Red
         .moder15().output() // Blue
    });

    p.gpiod.otyper().modify(|_, w| {
        w.ot12().push_pull()
         .ot13().push_pull()
         .ot14().push_pull()
         .ot15().push_pull()
    });

    p.gpiod.ospeedr().modify(|_, w| {
        w.ospeedr12().very_high_speed()
         .ospeedr13().very_high_speed()
         .ospeedr14().very_high_speed()
         .ospeedr15().very_high_speed()
    });

    p.gpiod.pupdr().modify(|_, w| {
        w.pupdr12().pull_down()
         .pupdr13().pull_down()
         .pupdr14().pull_down()
         .pupdr15().pull_down()
    });

    // Configure user button (PA0)
    p.gpioa.moder().modify(|_, w| w.moder0().input());
    p.gpioa.pupdr().modify(|_, w| w.pupdr0().pull_down());

    // Initialize LEDs to off state
    p.gpiod.bsrr().write(|w| {
        w.br12().set_bit()
         .br13().set_bit()
         .br14().set_bit()
         .br15().set_bit()
    });
}

// System clock configuration
fn sysclock_config(p: &pac::Peripherals) {
    // Enable power interface clock
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());
    
    // Set voltage regulator scaling
    unsafe {
        p.pwr.cr().modify(|r, w| {
            let bits: u32 = r.bits() | (1 << 14);
            w.bits(bits)
        });

        // Enable HSE
        p.rcc.cr().modify(|_, w| w.hseon().set_bit());
        while p.rcc.cr().read().hserdy().bit_is_clear() {
            // wait
        }
        
        // Configure PLL
        p.rcc.pllcfgr().modify(|_, w| {
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

        // Configure bus dividers
        p.rcc.cfgr().modify(|_, w| {
            w.hpre().div1()
            .ppre1().div4()
            .ppre2().div2()
            .sw().pll()
        });

        // Wait for PLL to be selected as system clock
        while !p.rcc.cfgr().read().sws().is_pll() {
            // wait
        }
    }
}

// Send a string over UART
fn send_string(p: &pac::Peripherals, s: &str) {
    for c in s.bytes() {
        // Wait for TXE flag
        while p.usart2.sr().read().txe().bit_is_clear() {
            asm::nop();
        }
        
        // Send byte
        p.usart2.dr().write(|w| unsafe { w.bits(c as u32) });
    }
}

// Show message on UART
fn show_message(p: &pac::Peripherals, msg: Message) {
    match msg {
        Message::PrintHello => {
            send_string(p, " @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@@@@@@@@@@@@@@=====@@@@@@@@@@@@@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@@@@@@@@@===============@@@@@@@@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@@@@@@====================@@@@@@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@@@@========================@@@@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@@@===========================@@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@@=============================@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@===============================@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@================================@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@====@=@=====@@==@=@@====@=@@===@@@@@@@@@@\r\n");
            send_string(p, " @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
            send_string(p, " ----------------------------------------------------\r\n");
            send_string(p, " @@@@@/@@@@@@@/@@@@@/@@@@@@|@@@@\\@@@@@@\\@@@@\\@@@@@@@@\r\n");
            send_string(p, " @@@/@@@@@@@@/@@@@@@/@@@@@@|@@@@@\\@@@@@@\\@@@@@\\@@@@@@\r\n");
            send_string(p, " @/@@@@@@@@/@@@@@@#/@@@@@@@|@@@@@@\\@@@@@@@\\@@@@@@\\@@@\r\n");
            send_string(p, " CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\r\n");
            send_string(p, " }             WELCOME TO BOOTLOADER                {\r\n");
            send_string(p, " }                                                  {\r\n");
            send_string(p, " }                 by destrocore                    {\r\n");
            send_string(p, " CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\r\n\r\n");
        }
        Message::PrintOptions => {
            send_string(p, "Booting into application in 10 seconds.\r\n\r\n");
            send_string(p, "Select an option:\r\n\r\n");
            send_string(p, " > 'SHIFT+U' --> Go to UPDATER\r\n\r\n");
            send_string(p, " > 'ENTER'   --> Regular boot\r\n");
        }
        Message::PrintSelectionErr => {
            send_string(p, "Please select from available options.\r\n");
        }
        Message::PrintBootFirmware => {
            send_string(p, "\r\n Booting into user application...\r\n");
        }
        Message::PrintBootUpdater => {
            send_string(p, "\r\n Loading updater...\r\n");
        }
    }
}

// Handle key input
fn read_key(p: &pac::Peripherals) {
    unsafe {
        if RX_BYTE == 0 {
            // Check if we've received data
            if p.usart2.sr().read().rxne().bit_is_set() {
                RX_BYTE = p.usart2.dr().read().bits() as u8;
                
                match RX_BYTE {
                    b'U' => {
                        show_message(p, Message::PrintBootUpdater);
                        jump_to_updater();
                    },
                    b'\r' => {
                        // Check if user app is valid
                        let app_valid = *(SLOT_2_VER_ADDR as *const u32) != 0xFFFFFFFF;
                        
                        if !app_valid {
                            send_string(p, "There is no user application!\r\n");
                            RX_BYTE = 0;
                        } else {
                            show_message(p, Message::PrintBootFirmware);
                            LOAD_APPLICATION = true;
                        }
                    },
                    _ => {
                        if RX_BYTE != 0 {
                            show_message(p, Message::PrintSelectionErr);
                            RX_BYTE = 0;
                        }
                    }
                }
            }
        }
        
        // Check if timeout or load flag set
        if LOAD_APPLICATION || (get_tick() - START_TICK >= BOOT_TIMEOUT) {
            // First send notification
            send_string(p, "\r\nLoading user application now.\r\n\r\n");
            jump_to_user_application();
        }
    }
}

// Get current system tick
fn get_tick() -> u32 {
    // In a real implementation, you would use the SysTick counter
    // For this example, we'll just return the current value + 1
    static mut TICKS: u32 = 0;
    
    unsafe {
        TICKS += 1;
        TICKS
    }
}

#[entry]
fn main() -> ! {
    // Get peripherals
    let p = unsafe { pac::Peripherals::steal() };
    
    // Initialize system
    sysclock_config(&p);
    gpio_init(&p);
    uart_init(&p);
    
    // Show welcome messages
    show_message(&p, Message::PrintHello);
    show_message(&p, Message::PrintOptions);
    
    // Set start tick
    unsafe { START_TICK = get_tick(); }
    
    // Main loop
    loop {
        read_key(&p);
    }
}

#[cortex_m_rt::exception]
fn USART2() {
    let p = unsafe { pac::Peripherals::steal() };
    
    // Handle USART2 interrupt - simplified implementation
    if p.usart2.sr().read().rxne().bit_is_set() {
        // Data received - will be handled in the main loop
    }
    
    // Error handling
    if p.usart2.sr().read().ore().bit_is_set() ||
       p.usart2.sr().read().fe().bit_is_set() ||
       p.usart2.sr().read().pe().bit_is_set() {
        // Clear error flags by reading SR then DR
        let _ = p.usart2.sr().read().bits();
        let _ = p.usart2.dr().read().bits();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}