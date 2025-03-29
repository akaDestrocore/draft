#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::{asm, peripheral::{self, SCB, SYST}, register::{msp, psp}};
use cortex_m_rt::{entry, exception};
use stm32f4 as pac;
use stm32f4::Interrupt as interrupt;
use misc::RingBuffer;
use core::sync::atomic::{AtomicBool, Ordering};

// ring buffers
#[link_section = ".uninit.RX_BUFFER"]
static mut RX_BUFFER: RingBuffer = RingBuffer::new();

#[link_section = ".uninit.TX_BUFFER"]
static mut TX_BUFFER: RingBuffer = RingBuffer::new();

static mut USART2_PTR: Option<*mut stm32f4::usart1::RegisterBlock> = None;

// Constants for memory addresses
const SLOT_2_APP_ADDR: u32 = 0x08020200;
const UPDATER_ADDR: u32 = 0x08008000;
const SLOT_2_VER_ADDR: u32 = 0x08020000;


static TX_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static LOAD_APPLICATION: AtomicBool = AtomicBool::new(false);
static mut START_TICK: u32 = 0;
static BOOT_TIMEOUT: u32 = 10000; // 10 seconds

#[entry]
fn main() -> ! {
    let p: stm32f4::Peripherals = pac::Peripherals::take().unwrap();
    let mut cp: cortex_m::Peripherals = cortex_m::Peripherals::take().unwrap();
    
    // Initialize system clock
    system_clock_config(&p);
    
    // Initialize GPIO
    gpio_init(&p);
    
    // Initialize USART
    usart_init(&p);
    
    // Enable USART2 interrupt
    unsafe {
        USART2_PTR = Some(p.usart2.dr().as_ptr() as *mut _);
    }

    // enable USART irq
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
    }
    
    // Print welcome message
    show_message(Message::Hello);
    show_message(Message::Options);
    
    // Save start time
    unsafe {
        START_TICK = cortex_m::peripheral::SYST::get_current();
    }
    
    // Main loop - check for user input
    loop {
        read_key(&p, &mut cp);
        
        // Check for timeout
        unsafe {
            let current_tick: u32 = cortex_m::peripheral::SYST::get_current();
            if LOAD_APPLICATION.load(Ordering::SeqCst) || 
               (current_tick.wrapping_sub(START_TICK) >= BOOT_TIMEOUT) {
                jump_to_user_application(&p, &mut cp);
            }
        }
    }
}

fn system_clock_config(p: &pac::Peripherals) {
    // Enable PWR clock
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());
    
    // Scale 1
    p.pwr.cr().modify(|_, w| w.vos().set_bit());
    
    // Enable HSE
    p.rcc.cr().modify(|_, w | w.hseon().set_bit());
    while p.rcc.cr().read().hserdy().bit_is_clear() {
        // wait
    }
    
    // Configure PLL
    p.rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllsrc().set_bit() // 1 -> HSE
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
    
    // Configure flash latency
    p.flash.acr().modify(|_, w| unsafe {
        w.latency().ws5()
        .prften().set_bit()
        .icen().set_bit()
        .dcen().set_bit()
    });
    
    // Configure bus dividers
    p.rcc.cfgr().modify(|_, w| unsafe {
        w.hpre().div1()
        .ppre1().div4()
        .ppre2().div2()
    });
    
    // Set PLL as system clock
    p.rcc.cfgr().modify(|_, w| w.sw().pll());
    while !p.rcc.cfgr().read().sws().is_pll() {
        // wait
    }
}

fn gpio_init(p: &pac::Peripherals) {
    // Enable GPIO clocks
    p.rcc.ahb1enr().modify(|_, w| {
        w.gpioaen().set_bit()
        .gpioden().set_bit()
    });
    
    // Configure LED pins (PD12-15)
    p.gpiod.moder().modify(|_, w| unsafe {
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
    
    // Configure USART2 pins (PA2=TX, PA3=RX)
    p.gpioa.moder().modify(|_, w| unsafe {
        w.moder2().alternate()
        .moder3().alternate()
    });

    p.gpioa.ospeedr().modify(|_, w| unsafe {
        w.ospeedr2().very_high_speed()
        .ospeedr3().very_high_speed()
    });
    
    p.gpioa.afrl().modify(|_, w| unsafe {
        w.afrl2().af7()
        .afrl3().af7()
    });
}

fn usart_init(p: &pac::Peripherals) {
    // Enable USART2 clock
    p.rcc.apb1enr().modify(|_, w| w.usart2en().set_bit());

    // Configure USART2
    p.usart2.brr().write(|w| unsafe {
        w.div_fraction().bits(0x3)
        .div_mantissa().bits(0xc)
    });
    
    // Enable USART2, TX, RX, RXNE interrupt
    p.usart2.cr1().write(|w| unsafe {
        w.ue().enabled()
        .te().enabled()
        .re().enabled()
        .rxneie().enabled()
    });
    
    // Enable error interrupt
    p.usart2.cr3().modify(|_, w| w.eie().enabled());
}

// Messages to display
enum Message {
    Hello,
    Options,
    SelectionErr,
    BootUpdater,
    BootFirmware,
}

// Show messages
fn show_message(msg: Message) {
    match msg {
        Message::Hello => {
            send_string(" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@@@@@@@@@@@@@@=====@@@@@@@@@@@@@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@@@@@@@@@===============@@@@@@@@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@@@@@@====================@@@@@@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@@@@========================@@@@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@@@===========================@@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@@=============================@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@===============================@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@================================@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@====@=@=====@@==@=@@====@=@@===@@@@@@@@@@\r\n");
            send_string(" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
            send_string(" ----------------------------------------------------\r\n");
            send_string(" @@@@@/@@@@@@@/@@@@@/@@@@@@|@@@@\\@@@@@@\\@@@@\\@@@@@@@@\r\n");
            send_string(" @@@/@@@@@@@@/@@@@@@/@@@@@@|@@@@@\\@@@@@@\\@@@@@\\@@@@@@\r\n");
            send_string(" @/@@@@@@@@/@@@@@@#/@@@@@@@|@@@@@@\\@@@@@@@\\@@@@@@\\@@@\r\n");
            send_string(" CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\r\n");
            send_string(" }             WELCOME TO BOOTLOADER                {\r\n");
            send_string(" }                                                  {\r\n");
            send_string(" }                 by destrocore                    {\r\n");
            send_string(" CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\r\n\r\n");
        },
        Message::Options => {
            send_string("Booting into application in 10 seconds.\r\n\r\n");
            send_string("Select an option:\r\n\r\n");
            send_string(" > 'SHIFT+U' --> Go to UPDATER\r\n\r\n");
            send_string(" > 'ENTER'   --> Regular boot\r\n");
        },
        Message::SelectionErr => {
            send_string("Please select from available options.\r\n");
        },
        Message::BootFirmware => {
            send_string("\r\n Booting into user application...\r\n");
        },
        Message::BootUpdater => {
            send_string("\r\n Loading updater...\r\n");
        }
    }
}

// Send string via UART
fn send_string(s: &str) {
    for byte in s.bytes() {
        unsafe {
            TX_BUFFER.write(byte);
        }
    }
    transmit_buffer();
}

// Start buffer transmission
fn transmit_buffer() {
    cortex_m::interrupt::free(|_| {
        if !TX_IN_PROGRESS.load(Ordering::SeqCst) {
            unsafe {
                if let Some(byte) = TX_BUFFER.read() {
                    // Get USART2 register block
                    let p: stm32f4::Peripherals = pac::Peripherals::take().unwrap();
                    
                    // Write data to DR register
                    p.usart2.dr().write(|w| w.bits(byte as u16));
                    
                    // Enable TXE interrupt
                    p.usart2.cr1().modify(|_, w| w.txeie().set_bit());
                    
                    TX_IN_PROGRESS.store(true, Ordering::SeqCst);
                }
            }
        }
    });
}

// Read keypress from UART
fn read_key(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) {
    unsafe {
        let mut rx_byte: Option<u8> = None;
        
        // Try to read a byte from the RX buffer
        cortex_m::interrupt::free(|_| {
            rx_byte = RX_BUFFER.read();
        });
        
        if let Some(byte) = rx_byte {
            match byte {
                b'U' => {
                    show_message(Message::BootUpdater);
                    jump_to_updater(p, cp);
                },
                b'\r' => {
                    if *(SLOT_2_VER_ADDR as *const u32) == 0xFFFFFFFF {
                        send_string("There is no user application!\r\n");
                    } else {
                        show_message(Message::BootFirmware);
                        LOAD_APPLICATION.store(true, Ordering::SeqCst);
                    }
                },
                _ => {
                    show_message(Message::SelectionErr);
                }
            }
        }
    }
}

fn jump_to_user_application(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) -> ! {
    
    let reset_addr: u32 = SLOT_2_APP_ADDR + 4;
    let reset_ptr: u32 = unsafe { *(reset_addr as *const u32) };
    let stack_ptr: u32 = unsafe { *(SLOT_2_APP_ADDR as *const u32) };

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
        (*scb).vtor.write(SLOT_2_APP_ADDR);
        
        // SP
        core::arch::asm!("MSR msp, {0}", in(reg) stack_ptr);
        
        let jump_fn: unsafe extern "C" fn() -> ! = core::mem::transmute(reset_ptr);
        jump_fn();
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

// USART2 interrupt handler
#[cortex_m_rt::interrupt]
fn USART2() {
    unsafe {
        if let Some(usart2_ptr) = USART2_PTR {
            let usart2 = &*usart2_ptr;
            
            // Проверяем флаг приёма
            if (*usart2).sr().read().rxne().bit_is_set() {
                // Читаем данные
                let data = (*usart2).dr().read().bits() as u8;
                RX_BUFFER.write(data);
            }
            
            // Проверяем флаг передачи
            if (*usart2).sr().read().txe().bit_is_set() && (*usart2).cr1().read().txeie().bit_is_set() {
                TX_IN_PROGRESS.store(false, Ordering::SeqCst);
                
                if let Some(byte) = TX_BUFFER.read() {
                    // Отправляем следующий байт
                    (*usart2).dr().write(|w| w.bits(byte as u16));
                    TX_IN_PROGRESS.store(true, Ordering::SeqCst);
                } else {
                    // Данных больше нет, отключаем прерывание TXE
                    (*usart2).cr1().modify(|_, w| w.txeie().clear_bit());
                }
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}