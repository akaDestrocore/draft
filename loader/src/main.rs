#![no_std]
#![no_main]

use core::{
    cell::UnsafeCell, 
    panic::PanicInfo, 
    sync::atomic::{AtomicBool, AtomicU8, Ordering}
};

use cortex_m::{
    asm,
    peripheral::{NVIC, SCB, SYST}
};

use cortex_m_rt::{entry, exception};
use stm32f4::{self as pac, Peripherals};
use misc::{
    ring_buffer::RingBuffer,
    image::{ImageHeader, SharedMemory, IMAGE_MAGIC_LOADER, IMAGE_TYPE_LOADER, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER},
    systick,
    flash
};

// Constants for XMODEM protocol
const SOH: u8 = 0x01;  // Start of header (128 bytes)
const STX: u8 = 0x02;  // Start of header (1K bytes)
const EOT: u8 = 0x04;  // End of transmission
const ACK: u8 = 0x06;  // Acknowledge
const NAK: u8 = 0x15;  // Negative acknowledge
const CAN: u8 = 0x18;  // Cancel
const C: u8 = 0x43;    // ASCII 'C' for CRC mode

// State machine states
#[derive(Clone, Copy, PartialEq)]
pub enum XmodemState {
    Idle,          // Not receiving data
    Init,          // Initializing data reception
    Receiving,     // Receiving data
    Complete,      // Transfer complete
    Error,         // Error occurred
}

// Error codes for LED diagnostics
const ERROR_NONE: u8 = 0;
const ERROR_CANCEL: u8 = 1;
const ERROR_HEADER_INVALID: u8 = 2;
const ERROR_FLASH_ERASE: u8 = 3;
const ERROR_FLASH_WRITE: u8 = 4;
const ERROR_SEQUENCE: u8 = 5;
const ERROR_CRC: u8 = 6;
const ERROR_OVERFLOW: u8 = 7;
const ERROR_TIMEOUT: u8 = 8;

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

pub struct Mutex<T> {
    inner: UnsafeCell<T>
}

unsafe impl<T> Sync for Mutex<T> {
    // access to data is protected by critical sections
}

impl<T> Mutex<T> {
    pub const fn new(value: T) -> Self {
        Self { inner: UnsafeCell::new(value)}
    }

    pub fn get<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        // always call inside the critical section
        cortex_m::interrupt::free(|_| {
            let ptr: *mut T = self.inner.get();
            f(unsafe {
                &mut *ptr
            })
        })
    }
}

const UPDATER_ADDR: u32 = 0x08008000;
const APP_ADDR: u32 = 0x08020000;
const IMAGE_HDR_SIZE: u32 = 0x200;
const BOOT_TIMEOUT_MS: u32 = 10_000; // 10 sec

// pointer wrappers
struct PeripheralPtr<T>(*const T);
unsafe impl<T> Send for PeripheralPtr<T> {}
unsafe impl<T> Sync for PeripheralPtr<T> {}

pub static TX_BUFFER: Mutex<RingBuffer> = Mutex::new(RingBuffer::new());
pub static RX_BUFFER: Mutex<RingBuffer> = Mutex::new(RingBuffer::new());
static TX_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static LOAD_APPLICATION: AtomicBool = AtomicBool::new(false);
static LOAD_UPDATER: AtomicBool = AtomicBool::new(false);
static START_TIME: Mutex<u32> = Mutex::new(0);

// Global state variables for XMODEM
static XMODEM_STATE: AtomicU8 = AtomicU8::new(0);
static SEQUENCE: AtomicU8 = AtomicU8::new(1);
static FIRST_PACKET: AtomicBool = AtomicBool::new(true);
static TARGET_APP: AtomicBool = AtomicBool::new(true);
static UPDATE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
static ERROR_CODE: AtomicU8 = AtomicU8::new(ERROR_NONE);

static mut CURRENT_ADDRESS: u32 = 0;
static mut LAST_C_TIME: u32 = 0;
static mut C_SENT_COUNT: u8 = 0;
static mut PACKET_COUNTER: u32 = 0;  // Counts successful packets

// Buffer for packet reception
static mut PARTIAL_PACKET: [u8; 1029] = [0; 1029]; // Increased to handle 1K packets
static mut BUFFER_INDEX: usize = 0;

// handle like logic - using global pointers for peripherals
static USART2_PTR: Mutex<Option<PeripheralPtr<pac::usart2::RegisterBlock>>> =
    Mutex::new(None);
static GPIOD_PTR: Mutex<Option<PeripheralPtr<pac::gpiod::RegisterBlock>>> =
    Mutex::new(None);

#[entry]
fn main() -> ! {
    let p: Peripherals = match pac::Peripherals::take() {
        Some(p) => p,
        None => {
            loop {
                asm::nop();
            }
        }
    };
    
    let mut cp: cortex_m::Peripherals = match cortex_m::Peripherals::take() {
        Some(cp) => cp,
        None => {
            loop {
                asm::nop();
            }
        }
    };

    // clock setup
    setup_system_clock(&p);

    // get current time
    let current_ms: u32 = systick::get_tick_ms();
    START_TIME.get(|time: &mut u32| *time = current_ms);

    systick::setup_systick(&mut cp.SYST);

    setup_gpio(&p);

    setup_usart(&p);

    let usart2_ptr: &stm32f4::usart2::RegisterBlock = unsafe {
        &*(p.usart2.sr().as_ptr() as *const _ as *const pac::usart2::RegisterBlock)
    };
    USART2_PTR.get(|ptr| *ptr = Some(PeripheralPtr(usart2_ptr)));

    let gpiod_ptr: &stm32f4::gpiod::RegisterBlock = unsafe { 
        &*(p.gpiod.bsrr().as_ptr() as *const _ as *const pac::gpiod::RegisterBlock)
    };
    GPIOD_PTR.get(|ptr| *ptr = Some(PeripheralPtr(gpiod_ptr)));

    send_welcome_message(&p);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);

        // enable USART2 interrupts
        p.usart2.cr1().modify(|_, w| w
            .rxneie().enabled()
            .txeie().enabled()
        );
    }

    // Firmware update menu state variables
    let mut update_option_selected = false;
    
    loop {
        // Process input and firmware updates if not already in progress
        if !is_update_in_progress() {
            if let Some(byte) = RX_BUFFER.get(|buf| buf.read()) {
                handle_user_input(&p, byte, &mut update_option_selected);
            }
        } else {
            // Process XMODEM protocol if update is in progress
            TX_BUFFER.get(|tx_buf| {
                RX_BUFFER.get(|rx_buf| {
                    let result = process_xmodem(&p, tx_buf, rx_buf);
                    
                    // Handle completion or error state
                    if result {
                        match get_state() {
                            XmodemState::Complete => {
                                // Get diagnostic information
                                let (_, _, packets) = get_diagnostic_info();
                                
                                // Wait for last transmissions to complete
                                while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                                    ensure_transmitting();
                                }
                                
                                // Add small delay to ensure everything is processed
                                let start_ms = systick::get_tick_ms();
                                while !systick::wait_ms(start_ms, 100) {
                                    asm::nop();
                                }
                                
                                // Report diagnostics
                                TX_BUFFER.get(|buf| {
                                    queue_string(buf, "\r\nTransfer complete: ");
                                    // Convert packets to decimal
                                    write_decimal(buf, packets);
                                    queue_string(buf, " packets received.\r\n");
                                    
                                    // Boot the newly updated firmware based on target
                                    if TARGET_APP.load(Ordering::Relaxed) {
                                        queue_string(buf, "Application update successful, booting...\r\n");
                                    } else {
                                        queue_string(buf, "Updater update successful, booting...\r\n");
                                    }
                                });
                                
                                while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                                    ensure_transmitting();
                                }
                                
                                // Boot the appropriate firmware
                                if TARGET_APP.load(Ordering::Relaxed) {
                                    boot_application(&p, &mut cp);
                                } else {
                                    boot_updater(&p, &mut cp);
                                }
                            },
                            XmodemState::Error => {
                                // Reset to menu
                                set_state(XmodemState::Idle);
                                update_option_selected = false;
                                
                                // Get diagnostic information
                                let (_, error_code, packets) = get_diagnostic_info();
                                
                                // Report error with diagnostics
                                TX_BUFFER.get(|buf| {
                                    queue_string(buf, "\r\nUpdate failed! Error code: ");
                                    write_decimal(buf, error_code as u32);
                                    queue_string(buf, ", Packets received: ");
                                    write_decimal(buf, packets);
                                    queue_string(buf, "\r\n");
                                    
                                    // Provide more detailed error information
                                    queue_string(buf, "Error description: ");
                                    match error_code {
                                        1 => queue_string(buf, "Operation cancelled by sender"),
                                        2 => queue_string(buf, "Firmware header invalid"),
                                        3 => queue_string(buf, "Flash erase failed"),
                                        4 => queue_string(buf, "Flash write failed"),
                                        5 => queue_string(buf, "Sequence number error"),
                                        6 => queue_string(buf, "CRC check failed"),
                                        7 => queue_string(buf, "Buffer overflow"),
                                        8 => queue_string(buf, "Timeout"),
                                        _ => queue_string(buf, "Unknown error"),
                                    }
                                    queue_string(buf, "\r\n");
                                    
                                    // Return to menu
                                    show_menu(buf);
                                });
                            },
                            _ => {}
                        }
                    }
                });
            });
        }
        
        // check if boot actions are requested
        if LOAD_APPLICATION.load(Ordering::SeqCst) {
            boot_application(&p, &mut cp);
        }
        
        if LOAD_UPDATER.load(Ordering::SeqCst) {
            boot_updater(&p, &mut cp);
        }

        // check timeout - only if no firmware update is in progress
        if !is_update_in_progress() && !update_option_selected {
            let current_ms: u32 = systick::get_tick_ms();
            let start_ms: u32 = START_TIME.get(|time: &mut u32| *time);
            if (current_ms - start_ms) >= BOOT_TIMEOUT_MS {
                TX_BUFFER.get(|buf| {
                    queue_string(buf, "\r\nTimeout reached. Booting application...\r\n");
                });
                
                // wait to finish
                while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                    ensure_transmitting();
                }
                
                boot_application(&p, &mut cp);
            }
        }

        // Make sure UART transmission is continually processed
        ensure_transmitting();

        // Power-saving wait for interrupt
        asm::wfi();
    }
}

// Helper function to write decimal numbers to the output buffer
fn write_decimal(tx_buffer: &RingBuffer, value: u32) {
    if value == 0 {
        tx_buffer.write(b'0');
        return;
    }
    
    let mut num = value;
    let mut digits = [0u8; 10]; // Max 10 digits for u32
    let mut i = 0;
    
    while num > 0 && i < 10 {
        digits[i] = (num % 10) as u8;
        num /= 10;
        i += 1;
    }
    
    while i > 0 {
        i -= 1;
        tx_buffer.write(b'0' + digits[i]);
    }
}

// Helper function to write hexadecimal numbers
fn write_hex(tx_buffer: &RingBuffer, value: u32) {
    static HEX_CHARS: [u8; 16] = *b"0123456789ABCDEF";
    
    tx_buffer.write(HEX_CHARS[((value >> 28) & 0xF) as usize]);
    tx_buffer.write(HEX_CHARS[((value >> 24) & 0xF) as usize]);
    tx_buffer.write(HEX_CHARS[((value >> 20) & 0xF) as usize]);
    tx_buffer.write(HEX_CHARS[((value >> 16) & 0xF) as usize]);
    tx_buffer.write(HEX_CHARS[((value >> 12) & 0xF) as usize]);
    tx_buffer.write(HEX_CHARS[((value >> 8) & 0xF) as usize]);
    tx_buffer.write(HEX_CHARS[((value >> 4) & 0xF) as usize]);
    tx_buffer.write(HEX_CHARS[(value & 0xF) as usize]);
}

fn setup_system_clock(p: &Peripherals) {
    // PWR clock
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());

    // Scale 1
    p.pwr.cr().modify(|_, w| w.vos().scale1());

    // flash latency
    p.flash.acr().modify(|_, w| w
        .latency().ws5()
        .prften().set_bit()
        .icen().set_bit()
        .dcen().set_bit()
    );

    // Enable HSE
    p.rcc.cr().modify(|_, w| w.hseon().set_bit());
    while p.rcc.cr().read().hserdy().bit_is_clear() {
        // wait
    }

    // PLL configuration
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
        // wait
    }

    // bus dividers
    p.rcc.cfgr().modify(|_, w| {
        w.hpre().div1()
        .ppre1().div4()
        .ppre2().div2()
    });

    // PLL as sys clock
    p.rcc.cfgr().modify(|_, w| w.sw().pll());
    while !p.rcc.cfgr().read().sws().is_pll() {
        // wait
    }
}

fn setup_gpio(p: &Peripherals) {
    p.rcc.ahb1enr().modify(|_, w| {
        w.gpioaen().enabled()
        .gpioden().enabled()
    });

    p.gpioa.moder().modify(|_, w| {
        w.moder2().alternate()
        .moder3().alternate()
    });

    p.gpioa.ospeedr().modify(|_, w| {
        w.ospeedr2().high_speed()
         .ospeedr3().high_speed()
    });
    
    p.gpioa.afrl().modify(|_, w| {
        w.afrl2().af7()
         .afrl3().af7()
    });

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

    p.usart2.brr().write(|w| unsafe {
        w.div_mantissa().bits(0xc)
        .div_fraction().bits(0x3)
    });

    // enable USART
    p.usart2.cr1().write(|w| {
        w.ue().enabled()
        .te().enabled()
        .re().enabled()
    });
}

fn send_welcome_message(p: &Peripherals) {
    TX_BUFFER.get(|tx_buf| {
        queue_string(tx_buf, "\r\n****************************************\r\n");
        queue_string(tx_buf, "*            Bootloader v1.0.0           *\r\n");
        queue_string(tx_buf, "****************************************\r\n\r\n");
        queue_string(tx_buf, "Press 'U' to enter updater\r\n");
        queue_string(tx_buf, "Press 'F' to update firmware\r\n");
        queue_string(tx_buf, "Press 'D' to view diagnostics\r\n");
        queue_string(tx_buf, "Press 'Enter' to boot application\r\n");
        queue_string(tx_buf, "Will boot automatically in 10 seconds...\r\n");
    });
    
    // Ensure transmission starts
    ensure_transmitting();
}

pub fn ensure_transmitting() {
    if !TX_IN_PROGRESS.load(Ordering::SeqCst) {
        // Check if there is any data that can be transferred
        if let Some(byte) = TX_BUFFER.get(|buf| buf.read()) {
            USART2_PTR.get(|usart_opt| {
                if let Some(ref usart_ptr) = *usart_opt {
                    unsafe {
                        // get USART2
                        let usart2: &stm32f4::usart1::RegisterBlock = &*(usart_ptr.0 as *const pac::usart1::RegisterBlock);
                        
                        // Write to DR will fix TXE
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

fn handle_user_input(p: &pac::Peripherals, byte: u8, update_option_selected: &mut bool) {
    match byte {
        b'U' | b'u' => {
            // Boot updater if available
            let is_updater_valid: bool = unsafe { *(UPDATER_ADDR as *const u32) != 0xFFFFFFFF };
            
            if !is_updater_valid {
                TX_BUFFER.get(|tx_buf| {
                    queue_string(tx_buf, "\r\nValid updater not found!\r\n");
                });
            } else {
                TX_BUFFER.get(|tx_buf| {
                    queue_string(tx_buf, "\r\nBooting updater...\r\n");
                });
                LOAD_UPDATER.store(true, Ordering::SeqCst);
                
                while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                    ensure_transmitting();
                }
            }
        },
        b'F' | b'f' => {
            if !*update_option_selected {
                *update_option_selected = true;
                TX_BUFFER.get(|tx_buf| {
                    queue_string(tx_buf, "\r\n****************************************\r\n");
                    queue_string(tx_buf, "*           Firmware Update            *\r\n");
                    queue_string(tx_buf, "****************************************\r\n\r\n");
                    queue_string(tx_buf, "Please select an update method:\r\n\r\n");
                    queue_string(tx_buf, " > 'A' - Update Application image\r\n\r\n");
                    queue_string(tx_buf, " > 'U' - Update Updater image\r\n\r\n");
                });
            }
        },
        b'A' | b'a' => {
            if *update_option_selected {
                TX_BUFFER.get(|tx_buf| {
                    start_update(tx_buf, APP_ADDR, true);
                });
                *update_option_selected = false;
            }
        },
        b'U' => {
            if *update_option_selected {
                TX_BUFFER.get(|tx_buf| {
                    start_update(tx_buf, UPDATER_ADDR, false);
                });
                *update_option_selected = false;
            }
        },
        b'D' | b'd' => {
            // Show diagnostic information
            let (state, error_code, packets) = get_diagnostic_info();
            
            TX_BUFFER.get(|tx_buf| {
                queue_string(tx_buf, "\r\n--- Firmware Update Diagnostics ---\r\n");
                
                // Show current/last state
                queue_string(tx_buf, "State: ");
                match state {
                    XmodemState::Idle => queue_string(tx_buf, "Idle"),
                    XmodemState::Init => queue_string(tx_buf, "Initializing"),
                    XmodemState::Receiving => queue_string(tx_buf, "Receiving"),
                    XmodemState::Complete => queue_string(tx_buf, "Complete"),
                    XmodemState::Error => queue_string(tx_buf, "Error"),
                }
                queue_string(tx_buf, "\r\n");
                
                // Show error code (if any)
                queue_string(tx_buf, "Error code: ");
                write_decimal(tx_buf, error_code as u32);
                if error_code > 0 {
                    queue_string(tx_buf, " (");
                    match error_code {
                        1 => queue_string(tx_buf, "Operation cancelled by sender"),
                        2 => queue_string(tx_buf, "Firmware header invalid"),
                        3 => queue_string(tx_buf, "Flash erase failed"),
                        4 => queue_string(tx_buf, "Flash write failed"),
                        5 => queue_string(tx_buf, "Sequence number error"),
                        6 => queue_string(tx_buf, "CRC check failed"),
                        7 => queue_string(tx_buf, "Buffer overflow"),
                        8 => queue_string(tx_buf, "Timeout"),
                        _ => queue_string(tx_buf, "Unknown error"),
                    }
                    queue_string(tx_buf, ")");
                }
                queue_string(tx_buf, "\r\n");
                
                // Show packet statistics
                queue_string(tx_buf, "Packets received: ");
                write_decimal(tx_buf, packets);
                queue_string(tx_buf, "\r\n");
                
                // Show memory addresses
                queue_string(tx_buf, "Target address: 0x");
                write_hex(tx_buf, unsafe { CURRENT_ADDRESS });
                queue_string(tx_buf, "\r\n");
                
                // Show other useful information
                queue_string(tx_buf, "Target type: ");
                if TARGET_APP.load(Ordering::Relaxed) {
                    queue_string(tx_buf, "Application");
                } else {
                    queue_string(tx_buf, "Updater");
                }
                queue_string(tx_buf, "\r\n");
                
                queue_string(tx_buf, "\r\n--- End of Diagnostics ---\r\n\r\n");
                
                // Return to menu
                show_menu(tx_buf);
            });
        },
        b'\r' | b'\n' => {
            if !*update_option_selected {
                let is_app_valid: bool = unsafe { *(APP_ADDR as *const u32) != 0xFFFFFFFF };
                
                if !is_app_valid {
                    TX_BUFFER.get(|tx_buf| {
                        queue_string(tx_buf, "\r\nValid application not found!\r\n");
                    });
                } else {
                    TX_BUFFER.get(|tx_buf| {
                        queue_string(tx_buf, "\r\nBooting application...\r\n");
                    });
                    LOAD_APPLICATION.store(true, Ordering::SeqCst);
                    
                    while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                        ensure_transmitting();
                    }
                }
            }
        },
        _ => {
            if byte != 0 && !*update_option_selected {
                TX_BUFFER.get(|tx_buf| {
                    show_menu(tx_buf);
                });
            }
        },
    }
}

fn show_menu(tx_buffer: &RingBuffer) {
    queue_string(tx_buffer, "\r\nPlease select from available options:\r\n");
    queue_string(tx_buffer, " > 'U' - Boot updater\r\n");
    queue_string(tx_buffer, " > 'F' - Update firmware\r\n");
    queue_string(tx_buffer, " > 'D' - Show diagnostics\r\n");
    queue_string(tx_buffer, " > 'Enter' - Boot application\r\n");
}

fn rcc_deinit(p: &Peripherals) {
    // Reset clock
    p.rcc.cr().modify(|_, w| w.hsion().set_bit());
    while p.rcc.cr().read().hsirdy().bit_is_clear() {
        // wait
    }

    // Set HSITRIM[4:0] bits to the reset value
    p.rcc.cr().modify(|_, w| unsafe {
        w.hsitrim().bits(0x10)
    });

    p.rcc.cfgr().reset();
    while !p.rcc.cfgr().read().sws().is_hsi() {
        // wait
    }

    p.rcc.cr().modify(|_, w| w
        .hseon().clear_bit()
        .hsebyp().clear_bit()
        .csson().clear_bit()
    );
    while p.rcc.cr().read().hserdy().bit_is_set() {
        // wait
    }

    //reset PLL
    p.rcc.cr().modify(|_, w| w.pllon().clear_bit());
    while p.rcc.cr().read().pllrdy().bit_is_set() {
        // wait
    }

    // reset PLL configuration
    p.rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllm().bits(0x10)
        .plln().bits(0x040)
        .pllp().bits(0x080)
        .pllq().bits(0x4)
    });

    // disable all interrupts
    p.rcc.cir().modify(|_, w| w
        .lsirdyie().clear_bit()
        .lserdyie().clear_bit()
        .hsirdyie().clear_bit()
        .pllrdyie().clear_bit()
    );
    p.rcc.cir().modify(|_, w| w
        .lsirdyc().clear_bit()
        .lserdyc().clear_bit()
        .hsirdyc().clear_bit()
        .pllrdyc().clear_bit()
    );

    // reset all CSR flags
    p.rcc.csr().modify(|_, w| w.rmvf().set_bit());
}

fn deinit(p: &Peripherals) {
    // force reset for all peripherals
    p.rcc.apb1rstr().write(|w| unsafe { w.bits(0xF6FEC9FF) });
    p.rcc.apb1rstr().write(|w| unsafe { w.bits(0x0) });

    p.rcc.apb2rstr().write(|w| unsafe { w.bits(0x04777933) });
    p.rcc.apb2rstr().write(|w| unsafe { w.bits(0x0) });

    p.rcc.ahb1rstr().write(|w| unsafe { w.bits(0x226011FF) });
    p.rcc.ahb1rstr().write(|w| unsafe { w.bits(0x0) });

    p.rcc.ahb2rstr().write(|w| unsafe { w.bits(0x000000C1) });
    p.rcc.ahb2rstr().write(|w| unsafe { w.bits(0x0) });

    p.rcc.ahb3rstr().write(|w| unsafe { w.bits(0x00000001) });
    p.rcc.ahb3rstr().write(|w| unsafe { w.bits(0x0) });
}

fn boot_application(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) -> ! {
    let mut is_app_valid: bool = false;
    let header_ptr: *const ImageHeader = APP_ADDR as *const ImageHeader;
    unsafe {  
        // check if magic is correct
        if (*header_ptr).image_magic == IMAGE_MAGIC_APP {
            is_app_valid = true;
        }
    };

    if !is_app_valid {
        TX_BUFFER.get(|tx_buf| {
            queue_string(tx_buf, "\r\nValid application not found!\r\n");
        });
        
        while TX_IN_PROGRESS.load(Ordering::SeqCst) {
            ensure_transmitting();
        }
        
        loop {
            asm::nop();
        }
    }

    let reset_addr: u32 = APP_ADDR + IMAGE_HDR_SIZE + 4;
    let stack_addr: u32 = unsafe {
        *((APP_ADDR + IMAGE_HDR_SIZE) as *const u32)
    };
    let reset_vector: u32 = unsafe {
        *(reset_addr as *const u32)
    };

    rcc_deinit(p);
    deinit(p);

    // remap
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());
    p.syscfg.memrmp().write(|w| unsafe {
        w.bits(0x01)
    });

    // disable SysTick
    let mut cp: cortex_m::Peripherals = unsafe {
        cortex_m::Peripherals::steal()
    };
    cp.SYST.disable_counter();
    cp.SYST.disable_interrupt();

    unsafe {
        let scb: *const cortex_m::peripheral::scb::RegisterBlock = SCB::ptr();

        let icsr: u32 = (*scb).icsr.read();
        (*scb).icsr.write(icsr | (1 << 25));

        (*scb).shcsr.modify(|v: u32| v & !(
            (1 << 18) | (1 << 17) | (1 << 16)
        ));

        (*scb).vtor.write(APP_ADDR + IMAGE_HDR_SIZE);

        // set MSP
        core::arch::asm!("MSR msp, {0}", in(reg) stack_addr);

        let jump_fn: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
        jump_fn();
    }
}

fn boot_updater(p: &pac::Peripherals, cp: &mut cortex_m::Peripherals) -> ! {
    let mut is_updater_valid: bool = false;
    let header_ptr: *const ImageHeader = UPDATER_ADDR as *const ImageHeader;
    unsafe {  
        // check if magic is correct
        if (*header_ptr).image_magic == IMAGE_MAGIC_UPDATER {
            is_updater_valid = true;
        }
    };

    if !is_updater_valid {
        TX_BUFFER.get(|tx_buf| {
            queue_string(tx_buf, "\r\nValid updater not found!\r\n");
        });
        
        while TX_IN_PROGRESS.load(Ordering::SeqCst) {
            ensure_transmitting();
        }
        
        loop {
            asm::nop();
        }
    }

    let reset_addr: u32 = UPDATER_ADDR + IMAGE_HDR_SIZE + 4;
    let stack_addr: u32 = unsafe {
        *((UPDATER_ADDR + IMAGE_HDR_SIZE) as *const u32)
    };
    let reset_vector: u32 = unsafe {
        *(reset_addr as *const u32)
    };

    rcc_deinit(p);
    deinit(p);

    // remap
    p.rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());
    p.syscfg.memrmp().write(|w| unsafe {
        w.bits(0x01)
    });

    // disable SysTick
    let mut cp: cortex_m::Peripherals = unsafe {
        cortex_m::Peripherals::steal()
    };
    cp.SYST.disable_counter();
    cp.SYST.disable_interrupt();

    unsafe {
        let scb: *const cortex_m::peripheral::scb::RegisterBlock = SCB::ptr();

        let icsr: u32 = (*scb).icsr.read();
        (*scb).icsr.write(icsr | (1 << 25));

        (*scb).shcsr.modify(|v: u32| v & !(
            (1 << 18) | (1 << 17) | (1 << 16)
        ));

        (*scb).vtor.write(UPDATER_ADDR + IMAGE_HDR_SIZE);

        // set MSP
        core::arch::asm!("MSR msp, {0}", in(reg) stack_addr);

        let jump_fn: extern "C" fn() -> ! = core::mem::transmute(reset_vector);
        jump_fn();
    }
}

// LED diagnostic pins - assuming PD12-PD15
fn set_led_status(p: &pac::Peripherals, error_code: u8) {
    unsafe {
        // Clear all LEDs first
        p.gpiod.bsrr().write(|w| w
            .br12().set_bit()
            .br13().set_bit()
            .br14().set_bit()
            .br15().set_bit()
        );
        
        // Use binary pattern on LEDs to show error code
        if error_code & 0x01 != 0 {
            p.gpiod.bsrr().write(|w| w.bs12().set_bit());
        }
        if error_code & 0x02 != 0 {
            p.gpiod.bsrr().write(|w| w.bs13().set_bit());
        }
        if error_code & 0x04 != 0 {
            p.gpiod.bsrr().write(|w| w.bs14().set_bit());
        }
        if error_code & 0x08 != 0 {
            p.gpiod.bsrr().write(|w| w.bs15().set_bit());
        }
    }
}

// Get current XMODEM state
pub fn get_state() -> XmodemState {
    match XMODEM_STATE.load(Ordering::Relaxed) {
        0 => XmodemState::Idle,
        1 => XmodemState::Init,
        2 => XmodemState::Receiving,
        3 => XmodemState::Complete,
        4 => XmodemState::Error,
        _ => XmodemState::Idle,
    }
}

// Set XMODEM state
pub fn set_state(state: XmodemState) {
    let value = match state {
        XmodemState::Idle => 0,
        XmodemState::Init => 1,
        XmodemState::Receiving => 2,
        XmodemState::Complete => 3,
        XmodemState::Error => 4,
    };
    XMODEM_STATE.store(value, Ordering::Relaxed);
}

// Check if update is in progress
pub fn is_update_in_progress() -> bool {
    UPDATE_IN_PROGRESS.load(Ordering::Relaxed)
}

// Function to queue a string to the TX buffer
pub fn queue_string(tx_buffer: &RingBuffer, s: &str) {
    for byte in s.bytes() {
        tx_buffer.write(byte);
    }
}

// After completion, this function can report diagnostic info
pub fn get_diagnostic_info() -> (XmodemState, u8, u32) {
    let state = get_state();
    let error = ERROR_CODE.load(Ordering::Relaxed);
    let packets = unsafe { PACKET_COUNTER };
    
    (state, error, packets)
}

// Helper function to start the firmware update process
pub fn start_update(tx_buffer: &RingBuffer, target_addr: u32, is_app_update: bool) {
    // Reset state
    set_state(XmodemState::Init);
    SEQUENCE.store(1, Ordering::Relaxed);
    FIRST_PACKET.store(true, Ordering::Relaxed);
    TARGET_APP.store(is_app_update, Ordering::Relaxed);
    UPDATE_IN_PROGRESS.store(true, Ordering::Relaxed);
    ERROR_CODE.store(ERROR_NONE, Ordering::Relaxed);
    
    unsafe {
        CURRENT_ADDRESS = target_addr;
        BUFFER_INDEX = 0;
        PARTIAL_PACKET = [0; 1029];
        LAST_C_TIME = systick::get_tick_ms();
        C_SENT_COUNT = 0; // Reset the counter
        PACKET_COUNTER = 0;
    }
    
    queue_string(tx_buffer, "\r\nStarting firmware update. Send file using XMODEM-CRC protocol...\r\n");
    
    // Send 'C' to request XMODEM-CRC transfer
    tx_buffer.write(C);
}

// Process XMODEM state machine
pub fn process_xmodem(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) -> bool {
    let state = get_state();
    if state == XmodemState::Idle {
        return false;
    }
    
    // Update LED status based on state
    match state {
        XmodemState::Init => {
            // Blink PD12 to indicate initialization
            let current_time = systick::get_tick_ms();
            if (current_time / 500) % 2 == 0 {
                unsafe { p.gpiod.bsrr().write(|w| w.bs12().set_bit()); }
            } else {
                unsafe { p.gpiod.bsrr().write(|w| w.br12().set_bit()); }
            }
        },
        XmodemState::Receiving => {
            // PD12 solid on, PD13 blinks with each packet
            unsafe { 
                p.gpiod.bsrr().write(|w| w.bs12().set_bit());
                
                // Flash PD13 based on packet count
                let packet_count = PACKET_COUNTER;
                if (packet_count / 2) % 2 == 0 {
                    p.gpiod.bsrr().write(|w| w.bs13().set_bit());
                } else {
                    p.gpiod.bsrr().write(|w| w.br13().set_bit());
                }
            }
        },
        XmodemState::Complete => {
            // All LEDs on for success
            unsafe {
                p.gpiod.bsrr().write(|w| w
                    .bs12().set_bit()
                    .bs13().set_bit()
                    .bs14().set_bit()
                    .bs15().set_bit()
                );
            }
        },
        XmodemState::Error => {
            // Show error code on LEDs
            let error = ERROR_CODE.load(Ordering::Relaxed);
            set_led_status(p, error);
        },
        _ => {}
    }
    
    // Periodically send 'C' in Init state
    if state == XmodemState::Init {
        let current_time = systick::get_tick_ms();
        
        unsafe {
            // Send 'C' every 1 second, up to 10 attempts
            if current_time.wrapping_sub(LAST_C_TIME) >= 1000 {
                tx_buffer.write(C);
                LAST_C_TIME = current_time;
                C_SENT_COUNT += 1;
                
                // After 10 attempts, try one NAK
                if C_SENT_COUNT >= 10 {
                    tx_buffer.write(NAK);
                    set_state(XmodemState::Receiving);
                }
            }
        }
    }
    
    // Process available bytes in receiving state
    if state == XmodemState::Receiving || state == XmodemState::Init {
        while let Some(byte) = rx_buffer.read() {
            match process_byte(p, tx_buffer, byte) {
                true => return true, // Transfer completed or error
                false => continue,   // Continue processing
            }
        }
    }
    
    false
}

// Calculate simple CRC-16 CCITT for Xmodem
fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// Process a single byte in the XMODEM protocol
fn process_byte(p: &pac::Peripherals, tx_buffer: &RingBuffer, byte: u8) -> bool {
    unsafe {
        // Check for EOT (End of Transmission)
        if BUFFER_INDEX == 0 && byte == EOT {
            tx_buffer.write(ACK);
            
            queue_string(tx_buffer, "\r\nFirmware update completed successfully!\r\n");
            
            set_state(XmodemState::Complete);
            UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
            return true;
        }
        
        // Check for CAN (Cancel)
        if byte == CAN {
            ERROR_CODE.store(ERROR_CANCEL, Ordering::Relaxed);
            set_state(XmodemState::Error);
            UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
            queue_string(tx_buffer, "\r\nOperation cancelled by sender\r\n");
            return true;
        }
        
        // Add byte to buffer
        PARTIAL_PACKET[BUFFER_INDEX] = byte;
        BUFFER_INDEX += 1;
        
        // Detect packet type from first byte
        if BUFFER_INDEX == 1 {
            if byte != SOH && byte != STX {
                BUFFER_INDEX = 0;
                tx_buffer.write(NAK);
                return false;
            }
        }
        
        // Determine packet size based on first byte
        let packet_size = if BUFFER_INDEX > 0 && PARTIAL_PACKET[0] == STX {
            1029 // STX + SEQ + COMPLEMENTED_SEQ + DATA[1024] + CRC[2]
        } else {
            133  // SOH + SEQ + COMPLEMENTED_SEQ + DATA[128] + CRC[2]
        };
        
        // Process complete packet
        if BUFFER_INDEX == packet_size {
            // Reset buffer for next packet
            BUFFER_INDEX = 0;
            
            // Verify sequence number and its complement
            let seq = PARTIAL_PACKET[1];
            let comp_seq = PARTIAL_PACKET[2];
            
            // Check if sequence and complement match
            if (seq ^ comp_seq) != 0xFF {
                tx_buffer.write(NAK);
                ERROR_CODE.store(ERROR_SEQUENCE, Ordering::Relaxed);
                return false;
            }
            
            // Check if sequence number matches expected
            let expected_seq = SEQUENCE.load(Ordering::Relaxed);
            if seq != expected_seq {
                // If seq is 1 and we expected >1, this could be a resend of first packet
                if seq == 1 && expected_seq > 1 {
                    tx_buffer.write(ACK); // ACK anyway to avoid deadlock
                    return false;
                }
                
                tx_buffer.write(NAK);
                ERROR_CODE.store(ERROR_SEQUENCE, Ordering::Relaxed);
                return false;
            }
            
            // Calculate data length based on packet type
            let data_len = if PARTIAL_PACKET[0] == SOH { 128 } else { 1024 };
            
            // Verify CRC
            let data_offset = 3; // SOH/STX + SEQ + COMP
            let crc_offset = data_offset + data_len;
            
            // Extract CRC from packet (big-endian)
            let received_crc = ((PARTIAL_PACKET[crc_offset] as u16) << 8) | 
                                (PARTIAL_PACKET[crc_offset + 1] as u16);
                                
            // Calculate CRC over data portion
            let calculated_crc = calculate_crc16(&PARTIAL_PACKET[data_offset..crc_offset]);
            
            if calculated_crc != received_crc {
                tx_buffer.write(NAK);
                ERROR_CODE.store(ERROR_CRC, Ordering::Relaxed);
                return false;
            }
            
            // For the first packet, check firmware header
            if FIRST_PACKET.load(Ordering::Relaxed) {
                // Process header data (at data_offset)
                let valid = validate_header(&PARTIAL_PACKET[data_offset..]);
                if !valid {
                    // Send CAN to cancel transfer
                    tx_buffer.write(CAN);
                    tx_buffer.write(CAN);
                    tx_buffer.write(CAN);
                    
                    ERROR_CODE.store(ERROR_HEADER_INVALID, Ordering::Relaxed);
                    set_state(XmodemState::Error);
                    UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
                    queue_string(tx_buffer, "\r\nFirmware header validation failed!\r\n");
                    return true;
                }
                
                // Erase flash sector before writing first packet
                let erased_size = flash::erase_sector(p, CURRENT_ADDRESS);
                
                if erased_size == 0 {
                    tx_buffer.write(CAN);
                    tx_buffer.write(CAN);
                    tx_buffer.write(CAN);
                    
                    ERROR_CODE.store(ERROR_FLASH_ERASE, Ordering::Relaxed);
                    set_state(XmodemState::Error);
                    UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
                    queue_string(tx_buffer, "\r\nFlash erase failed!\r\n");
                    return true;
                }
                
                FIRST_PACKET.store(false, Ordering::Relaxed);
            }
            
            // Write data to flash
            let result = flash::write(p, &PARTIAL_PACKET[data_offset..(data_offset + data_len)], CURRENT_ADDRESS);
            
            if result != 0 {
                tx_buffer.write(CAN);
                tx_buffer.write(CAN);
                tx_buffer.write(CAN);
                
                ERROR_CODE.store(ERROR_FLASH_WRITE, Ordering::Relaxed);
                set_state(XmodemState::Error);
                UPDATE_IN_PROGRESS.store(false, Ordering::Relaxed);
                queue_string(tx_buffer, "\r\nFlash write failed!\r\n");
                return true;
            }
            
            // Update address and acknowledge packet
            CURRENT_ADDRESS += data_len as u32;
            tx_buffer.write(ACK);
            
            // Increment packet counter (for LED blinking)
            PACKET_COUNTER += 1;
            
            // Increment sequence number for next packet
            let next_seq = expected_seq.wrapping_add(1);
            SEQUENCE.store(next_seq, Ordering::Relaxed);
        } else if BUFFER_INDEX > packet_size {
            // Buffer overflow - reset
            BUFFER_INDEX = 0;
            tx_buffer.write(NAK);
            ERROR_CODE.store(ERROR_OVERFLOW, Ordering::Relaxed);
        }
    }
    
    false
}

// Function to validate the image header
fn validate_header(data: &[u8]) -> bool {
    if data.len() < core::mem::size_of::<ImageHeader>() {
        return false;
    }
    
    // Extract bytes for header fields manually to avoid alignment issues
    
    // image_magic: u32 (bytes 0-3)
    let magic = u32::from_le_bytes([
        data[0], data[1], data[2], data[3]
    ]);
    
    // image_type: u8 (byte 6)
    let image_type = data[6];
    
    // version fields: u8 (bytes 7-9)
    let major = data[7];
    let minor = data[8];
    let patch = data[9];
    
    // Check if target is application or updater
    let (expected_magic, expected_type) = if TARGET_APP.load(Ordering::Relaxed) {
        (IMAGE_MAGIC_APP, IMAGE_TYPE_APP)
    } else {
        (IMAGE_MAGIC_UPDATER, IMAGE_TYPE_UPDATER)
    };
    
    // Check magic number and type
    if magic != expected_magic || image_type != expected_type {
        return false;
    }
    
    // Check version if there's existing firmware
    unsafe {
        let dest_addr = CURRENT_ADDRESS;
        
        if *(dest_addr as *const u32) != 0xFFFFFFFF {
            // We have existing firmware, need to check version
            
            // Use manual memory access to avoid alignment issues
            let existing_magic = *(dest_addr as *const u32);
            
            // Only continue if magic matches expected
            if existing_magic == expected_magic {
                // Access version fields directly
                let existing_major = *((dest_addr + 7) as *const u8);
                let existing_minor = *((dest_addr + 8) as *const u8);
                let existing_patch = *((dest_addr + 9) as *const u8);
                
                // Compare versions
                let is_newer = 
                    major > existing_major ||
                    (major == existing_major && minor > existing_minor) ||
                    (major == existing_major && minor == existing_minor && patch > existing_patch);
                
                if !is_newer {
                    return false;
                }
            }
        }
    }
    
    true
}

#[no_mangle]
pub extern "C" fn USART2() {
    USART2_PTR.get(|usart_opt: &mut Option<PeripheralPtr<stm32f4::usart1::RegisterBlock>>| {
        if let Some(ref usart_ptr) = *usart_opt {
            unsafe {
                let usart2 = &*(usart_ptr.0 as *const pac::usart2::RegisterBlock);

                // check data in RX buffer
                if usart2.sr().read().rxne().bit_is_set() {
                    let data: u8 = usart2.dr().read().bits() as u8;
                    RX_BUFFER.get(|buf: &mut RingBuffer| {
                        buf.write(data);
                    });
                }

                // check if we can TX
                if usart2.sr().read().txe().bit_is_set() && usart2.cr1().read().txeie().bit_is_set() {
                    TX_IN_PROGRESS.store(false, Ordering::SeqCst);

                    if let Some(byte) = TX_BUFFER.get(|buf| buf.read()) {
                        usart2.dr().write(|w| unsafe {
                            w.bits(byte as u16)
                        });
                        TX_IN_PROGRESS.store(true, Ordering::SeqCst);
                    } else {
                        // disable TXE because no data left
                        usart2.cr1().modify(|_, w| w.txeie().disabled());
                    }
                }
            }
        }
    })
}

#[exception]
fn SysTick() {
    systick::increment_tick();
}

#[exception]
unsafe fn HardFault(_info: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {
        asm::nop();
    }
}

#[exception]
unsafe fn DefaultHandler(_irqn: i16) {
    loop {
        asm::nop();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}