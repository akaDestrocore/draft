#![no_std]
#![no_main]

use core::{
    cell::UnsafeCell, 
    panic::PanicInfo, 
    sync::atomic::{AtomicBool, Ordering}
};

use cortex_m::{
    asm,
    peripheral::{NVIC, SCB, SYST}
};

use cortex_m_rt::{entry, exception};
use stm32f4::{self as pac, Peripherals};
use misc::{
    ring_buffer::RingBuffer,
    image::{ImageHeader, SharedMemory, IMAGE_MAGIC_LOADER, IMAGE_TYPE_LOADER, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER},
    systick,
    flash,
    firmware_update::{
        process_xmodem, 
        start_update, 
        get_state, 
        set_state, 
        is_update_in_progress, 
        queue_string,
        XmodemState
    }
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
                                // Wait for last transmissions to complete
                                while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                                    ensure_transmitting();
                                }
                                
                                // Add small delay to ensure everything is processed
                                let start_ms = systick::get_tick_ms();
                                while !systick::wait_ms(start_ms, 100) {
                                    asm::nop();
                                }
                                
                                // Boot the newly updated firmware based on target
                                if unsafe { *(APP_ADDR as *const u32) != 0xFFFFFFFF } {
                                    TX_BUFFER.get(|buf| {
                                        queue_string(buf, "\r\nApplication update successful, booting...\r\n");
                                    });
                                    while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                                        ensure_transmitting();
                                    }
                                    boot_application(&p, &mut cp);
                                } else {
                                    TX_BUFFER.get(|buf| {
                                        queue_string(buf, "\r\nUpdater update successful, booting...\r\n");
                                    });
                                    while TX_IN_PROGRESS.load(Ordering::SeqCst) {
                                        ensure_transmitting();
                                    }
                                    boot_updater(&p, &mut cp);
                                }
                            },
                            XmodemState::Error => {
                                // Reset to menu
                                set_state(XmodemState::Idle);
                                update_option_selected = false;
                                TX_BUFFER.get(|buf| {
                                    queue_string(buf, "\r\nUpdate failed! Returning to menu.\r\n");
                                    show_menu(buf);
                                });
                            },
                            _ => {}
                        }
                    }
                });
            });
        }
        
        // Check if boot actions are requested
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