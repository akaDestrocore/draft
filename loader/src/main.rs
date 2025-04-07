#![no_std]
#![no_main]

mod bootloader;
mod xmodem;
mod uart;
mod led;

use bootloader::{BootOption, boot_application, boot_updater};
use core::panic::PanicInfo;
use cortex_m::{asm, peripheral::SYST};
use cortex_m_rt::{entry, exception};
use led::Leds;
use misc::{
    image::{ImageHeader, SharedMemory, IMAGE_MAGIC_LOADER, IMAGE_TYPE_LOADER},
    systick,
};
use stm32f4 as pac;
use stm32f4::Peripherals;
use uart::{UartManager, UartError};
use xmodem::{XmodemManager, XmodemError, XmodemState};

// Flash memory addresses
pub const UPDATER_ADDR: u32 = 0x08008000;
pub const APP_ADDR: u32 = 0x08020000;
pub const IMAGE_HDR_SIZE: u32 = 0x200;
pub const BOOT_TIMEOUT_MS: u32 = 10_000; // 10 seconds

#[no_mangle]
#[link_section = ".shared_memory"]
pub static mut SHARED_MEMORY: SharedMemory = SharedMemory::new();

#[no_mangle]
#[link_section = ".image_hdr"]
pub static IMAGE_HEADER: ImageHeader = ImageHeader::new(
    IMAGE_TYPE_LOADER,
    IMAGE_MAGIC_LOADER,
    1, 0, 0  // ver 1.0.0
);

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Setup system clock to 90MHz
    setup_system_clock(&p);

    // Enable peripheral clocks
    enable_peripherals(&p);

    // Setup SysTick
    systick::setup_systick(&mut cp.SYST);
    let start_time = systick::get_tick_ms();

    // Setup GPIO pins
    setup_gpio_pins(&p);

    // Initialize peripherals
    let mut leds = Leds::new(&p);
    let mut uart = UartManager::new(&p);
    let mut xmodem = XmodemManager::new();

    leds.init();
    uart.init();
    
    // Blink all LEDs to indicate bootloader started
    leds.set_all(true);
    
    uart.send_string("\r\n****************************************\r\n");
    uart.send_string("*            Bootloader v1.0.0           *\r\n");
    uart.send_string("****************************************\r\n\r\n");
    uart.send_string("Press 'U' to enter updater\r\n");
    uart.send_string("Press 'F' to update firmware using XMODEM\r\n");
    uart.send_string("Press 'A' to boot application\r\n");
    uart.send_string("Press 'Enter' to boot application\r\n");
    uart.send_string("Will boot automatically in 10 seconds...\r\n");

    // Enable USART2 interrupt in NVIC
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
    }

    let mut boot_option = BootOption::None;
    let mut update_in_progress = false;
    let mut firmware_target = APP_ADDR;

    loop {
        // Toggle LED to show we're alive
        leds.toggle(1);
        
        // Wait a bit
        systick::wait_ms(systick::get_tick_ms(), 500);
        
        // Process UART data
        uart.process();

        // Handle user input if not in update mode
        if !update_in_progress {
            if let Some(byte) = uart.read_byte() {
                match byte {
                    b'U' | b'u' => {
                        // Boot updater
                        if bootloader::is_firmware_valid(UPDATER_ADDR) {
                            uart.send_string("\r\nBooting updater...\r\n");
                            boot_option = BootOption::Updater;
                        } else {
                            uart.send_string("\r\nValid updater not found!\r\n");
                        }
                    },
                    b'F' | b'f' => {
                        // Start firmware update
                        uart.send_string("\r\nUpdate firmware using XMODEM - select target:\r\n");
                        uart.send_string("'A' - Application\r\n");
                        uart.send_string("'U' - Updater\r\n");
                        boot_option = BootOption::SelectUpdateTarget;
                    },
                    b'A' | b'a' => {
                        if boot_option == BootOption::SelectUpdateTarget {
                            // Start application update
                            uart.send_string("\r\nUpdating application...\r\n");
                            uart.send_string("Send file using XMODEM protocol\r\n");
                            firmware_target = APP_ADDR;
                            xmodem.start(firmware_target);
                            update_in_progress = true;
                            boot_option = BootOption::None;
                        } else {
                            // Directly boot application
                            if bootloader::is_firmware_valid(APP_ADDR) {
                                uart.send_string("\r\nBooting application...\r\n");
                                boot_option = BootOption::Application;
                            } else {
                                uart.send_string("\r\nValid application not found!\r\n");
                            }
                        }
                    },
                    b'D' | b'd' => {
                        // Debug info
                        uart.send_string("\r\n--- System Diagnostics ---\r\n");
                        let state_str = match xmodem.get_state() {
                            XmodemState::Idle => "Idle",
                            XmodemState::WaitSOH => "WaitSOH",
                            XmodemState::WaitIndex1 => "WaitIndex1",
                            XmodemState::WaitIndex2 => "WaitIndex2",
                            XmodemState::ReceiveData => "ReceiveData",
                            XmodemState::CheckCrc => "CheckCrc",
                            XmodemState::FinishTransfer => "FinishTransfer",
                            XmodemState::Error => "Error",
                        };
                        uart.send_string("XMODEM state: ");
                        uart.send_string(state_str);
                        uart.send_string("\r\n");
                        
                        uart.send_string("App valid: ");
                        uart.send_string(if bootloader::is_firmware_valid(APP_ADDR) { "Yes" } else { "No" });
                        uart.send_string("\r\n");
                        
                        uart.send_string("Updater valid: ");
                        uart.send_string(if bootloader::is_firmware_valid(UPDATER_ADDR) { "Yes" } else { "No" });
                        uart.send_string("\r\n");
                    },
                    b'\r' | b'\n' => {
                        // Boot application
                        if bootloader::is_firmware_valid(APP_ADDR) {
                            uart.send_string("\r\nBooting application...\r\n");
                            boot_option = BootOption::Application;
                        } else {
                            uart.send_string("\r\nValid application not found!\r\n");
                        }
                    },
                    _ => {
                        if byte != 0 {
                            if boot_option == BootOption::SelectUpdateTarget {
                                if byte == b'U' || byte == b'u' {
                                    // Update updater firmware
                                    uart.send_string("\r\nUpdating updater...\r\n");
                                    uart.send_string("Send file using XMODEM protocol\r\n");
                                    firmware_target = UPDATER_ADDR;
                                    xmodem.start(firmware_target);
                                    update_in_progress = true;
                                    boot_option = BootOption::None;
                                } else {
                                    uart.send_string("\r\nInvalid option, cancelled.\r\n");
                                    boot_option = BootOption::None;
                                }
                            } else {
                                uart.send_string("\r\nInvalid option, try again.\r\n");
                            }
                        }
                    },
                }
            }
        } else {
            // Handle XMODEM update
            if let Some(byte) = uart.read_byte() {
                match xmodem.process_byte(byte) {
                    Ok(true) => {
                        // Need to send a response
                        if let Some(response) = xmodem.get_response() {
                            uart.send_byte(response);
                        }
                    },
                    Ok(false) => {
                        // No response needed
                    },
                    Err(XmodemError::TransferComplete) => {
                        uart.send_string("\r\nTransfer complete! Firmware updated successfully.\r\n");
                        update_in_progress = false;
                    },
                    Err(XmodemError::Cancelled) => {
                        uart.send_string("\r\nTransfer cancelled.\r\n");
                        update_in_progress = false;
                    },
                    Err(XmodemError::Timeout) => {
                        uart.send_string("\r\nTransfer timed out.\r\n");
                        update_in_progress = false;
                    },
                    Err(XmodemError::InvalidPacket) => {
                        // XMODEM will handle retries, we just send responses
                        if let Some(response) = xmodem.get_response() {
                            uart.send_byte(response);
                        }
                    },
                    Err(XmodemError::FlashWriteError) => {
                        uart.send_string("\r\nError writing to flash memory.\r\n");
                        update_in_progress = false;
                    },
                }
            }
            
            // Send periodic 'C' during startup phase
            if xmodem.get_state() == XmodemState::WaitSOH {
                if xmodem.should_send_c() {
                    uart.send_byte(b'C');
                }
            }
            
            // Check if XMODEM is in an error state
            if xmodem.get_state() == XmodemState::Error {
                uart.send_string("\r\nXMODEM transfer error. Aborting.\r\n");
                update_in_progress = false;
            }
        }

        // Handle boot options
        match boot_option {
            BootOption::Application => {
                // Wait for UART to finish sending
                while !uart.is_tx_complete() {
                    uart.process();
                }
                boot_application(&p, &mut cp);
            },
            BootOption::Updater => {
                // Wait for UART to finish sending
                while !uart.is_tx_complete() {
                    uart.process();
                }
                boot_updater(&p, &mut cp);
            },
            _ => {}
        }

        // Check for auto-boot timeout (only if not updating and no option selected)
        if !update_in_progress && boot_option == BootOption::None {
            let current_time = systick::get_tick_ms();
            if current_time.wrapping_sub(start_time) >= BOOT_TIMEOUT_MS {
                if bootloader::is_firmware_valid(APP_ADDR) {
                    uart.send_string("\r\nAuto-boot timeout reached. Booting application...\r\n");
                    
                    // Wait for UART to finish sending
                    while !uart.is_tx_complete() {
                        uart.process();
                    }
                    
                    boot_application(&p, &mut cp);
                } else {
                    uart.send_string("\r\nAuto-boot timeout reached but no valid application found.\r\n");
                    uart.send_string("Please flash a valid application image.\r\n");
                    // Reset the timeout
                    //start_time = systick::get_tick_ms();
                }
            }
        }

        // Wait for interrupt (power saving)
        //asm::wfi();
    }
}

/// Enable peripheral clocks
fn enable_peripherals(p: &Peripherals) {
    // Enable GPIO clocks
    p.rcc.ahb1enr().modify(|_, w| {
        w.gpioaen().enabled()  // For USART2 pins
         .gpioden().enabled()  // For LEDs
    });
    
    // Enable USART2 clock
    p.rcc.apb1enr().modify(|_, w| {
        w.usart2en().enabled()
    });
    
    // Enable SYSCFG clock (needed for bootloader)
    p.rcc.apb2enr().modify(|_, w| {
        w.syscfgen().enabled()
    });
}

/// Setup GPIO pins for UART and LEDs
fn setup_gpio_pins(p: &Peripherals) {
    // Configure GPIOA for USART2 (PA2 = TX, PA3 = RX)
    p.gpioa.moder().modify(|_, w| {
        w.moder2().alternate()  // TX pin
         .moder3().alternate()  // RX pin
    });
    
    // Set alternate function 7 (USART2) for PA2 and PA3
    p.gpioa.afrl().modify(|_, w| {
        w.afrl2().af7()  // TX pin
         .afrl3().af7()  // RX pin
    });
    
    // Set high speed mode
    p.gpioa.ospeedr().modify(|_, w| {
        w.ospeedr2().high_speed()
         .ospeedr3().high_speed()
    });
    
    // No pull-up/pull-down
    p.gpioa.pupdr().modify(|_, w| {
        w.pupdr2().floating()
         .pupdr3().floating()
    });
}

fn setup_system_clock(p: &pac::Peripherals) {
    // Enable PWR clock
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());

    // Set voltage scale
    p.pwr.cr().modify(|_, w| w.vos().scale1());

    // Configure flash latency
    p.flash.acr().modify(|_, w| w
        .latency().ws5()
        .prften().set_bit()
        .icen().set_bit()
        .dcen().set_bit()
    );

    // Enable HSE
    p.rcc.cr().modify(|_, w| w.hseon().set_bit());
    while p.rcc.cr().read().hserdy().bit_is_clear() {
        // wait for HSE ready
    }

    // Configure PLL (HSE = 8MHz, PLLM = 4, PLLN = 90, PLLP = 2, PLLQ = 4)
    // → System clock = (8MHz / 4) * 90 / 2 = 90MHz
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
        // wait for PLL ready
    }

    // Configure bus clocks
    p.rcc.cfgr().modify(|_, w| w
        .hpre().div1()    // AHB = SYSCLK / 1 = 90MHz
        .ppre1().div4()   // APB1 = SYSCLK / 4 = 22.5MHz
        .ppre2().div2()   // APB2 = SYSCLK / 2 = 45MHz
    );

    // Switch to PLL as system clock source
    p.rcc.cfgr().modify(|_, w| w.sw().pll());
    while !p.rcc.cfgr().read().sws().is_pll() {
        // wait for PLL to be the system clock source
    }
}

#[exception]
fn SysTick() {
    systick::increment_tick();
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}