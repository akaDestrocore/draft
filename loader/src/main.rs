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
pub const BOOT_TIMEOUT_MS: u32 = 10_000;

// Time in which Enter key press is ignored (because 
// ExtraPuTTY sends Enter after XMODEM, but I want to use this key)
const ENTER_BLOCK_AFTER_UPDATE_MS: u32 = 3_000;
static mut ENTER_BLOCKED_UNTIL: u32 = 0;

#[no_mangle]
#[link_section = ".shared_memory"]
pub static mut SHARED_MEMORY: SharedMemory = SharedMemory::new();

#[no_mangle]
#[link_section = ".image_hdr"]
pub static IMAGE_HEADER: ImageHeader = ImageHeader::new(
    IMAGE_TYPE_LOADER,
    IMAGE_MAGIC_LOADER,
    1, 0, 0
);

// ANSI escape
fn clear_screen(uart: &mut UartManager) {
    uart.send_string("\x1B[2J\x1B[1;1H");
}

fn display_menu(uart: &mut UartManager) {
    clear_screen(uart);
    
    uart.send_string("\r\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\r\n");
    uart.send_string("xxxxxxxx  xxxxxxxxxxxxxxxxxxxx  xxxxxxxxx\r\n");
    uart.send_string("xxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxx\r\n");
    uart.send_string("xxxxxx  xxx  xxxxxxxxxxxxxxx  xx   xxxxxx\r\n");
    uart.send_string("xxxxxxxx  xx  xxxxxxxxxxxxx  xx  xxxxxxxx\r\n");
    uart.send_string("xxxx  xxx   xxxxxxxxxxxxxxxxx  xxx  xxxxx\r\n");
    uart.send_string("xxxxxx    xxxx  xxxxxxxx  xxx     xxxxxxx\r\n");
    uart.send_string("xxxxxxxx xxxxx xx      xx xxxx  xxxxxxxxx\r\n");
    uart.send_string("xxxx     xxxxx   xx  xx   xxxxx     xxxxx\r\n");
    uart.send_string("xxxxxxxx xxxxxxxxxx  xxxxxxxxxx  xxxxxxxx\r\n");
    uart.send_string("xxxxx    xxxxxx  xx  xx  xxxxxx    xxxxxx\r\n");
    uart.send_string("xxxxxxxx  xxxx xxxx  xxxx xxxxx xxxxxxxxx\r\n");
    uart.send_string("xxxxxxx    xxx  xxx  xxx  xxx    xxxxxxxx\r\n");
    uart.send_string("xxxxxxxxxx   xxxxxx  xxxxxx   xxxxxxxxxxx\r\n");
    uart.send_string("xxxxxxxxxxxxxx             xxxxxxxxxxxxxx\r\n");
    uart.send_string("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\r\n");
    uart.send_string("Press 'U' to enter updater\r\n");
    uart.send_string("Press 'F' to update firmware using XMODEM(CRC)\r\n");
    uart.send_string("Press 'Enter' to boot application\r\n");
    uart.send_string("Will boot automatically in 10 seconds\r\n");
}

fn check_application_valid(uart: &mut UartManager) -> bool {
    bootloader::is_firmware_valid(APP_ADDR)
}

// needed to work in ExtraPuTTY properly
fn is_enter_blocked(current_time: u32) -> bool {
    unsafe {
        if ENTER_BLOCKED_UNTIL > current_time {
            return true;
        }
        false
    }
}

fn block_enter_temporarily(current_time: u32) {
    unsafe {
        ENTER_BLOCKED_UNTIL = current_time + ENTER_BLOCK_AFTER_UPDATE_MS;
    }
}

#[entry]
fn main() -> ! {
    let p: Peripherals = pac::Peripherals::take().unwrap();
    let mut cp: cortex_m::Peripherals = cortex_m::Peripherals::take().unwrap();

    // Setup system clock to 90MHz
    setup_system_clock(&p);

    // Enable peripheral clocks
    enable_peripherals(&p);

    // Setup GPIO pins
    setup_gpio_pins(&p);

    // Setup SysTick
    systick::setup_systick(&mut cp.SYST);
    let mut autoboot_timer: u32 = systick::get_tick_ms();

    // Initialize peripherals
    let mut leds: Leds<'_> = Leds::new(&p);
    let mut uart: UartManager<'_> = UartManager::new(&p);
    let mut xmodem: XmodemManager = XmodemManager::new();

    leds.init();
    uart.init();
    
    // initial rx buffer cleanup
    clear_rx_buffer(&mut uart);
    
    display_menu(&mut uart);

    // Enable USART2 interrupt in NVIC
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
    }

    let mut boot_option: BootOption = BootOption::None;
    let mut update_in_progress: bool = false;
    let mut firmware_target: u32 = APP_ADDR;
    let mut led_toggle_time: u32 = systick::get_tick_ms();

    loop {
        // Process UART data
        uart.process();

        // blink green
        let current_time: u32 = systick::get_tick_ms();
        if current_time.wrapping_sub(led_toggle_time) >= 500 {
            leds.toggle(0);
            led_toggle_time = current_time;
        }

        // read keys if not in update mode
        if !update_in_progress {
            if let Some(byte) = uart.read_byte() {
                // if user presses anything reset the autoboot
                autoboot_timer = current_time;
                
                match byte {
                    b'U' | b'u' => {
                        // updater
                        if bootloader::is_firmware_valid(UPDATER_ADDR) {
                            uart.send_string("\r\nBooting updater...\r\n");
                            boot_option = BootOption::Updater;
                        } else {
                            uart.send_string("\r\nValid updater not found!\r\n");
                            systick::wait_ms(systick::get_tick_ms(), 1500);
                            display_menu(&mut uart);
                        }
                    },
                    b'F' | b'f' => {
                        // Start firmware update
                        clear_screen(&mut uart);
                        uart.send_string("Update firmware using XMODEM - select target:\r\n");
                        uart.send_string("'1' - Updater\r\n");
                        uart.send_string("'2' - Application\r\n");
                        boot_option = BootOption::SelectUpdateTarget;
                    },
                    b'1' => {
                        if boot_option == BootOption::SelectUpdateTarget {
                            // Start updater update
                            clear_screen(&mut uart);
                            uart.send_string("Updating updater...\r\n");
                            uart.send_string("Send file using XMODEM protocol with CRC-16\r\n");
                            firmware_target = UPDATER_ADDR;
                            xmodem.start(firmware_target);
                            update_in_progress = true;
                            boot_option = BootOption::None;
                            
                            leds.set(0, true);  // Green - system alive
                            leds.set(1, true);  // Orange - XMODEM active
                            leds.set(2, false); // Red - no error
                            leds.set(3, false); // Blue - no data received yet
                            
                            // send 'C'
                            if let Some(response) = xmodem.get_response() {
                                uart.send_byte(response);
                            }
                        } else {
                            // invalid option in main menu
                            display_menu(&mut uart);
                        }
                    },
                    b'2' => {
                        if boot_option == BootOption::SelectUpdateTarget {
                            // Start application update
                            clear_screen(&mut uart);
                            uart.send_string("Updating application...\r\n");
                            uart.send_string("Send file using XMODEM protocol with CRC-16\r\n");
                            firmware_target = APP_ADDR;
                            xmodem.start(firmware_target);
                            update_in_progress = true;
                            boot_option = BootOption::None;

                            leds.set(0, true);  // Green - system alive
                            leds.set(1, true);  // Orange - XMODEM active
                            leds.set(2, false); // Red - no error
                            leds.set(3, false); // Blue - no data received yet
                            
                            // send 'C'
                            if let Some(response) = xmodem.get_response() {
                                uart.send_byte(response);
                            }
                        } else {
                            // invalid option in main menu
                            display_menu(&mut uart);
                        }
                    },
                    b'A' | b'a' => {
                        // directly boot application
                        if check_application_valid(&mut uart) {
                            uart.send_string("\r\nBooting application...\r\n");
                            boot_option = BootOption::Application;
                        } else {
                            uart.send_string("\r\nValid application not found!\r\n");
                            systick::wait_ms(systick::get_tick_ms(), 1500);
                            display_menu(&mut uart);
                        }
                    },
                    b'I' => {
                        // diagnostic info
                        clear_screen(&mut uart);
                        uart.send_string("--- System Diagnostics ---\r\n");
                        uart.send_string("App valid: ");
                        uart.send_string(if bootloader::is_firmware_valid(APP_ADDR) { "Yes" } else { "No" });
                        uart.send_string("\r\n");
                        uart.send_string("Updater valid: ");
                        uart.send_string(if bootloader::is_firmware_valid(UPDATER_ADDR) { "Yes" } else { "No" });
                        uart.send_string("\r\n");
                        uart.send_string("Time until autoboot: ");
                        let remaining: u32 = BOOT_TIMEOUT_MS - current_time.wrapping_sub(autoboot_timer);
                        if remaining < BOOT_TIMEOUT_MS {
                            uart.send_string(itoa(remaining));
                        } else {
                            uart.send_string("0");
                        }
                        uart.send_string(" ms\r\n\r\n");
                        
                        uart.send_string("Press any key to return to menu...\r\n");
                        
                        loop {
                            uart.process();
                            if uart.read_byte().is_some() {
                                break;
                            }
                        }
                        
                        display_menu(&mut uart);
                    },
                    b'\r' | b'\n' => {
                        if is_enter_blocked(current_time) {
                            // ignore Enter after update because of ExtraPuTTY sending Enter key
                        } else {
                            if check_application_valid(&mut uart) {
                                uart.send_string("\r\nBooting application...\r\n");
                                boot_option = BootOption::Application;
                            } else {
                                uart.send_string("\r\nValid application not found!\r\n");
                                systick::wait_ms(systick::get_tick_ms(), 1500);
                                display_menu(&mut uart);
                            }
                        }
                    },
                    _ => {
                        if byte != 0 {
                            if boot_option == BootOption::SelectUpdateTarget {
                                clear_screen(&mut uart);
                                uart.send_string("\r\nInvalid option, cancelled.\r\n");
                                boot_option = BootOption::None;
                                systick::wait_ms(systick::get_tick_ms(), 1500);
                                display_menu(&mut uart);
                            } else {
                                // invalid option
                                clear_screen(&mut uart);
                                display_menu(&mut uart);
                            }
                        }
                    },
                }
            }
        } else {
            // handle XMODEM update
            if let Some(byte) = uart.read_byte() {
                // toggle blue LED to show data received
                leds.set(3, !leds.get(3));
                
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
                        // Send ACK for EOT
                        if let Some(response) = xmodem.get_response() {
                            uart.send_byte(response);
                        }

                        update_in_progress = false;
                        clear_rx_buffer(&mut uart);
                        clear_screen(&mut uart);
                        uart.send_string("\r\nTransfer complete! Firmware updated successfully.\r\n\r\n");

                        // reset autoboot timeout
                        autoboot_timer = systick::get_tick_ms();
                        
                        // set Enter lock
                        block_enter_temporarily(systick::get_tick_ms());

                        leds.set_all(true);
                        systick::wait_ms(systick::get_tick_ms(), 500);
                        leds.set_all(false);

                        while !uart.is_tx_complete() {
                            uart.process();
                        }
                        
                        systick::wait_ms(systick::get_tick_ms(), 500);
                        clear_screen(&mut uart);
                        uart.send_string("\r\nUpdate complete! Please select an option:\r\n\r\n");
                        display_menu(&mut uart);
                        while !uart.is_tx_complete() {
                            uart.process();
                        }
                    },
                    Err(XmodemError::Cancelled) => {
                        clear_screen(&mut uart);
                        uart.send_string("\r\nTransfer cancelled.\r\n");
                        clear_rx_buffer(&mut uart);
                        block_enter_temporarily(systick::get_tick_ms());
                        update_in_progress = false;
                        leds.set(2, true);
                        systick::wait_ms(systick::get_tick_ms(), 1500);
                        display_menu(&mut uart);
                    },
                    Err(XmodemError::Timeout) => {
                        clear_screen(&mut uart);
                        uart.send_string("\r\nTransfer timed out.\r\n");
                        clear_rx_buffer(&mut uart);
                        
                        // lock Enter 
                        block_enter_temporarily(systick::get_tick_ms());
                        update_in_progress = false;
                        leds.set(2, true);
                        systick::wait_ms(systick::get_tick_ms(), 1500);
                        display_menu(&mut uart);
                    },
                    Err(XmodemError::SequenceError) | Err(XmodemError::CrcError) => {
                        // XMODEM will handle retries, we just send responses
                        if let Some(response) = xmodem.get_response() {
                            uart.send_byte(response);
                        }
                    },
                    Err(XmodemError::InvalidPacket) => {
                        // XMODEM will handle retries, we just send responses
                        if let Some(response) = xmodem.get_response() {
                            uart.send_byte(response);
                        }
                    },
                    Err(XmodemError::FlashWriteError) => {
                        clear_screen(&mut uart);
                        uart.send_string("\r\nError writing to flash memory.\r\n");
                        clear_rx_buffer(&mut uart);
                        block_enter_temporarily(systick::get_tick_ms());
                        update_in_progress = false;
                        leds.set(2, true);
                        systick::wait_ms(systick::get_tick_ms(), 1500);
                        display_menu(&mut uart);
                    },
                    Err(XmodemError::InvalidMagic) => {
                        clear_screen(&mut uart);
                        uart.send_string("\r\nInvalid firmware image magic number.\r\n");
                        clear_rx_buffer(&mut uart);
                        block_enter_temporarily(systick::get_tick_ms());
                        update_in_progress = false;
                        leds.set(2, true);
                        systick::wait_ms(systick::get_tick_ms(), 1500);
                        display_menu(&mut uart);
                    },
                    Err(XmodemError::OlderVersion) => {
                        clear_screen(&mut uart);
                        uart.send_string("\r\nFirmware version is older than currently installed.\r\n");
                        clear_rx_buffer(&mut uart);
                        block_enter_temporarily(systick::get_tick_ms());
                        update_in_progress = false;
                        leds.set(2, true);
                        systick::wait_ms(systick::get_tick_ms(), 1500);
                        display_menu(&mut uart);
                    }
                }
            }
            
            // check if need to send 'C'
            if xmodem.should_send_byte() {
                if let Some(response) = xmodem.get_response() {
                    uart.send_byte(response);
                }
            }
            
            // Check for errors
            if xmodem.get_state() == XmodemState::Error {
                clear_screen(&mut uart);
                uart.send_string("\r\nXMODEM transfer error. Aborting.\r\n");
                clear_rx_buffer(&mut uart);
                block_enter_temporarily(systick::get_tick_ms());
                update_in_progress = false;
                leds.set(2, true);
                systick::wait_ms(systick::get_tick_ms(), 1500);
                display_menu(&mut uart);
            }
        }

        // Handle boot options
        match boot_option {
            BootOption::Application => {
                if check_application_valid(&mut uart) {
                    // Wait for UART to finish sending
                    while !uart.is_tx_complete() {
                        uart.process();
                    }
                    clear_rx_buffer(&mut uart);
                    boot_application(&p, &mut cp);
                } else {
                    clear_screen(&mut uart);
                    uart.send_string("\r\nApplication validation failed just before boot\r\n");
                    boot_option = BootOption::None;
                    systick::wait_ms(systick::get_tick_ms(), 1500);
                    display_menu(&mut uart);
                }
            },
            BootOption::Updater => {
                // Wait for UART to finish sending
                while !uart.is_tx_complete() {
                    uart.process();
                }
                clear_rx_buffer(&mut uart);
                
                boot_updater(&p, &mut cp);
            },
            _ => {}
        }

        // Check for autoboot timeout
        if !update_in_progress && boot_option == BootOption::None && !is_enter_blocked(current_time) {
            let current_time = systick::get_tick_ms();
            
            if current_time.wrapping_sub(autoboot_timer) >= BOOT_TIMEOUT_MS {
                if check_application_valid(&mut uart) {
                    uart.send_string("\r\nAuto-boot timeout reached. Booting application...\r\n");
                    while !uart.is_tx_complete() {
                        uart.process();
                    }
                    clear_rx_buffer(&mut uart);
                    
                    boot_option = BootOption::Application;
                } else {
                    uart.send_string("\r\nAuto-boot timeout reached but valid application not found!\r\n");
                    
                    // reset autoboot timer
                    autoboot_timer = current_time;
                    systick::wait_ms(systick::get_tick_ms(), 1500);
                    display_menu(&mut uart);
                }
            }
        }
    }
}

fn clear_rx_buffer(uart: &mut UartManager) {
    while uart.read_byte().is_some() {}
}

fn itoa(mut value: u32) -> &'static str {
    static mut BUFFER: [u8; 16] = [0; 16];
    
    if value == 0 {
        return "0";
    }
    
    let mut i: usize = 0;
    unsafe {
        while value > 0 && i < BUFFER.len() {
            BUFFER[i] = b'0' + (value % 10) as u8;
            value /= 10;
            i += 1;
        }
        
        // Reverse the digits
        let mut j: usize = 0;
        let mut k: usize = i - 1;
        while j < k {
            let temp: u8 = BUFFER[j];
            BUFFER[j] = BUFFER[k];
            BUFFER[k] = temp;
            j += 1;
            k -= 1;
        }
        
        BUFFER[i] = 0;
        
        // Convert to string slice
        core::str::from_utf8_unchecked(&BUFFER[0..i])
    }
}

fn enable_peripherals(p: &Peripherals) {
    // Enable GPIO clocks
    p.rcc.ahb1enr().modify(|_, w| {
        w.gpioaen().enabled()
         .gpioden().enabled()
    });
    
    // Enable USART2 clock
    p.rcc.apb1enr().modify(|_, w| {
        w.usart2en().enabled()
    });
    
    // Enable SYSCFG clock
    p.rcc.apb2enr().modify(|_, w| {
        w.syscfgen().enabled()
    });
}

fn setup_gpio_pins(p: &Peripherals) {
    // PA2 = TX, PA3 = RX
    p.gpioa.moder().modify(|_, w| {
        w.moder2().alternate()
         .moder3().alternate()
    });

    p.gpioa.afrl().modify(|_, w| {
        w.afrl2().af7()
         .afrl3().af7()
    });
    
    p.gpioa.ospeedr().modify(|_, w| {
        w.ospeedr2().high_speed()
         .ospeedr3().high_speed()
    });
    
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

    // 90 Mhz
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
        .hpre().div1()
        .ppre1().div4()
        .ppre2().div2()
    );

    // PLL as system clock
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