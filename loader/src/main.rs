#![no_std]
#![no_main]

mod led;
mod uart;

include!(concat!(env!("OUT_DIR"), "/header_values.rs"));

use misc::{
    bootloader::{self, BootOption, BootConfig},
    xmodem::{XmodemManager, XmodemConfig, XmodemError, XmodemState, CAN},
    image::{SharedMemory, IMAGE_MAGIC_LOADER, IMAGE_TYPE_LOADER},
    systick,
};
use core::panic::PanicInfo;
use cortex_m::{asm, peripheral::SYST};
use cortex_m_rt::{entry, exception};
use led::Leds;
use stm32f4 as pac;
use stm32f4::Peripherals;
use uart::{UartManager, UartError};

// Flash memory addresses
pub const UPDATER_ADDR: u32 = 0x08008000;
pub const APP_ADDR: u32 = 0x08020000;
pub const LOADER_ADDR: u32 = 0x08004000;
pub const IMAGE_HDR_SIZE: u32 = 0x200;
pub const BOOT_TIMEOUT_MS: u32 = 10_000;

// Time in which Enter key press is ignored (because 
// ExtraPuTTY sends Enter after XMODEM, but I want to use this key)
const ENTER_BLOCK_AFTER_UPDATE_MS: u32 = 3_000;
static mut ENTER_BLOCKED_UNTIL: u32 = 0;

// Added state for PostXmodem recovery
#[derive(PartialEq, Copy, Clone)]
enum PostXmodemState {
    Initial,
    Recovering,
    Complete
}

#[no_mangle]
#[link_section = ".shared_memory"]
pub static mut SHARED_MEMORY: SharedMemory = SharedMemory::new();

unsafe extern "C" {
    unsafe static __firmware_size: u32;
}

fn calculate_crc32(start_addr: u32, size: u32) -> u32 {
    let mut crc: u32 = 0xFFFFFFFF;
    
    for i in 0..size {
        let byte: u8 = unsafe { *(start_addr as *const u8).offset(i as isize) };
        crc ^= byte as u32;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    !crc
}

// ANSI escape
fn clear_screen(uart: &mut UartManager) {
    uart.send_string("\x1B[2J\x1B[1;1H");

    for _ in 0..5 {
        uart.process();
    }
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
    uart.send_string("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\r\n\r\n");
    uart.send_string("Press 'Enter' to boot application\r\n");
    uart.send_string("Press 'I' to get information about system state\r\n");
    uart.send_string("Press 'F' to update firmware using XMODEM(CRC)\r\n");
    uart.send_string("Press 'U' to enter updater\r\n");
    uart.send_string("Will boot automatically in 10 seconds\r\n");
    
    for _ in 0..10 {
        uart.process();
    }
}

fn check_application_valid(uart: &mut UartManager, boot_config: &BootConfig) -> bool {
    bootloader::is_firmware_valid(APP_ADDR, boot_config)
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

fn send_cancel_sequence(uart: &mut UartManager) {
    // XMODEM cancel sequence
    uart.send_byte(CAN);
    uart.send_byte(CAN);
    uart.send_byte(CAN);

    while !uart.is_tx_complete() {
        uart.process();
    }

    let start_time: u32 = systick::get_tick_ms();
    while !systick::wait_ms(start_time, 1000) {
        uart.process();
    }

    for _ in 0..30 {
        uart.process();
    }
}

fn recover_from_xmodem(uart: &mut UartManager, leds: &mut Leds, block_enter: bool) -> u32 {
    
    clear_rx_buffer(uart);
    let start_time: u32 = systick::get_tick_ms();
    systick::wait_ms(start_time, 3000);
    for _ in 0..50 {
        uart.process();
    }
    
    // clear screen
    clear_screen(uart);
    systick::wait_ms(systick::get_tick_ms(), 500);

    for _ in 0..50 {
        uart.process();
    }
    
    // block '\r' for proper functionality in ExtraPuTTY
    block_enter_temporarily(systick::get_tick_ms());
    
    // reset autoboot
    let new_autoboot_time: u32 = systick::get_tick_ms();
    
    // display main menu
    display_menu(uart);
    for _ in 0..50 {
        uart.process();
    }
    
    while !uart.is_tx_complete() {
        uart.process();
    }
    
    // return new time for timeout
    new_autoboot_time
}

#[entry]
fn main() -> ! {
    let p: Peripherals = pac::Peripherals::take().unwrap();
    let mut cp: cortex_m::Peripherals = cortex_m::Peripherals::take().unwrap();

    // Create configuration objects
    let boot_config: BootConfig = BootConfig {
        app_addr: APP_ADDR,
        updater_addr: UPDATER_ADDR,
        loader_addr: LOADER_ADDR,
        image_hdr_size: IMAGE_HDR_SIZE,
    };
    
    let xmodem_config: XmodemConfig = XmodemConfig {
        app_addr: APP_ADDR,
        updater_addr: UPDATER_ADDR,
        loader_addr: LOADER_ADDR,
        image_hdr_size: IMAGE_HDR_SIZE,
    };

    unsafe {
        // Get firmware size from linker script
        let size: *const u32 = &__firmware_size as *const u32;
        let firmware_size: u32 = *size;
        
        // Update only the size in the header
        IMAGE_HEADER.update_data_size(firmware_size);
        
        // TODO: IMAGE_HEADER.crc
    }

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
    let mut xmodem: XmodemManager = XmodemManager::new(xmodem_config);

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
    let mut post_xmodem_state: PostXmodemState = PostXmodemState::Complete;
    let mut xmodem_error_occurred: bool = false;

    loop {
        // Process UART data
        uart.process();

        // blink green
        let current_time: u32 = systick::get_tick_ms();
        if current_time.wrapping_sub(led_toggle_time) >= 500 {
            leds.toggle(0);
            led_toggle_time = current_time;
        }
        
        // post-update recovery
        if post_xmodem_state == PostXmodemState::Recovering {
            autoboot_timer = recover_from_xmodem(&mut uart, &mut leds, true);
            post_xmodem_state = PostXmodemState::Complete;
            update_in_progress = false;
            boot_option = BootOption::None;
            continue;
        }

        // read keys if not in update mode
        if !update_in_progress {
            if let Some(byte) = uart.read_byte() {
                // if user presses anything reset the autoboot
                autoboot_timer = current_time;
                
                match byte {
                    b'U' | b'u' => {
                        // updater
                        if bootloader::is_firmware_valid(UPDATER_ADDR, &boot_config) {
                            uart.send_string("\r\n Booting updater...\r\n");
                            boot_option = BootOption::Updater;
                        } else {
                            uart.send_string("\r\nValid updater not found!\r\n");
                            systick::wait_ms(systick::get_tick_ms(), 1500);
                            display_menu(&mut uart);
                        }
                    },
                    b'F' | b'f' => {
                        // start update
                        clear_screen(&mut uart);
                        uart.send_string("\r\nUpdate firmware using XMODEM - select target:\r\n");
                        uart.send_string("'1' - Updater\r\n");
                        uart.send_string("'2' - Application\r\n");
                        boot_option = BootOption::SelectUpdateTarget;
                    },
                    b'1' => {
                        if boot_option == BootOption::SelectUpdateTarget {
                            // start updater update
                            clear_screen(&mut uart);
                            uart.send_string("Updating updater...\r\n");
                            uart.send_string("Send file using XMODEM protocol with CRC-16. \r\nIf menu doesn't load after update is over, please press 'Esc'\r\n");
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
                            // start application update
                            clear_screen(&mut uart);
                            uart.send_string("Updating application...\r\n");
                            uart.send_string("Send file using XMODEM protocol with CRC-16.\r\nIf menu doesn't load after update is over, please press 'Esc'\r\n");
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

                    b'I' | b'i' => {
                        // diagnostic info
                        clear_screen(&mut uart);
                        uart.send_string("\r\n--- System Information ---\r\n\r\n");
                        
                        // loader
                        uart.send_string("Loader (this image) : ");

                        if let Some(header) = bootloader::get_firmware_header(LOADER_ADDR, &boot_config) {
                            uart.send_string("Valid\r\n");
                            
                            uart.send_string("  Version: ");
                            uart.send_string(itoa(header.version_major as u32));
                            uart.send_string(".");
                            uart.send_string(itoa(header.version_minor as u32));
                            uart.send_string(".");
                            uart.send_string(itoa(header.version_patch as u32));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Vector table: 0x");
                            uart.send_string(to_hex(header.vector_addr));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Data size: ");
                            uart.send_string(itoa(header.data_size));
                            uart.send_string(" bytes\r\n");
                            
                            uart.send_string("  CRC: 0x");
                            uart.send_string(to_hex(header.crc));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Address: 0x");
                            uart.send_string(to_hex(LOADER_ADDR));
                            uart.send_string("\r\n");
                        } else {
                            uart.send_string("Invalid or Not Found\r\n");
                        }
                        
                        uart.send_string("\r\n");
                        
                        // app
                        uart.send_string("Application: ");
                        if let Some(header) = bootloader::get_firmware_header(APP_ADDR, &boot_config) {
                            uart.send_string("Valid\r\n");
                            
                            uart.send_string("  Version: ");
                            uart.send_string(itoa(header.version_major as u32));
                            uart.send_string(".");
                            uart.send_string(itoa(header.version_minor as u32));
                            uart.send_string(".");
                            uart.send_string(itoa(header.version_patch as u32));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Vector table: 0x");
                            uart.send_string(to_hex(header.vector_addr));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Data size: ");
                            uart.send_string(itoa(header.data_size));
                            uart.send_string(" bytes\r\n");
                            
                            uart.send_string("  CRC: 0x");
                            uart.send_string(to_hex(header.crc));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Address: 0x");
                            uart.send_string(to_hex(APP_ADDR));
                            uart.send_string("\r\n");
                        } else {
                            uart.send_string("Invalid or Not Found\r\n");
                        }
                        
                        uart.send_string("\r\n");
                        
                        // updater
                        uart.send_string("Updater: ");
                        if let Some(header) = bootloader::get_firmware_header(UPDATER_ADDR, &boot_config) {
                            uart.send_string("Valid\r\n");
                            
                            uart.send_string("  Version: ");
                            uart.send_string(itoa(header.version_major as u32));
                            uart.send_string(".");
                            uart.send_string(itoa(header.version_minor as u32));
                            uart.send_string(".");
                            uart.send_string(itoa(header.version_patch as u32));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Vector table: 0x");
                            uart.send_string(to_hex(header.vector_addr));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Data size: ");
                            uart.send_string(itoa(header.data_size));
                            uart.send_string(" bytes\r\n");
                            
                            uart.send_string("  CRC: 0x");
                            uart.send_string(to_hex(header.crc));
                            uart.send_string("\r\n");
                            
                            uart.send_string("  Address: 0x");
                            uart.send_string(to_hex(UPDATER_ADDR));
                            uart.send_string("\r\n");
                        } else {
                            uart.send_string("Invalid or Not Found\r\n");
                        }
                        
                        uart.send_string("\r\n");
                        uart.send_string("System Info:\r\n");
                        uart.send_string("  Boot timeout: ");
                        uart.send_string(itoa(BOOT_TIMEOUT_MS / 1000));
                        uart.send_string(" seconds\r\n");
                        uart.send_string("  System uptime: ");
                        uart.send_string(itoa(systick::get_tick_ms() / 1000));
                        uart.send_string(" seconds\r\n");
                        
                        uart.send_string("\r\nPress 'Escape' to return to menu...\r\n");
                        
                        loop {
                            uart.process();
                            if let Some(key) = uart.read_byte() {
                                if key == b'\x1B' {
                                    autoboot_timer = systick::get_tick_ms();
                                    break;
                                }
                                autoboot_timer = systick::get_tick_ms();
                            }
                        }
                        
                        display_menu(&mut uart);
                    },
                    b'\r' | b'\n' => {
                        if is_enter_blocked(current_time) {
                            // ignore Enter after update because of ExtraPuTTY sending '\r'
                        } else {
                            if check_application_valid(&mut uart, &boot_config) {
                                uart.send_string("\r\n Booting application...\r\n");
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
                                boot_option = BootOption::None;
                            } 
                            if byte == b'\x1B' {
                                clear_screen(&mut uart);
                                display_menu(&mut uart);
                            } else {
                                // invalid option
                            }
                        }
                    },
                }
            }
        } else {
            // handle XMODEM update
            if let Some(byte) = uart.read_byte() {
                
                match xmodem.process_byte(byte) {
                    Ok(true) => {
                        // response required
                        if let Some(response) = xmodem.get_response() {
                            uart.send_byte(response);
                        }
                    },
                    Ok(false) => {
                        // no response needed
                    },
                    Err(XmodemError::TransferComplete) => {
                        // send ACK for EOT
                        if let Some(response) = xmodem.get_response() {
                            uart.send_byte(response);
                        }
                    
                        update_in_progress = false;
                        xmodem_error_occurred = false;

                        post_xmodem_state = PostXmodemState::Recovering;
                    },
                    Err(XmodemError::Cancelled) => {
                        // abortion
                        uart.send_string("\r\nTransfer cancelled.\r\n");
                        
                        while !uart.is_tx_complete() {
                            uart.process();
                        }
                        
                        xmodem_error_occurred = true;
                        post_xmodem_state = PostXmodemState::Recovering;
                    },
                    Err(XmodemError::Timeout) => {
                        // timeout
                        uart.send_string("\r\nTransfer timed out.\r\n");
                        
                        while !uart.is_tx_complete() {
                            uart.process();
                        }
                        
                        xmodem_error_occurred = true;
                        leds.set(2, true);
                        post_xmodem_state = PostXmodemState::Recovering;
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
                        xmodem.cancel_transfer();
                        send_cancel_sequence(&mut uart);

                        // flash error processing
                        uart.send_string("\r\nError writing to flash memory.\r\n");
                        
                        while !uart.is_tx_complete() {
                            uart.process();
                        }
                        
                        xmodem_error_occurred = true;
                        leds.set(2, true);
                        post_xmodem_state = PostXmodemState::Recovering;
                    },
                    Err(XmodemError::InvalidMagic) => {
                        xmodem.cancel_transfer();
                        send_cancel_sequence(&mut uart);
                        
                        // invalid magic
                        uart.send_string("\r\nInvalid firmware image magic number.\r\n");
                        
                        while !uart.is_tx_complete() {
                            uart.process();
                        }
                        
                        xmodem_error_occurred = true;
                        leds.set(2, true);
                        post_xmodem_state = PostXmodemState::Recovering;
                    },
                    Err(XmodemError::OlderVersion) => {
                        xmodem.cancel_transfer();
                        send_cancel_sequence(&mut uart);
                        
                        xmodem_error_occurred = true;
                        post_xmodem_state = PostXmodemState::Recovering;
                    }
                }
            }
            
            // check if need to send 'C'
            if xmodem.should_send_byte() {
                if let Some(response) = xmodem.get_response() {
                    uart.send_byte(response);
                }
            }
            
            // check for errors
            if xmodem.get_state() == XmodemState::Error {
                // general error processing
                uart.send_string("\r\nXMODEM transfer error. Aborting.\r\n");
                
                while !uart.is_tx_complete() {
                    uart.process();
                }
                
                xmodem_error_occurred = true;

                post_xmodem_state = PostXmodemState::Recovering;
            }
        }

        // Handle boot options
        match boot_option {
            BootOption::Application => {
                if check_application_valid(&mut uart, &boot_config) {
                    // wait for UART to finish sending
                    while !uart.is_tx_complete() {
                        uart.process();
                    }
                    clear_rx_buffer(&mut uart);
                    bootloader::boot_application(&p, &mut cp, &boot_config);
                } else {
                    clear_screen(&mut uart);
                    uart.send_string("\r\nApplication validation failed just before boot\r\n");
                    boot_option = BootOption::None;
                    systick::wait_ms(systick::get_tick_ms(), 1500);
                    display_menu(&mut uart);
                }
            },
            BootOption::Updater => {
                // wait for UART to finish sending
                while !uart.is_tx_complete() {
                    uart.process();
                }
                clear_rx_buffer(&mut uart);
                
                bootloader::boot_updater(&p, &mut cp, &boot_config);
            },
            _ => {}
        }

        // check for boot timeout
        if !update_in_progress && boot_option == BootOption::None && 
           !is_enter_blocked(current_time) && post_xmodem_state == PostXmodemState::Complete {
            let current_time: u32 = systick::get_tick_ms();
            
            if current_time.wrapping_sub(autoboot_timer) >= BOOT_TIMEOUT_MS {
                if check_application_valid(&mut uart, &boot_config) {
                    uart.send_string("\r\n Auto-boot timeout reached. Booting application...\r\n");
                    
                    for _ in 0..5 {
                        uart.process();
                    }
                    
                    while !uart.is_tx_complete() {
                        uart.process();
                    }
                    clear_rx_buffer(&mut uart);
                    
                    boot_option = BootOption::Application;
                } else {
                    uart.send_string("\r\n Auto-boot timeout reached but valid application not found!\r\n");
                    
                    for _ in 0..5 {
                        uart.process();
                    }
                    
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
        
        // convert to string slice
        core::str::from_utf8_unchecked(&BUFFER[0..i])
    }
}

fn to_hex(mut value: u32) -> &'static str {
    static mut BUFFER: [u8; 16] = [0; 16];
    
    if value == 0 {
        return "0";
    }
    
    let mut i: usize = 0;
    unsafe {
        while value > 0 && i < BUFFER.len() {
            let digit: u32 = value % 16;
            BUFFER[i] = if digit < 10 {
                b'0' + digit as u8
            } else {
                b'a' + (digit - 10) as u8
            };
            value /= 16;
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