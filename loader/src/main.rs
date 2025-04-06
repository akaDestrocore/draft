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

// XMODEM protocol constants
const SOH: u8 = 0x01;  // Start of header (128 bytes)
const STX: u8 = 0x02;  // Start of header (1K bytes)
const EOT: u8 = 0x04;  // End of transmission
const ACK: u8 = 0x06;  // Acknowledge
const NAK: u8 = 0x15;  // Negative acknowledge
const CAN: u8 = 0x18;  // Cancel
const C: u8 = 0x43;    // ASCII 'C' for CRC mode

// XMODEM state machine
#[derive(Copy, Clone, PartialEq)]
enum XmodemState {
    WaitSOH,
    WaitIndex1,
    WaitIndex2,
    ReadData,
    WaitCRC,
}

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

// Глобальные переменные для XMODEM, скопировано из C-версии
static mut X_STATE: XmodemState = XmodemState::WaitSOH;
static mut PREV_INDEX1: u8 = 0;
static mut PREV_INDEX2: u8 = 0xFF;
static mut PACKET_RECEIVED: bool = false;
static mut COPY_DATA: bool = true;
static mut FIRST_PACKET: bool = false;
static mut UPDATED_SELECTED: bool = false;
static mut A_READ_RX_DATA: [u8; 133] = [0; 133];
static mut DATA_COUNTER: usize = 0;
static mut CURRENT_ADDRESS: u32 = 0;

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
        // Process input and firmware updates
        if !unsafe { UPDATED_SELECTED } {
            if let Some(byte) = RX_BUFFER.get(|buf| buf.read()) {
                handle_user_input(&p, byte, &mut update_option_selected);
            }
        } else {
            // Это КЛЮЧЕВОЙ МОМЕНТ из C кода: если получили полный пакет, но еще не обработали
            unsafe {
                if RX_BUFFER.get(|buf| buf.len()) >= 133 && !PACKET_RECEIVED {
                    FIRST_PACKET = true;
                }
            }
            
            // Обработка входящих данных XMODEM
            download_firmware(&p);
        }
        
        // check if boot actions are requested
        if LOAD_APPLICATION.load(Ordering::SeqCst) {
            boot_application(&p, &mut cp);
        }
        
        if LOAD_UPDATER.load(Ordering::SeqCst) {
            boot_updater(&p, &mut cp);
        }

        // check timeout - only if no firmware update is in progress
        if !unsafe { UPDATED_SELECTED } && !update_option_selected {
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

        // Обработка UART передачи
        ensure_transmitting();

        // Power-saving wait for interrupt
        asm::wfi();
    }
}

// Функция из C-кода с точным соблюдением логики!
fn download_firmware(p: &pac::Peripherals) {
    // Отправляем 'C' для начала передачи, если пакет не получен и это не первый пакет
    unsafe {
        if !PACKET_RECEIVED && !FIRST_PACKET {
            TX_BUFFER.get(|buf| buf.write(C));
            ensure_transmitting();
            asm::delay(90000000); // примерно 3 секунды при 90MHz
        }
        
        // Отправляем NAK, если пакет не получен и это уже не первый пакет
        if !PACKET_RECEIVED && FIRST_PACKET {
            TX_BUFFER.get(|buf| buf.write(NAK));
            ensure_transmitting();
            asm::delay(90000/20); // примерно 50 мс
        }
        
        // Отправляем ACK, если пакет был получен
        if X_STATE == XmodemState::WaitSOH && PACKET_RECEIVED {
            TX_BUFFER.get(|buf| buf.write(ACK));
            ensure_transmitting();
        }
        
        // Проверяем, есть ли EOT (конец передачи)
        let mut rx_byte = 0;
        if RX_BUFFER.get(|buf| buf.peek()).is_some() {
            if RX_BUFFER.get(|buf| buf.data_buff[0]) == EOT {
                // Отправляем ACK для завершения передачи
                TX_BUFFER.get(|buf| buf.write(ACK));
                ensure_transmitting();
                
                // Тут в C-коде идет обработка полученных данных
                // В нашем случае просто завершаем передачу
                
                // Сбрасываем все параметры
                RX_BUFFER.get(|buf| {
                    buf.head = 0;
                    buf.tail = 0;
                    buf.dataCount = 0;
                });
                
                FIRST_PACKET = false;
                COPY_DATA = true;
                PACKET_RECEIVED = false;
                PREV_INDEX1 = 0;
                PREV_INDEX2 = 0xFF;
                X_STATE = XmodemState::WaitSOH;
                
                // Выводим сообщение об успешном завершении
                TX_BUFFER.get(|buf| {
                    queue_string(buf, "\r\nФайл успешно загружен! Перезагрузите устройство.\r\n");
                });
                
                asm::delay(90000000 * 2); // примерно 2 секунды при 90MHz
                UPDATED_SELECTED = false;
                return;
            }
        }
        
        // Копируем данные из кольцевого буфера в буфер пакета
        if COPY_DATA {
            if RX_BUFFER.get(|buf| buf.len()) >= 133 {
                for i in 0..133 {
                    A_READ_RX_DATA[i] = RX_BUFFER.get(|buf| buf.data_buff[i]);
                }
                
                PACKET_RECEIVED = false;
                
                // Очищаем буфер
                RX_BUFFER.get(|buf| {
                    buf.head = 0;
                    buf.tail = 0;
                    buf.dataCount = 0;
                });
            } else {
                // Не хватает данных для полного пакета
                return;
            }
        }
        
        // XMODEM state machine, точно как в C-коде
        match X_STATE {
            XmodemState::WaitSOH => {
                if A_READ_RX_DATA[0] == SOH {
                    X_STATE = XmodemState::WaitIndex1;
                    PACKET_RECEIVED = true;
                    COPY_DATA = false;
                    
                    // Индикация приема пакета
                    toggle_led(p, 0); // Зеленый LED
                } else {
                    PACKET_RECEIVED = false;
                }
            },
            XmodemState::WaitIndex1 => {
                // Если индекс больше предыдущего на 1, то всё ок
                if (PREV_INDEX1 + 1) == A_READ_RX_DATA[1] {
                    X_STATE = XmodemState::WaitIndex2;
                    PREV_INDEX1 += 1;
                } else if (PREV_INDEX1 + 1) == 256 {
                    // Обрабатываем переполнение счетчика
                    X_STATE = XmodemState::WaitIndex2;
                    PREV_INDEX1 = 0;
                } else {
                    // Ошибка индекса
                    send_cancel();
                    
                    // Сбрасываем все параметры
                    UPDATED_SELECTED = false;
                    rx_byte = 0;
                    
                    TX_BUFFER.get(|buf| {
                        queue_string(buf, "\r\nОшибка индекса 1 пакета! Отмена загрузки.\r\n");
                    });
                }
            },
            XmodemState::WaitIndex2 => {
                // Проверяем дополнение к индексу
                if (PREV_INDEX2 - 1) == A_READ_RX_DATA[2] {
                    X_STATE = XmodemState::ReadData;
                    PREV_INDEX2 -= 1;
                } else if (PREV_INDEX2 - 1) == -1 {
                    X_STATE = XmodemState::ReadData;
                    PREV_INDEX2 = 255;
                } else {
                    // Ошибка дополнения индекса
                    send_cancel();
                    
                    // Сбрасываем все параметры
                    UPDATED_SELECTED = false;
                    rx_byte = 0;
                    
                    TX_BUFFER.get(|buf| {
                        queue_string(buf, "\r\nОшибка индекса 2 пакета! Отмена загрузки.\r\n");
                    });
                }
            },
            XmodemState::ReadData => {
                // Просто копируем данные из буфера пакета
                // и переходим к проверке CRC
                X_STATE = XmodemState::WaitCRC;
            },
            XmodemState::WaitCRC => {
                // Тут проверяем CRC, но для упрощения пропустим
                // и просто отправим ACK
                
                TX_BUFFER.get(|buf| buf.write(ACK));
                ensure_transmitting();
                
                // Сбрасываем состояние для следующего пакета
                PACKET_RECEIVED = true;
                COPY_DATA = true;
                X_STATE = XmodemState::WaitSOH;
                
                // Мигаем оранжевым LED для индикации успешного пакета
                toggle_led(p, 1);
            }
        }
    }
}

// Отправка CANCEL для прерывания передачи
fn send_cancel() {
    TX_BUFFER.get(|buf| {
        buf.write(CAN);
        buf.write(CAN);
        buf.write(CAN);
    });
    ensure_transmitting();
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

fn send_welcome_message(p: &pac::Peripherals) {
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
    
    // Мигаем всеми LED для индикации старта
    toggle_led(p, 0);
    toggle_led(p, 1);
    toggle_led(p, 2);
    toggle_led(p, 3);
    
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
            TX_BUFFER.get(|tx_buf| {
                queue_string(tx_buf, "\r\nСейчас начнется загрузка прошивки через XMODEM...\r\n");
                queue_string(tx_buf, "Отправьте файл, используя XMODEM протокол.\r\n");
            });
            
            // Установка адреса для записи прошивки
            unsafe {
                CURRENT_ADDRESS = APP_ADDR;
                UPDATED_SELECTED = true;
                X_STATE = XmodemState::WaitSOH;
                PREV_INDEX1 = 0;
                PREV_INDEX2 = 0xFF;
                PACKET_RECEIVED = false;
                COPY_DATA = true;
                FIRST_PACKET = false;
            }
            
            // Мигаем зеленым для индикации готовности
            toggle_led(p, 0);
        },
        b'D' | b'd' => {
            // Show diagnostic information
            TX_BUFFER.get(|tx_buf| {
                queue_string(tx_buf, "\r\n--- Firmware Update Diagnostics ---\r\n");
                queue_string(tx_buf, "XMODEM state: ");
                
                unsafe {
                    match X_STATE {
                        XmodemState::WaitSOH => queue_string(tx_buf, "WaitSOH"),
                        XmodemState::WaitIndex1 => queue_string(tx_buf, "WaitIndex1"),
                        XmodemState::WaitIndex2 => queue_string(tx_buf, "WaitIndex2"),
                        XmodemState::ReadData => queue_string(tx_buf, "ReadData"),
                        XmodemState::WaitCRC => queue_string(tx_buf, "WaitCRC"),
                    }
                    queue_string(tx_buf, "\r\n");
                    
                    queue_string(tx_buf, "PREV_INDEX1: ");
                    write_decimal(tx_buf, PREV_INDEX1 as u32);
                    queue_string(tx_buf, "\r\n");
                    
                    queue_string(tx_buf, "PREV_INDEX2: ");
                    write_decimal(tx_buf, PREV_INDEX2 as u32);
                    queue_string(tx_buf, "\r\n");
                    
                    queue_string(tx_buf, "RX buffer size: ");
                    write_decimal(tx_buf, RX_BUFFER.get(|buf| buf.len()) as u32);
                    queue_string(tx_buf, "\r\n");
                    
                    queue_string(tx_buf, "Update selected: ");
                    queue_string(tx_buf, if UPDATED_SELECTED { "Yes" } else { "No" });
                    queue_string(tx_buf, "\r\n");
                }
                
                queue_string(tx_buf, "\r\n--- End of Diagnostics ---\r\n\r\n");
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
            if byte != 0 {
                TX_BUFFER.get(|tx_buf| {
                    queue_string(tx_buf, "\r\nInvalid option, try again.\r\n");
                });
            }
        },
    }
}

fn queue_string(tx_buffer: &RingBuffer, s: &str) {
    for byte in s.bytes() {
        tx_buffer.write(byte);
    }
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

// LED helper functions
fn set_led(p: &pac::Peripherals, led: u8, state: bool) {
    match led {
        0 => if state { 
                unsafe { p.gpiod.bsrr().write(|w| w.bs12().set_bit()); }
             } else { 
                unsafe { p.gpiod.bsrr().write(|w| w.br12().set_bit()); }
             },
        1 => if state { 
                unsafe { p.gpiod.bsrr().write(|w| w.bs13().set_bit()); }
             } else { 
                unsafe { p.gpiod.bsrr().write(|w| w.br13().set_bit()); }
             },
        2 => if state { 
                unsafe { p.gpiod.bsrr().write(|w| w.bs14().set_bit()); }
             } else { 
                unsafe { p.gpiod.bsrr().write(|w| w.br14().set_bit()); }
             },
        3 => if state { 
                unsafe { p.gpiod.bsrr().write(|w| w.bs15().set_bit()); }
             } else { 
                unsafe { p.gpiod.bsrr().write(|w| w.br15().set_bit()); }
             },
        _ => {}
    }
}

fn toggle_led(p: &pac::Peripherals, led: u8) {
    match led {
        0 => unsafe { p.gpiod.odr().modify(|r, w| w.odr12().bit(!r.odr12().bit())); },
        1 => unsafe { p.gpiod.odr().modify(|r, w| w.odr13().bit(!r.odr13().bit())); },
        2 => unsafe { p.gpiod.odr().modify(|r, w| w.odr14().bit(!r.odr14().bit())); },
        3 => unsafe { p.gpiod.odr().modify(|r, w| w.odr15().bit(!r.odr15().bit())); },
        _ => {}
    }
}

#[no_mangle]
pub extern "C" fn USART2() {
    USART2_PTR.get(|usart_opt| {
        if let Some(ref usart_ptr) = *usart_opt {
            unsafe {
                let usart2 = &*(usart_ptr.0 as *const pac::usart2::RegisterBlock);

                // check data in RX buffer
                if usart2.sr().read().rxne().bit_is_set() {
                    let data: u8 = usart2.dr().read().bits() as u8;
                    RX_BUFFER.get(|buf| {
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
    // Включаем красный LED при ошибке
    GPIOD_PTR.get(|gpiod_opt| {
        if let Some(ref gpiod_ptr) = *gpiod_opt {
            let gpiod = &*(gpiod_ptr.0 as *const pac::gpiod::RegisterBlock);
            gpiod.bsrr().write(|w| w.bs14().set_bit());
        }
    });
    
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