#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::{asm, peripheral::{SCB, SYST}, register::{msp, psp}};
use cortex_m_rt::{entry, interrupt};
use stm32f4_pac::{self as pac, usart1};
use misc::RingBuffer;

// Константы адресов памяти
const SLOT_2_APP_ADDR: u32 = 0x08010000; // Адрес приложения
const UPDATER_ADDR: u32 = 0x08008000;    // Адрес обновлятора
const SLOT_2_VER_ADDR: u32 = 0x08010100; // Адрес версии приложения

// Глобальные статические переменные
static mut RX_BUFFER: RingBuffer = RingBuffer::new();
static mut TX_BUFFER: RingBuffer = RingBuffer::new();
static mut RX_BYTE: u8 = 0;
static mut TX_IN_PROGRESS: bool = false;
static mut LOAD_APPLICATION: bool = false;
static mut BOOT_TIMEOUT: u32 = 10000; // 10 секунд
static mut START_TICK: u32 = 0;

#[entry]
fn main() -> ! {
    // Получаем доступ к периферии
    let p = unsafe { pac::Peripherals::steal() };
    let cp = unsafe { cortex_m::Peripherals::steal() };

    // Настройка системных часов
    system_clock_config(&p);
    
    // Инициализация GPIO
    mx_gpio_init(&p);
    
    // Инициализация USART2
    mx_usart2_uart_init(&p);
    
    // Включение прерывания USART2
    p.usart2.cr1().modify(|_, w| w.rxneie().set_bit() );


    // Вывод приветственных сообщений
    show_message(MessageType::PrintHello);
    show_message(MessageType::PrintOptions);
    
    // Запуск таймера автозагрузки
    unsafe {
        START_TICK = cp.SYST.cvr.read();
    }

    // Главный цикл
    loop {
        read_key();
    }
}

// Типы сообщений
enum MessageType {
    PrintHello,
    PrintOptions,
    PrintSelectionErr,
    PrintBootFirmware,
    PrintBootUpdater,
}

// Настройка системных часов
fn system_clock_config(p: &pac::Peripherals) {
    unsafe {
        let rcc = &p.rcc;
        let pwr = &p.pwr;
        let flash = &p.flash;

        // Включение PWR
        rcc.apb1enr().modify(|_, w| w.pwren().set_bit());
        
        // Настройка регулятора напряжения
        pwr.cr().modify(|r, w| {
            let bits: u32 = r.bits() | (1 << 14);
            w.bits(bits)
        });

        // Включение HSE
        rcc.cr().modify(|_, w| w.hseon().set_bit());
        while rcc.cr().read().hserdy().bit_is_clear() {}
        
        // Настройка PLL
        rcc.pllcfgr().modify(|_, w| {
            w.pllsrc().hse()
             .pllm().bits(4)
             .plln().bits(90)
             .pllp().div2()
             .pllq().bits(4)
        });

        // Включение PLL
        rcc.cr().modify(|_, w| w.pllon().set_bit());
        while rcc.cr().read().pllrdy().bit_is_clear() {}
        
        // Настройка делителей шин
        rcc.cfgr().modify(|_, w| {
            w.hpre().div1()
             .ppre1().div4()
             .ppre2().div2()
        });

        // Настройка задержек Flash
        flash.acr().modify(|_, w| {
            w.latency().bits(2)
             .prften().set_bit()
             .icen().set_bit()
             .dcen().set_bit()
        });

        // Переключение на PLL
        rcc.cfgr().modify(|_, w| w.sw().pll());
        while !rcc.cfgr().read().sws().is_pll() {}
    }
}

// Инициализация GPIO
fn mx_gpio_init(p: &pac::Peripherals) {
    unsafe {
        let rcc = &p.rcc;
        let gpioa = &p.gpioa;
        let gpiod = &p.gpiod;
        
        // Включение тактирования GPIO
        rcc.ahb1enr().modify(|_, w| {
            w.gpioaen().set_bit()
             .gpioden().set_bit()
        });
        
        // Настройка кнопки USER (PA0)
        gpioa.moder().modify(|_, w| w.moder0().input());
        gpioa.pupdr().modify(|_, w| w.pupdr0().pull_down());
        
        // Настройка светодиодов (PD12-PD15)
        gpiod.moder().modify(|_, w| {
            w.moder12().output()
             .moder13().output()
             .moder14().output()
             .moder15().output()
        });
        
        gpiod.otyper().modify(|_, w| {
            w.ot12().push_pull()
             .ot13().push_pull()
             .ot14().push_pull()
             .ot15().push_pull()
        });
        
        gpiod.ospeedr().modify(|_, w| {
            w.ospeedr12().very_high_speed()
             .ospeedr13().very_high_speed()
             .ospeedr14().very_high_speed()
             .ospeedr15().very_high_speed()
        });
        
        gpiod.pupdr().modify(|_, w| {
            w.pupdr12().pull_down()
             .pupdr13().pull_down()
             .pupdr14().pull_down()
             .pupdr15().pull_down()
        });
        
        // Выключение всех светодиодов
        gpiod.bsrr().write(|w| {
            w.br12().reset()
             .br13().reset()
             .br14().reset()
             .br15().reset()
        });
    }
}

// Инициализация UART
fn mx_usart2_uart_init(p: &pac::Peripherals) {
    unsafe {
        let rcc = &p.rcc;
        let gpioa = &p.gpioa;
        let usart2 = &p.usart2;
        
        // Включение тактирования USART2
        rcc.apb1enr().modify(|_, w| w.usart2en().set_bit());
        
        // Настройка пинов USART2 (PA2=TX, PA3=RX)
        gpioa.moder().modify(|_, w| {
            w.moder2().alternate()
             .moder3().alternate()
        });
        
        gpioa.afrl().modify(|_, w| {
            w.afrl2().af7()
             .afrl3().af7()
        });
        
        // Настройка USART2
        // 90 МГц / 4 = 22.5 МГц, для 115200 бод: 22500000/115200 = 195.3125 ≈ 195
        usart2.brr().write(|w| w.bits(195));
        
        // Включение USART2
        usart2.cr1().modify(|_, w| {
            w.ue().set_bit()
             .te().set_bit()
             .re().set_bit()
             .rxneie().set_bit()
        });
    }
}

// Отображение сообщений
fn show_message(msg_type: MessageType) {
    match msg_type {
        MessageType::PrintHello => {
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
        MessageType::PrintOptions => {
            send_string("Booting into application in 10 seconds.\r\n\r\n");
            send_string("Select an option:\r\n\r\n");
            send_string(" > 'SHIFT+U' --> Go to UPDATER\r\n\r\n");
            send_string(" > 'ENTER'   --> Regular boot\r\n");
        },
        MessageType::PrintSelectionErr => {
            send_string("Please select from available options.\r\n");
        },
        MessageType::PrintBootFirmware => {
            send_string("\r\n Booting into user application...\r\n");
        },
        MessageType::PrintBootUpdater => {
            send_string("\r\n Loading updater...\r\n");
        },
    }
}

// Отправка строки через UART
fn send_string(s: &str) {
    for byte in s.bytes() {
        unsafe {
            // Добавляем байт в буфер
            TX_BUFFER.write(byte);
        }
    }
    
    // Запускаем передачу, если она еще не запущена
    ring_buffer_transmit();
    
    // Небольшая задержка в 10 мс
    cortex_m::asm::delay(10000);
}

// Передача данных из кольцевого буфера
fn ring_buffer_transmit() {
    unsafe {
        let p: pac::Peripherals = unsafe { pac::Peripherals::steal() };

        if !TX_IN_PROGRESS {
            if let Some(temp) = TX_BUFFER.read() {
                // Получаем указатель на USART2
                let usart2 = &p.usart2;
                
                // Отправляем байт
                usart2.dr().write(|w| w.bits(temp as u32));
                
                // Включаем прерывание TX empty
                usart2.cr1().modify(|_, w| w.txeie().set_bit());
                
                TX_IN_PROGRESS = true;
            } else {
                // Если буфер пуст, отключаем прерывание TX empty
                let usart2 = &p.usart2;
                usart2.cr1().modify(|_, w| w.txeie().clear_bit());
            }
        }
    }
}

// Чтение входных данных и обработка команд
fn read_key() {
    unsafe {
        // Пытаемся получить байт из буфера приема
        if RX_BYTE == 0 {
            if let Some(byte) = RX_BUFFER.read() {
                RX_BYTE = byte;
            } else {
                // Проверяем таймаут
                let cp = cortex_m::Peripherals::steal();
                let current_tick = cp.SYST.cvr.read();
                if current_tick.wrapping_sub(START_TICK) >= BOOT_TIMEOUT {
                    jump_to_user_application();
                }
                return;
            }
        }
        
        // Обработка полученного байта
        match RX_BYTE {
            b'U' => {
                // Загрузка обновлятора
                show_message(MessageType::PrintBootUpdater);
                jump_to_updater();
            },
            b'\r' => {
                // Проверка наличия приложения
                if *(SLOT_2_VER_ADDR as *const u32) == 0xFFFFFFFF {
                    send_string("There is no user application!\r\n");
                    RX_BYTE = 0;
                } else {
                    show_message(MessageType::PrintBootFirmware);
                    LOAD_APPLICATION = true;
                }
            },
            _ => {
                // Неверный ввод
                show_message(MessageType::PrintSelectionErr);
                RX_BYTE = 0;
            }
        }
        
        // Проверяем флаг загрузки
        if LOAD_APPLICATION {
            jump_to_user_application();
        }
    }
}

// Переход к пользовательскому приложению
fn jump_to_user_application() -> ! {
    unsafe {
        send_string("\r\nLoading user application now.\r\n\r\n");
        
        // Адрес обработчика сброса
        let reset_addr = SLOT_2_APP_ADDR + 4;
        let reset_ptr = *(reset_addr as *const u32);
        
        // Деинициализация RCC
        let p = pac::Peripherals::steal();
        
        p.rcc.cr().modify(|_, w| w
            .hsion().set_bit()
            .hseon().clear_bit()
            .pllon().clear_bit()
        );
        
        while p.rcc.cr().read().hsirdy().bit_is_clear() {}
        
        p.rcc.cfgr().modify(|_, w| w.sw().hsi());
        while !p.rcc.cfgr().read().sws().is_hsi() {}
        
        p.rcc.cfgr().reset();
        
        // Рэмапинг памяти
        p.syscfg.memrm().modify(|_, w| w.mem_mode().bits(0x01));
        
        // Отключение SysTick
        let mut cp = cortex_m::Peripherals::steal();
        let systick: &mut SYST = &mut cp.SYST;
        systick.disable_counter();
        systick.disable_interrupt();
        
        // Очистка PendSV
        let scb = SCB::ptr();
        (*scb).icsr.write(1 << 27);
        
        // Отключение обработчиков ошибок SCB
        (*scb).shcsr.modify(|v| v & !(
            (1 << 18) | // USGFAULTENA
            (1 << 17) | // BUSFAULTENA
            (1 << 16)   // MEMFAULTENA
        ));
        
        // Установка таблицы векторов
        (*scb).vtor.write(SLOT_2_APP_ADDR);
        
        // Установка указателя стека
        let stack_ptr = *(SLOT_2_APP_ADDR as *const u32);
        msp::write(stack_ptr);
        psp::write(stack_ptr);
        
        // Переход к приложению
        let jump_fn: fn() -> ! = core::mem::transmute(reset_ptr);
        jump_fn();
    }
    
    // Не должны сюда попасть
    loop {
        asm::nop();
    }
}

// Переход к обновлятору
fn jump_to_updater() -> ! {
    unsafe {
        // Адрес обработчика сброса
        let reset_addr = UPDATER_ADDR + 4;
        let reset_ptr = *(reset_addr as *const u32);
        
        // Деинициализация RCC
        let p = pac::Peripherals::steal();
        
        p.rcc.cr().modify(|_, w| w
            .hsion().set_bit()
            .hseon().clear_bit()
            .pllon().clear_bit()
        );
        
        while p.rcc.cr().read().hsirdy().bit_is_clear() {}
        
        p.rcc.cfgr().modify(|_, w| w.sw().hsi());
        while !p.rcc.cfgr().read().sws().is_hsi() {}
        
        p.rcc.cfgr().reset();
        
        // Рэмапинг памяти
        p.syscfg.memrm().modify(|_, w| w.mem_mode().bits(0x01));
        
        // Отключение SysTick
        let mut cp = cortex_m::Peripherals::steal();
        let systick: &mut SYST = &mut cp.SYST;
        systick.disable_counter();
        systick.disable_interrupt();
        
        // Очистка PendSV
        let scb = SCB::ptr();
        (*scb).icsr.write(1 << 27);
        
        // Отключение обработчиков ошибок SCB
        (*scb).shcsr.modify(|v| v & !(
            (1 << 18) | // USGFAULTENA
            (1 << 17) | // BUSFAULTENA
            (1 << 16)   // MEMFAULTENA
        ));
        
        // Установка таблицы векторов
        (*scb).vtor.write(UPDATER_ADDR);
        
        // Установка указателя стека
        let stack_ptr = *(UPDATER_ADDR as *const u32);
        msp::write(stack_ptr);
        psp::write(stack_ptr);
        
        // Переход к обновлятору
        let jump_fn: fn() -> ! = core::mem::transmute(reset_ptr);
        jump_fn();
    }
    
    // Не должны сюда попасть
    loop {
        asm::nop();
    }
}

// Обработчик прерывания USART2
#[interrupt]
fn USART2() {
    let p: pac::Peripherals = unsafe { pac::Peripherals::steal() };

    unsafe {
        let usart2 = &p.usart2;
        
        // Обработка приема данных
        if usart2.sr().read().rxne().bit_is_set() && usart2.cr1().read().rxneie().bit_is_set() {
            let received_data = usart2.dr().read().bits() as u8;
            RX_BUFFER.write(received_data);
        }
        
        // Обработка передачи данных
        if usart2.sr().read().txe().bit_is_set() && usart2.cr1().read().txeie().bit_is_set() {
            TX_IN_PROGRESS = false;
            
            // Попытка отправить следующий байт
            ring_buffer_transmit();
        }
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}