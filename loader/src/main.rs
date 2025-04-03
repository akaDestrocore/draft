#![no_std]
#![no_main]

use core::{
    panic::PanicInfo, 
    sync::atomic::{AtomicBool, Ordering}
};
use cortex_m::{asm, peripheral::SYST};
use cortex_m_rt::entry;
use stm32f4::{self as pac, Peripherals};
use misc::{
    flash,
    image::{ImageHeader, SharedMemory, IMAGE_MAGIC_LOADER, IMAGE_TYPE_LOADER}
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

// Тестовые данные и константы
const TEST_ADDR: u32 = 0x08020000;  // Начало сектора 5
const TEST_SIZE: usize = 1024;      // Размер тестовых данных
const TEST_PATTERN1: u32 = 0xABCDABCD;
const TEST_PATTERN2: u32 = 0x12341234;

// Глобальные переменные для отслеживания состояния теста
static TEST_FAILED: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    // Получаем доступ к периферии
    let p = unsafe { pac::Peripherals::steal() };
    
    // Настройка системных часов
    setup_system_clock(&p);
    
    // Настройка светодиодов
    setup_leds(&p);
    
    // Подготовка тестовых данных
    let mut test_pattern1 = [0u8; TEST_SIZE];
    let mut test_pattern2 = [0u8; TEST_SIZE];
    let mut read_buffer = [0u8; TEST_SIZE];
    
    // Заполняем тестовые шаблоны
    for i in (0..TEST_SIZE).step_by(4) {
        // Шаблон 1: 0xABCDABCD
        test_pattern1[i] = (TEST_PATTERN1 & 0xFF) as u8;
        test_pattern1[i+1] = ((TEST_PATTERN1 >> 8) & 0xFF) as u8;
        test_pattern1[i+2] = ((TEST_PATTERN1 >> 16) & 0xFF) as u8;
        test_pattern1[i+3] = ((TEST_PATTERN1 >> 24) & 0xFF) as u8;
        
        // Шаблон 2: 0x12341234
        test_pattern2[i] = (TEST_PATTERN2 & 0xFF) as u8;
        test_pattern2[i+1] = ((TEST_PATTERN2 >> 8) & 0xFF) as u8;
        test_pattern2[i+2] = ((TEST_PATTERN2 >> 16) & 0xFF) as u8;
        test_pattern2[i+3] = ((TEST_PATTERN2 >> 24) & 0xFF) as u8;
    }
    
    // =========== ПОШАГОВЫЙ ТЕСТ FLASH ==============
    
    // ШАГ 0: Индикация начала теста (Все светодиоды)
    set_all_leds(&p, true);
    delay(500_000);
    set_all_leds(&p, false);
    delay(500_000);
    
    // ШАГ 1: Стирание сектора - зеленый светодиод
    toggle_led(&p, 12);  // Зеленый: начало стирания
    
    let erase_size: u32 = flash::erase_sector(&p, TEST_ADDR);
    if erase_size == 0 {
        // Ошибка стирания - мигаем красным
        error_blink(&p, 14);
        TEST_FAILED.store(true, Ordering::SeqCst);
        halt();
    }
    
    toggle_led(&p, 12);  // Зеленый: конец стирания
    delay(200_000);
    
    // Проверяем, что сектор действительно стерт (все байты 0xFF)
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for &byte in read_buffer.iter() {
        if byte != 0xFF {
            // Если хоть один байт не 0xFF, значит стирание не произошло
            error_blink(&p, 14);
            TEST_FAILED.store(true, Ordering::SeqCst);
            halt();
        }
    }
    
    // ШАГ 2: Запись первого шаблона - оранжевый светодиод
    toggle_led(&p, 13);  // Оранжевый: начало записи шаблона 1
    
    let write_result = flash::write(&p, &test_pattern1, TEST_ADDR);
    if write_result != 0 {
        // Ошибка записи - мигаем красным и оранжевым
        error_blink_dual(&p, 14, 13);
        TEST_FAILED.store(true, Ordering::SeqCst);
        halt();
    }
    
    toggle_led(&p, 13);  // Оранжевый: конец записи шаблона 1
    delay(200_000);
    
    // ШАГ 3: Проверка записи первого шаблона - синий светодиод
    toggle_led(&p, 15);  // Синий: начало проверки шаблона 1
    
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for i in 0..TEST_SIZE {
        if read_buffer[i] != test_pattern1[i] {
            // Если данные не совпадают, значит запись произошла с ошибкой
            error_blink_dual(&p, 14, 15);
            TEST_FAILED.store(true, Ordering::SeqCst);
            halt();
        }
    }
    
    toggle_led(&p, 15);  // Синий: конец проверки шаблона 1
    delay(200_000);
    
    // ШАГ 4: Повторное стирание сектора - зеленый светодиод
    toggle_led(&p, 12);  // Зеленый: начало повторного стирания
    
    let erase_size = flash::erase_sector(&p, TEST_ADDR);
    if erase_size == 0 {
        // Ошибка стирания - мигаем красным
        error_blink(&p, 14);
        TEST_FAILED.store(true, Ordering::SeqCst);
        halt();
    }
    
    toggle_led(&p, 12);  // Зеленый: конец повторного стирания
    delay(200_000);
    
    // Проверяем, что сектор действительно стерт (все байты 0xFF)
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for &byte in read_buffer.iter() {
        if byte != 0xFF {
            // Если хоть один байт не 0xFF, значит стирание не произошло
            error_blink(&p, 14);
            TEST_FAILED.store(true, Ordering::SeqCst);
            halt();
        }
    }
    
    // ШАГ 5: Запись второго шаблона - оранжевый светодиод
    toggle_led(&p, 13);  // Оранжевый: начало записи шаблона 2
    
    let write_result = flash::write(&p, &test_pattern2, TEST_ADDR);
    if write_result != 0 {
        // Ошибка записи - мигаем красным и оранжевым
        error_blink_dual(&p, 14, 13);
        TEST_FAILED.store(true, Ordering::SeqCst);
        halt();
    }
    
    toggle_led(&p, 13);  // Оранжевый: конец записи шаблона 2
    delay(200_000);
    
    // ШАГ 6: Проверка записи второго шаблона - синий светодиод
    toggle_led(&p, 15);  // Синий: начало проверки шаблона 2
    
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for i in 0..TEST_SIZE {
        if read_buffer[i] != test_pattern2[i] {
            // Если данные не совпадают, значит запись произошла с ошибкой
            error_blink_dual(&p, 14, 15);
            TEST_FAILED.store(true, Ordering::SeqCst);
            halt();
        }
    }
    
    toggle_led(&p, 15);  // Синий: конец проверки шаблона 2
    delay(200_000);
    
    // ШАГ 7: Тест функции множественного стирания - все светодиоды
    set_all_leds(&p, true);
    
    if !flash::erase(&p, TEST_ADDR) {
        // Ошибка множественного стирания
        error_blink(&p, 14);
        TEST_FAILED.store(true, Ordering::SeqCst);
        halt();
    }
    
    set_all_leds(&p, false);
    delay(200_000);
    
    // ШАГ 8: Проверка множественного стирания - чтение нескольких секторов
    toggle_led(&p, 15);  // Синий: начало проверки множественного стирания
    
    // Проверяем несколько адресов в разных секторах
    let check_addresses = [TEST_ADDR, TEST_ADDR + 0x10000, TEST_ADDR + 0x20000];
    for &addr in check_addresses.iter() {
        flash::read(addr, &mut read_buffer);
        
        // Проверяем только первые 16 байтов из каждого сектора
        for &byte in read_buffer[0..16].iter() {
            if byte != 0xFF {
                // Если хоть один байт не 0xFF, значит стирание не произошло
                error_blink(&p, 14);
                TEST_FAILED.store(true, Ordering::SeqCst);
                halt();
            }
        }
    }
    
    toggle_led(&p, 15);  // Синий: конец проверки множественного стирания
    delay(200_000);
    
    // Все тесты пройдены успешно!
    success_pattern(&p);
    
    // Бесконечный цикл с миганием зеленого светодиода в случае успеха
    loop {
        toggle_led(&p, 12);  // Зеленый: успешное выполнение
        delay(1_000_000);
    }
}

fn setup_system_clock(p: &pac::Peripherals) {
    // Включение PWR
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());
    
    // Установка Scale 1 для регулятора напряжения
    p.pwr.cr().modify(|_, w| unsafe {
        w.vos().scale1()
    });
    
    // Настройка Flash: 5 wait states, prefetch, инструкции и кэш данных
    p.flash.acr().modify(|_, w| w
        .latency().ws5()
        .prften().enabled()
        .icen().enabled()
        .dcen().enabled()
    );
    
    // Включение HSE
    p.rcc.cr().modify(|_, w| w.hseon().set_bit());
    while p.rcc.cr().read().hserdy().bit_is_clear() {}
    
    // Настройка PLL
    p.rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllsrc().hse()   // HSE как источник для PLL
         .pllm().bits(4)   // PLLM = 4 для HSE 8MHz: 8MHz/4 = 2MHz
         .plln().bits(168) // PLLN = 168: 2MHz * 168 = 336MHz 
         .pllp().div2()    // PLLP = 2: 336MHz/2 = 168MHz
         .pllq().bits(7)   // PLLQ = 7: 336MHz/7 = 48MHz для USB
    });
    
    // Включение PLL
    p.rcc.cr().modify(|_, w| w.pllon().set_bit());
    while p.rcc.cr().read().pllrdy().bit_is_clear() {}
    
    // Настройка делителей для шин AHB, APB1, APB2
    p.rcc.cfgr().modify(|_, w| {
        w.hpre().div1()     // AHB = SYSCLK/1 = 168MHz
         .ppre1().div4()    // APB1 = AHB/4 = 42MHz (макс. 42MHz)
         .ppre2().div2()    // APB2 = AHB/2 = 84MHz (макс. 84MHz)
    });
    
    // Установка PLL как источника системных часов
    p.rcc.cfgr().modify(|_, w| w.sw().pll());
    while !p.rcc.cfgr().read().sws().is_pll() {}
}

fn setup_leds(p: &pac::Peripherals) {
    // Включение тактирования GPIOD
    p.rcc.ahb1enr().modify(|_, w| w.gpioden().set_bit());
    
    // Настройка пинов светодиодов (PD12-15) как выходы
    p.gpiod.moder().modify(|_, w| w
        .moder12().output()  // PD12 - зеленый
        .moder13().output()  // PD13 - оранжевый
        .moder14().output()  // PD14 - красный
        .moder15().output()  // PD15 - синий
    );
    
    // Настройка типа выхода - push-pull
    p.gpiod.otyper().modify(|_, w| w
        .ot12().push_pull()
        .ot13().push_pull()
        .ot14().push_pull()
        .ot15().push_pull()
    );
    
    // Настройка скорости - низкая
    p.gpiod.ospeedr().modify(|_, w| w
        .ospeedr12().low_speed()
        .ospeedr13().low_speed()
        .ospeedr14().low_speed()
        .ospeedr15().low_speed()
    );
    
    // Выключаем все светодиоды
    set_all_leds(p, false);
}

// Функции для работы со светодиодами
fn toggle_led(p: &pac::Peripherals, pin: u8) {
    // Читаем текущее состояние светодиода
    let current = p.gpiod.odr().read().bits() & (1 << pin) != 0;
    
    if current {
        // Выключаем светодиод
        p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << (pin + 16)) });
    } else {
        // Включаем светодиод
        p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << pin) });
    }
}

fn set_led(p: &pac::Peripherals, pin: u8, state: bool) {
    if state {
        // Включаем светодиод
        p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << pin) });
    } else {
        // Выключаем светодиод
        p.gpiod.bsrr().write(|w| unsafe { w.bits(1 << (pin + 16)) });
    }
}

fn set_all_leds(p: &pac::Peripherals, state: bool) {
    for pin in 12..=15 {
        set_led(p, pin, state);
    }
}

fn error_blink(p: &pac::Peripherals, pin: u8) {
    for _ in 0..10 {
        set_led(p, pin, true);
        delay(100_000);
        set_led(p, pin, false);
        delay(100_000);
    }
}

fn error_blink_dual(p: &pac::Peripherals, pin1: u8, pin2: u8) {
    for _ in 0..10 {
        set_led(p, pin1, true);
        set_led(p, pin2, true);
        delay(100_000);
        set_led(p, pin1, false);
        set_led(p, pin2, false);
        delay(100_000);
    }
}

fn success_pattern(p: &pac::Peripherals) {
    // Последовательно проходим по всем светодиодам 3 раза
    for _ in 0..3 {
        for pin in 12..=15 {
            set_led(p, pin, true);
            delay(200_000);
            set_led(p, pin, false);
            delay(50_000);
        }
    }
    
    // Затем включаем и выключаем все светодиоды одновременно
    for _ in 0..3 {
        set_all_leds(p, true);
        delay(300_000);
        set_all_leds(p, false);
        delay(300_000);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        cortex_m::asm::nop();
    }
}

fn halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}