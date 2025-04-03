#![no_std]
#![no_main]

use core::{
    cell::UnsafeCell, panic::PanicInfo, sync::atomic::{AtomicBool, Ordering}
};
use cortex_m::{
    asm,
    peripheral::{self, SYST}
};
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

// Test address - beginning of sector 2 (0x08008000)
const TEST_ADDR: u32 = 0x08008000;
const TEST_PATTERN1: u32 = 0xABCDABCD;
const TEST_PATTERN2: u32 = 0x12341234;
const TEST_SIZE: usize = 1024; // 1KB of test data

// Для хранения глобальных указателей
pub struct Mutex<T> {
    inner: UnsafeCell<T>
}

unsafe impl<T> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    pub const fn new(value: T) -> Self {
        Self { inner: UnsafeCell::new(value) }
    }

    pub fn get<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        cortex_m::interrupt::free(|_| {
            let ptr = self.inner.get();
            f(unsafe { &mut *ptr })
        })
    }
}

// Обертка для указателей на периферию
struct PeripheralPtr<T>(*const T);
unsafe impl<T> Send for PeripheralPtr<T> {}
unsafe impl<T> Sync for PeripheralPtr<T> {}

// Глобальные указатели на GPIOD для работы со светодиодами
static GPIOD_PTR: Mutex<Option<PeripheralPtr<pac::gpiod::RegisterBlock>>> = Mutex::new(None);
static TEST_FAILED: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    // Инициализация периферии в стиле вашего основного файла
    let p: Peripherals = match pac::Peripherals::take() {
        Some(p) => p,
        None => {
            loop {
                asm::nop();
            }
        }
    };
    
    // Сохраняем указатель на GPIOD для использования в других функциях
    let gpiod_ptr: &pac::gpiod::RegisterBlock = unsafe {
        &*(p.gpiod.moder().as_ptr() as *const _ as *const pac::gpiod::RegisterBlock)
    };
    GPIOD_PTR.get(|ptr| *ptr = Some(PeripheralPtr(gpiod_ptr)));
    
    // Настраиваем системные часы
    setup_system_clock(&p);
    
    // Настраиваем светодиоды
    setup_leds(&p);
    
    // Подготавливаем тестовые данные
    let mut test_pattern1 = [0u8; TEST_SIZE];
    let mut test_pattern2 = [0u8; TEST_SIZE];
    
    // Заполняем тестовые данные повторяющимися значениями
    for i in (0..TEST_SIZE).step_by(4) {
        // Pattern 1: 0xABCDABCD
        test_pattern1[i] = (TEST_PATTERN1 & 0xFF) as u8;
        test_pattern1[i+1] = ((TEST_PATTERN1 >> 8) & 0xFF) as u8;
        test_pattern1[i+2] = ((TEST_PATTERN1 >> 16) & 0xFF) as u8;
        test_pattern1[i+3] = ((TEST_PATTERN1 >> 24) & 0xFF) as u8;
        
        // Pattern 2: 0x12341234
        test_pattern2[i] = (TEST_PATTERN2 & 0xFF) as u8;
        test_pattern2[i+1] = ((TEST_PATTERN2 >> 8) & 0xFF) as u8;
        test_pattern2[i+2] = ((TEST_PATTERN2 >> 16) & 0xFF) as u8;
        test_pattern2[i+3] = ((TEST_PATTERN2 >> 24) & 0xFF) as u8;
    }
    
    // Буфер для чтения данных
    let mut read_buffer = [0u8; TEST_SIZE];
    
    // ШАГ 1: Стираем целевой сектор
    blink_led(12); // Зеленый светодиод
    
    let erase_result = flash::erase_sector(&p, TEST_ADDR);
    if erase_result == 0 {
        error_blink(14); // Красный светодиод
        TEST_FAILED.store(true, Ordering::SeqCst);
        loop {
            asm::nop();
        }
    }
    
    // ШАГ 2: Записываем первый тестовый шаблон
    blink_led(13); // Оранжевый светодиод
    
    let write_result = flash::write(&p, &test_pattern1, TEST_ADDR);
    if write_result != 0 {
        error_blink(14); // Красный светодиод
        TEST_FAILED.store(true, Ordering::SeqCst);
        loop {
            asm::nop();
        }
    }
    
    // ШАГ 3: Читаем и проверяем первый шаблон
    blink_led(12); // Зеленый светодиод
    
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for i in 0..TEST_SIZE {
        if read_buffer[i] != test_pattern1[i] {
            error_blink(14); // Красный светодиод
            TEST_FAILED.store(true, Ordering::SeqCst);
            loop {
                asm::nop();
            }
        }
    }
    
    // ШАГ 4: Снова стираем сектор
    blink_led(13); // Оранжевый светодиод
    
    let erase_result = flash::erase_sector(&p, TEST_ADDR);
    if erase_result == 0 {
        error_blink(14); // Красный светодиод
        TEST_FAILED.store(true, Ordering::SeqCst);
        loop {
            asm::nop();
        }
    }
    
    // ШАГ 5: Проверяем, что сектор стерт (все 0xFF)
    blink_led(12); // Зеленый светодиод
    
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for i in 0..TEST_SIZE {
        if read_buffer[i] != 0xFF {
            error_blink(14); // Красный светодиод
            TEST_FAILED.store(true, Ordering::SeqCst);
            loop {
                asm::nop();
            }
        }
    }
    
    // ШАГ 6: Записываем второй тестовый шаблон
    blink_led(13); // Оранжевый светодиод
    
    let write_result = flash::write(&p, &test_pattern2, TEST_ADDR);
    if write_result != 0 {
        error_blink(14); // Красный светодиод
        TEST_FAILED.store(true, Ordering::SeqCst);
        loop {
            asm::nop();
        }
    }
    
    // ШАГ 7: Читаем и проверяем второй шаблон
    blink_led(12); // Зеленый светодиод
    
    flash::read(TEST_ADDR, &mut read_buffer);
    
    for i in 0..TEST_SIZE {
        if read_buffer[i] != test_pattern2[i] {
            error_blink(14); // Красный светодиод
            TEST_FAILED.store(true, Ordering::SeqCst);
            loop {
                asm::nop();
            }
        }
    }
    
    // Все тесты пройдены! Отмечаем успех
    if !TEST_FAILED.load(Ordering::SeqCst) {
        success_pattern();
    }
    
    loop {
        asm::nop();
    }
}

fn setup_system_clock(p: &pac::Peripherals) {
    // Включаем PWR
    p.rcc.apb1enr().modify(|_, w| w.pwren().set_bit());

    // Устанавливаем Scale 1
    p.pwr.cr().modify(|_, w| w.vos().scale1());

    // Настраиваем доступ к флэш-памяти
    p.flash.acr().modify(|_, w| w
        .latency().ws5()
        .prften().set_bit()
        .icen().set_bit()
        .dcen().set_bit()
    );

    // Включаем HSE
    p.rcc.cr().modify(|_, w| w.hseon().set_bit());
    while p.rcc.cr().read().hserdy().bit_is_clear() {
        // Ждем готовности HSE
    }

    // Настраиваем PLL
    p.rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllsrc().hse()
         .pllm().bits(4)
         .plln().bits(90)
         .pllp().div2()
         .pllq().bits(4)
    });

    // Включаем PLL
    p.rcc.cr().modify(|_, w| w.pllon().set_bit());
    while p.rcc.cr().read().pllrdy().bit_is_clear() {
        // Ждем готовности PLL
    }

    // Настраиваем делители шин
    p.rcc.cfgr().modify(|_, w| {
        w.hpre().div1()
        .ppre1().div4()
        .ppre2().div2()
    });

    // Устанавливаем PLL как источник системных часов
    p.rcc.cfgr().modify(|_, w| w.sw().pll());
    while !p.rcc.cfgr().read().sws().is_pll() {
        // Ждем переключения
    }
}

fn setup_leds(p: &pac::Peripherals) {
    // Включаем тактирование GPIOD
    p.rcc.ahb1enr().modify(|_, w| w.gpioden().set_bit());

    // Настраиваем LED (PD12-Green, PD13-Orange, PD14-Red, PD15-Blue)
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

// Функции работы со светодиодами через глобальные указатели
fn blink_led(pin: u8) {
    GPIOD_PTR.get(|gpiod_opt| {
        if let Some(ref gpiod_ptr) = *gpiod_opt {
            let gpiod = unsafe { &*gpiod_ptr.0 };
            
            // Включаем светодиод
            gpiod.bsrr().write(|w| unsafe { w.bits(1 << pin) });
            delay(200000);
            
            // Выключаем светодиод
            gpiod.bsrr().write(|w| unsafe { w.bits(1 << (pin + 16)) });
            delay(200000);
        }
    });
}

fn error_blink(pin: u8) {
    GPIOD_PTR.get(|gpiod_opt| {
        if let Some(ref gpiod_ptr) = *gpiod_opt {
            let gpiod = unsafe { &*gpiod_ptr.0 };
            
            // Быстро мигаем красным светодиодом 10 раз
            for _ in 0..10 {
                gpiod.bsrr().write(|w| unsafe { w.bits(1 << pin) });
                delay(50000);
                gpiod.bsrr().write(|w| unsafe { w.bits(1 << (pin + 16)) });
                delay(50000);
            }
        }
    });
}

fn success_pattern() {
    GPIOD_PTR.get(|gpiod_opt| {
        if let Some(ref gpiod_ptr) = *gpiod_opt {
            let gpiod = unsafe { &*gpiod_ptr.0 };
            
            // Последовательно проходим по всем светодиодам
            for _ in 0..3 {
                for pin in 12..=15 {
                    gpiod.bsrr().write(|w| unsafe { w.bits(1 << pin) });
                    delay(100000);
                    gpiod.bsrr().write(|w| unsafe { w.bits(1 << (pin + 16)) });
                }
            }
            
            // Затем включаем все одновременно
            gpiod.bsrr().write(|w| unsafe { w.bits(0xF000) });
            delay(500000);
            gpiod.bsrr().write(|w| unsafe { w.bits(0xF0000) });
        }
    });
}

fn delay(count: u32) {
    for _ in 0..count {
        asm::nop();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        asm::nop();
    }
}