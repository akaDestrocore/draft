use core::sync::atomic::{AtomicBool, Ordering};
use misc::ring_buffer::RingBuffer;
use stm32f4 as pac;

// Статические буферы для UART-коммуникации
static mut RX_BUFFER: RingBuffer = RingBuffer::new();
static mut TX_BUFFER: RingBuffer = RingBuffer::new();
static TX_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

#[derive(Debug)]
pub enum UartError {
    BufferFull,
}

pub struct UartManager<'a> {
    usart2: &'a pac::usart1::RegisterBlock,
}

impl<'a> UartManager<'a> {
    pub fn new(p: &'a pac::Peripherals) -> Self {
        Self {
            usart2: unsafe {
                &*(pac::Usart2::ptr())
            },
        }
    }

    pub fn init(&mut self) {
        // Инициализация UART
        unsafe {
            // Настройка USART2 @ 115200 бод, 8-бит, 1 стоп-бит, без четности
            self.usart2.brr().write(|w| w.bits(0xc3)); // 90MHz / 115200 ≈ 781.25 = 0xc3
            
            // Включение USART2
            self.usart2.cr1().modify(|_, w| {
                w.ue().enabled()     // Включение USART
                 .te().enabled()     // Включение передатчика
                 .re().enabled()     // Включение приемника
                 .rxneie().enabled() // Включение прерывания RXNE
            });
        }
    }

    pub fn process(&mut self) {
        // Проверка необходимости начать новую передачу
        if !TX_IN_PROGRESS.load(Ordering::SeqCst) {
            unsafe {
                if let Some(byte) = TX_BUFFER.read() {
                    // Запись байта в регистр данных
                    self.usart2.dr().write(|w| w.bits(byte as u16));
                    
                    // Включение прерывания TXE
                    self.usart2.cr1().modify(|_, w| w.txeie().enabled());
                    
                    TX_IN_PROGRESS.store(true, Ordering::SeqCst);
                }
            }
        }
    }

    pub fn read_byte(&mut self) -> Option<u8> {
        unsafe { RX_BUFFER.read() }
    }

    pub fn send_byte(&mut self, byte: u8) {
        unsafe {
            if !TX_BUFFER.write(byte) {
                // Буфер полон, но мы не можем обрабатывать ошибки здесь
                return;
            }
        }
        self.process();
    }

    pub fn send_string(&mut self, s: &str) {
        for byte in s.bytes() {
            self.send_byte(byte);
        }
    }

    pub fn is_tx_complete(&self) -> bool {
        unsafe { TX_BUFFER.is_empty() && !TX_IN_PROGRESS.load(Ordering::SeqCst) }
    }
}

// Эта функция вызывается из обработчика прерывания USART2
#[no_mangle]
pub extern "C" fn process_uart_interrupt() {
    use pac::Peripherals;
    
    let p = unsafe { Peripherals::steal() };
    
    // Проверка RX
    if p.usart2.sr().read().rxne().bit_is_set() {
        let data = p.usart2.dr().read().bits() as u8;
        unsafe {
            let _ = RX_BUFFER.write(data);
        }
    }
    
    // Проверка TX
    if p.usart2.sr().read().txe().bit_is_set() && p.usart2.cr1().read().txeie().bit_is_set() {
        unsafe {
            if let Some(byte) = TX_BUFFER.read() {
                // Отправка следующего байта
                p.usart2.dr().write(|w| w.bits(byte as u16));
            } else {
                // Больше нет данных для отправки, отключаем прерывание TXE
                p.usart2.cr1().modify(|_, w| w.txeie().disabled());
                TX_IN_PROGRESS.store(false, Ordering::SeqCst);
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn USART2() {
    process_uart_interrupt();
}