use core::sync::atomic::{AtomicBool, Ordering};
use misc::ring_buffer::RingBuffer;
use stm32f4 as pac;

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
        // UART init
        unsafe {
            // USART2 115200 baudrate, 8-bit, 1 stop bit, no parity
            self.usart2.brr().write(|w| w.bits(0xc3)); // 90MHz / 115200 = 0xc3
            
            // enable USART2
            self.usart2.cr1().modify(|_, w| {
                w.ue().enabled()
                 .te().enabled()
                 .re().enabled()
                 .rxneie().enabled() // enable RXNE interrupt
            });
        }
    }

    pub fn process(&mut self) {
        if !TX_IN_PROGRESS.load(Ordering::SeqCst) {
            unsafe {
                if let Some(byte) = TX_BUFFER.read() {
                    self.usart2.dr().write(|w| w.bits(byte as u16));
                    
                    // enbale TX interrupt
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
                // buffer is full
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

// call this from USART2 irq handler
#[no_mangle]
pub extern "C" fn process_uart_interrupt() {
    use pac::Peripherals;
    
    let p = unsafe { Peripherals::steal() };
    
    // Check RX
    if p.usart2.sr().read().rxne().bit_is_set() {
        let data = p.usart2.dr().read().bits() as u8;
        unsafe {
            let _ = RX_BUFFER.write(data);
        }
    }
    
    // Check TX
    if p.usart2.sr().read().txe().bit_is_set() && p.usart2.cr1().read().txeie().bit_is_set() {
        unsafe {
            if let Some(byte) = TX_BUFFER.read() {
                // Send next byte
                p.usart2.dr().write(|w| w.bits(byte as u16));
            } else {
                // No more data to send, disable TXE
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