use core::sync::atomic::{AtomicBool, Ordering};
use heapless::spsc::Queue;
use stm32f4 as pac;

pub const UART_BUFFER_SIZE: usize = 256;

// Static buffers for UART communications
static mut RX_BUFFER: Queue<u8, UART_BUFFER_SIZE> = Queue::new();
static mut TX_BUFFER: Queue<u8, UART_BUFFER_SIZE> = Queue::new();
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
            usart2: &*p.usart2.brr().as_ptr().cast::<pac::usart1::RegisterBlock>(),
        }
    }

    pub fn init(&mut self) {
        // Enable UART
        unsafe {
            // Configure USART2 @ 115200 baud, 8-bit, 1 stop bit, no parity
            self.usart2.brr().write(|w| w.bits(0xc3)); // 90MHz / 115200 ≈ 781.25 = 0xc3
            
            // Enable USART2
            self.usart2.cr1().modify(|_, w| {
                w.ue().enabled()  // Enable USART
                 .te().enabled()  // Enable transmitter
                 .re().enabled()  // Enable receiver
                 .rxneie().enabled() // Enable RXNE interrupt
            });
        }
    }

    pub fn process(&mut self) {
        // Check if we need to start a new transmission
        if !TX_IN_PROGRESS.load(Ordering::SeqCst) {
            unsafe {
                if let Some(byte) = TX_BUFFER.dequeue() {
                    // Write the byte to the data register
                    self.usart2.dr().write(|w| w.bits(byte as u16));
                    
                    // Enable TXE interrupt
                    self.usart2.cr1().modify(|_, w| w.txeie().enabled());
                    
                    TX_IN_PROGRESS.store(true, Ordering::SeqCst);
                }
            }
        }
    }

    pub fn read_byte(&mut self) -> Option<u8> {
        unsafe { RX_BUFFER.dequeue() }
    }

    pub fn send_byte(&mut self, byte: u8) {
        unsafe {
            if TX_BUFFER.enqueue(byte).is_err() {
                // Buffer is full, but we can't handle errors here
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

// This function is called from the USART2 interrupt handler
#[no_mangle]
pub extern "C" fn process_uart_interrupt() {
    use pac::Peripherals;
    
    let p = unsafe { Peripherals::steal() };
    
    // Check RX
    if p.usart2.sr().read().rxne().bit_is_set() {
        let data = p.usart2.dr().read().bits() as u8;
        unsafe {
            let _ = RX_BUFFER.enqueue(data);
        }
    }
    
    // Check TX
    if p.usart2.sr().read().txe().bit_is_set() && p.usart2.cr1().read().txeie().bit_is_set() {
        unsafe {
            if let Some(byte) = TX_BUFFER.dequeue() {
                // Send next byte
                p.usart2.dr().write(|w| w.bits(byte as u16));
            } else {
                // No more data to send, disable TXE interrupt
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