#![no_std]

use stm32f4 as pac;
use crate::{
    ring_buffer::RingBuffer,
    systick,
};

// XMODEM protocol constants
pub const SOH: u8 = 0x01;  // Start of Header (128-byte packet)
pub const STX: u8 = 0x02;  // Start of Text (1024-byte packet)
pub const EOT: u8 = 0x04;  // End of Transmission
pub const ACK: u8 = 0x06;  // Acknowledge
pub const NAK: u8 = 0x15;  // Negative Acknowledge
pub const CAN: u8 = 0x18;  // Cancel
pub const C: u8 = 0x43;    // ASCII 'C' for CRC mode

// Состояния XMODEM
#[derive(PartialEq, Clone, Copy, Debug)]
pub enum XmodemState {
    Idle,    // Ожидание
    Active,  // Приём данных
    Done     // Завершено
}

// Глобальные переменные
static mut STATE: XmodemState = XmodemState::Idle;
static mut ACTIVE: bool = false;
static mut BYTE_COUNTER: usize = 0;
static mut LAST_C_TIME: u32 = 0;
static mut LAST_BYTE: u8 = 0;
static mut SAW_EOT: bool = false;
static mut SENT_ACK: bool = false;

// Public API
pub fn get_state() -> XmodemState {
    unsafe { STATE }
}

pub fn is_update_in_progress() -> bool {
    unsafe { ACTIVE }
}

pub fn queue_string(tx_buffer: &RingBuffer, s: &str) {
    for byte in s.bytes() {
        tx_buffer.write(byte);
    }
}

// Начать приём
pub fn start_update(tx_buffer: &RingBuffer, _target_addr: u32, _is_app_update: bool) {
    queue_string(tx_buffer, "\r\nXMODEM простой приём...\r\n");
    
    unsafe {
        STATE = XmodemState::Active;
        ACTIVE = true;
        BYTE_COUNTER = 0;
        LAST_C_TIME = systick::get_tick_ms();
        LAST_BYTE = 0;
        SAW_EOT = false;
        SENT_ACK = false;
    }
    
    // Отправка первого 'C'
    tx_buffer.write(C);
}

// Основная функция обработки
pub fn process_xmodem(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) -> bool {
    let current_time = systick::get_tick_ms();
    
    // Индикация и периодическая отправка 'C'
    unsafe {
        match STATE {
            XmodemState::Idle => {
                // Зелёный LED
                set_led(p, 0, true);
                set_led(p, 1, false);
                set_led(p, 2, false);
                set_led(p, 3, false);
            },
            XmodemState::Active => {
                if !SAW_EOT {
                    // Отправляем 'C' каждую секунду, пока не получим данные
                    if BYTE_COUNTER == 0 && current_time.wrapping_sub(LAST_C_TIME) >= 1000 {
                        tx_buffer.write(C);
                        LAST_C_TIME = current_time;
                        
                        // Мигаем оранжевым при отправке 'C'
                        toggle_led(p, 1);
                    }
                    
                    // Синий горит во время приёма
                    set_led(p, 3, BYTE_COUNTER > 0);
                } else if !SENT_ACK {
                    // Получили EOT, но ещё не отправили ACK
                    // Мигаем красным
                    set_led(p, 2, (current_time / 100) % 2 == 0);
                    
                    // Отправим ACK
                    tx_buffer.write(ACK);
                    tx_buffer.write(ACK);  // Дважды для надёжности
                    SENT_ACK = true;
                } else {
                    // Отправили ACK после EOT
                    // Ждём немного и завершаем
                    if current_time.wrapping_sub(LAST_C_TIME) >= 500 {
                        STATE = XmodemState::Done;
                        ACTIVE = false;
                        
                        // Все LED включены при успешном завершении
                        set_led(p, 0, true);
                        set_led(p, 1, true);
                        set_led(p, 2, true);
                        set_led(p, 3, true);
                        
                        // Сообщение об успешном завершении
                        queue_string(tx_buffer, "\r\nПриём завершён успешно!\r\n");
                        return true;
                    }
                }
            },
            XmodemState::Done => {
                // Все LED включены на успешное завершение
                set_led(p, 0, true);
                set_led(p, 1, true);
                set_led(p, 2, true);
                set_led(p, 3, true);
            }
        }
    }
    
    // Обработка входных байтов
    while let Some(byte) = rx_buffer.read() {
        unsafe {
            LAST_BYTE = byte;
            BYTE_COUNTER += 1;
            
            // Проверка на EOT
            if byte == EOT {
                SAW_EOT = true;
                LAST_C_TIME = current_time;
                
                // Моментально отправим ACK дважды
                tx_buffer.write(ACK);
                tx_buffer.write(ACK);
                SENT_ACK = true;
                
                // Активируем все LED на мгновение
                set_led(p, 0, true);
                set_led(p, 1, true);
                set_led(p, 2, true);
                set_led(p, 3, true);
            }
            
            // На любой пакет (SOH/STX + 132 байта) отправляем ACK
            if (byte == SOH || byte == STX) && STATE == XmodemState::Active {
                // Это начало пакета, следующие 132 байта просто игнорируем
                // и отправляем ACK через 132 байта
                let ignore_count = BYTE_COUNTER + 132;
                
                // Дождёмся получения 132 байт и отправим ACK
                if BYTE_COUNTER >= ignore_count {
                    tx_buffer.write(ACK);
                }
            }
        }
    }
    
    // Продолжаем обработку
    false
}

// Вспомогательные функции для LED
fn set_led(p: &pac::Peripherals, led: u8, state: bool) {
    match led {
        0 => if state { p.gpiod.bsrr().write(|w| w.bs12().set_bit()); } // Green - PD12 
            else { p.gpiod.bsrr().write(|w| w.br12().set_bit()); },
        1 => if state { p.gpiod.bsrr().write(|w| w.bs13().set_bit()); } // Orange - PD13
            else { p.gpiod.bsrr().write(|w| w.br13().set_bit()); },
        2 => if state { p.gpiod.bsrr().write(|w| w.bs14().set_bit()); } // Red - PD14
            else { p.gpiod.bsrr().write(|w| w.br14().set_bit()); },
        3 => if state { p.gpiod.bsrr().write(|w| w.bs15().set_bit()); } // Blue - PD15
            else { p.gpiod.bsrr().write(|w| w.br15().set_bit()); },
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