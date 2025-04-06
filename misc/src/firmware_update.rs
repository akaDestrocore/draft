#![no_std]

use stm32f4 as pac;
use crate::{
    ring_buffer::RingBuffer,
    systick,
};
use core::sync::atomic::{AtomicU8, Ordering};

// XMODEM протокол - константы
pub const X_SOH: u8 = 0x01;  // Start of Header
pub const X_EOT: u8 = 0x04;  // End of Transmission
pub const X_ACK: u8 = 0x06;  // Acknowledge
pub const X_NAK: u8 = 0x15;  // Negative Acknowledge
pub const X_CAN: u8 = 0x18;  // Cancel
pub const X_C: u8 = 0x43;    // ASCII 'C' for CRC mode

// Состояния XMODEM - в точности как в C-коде
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum XmodemState {
    WAIT_SOH,
    WAIT_INDEX1,
    WAIT_INDEX2,
    READ_DATA,
    WAIT_CRC
}

// Глобальные переменные - как в C-коде
static mut STATE: XmodemState = XmodemState::WAIT_SOH;
static mut ACTIVE: bool = false;
static ERROR_CODE: AtomicU8 = AtomicU8::new(0);
static mut PACKET_COUNTER: u32 = 0;
static mut CURRENT_ADDRESS: u32 = 0;
static mut FIRST_PACKET: bool = false;
static mut COPY_DATA: bool = true;
static mut PACKET_RECEIVED: bool = false;
static mut PREV_INDEX1: u8 = 0;
static mut PREV_INDEX2: u8 = 0xFF;
static mut DATA_COUNTER: u8 = 0;
static mut CHECKSUM: u8 = 0;
static mut RX_BUFFER: [u8; 133] = [0; 133];
static mut ENC_DATA: [u8; 128] = [0; 128];

// Открытый API

/// Получить текущее состояние XMODEM
pub fn get_state() -> XmodemState {
    unsafe { STATE }
}

/// Проверить, активен ли процесс обновления
pub fn is_update_in_progress() -> bool {
    unsafe { ACTIVE }
}

/// Отправить строку в буфер передачи
pub fn queue_string(tx_buffer: &RingBuffer, s: &str) {
    for byte in s.bytes() {
        tx_buffer.write(byte);
    }
}

/// Получить диагностическую информацию
pub fn get_diagnostic_info() -> (XmodemState, u8, u32) {
    let state = get_state();
    let error = unsafe { ERROR_CODE.load(Ordering::Relaxed) };
    let packets = unsafe { PACKET_COUNTER };
    
    (state, error, packets)
}

/// Начать процесс обновления прошивки
pub fn start_update(tx_buffer: &RingBuffer, target_addr: u32, _is_app_update: bool) {
    unsafe {
        // Инициализация состояния
        STATE = XmodemState::WAIT_SOH;
        ACTIVE = true;
        ERROR_CODE.store(0, Ordering::Relaxed);
        PACKET_COUNTER = 0;
        CURRENT_ADDRESS = target_addr;
        FIRST_PACKET = false;
        COPY_DATA = true;
        PACKET_RECEIVED = false;
        PREV_INDEX1 = 0;
        PREV_INDEX2 = 0xFF;
        DATA_COUNTER = 0;
        CHECKSUM = 0;
    }
    
    // Отправляем 'C' для запроса передачи в режиме CRC
    tx_buffer.write(X_C);
}

/// Обработать данные XMODEM - главная функция
pub fn process_xmodem(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) -> bool {
    // Индикация на светодиодах
    unsafe {
        match STATE {
            XmodemState::WAIT_SOH => {
                // Зеленый светодиод
                set_led(p, 0, true); 
                set_led(p, 1, false);
                set_led(p, 2, false);
                set_led(p, 3, false);
            },
            XmodemState::WAIT_INDEX1 | XmodemState::WAIT_INDEX2 | XmodemState::READ_DATA => {
                // Зеленый + оранжевый
                set_led(p, 0, true);
                set_led(p, 1, true);
                set_led(p, 2, false);
                set_led(p, 3, false);
            },
            XmodemState::WAIT_CRC => {
                // Все светодиоды
                set_led(p, 0, true);
                set_led(p, 1, true);
                set_led(p, 2, true);
                set_led(p, 3, true);
            }
        }
    }
    
    // Логика в точности как в C-коде
    unsafe {
        // Отправляем 'C' если пакет еще не получен и это первый пакет
        if !PACKET_RECEIVED && !FIRST_PACKET {
            tx_buffer.write(X_C);
            // Добавим небольшую задержку чтобы не спамить C
            for _ in 0..10000 { core::hint::spin_loop(); }
        }
        
        // Отправляем NAK если пакет не получен, но это уже не первый пакет
        if !PACKET_RECEIVED && FIRST_PACKET {
            tx_buffer.write(X_NAK);
            // Небольшая задержка
            for _ in 0..1000 { core::hint::spin_loop(); }
        }
        
        // Если мы в состоянии ожидания SOH и пакет получен, отправляем ACK
        if STATE == XmodemState::WAIT_SOH && PACKET_RECEIVED {
            tx_buffer.write(X_ACK);
        }
    }
    
    // Проверяем, есть ли данные в буфере, достаточные для пакета
    if rx_buffer.get_count() >= 133 {
        unsafe { FIRST_PACKET = true; }
    }
    
    // Читаем байты из буфера, максимум 133 для одного пакета
    process_packets(p, tx_buffer, rx_buffer);
    
    // Завершено ли обновление
    unsafe {
        if STATE == XmodemState::WAIT_SOH && !ACTIVE {
            // Обновление завершено
            set_led(p, 0, true);
            set_led(p, 1, true);
            set_led(p, 2, true);
            set_led(p, 3, true);
            return true;
        }
    }
    
    false
}

// Внутренние функции

// Обработка пакетов - основа протокола
fn process_packets(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) {
    unsafe {
        // Проверяем наличие EOT в начале буфера
        if rx_buffer.get_count() > 0 {
            let mut byte = 0;
            if rx_buffer.peek(&mut byte) && byte == X_EOT {
                // Получили EOT, завершаем передачу
                rx_buffer.read(&mut byte); // Удаляем EOT из буфера
                tx_buffer.write(X_ACK);    // Подтверждаем завершение
                STATE = XmodemState::WAIT_SOH;
                ACTIVE = false;
                return;
            }
        }
        
        // Если нужно копировать данные в буфер
        if COPY_DATA && rx_buffer.get_count() >= 133 {
            // Копируем все 133 байта пакета
            for i in 0..133 {
                let mut byte = 0;
                if rx_buffer.read(&mut byte) {
                    RX_BUFFER[i] = byte;
                }
            }
            PACKET_RECEIVED = false;
            
            // Машина состояний обработки XMODEM - точно как в C-коде
            match STATE {
                XmodemState::WAIT_SOH => {
                    if RX_BUFFER[0] == X_SOH {
                        STATE = XmodemState::WAIT_INDEX1;
                        PACKET_RECEIVED = true;
                        COPY_DATA = false;
                    } else {
                        PACKET_RECEIVED = false;
                    }
                },
                XmodemState::WAIT_INDEX1 => {
                    // Проверка последовательности
                    if (PREV_INDEX1 + 1) == RX_BUFFER[1] {
                        STATE = XmodemState::WAIT_INDEX2;
                        PREV_INDEX1 += 1;
                    } else if (PREV_INDEX1 + 1) == 256 {
                        // Переход через 255
                        STATE = XmodemState::WAIT_INDEX2;
                        PREV_INDEX1 = 0;
                    } else {
                        // Отправляем отмену при ошибке
                        send_cancel(tx_buffer);
                        STATE = XmodemState::WAIT_SOH;
                        ACTIVE = false;
                        ERROR_CODE.store(5, Ordering::Relaxed); // Ошибка индекса
                        return;
                    }
                },
                XmodemState::WAIT_INDEX2 => {
                    // Проверка дополнения последовательности
                    if (PREV_INDEX2 - 1) == RX_BUFFER[2] {
                        STATE = XmodemState::READ_DATA;
                        PREV_INDEX2 -= 1;
                    } else if (PREV_INDEX2 - 1) == 255 {
                        // Переход через 0
                        STATE = XmodemState::READ_DATA;
                        PREV_INDEX2 = 255;
                    } else {
                        // Отправляем отмену при ошибке
                        send_cancel(tx_buffer);
                        STATE = XmodemState::WAIT_SOH;
                        ACTIVE = false;
                        ERROR_CODE.store(5, Ordering::Relaxed); // Ошибка индекса
                        return;
                    }
                },
                XmodemState::READ_DATA => {
                    // Копируем данные
                    for i in 0..128 {
                        ENC_DATA[i] = RX_BUFFER[i + 3];
                    }
                    STATE = XmodemState::WAIT_CRC;
                },
                XmodemState::WAIT_CRC => {
                    // Проверка CRC
                    let crc_received = ((RX_BUFFER[131] as u16) << 8) | (RX_BUFFER[132] as u16);
                    let crc_calculated = crc16_calculate(&ENC_DATA, 128);
                    
                    if crc_received == crc_calculated {
                        // CRC верный, обрабатываем пакет
                        
                        // Здесь должен быть код записи во флеш, но заменяем его на имитацию
                        // Мигаем зеленым светодиодом для индикации успешной обработки пакета
                        toggle_led(p, 0);
                        
                        // Отправляем ACK
                        tx_buffer.write(X_ACK);
                        
                        // Сбрасываем для следующего пакета
                        PACKET_RECEIVED = true;
                        COPY_DATA = true;
                        STATE = XmodemState::WAIT_SOH;
                        PACKET_COUNTER += 1;
                        
                        // Сброс буфера для следующего пакета
                        for i in 0..133 {
                            RX_BUFFER[i] = 0;
                        }
                    } else {
                        // CRC неверный, отправляем NAK
                        tx_buffer.write(X_NAK);
                        ERROR_CODE.store(6, Ordering::Relaxed); // Ошибка CRC
                        COPY_DATA = true;
                        STATE = XmodemState::WAIT_SOH;
                    }
                }
            }
        }
    }
}

// Отправка отмены передачи
fn send_cancel(tx_buffer: &RingBuffer) {
    // Отправляем CAN трижды, как в C-коде
    tx_buffer.write(X_CAN);
    tx_buffer.write(X_CAN);
    tx_buffer.write(X_CAN);
}

// Расчет CRC-16 - точная копия алгоритма из C-кода
fn crc16_calculate(ptr: &[u8], count: usize) -> u16 {
    let mut crc: u16 = 0;
    let mut cnt = count;
    
    let mut idx = 0;
    while cnt > 0 {
        cnt -= 1;
        crc = crc ^ ((ptr[idx] as u16) << 8);
        idx += 1;
        
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    crc & 0xFFFF
}

// Управление светодиодом
fn set_led(p: &pac::Peripherals, led: u8, state: bool) {
    match led {
        0 => if state { // Green (PD12)
                unsafe { p.gpiod.bsrr().write(|w| w.bs12().set_bit()); }
            } else {
                unsafe { p.gpiod.bsrr().write(|w| w.br12().set_bit()); }
            },
        1 => if state { // Orange (PD13)
                unsafe { p.gpiod.bsrr().write(|w| w.bs13().set_bit()); }
            } else {
                unsafe { p.gpiod.bsrr().write(|w| w.br13().set_bit()); }
            },
        2 => if state { // Red (PD14)
                unsafe { p.gpiod.bsrr().write(|w| w.bs14().set_bit()); }
            } else {
                unsafe { p.gpiod.bsrr().write(|w| w.br14().set_bit()); }
            },
        3 => if state { // Blue (PD15)
                unsafe { p.gpiod.bsrr().write(|w| w.bs15().set_bit()); }
            } else {
                unsafe { p.gpiod.bsrr().write(|w| w.br15().set_bit()); }
            },
        _ => {}
    }
}

// Переключение светодиода
fn toggle_led(p: &pac::Peripherals, led: u8) {
    match led {
        0 => unsafe { p.gpiod.odr().modify(|r, w| w.odr12().bit(!r.odr12().bit())); },
        1 => unsafe { p.gpiod.odr().modify(|r, w| w.odr13().bit(!r.odr13().bit())); },
        2 => unsafe { p.gpiod.odr().modify(|r, w| w.odr14().bit(!r.odr14().bit())); },
        3 => unsafe { p.gpiod.odr().modify(|r, w| w.odr15().bit(!r.odr15().bit())); },
        _ => {}
    }
}