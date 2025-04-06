use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use stm32f4 as pac;

use crate::{
    flash,
    image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER},
    ring_buffer::RingBuffer,
    systick,
};

// XMODEM протокол - константы
pub const SOH: u8 = 0x01;  // Start of Header (128-байтовый пакет)
pub const STX: u8 = 0x02;  // Start of Text (1024-байтовый пакет)
pub const EOT: u8 = 0x04;  // End of Transmission
pub const ACK: u8 = 0x06;  // Acknowledge
pub const NAK: u8 = 0x15;  // Negative Acknowledge
pub const CAN: u8 = 0x18;  // Cancel
pub const C: u8 = 0x43;    // ASCII 'C' для режима CRC

// Счётчики для отладки
static mut PACKETS_COUNT: u32 = 0;
static mut EOT_RECEIVED: bool = false;
static mut ACK_SENT_FOR_EOT: bool = false;

// Состояния XMODEM - точно как в C коде
#[derive(PartialEq, Clone, Copy, Debug)]
pub enum XmodemState {
    WaitSOH,      // Ожидание SOH
    WaitIndex1,   // Ожидание первого байта индекса
    WaitIndex2,   // Ожидание второго байта индекса
    ReadData,     // Чтение данных
    WaitCRC       // Ожидание CRC
}

// Глобальные переменные состояния для XMODEM
static mut X_STATE: XmodemState = XmodemState::WaitSOH;
static mut PACKET_RECEIVED: bool = false;
static mut FIRST_PACKET: bool = false;
static mut COPY_DATA: bool = true;
static mut UPDATE_IN_PROGRESS: bool = false;

// Переменные для отслеживания пакетов
static mut PREV_INDEX1: u8 = 0;
static mut PREV_INDEX2: u8 = 0xFF;
static mut DATA_COUNTER: u8 = 0;
static mut CHECKSUM: u8 = 0;
static mut CURRENT_ADDRESS: u32 = 0;
static mut RX_BYTE: u8 = 0;

// Буферы для пакетов
static mut READ_RX_DATA: [u8; 133] = [0; 133]; // SOH + IDX1 + IDX2 + DATA[128] + CRC[2]

// Управление диагностическими LED
fn toggle_green_led(p: &pac::Peripherals) {
    unsafe {
        p.gpiod.odr().modify(|r, w| w.odr12().bit(!r.odr12().bit()));
    }
}

fn toggle_orange_led(p: &pac::Peripherals) {
    unsafe {
        p.gpiod.odr().modify(|r, w| w.odr13().bit(!r.odr13().bit()));
    }
}

fn toggle_red_led(p: &pac::Peripherals) {
    unsafe {
        p.gpiod.odr().modify(|r, w| w.odr14().bit(!r.odr14().bit()));
    }
}

fn toggle_blue_led(p: &pac::Peripherals) {
    unsafe {
        p.gpiod.odr().modify(|r, w| w.odr15().bit(!r.odr15().bit()));
    }
}

// Включить-выключить LED
fn set_green_led(p: &pac::Peripherals, state: bool) {
    unsafe {
        if state {
            p.gpiod.bsrr().write(|w| w.bs12().set_bit());
        } else {
            p.gpiod.bsrr().write(|w| w.br12().set_bit());
        }
    }
}

fn set_orange_led(p: &pac::Peripherals, state: bool) {
    unsafe {
        if state {
            p.gpiod.bsrr().write(|w| w.bs13().set_bit());
        } else {
            p.gpiod.bsrr().write(|w| w.br13().set_bit());
        }
    }
}

fn set_red_led(p: &pac::Peripherals, state: bool) {
    unsafe {
        if state {
            p.gpiod.bsrr().write(|w| w.bs14().set_bit());
        } else {
            p.gpiod.bsrr().write(|w| w.br14().set_bit());
        }
    }
}

fn set_blue_led(p: &pac::Peripherals, state: bool) {
    unsafe {
        if state {
            p.gpiod.bsrr().write(|w| w.bs15().set_bit());
        } else {
            p.gpiod.bsrr().write(|w| w.br15().set_bit());
        }
    }
}

// Простая задержка
fn delay_ms(ms: u32) {
    let start = systick::get_tick_ms();
    while !systick::wait_ms(start, ms) {
        cortex_m::asm::nop();
    }
}

// Мигнуть LED несколько раз с заданной частотой
fn blink_led_pattern(p: &pac::Peripherals, led: u8, count: u8, delay: u32) {
    for _ in 0..count {
        match led {
            1 => set_green_led(p, true),
            2 => set_orange_led(p, true),
            3 => set_red_led(p, true),
            4 => set_blue_led(p, true),
            _ => {}
        }
        delay_ms(delay);
        
        match led {
            1 => set_green_led(p, false),
            2 => set_orange_led(p, false),
            3 => set_red_led(p, false),
            4 => set_blue_led(p, false),
            _ => {}
        }
        delay_ms(delay);
    }
}

// Специальный паттерн "SOS" для серьезных ошибок (красный LED)
fn blink_sos(p: &pac::Peripherals) {
    for _ in 0..3 {
        blink_led_pattern(p, 3, 3, 100); // Короткие сигналы
        delay_ms(300);
        blink_led_pattern(p, 3, 3, 300); // Длинные сигналы
        delay_ms(300);
        blink_led_pattern(p, 3, 3, 100); // Короткие сигналы
        delay_ms(1000);
    }
}

// Получить текущее состояние XMODEM
pub fn get_state() -> XmodemState {
    unsafe { X_STATE }
}

// Установить состояние XMODEM
pub fn set_state(state: XmodemState) {
    unsafe { X_STATE = state; }
}

// Проверить, идет ли обновление
pub fn is_update_in_progress() -> bool {
    unsafe { UPDATE_IN_PROGRESS }
}

// Функция для отправки строки в TX буфер
pub fn queue_string(tx_buffer: &RingBuffer, s: &str) {
    for byte in s.bytes() {
        tx_buffer.write(byte);
    }
}

// Отправка CAN для отмены передачи - прямой аналог X_SendCan() из C кода
fn send_can(tx_buffer: &RingBuffer) {
    tx_buffer.write(CAN);
    tx_buffer.write(CAN);
    tx_buffer.write(CAN);
}

// Вспомогательная функция для запуска процесса обновления прошивки
pub fn start_update(tx_buffer: &RingBuffer, target_addr: u32, is_app_update: bool) {
    // Сброс состояния
    set_state(XmodemState::WaitSOH);
    unsafe {
        PACKET_RECEIVED = false;
        FIRST_PACKET = false;
        COPY_DATA = true;
        UPDATE_IN_PROGRESS = true;
        
        CURRENT_ADDRESS = target_addr;
        PREV_INDEX1 = 0;
        PREV_INDEX2 = 0xFF;
        DATA_COUNTER = 0;
        CHECKSUM = 0;
        
        // Сброс отладочных счетчиков
        PACKETS_COUNT = 0;
        EOT_RECEIVED = false;
        ACK_SENT_FOR_EOT = false;
        
        // Очистка буфера
        READ_RX_DATA = [0; 133];
    }
    
    // Отправляем сообщение перед началом протокола
    queue_string(tx_buffer, "\r\nStarting firmware update. Send file using XMODEM-CRC protocol...\r\n");
    
    // Отправляем 'C' для запроса XMODEM-CRC передачи
    tx_buffer.write(C);
}

// Функция для расчёта CRC по аналогии с X_CRC16_Calculate
fn calculate_crc16(data: &[u8], count: usize) -> u16 {
    let mut crc: u16 = 0;
    
    for i in 0..count {
        crc ^= (data[i] as u16) << 8;
        
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

// Обработка XMODEM - аналог Download_Firmware из C кода
pub fn process_xmodem(p: &pac::Peripherals, tx_buffer: &RingBuffer, rx_buffer: &RingBuffer) -> bool {
    // Показываем текущее состояние через LED
    unsafe {
        match X_STATE {
            XmodemState::WaitSOH => {
                set_green_led(p, true);
                set_orange_led(p, false);
                set_red_led(p, false);
                set_blue_led(p, false);
            },
            XmodemState::WaitIndex1 => {
                set_green_led(p, false);
                set_orange_led(p, true);
                set_red_led(p, false);
                set_blue_led(p, false);
            },
            XmodemState::WaitIndex2 => {
                set_green_led(p, false);
                set_orange_led(p, false);
                set_red_led(p, true);
                set_blue_led(p, false);
            },
            XmodemState::ReadData => {
                set_green_led(p, false);
                set_orange_led(p, false);
                set_red_led(p, false);
                set_blue_led(p, true);
            },
            XmodemState::WaitCRC => {
                set_green_led(p, true);
                set_orange_led(p, true);
                set_red_led(p, false);
                set_blue_led(p, false);
            },
        }

        // Если EOT получен, но ACK не отправлен, индикатор
        if EOT_RECEIVED && !ACK_SENT_FOR_EOT {
            blink_led_pattern(p, 2, 5, 100); // Быстро мигаем оранжевым - ждем отправки ACK
        }
    }
    
    // В начале КОПИРУЕМ ЛОГИКУ из C кода
    
    // Проверяем EOT в самом начале буфера
    unsafe {
        if rx_buffer.len() > 0 {
            if let Some(byte) = rx_buffer.peek() {
                if byte == EOT {
                    // Диагностика - мигаем синим при получении EOT
                    blink_led_pattern(p, 4, 3, 200);
                    EOT_RECEIVED = true;
                    
                    // Удаляем байт EOT из буфера
                    rx_buffer.read();
                    
                    // Отправляем ACK
                    tx_buffer.write(ACK);
                    // Диагностика - мигаем зеленым при отправке ACK
                    blink_led_pattern(p, 1, 3, 200);
                    ACK_SENT_FOR_EOT = true;
                    
                    // Сообщаем об успешном окончании
                    queue_string(tx_buffer, "\r\nFirmware update completed successfully!\r\n");
                    queue_string(tx_buffer, "\r\nReceived ");
                    
                    // Формируем число пакетов как строку
                    let mut digit_buffer = [0u8; 10];
                    let mut len = 0;
                    let mut val = PACKETS_COUNT;
                    
                    if val == 0 {
                        digit_buffer[0] = b'0';
                        len = 1;
                    } else {
                        while val > 0 {
                            digit_buffer[len] = b'0' + (val % 10) as u8;
                            val /= 10;
                            len += 1;
                        }
                    }
                    
                    // Разворачиваем цифры
                    for i in 0..len/2 {
                        let temp = digit_buffer[i];
                        digit_buffer[i] = digit_buffer[len-1-i];
                        digit_buffer[len-1-i] = temp;
                    }
                    
                    // Добавляем к сообщению
                    for i in 0..len {
                        tx_buffer.write(digit_buffer[i]);
                    }
                    
                    queue_string(tx_buffer, " packets\r\n");
                    
                    // Мигаем всеми LED для индикации успешного завершения
                    set_green_led(p, true);
                    set_red_led(p, false);
                    set_orange_led(p, false);
                    set_blue_led(p, false);
                    
                    delay_ms(500);
                    toggle_green_led(p);
                    toggle_orange_led(p);
                    delay_ms(500);
                    toggle_orange_led(p);
                    toggle_red_led(p);
                    delay_ms(500);
                    toggle_red_led(p);
                    toggle_blue_led(p);
                    delay_ms(500);
                    
                    UPDATE_IN_PROGRESS = false;
                    
                    // Сбрасываем состояние
                    rx_buffer.clear();
                    FIRST_PACKET = false;
                    COPY_DATA = true;
                    PACKET_RECEIVED = false;
                    PREV_INDEX1 = 0;
                    PREV_INDEX2 = 0xFF;
                    DATA_COUNTER = 0;
                    CHECKSUM = 0;
                    
                    return true;
                }
            }
        }
    }
    
    // Спам символами 'C' для инициации коммуникации, если пакет не получен и не первый
    unsafe {
        if !PACKET_RECEIVED && !FIRST_PACKET {
            // Мигаем оранжевым во время отправки C
            toggle_orange_led(p);
            
            tx_buffer.write(C);
            return false;
        }
        
        // Отправляем NAK если пакет не был получен, но мы ждем данные
        if !PACKET_RECEIVED && FIRST_PACKET {
            // Мигаем красным во время отправки NAK
            toggle_red_led(p);
            
            tx_buffer.write(NAK);
            return false;
        }
        
        // Важная проверка: если X_STATE == WAIT_SOH и пакет получен, то отправляем ACK
        if X_STATE == XmodemState::WaitSOH {
            if PACKET_RECEIVED {
                // Мигаем зеленым во время отправки ACK
                toggle_green_led(p);
                
                tx_buffer.write(ACK);
            }
            
            // Проверяем, достаточно ли данных в буфере
            if rx_buffer.len() >= 133 {
                FIRST_PACKET = true;
            }
        }
    }
    
    // Читаем байт из RX буфера - это критично
    if let Some(byte) = rx_buffer.read() {
        unsafe { RX_BYTE = byte; }
    }
    
    // Копируем данные из кольцевого буфера в буфер пакета, если он полный
    unsafe {
        if COPY_DATA && rx_buffer.len() >= 133 {
            // Мигаем синим во время копирования данных
            toggle_blue_led(p);
            
            // Копируем один пакет XMODEM из кольцевого буфера
            for i in 0..133 {
                if let Some(byte) = rx_buffer.read() {
                    READ_RX_DATA[i] = byte;
                }
            }
            
            PACKET_RECEIVED = false;
            
            // Опустошаем кольцевой буфер
            rx_buffer.clear();
        }
    }
    
    // Обработка состояний XMODEM - аналог switch-case в C коде
    match get_state() {
        XmodemState::WaitSOH => {
            unsafe {
                if READ_RX_DATA[0] == SOH {
                    // Мигаем зеленым при получении SOH
                    toggle_green_led(p);
                    
                    set_state(XmodemState::WaitIndex1);
                    PACKET_RECEIVED = true;
                    COPY_DATA = false;
                } else {
                    PACKET_RECEIVED = false;
                }
            }
        },
        XmodemState::WaitIndex1 => {
            unsafe {
                // Если индекс на 1 больше предыдущего, то он верный
                if (PREV_INDEX1 + 1) == READ_RX_DATA[1] {
                    set_state(XmodemState::WaitIndex2);
                    PREV_INDEX1 += 1;
                } else if PREV_INDEX1.wrapping_add(1) == 0 {
                    set_state(XmodemState::WaitIndex2);
                    PREV_INDEX1 = 0;
                } else {
                    // Мигаем красным SOS при ошибке индекса
                    blink_sos(p);
                    
                    // Отправляем отмену
                    send_can(tx_buffer);
                    
                    // Печатаем сообщение об ошибке
                    queue_string(tx_buffer, "\r\nIndex error! Aborting.\r\n");
                    
                    // Сбрасываем состояние
                    UPDATE_IN_PROGRESS = false;
                    return true;
                }
            }
        },
        XmodemState::WaitIndex2 => {
            unsafe {
                // Индекс 2 должен быть равен 255 - индекс1
                if (PREV_INDEX2 - 1) == READ_RX_DATA[2] {
                    set_state(XmodemState::ReadData);
                    PREV_INDEX2 -= 1;
                } else if PREV_INDEX2.wrapping_sub(1) == 255 {
                    set_state(XmodemState::ReadData);
                    PREV_INDEX2 = 255;
                } else {
                    // Мигаем красным SOS при ошибке индекса
                    blink_sos(p);
                    
                    // Отправляем отмену
                    send_can(tx_buffer);
                    
                    // Печатаем сообщение об ошибке
                    queue_string(tx_buffer, "\r\nIndex2 error! Aborting.\r\n");
                    
                    // Сбрасываем состояние
                    UPDATE_IN_PROGRESS = false;
                    return true;
                }
            }
        },
        XmodemState::ReadData => {
            // Мигаем синим при чтении данных
            toggle_blue_led(p);
            
            set_state(XmodemState::WaitCRC);
        },
        XmodemState::WaitCRC => {
            unsafe {
                // Проверка CRC
                let crc_received = ((READ_RX_DATA[131] as u16) << 8) | (READ_RX_DATA[132] as u16);
                let crc_calculated = calculate_crc16(&READ_RX_DATA[3..131], 128);
                
                if crc_received != crc_calculated {
                    // Мигаем красным SOS при ошибке CRC
                    blink_sos(p);
                    
                    // Отправляем отмену при ошибке CRC
                    send_can(tx_buffer);
                    
                    // Печатаем сообщение об ошибке
                    queue_string(tx_buffer, "\r\nCRC error! Aborting.\r\n");
                    
                    // Сбрасываем состояние
                    UPDATE_IN_PROGRESS = false;
                    return true;
                }
                
                // Если CRC верный, продолжаем обработку
                if READ_RX_DATA[1] == 1 || READ_RX_DATA[1] > SOH || (READ_RX_DATA[1] == 0 && READ_RX_DATA[0] != 0) {
                    // Увеличиваем счетчик пакетов для отладки
                    PACKETS_COUNT += 1;
                    
                    // Мигаем зеленым при успешной обработке пакета
                    toggle_green_led(p);
                    
                    // Сбрасываем буферы и готовимся к следующему пакету
                    rx_buffer.clear();
                    
                    for i in 0..READ_RX_DATA.len() {
                        READ_RX_DATA[i] = 0;
                    }
                    
                    PACKET_RECEIVED = true;
                    COPY_DATA = true;
                    set_state(XmodemState::WaitSOH);
                }
            }
        }
    }
    
    false
}