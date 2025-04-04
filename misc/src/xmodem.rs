#![no_std]

use core::{
    fmt, 
    mem,
};
use crate::ring_buffer::RingBuffer;
use crate::image::ImageHeader;
use crate::flash;
use crate::systick;
use rmodem::{
    Control, Error as RmodemError, Protocol, Result as RmodemResult, Sequence, 
    XmodemData, XmodemPacket, OneKData, XmodemOneKPacket, XmodemCrcPacket,
    ACK, CAN, EOT, IDLE, NAK, SOH, STX
};
use stm32f4 as pac;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XmodemError {
    Crc,
    PacketNumber,
    Canceled,
    InvalidMagic,
    OlderVersion,
    Timeout,
    FlashError,
    InvalidPacket,
    EOT,
}

impl fmt::Display for XmodemError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            XmodemError::Crc => write!(f, "CRC error"),
            XmodemError::PacketNumber => write!(f, "Packet number error"),
            XmodemError::Canceled => write!(f, "Transfer canceled"),
            XmodemError::InvalidMagic => write!(f, "Invalid firmware magic"),
            XmodemError::OlderVersion => write!(f, "Older firmware version"),
            XmodemError::Timeout => write!(f, "Timeout"),
            XmodemError::FlashError => write!(f, "Flash write error"),
            XmodemError::InvalidPacket => write!(f, "Invalid packet"),
            XmodemError::EOT => write!(f, "Unexpected EOT"),
        }
    }
}

// Convert rmodem errors to our custom errors
impl From<RmodemError> for XmodemError {
    fn from(error: RmodemError) -> Self {
        match error {
            RmodemError::ChecksumMismatch => XmodemError::Crc,
            RmodemError::SequenceMismatch => XmodemError::PacketNumber,
            RmodemError::Cancelled => XmodemError::Canceled,
            _ => XmodemError::InvalidPacket,
        }
    }
}

pub fn receive_firmware<F>(
    rx_buf: &mut RingBuffer,
    tx_buf: &mut RingBuffer,
    target_address: u32,
    expected_magic: u32,
    slot_size: u32,
    mut transmit_fn: F
) -> Result<(), XmodemError>
where
    F: FnMut(&mut RingBuffer)
{
    // Очищаем буферы перед началом передачи
    rx_buf.clear();
    
    // Буфер для хранения принятых пакетов
    let mut total_bytes_received = 0;
    let mut flash_addr = target_address;
    
    // Подготавливаем флеш для записи
    let p = unsafe { pac::Peripherals::steal() };
    
    // Начинаем передачу
    let mut header_received = false;
    let mut image_header = ImageHeader::new(0, 0, 0, 0, 0);
    let mut sequence = Sequence::new();
    
    // Отправляем 'C' для начала передачи с XMODEM-CRC или XMODEM-1K
    tx_buf.write(IDLE);
    transmit_fn(tx_buf);
    
    // Таймаут для начала передачи - 30 секунд
    let start_ms = systick::get_tick_ms();
    let timeout_ms = 30000;
    
    // Основной цикл приема данных
    while total_bytes_received < slot_size {
        // Обработка передачи
        transmit_fn(tx_buf);
        
        // Проверяем, есть ли данные для чтения
        if rx_buf.is_empty() {
            // Проверяем таймаут
            if systick::get_tick_ms().wrapping_sub(start_ms) > timeout_ms {
                return Err(XmodemError::Timeout);
            }
            
            // Небольшая задержка, чтобы не перегружать процессор
            cortex_m::asm::nop();
            continue;
        }
        
        // Определяем тип пакета по первому байту
        let first_byte = match rx_buf.peek() {
            Some(byte) => byte,
            None => continue,
        };
        
        match first_byte {
            SOH => {
                // Стандартный пакет XMODEM (128 байт)
                if rx_buf.len() < 132 { // SOH + seq + ~seq + 128 data + 2 CRC
                    continue; // Ждем полный пакет
                }
                
                // Подготовка данных для парсинга пакета
                let mut packet_bytes = [0u8; 132];
                for i in 0..132 {
                    packet_bytes[i] = match rx_buf.read() {
                        Some(byte) => byte,
                        None => return Err(XmodemError::InvalidPacket),
                    };
                }
                
                // Проверка и обработка пакета
                match XmodemCrcPacket::from_bytes(&packet_bytes) {
                    Ok(packet) => {
                        // Проверка последовательности пакетов
                        if packet.sequence() != sequence {
                            // Отправляем NAK для переповтора пакета
                            tx_buf.write(NAK);
                            transmit_fn(tx_buf);
                            continue;
                        }
                        
                        // Получаем данные
                        let data = packet.data().as_ref();
                        
                        // Обработка первого пакета с заголовком
                        if !header_received && total_bytes_received == 0 {
                            if data.len() >= mem::size_of::<ImageHeader>() {
                                // Копируем заголовок
                                let mut header_bytes = [0u8; mem::size_of::<ImageHeader>()];
                                header_bytes.copy_from_slice(&data[..mem::size_of::<ImageHeader>()]);
                                
                                // Парсим заголовок
                                unsafe {
                                    image_header = core::ptr::read(header_bytes.as_ptr() as *const ImageHeader);
                                }
                                
                                // Проверяем magic number
                                if image_header.image_magic != expected_magic {
                                    // Отменяем передачу
                                    tx_buf.write(CAN);
                                    tx_buf.write(CAN);
                                    transmit_fn(tx_buf);
                                    return Err(XmodemError::InvalidMagic);
                                }
                                
                                // Проверяем, что версия новее
                                if target_address != flash::FLASH_BASE {
                                    let mut current_header_bytes = [0u8; mem::size_of::<ImageHeader>()];
                                    flash::read(target_address, &mut current_header_bytes);
                                    
                                    let current_header: ImageHeader = unsafe {
                                        core::ptr::read(current_header_bytes.as_ptr() as *const ImageHeader)
                                    };
                                    
                                    if current_header.is_valid() && !image_header.is_newer_than(&current_header) {
                                        // Отменяем передачу
                                        tx_buf.write(CAN);
                                        tx_buf.write(CAN);
                                        transmit_fn(tx_buf);
                                        return Err(XmodemError::OlderVersion);
                                    }
                                }
                                
                                header_received = true;
                            }
                        }
                        
                        // Записываем данные во флеш
                        if flash::write(&p, data, flash_addr) != 0 {
                            // Отменяем передачу при ошибке записи
                            tx_buf.write(CAN);
                            tx_buf.write(CAN);
                            transmit_fn(tx_buf);
                            return Err(XmodemError::FlashError);
                        }
                        
                        // Обновляем адрес и счетчик
                        flash_addr += data.len() as u32;
                        total_bytes_received += data.len() as u32;
                        
                        // Инкрементируем последовательность
                        sequence.increment();
                        
                        // Отправляем ACK
                        tx_buf.write(ACK);
                        transmit_fn(tx_buf);
                    },
                    Err(_) => {
                        // Ошибка в пакете - отправляем NAK
                        tx_buf.write(NAK);
                        transmit_fn(tx_buf);
                    }
                }
            },
            STX => {
                // Пакет XMODEM-1K (1024 байт)
                if rx_buf.len() < 1029 { // STX + seq + ~seq + 1024 data + 2 CRC
                    continue; // Ждем полный пакет
                }
                
                // Подготовка данных для парсинга пакета
                let mut packet_bytes = [0u8; 1029];
                for i in 0..1029 {
                    packet_bytes[i] = match rx_buf.read() {
                        Some(byte) => byte,
                        None => return Err(XmodemError::InvalidPacket),
                    };
                }
                
                // Проверка и обработка пакета
                match XmodemOneKPacket::from_bytes(&packet_bytes) {
                    Ok(packet) => {
                        // Проверка последовательности пакетов
                        if packet.sequence() != sequence {
                            // Отправляем NAK для переповтора пакета
                            tx_buf.write(NAK);
                            transmit_fn(tx_buf);
                            continue;
                        }
                        
                        // Получаем данные
                        let data = packet.data().as_ref();
                        
                        // Обработка первого пакета с заголовком
                        if !header_received && total_bytes_received == 0 {
                            if data.len() >= mem::size_of::<ImageHeader>() {
                                // Копируем заголовок
                                let mut header_bytes = [0u8; mem::size_of::<ImageHeader>()];
                                header_bytes.copy_from_slice(&data[..mem::size_of::<ImageHeader>()]);
                                
                                // Парсим заголовок
                                unsafe {
                                    image_header = core::ptr::read(header_bytes.as_ptr() as *const ImageHeader);
                                }
                                
                                // Проверяем magic number
                                if image_header.image_magic != expected_magic {
                                    // Отменяем передачу
                                    tx_buf.write(CAN);
                                    tx_buf.write(CAN);
                                    transmit_fn(tx_buf);
                                    return Err(XmodemError::InvalidMagic);
                                }
                                
                                // Проверяем, что версия новее
                                if target_address != flash::FLASH_BASE {
                                    let mut current_header_bytes = [0u8; mem::size_of::<ImageHeader>()];
                                    flash::read(target_address, &mut current_header_bytes);
                                    
                                    let current_header: ImageHeader = unsafe {
                                        core::ptr::read(current_header_bytes.as_ptr() as *const ImageHeader)
                                    };
                                    
                                    if current_header.is_valid() && !image_header.is_newer_than(&current_header) {
                                        // Отменяем передачу
                                        tx_buf.write(CAN);
                                        tx_buf.write(CAN);
                                        transmit_fn(tx_buf);
                                        return Err(XmodemError::OlderVersion);
                                    }
                                }
                                
                                header_received = true;
                            }
                        }
                        
                        // Записываем данные во флеш
                        if flash::write(&p, data, flash_addr) != 0 {
                            // Отменяем передачу при ошибке записи
                            tx_buf.write(CAN);
                            tx_buf.write(CAN);
                            transmit_fn(tx_buf);
                            return Err(XmodemError::FlashError);
                        }
                        
                        // Обновляем адрес и счетчик
                        flash_addr += data.len() as u32;
                        total_bytes_received += data.len() as u32;
                        
                        // Инкрементируем последовательность
                        sequence.increment();
                        
                        // Отправляем ACK
                        tx_buf.write(ACK);
                        transmit_fn(tx_buf);
                    },
                    Err(_) => {
                        // Ошибка в пакете - отправляем NAK
                        tx_buf.write(NAK);
                        transmit_fn(tx_buf);
                    }
                }
            },
            EOT => {
                // Конец передачи
                rx_buf.read(); // Удаляем EOT из буфера
                
                // Подтверждаем получение
                tx_buf.write(ACK);
                transmit_fn(tx_buf);
                
                // Передача завершена успешно
                break;
            },
            CAN => {
                // Отмена передачи от отправителя
                rx_buf.read(); // Удаляем CAN из буфера
                
                // Проверяем, есть ли второй CAN (по протоколу)
                if rx_buf.peek() == Some(CAN) {
                    rx_buf.read(); // Удаляем второй CAN
                    return Err(XmodemError::Canceled);
                }
            },
            _ => {
                // Неизвестный или неожиданный байт
                rx_buf.read(); // Удаляем его из буфера
            }
        }
    }
    
    // Очищаем буферы
    rx_buf.clear();
    
    Ok(())
}