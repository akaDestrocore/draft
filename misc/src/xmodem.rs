#![no_std]

use core::{
    cell::RefCell,
    fmt, 
    mem,
    sync::atomic::{AtomicBool, Ordering}
};
use crate::ring_buffer::RingBuffer;
use crate::image::ImageHeader;
use crate::flash;
use crate::systick;
use rmodem::{Xmodem, XmodemConfig, XmodemVariant};
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
impl From<rmodem::Error> for XmodemError {
    fn from(error: rmodem::Error) -> Self {
        match error {
            rmodem::Error::Crc => XmodemError::Crc,
            rmodem::Error::Sequence => XmodemError::PacketNumber,
            rmodem::Error::Cancelled => XmodemError::Canceled,
            rmodem::Error::Timeout => XmodemError::Timeout,
            rmodem::Error::Unknown => XmodemError::InvalidPacket,
            _ => XmodemError::InvalidPacket,
        }
    }
}

// Адаптер для нашего RingBuffer, чтобы работать с rmodem
struct RingBufferAdapter<'a, F>
where
    F: FnMut(&mut RingBuffer) + 'a,
{
    rx_buffer: &'a mut RingBuffer,
    tx_buffer: &'a mut RingBuffer,
    transmit_fn: &'a mut F,
    last_rx_check: u32,
}

impl<'a, F> rmodem::Serial for RingBufferAdapter<'a, F>
where
    F: FnMut(&mut RingBuffer),
{
    type Error = XmodemError;

    fn read_timeout(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, Self::Error> {
        let start_ms = systick::get_tick_ms();
        let mut bytes_read = 0;
        
        while bytes_read < buf.len() {
            // Запускаем обработчик передачи для поддержания обмена данными
            (self.transmit_fn)(self.tx_buffer);
            
            if let Some(byte) = self.rx_buffer.read() {
                buf[bytes_read] = byte;
                bytes_read += 1;
                // Сбрасываем время для следующего таймаута, т.к. получили данные
                self.last_rx_check = systick::get_tick_ms();
            } else {
                // Проверяем таймаут только если не читаем данные
                let current_ms = systick::get_tick_ms();
                if bytes_read == 0 && current_ms.wrapping_sub(self.last_rx_check) >= timeout_ms {
                    // Полный таймаут только если ничего не прочитали
                    return Err(XmodemError::Timeout);
                } else if bytes_read > 0 {
                    // Если что-то прочитали, возвращаем это
                    break;
                }
                
                // Небольшая задержка, чтобы не перегружать процессор
                cortex_m::asm::delay(1000);
            }
            
            // Защита от бесконечного цикла - общий таймаут
            if systick::get_tick_ms().wrapping_sub(start_ms) > timeout_ms * 3 {
                if bytes_read > 0 {
                    break;
                }
                return Err(XmodemError::Timeout);
            }
        }
        
        Ok(bytes_read)
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        for &byte in buf {
            if !self.tx_buffer.write(byte) {
                return Err(XmodemError::InvalidPacket);
            }
        }
        
        // Запускаем обработчик передачи
        (self.transmit_fn)(self.tx_buffer);
        
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), Self::Error> {
        // Запускаем обработчик передачи для отправки всех данных
        (self.transmit_fn)(self.tx_buffer);
        
        // Ждем небольшую задержку для уверенности, что данные отправлены
        let start_ms = systick::get_tick_ms();
        while !self.tx_buffer.is_empty() && systick::get_tick_ms().wrapping_sub(start_ms) < 100 {
            (self.transmit_fn)(self.tx_buffer);
            cortex_m::asm::delay(1000);
        }
        
        Ok(())
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
    
    let mut adapter = RingBufferAdapter {
        rx_buffer: rx_buf,
        tx_buffer: tx_buf,
        transmit_fn: &mut transmit_fn,
        last_rx_check: systick::get_tick_ms(),
    };
    
    // Конфигурация для XMODEM-1K
    let config = XmodemConfig::new(XmodemVariant::Xmodem1k);
    let mut xmodem = Xmodem::new(&config);
    
    // Буфер для хранения принятых пакетов (1K + запас)
    let mut buffer = [0u8; 1024];
    let mut total_bytes_received = 0;
    let mut flash_addr = target_address;
    
    // Подготавливаем флеш для записи
    let p = unsafe { pac::Peripherals::steal() };
    
    // Начинаем передачу
    let mut header_received = false;
    let mut image_header = ImageHeader::new(0, 0, 0, 0, 0);
    
    // Отправляем NAK для начала передачи
    adapter.write(&[0x15])?; // NAK для стандартного XMODEM
    
    while total_bytes_received < slot_size {
        // Обрабатываем передачу данных
        transmit_fn(tx_buf);
        
        // Принимаем пакет
        let receive_result = xmodem.receive_packet(&mut adapter, &mut buffer);
        
        match receive_result {
            Ok(bytes) => {
                if bytes == 0 {
                    // EOT получен, передача завершена
                    break;
                }
                
                // Первый пакет должен содержать заголовок образа
                if !header_received && bytes >= mem::size_of::<ImageHeader>() {
                    // Копируем заголовок
                    let mut header_bytes = [0u8; mem::size_of::<ImageHeader>()];
                    header_bytes.copy_from_slice(&buffer[..mem::size_of::<ImageHeader>()]);
                    
                    // Парсим заголовок
                    unsafe {
                        image_header = core::ptr::read(header_bytes.as_ptr() as *const ImageHeader);
                    }
                    
                    // Проверяем magic number
                    if image_header.image_magic != expected_magic {
                        return Err(XmodemError::InvalidMagic);
                    }
                    
                    // Проверяем, что версия новее (если имеющаяся прошивка присутствует)
                    if target_address != flash::FLASH_BASE {
                        let mut current_header_bytes = [0u8; mem::size_of::<ImageHeader>()];
                        flash::read(target_address, &mut current_header_bytes);
                        
                        let current_header: ImageHeader = unsafe {
                            core::ptr::read(current_header_bytes.as_ptr() as *const ImageHeader)
                        };
                        
                        if current_header.is_valid() && !image_header.is_newer_than(&current_header) {
                            return Err(XmodemError::OlderVersion);
                        }
                    }
                    
                    header_received = true;
                }
                
                // Записываем данные во флеш
                if flash::write(&p, &buffer[..bytes], flash_addr) != 0 {
                    return Err(XmodemError::FlashError);
                }
                
                flash_addr += bytes as u32;
                total_bytes_received += bytes as u32;
                
                // Обеспечиваем передачу буферизованных ответов
                transmit_fn(tx_buf);
                
                // Небольшая задержка для стабильности
                let start_ms = systick::get_tick_ms();
                while systick::get_tick_ms().wrapping_sub(start_ms) < 5 {
                    cortex_m::asm::nop();
                }
            },
            Err(rmodem::Error::EndOfTransmission) => {
                // Нормальное завершение передачи
                break;
            },
            Err(err) => {
                return Err(err.into());
            }
        }
    }
    
    // Завершаем передачу и отправляем ACK
    adapter.write(&[0x06])?; // ACK
    
    // Очистка буферов
    rx_buf.clear();
    
    Ok(())
}