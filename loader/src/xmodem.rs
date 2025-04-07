use misc::{flash, systick};
use stm32f4 as pac;

// XMODEM protocol constants
pub const SOH: u8 = 0x01;  // Start of header (128 bytes)
pub const STX: u8 = 0x02;  // Start of header (1K bytes)
pub const EOT: u8 = 0x04;  // End of transmission
pub const ACK: u8 = 0x06;  // Acknowledge
pub const NAK: u8 = 0x15;  // Negative acknowledge
pub const CAN: u8 = 0x18;  // Cancel (24)
pub const C: u8 = 0x43;    // ASCII 'C' for CRC mode

// Максимальное количество попыток для пакета
const MAX_RETRIES: u8 = 10;
// Интервал отправки символа 'C' для инициации передачи (мс)
const C_INTERVAL_MS: u32 = 1000;

// XMODEM state machine
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum XmodemState {
    Idle,           // Не принимаем данные
    WaitSOH,        // Ожидаем начала пакета (SOH/STX)
    WaitIndex1,     // Ожидаем номер пакета
    WaitIndex2,     // Ожидаем комплементарный номер пакета
    ReceiveData,    // Получение данных
    CheckCrc,       // Проверка CRC
    FinishTransfer, // Завершение передачи
    Error,          // Состояние ошибки
}

#[derive(Debug)]
pub enum XmodemError {
    InvalidPacket,
    Cancelled,
    Timeout,
    TransferComplete,
    FlashWriteError,
}

/// Функция для вычисления CRC-16 CCITT для данных XMODEM
fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    
    for &byte in data {
        crc ^= (byte as u16) << 8;
        
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    crc
}

pub struct XmodemManager {
    state: XmodemState,           // Текущее состояние
    target_address: u32,          // Целевой адрес для прошивки
    current_address: u32,         // Текущий адрес для записи
    buffer: [u8; 133],            // Буфер для пакета (128 данных + 5 служебных байтов)
    buffer_index: usize,          // Индекс в буфере
    data_size: usize,             // Размер данных в пакете (128 или 1024)
    packet_number: u8,            // Ожидаемый номер пакета
    prev_packet_number: u8,       // Предыдущий номер пакета
    last_c_time: u32,             // Время последней отправки 'C'
    response: Option<u8>,         // Байт для отправки
    retries: u8,                  // Счетчик попыток
    first_packet: bool,           // Флаг первого пакета
    last_sector_erased: u32,      // Последний стертый сектор
}

impl XmodemManager {
    pub fn new() -> Self {
        Self {
            state: XmodemState::Idle,
            target_address: 0,
            current_address: 0,
            buffer: [0; 133],
            buffer_index: 0,
            data_size: 128,        // По умолчанию 128 байт
            packet_number: 1,      // Начальный номер пакета
            prev_packet_number: 0, // Предыдущий номер пакета
            last_c_time: 0,
            response: None,
            retries: 0,
            first_packet: true,
            last_sector_erased: 0,
        }
    }

    /// Инициализация приема по XMODEM
    pub fn start(&mut self, address: u32) {
        self.state = XmodemState::WaitSOH;
        self.target_address = address;
        self.current_address = address;
        self.buffer_index = 0;
        self.packet_number = 1;
        self.prev_packet_number = 0;
        self.last_c_time = systick::get_tick_ms();
        self.response = Some(C);  // Отправить начальный 'C'
        self.retries = 0;
        self.first_packet = true;
        
        // Стираем первый сектор перед началом приема
        let result = unsafe { 
            flash::erase_sector(&pac::Peripherals::steal(), address) 
        };
        
        if result > 0 {
            self.last_sector_erased = address;
        } else {
            self.state = XmodemState::Error;
        }
    }

    /// Получить текущее состояние
    pub fn get_state(&self) -> XmodemState {
        self.state
    }

    /// Проверка необходимости отправки 'C' для инициации передачи
    pub fn should_send_c(&mut self) -> bool {
        if self.state == XmodemState::WaitSOH {
            let current_time = systick::get_tick_ms();
            if current_time.wrapping_sub(self.last_c_time) >= C_INTERVAL_MS {
                self.last_c_time = current_time;
                self.response = Some(C);
                return true;
            }
        }
        false
    }

    /// Получить байт для отправки
    pub fn get_response(&mut self) -> Option<u8> {
        let resp = self.response;
        self.response = None;
        resp
    }

    /// Обработка принятого байта
    pub fn process_byte(&mut self, byte: u8) -> Result<bool, XmodemError> {
        match self.state {
            XmodemState::Idle => {
                // Не выполняем прием
                Ok(false)
            },
            
            XmodemState::WaitSOH => {
                match byte {
                    SOH => {
                        // Начало пакета 128 байт
                        self.buffer[0] = byte;
                        self.buffer_index = 1;
                        self.data_size = 128;
                        self.state = XmodemState::WaitIndex1;
                        Ok(false)  // Еще не готовы отправить ответ
                    },
                    STX => {
                        // Начало пакета 1024 байт
                        self.buffer[0] = byte;
                        self.buffer_index = 1;
                        self.data_size = 1024;
                        self.state = XmodemState::WaitIndex1;
                        Ok(false)
                    },
                    EOT => {
                        // Конец передачи
                        self.response = Some(ACK);
                        self.state = XmodemState::Idle;
                        Err(XmodemError::TransferComplete)
                    },
                    CAN => {
                        // Передача отменена
                        self.state = XmodemState::Idle;
                        Err(XmodemError::Cancelled)
                    },
                    _ => {
                        // Игнорируем другие байты
                        Ok(false)
                    }
                }
            },
            
            XmodemState::WaitIndex1 => {
                // Получен номер пакета
                self.buffer[1] = byte;
                self.buffer_index = 2;
                self.state = XmodemState::WaitIndex2;
                Ok(false)
            },
            
            XmodemState::WaitIndex2 => {
                // Получен комплементарный номер пакета
                self.buffer[2] = byte;
                self.buffer_index = 3;
                
                // Проверяем, что номер пакета и его комплемент корректны
                if self.buffer[1] + self.buffer[2] != 255 {
                    // Ошибка в номере пакета
                    self.response = Some(NAK);
                    self.state = XmodemState::WaitSOH;
                    self.retries += 1;
                    
                    if self.retries > MAX_RETRIES {
                        self.state = XmodemState::Error;
                        return Err(XmodemError::InvalidPacket);
                    }
                    
                    return Ok(true);
                }
                
                // Проверяем номер пакета
                if self.first_packet {
                    // Для первого пакета принимаем любой номер (в пределах разумного)
                    self.packet_number = self.buffer[1];
                    self.first_packet = false;
                } else if self.buffer[1] != self.packet_number {
                    // Несоответствие номера пакета ожидаемому
                    // Если это повторный пакет, принимаем его
                    if self.buffer[1] == self.prev_packet_number {
                        // Это повторный пакет, который мы уже получили, просто подтверждаем
                        self.response = Some(ACK);
                        self.state = XmodemState::WaitSOH;
                        return Ok(true);
                    } else {
                        // Неправильный номер пакета
                        self.response = Some(NAK);
                        self.state = XmodemState::WaitSOH;
                        self.retries += 1;
                        
                        if self.retries > MAX_RETRIES {
                            self.state = XmodemState::Error;
                            return Err(XmodemError::InvalidPacket);
                        }
                        
                        return Ok(true);
                    }
                }
                
                // Номер пакета корректный, переходим к приему данных
                self.state = XmodemState::ReceiveData;
                Ok(false)
            },
            
            XmodemState::ReceiveData => {
                // Получение данных пакета
                self.buffer[self.buffer_index] = byte;
                self.buffer_index += 1;
                
                // Проверяем, получили ли все данные и CRC
                if self.buffer_index == 3 + self.data_size + 2 {
                    // Получен весь пакет (SOH + номер пакета + комплемент + данные + CRC)
                    self.state = XmodemState::CheckCrc;
                    
                    // Проверяем CRC
                    let crc_received = ((self.buffer[3 + self.data_size] as u16) << 8) | 
                                       (self.buffer[3 + self.data_size + 1] as u16);
                    
                    let crc_calculated = calculate_crc16(&self.buffer[3..3 + self.data_size]);
                    
                    if crc_received != crc_calculated {
                        // Ошибка CRC
                        self.response = Some(NAK);
                        self.state = XmodemState::WaitSOH;
                        self.retries += 1;
                        
                        if self.retries > MAX_RETRIES {
                            self.state = XmodemState::Error;
                            return Err(XmodemError::InvalidPacket);
                        }
                        
                        return Ok(true);
                    }
                    
                    // Проверяем, нужно ли стереть следующий сектор
                    let next_address = self.current_address + self.data_size as u32;
                    let current_sector = self.current_address & 0xFFFF0000;
                    let next_sector = next_address & 0xFFFF0000;
                    
                    if current_sector != next_sector && next_sector != self.last_sector_erased {
                        // Нужно стереть новый сектор
                        let result = unsafe { 
                            flash::erase_sector(&pac::Peripherals::steal(), next_sector) 
                        };
                        
                        if result == 0 {
                            // Ошибка стирания
                            self.state = XmodemState::Error;
                            return Err(XmodemError::FlashWriteError);
                        }
                        
                        self.last_sector_erased = next_sector;
                    }
                    
                    // Записываем данные во flash
                    let result = unsafe { 
                        flash::write(&pac::Peripherals::steal(), 
                                    &self.buffer[3..3 + self.data_size], 
                                    self.current_address)
                    };
                    
                    if result != 0 {
                        // Ошибка записи
                        self.state = XmodemState::Error;
                        return Err(XmodemError::FlashWriteError);
                    }
                    
                    // Сохраняем предыдущий номер пакета
                    self.prev_packet_number = self.packet_number;
                    
                    // Увеличиваем адрес и номер пакета
                    self.current_address += self.data_size as u32;
                    self.packet_number = self.packet_number.wrapping_add(1);
                    
                    // Успешно приняли и записали пакет
                    self.response = Some(ACK);
                    self.state = XmodemState::WaitSOH;
                    self.buffer_index = 0;
                    self.retries = 0;
                    
                    return Ok(true);
                }
                
                Ok(false)
            },
            
            XmodemState::CheckCrc => {
                // Это состояние не используется в такой реализации
                // (CRC проверяется в ReceiveData)
                self.state = XmodemState::WaitSOH;
                Ok(false)
            },
            
            XmodemState::FinishTransfer => {
                // Завершение передачи
                self.state = XmodemState::Idle;
                self.response = Some(ACK);
                Err(XmodemError::TransferComplete)
            },
            
            XmodemState::Error => {
                // Состояние ошибки
                self.state = XmodemState::Idle;
                Ok(false)
            }
        }
    }
}