use misc::{flash, systick};
use misc::image::{ImageHeader, IMAGE_MAGIC_APP, IMAGE_MAGIC_UPDATER, IMAGE_TYPE_APP, IMAGE_TYPE_UPDATER};
use stm32f4 as pac;
use cortex_m::asm;
use core::mem;

// XMODEM protocol constants
pub const SOH: u8 = 0x01;  // Start of header (128 bytes)
pub const STX: u8 = 0x02;  // Start of header (1K bytes)
pub const EOT: u8 = 0x04;  // End of transmission
pub const ACK: u8 = 0x06;  // Acknowledge
pub const NAK: u8 = 0x15;  // Negative acknowledge
pub const CAN: u8 = 0x18;  // Cancel (24)
pub const C: u8 = 0x43;    // ASCII 'C' for CRC mode

// XMODEM state machine
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum XmodemState {
    Idle,           // Не принимаем данные
    WaitingForData, // Ожидаем данные (отправили 'C' или NAK)
    ReceivingData,  // Получаем данные
    Error,          // Состояние ошибки
    Complete,       // Завершение передачи
}

#[derive(Debug)]
pub enum XmodemError {
    InvalidPacket,
    Cancelled,
    Timeout,
    TransferComplete,
    FlashWriteError,
    InvalidMagic,     // Неверное магическое число
    OlderVersion,     // Версия прошивки старее текущей
}

// Для отладочных целей - вынести важные значения
static mut DEBUG_PACKET_NUM: u8 = 0;
static mut DEBUG_COMPLEMENT: u8 = 0;
static mut DEBUG_CHECKSUM_RECV: u8 = 0; 
static mut DEBUG_CHECKSUM_CALC: u8 = 0;
static mut DEBUG_FLASH_RESULT: u8 = 0;
static mut DEBUG_CURRENT_STATE: u8 = 0;
static mut DEBUG_LAST_BYTE: u8 = 0;
static mut DEBUG_ERROR_CODE: u8 = 0;
static mut DEBUG_MAGIC: u32 = 0;
static mut DEBUG_VERSION: [u8; 3] = [0, 0, 0];

#[inline(never)]
fn breakpoint_helper(val: u8) -> u8 {
    // Нужно для отладки - создает точку остановки, которую компилятор не оптимизирует
    asm::nop();
    val
}

// Стандартный расчет контрольной суммы для XMODEM
fn calculate_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &x| acc.wrapping_add(x))
}

// Проверка валидности образа
fn validate_image_header(data: &[u8], expected_magic: u32) -> Result<(), XmodemError> {
    if data.len() < mem::size_of::<ImageHeader>() {
        return Err(XmodemError::InvalidPacket);
    }

    // Чтение заголовка из данных
    let header = unsafe {
        let header_ptr = data.as_ptr() as *const ImageHeader;
        *header_ptr
    };

    // Сохраняем для отладки
    unsafe {
        DEBUG_MAGIC = header.image_magic;
        DEBUG_VERSION[0] = header.version_major;
        DEBUG_VERSION[1] = header.version_minor;
        DEBUG_VERSION[2] = header.version_patch;
    }

    // Проверка магического числа
    if header.image_magic != expected_magic {
        return Err(XmodemError::InvalidMagic);
    }

    // Тут можно добавить проверку версии, сравнив с текущей
    // Например, прочитать текущую установленную версию из флеш-памяти
    // и сравнить с header.version_major, header.version_minor, header.version_patch

    Ok(())
}

pub struct XmodemManager {
    state: XmodemState,        // Текущее состояние
    target_address: u32,       // Целевой адрес для прошивки
    current_address: u32,      // Текущий адрес для записи
    packet_number: u8,         // Ожидаемый номер пакета
    last_poll_time: u32,       // Время последней отправки NAK/C
    buffer: [u8; 132],         // Буфер для данных пакета (1+1+1+128+1)
    buffer_index: usize,       // Индекс в буфере
    last_sector_erased: u32,   // Последний стертый сектор
    next_byte_to_send: Option<u8>, // Следующий байт для отправки
    packet_count: u16,         // Счетчик принятых пакетов
    use_crc: bool,             // Использовать ли CRC вместо простой контрольной суммы
    expected_magic: u32,       // Ожидаемое магическое число образа
    first_packet_validated: bool, // Флаг, что первый пакет был проверен
}

impl XmodemManager {
    pub fn new() -> Self {
        Self {
            state: XmodemState::Idle,
            target_address: 0,
            current_address: 0,
            packet_number: 1,
            last_poll_time: 0,
            buffer: [0; 132],
            buffer_index: 0,
            last_sector_erased: 0,
            next_byte_to_send: None,
            packet_count: 0,
            use_crc: false,
            expected_magic: 0,
            first_packet_validated: false,
        }
    }

    /// Начать прием XMODEM
    pub fn start(&mut self, address: u32) {
        let _ = breakpoint_helper(1); // Брейкпоинт: XMODEM Start
        
        self.state = XmodemState::WaitingForData;
        self.target_address = address;
        self.current_address = address;
        self.packet_number = 1;
        self.last_poll_time = systick::get_tick_ms();
        self.next_byte_to_send = Some(NAK);  // Стандартный XMODEM начинается с NAK
        self.buffer_index = 0;
        self.packet_count = 0;
        self.use_crc = false;
        self.first_packet_validated = false;
        
        // Устанавливаем ожидаемое магическое число в зависимости от адреса
        if address == crate::APP_ADDR {
            self.expected_magic = IMAGE_MAGIC_APP;
        } else if address == crate::UPDATER_ADDR {
            self.expected_magic = IMAGE_MAGIC_UPDATER;
        } else {
            // Неизвестный адрес - ошибка
            unsafe { DEBUG_ERROR_CODE = 10; }
            self.state = XmodemState::Error;
            return;
        }
        
        // Стираем первый сектор перед началом записи
        let result = unsafe { 
            flash::erase_sector(&pac::Peripherals::steal(), address) 
        };
        
        if result > 0 {
            self.last_sector_erased = address;
        } else {
            let _ = breakpoint_helper(2); // Брейкпоинт: Erase Error
            self.state = XmodemState::Error;
        }
    }

    /// Получить текущее состояние
    pub fn get_state(&self) -> XmodemState {
        self.state
    }

    /// Проверка необходимости отправки NAK/C
    pub fn should_send_c(&mut self) -> bool {
        if self.state == XmodemState::WaitingForData {
            let current_time = systick::get_tick_ms();
            if current_time.wrapping_sub(self.last_poll_time) >= 3000 {
                self.last_poll_time = current_time;
                
                // В стандартном режиме отправляем NAK, в CRC режиме отправляем 'C'
                if self.use_crc {
                    self.next_byte_to_send = Some(C);
                    let _ = breakpoint_helper(3); // Брейкпоинт: Sending 'C'
                } else {
                    self.next_byte_to_send = Some(NAK);
                    let _ = breakpoint_helper(4); // Брейкпоинт: Sending NAK
                }
                
                return true;
            }
        }
        false
    }

    /// Получить байт для отправки
    pub fn get_response(&mut self) -> Option<u8> {
        let response = self.next_byte_to_send;
        if let Some(byte) = response {
            unsafe { DEBUG_LAST_BYTE = byte; }
            let _ = breakpoint_helper(5); // Брейкпоинт: Sending byte
        }
        self.next_byte_to_send = None;
        response
    }

    /// Количество успешно принятых пакетов
    pub fn get_packet_count(&self) -> u16 {
        self.packet_count
    }

    /// Обработка принятого байта
    pub fn process_byte(&mut self, byte: u8) -> Result<bool, XmodemError> {
        unsafe { 
            DEBUG_LAST_BYTE = byte;
            DEBUG_CURRENT_STATE = match self.state {
                XmodemState::Idle => 0,
                XmodemState::WaitingForData => 1,
                XmodemState::ReceivingData => 2,
                XmodemState::Error => 3,
                XmodemState::Complete => 4,
            };
        }
        
        match self.state {
            XmodemState::Idle => {
                // Нет активной передачи
                Ok(false)
            },
            
            XmodemState::WaitingForData => {
                // Ожидаем начала передачи (SOH, STX или EOT)
                match byte {
                    SOH => {
                        // Сохраняем для отладки, чтобы смотреть в окне watch
                        unsafe { 
                            DEBUG_PACKET_NUM = self.packet_count as u8;
                        }
                        let _ = breakpoint_helper(10); // Брейкпоинт: SOH received
                        
                        // Начало пакета - 128 байт
                        self.state = XmodemState::ReceivingData;
                        self.buffer[0] = byte;
                        self.buffer_index = 1;
                        Ok(false)
                    },
                    STX => {
                        let _ = breakpoint_helper(11); // Брейкпоинт: STX received
                        
                        // Начало 1K пакета - не поддерживается в этой версии
                        self.next_byte_to_send = Some(NAK);
                        Ok(true)
                    },
                    EOT => {
                        let _ = breakpoint_helper(12); // Брейкпоинт: EOT received
                        
                        // Конец передачи
                        self.next_byte_to_send = Some(ACK);
                        self.state = XmodemState::Complete;
                        Err(XmodemError::TransferComplete)
                    },
                    CAN => {
                        let _ = breakpoint_helper(13); // Брейкпоинт: CAN received
                        
                        // Отмена передачи
                        self.state = XmodemState::Error;
                        Err(XmodemError::Cancelled)
                    },
                    _ => {
                        // Игнорируем другие байты в этом состоянии
                        Ok(false)
                    }
                }
            },
            
            XmodemState::ReceivingData => {
                // Сохраняем байт в буфер
                self.buffer[self.buffer_index] = byte;
                self.buffer_index += 1;
                
                // Проверяем, получили ли весь пакет
                // SOH(1) + номер пакета(1) + ~номер(1) + данные(128) + контрольная сумма(1) = 132 байта
                if self.buffer_index == 132 {
                    let _ = breakpoint_helper(20); // Брейкпоинт: Full packet received
                    
                    let packet_num = self.buffer[1];
                    let packet_num_complement = self.buffer[2];
                    
                    // Сохраняем для отладки через окно watch
                    unsafe {
                        DEBUG_PACKET_NUM = packet_num;
                        DEBUG_COMPLEMENT = packet_num_complement;
                    }
                    
                    // Проверка соответствия номера пакета и его комплемента
                    if packet_num + packet_num_complement != 255 {
                        let _ = breakpoint_helper(21); // Брейкпоинт: Invalid packet numbers
                        
                        // Некорректный номер пакета
                        self.next_byte_to_send = Some(NAK);
                        self.state = XmodemState::WaitingForData;
                        self.buffer_index = 0;
                        return Ok(true);
                    }
                    
                    // Проверка соответствия ожидаемому номеру пакета
                    if packet_num != self.packet_number {
                        // Сохраняем для отладки
                        unsafe {
                            DEBUG_PACKET_NUM = packet_num;
                            DEBUG_COMPLEMENT = self.packet_number;
                        }
                        let _ = breakpoint_helper(22); // Брейкпоинт: Unexpected packet number
                        
                        // Неправильный номер пакета
                        self.next_byte_to_send = Some(NAK);
                        self.state = XmodemState::WaitingForData;
                        self.buffer_index = 0;
                        return Ok(true);
                    }
                    
                    // Брейкпоинт для пакета №7
                    if packet_num == 7 {
                        let _ = breakpoint_helper(23); // Брейкпоинт: Processing packet #7
                    }
                    
                    // Проверка контрольной суммы
                    let received_checksum = self.buffer[131];
                    let calculated_checksum = calculate_checksum(&self.buffer[3..131]);
                    
                    // Сохраняем для отладки
                    unsafe {
                        DEBUG_CHECKSUM_RECV = received_checksum;
                        DEBUG_CHECKSUM_CALC = calculated_checksum;
                    }
                    
                    if received_checksum != calculated_checksum {
                        let _ = breakpoint_helper(24); // Брейкпоинт: Checksum mismatch
                        
                        // Ошибка контрольной суммы
                        self.next_byte_to_send = Some(NAK);
                        self.state = XmodemState::WaitingForData;
                        self.buffer_index = 0;
                        return Ok(true);
                    }
                    
                    // Для первого пакета проверяем магическое число и версию
                    if packet_num == 1 && !self.first_packet_validated {
                        let _ = breakpoint_helper(40); // Брейкпоинт: Validating first packet
                        
                        // Проверка магического числа и версии
                        match validate_image_header(&self.buffer[3..131], self.expected_magic) {
                            Ok(_) => {
                                // Проверка успешна
                                self.first_packet_validated = true;
                                let _ = breakpoint_helper(41); // Брейкпоинт: Magic valid
                            },
                            Err(e) => {
                                // Ошибка валидации образа
                                unsafe { 
                                    match e {
                                        XmodemError::InvalidMagic => DEBUG_ERROR_CODE = 50,
                                        XmodemError::OlderVersion => DEBUG_ERROR_CODE = 51,
                                        _ => DEBUG_ERROR_CODE = 52,
                                    }
                                }
                                let _ = breakpoint_helper(42); // Брейкпоинт: Magic invalid
                                
                                self.state = XmodemState::Error;
                                return Err(e);
                            }
                        }
                    }
                    
                    // Проверка необходимости стирания следующего сектора
                    let next_address = self.current_address + 128;
                    let current_sector = self.current_address & 0xFFFF0000;
                    let next_sector = next_address & 0xFFFF0000;
                    
                    if current_sector != next_sector && next_sector != self.last_sector_erased {
                        let _ = breakpoint_helper(25); // Брейкпоинт: Erasing new sector
                        
                        // Стираем новый сектор
                        let result = unsafe { 
                            flash::erase_sector(&pac::Peripherals::steal(), next_sector) 
                        };
                        
                        if result == 0 {
                            unsafe { DEBUG_ERROR_CODE = 1; } // Code for flash erase error
                            let _ = breakpoint_helper(26); // Брейкпоинт: Flash erase error
                            
                            self.state = XmodemState::Error;
                            return Err(XmodemError::FlashWriteError);
                        }
                        
                        self.last_sector_erased = next_sector;
                    }
                    
                    // Запись данных во flash
                    let _ = breakpoint_helper(27); // Брейкпоинт: Before flash write
                    
                    let result = unsafe { 
                        // Сохраняем адрес для отладки через окно watch
                        let addr = self.current_address;
                        let flash_result = flash::write(&pac::Peripherals::steal(), 
                                                    &self.buffer[3..131], 
                                                    addr);
                        DEBUG_FLASH_RESULT = flash_result;
                        flash_result
                    };
                    
                    if result != 0 {
                        unsafe { DEBUG_ERROR_CODE = 2; } // Code for flash write error
                        let _ = breakpoint_helper(28); // Брейкпоинт: Flash write error
                        
                        // Ошибка записи
                        self.state = XmodemState::Error;
                        return Err(XmodemError::FlashWriteError);
                    }
                    
                    let _ = breakpoint_helper(29); // Брейкпоинт: Successful write
                    
                    // Увеличиваем адрес и номер пакета
                    self.current_address += 128;
                    self.packet_number = self.packet_number.wrapping_add(1);
                    self.packet_count += 1;
                    
                    // Отправляем ACK
                    self.next_byte_to_send = Some(ACK);
                    self.state = XmodemState::WaitingForData;
                    self.buffer_index = 0;
                    
                    return Ok(true);
                }
                
                Ok(false)
            },
            
            XmodemState::Error => {
                unsafe { 
                    DEBUG_LAST_BYTE = byte;
                    DEBUG_ERROR_CODE = 99; // Generic error
                }
                let _ = breakpoint_helper(30); // Брейкпоинт: Error state
                Ok(false)
            },
            
            XmodemState::Complete => {
                unsafe { DEBUG_LAST_BYTE = byte; }
                let _ = breakpoint_helper(31); // Брейкпоинт: Complete state
                Ok(false)
            }
        }
    }
}