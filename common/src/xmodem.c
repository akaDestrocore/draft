#include "xmodem.h"

// Timeout values in ms
#define PACKET_TIMEOUT 5000
#define C_RETRY_INTERVAL 3000
#define MAX_RETRIES 10

#define DATA_SIZE 128

static uint16_t calculate_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

void xmodem_init(XmodemManager_t* manager, const XmodemConfig_t* config) {
    memset(manager, 0, sizeof(XmodemManager_t));
    manager->state = XMODEM_STATE_IDLE;
    manager->config = *config;
    manager->header_size = config->image_hdr_size;
}

void xmodem_start(XmodemManager_t* manager, uint32_t addr) {
    manager->state = XMODEM_STATE_SENDING_INITIAL_C;
    manager->target_addr = addr;
    manager->current_addr = addr;
    manager->expected_packet_num = 1;
    manager->buffer_index = 0;
    manager->packet_count = 0;
    manager->retries = 0;
    manager->first_packet_processed = 0;
    manager->next_byte_to_send = XMODEM_C;
    manager->last_poll_time = HAL_GetTick();
    manager->total_data_received = 0;
    manager->actual_firmware_size = 0;
    manager->received_eot = 0;
    manager->first_sector_erased = 0;
    manager->expected_total_packets = 0;
    manager->last_packet_useful_bytes = 0;
    
    // Set magic based on address
    if (addr == manager->config.app_addr) {
        manager->expected_magic = IMAGE_MAGIC_APP;
        manager->expected_img_type = IMAGE_TYPE_APP;
    } else if (addr == manager->config.updater_addr) {
        manager->expected_magic = IMAGE_MAGIC_UPDATER;
        manager->expected_img_type = IMAGE_TYPE_UPDATER;
    } else if (addr == manager->config.loader_addr) {
        manager->expected_magic = IMAGE_MAGIC_LOADER;
        manager->expected_img_type = IMAGE_TYPE_LOADER;
    } else {
        manager->state = XMODEM_STATE_ERROR;
        return;
    }
    
    manager->current_sector_base = addr;
}

static int process_first_packet(XmodemManager_t* manager, const uint8_t* data) {
    // First check if data is enough for header
    if (DATA_SIZE < sizeof(ImageHeader_t)) {
        return 0;
    }
    
    // Parse header
    ImageHeader_t* header = (ImageHeader_t*)data;
    
    // Check magic
    if (header->image_magic != manager->expected_magic) {
        return 0;
    }
    
    // Check image type
    if (header->image_type != manager->expected_img_type) {
        return 0;
    }
    
    // Store firmware size
    if (header->data_size > 0) {
        manager->actual_firmware_size = header->data_size + manager->header_size;
        
        // Calculate total packets
        uint32_t total_bytes = header->data_size + manager->header_size;
        uint32_t full_packets = total_bytes / DATA_SIZE;
        uint32_t remainder = total_bytes % DATA_SIZE;
        
        manager->expected_total_packets = full_packets + (remainder > 0 ? 1 : 0);
        
        if (remainder > 0) {
            manager->last_packet_useful_bytes = remainder;
        } else {
            manager->last_packet_useful_bytes = DATA_SIZE;
        }
    } else {
        // If no size info provided then use hardcoded value
        manager->actual_firmware_size = 0x40000; // 256 KB
    }
    
    // Check version
    ImageHeader_t current_header;
    if (manager->target_addr != 0xFFFFFFFF) {
        memcpy(&current_header, (void*)manager->target_addr, sizeof(ImageHeader_t));
        
        // Check if it's newer
        if (is_image_valid(&current_header) && !is_newer_version(header, &current_header)) {
            return 0;
        }
    }
    
    // Erase first sector if not done yet
    if (!manager->first_sector_erased) {
        if (!flash_erase_sector(manager->target_addr)) {
            return 0; // failed
        }
        manager->first_sector_erased = 1;
    }
    
    // Write first packet data to flash
    if (!flash_write(manager->current_addr, data, DATA_SIZE)) {
        return 0;
    }
    
    manager->total_data_received = DATA_SIZE;
    manager->current_addr += DATA_SIZE;
    manager->first_packet_processed = 1;
    
    return 1;
}

static size_t find_useful_bytes(XmodemManager_t* manager, const uint8_t* data, uint8_t packet_num) {
    if (manager->expected_total_packets > 0 && 
        packet_num == manager->expected_total_packets && 
        manager->last_packet_useful_bytes > 0) {
        return manager->last_packet_useful_bytes;
    }
    return DATA_SIZE;
}

XmodemError_t xmodem_process_byte(XmodemManager_t* manager, uint8_t byte) {
    uint32_t current_time = HAL_GetTick();
    
    switch (manager->state) {
        case XMODEM_STATE_IDLE:
            return XMODEM_ERROR_NONE;
            
        case XMODEM_STATE_SENDING_INITIAL_C:
            // Check for SOH
            if (byte == XMODEM_SOH) {
                manager->buffer[0] = byte;
                manager->buffer_index = 1;
                manager->state = XMODEM_STATE_RECEIVING_DATA;
                manager->last_poll_time = current_time;
                return XMODEM_ERROR_NONE;
            } else if (byte == XMODEM_CAN) {
                manager->state = XMODEM_STATE_ERROR;
                return XMODEM_ERROR_CANCELLED;
            } else {
                // Spam 'C'
                if (current_time - manager->last_poll_time >= C_RETRY_INTERVAL) {
                    manager->next_byte_to_send = XMODEM_C;
                    manager->last_poll_time = current_time;
                    manager->retries++;
                    
                    if (manager->retries >= MAX_RETRIES) {
                        manager->state = XMODEM_STATE_ERROR;
                        return XMODEM_ERROR_TIMEOUT;
                    }
                    
                    return XMODEM_ERROR_NONE;
                }
                return XMODEM_ERROR_NONE;
            }
            
        case XMODEM_STATE_WAITING_FOR_DATA:
            // Check for timeout
            if (current_time - manager->last_poll_time >= PACKET_TIMEOUT) {
                manager->state = XMODEM_STATE_ERROR;
                return XMODEM_ERROR_TIMEOUT;
            }
            
            // Process received byte
            if (byte == XMODEM_SOH) {
                manager->buffer[0] = byte;
                manager->buffer_index = 1;
                manager->state = XMODEM_STATE_RECEIVING_DATA;
                manager->last_poll_time = current_time;
                return XMODEM_ERROR_NONE;
            } else if (byte == XMODEM_EOT) {
                // End of transmission
                manager->received_eot = 1;
                manager->state = XMODEM_STATE_COMPLETE;
                manager->next_byte_to_send = XMODEM_ACK;
                
                // Verify size
                if (manager->actual_firmware_size > 0) {
                    if (manager->total_data_received < manager->actual_firmware_size) {
                        manager->state = XMODEM_STATE_ERROR;
                        return XMODEM_ERROR_INVALID_PACKET;
                    }
                    
                    // Adjust addr if received more than expected
                    if (manager->current_addr > manager->target_addr + manager->actual_firmware_size) {
                        manager->current_addr = manager->target_addr + manager->actual_firmware_size;
                    }
                }
                
                return XMODEM_ERROR_TRANSFER_COMPLETE;
            } else if (byte == XMODEM_CAN) {
                manager->state = XMODEM_STATE_ERROR;
                return XMODEM_ERROR_CANCELLED;
            }
            return XMODEM_ERROR_NONE;
            
        case XMODEM_STATE_RECEIVING_DATA:
            // Check for timeout
            if (current_time - manager->last_poll_time >= PACKET_TIMEOUT) {
                manager->state = XMODEM_STATE_ERROR;
                return XMODEM_ERROR_TIMEOUT;
            }
            
            // Add byte to buffer
            manager->buffer[manager->buffer_index++] = byte;
            
            // Check if we have a complete packet
            if (manager->buffer_index == 133) {
                manager->state = XMODEM_STATE_PROCESSING_PACKET;
                
                // Process the packet
                uint8_t packet_num = manager->buffer[1];
                uint8_t packet_num_comp = manager->buffer[2];
                
                // Check packet number integrity
                if ((packet_num + packet_num_comp) != 0xFF) {
                    manager->state = XMODEM_STATE_WAITING_FOR_DATA;
                    manager->buffer_index = 0;
                    manager->next_byte_to_send = XMODEM_NAK;
                    return XMODEM_ERROR_SEQUENCE_ERROR;
                }
                
                // Check packet sequence
                if (packet_num != manager->expected_packet_num) {
                    manager->state = XMODEM_STATE_WAITING_FOR_DATA;
                    manager->buffer_index = 0;
                    manager->next_byte_to_send = XMODEM_NAK;
                    return XMODEM_ERROR_SEQUENCE_ERROR;
                }
                
                // Verify CRC
                uint16_t received_crc = (manager->buffer[131] << 8) | manager->buffer[132];
                uint16_t calculated_crc = calculate_crc16(&manager->buffer[3], DATA_SIZE);
                
                if (received_crc != calculated_crc) {
                    manager->state = XMODEM_STATE_WAITING_FOR_DATA;
                    manager->buffer_index = 0;
                    manager->next_byte_to_send = XMODEM_NAK;
                    return XMODEM_ERROR_CRC_ERROR;
                }
                
                // Process data based on packet number
                if (packet_num == 1 && !manager->first_packet_processed) {
                    // First packet contains header
                    if (!process_first_packet(manager, &manager->buffer[3])) {
                        manager->state = XMODEM_STATE_ERROR;
                        return XMODEM_ERROR_INVALID_MAGIC;
                    }
                } else {
                    // Regular data packet
                    if (packet_num == 2 && !manager->first_packet_processed) {
                        manager->state = XMODEM_STATE_ERROR;
                        return XMODEM_ERROR_INVALID_PACKET;
                    }
                    
                    // Get useful data length
                    size_t useful_bytes = find_useful_bytes(manager, &manager->buffer[3], packet_num);
                    
                    if (useful_bytes == 0) {
                        // Skip useless
                        manager->state = XMODEM_STATE_WAITING_FOR_DATA;
                        manager->buffer_index = 0;
                        manager->expected_packet_num++;
                        manager->packet_count++;
                        manager->next_byte_to_send = XMODEM_ACK;
                        manager->last_poll_time = current_time;
                        return XMODEM_ERROR_NONE;
                    }
                    
                    // Track received data
                    manager->total_data_received += useful_bytes;
                    
                    if (manager->actual_firmware_size > 0 && 
                        manager->total_data_received > manager->actual_firmware_size) {
                        // Adjust useful bytes if we're receiving more than needed
                        uint32_t excess = manager->total_data_received - manager->actual_firmware_size;
                        if (excess < useful_bytes) {
                            useful_bytes -= excess;
                            manager->total_data_received = manager->actual_firmware_size;
                        } else {
                            useful_bytes = 0;
                        }
                    }
                    
                    // Check if we need to erase next sector
                    uint32_t next_addr = manager->current_addr + useful_bytes;
                    uint32_t current_sector_end = manager->current_sector_base + 0x20000;
                    
                    if (next_addr > current_sector_end && useful_bytes > 0) {
                        uint32_t next_sector_base = manager->current_sector_base + 0x20000;
                        
                        if (!flash_erase_sector(next_sector_base)) {
                            manager->state = XMODEM_STATE_ERROR;
                            return XMODEM_ERROR_FLASH_WRITE_ERROR;
                        }
                        
                        manager->current_sector_base = next_sector_base;
                    }
                    
                    // Write data to flash
                    if (useful_bytes > 0) {
                        if (!flash_write(manager->current_addr, &manager->buffer[3], useful_bytes)) {
                            manager->state = XMODEM_STATE_ERROR;
                            return XMODEM_ERROR_FLASH_WRITE_ERROR;
                        }
                        
                        manager->current_addr += useful_bytes;
                    }
                }
                
                // Move to next packet
                manager->state = XMODEM_STATE_WAITING_FOR_DATA;
                manager->buffer_index = 0;
                manager->expected_packet_num++;
                manager->packet_count++;
                manager->next_byte_to_send = XMODEM_ACK;
                manager->last_poll_time = current_time;
                return XMODEM_ERROR_NONE;
            }
            return XMODEM_ERROR_NONE;
            
        case XMODEM_STATE_ERROR:
        case XMODEM_STATE_COMPLETE:
        default:
            return XMODEM_ERROR_NONE;
    }
}

int xmodem_should_send_byte(XmodemManager_t* manager) {
    uint32_t current_time = HAL_GetTick();
    
    if (manager->state == XMODEM_STATE_SENDING_INITIAL_C) {
        if (current_time - manager->last_poll_time >= C_RETRY_INTERVAL || manager->next_byte_to_send != 0) {
            if (manager->next_byte_to_send == 0) {
                manager->next_byte_to_send = XMODEM_C;
            }
            manager->last_poll_time = current_time;
            manager->retries++;
            
            if (manager->retries >= MAX_RETRIES) {
                manager->state = XMODEM_STATE_ERROR;
            }
            
            return 1;
        }
        return 0;
    }
    
    return manager->next_byte_to_send != 0;
}

uint8_t xmodem_get_response(XmodemManager_t* manager) {
    uint8_t response = manager->next_byte_to_send;
    manager->next_byte_to_send = 0;
    return response;
}

XmodemState_t xmodem_get_state(XmodemManager_t* manager) {
    return manager->state;
}

uint16_t xmodem_get_packet_count(XmodemManager_t* manager) {
    return manager->packet_count;
}

void xmodem_cancel_transfer(XmodemManager_t* manager) {
    manager->state = XMODEM_STATE_ERROR;
    manager->next_byte_to_send = 0;
}