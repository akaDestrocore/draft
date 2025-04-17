#ifndef _XMODEM_H
#define _XMODEM_H

#include <stdint.h>
#include <stddef.h>

// XMODEM consts
#define XMODEM_SOH 0x01  // Start of header
#define XMODEM_EOT 0x04  // End of transmission
#define XMODEM_ACK 0x06  // Acknowledge
#define XMODEM_NAK 0x15  // Not acknowledge
#define XMODEM_CAN 0x18  // Cancel
#define XMODEM_C   0x43  // 'C' character

typedef enum {
    XMODEM_STATE_IDLE,
    XMODEM_STATE_SENDING_INITIAL_C,
    XMODEM_STATE_WAITING_FOR_DATA,
    XMODEM_STATE_RECEIVING_DATA,
    XMODEM_STATE_PROCESSING_PACKET,
    XMODEM_STATE_ERROR,
    XMODEM_STATE_COMPLETE
} XmodemState_t;

typedef enum {
    XMODEM_ERROR_NONE,
    XMODEM_ERROR_INVALID_PACKET,
    XMODEM_ERROR_SEQUENCE_ERROR,
    XMODEM_ERROR_CRC_ERROR,
    XMODEM_ERROR_CANCELLED,
    XMODEM_ERROR_TIMEOUT,
    XMODEM_ERROR_FLASH_WRITE_ERROR,
    XMODEM_ERROR_INVALID_MAGIC,
    XMODEM_ERROR_OLDER_VERSION,
    XMODEM_ERROR_TRANSFER_COMPLETE
} XmodemError_t;

typedef struct {
    uint32_t app_addr;
    uint32_t updater_addr;
    uint32_t loader_addr;
    uint32_t image_hdr_size;
} XmodemConfig_t;

// XMODEM manager struct
typedef struct {
    XmodemState_t state;
    uint32_t target_addr;
    uint32_t current_addr;
    uint8_t expected_packet_num;
    uint32_t last_poll_time;
    uint8_t buffer[133]; // SOH + packet_num + (FF - packet_num) + 128 data + CRC16
    size_t buffer_index;
    uint32_t expected_magic;
    uint8_t expected_img_type;
    uint16_t packet_count;
    uint8_t next_byte_to_send;
    uint8_t retries;
    int first_packet_processed;
    uint32_t current_sector_base;
    uint32_t total_data_received;
    uint32_t actual_firmware_size;
    uint32_t header_size;
    int received_eot;
    int first_sector_erased;
    uint16_t expected_total_packets;
    size_t last_packet_useful_bytes;
    XmodemConfig_t config;
} XmodemManager_t;

// Initialize XMODEM manager struct
void xmodem_init(XmodemManager_t* manager, const XmodemConfig_t* config);

// Start XMODEM transfer
void xmodem_start(XmodemManager_t* manager, uint32_t addr);

// Process received byte
XmodemError_t xmodem_process_byte(XmodemManager_t* manager, uint8_t byte);

// Check if response needed
int xmodem_should_send_byte(XmodemManager_t* manager);

// Get response
uint8_t xmodem_get_response(XmodemManager_t* manager);

// Get current state
XmodemState_t xmodem_get_state(XmodemManager_t* manager);

// Get packet count
uint16_t xmodem_get_packet_count(XmodemManager_t* manager);

// Cancel transfer
void xmodem_cancel_transfer(XmodemManager_t* manager);

#endif /* _XMODEM_H */