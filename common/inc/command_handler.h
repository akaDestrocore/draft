#ifndef _COMMAND_HANDLER_H
#define _COMMAND_HANDLER_H

#include "flash.h"
#include "crc.h"
#include "delta_update.h"
#include "image.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>

// Command IDs
typedef enum {
    CMD_GET_VERSION = 0x01,
    CMD_ERASE_FLASH = 0x02,
    CMD_WRITE_FLASH = 0x03,
    CMD_READ_FLASH = 0x04,
    CMD_VERIFY_CRC = 0x05,
    CMD_BOOT_IMAGE = 0x06,
    CMD_APPLY_PATCH = 0x07,
    // Add more as needed
} CommandID_t;

// Command response status
typedef enum {
    CMD_STATUS_SUCCESS = 0x00,
    CMD_STATUS_ERROR = 0x01,
    CMD_STATUS_INVALID_PARAM = 0x02,
    CMD_STATUS_CRC_ERROR = 0x03,
    // Add more as needed
} CommandStatus_t;

// Command handler function type
typedef CommandStatus_t (*CommandHandler_t)(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len);

// Process received command
CommandStatus_t process_command(const uint8_t* cmd_data, size_t cmd_len, uint8_t* response, size_t* resp_len);

// Register command handler
void register_command_handler(CommandID_t cmd_id, CommandHandler_t handler);

// Initialize command handlers
void init_command_handlers(void);

#endif /* _COMMAND_HANDLER_H */