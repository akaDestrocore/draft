#include "command_handler.h"

// Command handler array
static CommandHandler_t command_handlers[256] = {0};

// Function prototypes for command handlers
CommandStatus_t cmd_get_version(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len);
CommandStatus_t cmd_erase_flash(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len);
CommandStatus_t cmd_write_flash(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len);
CommandStatus_t cmd_read_flash(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len);
CommandStatus_t cmd_verify_crc(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len);
CommandStatus_t cmd_boot_image(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len);
CommandStatus_t cmd_apply_patch(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len);

// Process received command
CommandStatus_t process_command(const uint8_t* cmd_data, size_t cmd_len, uint8_t* response, size_t* resp_len) {
    if (cmd_len < 1) {
        return CMD_STATUS_INVALID_PARAM;
    }
    
    CommandID_t cmd_id = cmd_data[0];
    CommandHandler_t handler = command_handlers[cmd_id];
    
    if (handler == NULL) {
        return CMD_STATUS_ERROR;
    }
    
    return handler(cmd_data + 1, cmd_len - 1, response, resp_len);
}

// Register command handler
void register_command_handler(CommandID_t cmd_id, CommandHandler_t handler) {
    command_handlers[cmd_id] = handler;
}

// Initialize command handlers
void init_command_handlers(void) {
    register_command_handler(CMD_GET_VERSION, cmd_get_version);
    register_command_handler(CMD_ERASE_FLASH, cmd_erase_flash);
    register_command_handler(CMD_WRITE_FLASH, cmd_write_flash);
    register_command_handler(CMD_READ_FLASH, cmd_read_flash);
    register_command_handler(CMD_VERIFY_CRC, cmd_verify_crc);
    register_command_handler(CMD_BOOT_IMAGE, cmd_boot_image);
    register_command_handler(CMD_APPLY_PATCH, cmd_apply_patch);
}

// Get version handler
CommandStatus_t cmd_get_version(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len) {
    // Get version from current image header
    ImageHeader_t header;
    if (get_firmware_header(APP_ADDR, NULL, &header)) {
        response[0] = header.version_major;
        response[1] = header.version_minor;
        response[2] = header.version_patch;
        *resp_len = 3;
        return CMD_STATUS_SUCCESS;
    }
    
    // Default version if no valid header
    response[0] = 0; // major
    response[1] = 0; // minor
    response[2] = 0; // patch
    
    *resp_len = 3;
    return CMD_STATUS_SUCCESS;
}

// Erase flash handler
CommandStatus_t cmd_erase_flash(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len) {
    if (len < 4) {
        return CMD_STATUS_INVALID_PARAM;
    }
    
    uint32_t addr = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    
    if (!flash_erase_sector(addr)) {
        return CMD_STATUS_ERROR;
    }
    
    *resp_len = 0;
    return CMD_STATUS_SUCCESS;
}

// Write flash handler
CommandStatus_t cmd_write_flash(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len) {
    if (len < 5) { // 4 bytes for address + at least 1 byte of data
        return CMD_STATUS_INVALID_PARAM;
    }
    
    uint32_t addr = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    const uint8_t* write_data = data + 4;
    size_t write_len = len - 4;
    
    if (!flash_write(addr, write_data, write_len)) {
        return CMD_STATUS_ERROR;
    }
    
    *resp_len = 0;
    return CMD_STATUS_SUCCESS;
}

// Read flash handler
CommandStatus_t cmd_read_flash(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len) {
    if (len < 8) { // 4 bytes for address + 4 bytes for length
        return CMD_STATUS_INVALID_PARAM;
    }
    
    uint32_t addr = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    uint32_t read_len = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    
    if (read_len > 1024) { // Limit read size
        read_len = 1024;
    }
    
    flash_read(addr, response, read_len);
    
    *resp_len = read_len;
    return CMD_STATUS_SUCCESS;
}

// Verify CRC handler
CommandStatus_t cmd_verify_crc(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len) {
    if (len < 8) { // 4 bytes for address + 4 bytes for header size
        return CMD_STATUS_INVALID_PARAM;
    }
    
    uint32_t addr = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    uint32_t header_size = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    
    if (!verify_firmware_crc(addr, header_size)) {
        return CMD_STATUS_CRC_ERROR;
    }
    
    *resp_len = 0;
    return CMD_STATUS_SUCCESS;
}

// Boot image handler
CommandStatus_t cmd_boot_image(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len) {
    if (len < 4) { // 4 bytes for address
        return CMD_STATUS_INVALID_PARAM;
    }
    
    uint32_t addr = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    
    BootConfig_t config = {
        .app_addr = APP_ADDR,
        .updater_addr = UPDATER_ADDR,
        .loader_addr = LOADER_ADDR,
        .image_hdr_size = IMAGE_HDR_SIZE
    };
    
    // Boot to the specified image based on address
    if (addr == config.app_addr) {
        boot_application(&config);
    } else if (addr == config.updater_addr) {
        boot_updater(&config);
    } else if (addr == config.loader_addr) {
        boot_loader(&config);
    } else {
        return CMD_STATUS_INVALID_PARAM;
    }
    
    // Should not reach here if boot was successful
    return CMD_STATUS_ERROR;
}

// Apply patch handler
CommandStatus_t cmd_apply_patch(const uint8_t* data, size_t len, uint8_t* response, size_t* resp_len) {
    if (len < 20) { // 4 bytes for source addr + 4 bytes for patch addr + 4 bytes for target addr + 4 bytes for source size + 4 bytes for patch size
        return CMD_STATUS_INVALID_PARAM;
    }
    
    uint32_t source_addr = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    uint32_t patch_addr = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    uint32_t target_addr = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
    uint32_t source_size = (data[12] << 24) | (data[13] << 16) | (data[14] << 8) | data[15];
    uint32_t patch_size = (data[16] << 24) | (data[17] << 16) | (data[18] << 8) | data[19];
    
    if (apply_delta_patch(source_addr, patch_addr, target_addr, source_size, patch_size) != 0) {
        return CMD_STATUS_ERROR;
    }
    
    *resp_len = 0;
    return CMD_STATUS_SUCCESS;
}