#include "delta_update.h"

// Buffer size for patching operations
#define DELTA_BUFFER_SIZE 2048

// Static buffers for patching
static unsigned char source_buf[DELTA_BUFFER_SIZE];
static unsigned char target_buf[DELTA_BUFFER_SIZE];
static unsigned char patch_buf[DELTA_BUFFER_SIZE];

int apply_delta_patch(uint32_t source_addr, uint32_t patch_addr, uint32_t target_addr,
                     uint32_t source_size, uint32_t patch_size) {
    // Initialize janpatch context
    janpatch_ctx ctx;
    
    ctx.source_buffer.buffer = source_buf;
    ctx.source_buffer.size = DELTA_BUFFER_SIZE;
    ctx.source_buffer.current_page = 0xFFFFFFFF;
    
    ctx.patch_buffer.buffer = patch_buf;
    ctx.patch_buffer.size = DELTA_BUFFER_SIZE;
    ctx.patch_buffer.current_page = 0xFFFFFFFF;
    
    ctx.target_buffer.buffer = target_buf;
    ctx.target_buffer.size = DELTA_BUFFER_SIZE;
    ctx.target_buffer.current_page = 0xFFFFFFFF;
    
    ctx.fread = &sfio_fread;
    ctx.fwrite = &sfio_fwrite;
    ctx.fseek = &sfio_fseek;
    ctx.ftell = NULL;
    ctx.progress = NULL;
    
    // Prepare source stream
    sfio_stream_t source;
    source.type = SFIO_STREAM_SLOT;
    source.offset = 0;
    source.size = source_size;
    source.slot = source_addr;
    
    // Prepare patch stream
    sfio_stream_t patch;
    patch.type = SFIO_STREAM_SLOT;
    patch.offset = 0;
    patch.size = patch_size;
    patch.slot = patch_addr;
    
    // Prepare target stream
    sfio_stream_t target;
    target.type = SFIO_STREAM_SLOT;
    target.offset = 0;
    target.size = source_size + patch_size;
    target.slot = target_addr;
    
    // Apply the patch
    return janpatch(ctx, &source, &patch, &target);
}

int verify_patched_firmware(uint32_t target_addr, uint32_t header_size) {
    return verify_firmware_crc(target_addr, header_size);
}