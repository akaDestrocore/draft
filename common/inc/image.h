#ifndef _IMAGE_H
#define _IMAGE_H

#include <stdint.h>

// Magic numbers for each component
#define IMAGE_MAGIC_LOADER    0xDEADC0DE
#define IMAGE_MAGIC_UPDATER   0xFEEDFACE
#define IMAGE_MAGIC_APP       0xC0FFEE00

#define IMAGE_VERSION_CURRENT 0x0100

// Image types
typedef enum {
    IMAGE_TYPE_LOADER   = 1,
    IMAGE_TYPE_UPDATER  = 2,
    IMAGE_TYPE_APP      = 3
} ImageType_t;

// Image header struct
typedef struct __attribute__((packed)) {
    uint32_t image_magic;        // Magic number to identify the image type
    uint16_t image_hdr_version;  // Header version
    uint8_t  image_type;         // Type of image
    uint8_t  version_major;      // Major version number
    uint8_t  version_minor;      // Minor version number
    uint8_t  version_patch;      // Patch version number
    uint16_t _padding;           // Padding for alignment
    uint32_t vector_addr;        // Address of the vector table
    uint32_t crc;                // CRC of the image (excluding header)
    uint32_t data_size;          // Size of the image data
    uint8_t  reserved[0x1E0];    // Reserved space to make header 0x200 bytes
} ImageHeader_t;

// Shared memory structure for communication between components
typedef struct __attribute__((packed)) {
    uint8_t target_image_type;
    uint8_t update_requested;
    uint8_t reserved[62];
} SharedMemory_t;

// Functions for header operations
int is_image_valid(const ImageHeader_t* header);
int is_newer_version(const ImageHeader_t* new_header, const ImageHeader_t* current_header);
void update_header_crc(ImageHeader_t* header, uint32_t crc);
void update_header_data_size(ImageHeader_t* header, uint32_t size);
void update_header_vector_addr(ImageHeader_t* header, uint32_t addr);

#endif /* _IMAGE_H */