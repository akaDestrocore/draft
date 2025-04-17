#include <image.h>

int is_image_valid(const ImageHeader_t* header) {
    switch (header->image_type) {
        case IMAGE_TYPE_LOADER:
            return header->image_magic == IMAGE_MAGIC_LOADER;
        case IMAGE_TYPE_UPDATER:
            return header->image_magic == IMAGE_MAGIC_UPDATER;
        case IMAGE_TYPE_APP:
            return header->image_magic == IMAGE_MAGIC_APP;
        default:
            return 0;
    }
}

int is_newer_version(const ImageHeader_t* new_header, const ImageHeader_t* current_header) {
    // Compare major
    if (new_header->version_major > current_header->version_major) {
        return 1;
    }
    if (new_header->version_major < current_header->version_major) {
        return 0;
    }
    
    // If major is equal, compare minor
    if (new_header->version_minor > current_header->version_minor) {
        return 1;
    }
    if (new_header->version_minor < current_header->version_minor) {
        return 0;
    }
    
    // If minor equal, compare patch
    return new_header->version_patch > current_header->version_patch;
}

void update_header_crc(ImageHeader_t* header, uint32_t crc) {
    header->crc = crc;
}

void update_header_data_size(ImageHeader_t* header, uint32_t size) {
    header->data_size = size;
}

void update_header_vector_addr(ImageHeader_t* header, uint32_t addr) {
    header->vector_addr = addr;
}