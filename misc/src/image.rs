#![no_std]

pub const IMAGE_MAGIC_LOADER: u32 = 0xDEADC0DE;
pub const IMAGE_MAGIC_UPDATER: u32 = 0xFEEDFACE;
pub const IMAGE_MAGIC_APP: u32 = 0xC0FFEE00;

pub const IMAGE_VERSION_CURRENT: u16 = 0x0100;

// Image types
pub const IMAGE_TYPE_LOADER: u8 = 1;
pub const IMAGE_TYPE_UPDATER: u8 = 2;
pub const IMAGE_TYPE_APP: u8 = 3;

// Firmware image header structure
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct ImageHeader {
    pub image_magic: u32,           // Magic number to identify the image type
    pub image_hdr_version: u16,     // Header version
    pub image_type: u8,             // Type of image
    pub version_major: u8,          // Major version number
    pub version_minor: u8,          // Minor version number
    pub version_patch: u8,          // Patch version number
    pub _padding: u16,              // Padding for alignment
    pub vector_addr: u32,           // Address of the vector table
    pub crc: u32,                   // CRC of the image (excluding header)
    pub data_size: u32,             // Size of the image data
}

impl ImageHeader {
    pub const fn new(image_type: u8, magic: u32, major: u8, minor: u8, patch: u8) -> Self {
        Self {
            image_magic: magic,
            image_hdr_version: IMAGE_VERSION_CURRENT,
            image_type,
            version_major: major,
            version_minor: minor,
            version_patch: patch,
            _padding: 0,
            vector_addr: 0,
            crc: 0,
            data_size: 0,
        }
    }
    
    // Check if this image is valid
    pub fn is_valid(&self) -> bool {
        match self.image_type {
            IMAGE_TYPE_LOADER => self.image_magic == IMAGE_MAGIC_LOADER,
            IMAGE_TYPE_UPDATER => self.image_magic == IMAGE_MAGIC_UPDATER,
            IMAGE_TYPE_APP => self.image_magic == IMAGE_MAGIC_APP,
            _ => false,
        }
    }
    
    pub fn is_newer_than(&self, other: &Self) -> bool {
        if self.version_major > other.version_major {
            return true;
        }
        if self.version_major < other.version_major {
            return false;
        }
        
        if self.version_minor > other.version_minor {
            return true;
        }
        if self.version_minor < other.version_minor {
            return false;
        }
        
        self.version_patch > other.version_patch
    }
    
    // Update data size field in the header
    pub fn update_data_size(&mut self, size: u32) {
        self.data_size = size;
    }

    pub fn update_vector_addr(&mut self, addr: u32) {
        self.vector_addr = addr;
    }

    pub fn update_crc(&mut self, crc: u32) {
        self.crc = crc;
    }
}

// Shared memory
#[repr(C)]
pub struct SharedMemory {
    pub target_image_type: u8,
    pub update_requested: bool,
    pub reserved: [u8; 62],
}

impl SharedMemory {
    pub const fn new() -> Self {
        Self {
            target_image_type: 0,
            update_requested: false,
            reserved: [0; 62],
        }
    }
}