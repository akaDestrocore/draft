SECTIONS
{
    .image_hdr ORIGIN(FLASH_HDR) : ALIGN(4)
    {
        KEEP(*(.image_hdr));
        . = ALIGN(4);
    } > FLASH_HDR
    
    .shared_memory (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.shared_memory));
        . = ALIGN(4);
    } > RAM2
    
    __firmware_start = LOADADDR(.text);
    __firmware_end = LOADADDR(.text) + SIZEOF(.text) + SIZEOF(.rodata) + SIZEOF(.data);
}

__firmware_size = __firmware_end - __firmware_start;