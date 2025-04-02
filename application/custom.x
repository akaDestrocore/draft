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
}

