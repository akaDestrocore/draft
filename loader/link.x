INCLUDE memory.x

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
    
    /* The rest of the linker script is from cortex-m-rt */
    .vector_table ORIGIN(FLASH) :
    {
        /* Vector table */
        _svector_table = .;
        LONG(_stack_start);
        KEEP(*(.vector_table.reset_vector));
        KEEP(*(.vector_table.exceptions));
        KEEP(*(.vector_table.interrupts));
        _evector_table = .;
    } > FLASH
    
    PROVIDE(_stext = _evector_table);
    
    .text _stext :
    {
        *(.text .text.*);
        . = ALIGN(4);
        _etext = .;
    } > FLASH
    
    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.*);
        . = ALIGN(4);
    } > FLASH
    
    .data : ALIGN(4)
    {
        _sdata = .;
        *(.data .data.*);
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH
    _sidata = LOADADDR(.data);
    
    .bss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        *(.bss .bss.*);
        *(COMMON);
        . = ALIGN(4);
        _ebss = .;
    } > RAM
    
    /DISCARD/ :
    {
        *(.ARM.exidx .ARM.exidx.*);
    }
}

/* Asserts that check validity of the program */
ASSERT(_edata <= ORIGIN(RAM) + LENGTH(RAM), "
ERROR(cortex-m-rt): The data section must be placed in the RAM memory region");

ASSERT(_ebss <= ORIGIN(RAM) + LENGTH(RAM), "
ERROR(cortex-m-rt): The BSS section must be placed in the RAM memory region");