SECTIONS
{
    .shared_memory (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.shared_memory));
        . = ALIGN(4);
    } > RAM2
}

/* Include the main link script from cortex-m-rt */
INCLUDE cortex-m-rt/link.x