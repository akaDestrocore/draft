SECTIONS
{
    .shared_memory (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.shared_memory));
        . = ALIGN(4);
    } > RAM2
}

