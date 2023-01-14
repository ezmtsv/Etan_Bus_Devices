ENTRY(Reset_Handler)
MEMORY
{
    ROM  (rx): ORIGIN = 0x08000000, LENGTH = 16K
    RAM (rwx): ORIGIN = 0x20000000, LENGTH = 4K
}

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > ROM

    __etext = .;

    .data : AT (__etext)
    {
        __data_start__ = .;
        *(.ramfunc);
        *(.data*)
        . = ALIGN(4);
        __data_end__ = .;
    } > RAM

    .bss (NOLOAD) :
    {
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .heap (NOLOAD) :
    {
        __end__ = .;
        *(.heap*)
        __HeapLimit = .;
    } > RAM

    .stack_dummy (NOLOAD) :
    {
        *(.stack)
    } > RAM

    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);
    PROVIDE(_estack = __stack);
    PROVIDE(_sidata = __etext);
    PROVIDE(_sdata = __data_start__);
    PROVIDE(_edata = __data_end__);
    PROVIDE(_sbss = __bss_start__);
    PROVIDE(_ebss = __bss_end__);
}
