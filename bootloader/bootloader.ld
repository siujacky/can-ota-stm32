/* bootloader.ld — Linker script for STM32F103 CAN bootloader
 * Bootloader lives in the first 8KB of flash (pages 0-7).
 * Application starts at 0x08002000.
 * RAM: 4KB (STM32F103C8 has 20KB but bootloader only needs ~1KB of stack/data/bss).
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 8K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

/* Stack at end of RAM region used for the bootloader */
_estack = ORIGIN(RAM) + LENGTH(RAM);   /* 0x20001000 */

SECTIONS
{
    /* ---- Code and read-only data ---- */
    .text :
    {
        /* Vector table must be first */
        KEEP(*(.isr_vector))
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* ---- Initialised data: LMA in flash, VMA in RAM ---- */
    .data :
    {
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    /* LMA of .data section (where it lives in flash) */
    _ldata = LOADADDR(.data);

    /* ---- Zero-initialised data ---- */
    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* ---- Discard unwanted sections ---- */
    /DISCARD/ :
    {
        *(.ARM.exidx*)
        *(.ARM.extab*)
        *(.eh_frame*)
        *(.comment)
    }
}
