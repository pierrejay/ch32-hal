/*
 * CH32H417, the V3F boot core alone (the stress test does not start the V5F): from
 * ch32-hal's examples/ch32h417/v3f/memory.x, with the load image given 192 KiB of flash, over
 * the V5F's slot at 0x10000 (the board's full flash is saved before and written back after).
 * The code runs from the zero-wait shared SRAM: h417/link.x copies it there at startup, the
 * flash being non-zero-wait (about 38 cycles an instruction at 100 MHz, RM §1.2.1).
 */
MEMORY
{
    CODE   (rx)  : ORIGIN = 0x00000000, LENGTH = 192K
    RAM    (rwx) : ORIGIN = 0x20100000, LENGTH = 448K
    STACK  (rwx) : ORIGIN = 0x20170000, LENGTH =  32K
}

REGION_ALIAS("FLASH", CODE);
REGION_ALIAS("REGION_TEXT", CODE);
REGION_ALIAS("REGION_RODATA", CODE);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", STACK);
