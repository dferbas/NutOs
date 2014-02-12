/******************************************************************************
 *
 *  stm32f405xR.ld - linker script for nutos on cortex-m3 STM32F405xC
 *
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)

"bootloader_offset" = 0x00;

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    SRAM0 (rwx) : ORIGIN = 0x20000000, LENGTH = 112K
    DMA   (rwx) : ORIGIN = 0x2001c000, LENGTH = 16K
    BKPSRAM(rwx) : ORIGIN = 0x40024000, LENGTH = 4k
}

INCLUDE stm32f10x_ram.ld

