/******************************************************************************
 *
 *  stm32f107xC_flash.ld - linker script for nutos on cotrex-m3 STM32F107xC
 *
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)
"bootloader_offset" = 0x00;

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    SRAM0 (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

INCLUDE stm32f10x_ram.ld

