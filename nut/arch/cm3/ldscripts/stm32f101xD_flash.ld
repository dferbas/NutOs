/******************************************************************************
 *
 *  stm32f101xD_flash.ld - linker script for nutos on cotrex-m3 STM32F101xD
 *
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x08000000, LENGTH = 384K
    SRAM0 (rwx) : ORIGIN = 0x20000000, LENGTH = 48K
}

INCLUDE stm32f10x_flash.ld
