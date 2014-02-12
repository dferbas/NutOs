/******************************************************************************
 *
 *  stm32f105x8_flash.ld - linker script for nutos on cortex-m3 STM32F105x8
 *
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    SRAM0 (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

INCLUDE stm32f10x_flash.ld
