/******************************************************************************
 *
 *  stm32f103xC_flash.ld - linker script for nutos on cortex-m3 STM32F103xC
 *
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    SRAM0 (rwx) : ORIGIN = 0x20000000, LENGTH = 48K
}

INCLUDE stm32f10x_flash.ld
