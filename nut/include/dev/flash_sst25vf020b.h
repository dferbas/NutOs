/*
 * dfsst25vf020b.h
 *
 *  Created on: Nov 5, 2012
 *      Author: dchvalkovsky
 */

#ifndef DFSST25VF020_H_
#define DFSST25VF020_H_

#include <stdint.h>
#include <arch/m68k/coldfire/mcf51cn/spi_mcf51cn.h>

#define SPI_FLASH_PAGE_SIZE		0x1000 // sector erase 4k
#define SPI_FLASH_PAGE_COUNT	64

/*
 * params sc - slave select value from spi_mcf51cn.h
 */
void flash_init(int sc);

void flash_chip_erase(void);

void flash_sector_erase_4K(uint32_t Dst);

void flash_block_erase_32K(uint32_t Dst);

void flash_block_erase_64K(uint32_t Dst);

void flash_writeStatus(uint8_t status);

uint8_t flash_readStatus(void);

void flash_write_byte(uint32_t address, uint8_t byte);

void flash_write_block(uint32_t address, uint8_t * buffer, uint32_t count);

void flash_read_block(uint32_t address, uint8_t * buffer, uint32_t count);

void flash_higher_speed_read_block(uint32_t address, uint8_t * buffer, uint32_t count);

#endif /* DFSST25VF020_H_ */
