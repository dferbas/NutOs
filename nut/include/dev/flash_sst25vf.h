/*
 * dfsst25vf.h
 *
 *  Created on: Jul 25, 2012
 *      Author: dchvalkovsky
 */

#ifndef DFSST25VF_H_
#define DFSST25VF_H_

void flash_init(void);

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

#endif /* DFSST25VF_H_ */
