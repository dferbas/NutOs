/*
 * fm25l04b.h - 4Kb SPI FRAM Memory
 *
 *  Created on: Aug 21, 2013
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <arch/m68k/coldfire/mcf51cn/spi_mcf51cn.h>

/*
 * Fram size (4096 / 8) = 512B
 */
#define FRAM_SIZE 	0x200 //512B

/*
 * Set slave select PTE2
 */
void fm25l04b_init(void);

/*
 * Table Status register
 * Bit  7 6 5 4 3   2   1   0
 * Name 0 0 0 0 BP1 BP0 WEL 0
 *
 * Table Block Memory Write Protection
 * BP1 BP0 Protected Address Range
 * 0   0   None
 * 0   1   180h to 1FFh
 * 1   0   100h to 1FFh
 * 1   1   000h to 1FFh (all)
 *
 * The WEL flag indicates the state of the
 * Write Enable Latch. Attempting to directly write the
 * WEL bit in the Status Register has no effect on its
 * state. This bit is internally set and cleared via the
 * WREN and WRDI commands, respectively.
*/
uint8_t fm25l04b_readStatus(void);
void 	fm25l04b_writeStatus(uint8_t status);

/*
 * Write fram block.
 * param/ address 0x0 - 0x1FF
 * param/ data - write buffer length of param/ size
 * param/ size - write data count
 */
void fm25l04b_write_block(uint16_t address, char *data, uint32_t size);

/*
 * Read fram block.
 * param/ address 0x0 - 0x1FF
 * param/ data - read buffer length of param/ size
 * param/ size - read data count
 */
void fm25l04b_read_block(uint16_t address, char *data, uint32_t size);
