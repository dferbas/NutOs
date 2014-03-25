/*
 * 	fm25l04b.c
 *
 *	FM25L04B 4Kb Serial 3V F-RAM Memory32 Mbit SPI Serial Flash SST25VF
 *
 *	9-bits specifies each byte address 0x0 - 0x1FF
 *
 *  Created on: Sep 13, 2013
 *      Author: dchvalkovsky
 */
#include <arch/m68k.h>
#include <dev/board.h>
#include <stdio.h>
#include <dev/flash_sst25vf020b.h>

#define SPI_BUS_WAITING_TIMEOUT 1000

// Op-code Commands
#define WRSR 	0x1	//Write Status Register 0000 0001b
#define WRITE 	0x2	//Write Memory Data 0000 A010b
#define READ 	0x3	//Read Memory Data 0000 A011b
#define WRDI 	0x4	//Write Disable 0000 0100b
#define RDSR 	0x5	//Read Status Register 0000 0101b
#define WREN 	0x6	//Set Write Enable Latch 0000 0110b

static void fm25l04b_cs_lo(void){
	Mcf51cnSpiSelect(NODE_CS_PTE2);
}

static void fm25l04b_cs_hi(void){
	Mcf51cnSpiDeselect(NODE_CS_PTE2);
}

/*
 * Init slave select
 */
void fm25l04b_init(void) {
	GpioPinSetHigh(PORTE, 2); // SM2-RM
	GpioPinConfigSet(PORTE, 2, GPIO_CFG_OUTPUT);
}

/*
 * WREN command is required before any write command
 */
static void fm25l04b_wren(void) {
	uint8_t data = WREN;
	fm25l04b_cs_lo();
	Mcf51cnSpiTransfer(&data, NULL, 1);
    fm25l04b_cs_hi();

}

/*
 * This procedure read the status register and returns the byte.
 */
uint8_t fm25l04b_readStatus(void) {
	uint8_t dataWrite[2] = {RDSR, 0xFF};
	uint8_t dataRead[2];

	fm25l04b_cs_lo();
	Mcf51cnSpiTransfer(dataWrite, dataRead, 2);
    fm25l04b_cs_hi();
    return dataRead[1];
}

void fm25l04b_writeStatus(uint8_t status) {

	uint8_t dataWrite[2];

	fm25l04b_wren();

	dataWrite[0] = WRSR;
	dataWrite[1] = status;

	fm25l04b_cs_lo();
	Mcf51cnSpiTransfer(dataWrite, NULL, 2);
    fm25l04b_cs_hi();
}

static void fm25l04b_CommandBegin(uint8_t command, uint16_t address) {
	uint8_t dataWrite[2];

	dataWrite[0] = (uint8_t) (command | ((address & 0x100) >> (8 - 3)));
	dataWrite[1] = (uint8_t) (address & 0xFF);

	fm25l04b_cs_lo();
	Mcf51cnSpiTransfer(dataWrite, NULL, 2);
}

static inline void fm25l04b_CommandEnd(void) {
	fm25l04b_cs_hi();
}

/*
 * Write fram block.
 * param/ address 0x0 - 0x1FF
 * param/ data - write buffer length of param/ size
 * param/ size - write data count
 */
void fm25l04b_write_block(uint16_t address, uint8_t *data, uint32_t size) {
	fm25l04b_wren();
	fm25l04b_CommandBegin(WRITE, address);
	Mcf51cnSpiTransfer(data, NULL, size);
	fm25l04b_CommandEnd();
}

/*
 * Read fram block.
 * param/ address 0x0 - 0x1FF
 * param/ data - read buffer length of param/ size
 * param/ size - read data count
 */
void fm25l04b_read_block(uint16_t address, uint8_t *data, uint32_t size) {
	fm25l04b_CommandBegin(READ, address);
	Mcf51cnSpiTransfer(NULL, data, size);
	fm25l04b_CommandEnd();
}
