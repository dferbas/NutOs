/*
 * flash_sst25vf020B.c
 *
 * 2 Mbit SPI Serial Flash SST25VF020 API for mcf51cn_spi.c
 *
 * Destination Address 00000H - 3FFFFH
 *
 * http://ww1.microchip.com/downloads/en/DeviceDoc/25078A.pdf
 *
 *  Created on: Nov 5, 2013
 *	  Author: dchvalkovsky
 */
#include <dev/board.h>
#include <dev/flash_sst25vf020b.h>
#include <stdio.h>
#include <sys/event.h>
#include <sys/timer.h>

// command codes for SST25VF032B
#define WRITE_STATUS    	0x01 // called WRSR in datasheet
#define WRITE           	0x02 // Write Byte
#define READ           	 	0x03
#define HIGHER_SPEED_READ   0x0B
#define WRDI            	0x04
#define READ_STATUS     	0x05 // called RDSR
#define WREN            	0x06
#define EWSR            	0x50 // Enable Write Status Register
#define SECTOR_ERASE_4K     0x20
#define BLOCK_ERASE_32K     0x52
#define BLOCK_ERASE_64K     0xD8
#define CHIP_ERASE     		0x60 // or 0xC7
#define AAIWRITE        	0xAD // word based write
#define EBSY				0x70
#define DBSY				0x80

static HANDLE handle_flash_trans;

static void flash_wait_busy(void);
static void flash_wait_busy_AAI(void);

static void flash_CommandBegin(uint8_t command, uint32_t address);
static void flash_CommandEnd(void);
static void flash_command(uint8_t data);

static int flash_sc;

static void flash_cs_lo(void){
	Mcf51cnSpiSelect(flash_sc);
}

static void flash_cs_hi(void){
	Mcf51cnSpiDeselect(flash_sc);
}

/*
 * params sc - slave select value from spi_mcf51cn.h
 */
void flash_init(int sc)
{
	flash_sc = sc;
	int port;

	if (flash_sc == NODE_CS_PTE2)
		port = PORTE; // Powernet
	else
		port = PORTB; // SM2-RM

	GpioPinSetHigh(port, 2);
	GpioPinConfigSet(port, 2, GPIO_CFG_OUTPUT); // init PTB2 as output

	/* Write-Enable */
	flash_writeStatus(0x82); // 0x82
	NutEventPost(&handle_flash_trans);
}

/*
 * This procedure waits until device is no longer busy (can be used by
 * Byte-Program, Sector-Erase, Block-Erase, Chip-Erase).
 */
static void flash_wait_busy(void){
	while ((flash_readStatus() & 0x03) == 0x03)
		;
}

/*
 * This procedure waits until device is no longer busy for AAI mode.
 */
static void flash_wait_busy_AAI(void)
{
	while ((flash_readStatus() & 0x43) == 0x43)
		;
}

/*
 * This procedure make one separate command.
 */
static void flash_command(uint8_t data) {
	flash_cs_lo();
	Mcf51cnSpiTransfer(&data, NULL, 1);
	flash_cs_hi();
}

/*
 * This procedure erases the entire Chip.
 */
void flash_chip_erase(void) {
	NutEventWait(&handle_flash_trans, NUT_WAIT_INFINITE);
	flash_command(WREN);
	flash_command(CHIP_ERASE);
	flash_wait_busy();
	NutEventPost(&handle_flash_trans);
}

/*
 * This procedure Sector Erases the Chip.
 */
void flash_sector_erase_4K(uint32_t Dst)
{
	NutEventWait(&handle_flash_trans, NUT_WAIT_INFINITE);
	flash_command(WREN);
	flash_CommandBegin(SECTOR_ERASE_4K, Dst);
	flash_CommandEnd();
	flash_wait_busy();
	NutEventPost(&handle_flash_trans);
}

/*
 * This procedure Block Erases 32 KByte of the Chip.
 */
void flash_block_erase_32K(uint32_t Dst)
{
	NutEventWait(&handle_flash_trans, NUT_WAIT_INFINITE);
	flash_command(WREN);
	flash_CommandBegin(BLOCK_ERASE_32K, Dst);
	flash_CommandEnd();
	flash_wait_busy();
	NutEventPost(&handle_flash_trans);
}

/*
 * This procedure Block Erases 64 KByte of the Chip.
 */
void flash_block_erase_64K(uint32_t Dst)
{
	NutEventWait(&handle_flash_trans, NUT_WAIT_INFINITE);
	flash_command(WREN);
	flash_CommandBegin(BLOCK_ERASE_64K, Dst);
	flash_CommandEnd();
	flash_wait_busy();
	NutEventPost(&handle_flash_trans);
}

void flash_writeStatus(uint8_t status) {
	uint8_t dataWrite[2];
	
	/* Enable Write StatusRegister */
	flash_command(EWSR);

	dataWrite[0] = WRITE_STATUS;
	dataWrite[1] = status;

	flash_cs_lo();
	Mcf51cnSpiTransfer(dataWrite, NULL, 2);
    flash_cs_hi();
}

/*
 * This procedure read the status register and returns the byte.
 */
uint8_t flash_readStatus(void) {
	uint8_t dataRead[2];
	uint8_t dataWrite[2] = { READ_STATUS, 0xFF };

	flash_cs_lo();
	Mcf51cnSpiTransfer(dataWrite, dataRead, 2);
	flash_cs_hi();
	return dataRead[1];
}

/*
 * Begin of command, which send address
 */
static void flash_CommandBegin(uint8_t command, uint32_t address) {
	uint8_t dataWrite[4];

	dataWrite[0] = command;
	dataWrite[1] = (uint8_t) ((address & 0x3FFFF) >> 16);
	dataWrite[2] = (uint8_t) ((address & 0xFFFF)>> 8);
	dataWrite[3] = (uint8_t) (address & 0xFF);

	flash_cs_lo();
	Mcf51cnSpiTransfer(dataWrite, NULL, 4);
}

/*
 * End of command started with begin command.
 */
static inline void flash_CommandEnd(void) {
	flash_cs_hi();
}

/*
 * Static function for write byte without event wait.
 * The selected address must be in the erased state (FFH) when write byte.
 * Destination Address 00000H - 3FFFFH
 */
static void flash_write_byte_noEvantWait(uint32_t address, uint8_t byte) {
    flash_command(WREN);
    flash_CommandBegin(WRITE, address);
    Mcf51cnSpiTransfer(&byte, NULL, 1);
    flash_CommandEnd();
    flash_wait_busy();
}

/*
 * The selected address must be in the erased state (FFH) when write byte.
 * Destination Address 00000H - 3FFFFH
 *
 */
void flash_write_byte(uint32_t address, uint8_t byte) {
	NutEventWait(&handle_flash_trans, NUT_WAIT_INFINITE);
	flash_write_byte_noEvantWait(address, byte);
	NutEventPost(&handle_flash_trans);
}

/*
 * The selected address range must be in the erased state (FFH) when write.
 * Destination Address 00000H - 3FFFFH
 */
void flash_write_block(uint32_t address, uint8_t * buffer, uint32_t count) {

	uint8_t flash_buff[6];
	uint8_t *p_buff = buffer;
	uint8_t *p_buff_end = p_buff + count;
	int j;

	NutEventWait(&handle_flash_trans, NUT_WAIT_INFINITE);

	/* Zapsani prvniho bytu samostatne, pokud je adresa licha */
	if (address & 0x1) {
		flash_write_byte_noEvantWait(address, *p_buff);
		buffer = ++p_buff;
		count--;
		address ++;
	}

	flash_command(WREN);
	/* Pred kazdy zapsany dva byty je potreba vlozit AAI write command */
	flash_buff[0] = AAIWRITE;
	while (p_buff < (p_buff_end - (count & 0x1))) {
		j = 1;
		flash_cs_lo();
		/* Zapsani adresy zacatku AAI write prikazu */
		if (p_buff == buffer) {
			flash_buff[j++] = (address & 0xFFFFFF) >> 16;
			flash_buff[j++] = (address & 0xFFFF) >> 8;
			flash_buff[j++] = address & 0xFF;
		}
		flash_buff[j++] = *p_buff++;
		flash_buff[j++] = *p_buff++;

		Mcf51cnSpiTransfer(flash_buff, NULL, j);

		flash_cs_hi();
		flash_wait_busy();
	}

	flash_command(WRDI);
	flash_wait_busy_AAI();

	/* zapsani posledniho bytu na lichou adresu podle countu */
	if (count & 0x1){
		flash_write_byte_noEvantWait(address + count - 1, *p_buff);
	}
	NutEventPost(&handle_flash_trans);
}

/*
 * Destination Address 00000H - 3FFFFH
 */
void flash_read_block(uint32_t address, uint8_t * buffer, uint32_t count) {
	NutEventWait(&handle_flash_trans, NUT_WAIT_INFINITE);
	flash_CommandBegin(READ, address);
	Mcf51cnSpiTransfer(NULL, buffer, count);
	flash_CommandEnd();
	NutEventPost(&handle_flash_trans);
}

/*
 * Destination Address 00000H - 3FFFFH
 */
void flash_higher_speed_read_block(uint32_t address, uint8_t * buffer, uint32_t count) {
	uint8_t dummyChar = 0xFF;
	NutEventWait(&handle_flash_trans, NUT_WAIT_INFINITE);
	flash_CommandBegin(HIGHER_SPEED_READ, address);

	Mcf51cnSpiTransfer(NULL, &dummyChar, 1); // read/write dummy byte, necessary for higher speed read
	Mcf51cnSpiTransfer(NULL, buffer, count);

	flash_CommandEnd();
	NutEventPost(&handle_flash_trans);
}
