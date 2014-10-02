/*
 * ds2482.h - 1 wire - Single-Channel 1-Wire Master
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>

#define ONE_WIRE_SLAVE_ADDRESS 0x18 //slave address

#define OW_TIMEOUT	10 //50ms

#define OW_STATUS_REGISTER 	0xF0
#define OW_DATA_REGISTER 	0xE1
#define OW_CONFIG_REGISTER	0xC3

#define OW_SR_DIR			0x80
#define OW_SR_TSB			0x40
#define OW_SR_SBR			0x20
#define OW_SR_RST			0x10
#define OW_SR_LL			0x08
#define OW_SR_SD			0x04
#define OW_SR_PPD			0x02
#define OW_SR_1WB			0x01

#define OW_CR_NOT_1WS		0x80
#define OW_CR_NOT_SPU		0x40
#define OW_CR_1				0x20
#define OW_CR_NOT_APU		0x10
#define OW_CR_1WS			0x08
#define OW_CR_SPU			0x04
#define OW_CR_0				0x02
#define OW_CR_APU			0x01

int OW_DeviceReset(void);
int OW_SetReadPointer(uint8_t reg);
int OW_WriteConfiguration(uint8_t pullup, uint8_t strong_pullup, uint8_t speed);
int OW_1Wire_Reset(void);
int OW_1Wire_WriteBit(uint8_t bit);
int OW_1Wire_ReadBit(void);
int OW_1Wire_WriteByte(uint8_t byte);
int OW_1Wire_ReadByte(uint8_t *p_byte);
int OW_1WireTriplet(uint8_t prev_bit, uint8_t *p_rslt_byte);


