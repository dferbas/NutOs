/*
 * ds2482.h - 1 wire - Single-Channel 1-Wire Master
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>

#define I2C_FIXED_SLA_ONE_WIRE 0x18 //slave address

#define OWM_TIMEOUT	10 //50ms

#define OWM_STATUS_REGISTER 0xF0
#define OWM_DATA_REGISTER 	0xE1
#define OWM_CONFIG_REGISTER	0xC3

#define OWM_SR_DIR			0x80
#define OWM_SR_TSB			0x40
#define OWM_SR_SBR			0x20
#define OWM_SR_RST			0x10
#define OWM_SR_LL			0x08
#define OWM_SR_SD			0x04
#define OWM_SR_PPD			0x02
#define OWM_SR_1WB			0x01

#define OWM_CR_NOT_1WS		0x80
#define OWM_CR_NOT_SPU		0x40
#define OWM_CR_1			0x20
#define OWM_CR_NOT_APU		0x10
#define OWM_CR_1WS			0x08
#define OWM_CR_SPU			0x04
#define OWM_CR_0			0x02
#define OWM_CR_APU			0x01

int OWM_DeviceReset(uint8_t psla);
int OWM_SetReadPointer(uint8_t psla, uint8_t reg);
int OWM_WriteConfiguration(uint8_t psla, uint8_t pullup, uint8_t strong_pullup, uint8_t speed);
int OWM_1Wire_Reset(uint8_t psla);
int OWM_1Wire_WriteBit(uint8_t psla, uint8_t bit);
int OWM_1Wire_ReadBit(uint8_t psla);
int OWM_1Wire_WriteByte(uint8_t psla, uint8_t byte);
int OWM_1Wire_ReadByte(uint8_t psla, uint8_t *p_byte);
int OWM_1WireTriplet(uint8_t psla, uint8_t prev_bit, uint8_t *p_rslt_byte);


