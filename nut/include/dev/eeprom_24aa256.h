/*
 * eeprom.c - 256K I2C™ CMOS Serial EEPROM
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>

int EEReadBlock(uint16_t addr, void *rxdata, uint16_t rxsiz, uint32_t tmo);
int EEWriteBlock(uint16_t addr, CONST void *txdata, uint16_t txsiz);
int EEReadByte(uint16_t addr, void *rxdata);
int EEWriteByte(uint16_t addr, CONST void *txdata);
