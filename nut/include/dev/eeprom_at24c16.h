/*
 * EEPROM AT24C16
 *
 * eeprom_at24c16.h - 2048KB Two-wire Serial EEPROM AT24C16
 *
 *  Created on: 25.5. 2015
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>

int At24c16Busy(void);
int At24c16ReadBlock(uint16_t addr, void *rxdata, uint16_t rxsiz);
int At24c16WriteBlock(uint16_t addr, CONST void *txdata, uint16_t txsiz);
int At24c16ReadByte(uint16_t addr, void *rxdata);
int At24c16WriteByte(uint16_t addr, CONST void *txdata);
