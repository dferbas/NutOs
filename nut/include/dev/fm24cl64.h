/*
 * fram.h - 16Kb Serial 3V F-RAM Memory
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>

/*
 * Fram size (64 kBit / 8)
 */
#define FRAM_SIZE 0x2000

int FMReadBlock(uint16_t addr, void *rxdata, uint16_t rxsiz, uint32_t tmo);
int FMWriteBlock(uint16_t addr, CONST void *txdata, uint16_t txsiz, uint32_t tmo);
int FMReadByte(uint16_t addr, void *rxdata);
int FMWriteByte(uint16_t addr, CONST void *txdata);
void FMSetSlaveAddress(uint8_t newSlaveAddress);
