/*
 * 24AA256/24LC256/24FC256
 *
 * eeprom24aa256.c - 256K I2C™ CMOS Serial EEPROM 24AA256
 * EEPROM array of (0000-7FFF)
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>
#include <sys/timer.h>
#include <dev/eeprom_24aa256.h>
#include <cfg/eeprom_24aa256.h>
#include <dev/twif.h>

#define EEPROM_BUSY_TIMEOUT	5 //5ms
#define EEPROM_RW_TIMEOUT	10 //50ms
#define EEPROM_PAGE_SIZE	64
#define EEPROM_BUSY_TESTS	10
#define min(a,b) ((a>b)?b:a)

extern int TwMasterRead(uint8_t sla, CONST void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo);
extern int TwMasterWrite(uint8_t sla, CONST void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz, uint32_t tmo);

/*!
 * \ACKNOWLEDGE POLLING - check if eeprom is busy
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 */
static void EeBusy(void)
{
	int i = EEPROM_BUSY_TESTS;
	while(TwMasterWrite(I2C_SLA_24AA256, 0, 0, 0, 0, EEPROM_BUSY_TIMEOUT) != 0 && i-- != 0);
}
/*!
 * \brief Receive Block data from Eeprom.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \param addr   Register address.
 *
 * \param rxdata Points to a buffer, where the received data will be
 *               stored. Ignored, if the maximum number of bytes to
 *               receive is zero.
 * \param rxsiz  Maximum number of bytes to receive. Set to zero, if
 *               no bytes are expected from the slave device.
 * \param tmo    Timeout in milliseconds. To disable timeout, set this
 *               parameter to NUT_WAIT_INFINITE.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int EEReadBlock(uint16_t addr, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
	EeBusy();
	if(rxsiz > 0){
		return TwMasterRead(I2C_SLA_24AA256, &addr, 2, rxdata, rxsiz, tmo);
	}
	return 0;
}

/*!
 * \brief Transmit Block data to Eeprom.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \param addr   Register address.
 *
 * \param txdata Points to the data to transmit. Ignored, if the number
 *               of data bytes to transmit is zero.
 * \param txlen  Number of data bytes to transmit. If zero, then the
 *               interface will not send any data to the slave device
 *               and will directly enter the master receive mode.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int EEWriteBlock(uint16_t addr, CONST void *txdata, uint16_t txsiz)
{
	uint8_t *p_txdata = (uint8_t *)txdata;
	uint16_t len = min(EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE), txsiz);

	while(txsiz > 0){
		EeBusy();
		if (TwMasterWrite(I2C_SLA_24AA256, &addr, 2, (void *)p_txdata, len, EEPROM_RW_TIMEOUT) < 0)
			return -1;
		txsiz -= len;
		p_txdata += len;
		addr += len;
		len = min(EEPROM_PAGE_SIZE, txsiz);
	}
	return 0;
}

/*!
 * \brief Receive Byte from Eeprom.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \param addr   Register address.
 *
 * \param txdata Points to the data to transmit. Ignored, if the number
 *               of data bytes to transmit is zero.
 * \param txlen  Number of data bytes to transmit. If zero, then the
 *               interface will not send any data to the slave device
 *               and will directly enter the master receive mode.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int EEReadByte(uint16_t addr, void *rxdata)
{
	EeBusy();
	return TwMasterRead(I2C_SLA_24AA256, &addr, 2, rxdata, 1, EEPROM_RW_TIMEOUT);
}

/*!
 * \brief Transmit Byte to Eeprom.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \param addr   Register address.
 *
 * \param txdata Points to the data to transmit. Ignored, if the number
 *               of data bytes to transmit is zero.
 * \param txlen  Number of data bytes to transmit. If zero, then the
 *               interface will not send any data to the slave device
 *               and will directly enter the master receive mode.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int EEWriteByte(uint16_t addr, CONST void *txdata)
{
	EeBusy();
	return TwMasterWrite(I2C_SLA_24AA256, &addr, 2, (void *)txdata, 1, EEPROM_RW_TIMEOUT);
}

