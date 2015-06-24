/*
 * EEPROM AT24C16
 *
 * eeprom_at24c16.c - 2048KB Two-wire Serial EEPROM AT24C16
 *
 *  Created on: 25.5. 2015
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>
#include <sys/timer.h>
#include <dev/eeprom_at24c16.h>
#include <cfg/eeprom_at24c16.h>
#include <dev/twif.h>

#define EEPROM_PAGE_SIZE 		16
#define EEPROM_PAGE_COUNT 		128
#define EEPROM_BLOCK_SIZE 		0x100
#define EEPROM_BUSY_TESTS 		10
#define EEPROM_BUSY_TIMEOUT		10 //5ms
#define EEPROM_RW_TIMEOUT		50 //50ms
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
int At24c16Busy(void)
{
	int i = EEPROM_BUSY_TESTS;
	while (TwMasterWrite(I2C_SLA_AT24C16, 0, 0, 0, 0, EEPROM_BUSY_TIMEOUT) != 0 && i-- != 0);

	if (i == 0)
		return -1;

	return 0;
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
int At24c16ReadBlock(uint16_t addr, void *rxdata, uint16_t rxsiz)
{
	uint8_t eeAddrL, eeAddrH;
	uint8_t *p_txdata = (uint8_t *) rxdata;
	uint16_t size = rxsiz;

	uint16_t len = min(EEPROM_BLOCK_SIZE - (addr % EEPROM_BLOCK_SIZE), size);

	while (size > 0) {

		eeAddrL = (uint8_t) addr;// prepocet pro kazdy blok
		eeAddrH = (uint8_t) (addr >> 8);
		At24c16Busy();
		if (TwMasterRead(I2C_SLA_AT24C16 + eeAddrH, &eeAddrL, 1, p_txdata, len, EEPROM_BUSY_TIMEOUT) == -1)
			return -1;
		size -= len;
		p_txdata += len;
		addr += len;
		len = min(EEPROM_BLOCK_SIZE, size);
	}
	return rxsiz;
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
int At24c16WriteBlock(uint16_t addr, CONST void *txdata, uint16_t txsiz)
{
	uint8_t eeAddrL, eeAddrH;
	uint8_t *p_txdata = (uint8_t *) txdata;
	uint16_t size = txsiz;

	uint16_t len = min(EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE), size);

	while (size > 0) {
		eeAddrL = (uint8_t) addr; // prepocet pro kazdy page
		eeAddrH = (uint8_t) (addr >> 8);
		At24c16Busy();
		if (TwMasterWrite(I2C_SLA_AT24C16 + eeAddrH, &eeAddrL, 1, p_txdata, len, EEPROM_RW_TIMEOUT) == -1)
			return -1;

		size -= len;
		p_txdata += len;
		addr += len;
		len = min(EEPROM_PAGE_SIZE, size);
	}
	return txsiz;
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
int At24c16ReadByte(uint16_t addr, void *rxdata)
{
	uint8_t eeAddrL = (uint8_t) addr; // prepocet pro kazdy page
	uint8_t eeAddrH = (uint8_t) (addr >> 8);

	At24c16Busy();
	return TwMasterRead(I2C_SLA_AT24C16 + eeAddrH, &eeAddrL, 1, rxdata, 1, EEPROM_RW_TIMEOUT);
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
int At24c16WriteByte(uint16_t addr, CONST void *txdata)
{
	uint8_t eeAddrL = (uint8_t) addr; // prepocet pro kazdy page
	uint8_t eeAddrH = (uint8_t) (addr >> 8);

	At24c16Busy();
	return TwMasterWrite(I2C_SLA_AT24C16 + eeAddrH, &eeAddrL, 1, (void *) txdata, 1, EEPROM_RW_TIMEOUT);
}

