/*
 * fm24cl64.c - 64Kb Serial 3V F-RAM Memory
 *
 * driver for FM24CL64
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <dev/fm24cl64.h>
#include <cfg/fm24cl64.h>
#include <dev/twif.h>

#define FRAM_RW_TIMEOUT	200 //200ms

uint8_t slaveAddress = I2C_SLA_FM24CL64;

extern int TwMasterRead(uint8_t sla, CONST void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo);
extern int TwMasterWrite(uint8_t sla, CONST void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz, uint32_t tmo);

/*!
 * \brief Receive Block data from F-RAM Memory.
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
int FMReadBlock(uint16_t addr, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
	return TwMasterRead(slaveAddress, &addr, 2, rxdata, rxsiz, tmo);
}

/*!
 * \brief Transmit Block data to F-RAM Memory.
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
 * \param tmo    Timeout in milliseconds. To disable timeout, set this
 *               parameter to NUT_WAIT_INFINITE.
 *
 * \return 0 if success, -1 in case of an error or timeout.
 */
int FMWriteBlock(uint16_t addr, CONST void *txdata, uint16_t txsiz, uint32_t tmo)
{
	return TwMasterWrite(slaveAddress, &addr, 2, (void *)txdata, txsiz, tmo);
}

/*!
 * \brief Receive Byte from F-RAM Memory.
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
int FMReadByte(uint16_t addr, void *rxdata)
{
	return TwMasterRead(slaveAddress, &addr, 2, (void *)rxdata, 1, FRAM_RW_TIMEOUT);
}

/*!
 * \brief Transmit Byte to F-RAM Memory.
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
int FMWriteByte(uint16_t addr, CONST void *txdata)
{
	return TwMasterWrite(slaveAddress, &addr, 2, (void *)txdata, 1, FRAM_RW_TIMEOUT);
}

/*!
 *
 * Set slave address, if not the same as in Nut/OS Configurator
 */
void FMSetSlaveAddress(uint8_t newSlaveAddress)
{
	slaveAddress = newSlaveAddress;
}
