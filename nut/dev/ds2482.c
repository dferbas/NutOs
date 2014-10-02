/*
 * one_wire.c - Single-Channel 1-Wire Master
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>
#include <dev/ds2482.h>

extern int TwMasterRead(uint8_t sla, CONST void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo);
extern int TwMasterWrite(uint8_t sla, CONST void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz, uint32_t tmo);

/*
 * 1WB - 1-Wire busy check
 */
static uint8_t OwBusy(void)
{
	uint8_t status = 0;

	OW_SetReadPointer(OW_STATUS_REGISTER);

	while((TwMasterRead(ONE_WIRE_SLAVE_ADDRESS, 0, 0, &status, 1, OW_TIMEOUT) != 0)
			&& (status & OW_SR_1WB))
	{
		;
	}
	return status;
}

//TODO: comment
int OW_DeviceReset(void)
{
	/* Read one confirm byte, but we are not interested in it's content */
	uint8_t dummy;
	uint8_t addr = 0xF0;
	return TwMasterRead(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, &dummy, 1, OW_TIMEOUT);
}

//TODO: comment
int OW_SetReadPointer(uint8_t reg)
{
	uint8_t addr = 0xE1;
	return TwMasterWrite(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, &reg, 1, OW_TIMEOUT);
}

//TODO: comment
int OW_WriteConfiguration(uint8_t pullup, uint8_t strong_pullup, uint8_t speed)
{
	OwBusy();

	uint8_t control = (pullup ? OW_CR_APU : 0)
					| (strong_pullup ? OW_CR_SPU : 0)
					| (speed ? OW_CR_1WS : 0);

	control |= ((control ^ 0xFF) << 4) & 0xF0;

	uint8_t addr = 0xD2;
	return TwMasterWrite(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, &control, 1, OW_TIMEOUT);
}

//TODO: comment
int OW_1Wire_Reset(void)
{
	OwBusy();

	uint8_t addr = 0xB4;

	if (TwMasterWrite(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, 0, 0, OW_TIMEOUT) < 0)
		return -1;

	uint8_t state = OwBusy();

	int rc = 1;

	if (state & OW_SR_SD)
		rc = -1;

	if (state & OW_SR_PPD)
		rc = 0;

	return rc;
}

//TODO: comment
int OW_1Wire_WriteBit(uint8_t bit)
{
	OwBusy();

	bit = bit ? 0x80 : 0;

	uint8_t addr = 0x87;
	return TwMasterWrite(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, &bit, 1, OW_TIMEOUT);
}

//TODO: comment
int OW_1Wire_ReadBit(void)
{
	OwBusy();

	uint8_t bit = 0;
	uint8_t addr = 0x87;

	if (TwMasterWrite(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, &bit, 1, OW_TIMEOUT))
		return -1;

	uint8_t state = OwBusy();

	if (state & OW_SR_SBR)
		return 1;
	return 0;
}

//TODO: comment
int OW_1Wire_WriteByte(uint8_t byte)
{
	OwBusy();

	uint8_t addr = 0xA5;
	return TwMasterWrite(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, &byte, 1, OW_TIMEOUT);
}

//TODO: comment
int OW_1Wire_ReadByte(uint8_t *p_byte)
{
	OwBusy();

	uint8_t addr = 0x96;
	if (TwMasterWrite(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, 0, 0, OW_TIMEOUT) < 0)
		return -1;

	OwBusy();

	uint8_t read_byte_cmd[] = {0xE1, OW_DATA_REGISTER};
	return TwMasterRead(ONE_WIRE_SLAVE_ADDRESS, read_byte_cmd, 2, p_byte, 1, OW_TIMEOUT);
}

//TODO: comment
int OW_1WireTriplet(uint8_t prev_bit, uint8_t *p_rslt_byte)
{
	OwBusy();

	prev_bit = prev_bit ? 0x80 : 0;

	uint8_t addr = 0x78;
	if (TwMasterWrite(ONE_WIRE_SLAVE_ADDRESS, &addr, 1, &prev_bit, 1, OW_TIMEOUT) < 0)
		return -1;

	uint8_t state = OwBusy();

	*p_rslt_byte = ((state & OW_SR_TSB) ? 0x01 : 0)
				 | ((state & OW_SR_SBR) ? 0x02 : 0)
				 | ((state & OW_SR_DIR) ? 0x04 : 0);

	return 0;
}
