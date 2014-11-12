/*
 * one_wire.c - Single-Channel 1-Wire Master
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>
#include <dev/ds2482_multi_sla.h>

extern int TwMasterRead(uint8_t sla, CONST void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo);
extern int TwMasterWrite(uint8_t sla, CONST void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz, uint32_t tmo);

/*
 * 1WB - 1-Wire busy check
 */
static uint8_t OWMBusy(uint8_t psla)
{
	uint8_t status = 0;

	OWM_SetReadPointer(psla, OWM_STATUS_REGISTER);

	while((TwMasterRead(I2C_FIXED_SLA_ONE_WIRE + psla, 0, 0, &status, 1, OWM_TIMEOUT) != 0)
			&& (status & OWM_SR_1WB))
	{
		;
	}
	return status;
}

//TODO: comment
int OWM_DeviceReset(uint8_t psla)
{
	/* Read one confirm byte, but we are not interested in it's content */
	uint8_t dummy;
	uint8_t addr = 0xF0;
	return TwMasterRead(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, &dummy, 1, OWM_TIMEOUT);
}

//TODO: comment
int OWM_SetReadPointer(uint8_t psla, uint8_t reg)
{
	uint8_t addr = 0xE1;
	return TwMasterWrite(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, &reg, 1, OWM_TIMEOUT);
}

//TODO: comment
int OWM_WriteConfiguration(uint8_t psla, uint8_t pullup, uint8_t strong_pullup, uint8_t speed)
{
	OWMBusy(psla);

	uint8_t control = (pullup ? OWM_CR_APU : 0)
					| (strong_pullup ? OWM_CR_SPU : 0)
					| (speed ? OWM_CR_1WS : 0);

	control |= ((control ^ 0xFF) << 4) & 0xF0;

	uint8_t addr = 0xD2;
	return TwMasterWrite(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, &control, 1, OWM_TIMEOUT);
}

//TODO: comment
int OWM_1Wire_Reset(uint8_t psla)
{
	OWMBusy(psla);

	uint8_t addr = 0xB4;

	if (TwMasterWrite(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, 0, 0, OWM_TIMEOUT) < 0)
		return -1;

	uint8_t state = OWMBusy(psla);

	int rc = 1;

	if (state & OWM_SR_SD)
		rc = -1;

	if (state & OWM_SR_PPD)
		rc = 0;

	return rc;
}

//TODO: comment
int OWM_1Wire_WriteBit(uint8_t psla, uint8_t bit)
{
	OWMBusy(psla);

	bit = bit ? 0x80 : 0;

	uint8_t addr = 0x87;
	return TwMasterWrite(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, &bit, 1, OWM_TIMEOUT);
}

//TODO: comment
int OWM_1Wire_ReadBit(uint8_t psla)
{
	OWMBusy(psla);

	uint8_t bit = 0;
	uint8_t addr = 0x87;

	if (TwMasterWrite(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, &bit, 1, OWM_TIMEOUT))
		return -1;

	uint8_t state = OWMBusy(psla);

	if (state & OWM_SR_SBR)
		return 1;
	return 0;
}

//TODO: comment
int OWM_1Wire_WriteByte(uint8_t psla, uint8_t byte)
{
	OWMBusy(psla);

	uint8_t addr = 0xA5;
	return TwMasterWrite(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, &byte, 1, OWM_TIMEOUT);
}

//TODO: comment
int OWM_1Wire_ReadByte(uint8_t psla, uint8_t *p_byte)
{
	OWMBusy(psla);

	uint8_t addr = 0x96;
	if (TwMasterWrite(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, 0, 0, OWM_TIMEOUT) < 0)
		return -1;

	OWMBusy(psla);

	uint8_t read_byte_cmd[] = {0xE1, OWM_DATA_REGISTER};
	return TwMasterRead(I2C_FIXED_SLA_ONE_WIRE + psla, read_byte_cmd, 2, p_byte, 1, OWM_TIMEOUT);
}

//TODO: comment
int OWM_1WireTriplet(uint8_t psla, uint8_t prev_bit, uint8_t *p_rslt_byte)
{
	OWMBusy(psla);

	prev_bit = prev_bit ? 0x80 : 0;

	uint8_t addr = 0x78;
	if (TwMasterWrite(I2C_FIXED_SLA_ONE_WIRE + psla, &addr, 1, &prev_bit, 1, OWM_TIMEOUT) < 0)
		return -1;

	uint8_t state = OWMBusy(psla);

	*p_rslt_byte = ((state & OWM_SR_TSB) ? 0x01 : 0)
				 | ((state & OWM_SR_SBR) ? 0x02 : 0)
				 | ((state & OWM_SR_DIR) ? 0x04 : 0);

	return 0;
}
