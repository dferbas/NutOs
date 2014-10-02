/*
 * silicon.c - 256K I2C™ CMOS Serial EEPROM
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>
#include <dev/ds28cm00.h>
#include <string.h>

#define SILICON_ID_SLAVE_ADDRESS 0x50 //slave address
#define SILICON_TIMEOUT	50

#define SILICON_ID_LENGTH 7

#define SILICON_MODE_I2C 0x00
#define SILICON_MODE_SMB 0x01

extern int TwMasterRead(uint8_t sla, CONST void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo);
extern int TwMasterWrite(uint8_t sla, CONST void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz, uint32_t tmo);

/*!
 * \brief Init Silicon ID IOControl register.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \return 0 success, -1 in case of an error or timeout.
 */
int SiliconID_Init(void)
{
	return SiliconID_IOControl(SILICON_MODE_I2C);
}

/*
 * \brief Silicon ID IOControl register.
 *
 * \param control allows switching between:
 * 		SILICON_MODE_I2C 0x00 - I2C mode
 * 		SILICON_MODE_SMB 0x01 - SMBus mode
 * \return 0 success, -1 in case of an error or timeout.
 */
int SiliconID_IOControl(uint8_t control)
{
	uint8_t address = 0x08;
	if(TwMasterWrite(SILICON_ID_SLAVE_ADDRESS, &address, 1, &control, 1, SILICON_TIMEOUT) != 0){
		return -1;
	}
	return 0;
}

static uint8_t getCRC8( uint8_t *addr, uint8_t len)
{
	uint8_t crc=0, i, j;

	for (i=0; i<len;i++)
	{
		uint8_t inbyte = addr[i];
		for (j=0;j<8;j++)
		{
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0x8C;

			inbyte >>= 1;
		}
	}
	return crc;
}

/*!
 * \brief Silicon ID read registers.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \return 0 if success, -1 in case of an error or timeout.
 */
int SiliconID_GetID(uint8_t *p_id)
{
	uint8_t crc;
	uint8_t siliconId[SILICON_ID_LENGTH + 1];
	uint8_t address = 0x00;

	//Precte ID, overi CRC a vrati 0, pokud OK. Do p_id zkopiruje uz jen zvalidovane ID.
	if(TwMasterRead(SILICON_ID_SLAVE_ADDRESS, &address, 1, siliconId, SILICON_ID_LENGTH + 1, SILICON_TIMEOUT) != (SILICON_ID_LENGTH + 1)){
		return -1;
	}
	crc = getCRC8(siliconId, SILICON_ID_LENGTH);
	if(crc == siliconId[SILICON_ID_LENGTH]){
		memcpy(p_id, siliconId, SILICON_ID_LENGTH);
		return 0;
	}

	return -1;
}
