/*
 * mcp9800.c - TEMPERATURE SENSOR
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>
#include <dev/mcp9800.h>

#define MCP9800_SLAVE_ADDRESS 0x48 //slave address
#define MCP9800_TIMEOUT	5 //5ms

extern int TwMasterRead(uint8_t sla, CONST void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo);
extern int TwMasterWrite(uint8_t sla, CONST void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz, uint32_t tmo);

/*
 * \param temperature	Points to a buffer at size 2, where the received data
 * 						will be stored.
 *              		Upper Half (temperature[0]) bits are:
 *              			7 - Sign
 *              			6 - 2^6 Celsius
 *              			5 - 2^5 Celsius
 *              			4 - 2^4 Celsius
 *              			3 - 2^3 Celsius
 *              			2 - 2^2 Celsius
 *              			1 - 2^1 Celsius
 *              			0 - 2^0 Celsius
 *              		Lower Half (temperature[1]):
 *							7 - 2^-1 Celsius
 *              			6 - 2^-2 Celsius
 *              			5 - 2^-3 Celsius
 *              			4 - 2^-4 Celsius
 *              			0-3 - Bits are cleared
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int mcp9800GetTemperature(void *temperature)
{
    /* Temperature measurement */
	uint8_t addr = MCP9800_TEMPERATURE;
    return TwMasterRead(MCP9800_SLAVE_ADDRESS, &addr, 1, temperature, 2, MCP9800_TIMEOUT);
}

/*
 * \param req	- Choose register
 *					MCP9800_GET_RESOLUTION
 *					MCP9800_SET_RESOLUTION
 *				- where resolution values are:
 *					00 = 9 bit or 0.5°C (Power-up default)
 *					01 = 10 bit or 0.25°C
 *					10 = 11 bit or 0.125°C
 *					11 = 12 bit or 0.0625°C
 * \param req	- register value
 */
int mcp9800IOCtl(int cmd, void *conf)
{
	int rc = 0;
	uint8_t data;
	uint8_t addr = MCP9800_GET_CONF;
	if(TwMasterRead(MCP9800_SLAVE_ADDRESS, &addr, 1, &data, 1, MCP9800_TIMEOUT) == -1){
	    return -1;
	}

	switch (cmd) {
	case MCP9800_GET_CONF_RESOLUTION:
		*((uint8_t *)conf) = ((data >> 5) & 0x3);
		break;
	case MCP9800_SET_CONF_RESOLUTION:
		data &= ~0x60;
		data |= ((*(uint8_t *)conf) & 0x3) << 5;
		TwMasterWrite(MCP9800_SLAVE_ADDRESS, &addr, 1, &data, 1, MCP9800_TIMEOUT);
		break;
	default:
		rc = -1;
		break;
	}
	return rc;

}
