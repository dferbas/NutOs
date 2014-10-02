/*
 * mcp9800.h - TEMPERATURE SENSOR
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>

#define MCP9800_TEMPERATURE 		0
#define MCP9800_GET_CONF		 	1
#define MCP9800_GET_HYSTERESIS 		2
#define MCP9800_GET_LIMIT_SET 		3

#define MCP9800_GET_CONF_RESOLUTION	0
#define MCP9800_SET_CONF_RESOLUTION 1


int mcp9800GetTemperature(void *temperature);

int mcp9800IOCtl(int req, void *conf);

