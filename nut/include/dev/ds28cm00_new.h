/*
 * silicon.h - 256K I2C™ CMOS Serial EEPROM
 *
 *  Created on: Mar 15, 2012
 *      Author: dchvalkovsky
 */
#include <dev/board.h>
#include <stdio.h>


int SiliconID_Init(void);
int SiliconID_IOControl(uint8_t control);
int SiliconID_GetID(uint8_t *p_id);
