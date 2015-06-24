/*
 * mcf5xxxx_twi.h
 *
 *  Created on: Aug 17, 2012
 *      Author: dchvalkovsky
 */

#ifndef MCF5XXXX_TWI_H_
#define MCF5XXXX_TWI_H_

#include <sys/types.h>


/* Mask definitions used in 'MainComm' internal method */

#define DCB_BASE_TWI1 	0x0
#define DCB_BASE_TWI2 	0x80
#define DCB_BASE_MASK 	0x80

#define DCB_BASE(addr) 	((((addr) & DCB_BASE_MASK) >> 7) + 1)
#define TWI_THREAD_WAIT 100

int TwMasterCommon(uint8_t sla, const void *addr, uint16_t addrsiz, void *data, uint16_t siz, uint32_t tmo, uint8_t write);

int TwMasterTransact(uint8_t sla, const void *txdata, uint16_t txlen, void *rxdata, uint16_t rxsiz, uint32_t tmo);

int TwMasterRead(uint8_t sla, const void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo);

int TwMasterWrite(uint8_t sla, const void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz, uint32_t tmo);

/* Only for I2C0 */
int TwMasterError(void);


int TwIOCtl(int req, void *conf);

int TwInit(uint8_t sla);

#endif /* MCF5XXXX_TWI_H_ */
