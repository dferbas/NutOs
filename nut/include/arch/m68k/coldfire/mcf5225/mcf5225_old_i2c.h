/*
 * Copyright 2012-2016 by Embedded Technologies s.r.o. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

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

#define DCB_BASE_TWI0 	0x0
#define DCB_BASE_TWI1 	0x80
#define DCB_BASE_MASK 	0x80

#define DCB_BASE(addr) 	(((addr) & DCB_BASE_MASK) >> 7)
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
