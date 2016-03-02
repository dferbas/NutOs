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

#include <cfg/clock.h>
#include <arch/m68k.h>
#include <sys/nutdebug.h>

/*!
 * \addtogroup xgMcf51qe
 */
/*@{*/

void Mcf51qeIcsInitClock(void)
{
	/*  System clock initialization */
	/* ICSC1: CLKS=0,RDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
	MCF_ICS_C1 = 0x1AU; /* Initialization of the ICS control register 1 */
	/* ICSC2: BDIV=0,RANGE=1,HGO=1,LP=0,EREFS=1,ERCLKEN=0,EREFSTEN=0 */
	MCF_ICS_C2 = 0x34U; /* Initialization of the ICS control register 2 */
	while (!(MCF_ICS_SC & MCF_ICS_SC_OSCINIT))
	{ /* Wait until the initialization of the external crystal oscillator is completed */
		MCF_SRS = 0x00U; /* Reset watchdog counter */
	}
	/* ICSSC: DRST_DRS=1,DMX32=0 */
	MCF_ICS_SC = (unsigned char) ((MCF_ICS_SC & (unsigned char) ~0xA0U) | (unsigned char) 0x40U); /* Initialization of the ICS status and control */
	while ((MCF_ICS_SC & 0xC0U) != 0x40U)
	{ /* Wait until the FLL switches to Mid range DCO mode */
		MCF_SRS = 0x00U; /* Reset watchdog counter */
	}

	/* INTC_WCR: ENB=0,MASK=0 */
	MCF_INTC_WCR = 0x00U;
}
