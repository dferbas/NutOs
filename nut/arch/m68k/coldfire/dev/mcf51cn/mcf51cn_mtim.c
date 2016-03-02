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

#include <arch/m68k.h>
#include <dev/irqreg.h>
#include <arch/m68k/coldfire/mcf51cn/mcg_mcf51cn.h>

#define MODULO_COUNTER_SIZE 0x100

void Mcf51cnMtimInitClock(void (*handler)(void *))
{

	uint32_t mcgFFClk;
	uint8_t psCounter = 0;
	uint16_t moduloCounter;

	// Bus clock to the MTIM1 module is enabled.
	MCF_SCGC1 |= MCF_SCGC1_MTIM1;

	// Stop timer
	MCF_MTIM_SC(1) = MCF_MTIM_SC_TSTP;

	// Get Fixed-frequency clock
	mcgFFClk = Mcf51cnMcgGetFFCLK() / (2 * 1000); // divide by tick frequency

	if (mcgFFClk)
	{
		moduloCounter = mcgFFClk / MODULO_COUNTER_SIZE;

		// count prescaler
		while (moduloCounter >= 1)
		{
			moduloCounter >>= 1;
			psCounter++;
		}

		// set modulo value
		moduloCounter = (mcgFFClk / (1 << psCounter));
		MCF_MTIM_MOD(1) = (uint8_t) moduloCounter;

		// if psCounter >= 8 than divide MTIM clock source by 256.
		MCF_MTIM_CLK(1) = (1 << MCF_MTIM_CLK_CLKS_BITNUM) | (psCounter << MCF_MTIM_CLK_PS_BITNUM);

		NutRegisterIrqHandler(&sig_MTIM1, handler, 0);

		/* Reset and start counter */
		MCF_MTIM_SC(1) = MCF_MTIM_SC_TRST;
	}
}
