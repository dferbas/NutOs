/*
 * Copyright 2012 by Embedded Technologies s.r.o
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
#include <dev/gpio.h>
#include <sys/timer.h>

#define MODULO_COUNTER_SIZE 0x100

/* System timer ticks, one tick per 1ms */
void Mcf51qeTpmInitClock(void (*handler) (void *))
{
	uint16_t mod;

	/* Enable Peripheral clock gating */
	MCF_SCGC1 |= MCF_SCGC1_TPM1;
	MCF_TPM_SC(1) = 0;

	mod = ((NutGetCpuClock() / 2) / 1000);
	MCF_TPM_MOD(1) = mod;

	NutRegisterIrqHandler(&sig_TPM1_OVFL, handler, 0);

	/* PS 000 - Clock source divided by 1,  CLKS 01 - Bus rate clock, Enable Overflow IRQ */
	MCF_TPM_SC(1) = MCF_TPM_SC_PS(0x0) | MCF_TPM_SC_CLKSx(0x01) | MCF_TPM_SC_TOIE;
}

