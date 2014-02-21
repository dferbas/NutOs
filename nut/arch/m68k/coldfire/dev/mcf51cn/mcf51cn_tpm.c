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

int Mcf51cnPtmInitInputCapture(void (*handler) (void *), void *handler_arg, uint32_t devnum, uint32_t channel){
	int bank, bit;
	uint32_t flags = GPIO_CFG_ALT3;
	IRQ_HANDLER *p_irqHandler = 0;


	if (devnum == 1)
	{
		bank = PORTE;
		if (channel == 0)
		{
			bit= 3;
			p_irqHandler = &sig_TPM1_CH0;
		}
		else if (channel == 1)
		{
			bit= 4;
			p_irqHandler = &sig_TPM1_CH1;
		}
		else if (channel == 2)
		{
			bit= 5;
			p_irqHandler = &sig_TPM1_CH2;
		}
		else
			return -1;

	}
	else if (devnum == 2)
	{
		// TODO test it, more choices of using ports
		if (channel == 0)
		{
			bank = PORTB;
			bit= 6;
			p_irqHandler = &sig_TPM2_CH0;
		}
		else if (channel == 1)
		{
			bank = PORTB;
			bit= 7;
			p_irqHandler = &sig_TPM2_CH1;
		}
		else if (channel == 2)
		{
			bank = PORTC;
			bit= 0;
			p_irqHandler = &sig_TPM2_CH2;
		}
		else
			return -1;
	}
	else
		return -1;

	GpioPinConfigSet(bank, bit, flags);

	/* Stop HW */
	MCF_TPM_SC(devnum) = 0;

	/* Disable modulo register */
	MCF_TPM_MOD(devnum) = 0;

	/* Reset counter */
	MCF_TPM_CNT(devnum) = 0;

	/* Clear capture register */
	MCF_TPM_CV(devnum, channel) = 0;

	/* Set Prescaler, not used */
//	MCF_TPM_SC(devnum) |= MCF_TPM_SC_PS(0x7);
	MCF_TPM_SC(devnum) &= ~MCF_TPM_SC_PS(0x7);

	/* Input capture, Capture on rising edge only */
	MCF_TPM_CSC(devnum, channel) = 0 | MCF_TPM_CSC_MSnx(0x0) | MCF_TPM_CSC_ELSnx(0x2);

	NutRegisterIrqHandler(p_irqHandler, handler, handler_arg);

	NutIrqEnable(p_irqHandler);

	/* Run counter. */
	MCF_TPM_SC(devnum) |= MCF_TPM_SC_CLKSx(0x1);

	return 0;
}
