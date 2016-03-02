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
#include <dev/gpio.h>

uint16_t Mcf51qeAdcValue(void)
{
	MCF_ADC_SC1 = 0x5F; // One conversion only (continuous conversions disabled)
	return MCF_ADC_R;
}

void Mcf51qeAdcInit(void (*handler)(void *), void *handler_arg)
{
	MCF_SCGC1 |= MCF_SCGC1_ADC; // enable system clock
	NutRegisterIrqHandler(&sig_ADC, handler, handler_arg);
	MCF_APCTL2 = MCF_APCTL2_ADPC10 | MCF_APCTL2_ADPC11 | MCF_APCTL2_ADPC12;
	MCF_ADC_CFG = MCF_ADC_CFG_MODE(0x1) | MCF_ADC_CFG_ADLSMP | MCF_ADC_CFG_ADLPC
			| MCF_ADC_CFG_ADIV(0x3); // 12bit conversion
	MCF_ADC_CV = 0;
	MCF_ADC_SC2 = 0;
	MCF_ADC_SC1 = 0x5F; // One conversion only (continuous conversions disabled)
//	NutIrqEnable(&sig_ADC);
}

void Mcf51qeAdcStartConversion(uint8_t channel, uint8_t mode)
{
//	uint8_t conf;
//	conf = (MCF_ADC_CFG & ~MCF_ADC_CFG_MODE_MASK) | MCF_ADC_CFG_MODE(mode);
//	MCF_ADC_CFG = conf;
//	//Input channel 1 selected as ADC input channel
	MCF_ADC_SC1 = 0 | MCF_ADC_SC1_AIEN | MCF_ADC_SC1_ADCH(channel); //this starts AD conversion
}

