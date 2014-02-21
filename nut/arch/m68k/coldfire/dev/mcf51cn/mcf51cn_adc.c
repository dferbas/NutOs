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
#include <arch/m68k/coldfire/mcf51cn/adc_mcf51cn.h>
#include <cfg/peripherals.h>
#include <dev/gpio.h>
#include <dev/irqreg.h>
#include <stdio.h>
#include <sys/atom.h>
#include <sys/event.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/timer.h>

#ifndef MCF51_ADC_INITIAL_MODE
#define MCF51_ADC_INITIAL_MODE SINGLE_CONVERSION
#endif

#ifndef MCF51_ADC_INITIAL_PRESCALE
#define MCF51_ADC_INITIAL_PRESCALE 55
#endif

#define MCF51_ADC_BUF_SIZE 2 // this may only be a power of two

#define _adc_buf_head MCF51_ADC_BUF_SIZE
#define _adc_buf_tail MCF51_ADC_BUF_SIZE+1

uint16_t **ADC_Buffer = NULL;
uint32_t adcEnableChannels;

static HANDLE readHandle;

/*!
 * \brief Reads data from the adc buffer
 *
 * \param channel  Specifies the channel to read data from
 * \param read     Variable to store the data in
 * \return 0: data read succesfully, 1: no data available
 */

int Mcf51cnAdcBufRead(uint16_t channel, uint16_t * read)
{
    uint16_t tail, head;
    tail = ADC_Buffer[channel][_adc_buf_tail];
    head = ADC_Buffer[channel][_adc_buf_head];
    if (head != tail) {
        *read = ADC_Buffer[channel][tail];
        ADC_Buffer[channel][_adc_buf_tail] = (tail + 1) & (MCF51_ADC_BUF_SIZE - 1);

        return 0;
    }
    return 1;
}

/* Store data in the buffer, called from interrupt */
static inline int Mcf51cnAdcBufWrite(uint16_t channel, uint16_t write)
{
    uint16_t tail, head;
    tail = ADC_Buffer[channel][_adc_buf_tail];
    head = ADC_Buffer[channel][_adc_buf_head];
    if (((head + 1) & (MCF51_ADC_BUF_SIZE - 1)) != tail) {
        ADC_Buffer[channel][head] = write;
        ADC_Buffer[channel][_adc_buf_head] = (head + 1) & (MCF51_ADC_BUF_SIZE - 1);
        return 0;
    }

    return 1;
}

/*!
 * \brief Sets the data aquisition mode for the adc
 *
 * \param mode  Mode to set
 */
void Mcf51cnAdcSetMode(TADCMode mode)
{
    switch (mode) {
        case ADC_OFF:
            /*
             * The successive approximation converter subsystem
             * is turned off when the channel select bits are all set.
             */
            MCF_ADC_SC1 |= MCF_ADC_SC1_ADCH;
            break;
        case SINGLE_CONVERSION:

            break;
    }
}

/*!
 * \brief Enable a channel used to sample when conversion started
 *
 * \param channel  Specifies the channel to enable
 */
void Mcf51cnAdcEnableChannel(TADCChannel channel)
{
	uint8_t portGpio = 0;
	uint8_t pinGpio = 0;
	switch ((uint32_t) channel) {
	case ADC0:
		portGpio = PORTE; pinGpio = 2; break;
	case ADC1:
		portGpio = PORTE; pinGpio = 1; break;
	case ADC2:
		portGpio = PORTE; pinGpio = 0; break;
	case ADC3:
		portGpio = PORTD; pinGpio = 7; break;
	case ADC4:
		portGpio = PORTD; pinGpio = 3; break;
	case ADC5:
		portGpio = PORTD; pinGpio = 2; break;
	case ADC6:
		portGpio = PORTD; pinGpio = 1; break;
	case ADC7:
		portGpio = PORTD; pinGpio = 0; break;
	case ADC8:
		portGpio = PORTC; pinGpio = 7; break;
	case ADC9:
		portGpio = PORTC; pinGpio = 6; break;
	case ADC10:
		portGpio = PORTC; pinGpio = 5; break;
	case ADC11:
		portGpio = PORTC; pinGpio = 4; break;
	}
	GpioPinConfigSet(portGpio, pinGpio, GPIO_CFG_ALT3);
	adcEnableChannels |= 1 << channel;
}

/*!
 * \brief Disable a channel.
 *
 * \param channel  Specifies the channel to disable
 */
void Mcf51cnAdcDisableChannel(TADCChannel channel)
{
	adcEnableChannels &= ~(1 << channel);
}

/*!
 * \brief Set the prescaler for the adc
 *
 * \param prescale  Prescaler value 0-8
 */
void Mcf51cnAdcSetPrescale(uint32_t prescale)
{
	uint8_t cfg_reg;

    if (prescale >= 8) prescale = 7;

    cfg_reg = 0 | MCF_ADC_CFG_MODE_12BIT | MCF_ADC_CFG_ADICLK_ASYNC;

    /*
     * ADIV  prescale Ratio Clock Rate
     * 00 	 1        Input clock
     * 01 	 2        Input clock ÷ 2
     * 10    4        Input clock ÷ 4
     * 11    8        Input clock ÷ 8
     */
    cfg_reg |= (prescale / 2)  << MCF_ADC_CFG_ADIV_BITNUM;

    MCF_ADC_CFG = cfg_reg;
}

/*!
 * \brief do conversion
 */
void Mcf51cnAdcStartConversion(void)
{
	uint32_t detectChannel = adcEnableChannels;
	uint16_t adcReadChannel = 0;

	while(detectChannel != 0){
		if(detectChannel & 0x00000001){

			MCF_ADC_SC1 = MCF_ADC_SC1_AIEN | (adcReadChannel & MCF_ADC_SC1_ADCH);
			if(NutEventWait(&readHandle, 1000)){
				return;
			}
		}
		detectChannel >>= 1;
		adcReadChannel ++;
	}
}

/*
 * ADC interrupt handler.
 */
static void Mcf51cnAdcInterrupt(void *arg)
{
	/* MCF_ADC_SC1_COCO bit is cleared when MCF_ADC_R register is read. */
	uint16_t ADC_Value = MCF_ADC_R;
	uint16_t adcReadChannel = MCF_ADC_SC1 & MCF_ADC_SC1_ADCH;

    Mcf51cnAdcBufWrite(adcReadChannel, ADC_Value);

    NutEventPostFromIrq(&readHandle);
}

/*!
 * \brief Initialize the adc to the configured default values and enable interrupt
 */
void Mcf51cnAdcInit(void)
{
    int channel;

    /* Only init once */
    if (ADC_Buffer) return;

    Mcf51cnAdcSetMode(ADC_OFF);

    /* Basic configuration: Disable all channels and set mode and prescaler */
    Mcf51cnAdcSetPrescale(8);
    adcEnableChannels = 0;

    Mcf51cnAdcSetMode(MCF51_ADC_INITIAL_MODE);

    /* Init adc buffers. One for every channel as we can sample all by automatic sequence */
    ADC_Buffer = NutHeapAlloc(sizeof(uint16_t *) * ADC_MAX_CHANNEL);
    for (channel = 0; channel < ADC_MAX_CHANNEL; channel ++) {
        ADC_Buffer[channel] = NutHeapAlloc(sizeof(uint16_t) * MCF51_ADC_BUF_SIZE + 2);
        ADC_Buffer[channel][_adc_buf_head] = 0;
        ADC_Buffer[channel][_adc_buf_tail] = 0;
    }

    //NutEventPost(&readHandle);

    if (NutRegisterIrqHandler(&sig_ADC, Mcf51cnAdcInterrupt, NULL)) {
        // We do not free buffer as this would cost ROM and is not likely
        return;
    }
    NutIrqEnable(&sig_ADC);
}
