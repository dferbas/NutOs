/*
 * Copyright 2014-2016 by Embedded Technologies s.r.o. All rights reserved.
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

#ifndef ADC_MCF51CN_H_
#define ADC_MCF51CN_H_

#include <stdint.h>

/*!
 * \enum  adc_moded_type dev/at91_adc.h
 * \brief enum declaring possible ADC modes
 *
 * ADC_OFF:
 *    Switch off adc and enable slep mode
 * FREE_RUNNING_x:
 *    This is a pseudo free running mode as we don't want to use a
 *    dedicated timer (0..3) for this purpose we retrigger sampling in
 *    the interrupt handler. Starting when ADC_start_conversion() is called.
 * SINGLE_CONVERSION:
 *    Single-conversion mode. One sample taken every time
 *    ADC_start_conversion() is called.
 *
 */

typedef enum adc_mode_type
{
    ADC_OFF,
    SINGLE_CONVERSION
} TADCMode;


/*!
 * \enum  adc_channel_type dev/at91_adc.h
 * \brief enum declaring possible ADC channels
 */

typedef enum adc_channel_type
{
	/*  original  */
/*  ADC0=0,
    ADC1=1,
    ADC2=2,
    ADC3=3,
    ADC4=4,
    ADC5=5,
    ADC6=6,
    ADC7=7,
    ADC8=8,
    ADC9=9,
    ADC10=10,
    ADC11=11,
    ADC_MAX_CHANNEL=12*/


    ADC0=0,
    ADC1=1,
    ADC2=2,
    ADC3=3,
    ADC4=4,
    ADC5=5,
    ADC6=6,
    ADC7=7,
    ADC8=8,
    ADC9=9,
    ADC10=10,
    ADC11=11,
    ADCVrefH=29,
    ADCVrefL=30,
    ADC_MAX_CHANNEL=31
} TADCChannel;

/* Function prototypes */

void Mcf51cnAdcInit(void);


// AdcStartConversion
//
// Begins ADC conversion. The conversion will process all
// enabled channels one after the other.
//
// NOTE: Converted values from the ADC are stored
//       in a local buffer. The user must call
//       ADC_read to obtain these values.
//
// pre:  none
// post: The ADC has started conversion. Completion of
//       any conversions is not guaranteed.

void Mcf51cnAdcConvertChannels(void);


// AdcSetPrescale
//
// Allows setting of ADC clock prescalar (ADC rate).
// The  ADC rate is given by the system clock rate
// divided by the prescalar value. Possible prescalar
// values range from 2-128
//
// pre: "prescalar" is a valid ADC reference from the
//       choices given above
// post: ADC prescalar set to desired choice

void Mcf51cnAdcSetPrescale(uint32_t prescale);


// AdcDisableChannel
// AdcEnableChannel
//
// Enables/disables a channel to be sampled on the next conversion
//
// pre: none
// post: Channel is selected / deselected. Next conversion will respect these settings

void Mcf51cnAdcDisableChannel(TADCChannel channel);
void Mcf51cnAdcEnableChannel(TADCChannel channel);


// AdcSetMode
//
// Possible values:
//    - ADC_OFF
//    - SINGLE_CONVERSION
//    - FREE_RUNNING_T0
//    - FREE_RUNNING_T1
//    - FREE_RUNNING_T2
//      These depend on a timer t0 / t1 / t2
//    - FREE_RUNNING_EXT
//      External trigger
//
// pre: none
// post: Set adc conversion to the selected value.

void Mcf51cnAdcSetMode(TADCMode mode);

// AdcBufRead
//
// Reads the next sampled value of the given channel from the buffer.
//
// pre: Sample completed
// post: Value will be removed from buffer

int Mcf51cnAdcBufRead(uint16_t channel, uint16_t * read);

#endif  /* ADC_MCF51CN_H_ */
