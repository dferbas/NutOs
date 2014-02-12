/*!
 * Copyright (C) 2001-2010 by egnite Software GmbH
 *
 * All rights reserved.
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
 *
 * Portions Copyright Atmel Corporation, see the following note.
 */


#ifndef _AVR32_GPIO_H_
#define _AVR32_GPIO_H_

#include <arch/avr32.h>
#include <avr32/io.h>

#include <dev/gpio.h>

typedef struct
{
  unsigned char pin;              //!< Module pin.
  unsigned char function;         //!< Module function.
} gpio_map_t[];

extern int gpio_enable_module(const gpio_map_t gpiomap, unsigned int size);

void inline gpio_enable_module_pin(unsigned int pin, unsigned int function)
{
    unsigned int peripheral = 0;
    switch ( function )
    {
    case 0:     peripheral = GPIO_CFG_PERIPHERAL0;      break;
    case 1:     peripheral = GPIO_CFG_PERIPHERAL1;      break;
    case 2:     peripheral = GPIO_CFG_PERIPHERAL2;      break;
    default:
        while(1); // Unrecognized peripheral choice
        break;
    }
    GpioPinConfigSet( pin >> 5, pin & 0x1F, peripheral );
    //enable_module_pin(pin >> 5, _BV(pin & 0x1F), peripheral);
}


#endif  // _AVR32_GPIO_H_
