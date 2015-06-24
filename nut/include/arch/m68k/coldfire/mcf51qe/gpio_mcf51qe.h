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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTEON) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

#ifndef _DEV_GPIO_H_
#error "Do not include this file directly. Use dev/gpio.h instead!"
#endif

#include <stdint.h>

/*
 * GPIO Port Definitions
 */
#define PORTA       0
#define PORTB       1
#define PORTC       2
#define PORTD       3
#define PORTE       4
#define PORTF       5
#define PORTG       6 // For Data and Data Direction register address offset is 14, set in mcf51qe_gpio.h
#define PORTH       7 // For Data and Data Direction register address offset is 15, set in mcf51qe_gpio.h
#define PORTJ       8 // For Data and Data Direction register address offset is 23, set in mcf51qe_gpio.h

/*
 * GPIO PortPin Definitions
 */

/*
 * GPIO PortPins Initialization
 */

/*
 * GPIO API
 */

//default GPIO, enable periphery in her driver
#define GPIO_CFG_PERIPHERAL_MASK    0x00000003 // not supported
#define GPIO_CFG_ALT1               0x00000001 // not supported
#define GPIO_CFG_ALT2               0x00000002 // not supported
#define GPIO_CFG_ALT3               0x00000003 // not supported

#define GPIO_CFG_INPUT              0x00000000
#define GPIO_CFG_OUTPUT             0x00000004
#define GPIO_CFG_PULLUP             0x00000010
#define GPIO_CFG_SLEW_RATE          0x00000020
#define GPIO_CFG_DRIVE_STRENGTH     0x00000040
#define GPIO_CFG_INPUT_FILTER       0x00000080	// not supported
#define GPIO_CFG_DEBOUNCE           0   		// not supported


#define GpioPinGet(bank, bit)           ((MCF_GPIO_D(bank) >> bit) & 0x1)
#define GpioPinSet(bank, bit, value)    ((value) ? (GpioPinSetHigh(bank, bit)) : (GpioPinSetLow(bank, bit)))
#define GpioPinSetHigh(bank, bit)       MCF_GPIO_D(bank) |= _BV(bit)
#define GpioPinSetLow(bank, bit)        MCF_GPIO_D(bank) &= ~_BV(bit)

#define GpioPortGet(bank)               MCF_GPIO_D(bank)
#define GpioPortSet(bank, value)        MCF_GPIO_D(bank) = (value)
#define GpioPortSetHigh(bank, mask)     MCF_GPIO_D(bank) |= (mask)
#define GpioPortSetLow(bank, mask)      MCF_GPIO_D(bank) &= ~(mask)

extern uint32_t GpioPinConfigGet(int bank, int bit);
extern int GpioPinConfigSet(int bank, int bit, uint32_t flags);
extern int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags);
