#ifndef _CFG_ARCH_GPIO_H_
#define _CFG_ARCH_GPIO_H_

/*
 * Copyright (C) 2006 by egnite Software GmbH. All rights reserved.
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
 */

/*
 * $Log$
 * Revision 1.1  2006/04/07 15:31:19  haraldkipp
 * First check in
 *
 */

/*!
 * \file cfg/arch/gpio.h
 * \brief Port configuration.
 */

#if defined(__AVR__)
#include <cfg/arch/avr.h>
#elif defined(__arm__)
#include <cfg/arch/armpio.h>
#elif defined(__AVR32__)
#include <cfg/arch/avr32.h>
#endif

/*
   Alternate pin sets for driver configuration of the
   peripheral GPIO.
*/
#define ALTERNATE_PIN_SET1 1
#define ALTERNATE_PIN_SET2 2
#define ALTERNATE_PIN_SET3 3
#define ALTERNATE_PIN_SET4 4
#define ALTERNATE_PIN_SET5 5
#define ALTERNATE_PIN_SET6 6
#define ALTERNATE_PIN_SET7 7

#endif

