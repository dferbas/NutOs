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

#ifndef _DEV_GPIO_H_
#error "Do not include this file directly. Use dev/gpio.h instead!"
#endif

#include <cfg/sci.h>

/*
 * Default Peripheral Configuration
 */
#ifndef SCI1_TXD_PORTPIN
#define SCI1_TXD_PORTPIN       PTD0
#endif

#ifndef SCI1_RXD_PORTPIN
#define SCI1_RXD_PORTPIN       PTD1
#endif

/*
 * Peripheral GPIO Configuration
 */
#if SCI1_TXD_PORTPIN == PTD0
#define SCI1_TXD_PORT            PORTD
#define SCI1_TXD_PIN             0
#define SCI1_TXD_PERIPHERAL      GPIO_CFG_ALT2
#else
#warning "Illegal SCI1 TXD pin assignement"
#endif

#if SCI1_RXD_PORTPIN == PTD1
#define SCI1_RXD_PORT            PORTD
#define SCI1_RXD_PIN             1
#define SCI1_RXD_PERIPHERAL      GPIO_CFG_ALT2
#else
#warning "Illegal SCI1 RXD pin assignement"
#endif
