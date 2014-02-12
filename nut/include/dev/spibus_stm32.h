#ifndef _DEV_SPIBUS_STM32_H_
#define _DEV_SPIBUS_STM32_H_
/*
 * Copyright (C) 2008-2009 by egnite GmbH
 * Copyright (C) 2012 Uwe Bonnes
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
 */

/*!
 * \file dev/spibus_stm32.h
 * \brief STM32 specific SPI bus declarations.
 *
 * \verbatim
 * $Id:
 * \endverbatim
 */

#include <dev/spibus.h>

extern NUTSPIBUS spiBus0Stm32;
extern NUTSPIBUS spiBus1Stm32;

extern int Stm32SpiBus1Select(NUTSPINODE * node, uint32_t tmo);
extern int Stm32SpiBus1Deselect(NUTSPINODE * node);

extern int Stm32SpiBus2Select(NUTSPINODE * node, uint32_t tmo);
extern int Stm32SpiBus2Deselect(NUTSPINODE * node);

extern int Stm32SpiSetup(NUTSPINODE * node);
extern int Stm32SpiBusNodeInit(NUTSPINODE * node);
extern int Stm32SpiBusTransfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen);
extern int Stm32SpiBusDblBufTransfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen);
extern int Stm32SpiBusPollTransfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen);
extern int Stm32SpiBusWait(NUTSPINODE * node, uint32_t tmo);

#endif
