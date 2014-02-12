/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
 * Copyright (C) 2012 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/arch.h>

#if defined(MCU_STM32F1)||defined(MCU_STM32L1)
#include <arch/cm3/stm/stm32f1_dma.h>
#elif  defined(MCU_STM32F2)||defined(MCU_STM32F4)
#include <arch/cm3/stm/stm32f2_dma.h>
#else
#warning "STM32 family has no implemented DMA"
#endif

/*!
 * \brief STM32 DMA Status and Interrupt Flags.
 */
#define DMA_TEIF 0x8  /*< Channel x Transfer Error Flag */
#define DMA_HTIF 0x4  /*< Channel x Half Transfer Complete Flag */
#define DMA_TCIF 0x2  /*< Channel x Transfer Complete Flag */
#define DMA_GIF  0x1  /*< Channel x Global Interrupt Flag */

#define DMA_FLAGMASK 0xF
#define DMA_IRQMASK  0x1

#define DMA_CONTROL0 0x00
#define DMA_CONTROL1 0x80
#define DMA_STREAM0  0x00
#define DMA_STREAM1  0x10
#define DMA_STREAM2  0x20
#define DMA_STREAM3  0x30
#define DMA_STREAM4  0x40
#define DMA_STREAM5  0x50
#define DMA_STREAM6  0x60
#define DMA_STREAM7  0x70
#define DMA_CHANNEL0 0x00
#define DMA_CHANNEL1 0x01
#define DMA_CHANNEL2 0x02
#define DMA_CHANNEL3 0x03
#define DMA_CHANNEL4 0x04
#define DMA_CHANNEL5 0x05
#define DMA_CHANNEL6 0x06
#define DMA_CHANNEL7 0x07

/*
 * DMA Handles and Interrupt Entry Points
 */
extern void Dma1IrqEntry(void *arg);
#ifdef STM_HAS_DMA2
extern void Dma2IrqEntry(void *arg);
#endif

/*
 * DMA Control Functions
 */
void DMA_Setup( uint8_t ch, void* dst, void* src, uint16_t length, uint32_t flags);
void DMA_Enable(uint8_t ch);
void DMA_Disable(uint8_t ch);
void DMA_Init(void);
void DMA_IrqMask( uint8_t ch, uint32_t mask, uint8_t ena);
void DMA_ClearFlag( uint8_t ch, uint32_t flags);
uint32_t DMA_GetFlag( uint8_t ch);

