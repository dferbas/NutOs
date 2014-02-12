#ifndef _DEV_IRQREG_ARCH_CM3_STM32_H_
#define _DEV_IRQREG_ARCH_CM3_STM32_H_

/*
 * Copyright (C) 2001-2007 by egnite Software GmbH. All rights reserved.
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
 * \verbatim
 * $Id: $
 * \endverbatim
 */

extern IRQ_HANDLER sig_INTERRUPT0;      // EXTI 0
extern IRQ_HANDLER sig_INTERRUPT1;      // EXTI 1
extern IRQ_HANDLER sig_INTERRUPT2;      // EXTI 2
extern IRQ_HANDLER sig_INTERRUPT3;      // EXTI 3
extern IRQ_HANDLER sig_INTERRUPT4;      // EXTI 4
extern IRQ_HANDLER sig_INTERRUPT9_5;    // EXTI 9_5
extern IRQ_HANDLER sig_INTERRUPT15_10;  // EXTI 15_10
extern IRQ_HANDLER sig_RTC;         // Real Time Clock
extern IRQ_HANDLER sig_SPI1;        // SPI 1 Controller
extern IRQ_HANDLER sig_SPI2;        // SPI 2 Controller
extern IRQ_HANDLER sig_TWI1_EV;     // I2C 1 Data/Event
extern IRQ_HANDLER sig_TWI2_EV;     // I2C 2 Data/Event
extern IRQ_HANDLER sig_TWI1_ER;     // I2C 1 Error
extern IRQ_HANDLER sig_TWI2_ER;     // I2C 2 Error
extern IRQ_HANDLER sig_CAN1_TX;     // CAN 1 TX
extern IRQ_HANDLER sig_CAN1_RX0;    // CAN 1 RX0
extern IRQ_HANDLER sig_CAN1_RX1;    // CAN 1 RX1
extern IRQ_HANDLER sig_CAN1_SCE;    // CAN 1 SCE
extern IRQ_HANDLER sig_CAN2_TX;     // CAN 2 TX
extern IRQ_HANDLER sig_CAN2_RX0;    // CAN 2 RX0
extern IRQ_HANDLER sig_CAN2_RX1;    // CAN 2 RX1
extern IRQ_HANDLER sig_CAN2_SCE;    // CAN 2 SCE
extern IRQ_HANDLER sig_USART1;      // USART 1
extern IRQ_HANDLER sig_USART2;      // USART 2
extern IRQ_HANDLER sig_USART3;      // USART 3
extern IRQ_HANDLER sig_UART4;       // UART 4
extern IRQ_HANDLER sig_UART5;       // UART 5
extern IRQ_HANDLER sig_USART6;      // USART 6
extern IRQ_HANDLER sig_OTG_FS;      // USB
extern IRQ_HANDLER sig_DMA1_CH1;    // DMA Controller 1 Channel 1
extern IRQ_HANDLER sig_DMA1_CH2;    // DMA Controller 1 Channel 2
extern IRQ_HANDLER sig_DMA1_CH3;    // DMA Controller 1 Channel 3
extern IRQ_HANDLER sig_DMA1_CH4;    // DMA Controller 1 Channel 4
extern IRQ_HANDLER sig_DMA1_CH5;    // DMA Controller 1 Channel 5
extern IRQ_HANDLER sig_DMA1_CH6;    // DMA Controller 1 Channel 6
extern IRQ_HANDLER sig_DMA1_CH7;    // DMA Controller 1 Channel 7
extern IRQ_HANDLER sig_DMA2_CH1;    // DMA Controller 2 Channel 1
extern IRQ_HANDLER sig_DMA2_CH2;    // DMA Controller 2 Channel 2
extern IRQ_HANDLER sig_DMA2_CH3;    // DMA Controller 2 Channel 3
extern IRQ_HANDLER sig_DMA2_CH4;    // DMA Controller 2 Channel 4
extern IRQ_HANDLER sig_DMA2_CH5;    // DMA Controller 2 Channel 5
extern IRQ_HANDLER sig_EMAC;        // Ethernet global interrupt

#endif
