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

#ifndef _ARCH_M68K_H_
#error "Do not include this file directly. Use arch/m68k.h instead!"
#endif

/* SPI(x) Control Register 1 */
#define MCF_SPI_C1(x) 						(*(volatile uint8_t *)(0xFFFF81C0 + ((x - 1) * 0x20)))
#define MCF_SPI_C1_LSBFE                    0x01
#define MCF_SPI_C1_SSOE                     0x02
#define MCF_SPI_C1_CPHA                     0x04
#define MCF_SPI_C1_CPOL                     0x08
#define MCF_SPI_C1_MSTR                     0x10
#define MCF_SPI_C1_SPTIE                    0x20
#define MCF_SPI_C1_SPE                      0x40
#define MCF_SPI_C1_SPIE                     0x80


/* SPI(x) Control Register 2 */
#define MCF_SPI_C2(x) 						(*(volatile uint8_t *)(0xFFFF81C1 + ((x - 1) * 0x20)))
#define MCF_SPI_C2_SPC0                     0x01
#define MCF_SPI_C2_SPISWAI                  0x02
#define MCF_SPI_C2_BIDIROE                  0x08
#define MCF_SPI_C2_MODFEN                   0x10


/* SPI(x) Baud Rate Register */
#define MCF_SPI_BR(x) 						(*(volatile uint8_t *)(0xFFFF81C2 + ((x - 1) * 0x20)))
#define MCF_SPI_BR_SPR0                     0x01
#define MCF_SPI_BR_SPR1                     0x02
#define MCF_SPI_BR_SPR2                     0x04
#define MCF_SPI_BR_SPPR0                    0x10
#define MCF_SPI_BR_SPPR1                    0x20
#define MCF_SPI_BR_SPPR2                    0x40
#define MCF_SPI_BR_SPR                      0x07
#define MCF_SPI_BR_SPR_BITNUM               0x00
#define MCF_SPI_BR_SPPR                     0x70
#define MCF_SPI_BR_SPPR_BITNUM              0x04


/* SPI(x) Status Register */
#define MCF_SPI_S(x) 						(*(volatile uint8_t *)(0xFFFF81C3 + ((x - 1) * 0x20)))
#define MCF_SPI_S_MODF                      0x10
#define MCF_SPI_S_SPTEF                     0x20
#define MCF_SPI_S_SPRF                      0x80


/* SPI(x) Data Register */
#define MCF_SPI_D(x) 						(*(volatile uint8_t *)(0xFFFF81C5 + ((x - 1) * 0x20)))
