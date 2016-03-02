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

#ifndef _ARCH_M68K_H_
#error "Do not include this file directly. Use arch/m68k.h instead!"
#endif

/* MTIM Clock Configuration Register */
#define MCF_MTIM_SC(x) 						(*(volatile uint8_t *)(0xFFFF82A0 + ((x - 1) * 0x60)))
#define MCF_MTIM_SC_TSTP                    0x10
#define MCF_MTIM_SC_TRST                    0x20
#define MCF_MTIM_SC_TOIE                    0x40
#define MCF_MTIM_SC_TOF                     0x80


/* MTIM Clock Configuration Register */
#define MCF_MTIM_CLK(x)						(*(volatile uint8_t *)(0xFFFF82A1 + ((x - 1) * 0x60)))
#define MCF_MTIM_CLK_PS0                    0x01
#define MCF_MTIM_CLK_PS1                    0x02
#define MCF_MTIM_CLK_PS2                    0x04
#define MCF_MTIM_CLK_PS3                    0x08
#define MCF_MTIM_CLK_CLKS0                  0x10
#define MCF_MTIM_CLK_CLKS1                  0x20
#define MCF_MTIM_CLK_PS                     0x0F
#define MCF_MTIM_CLK_PS_BITNUM              0x00
#define MCF_MTIM_CLK_CLKS                   0x30
#define MCF_MTIM_CLK_CLKS_BITNUM            0x04


/* MTIM Counter Register */
#define MCF_MTIM_CNT(x)						(*(volatile uint8_t *)(0xFFFF82A2 + ((x - 1) * 0x60)))

/* MTIM Modulo Register */
#define MCF_MTIM_MOD(x)						(*(volatile uint8_t *)(0xFFFF82A3 + ((x - 1) * 0x60)))
