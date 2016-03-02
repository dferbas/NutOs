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

/* SCI - Serial Communication Interface */

/* Baud Rate Register */
#define MCF_SCI_BD(x) 						(*(volatile uint16_t *)(0xFFFF8160 + ((x - 1) * 0x20)))
#define MCF_SCI_BD_RXEDGIE                  0x4000
#define MCF_SCI_BD_LBKDIE                   0x8000
#define MCF_SCI_BD_SBR                      0x1FFF
#define MCF_SCI_BD_SBR_BITNUM               0x00


/* Control Register 1 */
#define MCF_SCI_C1(x) 						(*(volatile uint8_t *)(0xFFFF8162 + ((x - 1) * 0x20)))
#define MCF_SCI_C1_PT                       0x01
#define MCF_SCI_C1_PE                       0x02
#define MCF_SCI_C1_ILT                      0x04
#define MCF_SCI_C1_WAKE                     0x08
#define MCF_SCI_C1_M                        0x10
#define MCF_SCI_C1_RSRC                     0x20
#define MCF_SCI_C1_SCISWAI                  0x40
#define MCF_SCI_C1_LOOPS                    0x80


/* Control Register 2 */
#define MCF_SCI_C2(x) 						(*(volatile uint8_t *)(0xFFFF8163 + ((x - 1) * 0x20)))
#define MCF_SCI_C2_SBK                      0x01
#define MCF_SCI_C2_RWU                      0x02
#define MCF_SCI_C2_RE                       0x04
#define MCF_SCI_C2_TE                       0x08
#define MCF_SCI_C2_ILIE                     0x10
#define MCF_SCI_C2_RIE                      0x20
#define MCF_SCI_C2_TCIE                     0x40
#define MCF_SCI_C2_TIE                      0x80


/* Status Register 1 */
#define MCF_SCI_S1(x) 						(*(volatile uint8_t *)(0xFFFF8164 + ((x - 1) * 0x20)))
#define MCF_SCI_S1_PF                       0x01
#define MCF_SCI_S1_FE                       0x02
#define MCF_SCI_S1_NF                       0x04
#define MCF_SCI_S1_OR                       0x08
#define MCF_SCI_S1_IDLE                     0x10
#define MCF_SCI_S1_RDRF                     0x20
#define MCF_SCI_S1_TC                       0x40
#define MCF_SCI_S1_TDRE                     0x80


/* Status Register 2 */
#define MCF_SCI_S2(x) 						(*(volatile uint8_t *)(0xFFFF8165 + ((x - 1) * 0x20)))
#define MCF_SCI_S2_RAF                      0x01
#define MCF_SCI_S2_LBKDE                    0x02
#define MCF_SCI_S2_BRK13                    0x04
#define MCF_SCI_S2_RWUID                    0x08
#define MCF_SCI_S2_RXINV                    0x10
#define MCF_SCI_S2_RXEDGIF                  0x40
#define MCF_SCI_S2_LBKDIF                   0x80


/* Control Register 3 */
#define MCF_SCI_C3(x) 						(*(volatile uint8_t *)(0xFFFF8166 + ((x - 1) * 0x20)))
#define MCF_SCI_C3_PEIE                     0x01
#define MCF_SCI_C3_FEIE                     0x02
#define MCF_SCI_C3_NEIE                     0x04
#define MCF_SCI_C3_ORIE                     0x08
#define MCF_SCI_C3_TXINV                    0x10
#define MCF_SCI_C3_TXDIR                    0x20
#define MCF_SCI_C3_T8                       0x40
#define MCF_SCI_C3_R8                       0x80


/* Data Register */
#define MCF_SCI_D(x) 						(*(volatile uint8_t *)(0xFFFF8167 + ((x - 1) * 0x20)))
