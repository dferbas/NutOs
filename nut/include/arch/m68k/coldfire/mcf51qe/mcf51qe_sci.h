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

/* SCI_BD - SCI Baud Rate Register (SCI1 - 0xFFFF8020, SCI2 - 0xFFFF9870) */
#define MCF_SCI_BD(x) 						(*(volatile uint16_t *)(0xFFFF8020 + ((x - 1) * 0x1850)))

#define MCF_SCI_BD_SBR0                     0x01
#define MCF_SCI_BD_SBR1                     0x02
#define MCF_SCI_BD_SBR2                     0x04
#define MCF_SCI_BD_SBR3                     0x08
#define MCF_SCI_BD_SBR4                     0x10
#define MCF_SCI_BD_SBR5                     0x20
#define MCF_SCI_BD_SBR6                     0x40
#define MCF_SCI_BD_SBR7                     0x80
#define MCF_SCI_BD_SBR8                     0x0100
#define MCF_SCI_BD_SBR9                     0x0200
#define MCF_SCI_BD_SBR10                    0x0400
#define MCF_SCI_BD_SBR11                    0x0800
#define MCF_SCI_BD_SBR12                    0x1000
#define MCF_SCI_BD_RXEDGIE                  0x4000
#define MCF_SCI_BD_LBKDIE                   0x8000
#define MCF_SCI_BD_SBR                      0x1FFF
#define MCF_SCI_BD_SBR_BITNUM               0x00


/* SCI_C1 - SCI Control Register 1 (SCI1 - 0xFFFF8022, SCI2 - 0xFFFF9872) */
#define MCF_SCI_C1(x) 						(*(volatile uint8_t *)(0xFFFF8022 + ((x - 1) * 0x1850)))

#define MCF_SCI_C1_PT                       0x01
#define MCF_SCI_C1_PE                       0x02
#define MCF_SCI_C1_ILT                      0x04
#define MCF_SCI_C1_WAKE                     0x08
#define MCF_SCI_C1_M                        0x10
#define MCF_SCI_C1_RSRC                     0x20
#define MCF_SCI_C1_SCISWAI                  0x40
#define MCF_SCI_C1_LOOPS                    0x80


/* SCI_C2 - SCI Control Register 2 (SCI1 - 0xFFFF8023, SCI2 - 0xFFFF9873) */
#define MCF_SCI_C2(x) 						(*(volatile uint8_t *)(0xFFFF8023 + ((x - 1) * 0x1850)))

#define MCF_SCI_C2_SBK                      0x01
#define MCF_SCI_C2_RWU                      0x02
#define MCF_SCI_C2_RE                       0x04
#define MCF_SCI_C2_TE                       0x08
#define MCF_SCI_C2_ILIE                     0x10
#define MCF_SCI_C2_RIE                      0x20
#define MCF_SCI_C2_TCIE                     0x40
#define MCF_SCI_C2_TIE                      0x80


/* SCI_S1 - SCI Status Register 1 (SCI1 - 0xFFFF8024, SCI2 - 0xFFFF9874) */
#define MCF_SCI_S1(x) 						(*(volatile uint8_t *)(0xFFFF8024 + ((x - 1) * 0x1850)))

#define MCF_SCI_S1_PF                       0x01
#define MCF_SCI_S1_FE                       0x02
#define MCF_SCI_S1_NF                       0x04
#define MCF_SCI_S1_OR                       0x08
#define MCF_SCI_S1_IDLE                     0x10
#define MCF_SCI_S1_RDRF                     0x20
#define MCF_SCI_S1_TC                       0x40
#define MCF_SCI_S1_TDRE                     0x80


/* SCI_S2 - SCI Status Register 2 (SCI1 - 0xFFFF8025, SCI2 - 0xFFFF9875) */
#define MCF_SCI_S2(x) 						(*(volatile uint8_t *)(0xFFFF8025 + ((x - 1) * 0x1850)))

#define MCF_SCI_S2_RAF                      0x01
#define MCF_SCI_S2_LBKDE                    0x02
#define MCF_SCI_S2_BRK13                    0x04
#define MCF_SCI_S2_RWUID                    0x08
#define MCF_SCI_S2_RXINV                    0x10
#define MCF_SCI_S2_RXEDGIF                  0x40
#define MCF_SCI_S2_LBKDIF                   0x80


/* SCI_C3 - SCI Control Register 3 (SCI1 - 0xFFFF8026, SCI2 - 0xFFFF9876) */
#define MCF_SCI_C3(x) 						(*(volatile uint8_t *)(0xFFFF8026 + ((x - 1) * 0x1850)))

#define MCF_SCI_C3_PEIE                     0x01
#define MCF_SCI_C3_FEIE                     0x02
#define MCF_SCI_C3_NEIE                     0x04
#define MCF_SCI_C3_ORIE                     0x08
#define MCF_SCI_C3_TXINV                    0x10
#define MCF_SCI_C3_TXDIR                    0x20
#define MCF_SCI_C3_T8                       0x40
#define MCF_SCI_C3_R8                       0x80


/* SCI_D - SCI Data Register (SCI1 - 0xFFFF8027, SCI2 - 0xFFFF9877) */
#define MCF_SCI_D(x) 						(*(volatile uint8_t *)(0xFFFF8027 + ((x - 1) * 0x1850)))

