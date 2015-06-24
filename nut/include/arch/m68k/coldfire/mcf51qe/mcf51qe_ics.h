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


/*** ICSC1 - ICS Control Register 1)) 0xFFFF8038 ***/
#define MCF_ICS_C1 (*(volatile uint8_t *)(0xFFFF8038))

#define MCF_ICS_C1_IREFSTEN                  0x01
#define MCF_ICS_C1_IRCLKEN                   0x02
#define MCF_ICS_C1_IREFS                     0x04
#define MCF_ICS_C1_RDIV0                     0x08
#define MCF_ICS_C1_RDIV1                     0x10
#define MCF_ICS_C1_RDIV2                     0x20
#define MCF_ICS_C1_CLKS0                     0x40
#define MCF_ICS_C1_CLKS1                     0x80
#define MCF_ICS_C1_RDIV                      0x38
#define MCF_ICS_C1_RDIV_BITNUM               0x03
#define MCF_ICS_C1_CLKS                      0xC0
#define MCF_ICS_C1_CLKS_BITNUM               0x06


/*** ICSC2 - ICS Control Register 2)) 0xFFFF8039 ***/
#define MCF_ICS_C2 (*(volatile uint8_t *)(0xFFFF8039))

#define MCF_ICS_C2_EREFSTEN                  0x01
#define MCF_ICS_C2_ERCLKEN                   0x02
#define MCF_ICS_C2_EREFS                     0x04
#define MCF_ICS_C2_LP                        0x08
#define MCF_ICS_C2_HGO                       0x10
#define MCF_ICS_C2_RANGE                     0x20
#define MCF_ICS_C2_BDIV0                     0x40
#define MCF_ICS_C2_BDIV1                     0x80
#define MCF_ICS_C2_BDIV                      0xC0
#define MCF_ICS_C2_BDIV_BITNUM               0x06

/*** ICM - ICS Trim Register)) 0xFFFF803A ***/
#define MCF_ICS_ICM (*(volatile uint8_t *)(0xFFFF803A))

/*** ICSSC - ICS Status and Control Register)) 0xFFFF803B ***/
#define MCF_ICS_SC (*(volatile uint8_t *)(0xFFFF803B))

#define MCF_ICS_SC_FTRIM                     0x01
#define MCF_ICS_SC_OSCINIT                   0x02
#define MCF_ICS_SC_CLKST0                    0x04
#define MCF_ICS_SC_CLKST1                    0x08
#define MCF_ICS_SC_IREFST                    0x10
#define MCF_ICS_SC_DMX32                     0x20
#define MCF_ICS_SC_DRST_DRS0                 0x40
#define MCF_ICS_SC_DRST_DRS1                 0x80
#define MCF_ICS_SC_CLKST                     0x0C
#define MCF_ICS_SC_CLKST_BITNUM              0x02
#define MCF_ICS_SC_DRST_DRS                  0xC0
#define MCF_ICS_SC_DRST_DRS_BITNUM           0x06

void Mcf51qeIcsInitClock(void);
