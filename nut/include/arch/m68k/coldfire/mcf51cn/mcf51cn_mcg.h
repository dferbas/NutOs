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

/* MCG Control Register 1*/
#define MCF_MCG_C1 							 (*(volatile uint8_t *)(0xFFFF8240))
#define MCF_MCG_C1_IREFSTEN                  0x01
#define MCF_MCG_C1_IRCLKEN                   0x02
#define MCF_MCG_C1_IREFS                     0x04
#define MCF_MCG_C1_RDIV0                     0x08
#define MCF_MCG_C1_RDIV1                     0x10
#define MCF_MCG_C1_RDIV2                     0x20
#define MCF_MCG_C1_CLKS0                     0x40
#define MCF_MCG_C1_CLKS1                     0x80
#define MCF_MCG_C1_RDIV                      0x38
#define MCF_MCG_C1_RDIV_BITNUM               0x03
#define MCF_MCG_C1_CLKS                      0xC0
#define MCF_MCG_C1_CLKS_BITNUM               0x06


/* MCG Control Register 2 */
#define MCF_MCG_C2 							 (*(volatile uint8_t *)(0xFFFF8241))
#define MCF_MCG_C2_EREFSTEN                  0x01
#define MCF_MCG_C2_ERCLKEN                   0x02
#define MCF_MCG_C2_EREFS                     0x04
#define MCF_MCG_C2_LP                        0x08
#define MCF_MCG_C2_HGO                       0x10
#define MCF_MCG_C2_RANGE                     0x20
#define MCF_MCG_C2_BDIV0                     0x40
#define MCF_MCG_C2_BDIV1                     0x80
#define MCF_MCG_C2_BDIV                      0xC0
#define MCF_MCG_C2_BDIV_BITNUM               0x06


/* MCG Trim Register */
#define MCF_MCG_TRM 						 (*(volatile uint8_t *)(0xFFFF8242))
#define MCF_MCG_TRM_TRIM0                    0x01
#define MCF_MCG_TRM_TRIM1                    0x02
#define MCF_MCG_TRM_TRIM2                    0x04
#define MCF_MCG_TRM_TRIM3                    0x08
#define MCF_MCG_TRM_TRIM4                    0x10
#define MCF_MCG_TRM_TRIM5                    0x20
#define MCF_MCG_TRM_TRIM6                    0x40
#define MCF_MCG_TRM_TRIM7                    0x80


/* MCG Status and Control Register */
#define MCF_MCG_SC 							 (*(volatile uint8_t *)(0xFFFF8243))
#define MCF_MCG_SC_FTRIM                     0x01
#define MCF_MCG_SC_OSCINIT                   0x02
#define MCF_MCG_SC_CLKST0                    0x04
#define MCF_MCG_SC_CLKST1                    0x08
#define MCF_MCG_SC_IREFST                    0x10
#define MCF_MCG_SC_PLLST                     0x20
#define MCF_MCG_SC_LOCK                      0x40
#define MCF_MCG_SC_LOLS                      0x80
#define MCF_MCG_SC_CLKST                     0x0C
#define MCF_MCG_SC_CLKST_BITNUM              0x02


/* MCG Control Register 3 */
#define MCF_MCG_C3 							 (*(volatile uint8_t *)(0xFFFF8244))
#define MCF_MCG_C3_VDIV0                     0x01
#define MCF_MCG_C3_VDIV1                     0x02
#define MCF_MCG_C3_VDIV2                     0x04
#define MCF_MCG_C3_VDIV3                     0x08
#define MCF_MCG_C3_DIV32                     0x10
#define MCF_MCG_C3_CME                       0x20
#define MCF_MCG_C3_PLLS                      0x40
#define MCF_MCG_C3_LOLIE                     0x80
#define MCF_MCG_C3_VDIV                      0x0F
#define MCF_MCG_C3_VDIV_BITNUM               0x00


/* MCG Control Register 4 */
#define MCF_MCG_C4 							 (*(volatile uint8_t *)(0xFFFF8245))
#define MCF_MCG_C4_DRST_DRS0                 0x01
#define MCF_MCG_C4_DRST_DRS1                 0x02
#define MCF_MCG_C4_DMX32                     0x20
#define MCF_MCG_C4_DRST_DRS                  0x03
#define MCF_MCG_C4_DRST_DRS_BITNUM           0x00
