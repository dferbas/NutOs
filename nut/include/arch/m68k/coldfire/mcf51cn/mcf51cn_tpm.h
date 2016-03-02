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

/*** TPMSC - TPM Status and Control Register; 0xFFFF8260 ***/
#define MCF_TPM_SC(x) 					   	(*(volatile uint8_t*)(0xFFFF8260 + ((x - 1) * 0x20)))
#define MCF_TPM_SC_CPWMS                    0x20U
#define MCF_TPM_SC_TOIE                     0x40U
#define MCF_TPM_SC_TOF                      0x80U
#define MCF_TPM_SC_PS(x)					(((x)&0x7)<<0)
#define MCF_TPM_SC_PS_MASK                  0x07U
#define MCF_TPM_SC_CLKSx(x)					(((x)&0x3)<<3)
#define MCF_TPM_SC_CLKSx_MASK               0x18U


/*** TPMCNT - TPM Timer Counter Register; 0xFFFF8261 ***/
#define MCF_TPM_CNT(x)						(*(volatile uint16_t*)(0xFFFF8261 + ((x - 1) * 0x20)))


/*** TPMMOD - TPM Timer Counter Modulo Register; 0xFFFF8263 ***/
#define MCF_TPM_MOD(x)						(*(volatile uint16_t*)(0xFFFF8263 + ((x - 1) * 0x20)))


/*** TPMC1SC - TPM Timer Channel n Status and Control Register; 0xFFFF8268 ***/
#define MCF_TPM_CSC(x, n)					(*(volatile uint8_t*)(0xFFFF8265 + ((x - 1) * 0x20) + ((n) * 0x3)))
#define MCF_TPM_CSC_CHnIE                  	0x40U
#define MCF_TPM_CSC_CHnF                   	0x80U
#define MCF_TPM_CSC_ELSnx(x)				(((x)&0x3)<<2)
#define MCF_TPM_CSC_ELSnx_MASK             	0x0CU
#define MCF_TPM_CSC_MSnx(x)					(((x)&0x3)<<4)
#define MCF_TPM_CSC_MSnx_MASK              	0x30U


/*** TPMCV - TPM Timer Channel n Value Register; 0xFFFF8269 ***/
#define MCF_TPM_CV(x, n)					(*(volatile uint16_t*)(0xFFFF8266 + ((x - 1) * 0x20) + ((n) * 0x3)))
