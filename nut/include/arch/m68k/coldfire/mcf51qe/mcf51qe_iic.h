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

/*** IIC_ADR - IIC Address Register)) 0xFFFF8030, 0xFFFF9868 ***/
#define MCF_IIC_ADR(x)						(*(volatile uint8_t *)(0xFFFF8030 + (((x) - 1) * 0x1838)))

#define MCF_IIC_ADR_MASK                    0xFE
#define MCF_IIC_ADR_BITNUM					0x01
#define MCF_IIC_ADR_ADR(x)                	(((x) & 0x7F) << 0x1)


/*** IIC_FDR - IIC Frequency Divider Register)) 0xFFFF8031, 0xFFFF9869 ***/
#define MCF_IIC_FDR(x) 						(*(volatile uint8_t *)(0xFFFF8031 + (((x) - 1) * 0x1838)))

#define MCF_IIC_FDR_ICR_MASK                0x3F
#define MCF_IIC_FDR_ICR_BITNUM              0x00
#define MCF_IIC_FDR_ICR(x)                 	(((x) & 0x3F) << 0)
#define MCF_IIC_FDR_MULT_MASK               0xC0
#define MCF_IIC_FDR_MULT_BITNUM             0x06
#define MCF_IIC_FDR_MULT(x)                	(((x) & 0x3) << 0x06)


/*** IIC_CR - IIC Control Register 1)) 0xFFFF8032, 0xFFFF986A ***/
#define MCF_IIC_CR(x) 						(*(volatile uint8_t *)(0xFFFF8032 + (((x) - 1) * 0x1838)))
#define MCF_IIC_CR_RSTA                 	0x04
#define MCF_IIC_CR_TXAK                 	0x08
#define MCF_IIC_CR_TX                   	0x10
#define MCF_IIC_CR_MST                  	0x20
#define MCF_IIC_CR_IICIE               		0x40
#define MCF_IIC_CR_IICEN                	0x80

/*** IIC_SR - IIC Status Register)) 0xFFFF8033, 0xFFFF986B ***/
#define MCF_IIC_SR(x) 						(*(volatile uint8_t *)(0xFFFF8033 + (((x) - 1) * 0x1838)))

#define MCF_IIC_SR_RXAK                     0x01
#define MCF_IIC_SR_IICIF                    0x02
#define MCF_IIC_SR_SRW                      0x04
#define MCF_IIC_SR_ARBL                     0x10
#define MCF_IIC_SR_BUSY                     0x20
#define MCF_IIC_SR_IAAS                     0x40
#define MCF_IIC_SR_TCF                      0x80


/*** IIC_DR - IIC Data I/O Register)) 0xFFFF8034, 0xFFFF986C ***/
#define MCF_IIC_DR(x) 						(*(volatile uint8_t *)(0xFFFF8034 + (((x) - 1) * 0x1838)))


/*** IIC_CR2 - IIC Control Register 2)) 0xFFFF8035, 0xFFFF986D ***/
#define MCF_IIC_CR2(x) 						(*(volatile uint8_t *)(0xFFFF8035 + (((x) - 1) * 0x1838)))

#define MCF_IIC_CR2_ADEXT                   0x40
#define MCF_IIC_CR2_GCAEN                   0x80
#define MCF_IIC_CR2_AD                     	0x07
#define MCF_IIC_CR2_AD_BITNUM              	0x00

