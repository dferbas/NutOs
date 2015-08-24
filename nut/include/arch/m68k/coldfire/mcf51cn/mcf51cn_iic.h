/*
 * Copyright 2012-2015 by Embedded Technologies s.r.o
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

/* IIC Address Register */
#define MCF_IIC_ADR(x)                      (*(volatile uint8_t *)(0xFFFF8200 + ((x) * 0x20)))
#define MCF_IIC_ADR_MASK                    0xFE
#define MCF_IIC_ADR_BITNUM                  0x01
#define MCF_IIC_ADR_ADR(x)                  (((x) & 0x7F) << 1)

/* IIC Frequency Divider Register */
#define MCF_IIC_FDR(x)                      (*(volatile uint8_t *)(0xFFFF8201 + ((x) * 0x20)))
#define MCF_IIC_FDR_ICR_MASK                0x3F
#define MCF_IIC_FDR_ICR_BITNUM              0x00
#define MCF_IIC_FDR_ICR(x)                  (((x) & 0x3F) << 0)
#define MCF_IIC_FDR_MULT_MASK               0xC0
#define MCF_IIC_FDR_MULT_BITNUM             0x06
#define MCF_IIC_FDR_MULT(x)                 (((x) & 0x03) << 6)

/* IIC Control Register 1 */
#define MCF_IIC_CR(x)                       (*(volatile uint8_t *)(0xFFFF8202 + ((x) * 0x20)))
#define MCF_IIC_CR_RSTA                     0x04
#define MCF_IIC_CR_TXAK                     0x08
#define MCF_IIC_CR_TX                       0x10
#define MCF_IIC_CR_MST                      0x20
#define MCF_IIC_CR_IICIE                    0x40
#define MCF_IIC_CR_IICEN                    0x80

/* IIC Status Register */
#define MCF_IIC_SR(x)                       (*(volatile uint8_t *)(0xFFFF8203 + ((x) * 0x20)))
#define MCF_IIC_SR_RXAK                     0x01
#define MCF_IIC_SR_IICIF                    0x02
#define MCF_IIC_SR_SRW                      0x04
#define MCF_IIC_SR_ARBL                     0x10
#define MCF_IIC_SR_BUSY                     0x20
#define MCF_IIC_SR_IAAS                     0x40
#define MCF_IIC_SR_TCF                      0x80

/* IIC Data I/O Register */
#define MCF_IIC_DR(x)                       (*(volatile uint8_t *)(0xFFFF8204 + ((x) * 0x20)))

/* IIC Control Register 2 */
#define MCF_IIC_CR2(x)                      (*(volatile uint8_t *)(0xFFFF8205 + ((x) * 0x20)))
#define MCF_IIC_CR2_ADEXT                   0x40
#define MCF_IIC_CR2_GCAEN                   0x80
#define MCF_IIC_CR2_AD                      0x07
#define MCF_IIC_CR2_AD_BITNUM               0x00

