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

#ifndef MCF51CN_IIC_H_
#define MCF51CN_IIC_H_

/* IIC Address Register */
#define MCF_IIC_A1(x) 						(*(volatile uint8_t *)(0xFFFF8200 + ((x - 1) * 0x20)))

/* IIC Frequency Divider Register */
#define MCF_IIC_F(x) 						(*(volatile uint8_t *)(0xFFFF8201 + ((x - 1) * 0x20)))
#define MCF_IIC_F_ICR0                      0x01
#define MCF_IIC_F_ICR1                      0x02
#define MCF_IIC_F_ICR2                      0x04
#define MCF_IIC_F_ICR3                      0x08
#define MCF_IIC_F_ICR4                      0x10
#define MCF_IIC_F_ICR5                      0x20
#define MCF_IIC_F_MULT0                     0x40
#define MCF_IIC_F_MULT1                     0x80
#define MCF_IIC_F_ICR                       0x3F
#define MCF_IIC_F_ICR_BITNUM                0x00
#define MCF_IIC_F_MULT                      0xC0
#define MCF_IIC_F_MULT_BITNUM               0x06


/* IIC Control Register 1 */
#define MCF_IIC_C1(x) 						(*(volatile uint8_t *)(0xFFFF8202 + ((x - 1) * 0x20)))
#define MCF_IIC_C1_RSTA                 	0x04
#define MCF_IIC_C1_TXAK                 	0x08
#define MCF_IIC_C1_TX                   	0x10
#define MCF_IIC_C1_MST                  	0x20
#define MCF_IIC_C1_IICIE                	0x40
#define MCF_IIC_C1_IICEN                	0x80


/* IIC Status Register */
#define MCF_IIC_S(x) 						(*(volatile uint8_t *)(0xFFFF8203 + ((x - 1) * 0x20)))
#define MCF_IIC_S_RXAK                      0x01
#define MCF_IIC_S_IICIF                     0x02
#define MCF_IIC_S_SRW                       0x04
#define MCF_IIC_S_ARBL                      0x10
#define MCF_IIC_S_BUSY                      0x20
#define MCF_IIC_S_IAAS                      0x40
#define MCF_IIC_S_TCF                       0x80


/* IIC Data I/O Register */
#define MCF_IIC_D(x) 						(*(volatile uint8_t *)(0xFFFF8204 + ((x - 1) * 0x20)))
#define MCF_IIC_D_DATA0                     0x01
#define MCF_IIC_D_DATA1                     0x02
#define MCF_IIC_D_DATA2                     0x04
#define MCF_IIC_D_DATA3                     0x08
#define MCF_IIC_D_DATA4                     0x10
#define MCF_IIC_D_DATA5                     0x20
#define MCF_IIC_D_DATA6                     0x40
#define MCF_IIC_D_DATA7                     0x80


/* IIC Control Register 2 */
#define MCF_IIC_C2(x) 						(*(volatile uint8_t *)(0xFFFF8205 + ((x - 1) * 0x20)))
#define MCF_IIC_C2_AD8                      0x01
#define MCF_IIC_C2_AD9                      0x02
#define MCF_IIC_C2_AD10                     0x04
#define MCF_IIC_C2_ADEXT                    0x40
#define MCF_IIC_C2_GCAEN                    0x80
#define MCF_IIC_C2_AD_8                     0x07
#define MCF_IIC_C2_AD_8_BITNUM              0x00


/* IIC SMBus Control and Status Register */
#define MCF_IIC_SMB(x) 						(*(volatile uint8_t *)(0xFFFF8206 + ((x - 1) * 0x20)))
#define MCF_IIC_SMB_SHTF                    0x04
#define MCF_IIC_SMB_SLTF                    0x08
#define MCF_IIC_SMB_TCKSEL                  0x10
#define MCF_IIC_SMB_SIICAEN                 0x20
#define MCF_IIC_SMB_ALERTEN                 0x40
#define MCF_IIC_SMB_FACK                    0x80


/* IIC Address Register 2 */
#define MCF_IIC_A2(x) 						(*(volatile uint8_t *)(0xFFFF8207 + ((x - 1) * 0x20)))
#define MCF_IIC_A2_SAD1                     0x02
#define MCF_IIC_A2_SAD2                     0x04
#define MCF_IIC_A2_SAD3                     0x08
#define MCF_IIC_A2_SAD4                     0x10
#define MCF_IIC_A2_SAD5                     0x20
#define MCF_IIC_A2_SAD6                     0x40
#define MCF_IIC_A2_SAD7                     0x80
#define MCF_IIC_A2_SAD_1                    0xFE
#define MCF_IIC_A2_SAD_1_BITNUM             0x01


/* IIC SCL Low Time Out register */
#define MCF_IIC_SLT(x) 						(*(volatile uint16_t *)(0xFFFF8208 + ((x - 1) * 0x20)))
#define MCF_IIC_SLT_SSLT0                   0x01
#define MCF_IIC_SLT_SSLT1                   0x02
#define MCF_IIC_SLT_SSLT2                   0x04
#define MCF_IIC_SLT_SSLT3                   0x08
#define MCF_IIC_SLT_SSLT4                   0x10
#define MCF_IIC_SLT_SSLT5                   0x20
#define MCF_IIC_SLT_SSLT6                   0x40
#define MCF_IIC_SLT_SSLT7                   0x80
#define MCF_IIC_SLT_SSLT8                   0x0100
#define MCF_IIC_SLT_SSLT9                   0x0200
#define MCF_IIC_SLT_SSLT10                  0x0400
#define MCF_IIC_SLT_SSLT11                  0x0800
#define MCF_IIC_SLT_SSLT12                  0x1000
#define MCF_IIC_SLT_SSLT13                  0x2000
#define MCF_IIC_SLT_SSLT14                  0x4000
#define MCF_IIC_SLT_SSLT15                  0x8000


/* IIC Programmable Input Glitch Filter */
#define MCF_IIC_FLT(x) 						(*(volatile uint8_t *)(0xFFFF820A + ((x - 1) * 0x20)))
#define MCF_IIC_FLT_FLT0                    0x01
#define MCF_IIC_FLT_FLT1                    0x02
#define MCF_IIC_FLT_FLT2                    0x04
#define MCF_IIC_FLT_FLT3                    0x08
#define MCF_IIC_FLT_FLT                     0x0F
#define MCF_IIC_FLT_FLT_BITNUM              0x00

#endif /* MCF51CN_IIC_H_ */
