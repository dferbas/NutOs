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


/*** ADC_SC1 - Status and Control Register 1)) 0xFFFF8010 ***/
#define MCF_ADC_SC1 						(*(volatile uint8_t *)(0xFFFF8010))

#define MCF_ADC_SC1_ADCH0                    0x01
#define MCF_ADC_SC1_ADCH1                    0x02
#define MCF_ADC_SC1_ADCH2                    0x04
#define MCF_ADC_SC1_ADCH3                    0x08
#define MCF_ADC_SC1_ADCH4                    0x10
#define MCF_ADC_SC1_ADCO                     0x20
#define MCF_ADC_SC1_AIEN                     0x40
#define MCF_ADC_SC1_COCO                     0x80
#define MCF_ADC_SC1_ADCH_MASK                0x1F
#define MCF_ADC_SC1_ADCH_BITNUM              0x00
#define MCF_ADC_SC1_ADCH(x)					 (((x)&0x1F)<<0)


/*** ADC_SC2 - Status and Control Register 2)) 0xFFFF8011 ***/
#define MCF_ADC_SC2 						(*(volatile uint8_t *)(0xFFFF8011))

#define MCF_ADC_SC2_ACFGT                    0x10
#define MCF_ADC_SC2_ACFE                     0x20
#define MCF_ADC_SC2_ADTRG                    0x40
#define MCF_ADC_SC2_ADACT                    0x80


/*** ADC_R - Data Result Register)) 0xFFFF8012 ***/
#define MCF_ADC_R 							(*(volatile uint16_t *)(0xFFFF8012))

#define MCF_ADC_R_ADR                        0x0FFF


/*** ADC_CV - Compare Value Register)) 0xFFFF8014 ***/
#define MCF_ADC_CV 							(*(volatile uint16_t *)(0xFFFF8014))

#define MCF_ADC_CV_ADCV                      0x0FFF


/*** ADC_CFG - Configuration Register)) 0xFFFF8016 ***/
#define MCF_ADC_CFG 						(*(volatile uint16_t *)(0xFFFF8016))

#define MCF_ADC_CFG_ADICLK0                  0x01
#define MCF_ADC_CFG_ADICLK1                  0x02
#define MCF_ADC_CFG_ADICLK_BUS               0x00
#define MCF_ADC_CFG_ADICLK_BUS_DIV2          0x01
#define MCF_ADC_CFG_ADICLK_ALT               0x02
#define MCF_ADC_CFG_ADICLK_ASYNC             0x03
#define MCF_ADC_CFG_MODE_8BIT                0x00
#define MCF_ADC_CFG_MODE_12BIT               0x04
#define MCF_ADC_CFG_MODE_10BIT               0x08
#define MCF_ADC_CFG_MODE(x)					 (((x)&0x3)<<2)
#define MCF_ADC_CFG_MODE0                    0x04
#define MCF_ADC_CFG_MODE1                    0x08
#define MCF_ADC_CFG_ADLSMP                   0x10
#define MCF_ADC_CFG_ADIV0                    0x20
#define MCF_ADC_CFG_ADIV1                    0x40
#define MCF_ADC_CFG_ADLPC                    0x80
#define MCF_ADC_CFG_ADICLK                   0x03
#define MCF_ADC_CFG_ADICLK_BITNUM            0x00
#define MCF_ADC_CFG_MODE_MASK                0x0C
#define MCF_ADC_CFG_MODE_BITNUM              0x02
#define MCF_ADC_CFG_ADIV(x)                  (((x)&0x3)<<5)
#define MCF_ADC_CFG_ADIV_MASK                0x60
#define MCF_ADC_CFG_ADIV_BITNUM              0x05
