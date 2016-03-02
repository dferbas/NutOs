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


/*
 * For PORTG (bank 6) Data and Data Direction register address offset is 14.
 * For PORTH (bank 7) Data and Data Direction register address offset is 15.
 * For PORTJ (bank 8) Data and Data Direction register address offset is 23.
 * Otherwise bank = bank
 */
#define set_offset(bank) ((bank == 6) ? 14 : ((bank == 7) ? 15 : ((bank == 8) ? 23 : bank)))

/* Port A, B, C, D, E, F, G, H, J Data Register */
#define MCF_GPIO_D(bank) (*(volatile uint8_t *) (0xFFFF8000 + 0x2 * set_offset(bank)))

/* Port A, B, C, D, E, F, G, H, J Data Direction Register */
#define MCF_GPIO_DD(bank) (*(volatile uint8_t *) (0xFFFF8001 + 0x2 * set_offset(bank)))

/* Port A, B, C, D, E, F, G, H, J Pull Enable Register */
#define MCF_GPIO_PE(bank)   (*(volatile uint8_t *) (0xFFFF9840 + 0x4*(bank)))

/* Port A, B, C, D, E, F, G, H, J Slew Rate Enable Register */
#define MCF_GPIO_SE(bank)   (*(volatile uint8_t *) (0xFFFF9841 + 0x4*(bank)))

/* Port A, B, C, D, E, F, G, H, J Slew Rate Enable Register */
#define MCF_GPIO_DS(bank)   (*(volatile uint8_t *) (0xFFFF9842 + 0x4*(bank)))


/*** PTCSET - Port C Data Set Register)) 0xFFFF9878 ***/
#define MCF_GPIO_SET_PTC (*(volatile uint8_t *)(0xFFFF9878))


/*** PTESET - Port E Data Set Register)) 0xFFFF9879 ***/
#define MCF_GPIO_SET_PTE (*(volatile uint8_t *)(0xFFFF9879))


/*** PTCCLR - Port C Data Clear Register)) 0xFFFF987A ***/
#define MCF_GPIO_CLR_PTC (*(volatile uint8_t *)(0xFFFF987A))


/*** PTECLR - Port E Data Clear Register)) 0xFFFF987B ***/
#define MCF_GPIO_CLR_PTE (*(volatile uint8_t *)(0xFFFF987B))


/*** PTCTOG - Port C Toggle Register)) 0xFFFF987C ***/
#define MCF_GPIO_TOG_PTC (*(volatile uint8_t *)(0xFFFF987C))


/*** PTETOG - Port E Toggle Register)) 0xFFFF987D ***/
#define MCF_GPIO_TOG_PTE (*(volatile uint8_t *)(0xFFFF987D))
