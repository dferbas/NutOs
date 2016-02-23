/*
 * Copyright 2014 by Embedded Technologies s.r.o
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

/* GPT Registers */
#define MCF_GPT_GPTIOS                       (*(volatile uint8_t *)(0x401A0000))
#define MCF_GPT_GPTCFORC                     (*(volatile uint8_t *)(0x401A0001))
#define MCF_GPT_GPTOC3M                      (*(volatile uint8_t *)(0x401A0002))
#define MCF_GPT_GPTOC3D                      (*(volatile uint8_t *)(0x401A0003))
#define MCF_GPT_GPTCNT                       (*(volatile uint16_t*)(0x401A0004))
#define MCF_GPT_GPTSCR1                      (*(volatile uint8_t *)(0x401A0006))
#define MCF_GPT_GPTTOV                       (*(volatile uint8_t *)(0x401A0008))
#define MCF_GPT_GPTCTL1                      (*(volatile uint8_t *)(0x401A0009))
#define MCF_GPT_GPTCTL2                      (*(volatile uint8_t *)(0x401A000B))
#define MCF_GPT_GPTIE                        (*(volatile uint8_t *)(0x401A000C))
#define MCF_GPT_GPTSCR2                      (*(volatile uint8_t *)(0x401A000D))
#define MCF_GPT_GPTFLG1                      (*(volatile uint8_t *)(0x401A000E))
#define MCF_GPT_GPTFLG2                      (*(volatile uint8_t *)(0x401A000F))
#define MCF_GPT_GPTC0                        (*(volatile uint16_t*)(0x401A0010))
#define MCF_GPT_GPTC1                        (*(volatile uint16_t*)(0x401A0012))
#define MCF_GPT_GPTC2                        (*(volatile uint16_t*)(0x401A0014))
#define MCF_GPT_GPTC3                        (*(volatile uint16_t*)(0x401A0016))
#define MCF_GPT_GPTPACTL                     (*(volatile uint8_t *)(0x401A0018))
#define MCF_GPT_GPTPAFLG                     (*(volatile uint8_t *)(0x401A0019))
#define MCF_GPT_GPTPACNT                     (*(volatile uint16_t*)(0x401A001A))
#define MCF_GPT_GPTPORT                      (*(volatile uint8_t *)(0x401A001D))
#define MCF_GPT_GPTDDR                       (*(volatile uint8_t *)(0x401A001E))
#define MCF_GPT_GPTC(ch)                     (*(volatile uint16_t*)(0x401A0010 + 2*(ch)))

/* GPT Channels */
#define MCF_GPT_CHANNEL0                     0
#define MCF_GPT_CHANNEL1                     1
#define MCF_GPT_CHANNEL2                     2
#define MCF_GPT_CHANNEL3                     3
#define MCF_GPT_CHANNEL_COUNT                (MCF_GPT_CHANNEL3 + 1)

/* MCF_GPT_GPTIOS */
/*
#define MCF_GPT_GPTIOS_IOS0                  (0x1)
#define MCF_GPT_GPTIOS_IOS1                  (0x2)
#define MCF_GPT_GPTIOS_IOS2                  (0x4)
*/
#define MCF_GPT_GPTIOS_IOS3                  (0x8)

#define MCF_GPT_GPTIOS_IOS(ch)               ((1 << (ch)) & 0xF)

/* MCF_GPT_GPTCFORC */
#define MCF_GPT_GPTCFORC_FOC0                (0x1)
#define MCF_GPT_GPTCFORC_FOC1                (0x2)
#define MCF_GPT_GPTCFORC_FOC2                (0x4)
#define MCF_GPT_GPTCFORC_FOC3                (0x8)

/* MCF_GPT_GPTOC3M */
#define MCF_GPT_GPTOC3M_OC3M0                (0x1)
#define MCF_GPT_GPTOC3M_OC3M1                (0x2)
#define MCF_GPT_GPTOC3M_OC3M2                (0x4)
#define MCF_GPT_GPTOC3M_OC3M3                (0x8)

/* MCF_GPT_GPTOC3D */
#define MCF_GPT_GPTOC3D_OC3D0                (0x1)
#define MCF_GPT_GPTOC3D_OC3D1                (0x2)
#define MCF_GPT_GPTOC3D_OC3D2                (0x4)
#define MCF_GPT_GPTOC3D_OC3D3                (0x8)

/* MCF_GPT_GPTCNT */
#define MCF_GPT_GPTCNT_CNTR(x)               (((x)&0xFFFF)<<0)

/* MCF_GPT_GPTSCR1 */
#define MCF_GPT_GPTSCR1_TFFCA                (0x10)
#define MCF_GPT_GPTSCR1_GPTEN                (0x80)

/* MCF_GPT_GPTTOV */
#define MCF_GPT_GPTTOV_TOV0                  (0x1)
#define MCF_GPT_GPTTOV_TOV1                  (0x2)
#define MCF_GPT_GPTTOV_TOV2                  (0x4)
#define MCF_GPT_GPTTOV_TOV3                  (0x8)

/* MCF_GPT_GPTCTL1 */
#define MCF_GPT_GPTCTL1_OL0                  (0x1)
#define MCF_GPT_GPTCTL1_OM0                  (0x2)
#define MCF_GPT_GPTCTL1_OL1                  (0x4)
#define MCF_GPT_GPTCTL1_OM1                  (0x8)
#define MCF_GPT_GPTCTL1_OL2                  (0x10)
#define MCF_GPT_GPTCTL1_OM2                  (0x20)
#define MCF_GPT_GPTCTL1_OL3                  (0x40)
#define MCF_GPT_GPTCTL1_OM3                  (0x80)
#define MCF_GPT_GPTCTL1_OUTPUT0_NOTHING      (0)
#define MCF_GPT_GPTCTL1_OUTPUT0_TOGGLE       (0x1)
#define MCF_GPT_GPTCTL1_OUTPUT0_CLEAR        (0x2)
#define MCF_GPT_GPTCTL1_OUTPUT0_SET          (0x3)
#define MCF_GPT_GPTCTL1_OUTPUT1_NOTHING      (0)
#define MCF_GPT_GPTCTL1_OUTPUT1_TOGGLE       (0x4)
#define MCF_GPT_GPTCTL1_OUTPUT1_CLEAR        (0x8)
#define MCF_GPT_GPTCTL1_OUTPUT1_SET          (0xC)
#define MCF_GPT_GPTCTL1_OUTPUT2_NOTHING      (0)
#define MCF_GPT_GPTCTL1_OUTPUT2_TOGGLE       (0x10)
#define MCF_GPT_GPTCTL1_OUTPUT2_CLEAR        (0x20)
#define MCF_GPT_GPTCTL1_OUTPUT2_SET          (0x30)
#define MCF_GPT_GPTCTL1_OUTPUT3_NOTHING      (0)
#define MCF_GPT_GPTCTL1_OUTPUT3_TOGGLE       (0x40)
#define MCF_GPT_GPTCTL1_OUTPUT3_CLEAR        (0x80)
#define MCF_GPT_GPTCTL1_OUTPUT3_SET          (0xC0)

#define MCF_GPT_GPTCTL1_OUTPUT_NOTHING(ch)   (0x0 << 2*(ch))
#define MCF_GPT_GPTCTL1_OUTPUT_TOGGLE(ch)    (0x1 << 2*(ch))
#define MCF_GPT_GPTCTL1_OUTPUT_CLEAR(ch)     (0x2 << 2*(ch))
#define MCF_GPT_GPTCTL1_OUTPUT_SET(ch)       (0x3 << 2*(ch))
#define MCF_GPT_GPTCTL1_OUTPUT_MASK(ch)      (0x3 << 2*(ch))

/* MCF_GPT_GPTCTL2 */
/*
#define MCF_GPT_GPTCTL2_EDG0A                (0x1)
#define MCF_GPT_GPTCTL2_EDG0B                (0x2)
#define MCF_GPT_GPTCTL2_EDG1A                (0x4)
#define MCF_GPT_GPTCTL2_EDG1B                (0x8)
#define MCF_GPT_GPTCTL2_EDG2A                (0x10)
#define MCF_GPT_GPTCTL2_EDG2B                (0x20)
#define MCF_GPT_GPTCTL2_EDG3A                (0x40)
#define MCF_GPT_GPTCTL2_EDG3B                (0x80)
#define MCF_GPT_GPTCTL2_INPUT0_DISABLED      (0)
#define MCF_GPT_GPTCTL2_INPUT0_RISING        (0x1)
#define MCF_GPT_GPTCTL2_INPUT0_FALLING       (0x2)
#define MCF_GPT_GPTCTL2_INPUT0_ANY           (0x3)
#define MCF_GPT_GPTCTL2_INPUT1_DISABLED      (0)
#define MCF_GPT_GPTCTL2_INPUT1_RISING        (0x4)
#define MCF_GPT_GPTCTL2_INPUT1_FALLING       (0x8)
#define MCF_GPT_GPTCTL2_INPUT1_ANY           (0xC)
#define MCF_GPT_GPTCTL2_INPUT2_DISABLED      (0)
#define MCF_GPT_GPTCTL2_INPUT2_RISING        (0x10)
#define MCF_GPT_GPTCTL2_INPUT2_FALLING       (0x20)
#define MCF_GPT_GPTCTL2_INPUT2_ANY           (0x30)
#define MCF_GPT_GPTCTL2_INPUT3_DISABLED      (0)
#define MCF_GPT_GPTCTL2_INPUT3_RISING        (0x40)
#define MCF_GPT_GPTCTL2_INPUT3_FALLING       (0x80)
#define MCF_GPT_GPTCTL2_INPUT3_ANY           (0xC0)
*/
#define MCF_GPT_GPTCTL2_INPUT_DISABLED(ch)   (0x0 << 2*(ch))
#define MCF_GPT_GPTCTL2_INPUT_RISING(ch)     (0x1 << 2*(ch))
#define MCF_GPT_GPTCTL2_INPUT_FALLING(ch)    (0x2 << 2*(ch))
#define MCF_GPT_GPTCTL2_INPUT_ANY(ch)        (0x3 << 2*(ch))
#define MCF_GPT_GPTCTL2_INPUT_MASK(ch)       (0x3 << 2*(ch))

/* MCF_GPT_GPTIE */
/*
#define MCF_GPT_GPTIE_CI0                    (0x1)
#define MCF_GPT_GPTIE_CI1                    (0x2)
#define MCF_GPT_GPTIE_CI2                    (0x4)
#define MCF_GPT_GPTIE_CI3                    (0x8)
*/
#define MCF_GPT_GPTIE_CI(ch)                 (1 << ch)

/* MCF_GPT_GPTSCR2 */
#define MCF_GPT_GPTSCR2_PR(x)                (((x)&0x7)<<0)
#define MCF_GPT_GPTSCR2_PR_1                 (0)
#define MCF_GPT_GPTSCR2_PR_2                 (0x1)
#define MCF_GPT_GPTSCR2_PR_4                 (0x2)
#define MCF_GPT_GPTSCR2_PR_8                 (0x3)
#define MCF_GPT_GPTSCR2_PR_16                (0x4)
#define MCF_GPT_GPTSCR2_PR_32                (0x5)
#define MCF_GPT_GPTSCR2_PR_64                (0x6)
#define MCF_GPT_GPTSCR2_PR_128               (0x7)
#define MCF_GPT_GPTSCR2_TCRE                 (0x8)
#define MCF_GPT_GPTSCR2_RDPT                 (0x10)
#define MCF_GPT_GPTSCR2_PUPT                 (0x20)
#define MCF_GPT_GPTSCR2_TOI                  (0x80)

/* MCF_GPT_GPTFLG1 */
/*
#define MCF_GPT_GPTFLG1_CF0                  (0x1)
#define MCF_GPT_GPTFLG1_CF1                  (0x2)
#define MCF_GPT_GPTFLG1_CF2                  (0x4)
#define MCF_GPT_GPTFLG1_CF3                  (0x8)
*/
#define MCF_GPT_GPTFLG1_CF(ch)               (1 << (ch))

/* MCF_GPT_GPTFLG2 */
#define MCF_GPT_GPTFLG2_TOF                  (0x80)

/* MCF_GPT_GPTC */
#define MCF_GPT_GPTC_CCNT(x)                 (((x)&0xFFFF)<<0)

/* MCF_GPT_GPTPACTL */
#define MCF_GPT_GPTPACTL_PAI                 (0x1)
#define MCF_GPT_GPTPACTL_PAOVI               (0x2)
#define MCF_GPT_GPTPACTL_CLK(x)              (((x)&0x3)<<0x2)
#define MCF_GPT_GPTPACTL_CLK_GPTPR           (0)
#define MCF_GPT_GPTPACTL_CLK_PACLK           (0x1)
#define MCF_GPT_GPTPACTL_CLK_PACLK_256       (0x2)
#define MCF_GPT_GPTPACTL_CLK_PACLK_65536     (0x3)
#define MCF_GPT_GPTPACTL_PEDGE               (0x10)
#define MCF_GPT_GPTPACTL_PAMOD               (0x20)
#define MCF_GPT_GPTPACTL_PAE                 (0x40)

/* MCF_GPT_GPTPAFLG */
#define MCF_GPT_GPTPAFLG_PAIF                (0x1)
#define MCF_GPT_GPTPAFLG_PAOVF               (0x2)

/* MCF_GPT_GPTPACNT */
#define MCF_GPT_GPTPACNT_PACNT(x)            (((x)&0xFFFF)<<0)

/* MCF_GPT_GPTPORT */
#define MCF_GPT_GPTPORT_PORTT0               (0x1)
#define MCF_GPT_GPTPORT_PORTT1               (0x2)
#define MCF_GPT_GPTPORT_PORTT2               (0x4)
#define MCF_GPT_GPTPORT_PORTT3               (0x8)

/* MCF_GPT_GPTDDR */
#define MCF_GPT_GPTDDR_DDRT0                 (0x1)
#define MCF_GPT_GPTDDR_DDRT1                 (0x2)
#define MCF_GPT_GPTDDR_DDRT2                 (0x4)
#define MCF_GPT_GPTDDR_DDRT3                 (0x8)
