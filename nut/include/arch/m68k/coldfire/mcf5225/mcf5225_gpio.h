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
 * GPIO Registers
 */
#define MCF_GPIO_PORT(bank)  (*(volatile uint8_t *)(0x40100000 + (bank)))
#define MCF_GPIO_DDR(bank)   (*(volatile uint8_t *)(0x40100018 + (bank)))
#define MCF_GPIO_PIN(bank)   (*(volatile uint8_t *)(0x40100030 + (bank)))
#define MCF_GPIO_SET(bank)   (*(volatile uint8_t *)(0x40100030 + (bank)))
#define MCF_GPIO_CLR(bank)   (*(volatile uint8_t *)(0x40100048 + (bank)))
#define MCF_GPIO_PAR8(bank)  (*(volatile uint8_t *)(0x40100060 + (bank)))
#define MCF_GPIO_PAR16(bank) (*(volatile uint16_t *)(0x40100060 + ((bank == 3) ? 0x30 : (bank))))


/*********************************************************************
*
* General Purpose I/O (GPIO)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_GPIO_PORTTE                      (*(volatile uint8_t *)(0x40100000))
#define MCF_GPIO_DDRTE                       (*(volatile uint8_t *)(0x40100018))
#define MCF_GPIO_SETTE                       (*(volatile uint8_t *)(0x40100030))
#define MCF_GPIO_CLRTE                       (*(volatile uint8_t *)(0x40100048))
#define MCF_GPIO_PTEPAR                      (*(volatile uint8_t *)(0x40100060))

#define MCF_GPIO_PORTTF                      (*(volatile uint8_t *)(0x40100001))
#define MCF_GPIO_DDRTF                       (*(volatile uint8_t *)(0x40100019))
#define MCF_GPIO_SETTF                       (*(volatile uint8_t *)(0x40100031))
#define MCF_GPIO_CLRTF                       (*(volatile uint8_t *)(0x40100049))
#define MCF_GPIO_PTFPAR                      (*(volatile uint8_t *)(0x40100061))

#define MCF_GPIO_PORTTG                      (*(volatile uint8_t *)(0x40100002))
#define MCF_GPIO_DDRTG                       (*(volatile uint8_t *)(0x4010001A))
#define MCF_GPIO_SETTG                       (*(volatile uint8_t *)(0x40100032))
#define MCF_GPIO_CLRTG                       (*(volatile uint8_t *)(0x4010004A))
#define MCF_GPIO_PTGPAR                      (*(volatile uint8_t *)(0x40100062))

#define MCF_GPIO_PORTTH                      (*(volatile uint8_t *)(0x40100003))
#define MCF_GPIO_DDRTH                       (*(volatile uint8_t *)(0x4010001B))
#define MCF_GPIO_SETTH                       (*(volatile uint8_t *)(0x40100033))
#define MCF_GPIO_CLRTH                       (*(volatile uint8_t *)(0x4010004B))
#define MCF_GPIO_PTHPAR                      (*(volatile uint16_t*)(0x40100090))

#define MCF_GPIO_PORTTI                      (*(volatile uint8_t *)(0x40100004))
#define MCF_GPIO_DDRTI                       (*(volatile uint8_t *)(0x4010001C))
#define MCF_GPIO_SETTI                       (*(volatile uint8_t *)(0x40100034))
#define MCF_GPIO_CLRTI                       (*(volatile uint8_t *)(0x4010004C))
#define MCF_GPIO_PTIPAR                      (*(volatile uint8_t *)(0x40100064))

#define MCF_GPIO_PORTTJ                      (*(volatile uint8_t *)(0x40100006))
#define MCF_GPIO_DDRTJ                       (*(volatile uint8_t *)(0x4010001E))
#define MCF_GPIO_SETTJ                       (*(volatile uint8_t *)(0x40100036))
#define MCF_GPIO_CLRTJ                       (*(volatile uint8_t *)(0x4010004E))
#define MCF_GPIO_PTJPAR                      (*(volatile uint8_t *)(0x40100066))

#define MCF_GPIO_PORTNQ                      (*(volatile uint8_t *)(0x40100008))
#define MCF_GPIO_DDRNQ                       (*(volatile uint8_t *)(0x40100020))
#define MCF_GPIO_SETNQ                       (*(volatile uint8_t *)(0x40100038))
#define MCF_GPIO_CLRNQ                       (*(volatile uint8_t *)(0x40100050))
#define MCF_GPIO_PNQPAR                      (*(volatile uint16_t*)(0x40100068))

#define MCF_GPIO_PORTAN                      (*(volatile uint8_t *)(0x4010000A))
#define MCF_GPIO_DDRAN                       (*(volatile uint8_t *)(0x40100022))
#define MCF_GPIO_SETAN                       (*(volatile uint8_t *)(0x4010003A))
#define MCF_GPIO_CLRAN                       (*(volatile uint8_t *)(0x40100052))
#define MCF_GPIO_PANPAR                      (*(volatile uint8_t *)(0x4010006A))

#define MCF_GPIO_PORTAS                      (*(volatile uint8_t *)(0x4010000B))
#define MCF_GPIO_DDRAS                       (*(volatile uint8_t *)(0x40100023))
#define MCF_GPIO_SETAS                       (*(volatile uint8_t *)(0x4010003B))
#define MCF_GPIO_CLRAS                       (*(volatile uint8_t *)(0x40100053))
#define MCF_GPIO_PASPAR                      (*(volatile uint8_t *)(0x4010006B))

#define MCF_GPIO_PORTQS                      (*(volatile uint8_t *)(0x4010000C))
#define MCF_GPIO_DDRQS                       (*(volatile uint8_t *)(0x40100024))
#define MCF_GPIO_SETQS                       (*(volatile uint8_t *)(0x4010003C))
#define MCF_GPIO_CLRQS                       (*(volatile uint8_t *)(0x40100054))
#define MCF_GPIO_PQSPAR                      (*(volatile uint16_t*)(0x4010006C))

#define MCF_GPIO_PORTTA                      (*(volatile uint8_t *)(0x4010000E))
#define MCF_GPIO_DDRTA                       (*(volatile uint8_t *)(0x40100026))
#define MCF_GPIO_SETTA                       (*(volatile uint8_t *)(0x4010003E))
#define MCF_GPIO_CLRTA                       (*(volatile uint8_t *)(0x40100056))
#define MCF_GPIO_PTAPAR                      (*(volatile uint8_t *)(0x4010006E))

#define MCF_GPIO_PORTTC                      (*(volatile uint8_t *)(0x4010000F))
#define MCF_GPIO_DDRTC                       (*(volatile uint8_t *)(0x40100027))
#define MCF_GPIO_SETTC                       (*(volatile uint8_t *)(0x4010003F))
#define MCF_GPIO_CLRTC                       (*(volatile uint8_t *)(0x40100057))
#define MCF_GPIO_PTCPAR                      (*(volatile uint8_t *)(0x4010006F))

#define MCF_GPIO_PORTUA                      (*(volatile uint8_t *)(0x40100011))
#define MCF_GPIO_DDRUA                       (*(volatile uint8_t *)(0x40100029))
#define MCF_GPIO_SETUA                       (*(volatile uint8_t *)(0x40100041))
#define MCF_GPIO_CLRUA                       (*(volatile uint8_t *)(0x40100059))
#define MCF_GPIO_PUAPAR                      (*(volatile uint8_t *)(0x40100071))

#define MCF_GPIO_PORTUB                      (*(volatile uint8_t *)(0x40100012))
#define MCF_GPIO_DDRUB                       (*(volatile uint8_t *)(0x4010002A))
#define MCF_GPIO_SETUB                       (*(volatile uint8_t *)(0x40100042))
#define MCF_GPIO_CLRUB                       (*(volatile uint8_t *)(0x4010005A))
#define MCF_GPIO_PUBPAR                      (*(volatile uint8_t *)(0x40100072))

#define MCF_GPIO_PORTUC                      (*(volatile uint8_t *)(0x40100013))
#define MCF_GPIO_DDRUC                       (*(volatile uint8_t *)(0x4010002B))
#define MCF_GPIO_SETUC                       (*(volatile uint8_t *)(0x40100043))
#define MCF_GPIO_CLRUC                       (*(volatile uint8_t *)(0x4010005B))
#define MCF_GPIO_PUCPAR                      (*(volatile uint8_t *)(0x40100073))

#define MCF_GPIO_PORTDD                      (*(volatile uint8_t *)(0x40100014))
#define MCF_GPIO_DDRDD                       (*(volatile uint8_t *)(0x4010002C))
#define MCF_GPIO_SETDD                       (*(volatile uint8_t *)(0x40100044))
#define MCF_GPIO_CLRDD                       (*(volatile uint8_t *)(0x4010005C))
#define MCF_GPIO_PDDPAR                      (*(volatile uint8_t *)(0x40100074))



/* Bit definitions and macros for MCF_GPIO_PORTTE */
#define MCF_GPIO_PORTTE_PORTTE0              (0x1)
#define MCF_GPIO_PORTTE_PORTTE1              (0x2)
#define MCF_GPIO_PORTTE_PORTTE2              (0x4)
#define MCF_GPIO_PORTTE_PORTTE3              (0x8)
#define MCF_GPIO_PORTTE_PORTTE4              (0x10)
#define MCF_GPIO_PORTTE_PORTTE5              (0x20)
#define MCF_GPIO_PORTTE_PORTTE6              (0x40)
#define MCF_GPIO_PORTTE_PORTTE7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRTE */
#define MCF_GPIO_DDRTE_DDRTE0                (0x1)
#define MCF_GPIO_DDRTE_DDRTE1                (0x2)
#define MCF_GPIO_DDRTE_DDRTE2                (0x4)
#define MCF_GPIO_DDRTE_DDRTE3                (0x8)
#define MCF_GPIO_DDRTE_DDRTE4                (0x10)
#define MCF_GPIO_DDRTE_DDRTE5                (0x20)
#define MCF_GPIO_DDRTE_DDRTE6                (0x40)
#define MCF_GPIO_DDRTE_DDRTE7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETTE */
#define MCF_GPIO_SETTE_SETTE0                (0x1)
#define MCF_GPIO_SETTE_SETTE1                (0x2)
#define MCF_GPIO_SETTE_SETTE2                (0x4)
#define MCF_GPIO_SETTE_SETTE3                (0x8)
#define MCF_GPIO_SETTE_SETTE4                (0x10)
#define MCF_GPIO_SETTE_SETTE5                (0x20)
#define MCF_GPIO_SETTE_SETTE6                (0x40)
#define MCF_GPIO_SETTE_SETTE7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRTE */
#define MCF_GPIO_CLRTE_CLRTE0                (0x1)
#define MCF_GPIO_CLRTE_CLRTE1                (0x2)
#define MCF_GPIO_CLRTE_CLRTE2                (0x4)
#define MCF_GPIO_CLRTE_CLRTE3                (0x8)
#define MCF_GPIO_CLRTE_CLRTE4                (0x10)
#define MCF_GPIO_CLRTE_CLRTE5                (0x20)
#define MCF_GPIO_CLRTE_CLRTE6                (0x40)
#define MCF_GPIO_CLRTE_CLRTE7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PTEPAR */
#define MCF_GPIO_PTEPAR_PTEPAR0              (0x1)
#define MCF_GPIO_PTEPAR_MB_A0_GPIO           (0)
#define MCF_GPIO_PTEPAR_MB_A0_MB_A0          (0x1)
#define MCF_GPIO_PTEPAR_PTEPAR1              (0x2)
#define MCF_GPIO_PTEPAR_MB_A1_GPIO           (0)
#define MCF_GPIO_PTEPAR_MB_A1_MB_A1          (0x2)
#define MCF_GPIO_PTEPAR_PTEPAR2              (0x4)
#define MCF_GPIO_PTEPAR_MB_A2_GPIO           (0)
#define MCF_GPIO_PTEPAR_MB_A2_MB_A2          (0x4)
#define MCF_GPIO_PTEPAR_PTEPAR3              (0x8)
#define MCF_GPIO_PTEPAR_MB_A3_GPIO           (0)
#define MCF_GPIO_PTEPAR_MB_A3_MB_A3          (0x8)
#define MCF_GPIO_PTEPAR_PTEPAR4              (0x10)
#define MCF_GPIO_PTEPAR_MB_A4_GPIO           (0)
#define MCF_GPIO_PTEPAR_MB_A4_MB_A4          (0x10)
#define MCF_GPIO_PTEPAR_PTEPAR5              (0x20)
#define MCF_GPIO_PTEPAR_MB_A5_GPIO           (0)
#define MCF_GPIO_PTEPAR_MB_A5_MB_A5          (0x20)
#define MCF_GPIO_PTEPAR_PTEPAR6              (0x40)
#define MCF_GPIO_PTEPAR_MB_A6_GPIO           (0)
#define MCF_GPIO_PTEPAR_MB_A6_MB_A6          (0x40)
#define MCF_GPIO_PTEPAR_PTEPAR7              (0x80)
#define MCF_GPIO_PTEPAR_MB_A7_GPIO           (0)
#define MCF_GPIO_PTEPAR_MB_A7_MB_A7          (0x80)

/* Bit definitions and macros for MCF_GPIO_PORTTF */
#define MCF_GPIO_PORTTF_PORTTF0              (0x1)
#define MCF_GPIO_PORTTF_PORTTF1              (0x2)
#define MCF_GPIO_PORTTF_PORTTF2              (0x4)
#define MCF_GPIO_PORTTF_PORTTF3              (0x8)
#define MCF_GPIO_PORTTF_PORTTF4              (0x10)
#define MCF_GPIO_PORTTF_PORTTF5              (0x20)
#define MCF_GPIO_PORTTF_PORTTF6              (0x40)
#define MCF_GPIO_PORTTF_PORTTF7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRTF */
#define MCF_GPIO_DDRTF_DDRTF0                (0x1)
#define MCF_GPIO_DDRTF_DDRTF1                (0x2)
#define MCF_GPIO_DDRTF_DDRTF2                (0x4)
#define MCF_GPIO_DDRTF_DDRTF3                (0x8)
#define MCF_GPIO_DDRTF_DDRTF4                (0x10)
#define MCF_GPIO_DDRTF_DDRTF5                (0x20)
#define MCF_GPIO_DDRTF_DDRTF6                (0x40)
#define MCF_GPIO_DDRTF_DDRTF7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETTF */
#define MCF_GPIO_SETTF_SETTF0                (0x1)
#define MCF_GPIO_SETTF_SETTF1                (0x2)
#define MCF_GPIO_SETTF_SETTF2                (0x4)
#define MCF_GPIO_SETTF_SETTF3                (0x8)
#define MCF_GPIO_SETTF_SETTF4                (0x10)
#define MCF_GPIO_SETTF_SETTF5                (0x20)
#define MCF_GPIO_SETTF_SETTF6                (0x40)
#define MCF_GPIO_SETTF_SETTF7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRTF */
#define MCF_GPIO_CLRTF_CLRTF0                (0x1)
#define MCF_GPIO_CLRTF_CLRTF1                (0x2)
#define MCF_GPIO_CLRTF_CLRTF2                (0x4)
#define MCF_GPIO_CLRTF_CLRTF3                (0x8)
#define MCF_GPIO_CLRTF_CLRTF4                (0x10)
#define MCF_GPIO_CLRTF_CLRTF5                (0x20)
#define MCF_GPIO_CLRTF_CLRTF6                (0x40)
#define MCF_GPIO_CLRTF_CLRTF7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PTFPAR */
#define MCF_GPIO_PTFPAR_PTFPAR0              (0x1)
#define MCF_GPIO_PTFPAR_MB_A8_GPIO           (0)
#define MCF_GPIO_PTFPAR_MB_A8_MB_A8          (0x1)
#define MCF_GPIO_PTFPAR_PTFPAR1              (0x2)
#define MCF_GPIO_PTFPAR_MB_A9_GPIO           (0)
#define MCF_GPIO_PTFPAR_MB_A9_MB_A9          (0x2)
#define MCF_GPIO_PTFPAR_PTFPAR2              (0x4)
#define MCF_GPIO_PTFPAR_MB_A10_GPIO          (0)
#define MCF_GPIO_PTFPAR_MB_A10_MB_A10        (0x4)
#define MCF_GPIO_PTFPAR_PTFPAR3              (0x8)
#define MCF_GPIO_PTFPAR_MB_A11_GPIO          (0)
#define MCF_GPIO_PTFPAR_MB_A11_MB_A11        (0x8)
#define MCF_GPIO_PTFPAR_PTFPAR4              (0x10)
#define MCF_GPIO_PTFPAR_MB_A12_GPIO          (0)
#define MCF_GPIO_PTFPAR_MB_A12_MB_A12        (0x10)
#define MCF_GPIO_PTFPAR_PTFPAR5              (0x20)
#define MCF_GPIO_PTFPAR_MB_A13_GPIO          (0)
#define MCF_GPIO_PTFPAR_MB_A13_MB_A13        (0x20)
#define MCF_GPIO_PTFPAR_PTFPAR6              (0x40)
#define MCF_GPIO_PTFPAR_MB_A14_GPIO          (0)
#define MCF_GPIO_PTFPAR_MB_A14_MB_A14        (0x40)
#define MCF_GPIO_PTFPAR_PTFPAR7              (0x80)
#define MCF_GPIO_PTFPAR_MB_A15_GPIO          (0)
#define MCF_GPIO_PTFPAR_MB_A15_MB_A15        (0x80)

/* Bit definitions and macros for MCF_GPIO_PORTTG */
#define MCF_GPIO_PORTTG_PORTTG0              (0x1)
#define MCF_GPIO_PORTTG_PORTTG1              (0x2)
#define MCF_GPIO_PORTTG_PORTTG2              (0x4)
#define MCF_GPIO_PORTTG_PORTTG3              (0x8)
#define MCF_GPIO_PORTTG_PORTTG4              (0x10)
#define MCF_GPIO_PORTTG_PORTTG5              (0x20)
#define MCF_GPIO_PORTTG_PORTTG6              (0x40)
#define MCF_GPIO_PORTTG_PORTTG7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRTG */
#define MCF_GPIO_DDRTG_DDRTG0                (0x1)
#define MCF_GPIO_DDRTG_DDRTG1                (0x2)
#define MCF_GPIO_DDRTG_DDRTG2                (0x4)
#define MCF_GPIO_DDRTG_DDRTG3                (0x8)
#define MCF_GPIO_DDRTG_DDRTG4                (0x10)
#define MCF_GPIO_DDRTG_DDRTG5                (0x20)
#define MCF_GPIO_DDRTG_DDRTG6                (0x40)
#define MCF_GPIO_DDRTG_DDRTG7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETTG */
#define MCF_GPIO_SETTG_SETTG0                (0x1)
#define MCF_GPIO_SETTG_SETTG1                (0x2)
#define MCF_GPIO_SETTG_SETTG2                (0x4)
#define MCF_GPIO_SETTG_SETTG3                (0x8)
#define MCF_GPIO_SETTG_SETTG4                (0x10)
#define MCF_GPIO_SETTG_SETTG5                (0x20)
#define MCF_GPIO_SETTG_SETTG6                (0x40)
#define MCF_GPIO_SETTG_SETTG7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRTG */
#define MCF_GPIO_CLRTG_CLRTG0                (0x1)
#define MCF_GPIO_CLRTG_CLRTG1                (0x2)
#define MCF_GPIO_CLRTG_CLRTG2                (0x4)
#define MCF_GPIO_CLRTG_CLRTG3                (0x8)
#define MCF_GPIO_CLRTG_CLRTG4                (0x10)
#define MCF_GPIO_CLRTG_CLRTG5                (0x20)
#define MCF_GPIO_CLRTG_CLRTG6                (0x40)
#define MCF_GPIO_CLRTG_CLRTG7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PTGPAR */
#define MCF_GPIO_PTGPAR_PTGPAR0              (0x1)
#define MCF_GPIO_PTGPAR_MB_A16_GPIO          (0)
#define MCF_GPIO_PTGPAR_MB_A16_MB_A16        (0x1)
#define MCF_GPIO_PTGPAR_PTGPAR1              (0x2)
#define MCF_GPIO_PTGPAR_MB_A17_GPIO          (0)
#define MCF_GPIO_PTGPAR_MB_A17_MB_A17        (0x2)
#define MCF_GPIO_PTGPAR_PTGPAR2              (0x4)
#define MCF_GPIO_PTGPAR_MB_A18_GPIO          (0)
#define MCF_GPIO_PTGPAR_MB_A18_MB_A18        (0x4)
#define MCF_GPIO_PTGPAR_PTGPAR3              (0x8)
#define MCF_GPIO_PTGPAR_MB_A19_GPIO          (0)
#define MCF_GPIO_PTGPAR_MB_A19_MB_A19        (0x8)
#define MCF_GPIO_PTGPAR_PTGPAR4              (0x10)
#define MCF_GPIO_PTGPAR_PTGPAR5              (0x20)
#define MCF_GPIO_PTGPAR_MB_CS1_GPIO          (0)
#define MCF_GPIO_PTGPAR_MB_CS0_MB_CS0        (0x20)
#define MCF_GPIO_PTGPAR_PTGPAR6              (0x40)
#define MCF_GPIO_PTGPAR_MB_OE_GPIO           (0)
#define MCF_GPIO_PTGPAR_MB_OE_MB_OE          (0x40)
#define MCF_GPIO_PTGPAR_PTGPAR7              (0x80)
#define MCF_GPIO_PTGPAR_MB_RW_GPIO           (0)
#define MCF_GPIO_PTGPAR_MB_RW_MB_RW          (0x80)

/* Bit definitions and macros for MCF_GPIO_PORTTH */
#define MCF_GPIO_PORTTH_PORTTH0              (0x1)
#define MCF_GPIO_PORTTH_PORTTH1              (0x2)
#define MCF_GPIO_PORTTH_PORTTH2              (0x4)
#define MCF_GPIO_PORTTH_PORTTH3              (0x8)
#define MCF_GPIO_PORTTH_PORTTH4              (0x10)
#define MCF_GPIO_PORTTH_PORTTH5              (0x20)
#define MCF_GPIO_PORTTH_PORTTH6              (0x40)
#define MCF_GPIO_PORTTH_PORTTH7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRTH */
#define MCF_GPIO_DDRTH_DDRTH0                (0x1)
#define MCF_GPIO_DDRTH_DDRTH1                (0x2)
#define MCF_GPIO_DDRTH_DDRTH2                (0x4)
#define MCF_GPIO_DDRTH_DDRTH3                (0x8)
#define MCF_GPIO_DDRTH_DDRTH4                (0x10)
#define MCF_GPIO_DDRTH_DDRTH5                (0x20)
#define MCF_GPIO_DDRTH_DDRTH6                (0x40)
#define MCF_GPIO_DDRTH_DDRTH7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETTH */
#define MCF_GPIO_SETTH_SETTH0                (0x1)
#define MCF_GPIO_SETTH_SETTH1                (0x2)
#define MCF_GPIO_SETTH_SETTH2                (0x4)
#define MCF_GPIO_SETTH_SETTH3                (0x8)
#define MCF_GPIO_SETTH_SETTH4                (0x10)
#define MCF_GPIO_SETTH_SETTH5                (0x20)
#define MCF_GPIO_SETTH_SETTH6                (0x40)
#define MCF_GPIO_SETTH_SETTH7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRTH */
#define MCF_GPIO_CLRTH_CLRTH0                (0x1)
#define MCF_GPIO_CLRTH_CLRTH1                (0x2)
#define MCF_GPIO_CLRTH_CLRTH2                (0x4)
#define MCF_GPIO_CLRTH_CLRTH3                (0x8)
#define MCF_GPIO_CLRTH_CLRTH4                (0x10)
#define MCF_GPIO_CLRTH_CLRTH5                (0x20)
#define MCF_GPIO_CLRTH_CLRTH6                (0x40)
#define MCF_GPIO_CLRTH_CLRTH7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PTHPAR */
#define MCF_GPIO_PTHPAR_PTHPAR0(x)           (((x)&0x3)<<0)
#define MCF_GPIO_PTHPAR_MB_D0_GPIO           (0)
#define MCF_GPIO_PTHPAR_MB_D0_MB_D0          (0x1)
#define MCF_GPIO_PTHPAR_MB_D0_SYNCB          (0x2)
#define MCF_GPIO_PTHPAR_PTHPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PTHPAR_MB_D1_GPIO           (0)
#define MCF_GPIO_PTHPAR_MB_D1_MB_D1          (0x4)
#define MCF_GPIO_PTHPAR_MB_D1_SYNCA          (0x8)
#define MCF_GPIO_PTHPAR_PTHPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PTHPAR_MB_D2_GPIO           (0)
#define MCF_GPIO_PTHPAR_MB_D2_MB_D2          (0x10)
#define MCF_GPIO_PTHPAR_MB_D2_USB_VBUSE      (0x20)
#define MCF_GPIO_PTHPAR_PTHPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_PTHPAR_MB_D3_GPIO           (0)
#define MCF_GPIO_PTHPAR_MB_D3_MB_D3          (0x40)
#define MCF_GPIO_PTHPAR_MB_D3_USB_VBUSD      (0x80)
#define MCF_GPIO_PTHPAR_PTHPAR4(x)           (((x)&0x3)<<0x8)
#define MCF_GPIO_PTHPAR_MB_D4_GPIO           (0)
#define MCF_GPIO_PTHPAR_MB_D4_MB_D4          (0x100)
#define MCF_GPIO_PTHPAR_MB_D4_SDA1           (0x200)
#define MCF_GPIO_PTHPAR_PTHPAR5(x)           (((x)&0x3)<<0xA)
#define MCF_GPIO_PTHPAR_MB_D5_GPIO           (0)
#define MCF_GPIO_PTHPAR_MB_D5_MB_D5          (0x400)
#define MCF_GPIO_PTHPAR_MB_D5_SCL1           (0x800)
#define MCF_GPIO_PTHPAR_PTHPAR6(x)           (((x)&0x3)<<0xC)
#define MCF_GPIO_PTHPAR_MB_D6_GPIO           (0)
#define MCF_GPIO_PTHPAR_MB_D6_MB_D6          (0x1000)
#define MCF_GPIO_PTHPAR_MB_D6_CANTX          (0x2000)
#define MCF_GPIO_PTHPAR_PTHPAR7(x)           (((x)&0x3)<<0xE)
#define MCF_GPIO_PTHPAR_MB_D7_GPIO           (0)
#define MCF_GPIO_PTHPAR_MB_D7_MB_D7          (0x4000)
#define MCF_GPIO_PTHPAR_MB_D7_CANRX          (0x8000)

/* Bit definitions and macros for MCF_GPIO_PORTTI */
#define MCF_GPIO_PORTTI_PORTTI0              (0x1)
#define MCF_GPIO_PORTTI_PORTTI1              (0x2)
#define MCF_GPIO_PORTTI_PORTTI2              (0x4)
#define MCF_GPIO_PORTTI_PORTTI3              (0x8)
#define MCF_GPIO_PORTTI_PORTTI4              (0x10)
#define MCF_GPIO_PORTTI_PORTTI5              (0x20)
#define MCF_GPIO_PORTTI_PORTTI6              (0x40)
#define MCF_GPIO_PORTTI_PORTTI7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRTI */
#define MCF_GPIO_DDRTI_DDRTI0                (0x1)
#define MCF_GPIO_DDRTI_DDRTI1                (0x2)
#define MCF_GPIO_DDRTI_DDRTI2                (0x4)
#define MCF_GPIO_DDRTI_DDRTI3                (0x8)
#define MCF_GPIO_DDRTI_DDRTI4                (0x10)
#define MCF_GPIO_DDRTI_DDRTI5                (0x20)
#define MCF_GPIO_DDRTI_DDRTI6                (0x40)
#define MCF_GPIO_DDRTI_DDRTI7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETTI */
#define MCF_GPIO_SETTI_SETTI0                (0x1)
#define MCF_GPIO_SETTI_SETTI1                (0x2)
#define MCF_GPIO_SETTI_SETTI2                (0x4)
#define MCF_GPIO_SETTI_SETTI3                (0x8)
#define MCF_GPIO_SETTI_SETTI4                (0x10)
#define MCF_GPIO_SETTI_SETTI5                (0x20)
#define MCF_GPIO_SETTI_SETTI6                (0x40)
#define MCF_GPIO_SETTI_SETTI7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRTI */
#define MCF_GPIO_CLRTI_CLRTI0                (0x1)
#define MCF_GPIO_CLRTI_CLRTI1                (0x2)
#define MCF_GPIO_CLRTI_CLRTI2                (0x4)
#define MCF_GPIO_CLRTI_CLRTI3                (0x8)
#define MCF_GPIO_CLRTI_CLRTI4                (0x10)
#define MCF_GPIO_CLRTI_CLRTI5                (0x20)
#define MCF_GPIO_CLRTI_CLRTI6                (0x40)
#define MCF_GPIO_CLRTI_CLRTI7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PTIPAR */
#define MCF_GPIO_PTIPAR_PTIPAR0              (0x1)
#define MCF_GPIO_PTIPAR_FEC_COL_GPIO         (0)
#define MCF_GPIO_PTIPAR_FEC_COL_FEC_COL      (0x1)
#define MCF_GPIO_PTIPAR_PTIPAR1              (0x2)
#define MCF_GPIO_PTIPAR_FEC_CRS_GPIO         (0)
#define MCF_GPIO_PTIPAR_FEC_CRS_FEC_CRS      (0x2)
#define MCF_GPIO_PTIPAR_PTIPAR2              (0x4)
#define MCF_GPIO_PTIPAR_FEC_RXCLK_GPIO       (0)
#define MCF_GPIO_PTIPAR_FEC_RXCLK_FEC_RXCLK  (0x4)
#define MCF_GPIO_PTIPAR_PTIPAR3              (0x8)
#define MCF_GPIO_PTIPAR_FEC_RXD0_GPIO        (0)
#define MCF_GPIO_PTIPAR_FEC_RXD0_FEC_RXD0    (0x8)
#define MCF_GPIO_PTIPAR_PTIPAR4              (0x10)
#define MCF_GPIO_PTIPAR_FEC_RXD1_GPIO        (0)
#define MCF_GPIO_PTIPAR_FEC_RXD1_FEC_RXD1    (0x10)
#define MCF_GPIO_PTIPAR_PTIPAR5              (0x20)
#define MCF_GPIO_PTIPAR_FEC_RXD2_GPIO        (0)
#define MCF_GPIO_PTIPAR_FEC_RXD2_FEC_RXD2    (0x20)
#define MCF_GPIO_PTIPAR_PTIPAR6              (0x40)
#define MCF_GPIO_PTIPAR_FEC_RXD3_GPIO        (0)
#define MCF_GPIO_PTIPAR_FEC_RXD3_FEC_RXD3    (0x40)
#define MCF_GPIO_PTIPAR_PTIPAR7              (0x80)
#define MCF_GPIO_PTIPAR_FEC_RXDV_GPIO        (0)
#define MCF_GPIO_PTIPAR_FEC_RXDV_FEC_RXDV    (0x80)

/* Bit definitions and macros for MCF_GPIO_PORTTJ */
#define MCF_GPIO_PORTTJ_PORTTJ0              (0x1)
#define MCF_GPIO_PORTTJ_PORTTJ1              (0x2)
#define MCF_GPIO_PORTTJ_PORTTJ2              (0x4)
#define MCF_GPIO_PORTTJ_PORTTJ3              (0x8)
#define MCF_GPIO_PORTTJ_PORTTJ4              (0x10)
#define MCF_GPIO_PORTTJ_PORTTJ5              (0x20)
#define MCF_GPIO_PORTTJ_PORTTJ6              (0x40)
#define MCF_GPIO_PORTTJ_PORTTJ7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRTJ */
#define MCF_GPIO_DDRTJ_DDRTJ0                (0x1)
#define MCF_GPIO_DDRTJ_DDRTJ1                (0x2)
#define MCF_GPIO_DDRTJ_DDRTJ2                (0x4)
#define MCF_GPIO_DDRTJ_DDRTJ3                (0x8)
#define MCF_GPIO_DDRTJ_DDRTJ4                (0x10)
#define MCF_GPIO_DDRTJ_DDRTJ5                (0x20)
#define MCF_GPIO_DDRTJ_DDRTJ6                (0x40)
#define MCF_GPIO_DDRTJ_DDRTJ7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETTJ */
#define MCF_GPIO_SETTJ_SETTJ0                (0x1)
#define MCF_GPIO_SETTJ_SETTJ1                (0x2)
#define MCF_GPIO_SETTJ_SETTJ2                (0x4)
#define MCF_GPIO_SETTJ_SETTJ3                (0x8)
#define MCF_GPIO_SETTJ_SETTJ4                (0x10)
#define MCF_GPIO_SETTJ_SETTJ5                (0x20)
#define MCF_GPIO_SETTJ_SETTJ6                (0x40)
#define MCF_GPIO_SETTJ_SETTJ7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRTJ */
#define MCF_GPIO_CLRTJ_CLRTJ0                (0x1)
#define MCF_GPIO_CLRTJ_CLRTJ1                (0x2)
#define MCF_GPIO_CLRTJ_CLRTJ2                (0x4)
#define MCF_GPIO_CLRTJ_CLRTJ3                (0x8)
#define MCF_GPIO_CLRTJ_CLRTJ4                (0x10)
#define MCF_GPIO_CLRTJ_CLRTJ5                (0x20)
#define MCF_GPIO_CLRTJ_CLRTJ6                (0x40)
#define MCF_GPIO_CLRTJ_CLRTJ7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PTJPAR */
#define MCF_GPIO_PTJPAR_PTJPAR0              (0x1)
#define MCF_GPIO_PTJPAR_FEC_RXER_GPIO        (0)
#define MCF_GPIO_PTJPAR_FEC_RXER_FEC_RXER    (0x1)
#define MCF_GPIO_PTJPAR_PTJPAR1              (0x2)
#define MCF_GPIO_PTJPAR_FEC_TXCLK_GPIO       (0)
#define MCF_GPIO_PTJPAR_FEC_TXCLK_FEC_TXCLK  (0x2)
#define MCF_GPIO_PTJPAR_PTJPAR2              (0x4)
#define MCF_GPIO_PTJPAR_FEC_TXD0_GPIO        (0)
#define MCF_GPIO_PTJPAR_FEC_TXD0_FEC_TXD0    (0x4)
#define MCF_GPIO_PTJPAR_PTJPAR3              (0x8)
#define MCF_GPIO_PTJPAR_FEC_TXD1_GPIO        (0)
#define MCF_GPIO_PTJPAR_FEC_TXD1_FEC_TXD1    (0x8)
#define MCF_GPIO_PTJPAR_PTJPAR4              (0x10)
#define MCF_GPIO_PTJPAR_FEC_TXD2_GPIO        (0)
#define MCF_GPIO_PTJPAR_FEC_TXD2_FEC_TXD2    (0x10)
#define MCF_GPIO_PTJPAR_PTJPAR5              (0x20)
#define MCF_GPIO_PTJPAR_FEC_TXD3_GPIO        (0)
#define MCF_GPIO_PTJPAR_FEC_TXD3_FEC_TXD3    (0x20)
#define MCF_GPIO_PTJPAR_PTJPAR6              (0x40)
#define MCF_GPIO_PTJPAR_FEC_TXEN_GPIO        (0)
#define MCF_GPIO_PTJPAR_FEC_TXEN_FEC_TXEN    (0x40)
#define MCF_GPIO_PTJPAR_PTJPAR7              (0x80)
#define MCF_GPIO_PTJPAR_FEC_TXER_GPIO        (0)
#define MCF_GPIO_PTJPAR_FEC_TXER_FEC_TXER    (0x80)

/* Bit definitions and macros for MCF_GPIO_PORTNQ */
#define MCF_GPIO_PORTNQ_PORTNQ1              (0x2)
#define MCF_GPIO_PORTNQ_PORTNQ2              (0x4)
#define MCF_GPIO_PORTNQ_PORTNQ3              (0x8)
#define MCF_GPIO_PORTNQ_PORTNQ4              (0x10)
#define MCF_GPIO_PORTNQ_PORTNQ5              (0x20)
#define MCF_GPIO_PORTNQ_PORTNQ6              (0x40)
#define MCF_GPIO_PORTNQ_PORTNQ7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRNQ */
#define MCF_GPIO_DDRNQ_DDRNQ1                (0x2)
#define MCF_GPIO_DDRNQ_DDRNQ2                (0x4)
#define MCF_GPIO_DDRNQ_DDRNQ3                (0x8)
#define MCF_GPIO_DDRNQ_DDRNQ4                (0x10)
#define MCF_GPIO_DDRNQ_DDRNQ5                (0x20)
#define MCF_GPIO_DDRNQ_DDRNQ6                (0x40)
#define MCF_GPIO_DDRNQ_DDRNQ7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETNQ */
#define MCF_GPIO_SETNQ_SETNQ1                (0x2)
#define MCF_GPIO_SETNQ_SETNQ2                (0x4)
#define MCF_GPIO_SETNQ_SETNQ3                (0x8)
#define MCF_GPIO_SETNQ_SETNQ4                (0x10)
#define MCF_GPIO_SETNQ_SETNQ5                (0x20)
#define MCF_GPIO_SETNQ_SETNQ6                (0x40)
#define MCF_GPIO_SETNQ_SETNQ7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRNQ */
#define MCF_GPIO_CLRNQ_CLRNQ1                (0x2)
#define MCF_GPIO_CLRNQ_CLRNQ2                (0x4)
#define MCF_GPIO_CLRNQ_CLRNQ3                (0x8)
#define MCF_GPIO_CLRNQ_CLRNQ4                (0x10)
#define MCF_GPIO_CLRNQ_CLRNQ5                (0x20)
#define MCF_GPIO_CLRNQ_CLRNQ6                (0x40)
#define MCF_GPIO_CLRNQ_CLRNQ7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PNQPAR */
#define MCF_GPIO_PNQPAR_PNQPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PNQPAR_IRQ1_GPIO            (0)
#define MCF_GPIO_PNQPAR_IRQ1_IRQ1            (0x4)
#define MCF_GPIO_PNQPAR_IRQ1_USB_ALT_CLK     (0xC)
#define MCF_GPIO_PNQPAR_PNQPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PNQPAR_PNQPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_PNQPAR_IRQ3_GPIO            (0)
#define MCF_GPIO_PNQPAR_IRQ3_IRQ3            (0x40)
#define MCF_GPIO_PNQPAR_IRQ3_FEC_MDIO        (0x80)
#define MCF_GPIO_PNQPAR_PNQPAR4(x)           (((x)&0x3)<<0x8)
#define MCF_GPIO_PNQPAR_PNQPAR5(x)           (((x)&0x3)<<0xA)
#define MCF_GPIO_PNQPAR_IRQ5_GPIO            (0)
#define MCF_GPIO_PNQPAR_IRQ5_IRQ5            (0x400)
#define MCF_GPIO_PNQPAR_IRQ5_FEC_MDC         (0x800)
#define MCF_GPIO_PNQPAR_PNQPAR6(x)           (((x)&0x3)<<0xC)
#define MCF_GPIO_PNQPAR_PNQPAR7(x)           (((x)&0x3)<<0xE)
#define MCF_GPIO_PNQPAR_IRQ7_GPIO            (0)
#define MCF_GPIO_PNQPAR_IRQ7_IRQ7            (0x4000)

/* Bit definitions and macros for MCF_GPIO_PORTAN */
#define MCF_GPIO_PORTAN_PORTAN0              (0x1)
#define MCF_GPIO_PORTAN_PORTAN1              (0x2)
#define MCF_GPIO_PORTAN_PORTAN2              (0x4)
#define MCF_GPIO_PORTAN_PORTAN3              (0x8)
#define MCF_GPIO_PORTAN_PORTAN4              (0x10)
#define MCF_GPIO_PORTAN_PORTAN5              (0x20)
#define MCF_GPIO_PORTAN_PORTAN6              (0x40)
#define MCF_GPIO_PORTAN_PORTAN7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRAN */
#define MCF_GPIO_DDRAN_DDRAN0                (0x1)
#define MCF_GPIO_DDRAN_DDRAN1                (0x2)
#define MCF_GPIO_DDRAN_DDRAN2                (0x4)
#define MCF_GPIO_DDRAN_DDRAN3                (0x8)
#define MCF_GPIO_DDRAN_DDRAN4                (0x10)
#define MCF_GPIO_DDRAN_DDRAN5                (0x20)
#define MCF_GPIO_DDRAN_DDRAN6                (0x40)
#define MCF_GPIO_DDRAN_DDRAN7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETAN */
#define MCF_GPIO_SETAN_SETAN0                (0x1)
#define MCF_GPIO_SETAN_SETAN1                (0x2)
#define MCF_GPIO_SETAN_SETAN2                (0x4)
#define MCF_GPIO_SETAN_SETAN3                (0x8)
#define MCF_GPIO_SETAN_SETAN4                (0x10)
#define MCF_GPIO_SETAN_SETAN5                (0x20)
#define MCF_GPIO_SETAN_SETAN6                (0x40)
#define MCF_GPIO_SETAN_SETAN7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRAN */
#define MCF_GPIO_CLRAN_CLRAN0                (0x1)
#define MCF_GPIO_CLRAN_CLRAN1                (0x2)
#define MCF_GPIO_CLRAN_CLRAN2                (0x4)
#define MCF_GPIO_CLRAN_CLRAN3                (0x8)
#define MCF_GPIO_CLRAN_CLRAN4                (0x10)
#define MCF_GPIO_CLRAN_CLRAN5                (0x20)
#define MCF_GPIO_CLRAN_CLRAN6                (0x40)
#define MCF_GPIO_CLRAN_CLRAN7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PANPAR */
#define MCF_GPIO_PANPAR_PANPAR0              (0x1)
#define MCF_GPIO_PANPAR_AN0_GPIO             (0)
#define MCF_GPIO_PANPAR_AN0_AN0              (0x1)
#define MCF_GPIO_PANPAR_PANPAR1              (0x2)
#define MCF_GPIO_PANPAR_AN1_GPIO             (0)
#define MCF_GPIO_PANPAR_AN1_AN1              (0x2)
#define MCF_GPIO_PANPAR_PANPAR2              (0x4)
#define MCF_GPIO_PANPAR_AN2_GPIO             (0)
#define MCF_GPIO_PANPAR_AN2_AN2              (0x4)
#define MCF_GPIO_PANPAR_PANPAR3              (0x8)
#define MCF_GPIO_PANPAR_AN3_GPIO             (0)
#define MCF_GPIO_PANPAR_AN3_AN3              (0x8)
#define MCF_GPIO_PANPAR_PANPAR4              (0x10)
#define MCF_GPIO_PANPAR_AN4_GPIO             (0)
#define MCF_GPIO_PANPAR_AN4_AN4              (0x10)
#define MCF_GPIO_PANPAR_PANPAR5              (0x20)
#define MCF_GPIO_PANPAR_AN5_GPIO             (0)
#define MCF_GPIO_PANPAR_AN5_AN5              (0x20)
#define MCF_GPIO_PANPAR_PANPAR6              (0x40)
#define MCF_GPIO_PANPAR_AN6_GPIO             (0)
#define MCF_GPIO_PANPAR_AN6_AN6              (0x40)
#define MCF_GPIO_PANPAR_PANPAR7              (0x80)
#define MCF_GPIO_PANPAR_AN7_GPIO             (0)
#define MCF_GPIO_PANPAR_AN7_AN7              (0x80)

/* Bit definitions and macros for MCF_GPIO_PORTAS */
#define MCF_GPIO_PORTAS_PORTAS0              (0x1)
#define MCF_GPIO_PORTAS_PORTAS1              (0x2)
#define MCF_GPIO_PORTAS_PORTAS2              (0x4)

/* Bit definitions and macros for MCF_GPIO_DDRAS */
#define MCF_GPIO_DDRAS_DDRAS0                (0x1)
#define MCF_GPIO_DDRAS_DDRAS1                (0x2)
#define MCF_GPIO_DDRAS_DDRAS2                (0x4)

/* Bit definitions and macros for MCF_GPIO_SETAS */
#define MCF_GPIO_SETAS_SETAS0                (0x1)
#define MCF_GPIO_SETAS_SETAS1                (0x2)
#define MCF_GPIO_SETAS_SETAS2                (0x4)

/* Bit definitions and macros for MCF_GPIO_CLRAS */
#define MCF_GPIO_CLRAS_CLRAS0                (0x1)
#define MCF_GPIO_CLRAS_CLRAS1                (0x2)
#define MCF_GPIO_CLRAS_CLRAS2                (0x4)

/* Bit definitions and macros for MCF_GPIO_PASPAR */
#define MCF_GPIO_PASPAR_PASPAR0(x)           (((x)&0x3)<<0)
#define MCF_GPIO_PASPAR_SCL0_GPIO            (0)
#define MCF_GPIO_PASPAR_SCL0_SCL0            (0x1)
#define MCF_GPIO_PASPAR_SCL0_UTXD2           (0x3)
#define MCF_GPIO_PASPAR_PASPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PASPAR_SDA0_GPIO            (0)
#define MCF_GPIO_PASPAR_SDA0_SDA0            (0x4)
#define MCF_GPIO_PASPAR_SDA0_URXD2           (0xC)
#define MCF_GPIO_PASPAR_PASPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PASPAR_MB_ALE_GPIO          (0)
#define MCF_GPIO_PASPAR_MB_ALE_MB_ALE        (0x10)
#define MCF_GPIO_PASPAR_MB_ALE_MB_CS1        (0x20)

/* Bit definitions and macros for MCF_GPIO_PORTQS */
#define MCF_GPIO_PORTQS_PORTQS0              (0x1)
#define MCF_GPIO_PORTQS_PORTQS1              (0x2)
#define MCF_GPIO_PORTQS_PORTQS2              (0x4)
#define MCF_GPIO_PORTQS_PORTQS3              (0x8)
#define MCF_GPIO_PORTQS_PORTQS4              (0x10)
#define MCF_GPIO_PORTQS_PORTQS5              (0x20)
#define MCF_GPIO_PORTQS_PORTQS6              (0x40)

/* Bit definitions and macros for MCF_GPIO_DDRQS */
#define MCF_GPIO_DDRQS_DDRQS0                (0x1)
#define MCF_GPIO_DDRQS_DDRQS1                (0x2)
#define MCF_GPIO_DDRQS_DDRQS2                (0x4)
#define MCF_GPIO_DDRQS_DDRQS3                (0x8)
#define MCF_GPIO_DDRQS_DDRQS4                (0x10)
#define MCF_GPIO_DDRQS_DDRQS5                (0x20)
#define MCF_GPIO_DDRQS_DDRQS6                (0x40)

/* Bit definitions and macros for MCF_GPIO_SETQS */
#define MCF_GPIO_SETQS_SETQS0                (0x1)
#define MCF_GPIO_SETQS_SETQS1                (0x2)
#define MCF_GPIO_SETQS_SETQS2                (0x4)
#define MCF_GPIO_SETQS_SETQS3                (0x8)
#define MCF_GPIO_SETQS_SETQS4                (0x10)
#define MCF_GPIO_SETQS_SETQS5                (0x20)
#define MCF_GPIO_SETQS_SETQS6                (0x40)

/* Bit definitions and macros for MCF_GPIO_CLRQS */
#define MCF_GPIO_CLRQS_CLRQS0                (0x1)
#define MCF_GPIO_CLRQS_CLRQS1                (0x2)
#define MCF_GPIO_CLRQS_CLRQS2                (0x4)
#define MCF_GPIO_CLRQS_CLRQS3                (0x8)
#define MCF_GPIO_CLRQS_CLRQS4                (0x10)
#define MCF_GPIO_CLRQS_CLRQS5                (0x20)
#define MCF_GPIO_CLRQS_CLRQS6                (0x40)

/* Bit definitions and macros for MCF_GPIO_PQSPAR */
#define MCF_GPIO_PQSPAR_PQSPAR0(x)           (((x)&0x3)<<0)
#define MCF_GPIO_PQSPAR_QSPI_DOUT_GPIO       (0)
#define MCF_GPIO_PQSPAR_QSPI_DOUT_DOUT       (0x1)
#define MCF_GPIO_PQSPAR_QSPI_DOUT_SCL1       (0x2)
#define MCF_GPIO_PQSPAR_QSPI_DOUT_UTXD1      (0x3)
#define MCF_GPIO_PQSPAR_PQSPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PQSPAR_QSPI_DIN_GPIO        (0)
#define MCF_GPIO_PQSPAR_QSPI_DIN_DIN         (0x4)
#define MCF_GPIO_PQSPAR_QSPI_DIN_SDA1        (0x8)
#define MCF_GPIO_PQSPAR_QSPI_DIN_URXD1       (0xC)
#define MCF_GPIO_PQSPAR_PQSPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PQSPAR_QSPI_CLK_GPIO        (0)
#define MCF_GPIO_PQSPAR_QSPI_CLK_CLK         (0x10)
#define MCF_GPIO_PQSPAR_QSPI_CLK_SCL0        (0x20)
#define MCF_GPIO_PQSPAR_QSPI_CLK_URTS1       (0x30)
#define MCF_GPIO_PQSPAR_PQSPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_PQSPAR_QSPI_CS0_GPIO        (0)
#define MCF_GPIO_PQSPAR_QSPI_CS0_CS0         (0x40)
#define MCF_GPIO_PQSPAR_QSPI_CS0_SDA0        (0x80)
#define MCF_GPIO_PQSPAR_QSPI_CS0_UCTS1       (0xC0)
#define MCF_GPIO_PQSPAR_PQSPAR4(x)           (((x)&0x3)<<0x8)
#define MCF_GPIO_PQSPAR_PQSPAR5(x)           (((x)&0x3)<<0xA)
#define MCF_GPIO_PQSPAR_QSPI_CS2_GPIO        (0)
#define MCF_GPIO_PQSPAR_QSPI_CS2_CS2         (0x400)
#define MCF_GPIO_PQSPAR_QSPI_CS2_SYNCB       (0x800)
#define MCF_GPIO_PQSPAR_QSPI_CS2_USB_DM_PDOWN (0xC00)
#define MCF_GPIO_PQSPAR_PQSPAR6(x)           (((x)&0x3)<<0xC)
#define MCF_GPIO_PQSPAR_QSPI_CS3_GPIO        (0)
#define MCF_GPIO_PQSPAR_QSPI_CS3_CS3         (0x1000)
#define MCF_GPIO_PQSPAR_QSPI_CS3_SYNCA       (0x2000)
#define MCF_GPIO_PQSPAR_QSPI_CS3_USB_DP_PDOWN (0x3000)

/* Bit definitions and macros for MCF_GPIO_PORTTA */
#define MCF_GPIO_PORTTA_PORTTA0              (0x1)
#define MCF_GPIO_PORTTA_PORTTA1              (0x2)
#define MCF_GPIO_PORTTA_PORTTA2              (0x4)
#define MCF_GPIO_PORTTA_PORTTA3              (0x8)

/* Bit definitions and macros for MCF_GPIO_DDRTA */
#define MCF_GPIO_DDRTA_DDRTA0                (0x1)
#define MCF_GPIO_DDRTA_DDRTA1                (0x2)
#define MCF_GPIO_DDRTA_DDRTA2                (0x4)
#define MCF_GPIO_DDRTA_DDRTA3                (0x8)

/* Bit definitions and macros for MCF_GPIO_SETTA */
#define MCF_GPIO_SETTA_SETTA0                (0x1)
#define MCF_GPIO_SETTA_SETTA1                (0x2)
#define MCF_GPIO_SETTA_SETTA2                (0x4)
#define MCF_GPIO_SETTA_SETTA3                (0x8)

/* Bit definitions and macros for MCF_GPIO_CLRTA */
#define MCF_GPIO_CLRTA_CLRTA0                (0x1)
#define MCF_GPIO_CLRTA_CLRTA1                (0x2)
#define MCF_GPIO_CLRTA_CLRTA2                (0x4)
#define MCF_GPIO_CLRTA_CLRTA3                (0x8)

/* Bit definitions and macros for MCF_GPIO_PTAPAR */
#define MCF_GPIO_PTAPAR_PTAPAR0(x)           (((x)&0x3)<<0)
#define MCF_GPIO_PTAPAR_ICOC0_GPIO           (0)
#define MCF_GPIO_PTAPAR_ICOC0_ICOC0          (0x1)
#define MCF_GPIO_PTAPAR_ICOC0_PWM1           (0x3)
#define MCF_GPIO_PTAPAR_PTAPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PTAPAR_ICOC1_GPIO           (0)
#define MCF_GPIO_PTAPAR_ICOC1_ICOC1          (0x4)
#define MCF_GPIO_PTAPAR_ICOC1_PWM3           (0xC)
#define MCF_GPIO_PTAPAR_PTAPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PTAPAR_ICOC2_GPIO           (0)
#define MCF_GPIO_PTAPAR_ICOC2_ICOC2          (0x10)
#define MCF_GPIO_PTAPAR_ICOC2_PWM5           (0x30)
#define MCF_GPIO_PTAPAR_PTAPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_PTAPAR_ICOC3_GPIO           (0)
#define MCF_GPIO_PTAPAR_ICOC3_ICOC3          (0x40)
#define MCF_GPIO_PTAPAR_ICOC3_PWM7           (0xC0)

/* Bit definitions and macros for MCF_GPIO_PORTTC */
#define MCF_GPIO_PORTTC_PORTTC0              (0x1)
#define MCF_GPIO_PORTTC_PORTTC1              (0x2)
#define MCF_GPIO_PORTTC_PORTTC2              (0x4)
#define MCF_GPIO_PORTTC_PORTTC3              (0x8)

/* Bit definitions and macros for MCF_GPIO_DDRTC */
#define MCF_GPIO_DDRTC_DDRTC0                (0x1)
#define MCF_GPIO_DDRTC_DDRTC1                (0x2)
#define MCF_GPIO_DDRTC_DDRTC2                (0x4)
#define MCF_GPIO_DDRTC_DDRTC3                (0x8)

/* Bit definitions and macros for MCF_GPIO_SETTC */
#define MCF_GPIO_SETTC_SETTC0                (0x1)
#define MCF_GPIO_SETTC_SETTC1                (0x2)
#define MCF_GPIO_SETTC_SETTC2                (0x4)
#define MCF_GPIO_SETTC_SETTC3                (0x8)

/* Bit definitions and macros for MCF_GPIO_CLRTC */
#define MCF_GPIO_CLRTC_CLRTC0                (0x1)
#define MCF_GPIO_CLRTC_CLRTC1                (0x2)
#define MCF_GPIO_CLRTC_CLRTC2                (0x4)
#define MCF_GPIO_CLRTC_CLRTC3                (0x8)

/* Bit definitions and macros for MCF_GPIO_PTCPAR */
#define MCF_GPIO_PTCPAR_PTCPAR0(x)           (((x)&0x3)<<0)
#define MCF_GPIO_PTCPAR_DTIN0_GPIO           (0)
#define MCF_GPIO_PTCPAR_DTIN0_DTIN0          (0x1)
#define MCF_GPIO_PTCPAR_DTIN0_DTOUT0         (0x2)
#define MCF_GPIO_PTCPAR_DTIN0_PWM0           (0x3)
#define MCF_GPIO_PTCPAR_PTCPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PTCPAR_DTIN1_GPIO           (0)
#define MCF_GPIO_PTCPAR_DTIN1_DTIN1          (0x4)
#define MCF_GPIO_PTCPAR_DTIN1_DTOUT1         (0x8)
#define MCF_GPIO_PTCPAR_DTIN1_PWM2           (0xC)
#define MCF_GPIO_PTCPAR_PTCPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PTCPAR_DTIN2_GPIO           (0)
#define MCF_GPIO_PTCPAR_DTIN2_DTIN2          (0x10)
#define MCF_GPIO_PTCPAR_DTIN2_DTOUT2         (0x20)
#define MCF_GPIO_PTCPAR_DTIN2_PWM4           (0x30)
#define MCF_GPIO_PTCPAR_PTCPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_PTCPAR_DTIN3_GPIO           (0)
#define MCF_GPIO_PTCPAR_DTIN3_DTIN3          (0x40)
#define MCF_GPIO_PTCPAR_DTIN3_DTOUT3         (0x80)
#define MCF_GPIO_PTCPAR_DTIN3_PWM6           (0xC0)

/* Bit definitions and macros for MCF_GPIO_PORTUA */
#define MCF_GPIO_PORTUA_PORTUA0              (0x1)
#define MCF_GPIO_PORTUA_PORTUA1              (0x2)
#define MCF_GPIO_PORTUA_PORTUA2              (0x4)
#define MCF_GPIO_PORTUA_PORTUA3              (0x8)

/* Bit definitions and macros for MCF_GPIO_DDRUA */
#define MCF_GPIO_DDRUA_DDRUA0                (0x1)
#define MCF_GPIO_DDRUA_DDRUA1                (0x2)
#define MCF_GPIO_DDRUA_DDRUA2                (0x4)
#define MCF_GPIO_DDRUA_DDRUA3                (0x8)

/* Bit definitions and macros for MCF_GPIO_SETUA */
#define MCF_GPIO_SETUA_SETUA0                (0x1)
#define MCF_GPIO_SETUA_SETUA1                (0x2)
#define MCF_GPIO_SETUA_SETUA2                (0x4)
#define MCF_GPIO_SETUA_SETUA3                (0x8)

/* Bit definitions and macros for MCF_GPIO_CLRUA */
#define MCF_GPIO_CLRUA_CLRUA0                (0x1)
#define MCF_GPIO_CLRUA_CLRUA1                (0x2)
#define MCF_GPIO_CLRUA_CLRUA2                (0x4)
#define MCF_GPIO_CLRUA_CLRUA3                (0x8)

/* Bit definitions and macros for MCF_GPIO_PUAPAR */
#define MCF_GPIO_PUAPAR_PUAPAR0(x)           (((x)&0x3)<<0)
#define MCF_GPIO_PUAPAR_UTXD0_GPIO           (0)
#define MCF_GPIO_PUAPAR_UTXD0_UTXD0          (0x1)
#define MCF_GPIO_PUAPAR_PUAPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PUAPAR_URXD0_GPIO           (0)
#define MCF_GPIO_PUAPAR_URXD0_URXD0          (0x4)
#define MCF_GPIO_PUAPAR_PUAPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PUAPAR_URTS0_GPIO           (0)
#define MCF_GPIO_PUAPAR_URTS0_URTS0          (0x10)
#define MCF_GPIO_PUAPAR_URTS0_USB_VBUSD      (0x30)
#define MCF_GPIO_PUAPAR_PUAPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_PUAPAR_UCTS0_GPIO           (0)
#define MCF_GPIO_PUAPAR_UCTS0_UCTS0          (0x40)
#define MCF_GPIO_PUAPAR_UCTS0_USB_VBUSE      (0xC0)

/* Bit definitions and macros for MCF_GPIO_PORTUB */
#define MCF_GPIO_PORTUB_PORTUB0              (0x1)
#define MCF_GPIO_PORTUB_PORTUB1              (0x2)
#define MCF_GPIO_PORTUB_PORTUB2              (0x4)
#define MCF_GPIO_PORTUB_PORTUB3              (0x8)

/* Bit definitions and macros for MCF_GPIO_DDRUB */
#define MCF_GPIO_DDRUB_DDRUB0                (0x1)
#define MCF_GPIO_DDRUB_DDRUB1                (0x2)
#define MCF_GPIO_DDRUB_DDRUB2                (0x4)
#define MCF_GPIO_DDRUB_DDRUB3                (0x8)

/* Bit definitions and macros for MCF_GPIO_SETUB */
#define MCF_GPIO_SETUB_SETUB0                (0x1)
#define MCF_GPIO_SETUB_SETUB1                (0x2)
#define MCF_GPIO_SETUB_SETUB2                (0x4)
#define MCF_GPIO_SETUB_SETUB3                (0x8)

/* Bit definitions and macros for MCF_GPIO_CLRUB */
#define MCF_GPIO_CLRUB_CLRUB0                (0x1)
#define MCF_GPIO_CLRUB_CLRUB1                (0x2)
#define MCF_GPIO_CLRUB_CLRUB2                (0x4)
#define MCF_GPIO_CLRUB_CLRUB3                (0x8)

/* Bit definitions and macros for MCF_GPIO_PUBPAR */
#define MCF_GPIO_PUBPAR_PUBPAR0(x)           (((x)&0x3)<<0)
#define MCF_GPIO_PUBPAR_UTXD1_GPIO           (0)
#define MCF_GPIO_PUBPAR_UTXD1_UTXD1          (0x1)
#define MCF_GPIO_PUBPAR_UTXD1_SCL1           (0x2)
#define MCF_GPIO_PUBPAR_PUBPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PUBPAR_URXD1_GPIO           (0)
#define MCF_GPIO_PUBPAR_URXD1_URXD1          (0x4)
#define MCF_GPIO_PUBPAR_URXD1_SDA1           (0x8)
#define MCF_GPIO_PUBPAR_PUBPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PUBPAR_URTS1_GPIO           (0)
#define MCF_GPIO_PUBPAR_URTS1_URTS1          (0x10)
#define MCF_GPIO_PUBPAR_URTS1_SYNCB          (0x20)
#define MCF_GPIO_PUBPAR_URTS1_UTXD2          (0x30)
#define MCF_GPIO_PUBPAR_PUBPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_PUBPAR_UCTS1_GPIO           (0)
#define MCF_GPIO_PUBPAR_UCTS1_UCTS1          (0x40)
#define MCF_GPIO_PUBPAR_UCTS1_SYNCA          (0x80)
#define MCF_GPIO_PUBPAR_UCTS1_URXD2          (0xC0)

/* Bit definitions and macros for MCF_GPIO_PORTUC */
#define MCF_GPIO_PORTUC_PORTUC0              (0x1)
#define MCF_GPIO_PORTUC_PORTUC1              (0x2)
#define MCF_GPIO_PORTUC_PORTUC2              (0x4)
#define MCF_GPIO_PORTUC_PORTUC3              (0x8)

/* Bit definitions and macros for MCF_GPIO_DDRUC */
#define MCF_GPIO_DDRUC_DDRUC0                (0x1)
#define MCF_GPIO_DDRUC_DDRUC1                (0x2)
#define MCF_GPIO_DDRUC_DDRUC2                (0x4)
#define MCF_GPIO_DDRUC_DDRUC3                (0x8)

/* Bit definitions and macros for MCF_GPIO_SETUC */
#define MCF_GPIO_SETUC_SETUC0                (0x1)
#define MCF_GPIO_SETUC_SETUC1                (0x2)
#define MCF_GPIO_SETUC_SETUC2                (0x4)
#define MCF_GPIO_SETUC_SETUC3                (0x8)

/* Bit definitions and macros for MCF_GPIO_CLRUC */
#define MCF_GPIO_CLRUC_CLRUC0                (0x1)
#define MCF_GPIO_CLRUC_CLRUC1                (0x2)
#define MCF_GPIO_CLRUC_CLRUC2                (0x4)
#define MCF_GPIO_CLRUC_CLRUC3                (0x8)

/* Bit definitions and macros for MCF_GPIO_PUCPAR */
#define MCF_GPIO_PUCPAR_PUCPAR0(x)           (((x)&0x3)<<0)
#define MCF_GPIO_PUCPAR_UTXD2_GPIO           (0)
#define MCF_GPIO_PUCPAR_UTXD2_UTXD2          (0x1)
#define MCF_GPIO_PUCPAR_UTXD2_CANTX          (0x2)
#define MCF_GPIO_PUCPAR_PUCPAR1(x)           (((x)&0x3)<<0x2)
#define MCF_GPIO_PUCPAR_URXD2_GPIO           (0)
#define MCF_GPIO_PUCPAR_URXD2_URXD2          (0x4)
#define MCF_GPIO_PUCPAR_URXD2_CANRX          (0x8)
#define MCF_GPIO_PUCPAR_PUCPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PUCPAR_URTS2_GPIO           (0)
#define MCF_GPIO_PUCPAR_URTS2_URTS2          (0x10)
#define MCF_GPIO_PUCPAR_URTS2_SDA1           (0x20)
#define MCF_GPIO_PUCPAR_URTS2_USB_VBUSDIS    (0x30)
#define MCF_GPIO_PUCPAR_PUCPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_PUCPAR_UCTS2_GPIO           (0)
#define MCF_GPIO_PUCPAR_UCTS2_UCTS2          (0x40)
#define MCF_GPIO_PUCPAR_UCTS2_SCL1           (0x80)
#define MCF_GPIO_PUCPAR_UCTS2_USB_VBUSCHG    (0xC0)

/* Bit definitions and macros for MCF_GPIO_PORTDD */
#define MCF_GPIO_PORTDD_PORTDD0              (0x1)
#define MCF_GPIO_PORTDD_PORTDD1              (0x2)
#define MCF_GPIO_PORTDD_PORTDD2              (0x4)
#define MCF_GPIO_PORTDD_PORTDD3              (0x8)
#define MCF_GPIO_PORTDD_PORTDD4              (0x10)
#define MCF_GPIO_PORTDD_PORTDD5              (0x20)
#define MCF_GPIO_PORTDD_PORTDD6              (0x40)
#define MCF_GPIO_PORTDD_PORTDD7              (0x80)

/* Bit definitions and macros for MCF_GPIO_DDRDD */
#define MCF_GPIO_DDRDD_DDRDD0                (0x1)
#define MCF_GPIO_DDRDD_DDRDD1                (0x2)
#define MCF_GPIO_DDRDD_DDRDD2                (0x4)
#define MCF_GPIO_DDRDD_DDRDD3                (0x8)
#define MCF_GPIO_DDRDD_DDRDD4                (0x10)
#define MCF_GPIO_DDRDD_DDRDD5                (0x20)
#define MCF_GPIO_DDRDD_DDRDD6                (0x40)
#define MCF_GPIO_DDRDD_DDRDD7                (0x80)

/* Bit definitions and macros for MCF_GPIO_SETDD */
#define MCF_GPIO_SETDD_SETDD0                (0x1)
#define MCF_GPIO_SETDD_SETDD1                (0x2)
#define MCF_GPIO_SETDD_SETDD2                (0x4)
#define MCF_GPIO_SETDD_SETDD3                (0x8)
#define MCF_GPIO_SETDD_SETDD4                (0x10)
#define MCF_GPIO_SETDD_SETDD5                (0x20)
#define MCF_GPIO_SETDD_SETDD6                (0x40)
#define MCF_GPIO_SETDD_SETDD7                (0x80)

/* Bit definitions and macros for MCF_GPIO_CLRDD */
#define MCF_GPIO_CLRDD_CLRDD0                (0x1)
#define MCF_GPIO_CLRDD_CLRDD1                (0x2)
#define MCF_GPIO_CLRDD_CLRDD2                (0x4)
#define MCF_GPIO_CLRDD_CLRDD3                (0x8)
#define MCF_GPIO_CLRDD_CLRDD4                (0x10)
#define MCF_GPIO_CLRDD_CLRDD5                (0x20)
#define MCF_GPIO_CLRDD_CLRDD6                (0x40)
#define MCF_GPIO_CLRDD_CLRDD7                (0x80)

/* Bit definitions and macros for MCF_GPIO_PDDPAR */
#define MCF_GPIO_PDDPAR_PDDPAR0              (0x1)
#define MCF_GPIO_PDDPAR_PST0_GPIO            (0)
#define MCF_GPIO_PDDPAR_PST0_PST0            (0x1)
#define MCF_GPIO_PDDPAR_PDDPAR1              (0x2)
#define MCF_GPIO_PDDPAR_PST1_GPIO            (0)
#define MCF_GPIO_PDDPAR_PST1_PST1            (0x2)
#define MCF_GPIO_PDDPAR_PDDPAR2              (0x4)
#define MCF_GPIO_PDDPAR_PST2_GPIO            (0)
#define MCF_GPIO_PDDPAR_PST2_PST2            (0x4)
#define MCF_GPIO_PDDPAR_PDDPAR3              (0x8)
#define MCF_GPIO_PDDPAR_PST3_GPIO            (0)
#define MCF_GPIO_PDDPAR_PST3_PST3            (0x8)
#define MCF_GPIO_PDDPAR_PDDPAR4              (0x10)
#define MCF_GPIO_PDDPAR_DDATA0_GPIO          (0)
#define MCF_GPIO_PDDPAR_DDATA0_DDATA0        (0x10)
#define MCF_GPIO_PDDPAR_PDDPAR5              (0x20)
#define MCF_GPIO_PDDPAR_DDATA1_GPIO          (0)
#define MCF_GPIO_PDDPAR_DDATA1_DDATA1        (0x20)
#define MCF_GPIO_PDDPAR_PDDPAR6              (0x40)
#define MCF_GPIO_PDDPAR_DDATA2_GPIO          (0)
#define MCF_GPIO_PDDPAR_DDATA2_DDATA2        (0x40)
#define MCF_GPIO_PDDPAR_PDDPAR7              (0x80)
#define MCF_GPIO_PDDPAR_DDATA3_GPIO          (0)
#define MCF_GPIO_PDDPAR_DDATA3_DDATA3        (0x80)


