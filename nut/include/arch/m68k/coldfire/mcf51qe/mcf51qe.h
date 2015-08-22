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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, ICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

#ifndef _ARCH_M68K_H_
#error "Do not include this file directly. Use arch/m68k.h instead!"
#endif

#include "mcf51qe_gpio.h"
#include "mcf51qe_ics.h"
#include "mcf51qe_sci.h"
#include "mcf51qe_tpm.h"
#include "mcf51qe_spi.h"
#include "mcf51qe_iic.h"
#include "mcf51qe_adc.h"
#include "mcf51qe_iic.h"

/**************** registers I/O map ****************/

/* Tip for register initialization in the user code:  const byte NVFTRIM_INIT(0x000003FE = <NVFTRIM_INITVAL>)) */
#define MCF_NVFTRIM (*(const uint8_t *)(0x000003FE))
#define MCF_NVFTRIM_FTRIM             0x01


/* Tip for register initialization in the user code:  const byte NVICM_INIT(0x000003FF = <NVICM_INITVAL>)) */
#define MCF_NVICM (*(const uint8_t *)(0x000003FF))


/*** NVBACKKEY0-7 - Backdoor Comparison Key 0-7 0x00000400 ***/
#define MCF_NVBACKKEY(x)			(*(volatile uint8_t*)(0x00000400 + ((x) * 0x1)))


/*** NVPROT - Nonvolatile Flash Protection Register)) 0x0000040D ***/
#define MCF_NVPROT (*(const uint8_t *)0x0000040D)

#define MCF_NVPROT_FPOPEN                   0x01
#define MCF_NVPROT_FPS                      0xFE
#define MCF_NVPROT_FPS_BITNUM               0x01


/*** NVOPT - Nonvolatile Flash Options Register)) 0x0000040F ***/
#define MCF_NVOPT (*(const uint8_t *)0x0000040F)

#define MCF_NVOPT_SEC0                      0x01
#define MCF_NVOPT_SEC1                      0x02
#define MCF_NVOPT_KEYEN0                    0x40
#define MCF_NVOPT_KEYEN1                    0x80
#define MCF_NVOPT_SEC                       0x03
#define MCF_NVOPT_SEC_BITNUM                0x00
#define MCF_NVOPT_KEYEN                     0xC0
#define MCF_NVOPT_KEYEN_BITNUM              0x06


/*** RGPIO_DIR - RGPIO Data Direction Register)) 0x00C00000 ***/
#define MCF_RGPIO_DIR (*(volatile uint16_t *)(0x00C00000))


/*** RGPIO_DATA - RGPIO Data Register)) 0x00C00002 ***/
#define MCF_RGPIO_DATA (*(volatile uint16_t *)(0x00C00002))


/*** RGPIO_ENB - RGPIO Pin Enable Register)) 0x00C00004 ***/
#define MCF_RGPIO_ENB (*(volatile uint16_t *)(0x00C00004))


/*** RGPIO_CLR - RGPIO Clear Data Register)) 0x00C00006 ***/
#define MCF_RGPIO_CLR (*(volatile uint16_t *)(0x00C00006))


/*** RGPIO_SET - RGPIO Set Data Register)) 0x00C0000A ***/
#define MCF_RGPIO_SET (*(volatile uint16_t *)(0x00C0000A))


/*** RGPIO_TOG - RGPIO Toggle Data Register)) 0x00C0000E ***/
#define MCF_RGPIO_TOG (*(volatile uint16_t *)(0x00C0000E))


/*** KBI1SC - KBI1 Status and Control Register)) 0xFFFF800C ***/
#define MCF_KBI1SC (*(volatile uint8_t *)(0xFFFF800C))

#define MCF_KBI1SC_KBIMOD                   0x01
#define MCF_KBI1SC_KBIE                     0x02
#define MCF_KBI1SC_KBACK                    0x04
#define MCF_KBI1SC_KBF                      0x08


/*** KBI1PE - KBI1 Pin Enable Register)) 0xFFFF800D ***/
#define MCF_KBI1PE (*(volatile uint8_t *)(0xFFFF800D))


/*** KBI1ES - KBI1 Edge Select Register)) 0xFFFF800E ***/
#define MCF_KBI1ES (*(volatile uint8_t *)(0xFFFF800E))


/*** IRQSC - Interrupt request status and control register)) 0xFFFF800F ***/
#define MCF_IRQSC (*(volatile uint8_t *)(0xFFFF800F))

#define MCF_IRQSC_IRQMOD                    0x01
#define MCF_IRQSC_IRQIE                     0x02
#define MCF_IRQSC_IRQACK                    0x04
#define MCF_IRQSC_IRQF                      0x08
#define MCF_IRQSC_IRQPE                     0x10
#define MCF_IRQSC_IRQEDG                    0x20
#define MCF_IRQSC_IRQPDD                    0x40


/*** APCTL1 - Pin Control 1 Register)) 0xFFFF8017 ***/
#define MCF_APCTL1 (*(volatile uint8_t *)(0xFFFF8017))

#define MCF_APCTL1_ADPC0                    0x01
#define MCF_APCTL1_ADPC1                    0x02
#define MCF_APCTL1_ADPC2                    0x04
#define MCF_APCTL1_ADPC3                    0x08
#define MCF_APCTL1_ADPC4                    0x10
#define MCF_APCTL1_ADPC5                    0x20
#define MCF_APCTL1_ADPC6                    0x40
#define MCF_APCTL1_ADPC7                    0x80


/*** APCTL2 - Pin Control 2 Register)) 0xFFFF8018 ***/
#define MCF_APCTL2 (*(volatile uint8_t *)(0xFFFF8018))

#define MCF_APCTL2_ADPC8                    0x01
#define MCF_APCTL2_ADPC9                    0x02
#define MCF_APCTL2_ADPC10                   0x04
#define MCF_APCTL2_ADPC11                   0x08
#define MCF_APCTL2_ADPC12                   0x10
#define MCF_APCTL2_ADPC13                   0x20
#define MCF_APCTL2_ADPC14                   0x40
#define MCF_APCTL2_ADPC15                   0x80


/*** APCTL3 - Pin Control 3 Register)) 0xFFFF8019 ***/
#define MCF_APCTL3 (*(volatile uint8_t *)(0xFFFF8019))

#define MCF_APCTL3_ADPC16                   0x01
#define MCF_APCTL3_ADPC17                   0x02
#define MCF_APCTL3_ADPC18                   0x04
#define MCF_APCTL3_ADPC19                   0x08
#define MCF_APCTL3_ADPC20                   0x10
#define MCF_APCTL3_ADPC21                   0x20
#define MCF_APCTL3_ADPC22                   0x40
#define MCF_APCTL3_ADPC23                   0x80


/*** ACMP1SC - ACMP1 Status and Control Register)) 0xFFFF801A ***/
#define MCF_ACMP1SC (*(volatile uint8_t *)(0xFFFF801A))

#define MCF_ACMP1SC_ACMOD0                  0x01
#define MCF_ACMP1SC_ACMOD1                  0x02
#define MCF_ACMP1SC_ACOPE                   0x04
#define MCF_ACMP1SC_ACO                     0x08
#define MCF_ACMP1SC_ACIE                    0x10
#define MCF_ACMP1SC_ACF                     0x20
#define MCF_ACMP1SC_ACBGS                   0x40
#define MCF_ACMP1SC_ACME                    0x80
#define MCF_ACMP1SC_ACMOD                   0x03
#define MCF_ACMP1SC_ACMOD_BITNUM            0x00


/*** ACMP2SC - ACMP2 Status and Control Register)) 0xFFFF801B ***/
#define MCF_ACMP2SC (*(volatile uint8_t *)(0xFFFF801B))

#define MCF_ACMP2SC_ACMOD0                  0x01
#define MCF_ACMP2SC_ACMOD1                  0x02
#define MCF_ACMP2SC_ACOPE                   0x04
#define MCF_ACMP2SC_ACO                     0x08
#define MCF_ACMP2SC_ACIE                    0x10
#define MCF_ACMP2SC_ACF                     0x20
#define MCF_ACMP2SC_ACBGS                   0x40
#define MCF_ACMP2SC_ACME                    0x80
#define MCF_ACMP2SC_ACMOD                   0x03
#define MCF_ACMP2SC_ACMOD_BITNUM            0x00


/*** KBI2SC - KBI2 Status and Control Register)) 0xFFFF803C ***/
#define MCF_KBI2SC (*(volatile uint8_t *)(0xFFFF803C))

#define MCF_KBI2SC_KBIMOD                   0x01
#define MCF_KBI2SC_KBIE                     0x02
#define MCF_KBI2SC_KBACK                    0x04
#define MCF_KBI2SC_KBF                      0x08


/*** KBI2PE - KBI2 Pin Enable Register)) 0xFFFF803D ***/
#define MCF_KBI2PE (*(volatile uint8_t *)(0xFFFF803D))


/*** KBI2ES - KBI2 Edge Select Register)) 0xFFFF803E ***/
#define MCF_KBI2ES (*(volatile uint8_t *)(0xFFFF803E))


/*** SRS - System Reset Status Register)) 0xFFFF9800 ***/
#define MCF_SRS (*(volatile uint8_t *)(0xFFFF9800))

#define MCF_SRS_LVD                         0x02
#define MCF_SRS_ILAD                        0x08
#define MCF_SRS_ILOP                        0x10
#define MCF_SRS_COP                         0x20
#define MCF_SRS_PIN                         0x40
#define MCF_SRS_POR                         0x80


/*** SOPT1 - System Options Register 1)) 0xFFFF9802 ***/
#define MCF_SOPT1 (*(volatile uint8_t *)(0xFFFF9802))

#define MCF_SOPT1_RSTPE                     0x01
#define MCF_SOPT1_BKGDPE                    0x02
#define MCF_SOPT1_RSTOPE                    0x04
#define MCF_SOPT1_WAITE                     0x10
#define MCF_SOPT1_STOPE                     0x20
#define MCF_SOPT1_COPT                      0x40
#define MCF_SOPT1_COPE                      0x80


/*** SOPT2 - System Options Register 2)) 0xFFFF9803 ***/
#define MCF_SOPT2 (*(volatile uint8_t *)(0xFFFF9803))

#define MCF_SOPT2_ACIC1                     0x01
#define MCF_SOPT2_IIC1PS                    0x02
#define MCF_SOPT2_ACIC2                     0x04
#define MCF_SOPT2_SPI1PS                    0x08
#define MCF_SOPT2_COPCLKS                   0x80


/*** SDID - System Device Identification Register)) 0xFFFF9806 ***/
#define MCF_SDID (*(volatile uint16_t *)(0xFFFF9806))

#define MCF_SDID_ID                         0x0FFF


/*** SPMSC1 - System Power Management Status and Control 1 Register)) 0xFFFF9808 ***/
#define MCF_SPMSC1 (*(volatile uint8_t *)(0xFFFF9808))

#define MCF_SPMSC1_BGBE                     0x01
#define MCF_SPMSC1_LVDE                     0x04
#define MCF_SPMSC1_LVDSE                    0x08
#define MCF_SPMSC1_LVDRE                    0x10
#define MCF_SPMSC1_LVDIE                    0x20
#define MCF_SPMSC1_LVDACK                   0x40
#define MCF_SPMSC1_LVDF                     0x80


/*** SPMSC2 - System Power Management Status and Control 2 Register)) 0xFFFF9809 ***/
#define MCF_SPMSC2 (*(volatile uint8_t *)(0xFFFF9809))

#define MCF_SPMSC2_PPDC                     0x01
#define MCF_SPMSC2_PPDE                     0x02
#define MCF_SPMSC2_PPDACK                   0x04
#define MCF_SPMSC2_PPDF                     0x08
#define MCF_SPMSC2_LPWUI                    0x20
#define MCF_SPMSC2_LPRS                     0x40
#define MCF_SPMSC2_LPR                      0x80


/*** SPMSC3 - System Power Management Status and Control 3 Register)) 0xFFFF980B ***/
#define MCF_SPMSC3 (*(volatile uint8_t *)(0xFFFF980B))

#define MCF_SPMSC3_LVWIE                    0x08
#define MCF_SPMSC3_LVWV                     0x10
#define MCF_SPMSC3_LVDV                     0x20
#define MCF_SPMSC3_LVWACK                   0x40
#define MCF_SPMSC3_LVWF                     0x80


/*** SCGC1 - System Clock Gating Control 1 Register)) 0xFFFF980E ***/
#define MCF_SCGC1 (*(volatile uint8_t *)(0xFFFF980E))

#define MCF_SCGC1_SCI1                      0x01
#define MCF_SCGC1_SCI2                      0x02
#define MCF_SCGC1_IIC1                      0x04
#define MCF_SCGC1_IIC2                      0x08
#define MCF_SCGC1_ADC                       0x10
#define MCF_SCGC1_TPM1                      0x20
#define MCF_SCGC1_TPM2                      0x40
#define MCF_SCGC1_TPM3                      0x80
#define MCF_SCGC1_SCI_1                     0x03
#define MCF_SCGC1_SCI_1_BITNUM              0x00
#define MCF_SCGC1_IIC_1                     0x0C
#define MCF_SCGC1_IIC_1_BITNUM              0x02
#define MCF_SCGC1_TPM_1                     0xE0
#define MCF_SCGC1_TPM_1_BITNUM              0x05


/*** SCGC2 - System Clock Gating Control 2 Register)) 0xFFFF980F ***/
#define MCF_SCGC2 (*(volatile uint8_t *)(0xFFFF980F))

#define MCF_SCGC2_SPI1                      0x01
#define MCF_SCGC2_SPI2                      0x02
#define MCF_SCGC2_RTC                       0x04
#define MCF_SCGC2_ACMP                      0x08
#define MCF_SCGC2_KBI                       0x10
#define MCF_SCGC2_IRQ                       0x20
#define MCF_SCGC2_FLS                       0x40
#define MCF_SCGC2_SPI_1                     0x03
#define MCF_SCGC2_SPI_1_BITNUM              0x00


/*** FCDIV - FLASH Clock Divider Register)) 0xFFFF9820 ***/
#define MCF_FCDIV (*(volatile uint8_t *)(0xFFFF9820))

#define MCF_FCDIV_FDIV0                     0x01
#define MCF_FCDIV_FDIV1                     0x02
#define MCF_FCDIV_FDIV2                     0x04
#define MCF_FCDIV_FDIV3                     0x08
#define MCF_FCDIV_FDIV4                     0x10
#define MCF_FCDIV_FDIV5                     0x20
#define MCF_FCDIV_PRDIV8                    0x40
#define MCF_FCDIV_FDIVLD                    0x80
#define MCF_FCDIV_FDIV                      0x3F
#define MCF_FCDIV_FDIV_BITNUM               0x00


/*** FOPT - Flash Options Register)) 0xFFFF9821 ***/
#define MCF_FOPT (*(volatile uint8_t *)(0xFFFF9821))

#define MCF_FOPT_SEC0                       0x01
#define MCF_FOPT_SEC1                       0x02
#define MCF_FOPT_KEYEN0                     0x40
#define MCF_FOPT_KEYEN1                     0x80
#define MCF_FOPT_SEC                        0x03
#define MCF_FOPT_SEC_BITNUM                 0x00
#define MCF_FOPT_KEYEN                      0xC0
#define MCF_FOPT_KEYEN_BITNUM               0x06


/*** FCNFG - Flash Configuration Register)) 0xFFFF9823 ***/
#define MCF_FCNFG (*(volatile uint8_t *)(0xFFFF9823))

#define MCF_MCF_FCNFG_KEYACC                    0x20


/*** FPROT - Flash Protection Register)) 0xFFFF9824 ***/
#define MCF_FPROT (*(volatile uint8_t *)(0xFFFF9824))

#define MCF_FPROT_FPOPEN                    0x01
#define MCF_FPROT_FPS0                      0x02
#define MCF_FPROT_FPS1                      0x04
#define MCF_FPROT_FPS2                      0x08
#define MCF_FPROT_FPS3                      0x10
#define MCF_FPROT_FPS4                      0x20
#define MCF_FPROT_FPS5                      0x40
#define MCF_FPROT_FPS6                      0x80
#define MCF_FPROT_FPS                       0xFE
#define MCF_FPROT_FPS_BITNUM                0x01


/*** FSTAT - Flash Status Register)) 0xFFFF9825 ***/
#define MCF_FSTAT (*(volatile uint8_t *)(0xFFFF9825))

#define MCF_FSTAT_FBLANK                    0x04
#define MCF_FSTAT_FACCERR                   0x10
#define MCF_FSTAT_FPVIOL                    0x20
#define MCF_FSTAT_FCCF                      0x40
#define MCF_FSTAT_FCBEF                     0x80


/*** FCMD - Flash Command Register)) 0xFFFF9826 ***/
#define MCF_FCMD (*(volatile uint8_t *)(0xFFFF9826))

#define MCF_FCMD_FCMD0                      0x01
#define MCF_FCMD_FCMD1                      0x02
#define MCF_FCMD_FCMD2                      0x04
#define MCF_FCMD_FCMD3                      0x08
#define MCF_FCMD_FCMD4                      0x10
#define MCF_FCMD_FCMD5                      0x20
#define MCF_FCMD_FCMD6                      0x40
#define MCF_FCMD_FCMD                       0x7F
#define MCF_FCMD_FCMD_BITNUM                0x00


/*** RTCSC - RTC Status and Control Register)) 0xFFFF9830 ***/
#define MCF_RTCSC (*(volatile uint8_t *)(0xFFFF9830))

#define MCF_RTCSC_RTCPS0                    0x01
#define MCF_RTCSC_RTCPS1                    0x02
#define MCF_RTCSC_RTCPS2                    0x04
#define MCF_RTCSC_RTCPS3                    0x08
#define MCF_RTCSC_RTIE                      0x10
#define MCF_RTCSC_RTCLKS0                   0x20
#define MCF_RTCSC_RTCLKS1                   0x40
#define MCF_RTCSC_RTIF                      0x80
#define MCF_RTCSC_RTCPS                     0x0F
#define MCF_RTCSC_RTCPS_BITNUM              0x00
#define MCF_RTCSC_RTCLKS                    0x60
#define MCF_RTCSC_RTCLKS_BITNUM             0x05


/*** RTCCNT - RTC Counter Register)) 0xFFFF9831 ***/
#define MCF_RTCCNT (*(volatile uint8_t *)(0xFFFF9831))


/*** RTCMOD - RTC Modulo Register)) 0xFFFF9832 ***/
#define MCF_RTCMOD (*(volatile uint8_t *)(0xFFFF9832))


/*** INTC_FRC - INTC Force Interrupt Register)) 0xFFFFFFD3 ***/
#define MCF_INTC_FRC (*(volatile uint8_t *)(0xFFFFFFD3))

#define MCF_INTC_FRC_LVL7                   0x01
#define MCF_INTC_FRC_LVL6                   0x02
#define MCF_INTC_FRC_LVL5                   0x04
#define MCF_INTC_FRC_LVL4                   0x08
#define MCF_INTC_FRC_LVL3                   0x10
#define MCF_INTC_FRC_LVL2                   0x20
#define MCF_INTC_FRC_LVL1                   0x40


/*** INTC_PL6P7 - INTC Programmable Level 6, Priority 7 Register)) 0xFFFFFFD8 ***/
#define MCF_INTC_PL6P7 (*(volatile uint8_t *)(0xFFFFFFD8))

#define MCF_INTC_PL6P7_REQN0                0x01
#define MCF_INTC_PL6P7_REQN1                0x02
#define MCF_INTC_PL6P7_REQN2                0x04
#define MCF_INTC_PL6P7_REQN3                0x08
#define MCF_INTC_PL6P7_REQN4                0x10
#define MCF_INTC_PL6P7_REQN                 0x1F
#define MCF_INTC_PL6P7_REQN_BITNUM          0x00


/*** INTC_PL6P6 - INTC Programmable Level 6, Priority 6 Register)) 0xFFFFFFD9 ***/
#define MCF_INTC_PL6P6 (*(volatile uint8_t *)(0xFFFFFFD9))

#define MCF_INTC_PL6P6_REQN0                0x01
#define MCF_INTC_PL6P6_REQN1                0x02
#define MCF_INTC_PL6P6_REQN2                0x04
#define MCF_INTC_PL6P6_REQN3                0x08
#define MCF_INTC_PL6P6_REQN4                0x10
#define MCF_INTC_PL6P6_REQN                 0x1F
#define MCF_INTC_PL6P6_REQN_BITNUM          0x00


/*** INTC_WCR - INTC Wake-up Control Register)) 0xFFFFFFDB ***/
#define MCF_INTC_WCR (*(volatile uint8_t *)(0xFFFFFFDB))

#define MCF_INTC_WCR0	                    0x01
#define MCF_INTC_WCR1                       0x02
#define MCF_INTC_WCR2                       0x04
#define MCF_INTC_WCR_ENB                    0x80
#define MCF_INTC_WCR_BITNUM                 0x00


/*** INTC_SFRC - INTC Set Interrupt Force Register)) 0xFFFFFFDE ***/
#define MCF_INTC_SFRC (*(volatile uint8_t *)(0xFFFFFFDE))

#define MCF_INTC_SFRC_SET0                  0x01
#define MCF_INTC_SFRC_SET1                  0x02
#define MCF_INTC_SFRC_SET2                  0x04
#define MCF_INTC_SFRC_SET3                  0x08
#define MCF_INTC_SFRC_SET4                  0x10
#define MCF_INTC_SFRC_SET5                  0x20
#define MCF_INTC_SFRC_SET                   0x3F
#define MCF_INTC_SFRC_SET_BITNUM            0x00


/*** INTC_CFRC - INTC Clear Interrupt Force Register)) 0xFFFFFFDF ***/
#define MCF_INTC_CFRC (*(volatile uint8_t *)(0xFFFFFFDF))

#define MCF_INTC_CFRC_CLR0                  0x01
#define MCF_INTC_CFRC_CLR1                  0x02
#define MCF_INTC_CFRC_CLR2                  0x04
#define MCF_INTC_CFRC_CLR3                  0x08
#define MCF_INTC_CFRC_CLR4                  0x10
#define MCF_INTC_CFRC_CLR5                  0x20
#define MCF_INTC_CFRC_CLR                   0x3F
#define MCF_INTC_CFRC_CLR_BITNUM            0x00


/*** INTC_SWIACK - INTC Software IACK Register)) 0xFFFFFFE0 ***/
#define MCF_INTC_SWIACK (*(volatile uint8_t *)(0xFFFFFFE0))

#define MCF_INTC_SWIACK_VECN0               0x01
#define MCF_INTC_SWIACK_VECN1               0x02
#define MCF_INTC_SWIACK_VECN2               0x04
#define MCF_INTC_SWIACK_VECN3               0x08
#define MCF_INTC_SWIACK_VECN4               0x10
#define MCF_INTC_SWIACK_VECN5               0x20
#define MCF_INTC_SWIACK_VECN6               0x40
#define MCF_INTC_SWIACK_VECN                0x7F
#define MCF_INTC_SWIACK_VECN_BITNUM         0x00


/*** INTC_LVL1IACK - INTC Level 1-7 IACK Register)) 0xFFFFFFE4 ***/
#define MCF_INTC_LVL_IACK(x)				(*(volatile uint8_t*)(0xFFFFFFE4 + ((x - 1) * 0x4)))
#define MCF_INTC_LVL_IACK_VECN              0x7F


/* Flash commands */
#define mBlank                          0x05
#define mBurstProg                      0x25
#define mByteProg                       0x20
#define mMassErase                      0x41
#define mPageErase                      0x40

