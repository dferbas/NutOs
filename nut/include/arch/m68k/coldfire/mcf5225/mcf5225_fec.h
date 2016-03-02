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

/* Coldfire C Header File
 * Copyright Freescale Semiconductor Inc
 * All rights reserved.
 *
 * 2008/05/23 Revision: 0.5
 *
 * (c) Copyright UNIS, a.s. 1997-2008
 * UNIS, a.s.
 * Jundrovska 33
 * 624 00 Brno
 * Czech Republic
 * http      : www.processorexpert.com
 * mail      : info@processorexpert.com
 */

#ifndef _ARCH_M68K_H_
#error "Do not include this file directly. Use arch/m68k.h instead!"
#endif


/*********************************************************************
*
* Fast Ethernet Controller(FEC)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_FEC_EIR                          (*(volatile uint32_t*)(0x40001004))
#define MCF_FEC_EIMR                         (*(volatile uint32_t*)(0x40001008))
#define MCF_FEC_RDAR                         (*(volatile uint32_t*)(0x40001010))
#define MCF_FEC_TDAR                         (*(volatile uint32_t*)(0x40001014))
#define MCF_FEC_ECR                          (*(volatile uint32_t*)(0x40001024))
#define MCF_FEC_MMFR                         (*(volatile uint32_t*)(0x40001040))
#define MCF_FEC_MSCR                         (*(volatile uint32_t*)(0x40001044))
#define MCF_FEC_MIBC                         (*(volatile uint32_t*)(0x40001064))
#define MCF_FEC_RCR                          (*(volatile uint32_t*)(0x40001084))
#define MCF_FEC_TCR                          (*(volatile uint32_t*)(0x400010C4))
#define MCF_FEC_PALR                         (*(volatile uint32_t*)(0x400010E4))
#define MCF_FEC_PAUR                         (*(volatile uint32_t*)(0x400010E8))
#define MCF_FEC_OPD                          (*(volatile uint32_t*)(0x400010EC))
#define MCF_FEC_IAUR                         (*(volatile uint32_t*)(0x40001118))
#define MCF_FEC_IALR                         (*(volatile uint32_t*)(0x4000111C))
#define MCF_FEC_GAUR                         (*(volatile uint32_t*)(0x40001120))
#define MCF_FEC_GALR                         (*(volatile uint32_t*)(0x40001124))
#define MCF_FEC_TFWR                         (*(volatile uint32_t*)(0x40001144))
#define MCF_FEC_FRBR                         (*(volatile uint32_t*)(0x4000114C))
#define MCF_FEC_FRSR                         (*(volatile uint32_t*)(0x40001150))
#define MCF_FEC_ERDSR                        (*(volatile uint32_t*)(0x40001180))
#define MCF_FEC_ETSDR                        (*(volatile uint32_t*)(0x40001184))
#define MCF_FEC_EMRBR                        (*(volatile uint32_t*)(0x40001188))
#define MCF_FEC_RMON_T_DROP                  (*(volatile uint32_t*)(0x40001200))
#define MCF_FEC_RMON_T_PACKETS               (*(volatile uint32_t*)(0x40001204))
#define MCF_FEC_RMON_T_BC_PKT                (*(volatile uint32_t*)(0x40001208))
#define MCF_FEC_RMON_T_MC_PKT                (*(volatile uint32_t*)(0x4000120C))
#define MCF_FEC_RMON_T_CRC_ALIGN             (*(volatile uint32_t*)(0x40001210))
#define MCF_FEC_RMON_T_UNDERSIZE             (*(volatile uint32_t*)(0x40001214))
#define MCF_FEC_RMON_T_OVERSIZE              (*(volatile uint32_t*)(0x40001218))
#define MCF_FEC_RMON_T_FRAG                  (*(volatile uint32_t*)(0x4000121C))
#define MCF_FEC_RMON_T_JAB                   (*(volatile uint32_t*)(0x40001220))
#define MCF_FEC_RMON_T_COL                   (*(volatile uint32_t*)(0x40001224))
#define MCF_FEC_RMON_T_P64                   (*(volatile uint32_t*)(0x40001228))
#define MCF_FEC_RMON_T_P65TO127              (*(volatile uint32_t*)(0x4000122C))
#define MCF_FEC_RMON_T_P128TO255             (*(volatile uint32_t*)(0x40001230))
#define MCF_FEC_RMON_T_P256TO511             (*(volatile uint32_t*)(0x40001234))
#define MCF_FEC_RMON_T_P512TO1023            (*(volatile uint32_t*)(0x40001238))
#define MCF_FEC_RMON_T_P1024TO2047           (*(volatile uint32_t*)(0x4000123C))
#define MCF_FEC_RMON_T_P_GTE2048             (*(volatile uint32_t*)(0x40001240))
#define MCF_FEC_RMON_T_OCTETS                (*(volatile uint32_t*)(0x40001244))
#define MCF_FEC_IEEE_T_DROP                  (*(volatile uint32_t*)(0x40001248))
#define MCF_FEC_IEEE_T_FRAME_OK              (*(volatile uint32_t*)(0x4000124C))
#define MCF_FEC_IEEE_T_1COL                  (*(volatile uint32_t*)(0x40001250))
#define MCF_FEC_IEEE_T_MCOL                  (*(volatile uint32_t*)(0x40001254))
#define MCF_FEC_IEEE_T_DEF                   (*(volatile uint32_t*)(0x40001258))
#define MCF_FEC_IEEE_T_LCOL                  (*(volatile uint32_t*)(0x4000125C))
#define MCF_FEC_IEEE_T_EXCOL                 (*(volatile uint32_t*)(0x40001260))
#define MCF_FEC_IEEE_T_MACERR                (*(volatile uint32_t*)(0x40001264))
#define MCF_FEC_IEEE_T_CSERR                 (*(volatile uint32_t*)(0x40001268))
#define MCF_FEC_IEEE_T_SQE                   (*(volatile uint32_t*)(0x4000126C))
#define MCF_FEC_IEEE_T_FDXFC                 (*(volatile uint32_t*)(0x40001270))
#define MCF_FEC_IEEE_T_OCTETS_OK             (*(volatile uint32_t*)(0x40001274))
#define MCF_FEC_RMON_R_PACKETS               (*(volatile uint32_t*)(0x40001284))
#define MCF_FEC_RMON_R_BC_PKT                (*(volatile uint32_t*)(0x40001288))
#define MCF_FEC_RMON_R_MC_PKT                (*(volatile uint32_t*)(0x4000128C))
#define MCF_FEC_RMON_R_CRC_ALIGN             (*(volatile uint32_t*)(0x40001290))
#define MCF_FEC_RMON_R_UNDERSIZE             (*(volatile uint32_t*)(0x40001294))
#define MCF_FEC_RMON_R_OVERSIZE              (*(volatile uint32_t*)(0x40001298))
#define MCF_FEC_RMON_R_FRAG                  (*(volatile uint32_t*)(0x4000129C))
#define MCF_FEC_RMON_R_JAB                   (*(volatile uint32_t*)(0x400012A0))
#define MCF_FEC_RMON_R_RESVD_0               (*(volatile uint32_t*)(0x400012A4))
#define MCF_FEC_RMON_R_P64                   (*(volatile uint32_t*)(0x400012A8))
#define MCF_FEC_RMON_R_P65TO127              (*(volatile uint32_t*)(0x400012AC))
#define MCF_FEC_RMON_R_P128TO255             (*(volatile uint32_t*)(0x400012B0))
#define MCF_FEC_RMON_R_P256TO511             (*(volatile uint32_t*)(0x400012B4))
#define MCF_FEC_RMON_R_P512TO1023            (*(volatile uint32_t*)(0x400012B8))
#define MCF_FEC_RMON_R_P1024TO2047           (*(volatile uint32_t*)(0x400012BC))
#define MCF_FEC_RMON_R_P_GTE2048             (*(volatile uint32_t*)(0x400012C0))
#define MCF_FEC_RMON_R_OCTETS                (*(volatile uint32_t*)(0x400012C4))
#define MCF_FEC_IEEE_R_DROP                  (*(volatile uint32_t*)(0x400012C8))
#define MCF_FEC_IEEE_R_FRAME_OK              (*(volatile uint32_t*)(0x400012CC))
#define MCF_FEC_IEEE_R_CRC                   (*(volatile uint32_t*)(0x400012D0))
#define MCF_FEC_IEEE_R_ALIGN                 (*(volatile uint32_t*)(0x400012D4))
#define MCF_FEC_IEEE_R_MACERR                (*(volatile uint32_t*)(0x400012D8))
#define MCF_FEC_IEEE_R_FDXFC                 (*(volatile uint32_t*)(0x400012DC))
#define MCF_FEC_IEEE_R_OCTETS_OK             (*(volatile uint32_t*)(0x400012E0))



/* Bit definitions and macros for MCF_FEC_EIR */
#define MCF_FEC_EIR_UN                       (0x80000)
#define MCF_FEC_EIR_RL                       (0x100000)
#define MCF_FEC_EIR_LC                       (0x200000)
#define MCF_FEC_EIR_EBERR                    (0x400000)
#define MCF_FEC_EIR_MII                      (0x800000)
#define MCF_FEC_EIR_RXB                      (0x1000000)
#define MCF_FEC_EIR_RXF                      (0x2000000)
#define MCF_FEC_EIR_TXB                      (0x4000000)
#define MCF_FEC_EIR_TXF                      (0x8000000)
#define MCF_FEC_EIR_GRA                      (0x10000000)
#define MCF_FEC_EIR_BABT                     (0x20000000)
#define MCF_FEC_EIR_BABR                     (0x40000000)
#define MCF_FEC_EIR_HBERR                    (0x80000000)
#define MCF_FEC_EIR_CLEAR_ALL                (0xFFFFFFFF)

/* Bit definitions and macros for MCF_FEC_EIMR */
#define MCF_FEC_EIMR_UN                      (0x80000)
#define MCF_FEC_EIMR_RL                      (0x100000)
#define MCF_FEC_EIMR_LC                      (0x200000)
#define MCF_FEC_EIMR_EBERR                   (0x400000)
#define MCF_FEC_EIMR_MII                     (0x800000)
#define MCF_FEC_EIMR_RXB                     (0x1000000)
#define MCF_FEC_EIMR_RXF                     (0x2000000)
#define MCF_FEC_EIMR_TXB                     (0x4000000)
#define MCF_FEC_EIMR_TXF                     (0x8000000)
#define MCF_FEC_EIMR_GRA                     (0x10000000)
#define MCF_FEC_EIMR_BABT                    (0x20000000)
#define MCF_FEC_EIMR_BABR                    (0x40000000)
#define MCF_FEC_EIMR_HBERR                   (0x80000000)
#define MCF_FEC_EIMR_MASK_ALL                (0)
#define MCF_FEC_EIMR_UNMASK_ALL              (0xFFFFFFFF)

/* Bit definitions and macros for MCF_FEC_RDAR */
#define MCF_FEC_RDAR_R_DES_ACTIVE            (0x1000000)

/* Bit definitions and macros for MCF_FEC_TDAR */
#define MCF_FEC_TDAR_X_DES_ACTIVE            (0x1000000)

/* Bit definitions and macros for MCF_FEC_ECR */
#define MCF_FEC_ECR_RESET                    (0x1)
#define MCF_FEC_ECR_ETHER_EN                 (0x2)

/* Bit definitions and macros for MCF_FEC_MMFR */
#define MCF_FEC_MMFR_DATA(x)                 (((x)&0xFFFF)<<0)
#define MCF_FEC_MMFR_TA(x)                   (((x)&0x3)<<0x10)
#define MCF_FEC_MMFR_TA_10                   (0x20000)
#define MCF_FEC_MMFR_RA(x)                   (((x)&0x1F)<<0x12)
#define MCF_FEC_MMFR_PA(x)                   (((x)&0x1F)<<0x17)
#define MCF_FEC_MMFR_OP(x)                   (((x)&0x3)<<0x1C)
#define MCF_FEC_MMFR_OP_READ                 (0x20000000)
#define MCF_FEC_MMFR_OP_WRITE                (0x10000000)
#define MCF_FEC_MMFR_ST(x)                   (((x)&0x3)<<0x1E)
#define MCF_FEC_MMFR_ST_01                   (0x40000000)

/* Bit definitions and macros for MCF_FEC_MSCR */
#define MCF_FEC_MSCR_MII_SPEED(x)            (((x)&0x3F)<<0x1)
#define MCF_FEC_MSCR_DIS_PREAMBLE            (0x80)

/* Bit definitions and macros for MCF_FEC_MIBC */
#define MCF_FEC_MIBC_MIB_IDLE                (0x40000000)
#define MCF_FEC_MIBC_MIB_DISABLE             (0x80000000)

/* Bit definitions and macros for MCF_FEC_RCR */
#define MCF_FEC_RCR_LOOP                     (0x1)
#define MCF_FEC_RCR_DRT                      (0x2)
#define MCF_FEC_RCR_MII_MODE                 (0x4)
#define MCF_FEC_RCR_PROM                     (0x8)
#define MCF_FEC_RCR_BC_REJ                   (0x10)
#define MCF_FEC_RCR_FCE                      (0x20)
#define MCF_FEC_RCR_MAX_FL(x)                (((x)&0x7FF)<<0x10)

/* Bit definitions and macros for MCF_FEC_TCR */
#define MCF_FEC_TCR_GTS                      (0x1)
#define MCF_FEC_TCR_HBC                      (0x2)
#define MCF_FEC_TCR_FDEN                     (0x4)
#define MCF_FEC_TCR_TFC_PAUSE                (0x8)
#define MCF_FEC_TCR_RFC_PAUSE                (0x10)

/* Bit definitions and macros for MCF_FEC_PALR */
#define MCF_FEC_PALR_PADDR1(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_PAUR */
#define MCF_FEC_PAUR_TYPE(x)                 (((x)&0xFFFF)<<0)
#define MCF_FEC_PAUR_PADDR2(x)               (((x)&0xFFFF)<<0x10)

/* Bit definitions and macros for MCF_FEC_OPD */
#define MCF_FEC_OPD_PAUSE_DUR(x)             (((x)&0xFFFF)<<0)
#define MCF_FEC_OPD_OPCODE(x)                (((x)&0xFFFF)<<0x10)

/* Bit definitions and macros for MCF_FEC_IAUR */
#define MCF_FEC_IAUR_IADDR1(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IALR */
#define MCF_FEC_IALR_IADDR2(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_GAUR */
#define MCF_FEC_GAUR_GADDR1(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_GALR */
#define MCF_FEC_GALR_GADDR2(x)               (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_TFWR */
#define MCF_FEC_TFWR_X_WMRK(x)               (((x)&0x3)<<0)
#define MCF_FEC_TFWR_X_WMRK_64               (0)
#define MCF_FEC_TFWR_X_WMRK_128              (0x2)
#define MCF_FEC_TFWR_X_WMRK_192              (0x3)

/* Bit definitions and macros for MCF_FEC_FRBR */
#define MCF_FEC_FRBR_R_BOUND(x)              (((x)&0xFF)<<0x2)

/* Bit definitions and macros for MCF_FEC_FRSR */
#define MCF_FEC_FRSR_R_FSTART(x)             (((x)&0xFF)<<0x2)

/* Bit definitions and macros for MCF_FEC_ERDSR */
#define MCF_FEC_ERDSR_R_DES_START(x)         (((x)&0x3FFFFFFF)<<0x2)

/* Bit definitions and macros for MCF_FEC_ETSDR */
#define MCF_FEC_ETSDR_X_DES_START(x)         (((x)&0x3FFFFFFF)<<0x2)

/* Bit definitions and macros for MCF_FEC_EMRBR */
#define MCF_FEC_EMRBR_R_BUF_SIZE(x)          (((x)&0x7F)<<0x4)

/* Bit definitions and macros for MCF_FEC_RMON_T_DROP */
#define MCF_FEC_RMON_T_DROP_Value(x)         (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_PACKETS */
#define MCF_FEC_RMON_T_PACKETS_Value(x)      (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_BC_PKT */
#define MCF_FEC_RMON_T_BC_PKT_Value(x)       (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_MC_PKT */
#define MCF_FEC_RMON_T_MC_PKT_Value(x)       (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_CRC_ALIGN */
#define MCF_FEC_RMON_T_CRC_ALIGN_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_UNDERSIZE */
#define MCF_FEC_RMON_T_UNDERSIZE_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_OVERSIZE */
#define MCF_FEC_RMON_T_OVERSIZE_Value(x)     (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_FRAG */
#define MCF_FEC_RMON_T_FRAG_Value(x)         (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_JAB */
#define MCF_FEC_RMON_T_JAB_Value(x)          (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_COL */
#define MCF_FEC_RMON_T_COL_Value(x)          (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_P64 */
#define MCF_FEC_RMON_T_P64_Value(x)          (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_P65TO127 */
#define MCF_FEC_RMON_T_P65TO127_Value(x)     (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_P128TO255 */
#define MCF_FEC_RMON_T_P128TO255_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_P256TO511 */
#define MCF_FEC_RMON_T_P256TO511_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_P512TO1023 */
#define MCF_FEC_RMON_T_P512TO1023_Value(x)   (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_P1024TO2047 */
#define MCF_FEC_RMON_T_P1024TO2047_Value(x)  (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_P_GTE2048 */
#define MCF_FEC_RMON_T_P_GTE2048_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_T_OCTETS */
#define MCF_FEC_RMON_T_OCTETS_Value(x)       (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_DROP */
#define MCF_FEC_IEEE_T_DROP_Value(x)         (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_FRAME_OK */
#define MCF_FEC_IEEE_T_FRAME_OK_Value(x)     (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_1COL */
#define MCF_FEC_IEEE_T_1COL_Value(x)         (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_MCOL */
#define MCF_FEC_IEEE_T_MCOL_Value(x)         (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_DEF */
#define MCF_FEC_IEEE_T_DEF_Value(x)          (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_LCOL */
#define MCF_FEC_IEEE_T_LCOL_Value(x)         (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_EXCOL */
#define MCF_FEC_IEEE_T_EXCOL_Value(x)        (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_MACERR */
#define MCF_FEC_IEEE_T_MACERR_Value(x)       (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_CSERR */
#define MCF_FEC_IEEE_T_CSERR_Value(x)        (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_SQE */
#define MCF_FEC_IEEE_T_SQE_Value(x)          (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_FDXFC */
#define MCF_FEC_IEEE_T_FDXFC_Value(x)        (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_T_OCTETS_OK */
#define MCF_FEC_IEEE_T_OCTETS_OK_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_PACKETS */
#define MCF_FEC_RMON_R_PACKETS_Value(x)      (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_BC_PKT */
#define MCF_FEC_RMON_R_BC_PKT_Value(x)       (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_MC_PKT */
#define MCF_FEC_RMON_R_MC_PKT_Value(x)       (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_CRC_ALIGN */
#define MCF_FEC_RMON_R_CRC_ALIGN_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_UNDERSIZE */
#define MCF_FEC_RMON_R_UNDERSIZE_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_OVERSIZE */
#define MCF_FEC_RMON_R_OVERSIZE_Value(x)     (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_FRAG */
#define MCF_FEC_RMON_R_FRAG_Value(x)         (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_JAB */
#define MCF_FEC_RMON_R_JAB_Value(x)          (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_RESVD_0 */
#define MCF_FEC_RMON_R_RESVD_0_Value(x)      (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_P64 */
#define MCF_FEC_RMON_R_P64_Value(x)          (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_P65TO127 */
#define MCF_FEC_RMON_R_P65TO127_Value(x)     (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_P128TO255 */
#define MCF_FEC_RMON_R_P128TO255_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_P256TO511 */
#define MCF_FEC_RMON_R_P256TO511_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_P512TO1023 */
#define MCF_FEC_RMON_R_P512TO1023_Value(x)   (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_P1024TO2047 */
#define MCF_FEC_RMON_R_P1024TO2047_Value(x)  (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_P_GTE2048 */
#define MCF_FEC_RMON_R_P_GTE2048_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_RMON_R_OCTETS */
#define MCF_FEC_RMON_R_OCTETS_Value(x)       (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_R_DROP */
#define MCF_FEC_IEEE_R_DROP_Value(x)         (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_R_FRAME_OK */
#define MCF_FEC_IEEE_R_FRAME_OK_Value(x)     (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_R_CRC */
#define MCF_FEC_IEEE_R_CRC_Value(x)          (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_R_ALIGN */
#define MCF_FEC_IEEE_R_ALIGN_Value(x)        (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_R_MACERR */
#define MCF_FEC_IEEE_R_MACERR_Value(x)       (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_R_FDXFC */
#define MCF_FEC_IEEE_R_FDXFC_Value(x)        (((x)&0xFFFFFFFF)<<0)

/* Bit definitions and macros for MCF_FEC_IEEE_R_OCTETS_OK */
#define MCF_FEC_IEEE_R_OCTETS_OK_Value(x)    (((x)&0xFFFFFFFF)<<0)

/* Bit definitions for Rx buffer descriptor */
#define MCF_FEC_RX_BD_EMPTY					(1<<15)
#define MCF_FEC_RX_BD_SOFTWARE_OWNERSHIP_1	(1<<14)
#define MCF_FEC_RX_BD_WRAP					(1<<13)
#define MCF_FEC_RX_BD_SOFTWARE_OWNERSHIP_2	(1<<12)
#define MCF_FEC_RX_BD_LAST_IN_FRAME			(1<<11)
#define MCF_FEC_RX_BD_MISS					(1<<8)
#define MCF_FEC_RX_BD_BROADCAST				(1<<7)
#define MCF_FEC_RX_BD_MULTICAST				(1<<6)
#define MCF_FEC_RX_BD_LENGTH_VIOLATION		(1<<5)
#define MCF_FEC_RX_BD_NON_OCTET_ALIGNED		(1<<4)
#define MCF_FEC_RX_BD_RX_CRC_ERROR			(1<<2)
#define MCF_FEC_RX_BD_OVERRUN				(1<<1)
#define MCF_FEC_RX_BD_TRUNCATED				(1<<0)

/* Bit definitions for Tx buffer descriptor*/
#define MCF_FEC_TX_BD_READY					(1<<15)
#define MCF_FEC_TX_BD_SOFTWARE_OWNERSHIP_1	(1<<14)
#define MCF_FEC_TX_BD_WRAP					(1<<13)
#define MCF_FEC_TX_BD_SOFTWARE_OWNERSHIP_2	(1<<12)
#define MCF_FEC_TX_BD_LAST_IN_FRAME			(1<<11)
#define MCF_FEC_TX_BD_TRANSMIT_CRC			(1<<10)
#define MCF_FEC_TX_BD_APPEND_BAD_CRC		(1<<9)

