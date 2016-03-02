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

#ifndef __MCF5225_QSPI_H__
#define __MCF5225_QSPI_H__

#ifndef _ARCH_M68K_H_
#error "Do not include this file directly. Use arch/m68k.h instead!"
#endif

/*********************************************************************
*
* Queued Serial Peripheral Interface (QSPI)
*
*********************************************************************/

/* Register read/write macros */
#define MCF_QSPI_QMR                         (*(volatile uint16_t*)(0x40000340))
#define MCF_QSPI_QDLYR                       (*(volatile uint16_t*)(0x40000344))
#define MCF_QSPI_QWR                         (*(volatile uint16_t*)(0x40000348))
#define MCF_QSPI_QIR                         (*(volatile uint16_t*)(0x4000034C))
#define MCF_QSPI_QAR                         (*(volatile uint16_t*)(0x40000350))
#define MCF_QSPI_QDR                         (*(volatile uint16_t*)(0x40000354))


/* Bit definitions and macros for MCF_QSPI_QMR */
#define MCF_QSPI_QMR_BAUD(x)                 (((x)&0xFF)<<0)
#define MCF_QSPI_QMR_CPHA                    (0x100)
#define MCF_QSPI_QMR_CPOL                    (0x200)
#define MCF_QSPI_QMR_BITS(x)                 (((x)&0xF)<<0xA)
#define MCF_QSPI_QMR_DOHIE                   (0x4000)
#define MCF_QSPI_QMR_MSTR                    (0x8000)

/* Bit definitions and macros for MCF_QSPI_QDLYR */
#define MCF_QSPI_QDLYR_DTL(x)                (((x)&0xFF)<<0)
#define MCF_QSPI_QDLYR_QCD(x)                (((x)&0x7F)<<0x8)
#define MCF_QSPI_QDLYR_SPE                   (0x8000)

/* Bit definitions and macros for MCF_QSPI_QWR */
#define MCF_QSPI_QWR_NEWQP(x)                (((x)&0xF)<<0)
#define MCF_QSPI_QWR_CPTQP(x)                (((x)&0xF)<<0x4)
#define MCF_QSPI_QWR_ENDQP(x)                (((x)&0xF)<<0x8)
#define MCF_QSPI_QWR_CSIV                    (0x1000)
#define MCF_QSPI_QWR_WRTO                    (0x2000)
#define MCF_QSPI_QWR_WREN                    (0x4000)
#define MCF_QSPI_QWR_HALT                    (0x8000)

/* Bit definitions and macros for MCF_QSPI_QIR */
#define MCF_QSPI_QIR_SPIF                    (0x1)
#define MCF_QSPI_QIR_ABRT                    (0x4)
#define MCF_QSPI_QIR_WCEF                    (0x8)
#define MCF_QSPI_QIR_SPIFE                   (0x100)
#define MCF_QSPI_QIR_ABRTE                   (0x400)
#define MCF_QSPI_QIR_WCEFE                   (0x800)
#define MCF_QSPI_QIR_ABRTL                   (0x1000)
#define MCF_QSPI_QIR_ABRTB                   (0x4000)
#define MCF_QSPI_QIR_WCEFB                   (0x8000)

/* Bit definitions and macros for MCF_QSPI_QAR */
#define MCF_QSPI_QAR_ADDR(x)                 (((x)&0x3F)<<0)
#define MCF_QSPI_QAR_TRANS                   (0)
#define MCF_QSPI_QAR_RECV                    (0x10)
#define MCF_QSPI_QAR_CMD                     (0x20)

/* Bit definitions and macros for MCF_QSPI_QDR */
#define MCF_QSPI_QDR_DATA(x)                 (((x)&0xFFFF)<<0)
#define MCF_QSPI_QDR_CONT                    (0x8000)
#define MCF_QSPI_QDR_BITSE                   (0x4000)
#define MCF_QSPI_QDR_DT                      (0x2000)
#define MCF_QSPI_QDR_DSCK                    (0x1000)
#define MCF_QSPI_QDR_QSPI_CS3                (0x800)
#define MCF_QSPI_QDR_QSPI_CS2                (0x400)
#define MCF_QSPI_QDR_QSPI_CS1                (0x200)
#define MCF_QSPI_QDR_QSPI_CS0                (0x100)
#define MCF_QSPI_QDR_CS(x)                 	 ((0xF<<8) &~(x))

#endif /* __MCF5225_QSPI_H__ */
