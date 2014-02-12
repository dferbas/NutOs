#ifndef _STM32_FLASH_H_
#define _STM32_FLASH_H_
/*
 * Copyright (C) 2006 by egnite Software GmbH. All rights reserved.
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
 *
 */

/*!
 * \file arch/cm3/stm/atm32_flash.h
 * \brief On-Chip flash memory access.
 * $Id: stm32_flash.h 3220 2010-11-12 13:04:17Z astralix $
 * \verbatim
 */

// #define FLASHDEBUG

/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#if defined (STM32F10X_HD) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE        ((uint16_t)0x800)
#else
  #define FLASH_PAGE_SIZE        ((uint16_t)0x400)
#endif

/* FLASH BANK addresses */
#define FLASH_BANK1_START_ADDR   ((void*)0x08000000)
#define FLASH_BANK1_END_ADDR     ((void*)0x0807FFFF)

#define FLASH_BANK2_START_ADDR   ((void*)0x08080000)
#define FLASH_BANK2_END_ADDR     ((void*)0x080FFFFF)

#define FLASH_TOTAL_SIZE         ((uint32_t)ESIG->FLASH_SIZE*1024UL)
#define FLASH_START_ADDR         FLASH_BANK1_START_ADDR
#define FLASH_END_ADDR           ((void*)FLASH_BANK1_START_ADDR+FLASH_TOTAL_SIZE-1)

/* Mask to calculate with sectors */
#define FLASH_PAGE_MASK          ((uint32_t)0x0FFFFFFF-(FLASH_PAGE_SIZE-1))
#define FLASH_PAGE_OFFS          ((uint32_t)(FLASH_PAGE_SIZE-1))

/* Nut/OS specific definition for config sector */
/* Assume size of config area is one page */
#ifndef FLASH_CONF_SIZE
#define FLASH_CONF_SIZE FLASH_PAGE_SIZE
#endif

/* Assume address of config area is last flash sector */
#ifndef FLASH_CONF_SECTOR
#define FLASH_CONF_SECTOR       ((void*)FLASH_BANK1_START_ADDR+FLASH_TOTAL_SIZE-FLASH_PAGE_SIZE)
#endif

/* Macros for NUTASSERT */
#define IS_FLASH_ADDRESS(a)  ((a>=FLASH_START_ADDR)&&(a<(FLASH_END_ADDR+1)))
#define IS_PAGE_ADDRESS(a)   ((a&FLASH_PAGE_OFFS)==0x00000000)


/*!
 * brief FLASH status returns of any operation.
 */
typedef enum
{
    FLASH_BUSY          =  1,   /*!< Flash operation pending */
    FLASH_COMPLETE      =  0,   /*!< Flash operation successfull completed */
    FLASH_ERROR_PG      = -1,   /*!< Flash programming failed */
    FLASH_ERROR_WRP     = -2,   /*!< Flash write protected */
    FLASH_LOCKED        = -3,   /*!< FLASH is locked, unlocking failed */
    FLASH_TIMEOUT       = -4,   /*!< Flash operation timed out */
    FLASH_BOUNDARY      = -5,   /*!< Flash accessed out of flash area */
    FLASH_COMPARE       = -6,   /*!< FLASH compare mismatch */
} FLASH_Status;

/* Internally used functions that should not be accessef by user
 * application without exact knowledge of the flash system.
 */
extern FLASH_Status FLASH_Unlock( void *addr, size_t len);
extern FLASH_Status Stm32FlashWaitReady(uint32_t Timeout);
extern int Stm32FlashErasePage(void *page);
extern int Stm32FlashWritePage( void *dst, void *src);

/* FLASH API functions to read and write at any FLASH address and
 * in any count of data.
 */
extern uint32_t GetTotalFlashSize( void);
extern void* GetFlashEndAddress( void);
extern int Stm32FlashRead( void* dst, void* src, size_t len);
extern int Stm32FlashWrite( void* dst, void* src, size_t len);

/* Nut/OS specific API functions to read and write configuration data.
 */
extern int Stm32FlashParamRead(unsigned int pos, void *data, unsigned int len);
extern int Stm32FlashParamWrite(unsigned int pos, const void *data, unsigned int len);

#endif /* _STM32_FLASH_H_ */
