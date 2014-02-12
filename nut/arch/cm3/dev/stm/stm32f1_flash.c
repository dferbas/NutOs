/**
  ******************************************************************************
  * @file    stm32f10x_flash.c
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file provides all the FLASH firmware functions.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 *
 * All rights reserved.
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

/*
 * \verbatim
 * $Id: stm32f1_flash.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

/* Includes ------------------------------------------------------------------*/
#include <cfg/os.h>
#include <cfg/arch.h>
#include <cfg/clock.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/heap.h>
#include <sys/nutdebug.h>

#include <dev/irqreg.h>
#include <arch/cm3/stm/stm32_flash.h>

/* Small abstraction of somoe FLASH registers
 * to handle bank1 and bank2 of XL devices in one function.
 */
typedef struct {
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
} FLASH_BankRegT;


/* Flash Access Control Register bits */
#define ACR_LATENCY_Mask         ((uint32_t)0x00000038)
#define ACR_HLFCYA_Mask          ((uint32_t)0xFFFFFFF7)
#define ACR_PRFTBE_Mask          ((uint32_t)0xFFFFFFEF)

/* Flash Access Control Register bits */
#define ACR_PRFTBS_Mask          ((uint32_t)0x00000020)

/* FLASH Mask */
#define RDPRT_Mask               ((uint32_t)0x00000002)
#define WRP0_Mask                ((uint32_t)0x000000FF)
#define WRP1_Mask                ((uint32_t)0x0000FF00)
#define WRP2_Mask                ((uint32_t)0x00FF0000)
#define WRP3_Mask                ((uint32_t)0xFF000000)
#define OB_USER_BFB2             ((uint16_t)0x0008)

/* FLASH Keys */
#define RDP_Key                  ((uint16_t)0x00A5)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

/* Delay definition */
#define EraseTimeout             ((uint32_t)0x000B0000)
#define ProgramTimeout           ((uint32_t)0x00002000)

/**
  * @brief  Sets the code latency value.
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *   This parameter can be one of the following values:
  *     @arg FLASH_Latency_0: FLASH Zero Latency cycle
  *     @arg FLASH_Latency_1: FLASH One Latency cycle
  *     @arg FLASH_Latency_2: FLASH Two Latency cycles
  * @retval None
  */
void FLASH_SetLatency(uint32_t FLASH_Latency)
{
    uint32_t tmpreg = 0;

    /* Check the parameters */
    NUTASSERT(IS_FLASH_LATENCY(FLASH_Latency));

    /* Read the ACR register */
    tmpreg = FLASH->ACR;

    /* Sets the Latency value */
    tmpreg &= ACR_LATENCY_Mask;
    tmpreg |= FLASH_Latency;

    /* Write the ACR register */
    FLASH->ACR = tmpreg;
}

/**
  * @brief  Enables or disables the Half cycle flash access.
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_HalfCycleAccess: specifies the FLASH Half cycle Access mode.
  *   This parameter can be one of the following values:
  *     @arg FLASH_HalfCycleAccess_Enable: FLASH Half Cycle Enable
  *     @arg FLASH_HalfCycleAccess_Disable: FLASH Half Cycle Disable
  * @retval None
  */
void FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess)
{
    /* Check the parameters */
    NUTASSERT(IS_FLASH_HALFCYCLEACCESS_STATE(FLASH_HalfCycleAccess));

    /* Enable or disable the Half cycle access */
    FLASH->ACR &= ACR_HLFCYA_Mask;
    FLASH->ACR |= FLASH_HalfCycleAccess;
}

/**
  * @brief  Enables or disables the Prefetch Buffer.
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_PrefetchBuffer: specifies the Prefetch buffer status.
  *   This parameter can be one of the following values:
  *     @arg FLASH_PrefetchBuffer_Enable: FLASH Prefetch Buffer Enable
  *     @arg FLASH_PrefetchBuffer_Disable: FLASH Prefetch Buffer Disable
  * @retval None
  */
void FLASH_PrefetchBufferCmd(uint32_t FLASH_PrefetchBuffer)
{
    /* Check the parameters */
    NUTASSERT(IS_FLASH_PREFETCHBUFFER_STATE(FLASH_PrefetchBuffer));

    /* Enable or disable the Prefetch Buffer */
    FLASH->ACR &= ACR_PRFTBE_Mask;
    FLASH->ACR |= FLASH_PrefetchBuffer;
}

/**
  * @brief  Clears the FLASH’s pending flags.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices, this function clears Bank1 or Bank2’s pending flags
  *         - For other devices, it clears Bank1’s pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *   This parameter can be any combination of the following values:
  *     @arg FLASH_FLAG_PGERR: FLASH Program error flag
  *     @arg FLASH_FLAG_WRPRTERR: FLASH Write protected error flag
  *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag
  * @retval None
  */
void FLASH_ClearFlag( uint32_t FLASH_FLAG)
{
    /* Clear the flags */
#if defined(STM32F10X_XL)
        FLASH->SR2 = FLASH_FLAG;
#endif /* STM32F10X_XL */
        FLASH->SR = FLASH_FLAG;
}

/**
  * @brief  Returns the FLASH Status.
  * @note   This function can be used for all STM32F10x devices, it is equivalent
  *    to FLASH_GetBank1Status function.
  * @param  addr: Address inside bank to check status of.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP or FLASH_COMPLETE
  */
FLASH_Status FLASH_GetStatus(void)
{
    FLASH_Status rs = FLASH_COMPLETE;
    register uint16_t flash_sr = FLASH->SR;

#if defined(STM32F10X_XL)
    flash_sr |= FLASH->SR2;
#endif /* STM32F10X_XL */

    /* Decode the Flash Status */
    if (flash_sr & FLASH_SR_BSY)
        rs = FLASH_BUSY;
    else if (flash_sr & FLASH_SR_WRPRTERR)
        rs = FLASH_ERROR_WRP;
    else if (flash_sr & FLASH_SR_PGERR)
        rs = FLASH_ERROR_PG;

    /* Return the Flash Status */
    return rs;
}

/**
  * @brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
  * @note   This function can be used for all STM32F10x devices,
  *         it is equivalent to FLASH_WaitForLastBank1Operation.
  *         - For STM32F10X_XL devices this function waits for a Bank1 Flash operation
  *           to complete or a TIMEOUT to occur.
  *         - For all other devices it waits for a Flash operation to complete
  *           or a TIMEOUT to occur.
  * @param  Timeout: FLASH progamming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status Stm32FlashWaitReady(uint32_t Timeout)
{
    FLASH_Status status;

    /* Wait for a Flash operation to complete or a TIMEOUT to occur */
    do {
        /* Check for the Flash Status */
        status = FLASH_GetStatus();
        if(--Timeout == 0 )
            status = FLASH_TIMEOUT;
    } while(status == FLASH_BUSY);

    /* Return the operation status */
    return status;
}

/**
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function unlocks Bank1 and Bank2.
  *         - For all other devices it unlocks Bank1 and it is equivalent
  *           to FLASH_UnlockBank1 function..
  * @param  addr Start address of area to unlock
  * @param  len  Length of area to unlock
  * @retval 0 on success, -1 if out of flash area.
  */
FLASH_Status FLASH_Unlock( void *addr, size_t len)
{
    FLASH_Status rs = FLASH_COMPLETE;
    uint16_t flash_cr = 0;

    /* Check the parameters */
    NUTASSERT(IS_FLASH_ADDRESS(addr));

    if (addr<FLASH_BANK2_START_ADDR) {
        /* Unlock Bank 1: Authorize the FPEC of Bank1 Access */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
        flash_cr = FLASH->CR&FLASH_CR_LOCK;
    }
#if defined(STM32F10X_XL)
    if ((addr+len ) > FLASH_BANK1_END_ADDR) {
        /* Unlock Bank 2: Authorize the FPEC of Bank2 Access */
        FLASH->KEYR2 = FLASH_KEY1;
        FLASH->KEYR2 = FLASH_KEY2;
        flash_cr |= FLASH->CR2&FLASH_CR_LOCK;
    }
#endif /* STM32F10X_XL */

    if (flash_cr) rs = FLASH_LOCKED;
    return rs;
}

/**
  * @brief  Locks the FLASH Program Erase Controller.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function Locks Bank1 and Bank2.
  *         - For all other devices it Locks Bank1 and it is equivalent
  *           to FLASH_LockBank1 function.
  * @param  None
  * @retval None
  */
void FLASH_Lock(void *addr, size_t len)
{
    if (addr<FLASH_BANK2_START_ADDR) {
        /* Set the Lock Bit to lock the FPEC and the CR of  Bank1 */
        FLASH->CR |= FLASH_CR_LOCK;
    }
#if defined(STM32F10X_XL)
    if ((addr+len ) > FLASH_BANK1_END_ADDR) {
        /* Set the Lock Bit to lock the FPEC and the CR of  Bank2 */
        FLASH->CR2 |= FLASH_CR_LOCK;
    }
#endif /* STM32F10X_XL */
}


#if 0
/**
  * @brief  Enables or disables the specified FLASH interrupts.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices, enables or disables the specified FLASH interrupts
              for Bank1 and Bank2.
  *         - For other devices it enables or disables the specified FLASH interrupts for Bank1.
  * @param  FLASH_IT: specifies the FLASH interrupt sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg FLASH_IT_ERROR: FLASH Error Interrupt
  *     @arg FLASH_IT_EOP: FLASH end of operation Interrupt
  * @param  NewState: new state of the specified Flash interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState)
{
#if defined(STM32F10X_XL)
    /* Check the parameters */
    assert_param(IS_FLASH_IT(FLASH_IT));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if((FLASH_IT & 0x80000000) != 0x0) {
        if(NewState != DISABLE) {
            /* Enable the interrupt sources */
            FLASH->CR2 |= (FLASH_IT & 0x7FFFFFFF);
        }
        else {
            /* Disable the interrupt sources */
            FLASH->CR2 &= ~(uint32_t)(FLASH_IT & 0x7FFFFFFF);
        }
    }
    else {
        if(NewState != DISABLE) {
            /* Enable the interrupt sources */
            FLASH->CR |= FLASH_IT;
        }
        else {
            /* Disable the interrupt sources */
            FLASH->CR &= ~(uint32_t)FLASH_IT;
        }
    }
#else
    /* Check the parameters */
    assert_param(IS_FLASH_IT(FLASH_IT));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if(NewState != DISABLE) {
        /* Enable the interrupt sources */
        FLASH->CR |= FLASH_IT;
    }
    else {
        /* Disable the interrupt sources */
        FLASH->CR &= ~(uint32_t)FLASH_IT;
    }
#endif /* STM32F10X_XL */

}
/**
  * @brief  Erases all Bank1 FLASH pages.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function erases all Bank1 pages.
  *         - For all other devices it erases all Bank1 pages and it is equivalent
  *           to FLASH_EraseAllPages function.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseAllBank1Pages(void)
{
    FLASH_Status status = FLASH_COMPLETE;

    /* Wait for last operation to be completed */
    status = Stm32FlashWaitReady( EraseTimeout);

    if(status == FLASH_COMPLETE) {
        /* if the previous operation is completed, proceed to erase all pages */
         FLASH->CR |= FLASH_CR_MER;
         FLASH->CR |= FLASH_CR_STRT;

        /* Wait for last operation to be completed */
        status = Stm32FlashWaitReady( EraseTimeout);
        if(status != FLASH_TIMEOUT) {
            /* if the erase operation is completed, disable the MER Bit */
            FLASH->CR &= ~FLASH_CR_MER;
        }
    }
    /* Return the Erase Status */
    return status;
}

#if defined(STM32F10X_XL)
/**
  * @brief  Erases all Bank2 FLASH pages.
  * @note   This function can be used only for STM32F10x_XL density devices.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseAllBank2Pages(void)
{
    FLASH_Status status = FLASH_COMPLETE;

    /* Wait for last operation to be completed */
    status = Stm32FlashWaitReady( FLASH_BANK2_START_ADDR, EraseTimeout);

    if(status == FLASH_COMPLETE) {
        /* if the previous operation is completed, proceed to erase all pages */
        FLASH->CR2 |= FLASH_CR_MER;
        FLASH->CR2 |= FLASH_CR_STRT;

        /* Wait for last operation to be completed */
        status = Stm32FlashWaitReady( FLASH_BANK2_START_ADDR, EraseTimeout);
        if(status != FLASH_TIMEOUT) {
            /* if the erase operation is completed, disable the MER Bit */
            FLASH->CR2 &= FLASH_CR_MER;
        }
    }
    /* Return the Erase Status */
    return status;
}
#endif /* STM32F10X_XL */

/**
  * @brief  Erases all FLASH pages.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseAllPages(void)
{
    FLASH_Status status = FLASH_COMPLETE;

    status = FLASH_EraseAllBank1Pages();
#if defined(STM32F10X_XL)
    if(status == FLASH_COMPLETE) {
        status = FLASH_EraseAllBank2Pages();
#endif /* STM32F10X_XL */

    /* Return the Erase Status */
    return status;
}

/**
  * @brief  Returns the FLASH User Option Bytes values.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval The FLASH User Option Bytes values:IWDG_SW(Bit0), RST_STOP(Bit1)
  *   and RST_STDBY(Bit2).
  */
uint32_t FLASH_GetUserOptionByte(void)
{
    /* Return the User Option Byte */
    return (FLASH->OBR >> 2);
}

/**
  * @brief  Erases the FLASH option bytes.
  * @note   This functions erases all option bytes except the Read protection (RDP).
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseOptionBytes(void)
{
    uint16_t rdptmp = RDP_Key;

    FLASH_Status status = FLASH_COMPLETE;

    /* Get the actual read protection Option Byte value */
    if(FLASH_GetReadOutProtectionStatus() != RESET) {
        rdptmp = 0x00;
    }

    /* Wait for last operation to be completed */
    status = Stm32FlashWaitReady(EraseTimeout);
    if(status == FLASH_COMPLETE) {
        /* Authorize the small information block programming */
        FLASH->OPTKEYR = FLASH_KEY1;
        FLASH->OPTKEYR = FLASH_KEY2;

        /* if the previous operation is completed, proceed to erase the option bytes */
        FLASH->CR |= FLASH_CR_OPTER;
        FLASH->CR |= FLASH_CR_STRT;
        /* Wait for last operation to be completed */
        status = Stm32FlashWaitReady(EraseTimeout);

        if(status == FLASH_COMPLETE) {
            /* if the erase operation is completed, disable the OPTER Bit */
            FLASH->CR &= ~FLASH_CR_OPTER;

            /* Enable the Option Bytes Programming operation */
            FLASH->CR |= FLASH_CR_OPTPG;
            /* Restore the last read protection Option Byte value */
            OB->RDP = (uint16_t)rdptmp;
            /* Wait for last operation to be completed */
            status = Stm32FlashWaitReady(ProgramTimeout);

            if(status != FLASH_TIMEOUT)
            {
            /* if the program operation is completed, disable the OPTPG Bit */
            FLASH->CR &= ~FLASH_CR_OPTPG;
            }
        }
        else {
            if (status != FLASH_TIMEOUT) {
                /* Disable the OPTPG Bit */
                FLASH->CR &= ~FLASH_CR_OPTPG;
            }
        }
    }

    /* Return the erase status */
    return status;
}

/**
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Address: specifies the address to be programmed.
  *   This parameter can be 0x1FFFF804 or 0x1FFFF806.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data)
{
    FLASH_Status status = FLASH_COMPLETE;
    /* Check the parameters */
    NUTASSERT(IS_OB_DATA_ADDRESS(Address));

    status = Stm32FlashWaitReady(ProgramTimeout);

    if(status == FLASH_COMPLETE) {
        /* Authorize the small information block programming */
        FLASH->OPTKEYR = FLASH_KEY1;
        FLASH->OPTKEYR = FLASH_KEY2;
        /* Enables the Option Bytes Programming operation */
        FLASH->CR |= FLASH_CR_OPTPG;
        *(__IO uint16_t*)Address = Data;

        /* Wait for last operation to be completed */
        status = Stm32FlashWaitReady(Address, ProgramTimeout);
        if(status != FLASH_TIMEOUT) {
            /* if the program operation is completed, disable the OPTPG Bit */
            FLASH->CR &= ~FLASH_CR_OPTPG;
        }
    }
    /* Return the Option Byte Data Program Status */
    return status;
}

#if defined(STM32F10X_XL)
/**
  * @brief  Configures to boot from Bank1 or Bank2.
  * @note   This function can be used only for STM32F10x_XL density devices.
  * @param  FLASH_BOOT: select the FLASH Bank to boot from.
  *   This parameter can be one of the following values:
  *     @arg FLASH_BOOT_Bank1: At startup, if boot pins are set in boot from user Flash
  *        position and this parameter is selected the device will boot from Bank1(Default).
  *     @arg FLASH_BOOT_Bank2: At startup, if boot pins are set in boot from user Flash
  *        position and this parameter is selected the device will boot from Bank2 or Bank1,
  *        depending on the activation of the bank. The active banks are checked in
  *        the following order: Bank2, followed by Bank1.
  *        The active bank is recognized by the value programmed at the base address
  *        of the respective bank (corresponding to the initial stack pointer value
  *        in the interrupt vector table).
  *        For more information, please refer to AN2606 from www.st.com.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  * FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_BootConfig(uint16_t FLASH_BOOT)
{
    FLASH_Status status = FLASH_COMPLETE;
    NUTASSERT(IS_FLASH_BOOT(FLASH_BOOT));
    /* Authorize the small information block programming */
    FLASH->OPTKEYR = FLASH_KEY1;
    FLASH->OPTKEYR = FLASH_KEY2;

    /* Wait for last operation to be completed */
    status = Stm32FlashWaitReady(ProgramTimeout);

    if(status == FLASH_COMPLETE)
    {
        /* Enable the Option Bytes Programming operation */
        FLASH->CR |= FLASH_CR_OPTPG;

        if(FLASH_BOOT == FLASH_BOOT_Bank1)
            OB->USER |= OB_USER_BFB2;
        else
            OB->USER &= (uint16_t)(~(uint16_t)(OB_USER_BFB2));

        /* Wait for last operation to be completed */
        status = Stm32FlashWaitReady(ProgramTimeout);
        if(status != FLASH_TIMEOUT) {
            /* if the program operation is completed, disable the OPTPG Bit */
            FLASH->CR &= ~FLASH_CR_OPTPG;
        }
    }
    /* Return the Option Byte program Status */
    return status;
}
#endif /* STM32F10X_XL */

#endif

/**
  * @brief  Get the total size of this CPU's FLASH.
  * @note   This function can be used for all STM32F10x devices.
  * @retval size of FLASH in bytes.
  */
uint32_t GetTotalFlashSize( void)
{
    return FLASH_TOTAL_SIZE;
}

/**
  * @brief  Get the last valid address in the FLASH.
  * @note   This function can be used for all STM32F10x devices.
  * @retval address.
  */
void* GetFlashEndAddress( void)
{
    return FLASH_END_ADDR;
}

/**
  * @brief  Programs a half word at a specified address.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_Program16(uint32_t addr, uint16_t Data)
{
    FLASH_Status status = FLASH_COMPLETE;
    FLASH_BankRegT *bankr = (FLASH_BankRegT*)&FLASH->SR;

    /* Check the parameters */
    NUTASSERT(IS_FLASH_ADDRESS(addr));

    /* Wait for last operation to be completed */
    status = Stm32FlashWaitReady(ProgramTimeout);
    if(status == FLASH_COMPLETE)
    {
#if defined(STM32F10X_XL)
        if(addr > (uint32_t)FLASH_BANK1_END_ADDR)
            bankr = (FLASH_BankRegT*)&FLASH->SR2;
#endif  /* STM32F10X_XL */

        bankr->CR |= FLASH_CR_PG;
        *(__IO uint16_t*)addr = Data;

        /* Wait for last operation to be completed */
        status = Stm32FlashWaitReady(ProgramTimeout);
        if(status != FLASH_TIMEOUT) {
            /* if the program operation is completed, disable the PG Bit */
            bankr->CR &= ~FLASH_CR_PG;
        }
    }

    /* Return the Program Status */
    return status;
}

/**
  * @brief  Write protects the desired pages
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_Pages: specifies the address of the pages to be write protected.
  *   This parameter can be:
  *     @arg For @b STM32_Low-density_devices: value between FLASH_WRProt_Pages0to3 and FLASH_WRProt_Pages28to31
  *     @arg For @b STM32_Medium-density_devices: value between FLASH_WRProt_Pages0to3
  *       and FLASH_WRProt_Pages124to127
  *     @arg For @b STM32_High-density_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to255
  *     @arg For @b STM32_Connectivity_line_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to127
  *     @arg For @b STM32_XL-density_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to511
  *     @arg FLASH_WRProt_AllPages
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Pages)
{
    uint16_t WRP0_Data = 0xFFFF, WRP1_Data = 0xFFFF, WRP2_Data = 0xFFFF, WRP3_Data = 0xFFFF;

    FLASH_Status status = FLASH_COMPLETE;

    /* Check the parameters */
    NUTASSERT(IS_FLASH_WRPROT_PAGE(FLASH_Pages));

    FLASH_Pages = (uint32_t)(~FLASH_Pages);
    WRP0_Data = (uint16_t)(FLASH_Pages & WRP0_Mask);
    WRP1_Data = (uint16_t)((FLASH_Pages & WRP1_Mask) >> 8);
    WRP2_Data = (uint16_t)((FLASH_Pages & WRP2_Mask) >> 16);
    WRP3_Data = (uint16_t)((FLASH_Pages & WRP3_Mask) >> 24);

    /* Wait for last operation to be completed */
    status = Stm32FlashWaitReady( ProgramTimeout);

    if(status == FLASH_COMPLETE)
    {
        /* Authorizes the small information block programming */
        FLASH->OPTKEYR = FLASH_KEY1;
        FLASH->OPTKEYR = FLASH_KEY2;
        FLASH->CR |= FLASH_CR_OPTPG;
        if(WRP0_Data != 0xFF) {
            OB->WRP0 = WRP0_Data;

            /* Wait for last operation to be completed */
            status = Stm32FlashWaitReady( ProgramTimeout);
        }
        if((status == FLASH_COMPLETE) && (WRP1_Data != 0xFF)) {
            OB->WRP1 = WRP1_Data;

            /* Wait for last operation to be completed */
            status = Stm32FlashWaitReady( ProgramTimeout);
        }
        if((status == FLASH_COMPLETE) && (WRP2_Data != 0xFF)) {
            OB->WRP2 = WRP2_Data;

            /* Wait for last operation to be completed */
            status = Stm32FlashWaitReady( ProgramTimeout);
        }

        if((status == FLASH_COMPLETE)&& (WRP3_Data != 0xFF)) {
            OB->WRP3 = WRP3_Data;

            /* Wait for last operation to be completed */
            status = Stm32FlashWaitReady( ProgramTimeout);
        }

        if(status != FLASH_TIMEOUT) {
            /* if the program operation is completed, disable the OPTPG Bit */
            FLASH->CR &= ~FLASH_CR_OPTPG;
        }
    }
    /* Return the write protection operation Status */
    return status;
}

/**
  * @brief  Returns the FLASH Write Protection Option Bytes Register value.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval The FLASH Write Protection  Option Bytes Register value
  */
uint32_t FLASH_GetProtectionOptionByte(void)
{
    /* Return the Flash write protection Register value */
    return (FLASH->WRPR);
}

/**
  * @brief  Checks whether the FLASH Read Out Protection Status is set or not.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval 0 for reset, -1 for set.
  */
int FLASH_GetProtection(void)
{
    int rs = 0;

    if (FLASH->OBR & FLASH_OBR_RDPRT)
        rs = -1;

    /* Return the new state of FLASH Protection Status (SET or RESET) */
    return rs;
}

/**
  * @brief  Checks whether the FLASH Prefetch Buffer status is set or not.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval FLASH Prefetch Buffer Status (SET or RESET).
  */
FlagStatus FLASH_GetPrefetchBufferStatus(void)
{
    FlagStatus bitstatus = RESET;

    if ((FLASH->ACR & FLASH_ACR_PRFTBS) != (uint32_t)RESET)
        bitstatus = SET;
    else
        bitstatus = RESET;

    /* Return the new state of FLASH Prefetch Buffer Status (SET or RESET) */
    return bitstatus;
}


/*!
 * \brief Read data from FLASH at specified address.
 *
 * This function copies data from FLASH to a user provided buffer.
 *
 * \param dst Pointer to address where to copy data to.
 * \param src Pointer to address in FLASH to read from.
 * \param len Number fo bytes to copy.
 *
 * \return FLASH Status.
 */
int Stm32FlashRead( void* dst, void* src, size_t len)
{
    int rs = 0;

    NUTASSERT(IS_FLASH_ADDRESS(src));

    memcpy( dst, src, len);

    return rs;
}

/*!
 * \brief Erase FLASH Page at specified address.
 *
 * This routine is called by Stm32FlashWritePage to erase
 * before programming.
 *
 * \param page Pointer to address of page to erase.
 *
 * \return FLASH Status.
 */
int Stm32FlashErasePage(void *page)
{
    FLASH_Status rs = FLASH_COMPLETE;
    FLASH_BankRegT *bankr = (FLASH_BankRegT*)&FLASH->SR;

    /* Check the parameters */
    NUTASSERT(IS_FLASH_ADDRESS(page));
    NUTASSERT(IS_PAGE_ADDRESS(page));

#if defined(STM32F10X_XL)
    if (page > FLASH_BANK1_END_ADDR)
        bankr = (FLASH_BankRegT*)&FLASH->SR2;
#endif /* STM32F10X_XL */

    /* Wait for last operation to be completed */
    rs = Stm32FlashWaitReady(EraseTimeout);
    if(rs == FLASH_COMPLETE) {
        /* if the previous operation is completed, proceed to erase the page */
        bankr->CR |= FLASH_CR_PER;
        bankr->AR = (uint32_t)page;
        bankr->CR |= FLASH_CR_STRT;

        /* Wait for last operation to be completed */
        rs = Stm32FlashWaitReady(EraseTimeout);
        if(rs != FLASH_TIMEOUT) {
            /* if the erase operation is completed, disable the PER Bit */
            bankr->CR &= ~FLASH_CR_PER;
        }
    }

    /* Return the Erase Status */
    return (int)rs;
}

/*!
 * \brief Program FLASH Page.
 *
 * This routine writes a complete page of FLASH.
 *
 * \param dst Pointer to beginning of a FLASH page.
 * \param src Pointer to source data to be written to page.
 *
 * \return FLASH Status.
 */
int Stm32FlashWritePage( void *dst, void *src)
{
    FLASH_Status rs = FLASH_COMPLETE;
    uint16_t cnt = FLASH_PAGE_SIZE/2;
    uint16_t *dptr = dst;
    uint16_t *sptr = src;
    FLASH_BankRegT *bankr = (FLASH_BankRegT*)&FLASH->SR;

    /* Check the parameters */
    NUTASSERT(IS_FLASH_ADDRESS(dst));
    NUTASSERT(IS_PAGE_ADDRESS(dst));

#if defined(STM32F10X_XL)
    if(dptr > (uint16_t*)FLASH_BANK1_END_ADDR)
        bankr = (FLASH_BankRegT*)&FLASH->SR2;
    else
#endif  /* STM32F10X_XL */

    /* Wait for last operation to be completed */
    rs = Stm32FlashWaitReady(ProgramTimeout);
    if (rs == FLASH_COMPLETE)
    {
        /* Enable Programming Mode */
        bankr->CR |= FLASH_CR_PG;

        /* Write data to page */
        while(cnt && (rs == FLASH_COMPLETE))
        {
            *(volatile uint16_t*)dptr++ = *sptr++;
            rs = Stm32FlashWaitReady(ProgramTimeout);
            cnt--;
        }

        if(rs != FLASH_TIMEOUT) {
            bankr->CR &= ~FLASH_CR_PG;
        }
    }

    return (int)rs;
}

/*!
 * \brief Program any data to FLASH.
 *
 * This function writes data from source address to FLASH.
 * It handles erasing and assembling of data automatically.
 *
 * \param dst Pointer to address anywhere in FLASH.
 * \param src Pointer to source data.
 * \param len Number of bytes to be written.
 *
 * \return FLASH Status.
 */
int Stm32FlashWrite( void* dst, void* src, size_t len)
{
    FLASH_Status rs = FLASH_COMPLETE;
    uint8_t *buffer;
    uint8_t *sector;
    uint32_t offset, length;
    uint8_t *rptr = src;
    uint8_t *wptr = dst;
    uint8_t *end = wptr+len;
    uint8_t *flashp;

    /* Check top boundary */
    if ((dst+len) > FLASH_END_ADDR)
        return -1;

    /* Reserve buffer to save data of partly programmed sector.
     * This happens usually at start and end of the range. */
    buffer = NutHeapAlloc(FLASH_PAGE_SIZE);
    if (buffer == NULL) {
        /* Not enough memory */
        return -2;
    }

    /* Unlock related banks and check for keeping boundaries */
    if ( (rs=FLASH_Unlock( dst, len)) != FLASH_COMPLETE) {
        /* Unlocking failed for any reason */
        return (int)rs;
    }

    /* Clear All pending flags */
    FLASH_ClearFlag( FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);

    while( (wptr<end) && (rs==FLASH_COMPLETE))
    {
        /* Get page related information */
        sector = (uint8_t*)((uint32_t)wptr&FLASH_PAGE_MASK);
        offset = ((uint32_t)wptr&FLASH_PAGE_OFFS);
        length = ((uint32_t)(end-wptr));
        if (length>FLASH_PAGE_SIZE)
            length=FLASH_PAGE_SIZE;

        if (offset) {
            /* First page to program:
             * Rescue unchanged content to buffer, merge new content
             * and flash buffer instead of src content.
             */
            memcpy( buffer, sector, FLASH_PAGE_SIZE);
            memcpy( buffer+offset, rptr, length);
            /* Program buffer instead of original source */
            flashp = buffer;
        }
        else if (length<FLASH_PAGE_SIZE) {
            /* Last page to program:
             * Rescue unchanged content to buffer, merge new content
             * and flash buffer instead of src content.
             */
            memcpy( buffer, wptr, FLASH_PAGE_SIZE);
            memcpy( buffer, rptr, length);
            /* Program buffer instead of original source */
            flashp = buffer;
        }
        else {
            /* complete sectors are handled by just reading from source */
            flashp = rptr;
        }

        /* Erase and Program Page */
        rs = Stm32FlashErasePage( sector);
        rs = Stm32FlashWritePage( sector, flashp);

        /* Increase source and destination pointers */
        wptr += length;
        rptr += length;
    }

    /* Lock the FLASH again */
    FLASH_Lock(dst, len);

    /* Release the temporary page buffer */
    NutHeapFree(buffer);

    /* Check the corectness of written data */
    wptr = dst;
    rptr = src;
    while((wptr<end) && (rs==FLASH_COMPLETE))
    {
        if((*(volatile uint8_t*)wptr++) != *rptr++)
        {
            rs = FLASH_COMPARE;
        }
    }

    return (int)rs;
}

/*!
 * \brief Nut/OS specific handling for parameters in FLASH.
 *
 * This function enables to read system specific parameters
 * from processors FLASH. The sectors used for storage are
 * configureable via nutconf.
 *
 * \param pos Offset of parameter(s) in configured page(s).
 * \param data Pointer where to copy data from flash to.
 * \param len Number of bytes to be copied.
 *
 * \return FLASH_Status.
 */
int Stm32FlashParamRead(unsigned int pos, void *data, size_t len)
{
    FLASH_Status rs = FLASH_BOUNDARY;
    uint8_t *addr = (uint8_t *)(FLASH_CONF_SECTOR + pos);
    uint8_t *max = (uint8_t *)(FLASH_CONF_SECTOR+FLASH_CONF_SIZE);

    /* Boundary checks */
    if (addr+len > max) {
        return rs;
    }
    else {
        rs = Stm32FlashRead( data, addr, len);
    }
    /* Return success or fault code */
    return (int)rs;
}

/*!
 * \brief Nut/OS specific handling for parameters in FLASH.
 *
 * This function enables to store system specific parameters
 * in processors FLASH. The sectors used for storage are
 * configureable via nutconf.
 * FLASH is only updated if content differs.
 *
 * \param pos Offset of parameter(s) in configured page(s).
 * \param data Pointer to source data.
 * \param len Number of bytes to be written.
 *
 * \return FLASH_Status.
 */
int Stm32FlashParamWrite(unsigned int pos, const void *data, size_t len)
{
    FLASH_Status rs = 0;
    uint8_t *addr = (uint8_t*)(FLASH_CONF_SECTOR+pos);
    uint8_t *max  = (uint8_t*)(FLASH_CONF_SECTOR+FLASH_CONF_SIZE);

    /* Boundary checks */
    if (addr+len < max)
    {
     /* Check if content needs update. */
        if (memcmp(addr, data, len))
        {
            rs = Stm32FlashWrite( addr, (void*)data, len);
        }
    }
    /* Return success or fault code */
    return (int)rs;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
