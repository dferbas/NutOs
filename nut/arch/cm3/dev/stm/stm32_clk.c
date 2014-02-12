/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 * $Id: stm32_gpio.c 3182 2010-10-17 21:46:04Z Astralix $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <arch/cm3.h>

#include <arch/cm3/stm/stm32_clk.h>
#include <cfg/clock.h>

#if defined(MCU_STM32F1)
#include <arch/cm3/stm/stm32f10x.h>
#else
#warning "Unknown STM32 family"
#endif


/* Prepare some defaults if configuration is incomplete */
#if !defined(SYSCLK_SOURCE)
#define SYSCLK_SOURCE SYSCLK_HSI
#endif

uint32_t SystemCoreClock;
static const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

/*----------------  Clock Setup Procedure ------------------------------
 *
 * Clock system ist arranged like this:
 *
 *                            ,--------------------------- USB
 *                            |           ,--------------- CPU
 *                            |           +--------------- SDIO
 * 4-16/25MHz HSE -+---PLLMUL-+-AHBPRES---+-- APB1PRESC--- APB1
 *                 |          |           +-- ABP2PRESC--- ABP2
 * 8MHz HSI -------+----------'           '-- ADCPRESC---- ADC
 *
 *
 *        ***** Setup of system clock configuration *****
 *
 * 1) Select system clock sources
 *
 * To setup system to use HSI call: SetSysClockSource( SYSCLK_HSI);
 * To setup system to use HSE call: SetSysClockSource( SYSCLK_HSE);
 *
 * To setup system to use the PLL output, first setup the PLL source:
 * SetPllClockSource( PLLCLK_HSI);
 * or
 * SetPllClockSource( PLLCLK_HSE);
 * Then call SetSysClockSource( SYSCLK_PLL);
 *
 * 2) Configure prescalers
 * After selecting the right clock sources, the prescalers need to
 * be configured:
 * Call SetSysClock(); to do this automatically.
 *
 */


/*!
 * \brief  Update SystemCoreClock according to Clock Register Values
 *
 * This function reads out the CPUs clock and PLL registers and assembles
 * the actual clock speed values into the SystemCoreClock global variable.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0, pllmull = 0, pllsource = 0;

#ifdef  STM32F10X_CL
    uint32_t prediv1source = 0, prediv1factor = 0, prediv2factor = 0, pll2mull = 0;
#endif /* STM32F10X_CL */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)
    uint32_t prediv1factor = 0;
#endif /* STM32F10X_LD_VL or STM32F10X_MD_VL */

    /* Get SYSCLK source -------------------------------------------------------*/
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
        case 0x00:  /* HSI used as system clock */
            SystemCoreClock = HSI_VALUE;
            break;
        case 0x04:  /* HSE used as system clock */
            SystemCoreClock = HSE_VALUE;
            break;
        case 0x08:  /* PLL used as system clock */
            /* Get PLL clock source and multiplication factor ----------------------*/
            pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
            pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

#ifndef STM32F10X_CL
            pllmull = ( pllmull >> 18) + 2;

            if (pllsource == 0x00) {
                /* HSI oscillator clock divided by 2 selected as PLL clock entry */
                SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
            }
            else {
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)
                prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
                /* HSE oscillator clock selected as PREDIV1 clock entry */
                SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;
#else
                /* HSE selected as PLL clock entry */
                if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET) {
                    /* HSE oscillator clock divided by 2 */
                    SystemCoreClock = (HSE_VALUE >> 1) * pllmull;
                }
                else {
                    SystemCoreClock = HSE_VALUE * pllmull;
                }
#endif
            }
#else
            pllmull = pllmull >> 18;

            if (pllmull != 0x0D) {
                pllmull += 2;
            }
            else {
                /* PLL multiplication factor = PLL input clock * 6.5 */
                pllmull = 13 / 2;
            }

            if (pllsource == 0x00) {
                /* HSI oscillator clock divided by 2 selected as PLL clock entry */
                SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
            }
            else {
                /* PREDIV1 selected as PLL clock entry */

                /* Get PREDIV1 clock source and division factor */
                prediv1source = RCC->CFGR2 & RCC_CFGR2_PREDIV1SRC;
                prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;

                if (prediv1source == 0) {
                    /* HSE oscillator clock selected as PREDIV1 clock entry */
                    SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;
                }
                else {
                    /* PLL2 clock selected as PREDIV1 clock entry */

                    /* Get PREDIV2 division factor and PLL2 multiplication factor */
                    prediv2factor = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> 4) + 1;
                    pll2mull = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> 8 ) + 2;
                    SystemCoreClock = (((HSE_VALUE / prediv2factor) * pll2mull) / prediv1factor) * pllmull;
                }
            }
#endif /* STM32F10X_CL */
            break;

        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    /* Compute HCLK clock frequency ----------------*/
    /* Get HCLK prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    /* HCLK clock frequency */
    SystemCoreClock >>= tmp;
}


/*!
 * \brief Control HSE clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on HSE start failed.
 */
int CtlHseClock( uint8_t ena)
{
    int rc = 0;
    uint32_t tout = HSE_STARTUP_TIMEOUT;
    volatile uint32_t HSEStatus = 0;

    if( ena) {
        /* Enable HSE clock */
        RCC->CR |= ((uint32_t)RCC_CR_HSEON);

        /* Wait till HSE is ready or time out is reached */
        do {
            HSEStatus = RCC->CR & RCC_CR_HSERDY;
            tout--;
        } while((HSEStatus == 0) && (tout > 0));

        if ((RCC->CR & RCC_CR_HSERDY) == RESET) {
            /* HSE failed to start */
            rc = -1;
        }
    }
    else {
        /* Disable HSE clock */
        RCC->CR &= ~RCC_CR_HSION;
    }

    return rc;
}

/*!
 * \brief Control HSI clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on HSI start failed.
 */
int CtlHsiClock( uint8_t ena)
{
    int rc = 0;
    uint32_t tout = HSE_STARTUP_TIMEOUT;
    volatile uint32_t HSIStatus = 0;

    if( ena) {
        /* Enable HSE clock */
        RCC->CR |= ((uint32_t)RCC_CR_HSION);

        /* Wait till HSE is ready or time out is reached */
        do {
            HSIStatus = RCC->CR & RCC_CR_HSIRDY;
            tout--;
        } while((HSIStatus == 0) && (tout > 0));

        if ((RCC->CR & RCC_CR_HSIRDY) == RESET) {
            /* HSE failed to start */
            rc = -1;
        }
    }
    else {
        /* Disable HSI clock */
        RCC->CR &= ~RCC_CR_HSION;
    }

    return rc;
}

/*!
 * \brief Control PLL clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on PLL start failed.
 */
int CtlPllClock( uint8_t ena)
{
    int rc = 0;

    uint32_t tout = HSE_STARTUP_TIMEOUT;
    volatile uint32_t PLLStatus = 0;

    if( ena) {
        /* Enable PLL */
        RCC->CR |= RCC_CR_PLLON;

        /* Wait till PLL is ready or time out is reached */
        do {
            PLLStatus = RCC->CR & RCC_CR_PLLRDY;
            tout--;
        } while((PLLStatus == 0) && (tout > 0));

        if ((RCC->CR & RCC_CR_PLLRDY) == RESET) {
            /* HSE failed to start */
            rc = -1;
        }
    }
    else {
        /* Disable HSE clock */
        RCC->CR &= ~RCC_CR_PLLON;
    }

    return rc;
}


/*!
 * \brief  Configures the System clock source: HSE or HSI.
 * \note   This function should be used only after reset.
 * \param  src is one of PLLCLK_HSE, PLLCLK_HSI.
 * \return 0 if clock is running ale -1.
 */
int SetPllClockSource( int src)
{
    int rc = -1;

    if (src == PLLCLK_HSE) {
        rc = CtlHseClock(ENABLE);
        if (rc==0) {
            /* Select HSE/2 as PLL clock source */
            RCC->CFGR |= RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE;
        }
    }
    else if (src == PLLCLK_HSI) {
        rc = CtlHsiClock(ENABLE);
        /* Select HSI/2 as PLL clock source */
            RCC->CFGR &= ~(RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE);
    }

    return rc;
}

/*!
 * \brief  Configures the System clock source: HSI, HS or PLL.
 * \note   This function should be used only after reset.
 * \param  src is one of SYSCLK_HSE, SYSCLK_HSI or SYSCLK_PLL.
 * \return 0 if selected clock is running else -1.
 */
int SetSysClockSource( int src)
{
    int rc = -1;

    if (src == SYSCLK_HSE) {
        rc = CtlHseClock(ENABLE);
        if (rc == 0) {
            /* Select HSE as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_HSE;

            /* Wait till HSE is used as system clock source */
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
        }
    }
    else if (src == SYSCLK_HSI) {
        rc = CtlHsiClock(ENABLE);
        if (rc == 0) {
            /* Select HSI as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_HSI;

            /* Wait till HSI is used as system clock source */
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
        }
    }
    else if (src == SYSCLK_PLL) {
        rc = CtlPllClock(ENABLE);
        if (rc == 0) {
            /* Select HSI as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_PLL;

            /* Wait till PLL is used as system clock source */
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
        }
    }

    /* Update core clock information */
    SystemCoreClockUpdate();

    return rc;
}
#if (SYSCLK_SOURCE == SYSCLK_HSI) || (SYSCLK_SOURCE == SYSCLK_HSE)
/*!
 * \brief  Configures the System clock coming from HSE or HSI oscillator.
 *
 * Enable HSI/HSE clock and setup HCLK, PCLK2 and PCLK1 prescalers.
 *
 * \param  None.
 * \return 0 on success, -1 on fault of HSE.
 */
int SetSysClock(void)
{
    int rc = 0;
    register uint32_t cfgr;

#if !defined(STM32F10X_LD_VL) && !defined(STM32F10X_MD_VL)
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 0 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);

#if (SYSCLK_SOURCE == SYSCLK_HSE)
    if (HSE_VALUE > 24000000) {
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_1;
    }
#endif
#endif

    cfgr = RCC->CFGR;

    cfgr &= ~(RCC_CFGR_HPRE|RCC_CFGR_PPRE1|RCC_CFGR_PPRE2);

    /* HCLK = SYSCLK */
    cfgr |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK */
    cfgr |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

    /* PCLK1 = HCLK */
    cfgr |= (uint32_t)RCC_CFGR_PPRE1_DIV1;

    RCC->CFGR = cfgr;

    rc = SetSysClockSource(SYSCLK_SOURCE);

    return rc;
}

#else /* (SYSCLK_SOURCE == SYSCLK_HSI) || (SYSCLK_SOURCE == SYSCLK_HSE) */

#if (PLLCLK_SOURCE==PLLCLK_HSE)
#define PLLCLK_IN HSE_VALUE
#else
#define PLLCLK_IN (HSI_VALUE/2)
#endif

/**
  * @brief  Sets System clock frequency to 72MHz and configure HCLK, PCLK2
  *          and PCLK1 prescalers.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
int SetSysClock(void)
{
    int rc = 0;
    uint32_t Multiplier = SYSCLK_FREQ / (PLLCLK_IN>>1);
    uint32_t DividerX2 = RCC_CFGR_PLLXTPRE;
    uint32_t FlashWs = 0;

    /* If half of HSE_VALUE is to small for reaching target frequency
     * deactivate /2 divider. */
    if (Multiplier > 16) {
        DividerX2 = 0;
        Multiplier = SYSCLK_FREQ / PLLCLK_IN;
    }

    /* Check Limits */
    if (DividerX2) {
        if (((PLLCLK_IN>>1)*Multiplier) > 72000000UL) Multiplier--;
    }
    else {
        if ((PLLCLK_IN*Multiplier) > 72000000UL) Multiplier--;
    }

    /* Shift multiplier to corresponding RCC_CFGR register bits */
    Multiplier = ((Multiplier-2)&0xF)<<18;

    /* Enable PLL clock source */
    rc = SetPllClockSource(PLLCLK_SOURCE);
    if (rc) {
        /* Something went wrong with the PLL startup! */
        SetSysClockSource(SYSCLK_HSI);
        return rc;
    }

    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash wait state calculation and setup */
    if( SYSCLK_FREQ > 24000000)
        FlashWs++;
    if( SYSCLK_FREQ > 48000000)
        FlashWs++;

    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FlashWs;

    /* HCLK = SYSCLK */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK */
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    if( SYSCLK_FREQ > 36000000)
        /* PCLK1 = HCLK/2 */
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    else
        /* PCLK1 = HCLK */
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;

#ifdef STM32F10X_CL
    /*  PLL configuration for Conetctivity Line Devices */

    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */

    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                          RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                         RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);

    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0);

    /* PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz */
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    // TODO: Calculate online
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 |
                        RCC_CFGR_PLLMULL9);
#elif defined(STM32F10X_MD_VL) || defined (STM32F10X_LD_VL)
    /*  PLL configuration for other devices */
    RCC->CFGR &= (uint32_t)(~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | Multiplier | DividerX2);
#else
    /*  PLL configuration for other devices */
    RCC->CFGR &= (uint32_t)(~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | Multiplier | DividerX2);
#endif /* STM32F10X_CL */

    /* Start PLL, wait ready and switch to it as clock source */
    rc = SetSysClockSource(SYSCLK_SOURCE);
    if (rc) {
        /* Something went wrong with the PLL startup! */
        SetSysClockSource(SYSCLK_HSI);
        return rc;
    }

    return rc;
}
#endif /* (SYSCLK_SOURCE == SYSCLK_HSI) || (SYSCLK_SOURCE == SYSCLK_HSE) */

/**
  * @brief  requests System clock frequency
  *
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
uint32_t SysCtlClockGet(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

