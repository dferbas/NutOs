/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * (C) 2011 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de
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

#include <arch/cm3/stm/stm32l1xx.h>


/* Prepare some defaults if configuration is incomplete */
#if !defined(SYSCLK_SOURCE)
#define SYSCLK_SOURCE SYSCLK_HSI
#endif

uint32_t SystemCoreClock;
const uint32_t MSIFreqTable[8] = {65536, 131072, 262144,  524288, 1048000, 2097000, 4194000, 0};

/*----------------  Clock Setup Procedure ------------------------------
 *
 * Clock system ist arranged like this:
 *
 *                            ,--------------------------- USB
 *                            |           ,--------------- CPU
 *                            |           +--------------- SDIO
 * 1-32MHz HSE ----+---PLLMUL-+-AHBPRES---+-- APB1PRESC--- APB1
 *                 |          |           +-- ABP2PRESC--- ABP2
 * 16MHz HSI ------+----------'-------------- ADCPRESC---- ADC
 *                            |
 *       MSI -----------------'
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
    uint32_t pllmull = 0, plldiv = 0, msirange;
    uint32_t rcc;

    rcc = RCC->CFGR;

    /* Get SYSCLK source -------------------------------------------------------*/
    switch (rcc & RCC_CFGR_SWS)
    {
        case RCC_CFGR_SWS_MSI:  /* MSI used as system clock , value depends on RCC_ICSCR/MSIRANGE[2:0]: */
            msirange = RCC->ICSCR & RCC_ICSCR_MSIRANGE ;
            msirange = msirange>>_BI16(RCC_ICSCR_MSIRANGE_1);
            SystemCoreClock = MSIFreqTable[msirange];
            break;
        case RCC_CFGR_SWS_HSI:  /* HSI used as system clock */
            SystemCoreClock = HSI_VALUE;
            break;
        case RCC_CFGR_SWS_HSE:  /* HSE used as system clock */
            SystemCoreClock = HSE_VALUE;
            break;
        case RCC_CFGR_SWS_PLL:
            /* Assume that values not allowed don't occure*/
            if      (rcc & RCC_CFGR_PLLMUL4)  pllmull =  4;
            else if (rcc & RCC_CFGR_PLLMUL6)  pllmull =  6;
            else if (rcc & RCC_CFGR_PLLMUL8)  pllmull =  8;
            else if (rcc & RCC_CFGR_PLLMUL12) pllmull = 12;
            else if (rcc & RCC_CFGR_PLLMUL16) pllmull = 16;
            else if (rcc & RCC_CFGR_PLLMUL24) pllmull = 24;
            else if (rcc & RCC_CFGR_PLLMUL32) pllmull = 32;
            else if (rcc & RCC_CFGR_PLLMUL48) pllmull = 48;
            else                              pllmull =  3;

            if((rcc & RCC_CFGR_PLLDIV4) == RCC_CFGR_PLLDIV4) plldiv = 4;
            else if (rcc & RCC_CFGR_PLLDIV3)                 plldiv = 3;
            else if (rcc & RCC_CFGR_PLLDIV2)                 plldiv = 2;
            else                                             plldiv = 1;

            if (rcc & RCC_CFGR_PLLSRC_HSE)
                SystemCoreClock = HSE_VALUE * pllmull / plldiv;
            else
                SystemCoreClock = HSI_VALUE * pllmull / plldiv;

    }

    /* Compute HCLK clock frequency ----------------*/
    if ((rcc & RCC_CFGR_HPRE_3))
        SystemCoreClock >>= ((rcc & (RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_1 |RCC_CFGR_HPRE_2)) +1);
}

/* Functional same as F1 */
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
        /* Enable HSE */
        RCC->CR |= RCC_CR_HSEON;

        /* Wait till HSE is ready or time out is reached */
        do {
            tout--;
            HSEStatus = RCC->CR & RCC_CR_HSERDY;
        } while((HSEStatus == 0) && (tout > 0));

        if ((RCC->CR & RCC_CR_HSERDY) == RESET) {
            /* HSE failed to start */
            rc = -1;
        }
    }
    else {
        /* Disable HSE clock */
        RCC->CR &= ~RCC_CR_HSEON;
    }

    return rc;
}

/* Functional same as F1 */
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
        /* Enable HSI */
        RCC->CR |= RCC_CR_HSION;

        /* Wait till HSI is ready or time out is reached */
        do {
            tout--;
            HSIStatus = RCC->CR & RCC_CR_HSIRDY;
        } while((HSIStatus == 0) && (tout > 0));

        if ((RCC->CR & RCC_CR_HSIRDY) == RESET) {
            /* HSI failed to start */
            rc = -1;
        }
    }
    else {
        /* Disable HSE clock */
        RCC->CR &= ~RCC_CR_HSION;
    }

    return rc;
}

/* Functional same as F1 */
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
            tout--;
            PLLStatus = RCC->CR & RCC_CR_PLLRDY;
        } while((PLLStatus == 0) && (tout > 0));

        if ((RCC->CR & RCC_CR_PLLRDY) == RESET) {
            /* PLL failed to start */
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
 * \note   This function should be used with PLL disables
 * \param  src is one of PLLCLK_HSE, PLLCLK_HSI.
 * \return 0 if clock is running ale -1.
 */
int SetPllClockSource( int src)
{
    int rc = -1;
    if (src == PLLCLK_HSE) {
        rc = CtlHseClock(ENABLE);
        if (rc==0) {
            CM3BBREG(RCC_BASE, RCC_TypeDef, CFGR, _BI32(RCC_CFGR_PLLSRC)) = 1;
        }
    }
    else if (src == PLLCLK_HSI) {
        rc = CtlHsiClock(ENABLE);
        /* Select HSI/2 as PLL clock source */
        if (rc==0) {
            CM3BBREG(RCC_BASE, RCC_TypeDef, CFGR, _BI32(RCC_CFGR_PLLSRC)) = 0;
        }
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

    /* Fixme: Set MSI source with MSI frequency parameter */
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
    else if (src == SYSCLK_HSE) {
        rc = CtlHseClock(ENABLE);
        if (rc == 0) {
            /* Select HSI as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_HSE;

            /* Wait till HSI is used as system clock source */
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
        }
    }
    else if (src == SYSCLK_PLL) {
        rc = CtlPllClock(ENABLE);
        if (rc == 0) {
            /* Select HSI as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_PLL;

            /* Wait till HSI is used as system clock source */
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

    /* Fixme: Allow more flexible Flash Setting
     * For the moment, use 32-bit access with no prefetch . Latency has no meaning
     * for 32-bit access
     */
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
#define PLLCLK_IN (HSI_VALUE)
#endif

/**
  * @brief  Sets System clock frequency to 8MHz and configure HCLK, PCLK2
  *          and PCLK1 prescalers.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
int SetSysClock(void)
{
    int rc = 0;
    /* FIXME*/
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

