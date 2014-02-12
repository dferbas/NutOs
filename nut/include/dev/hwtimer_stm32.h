/*
 * Copyright (C) 2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: hwtimer_stm32.c 3731 2012-01-12 18:23:31Z olereinhardt $
 * \endverbatim
 */

#include <stddef.h>
#include <string.h>
#include <cfg/arch.h>
#include <arch/arm/cm3.h>
#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32xxxx_rcc.h>

#define NUTTIMER1   TIM1_BASE
#define NUTTIMER2   TIM2_BASE
#define NUTTIMER3   TIM3_BASE
#define NUTTIMER4   TIM4_BASE
#define NUTTIMER5   TIM5_BASE
#define NUTTIMER6   TIM6_BASE
#define NUTTIMER7   TIM7_BASE
#define NUTTIMER8   TIM8_BASE
#define NUTTIMER9   TIM9_BASE
#define NUTTIMER10  TIM10_BASE
#define NUTTIMER11  TIM11_BASE
#define NUTTIMER12  TIM12_BASE
#define NUTTIMER13  TIM13_BASE
#define NUTTIMER14  TIM14_BASE
#define NUTTIMER15  TIM15_BASE
#define NUTTIMER16  TIM16_BASE
#define NUTTIMER17  TIM17_BASE

#if defined(MCU_STM32F1)
#define  TIM_Init(timer) \
    ((timer) == (NUTTIMER2))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM2)) = 1)) \
    :((timer) == (NUTTIMER3))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM3)) = 1)) \
    :((timer) == (NUTTIMER4))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM4)) = 1)) \
    :((timer) == (NUTTIMER5))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM5)) = 1)) \
    :((timer) == (NUTTIMER6))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM6)) = 1)) \
    :((timer) == (NUTTIMER7))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM7)) = 1)) :-1

#define TIM_NVIC_EnableIRQ(timer)                            \
    ((timer) == (NUTTIMER2))?NVIC_EnableIRQ(TIM2_IRQn)       \
    :((timer) == (NUTTIMER3))?NVIC_EnableIRQ(TIM3_IRQn)      \
    :((timer) == (NUTTIMER4))?NVIC_EnableIRQ(TIM4_IRQn)      \
    :((timer) == (NUTTIMER5))?NVIC_EnableIRQ(TIM5_IRQn)      \
    :((timer) == (NUTTIMER6))?NVIC_EnableIRQ(TIM6_IRQn)      \
    :((timer) == (NUTTIMER7))?NVIC_EnableIRQ(TIM7_IRQn):0

#define TIM_NVIC_DisableIRQ(timer)                             \
    ((timer) == (NUTTIMER2))?NVIC_DisableIRQ(TIM2_IRQn)       \
    :((timer) == (NUTTIMER3))?NVIC_DisableIRQ(TIM3_IRQn)      \
    :((timer) == (NUTTIMER4))?NVIC_DisableIRQ(TIM4_IRQn)      \
    :((timer) == (NUTTIMER5))?NVIC_DisableIRQ(TIM5_IRQn)      \
    :((timer) == (NUTTIMER6))?NVIC_DisableIRQ(TIM6_IRQn)      \
    :((timer) == (NUTTIMER7))?NVIC_DisableIRQ(TIM7_IRQn):0

#define TIM_Cortex_RegisterInt(timer, pfnHandler) \
    ((timer) == (NUTTIMER2))?Cortex_RegisterInt(TIM2_IRQn, pfnHandler)         \
    :((timer) == (NUTTIMER3))?Cortex_RegisterInt(TIM3_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER4))?Cortex_RegisterInt(TIM4_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER5))?Cortex_RegisterInt(TIM5_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER6))?Cortex_RegisterInt(TIM6_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER7))?Cortex_RegisterInt(TIM7_IRQn, pfnHandler):0

#define TIM_ClockVal(timer) \
    ((timer) == (NUTTIMER2))?NutClockGet(NUT_HWCLK_PCLK1)         \
    :((timer) == (NUTTIMER3))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER4))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER5))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER6))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER7))?:0
#elif defined(MCU_STM32L1)
#define  TIM_Init(timer) \
    ((timer) == (NUTTIMER2))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM2)) = 1)) \
    :((timer) == (NUTTIMER3))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM3)) = 1)) \
    :((timer) == (NUTTIMER4))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM4)) = 1)) \
    :((timer) == (NUTTIMER6))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM6)) = 1)) \
    :((timer) == (NUTTIMER7))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM7)) = 1)) :-1

#define TIM_NVIC_EnableIRQ(timer)                            \
    ((timer) == (NUTTIMER2))?NVIC_EnableIRQ(TIM2_IRQn)       \
    :((timer) == (NUTTIMER3))?NVIC_EnableIRQ(TIM3_IRQn)      \
    :((timer) == (NUTTIMER4))?NVIC_EnableIRQ(TIM4_IRQn)      \
    :((timer) == (NUTTIMER6))?NVIC_EnableIRQ(TIM6_IRQn)      \
    :((timer) == (NUTTIMER7))?NVIC_EnableIRQ(TIM7_IRQn):0

#define TIM_NVIC_DisableIRQ(timer)                            \
    ((timer) == (NUTTIMER2))?NVIC_DisableIRQ(TIM2_IRQn)       \
    :((timer) == (NUTTIMER3))?NVIC_DisableIRQ(TIM3_IRQn)      \
    :((timer) == (NUTTIMER4))?NVIC_DisableIRQ(TIM4_IRQn)      \
    :((timer) == (NUTTIMER6))?NVIC_DisableIRQ(TIM6_IRQn)      \
    :((timer) == (NUTTIMER7))?NVIC_DisableIRQ(TIM7_IRQn):0

#define TIM_Cortex_RegisterInt(timer, pfnHandler) \
    ((timer) == (NUTTIMER2))?Cortex_RegisterInt(TIM2_IRQn, pfnHandler)         \
    :((timer) == (NUTTIMER3))?Cortex_RegisterInt(TIM3_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER4))?Cortex_RegisterInt(TIM4_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER6))?Cortex_RegisterInt(TIM6_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER7))?Cortex_RegisterInt(TIM7_IRQn, pfnHandler):0

#define TIM_ClockVal(timer) \
    ((timer) == (NUTTIMER2))?NutClockGet(NUT_HWCLK_PCLK1)         \
    :((timer) == (NUTTIMER3))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER4))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER6))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER7))?:0
#elif defined(MCU_STM32F2) || defined(MCU_STM32F4)
#define  TIM_Init(timer) \
    ((timer) == (NUTTIMER2))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM2)) = 1)) \
    :((timer) == (NUTTIMER3))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM3)) = 1)) \
    :((timer) == (NUTTIMER4))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM4)) = 1)) \
    :((timer) == (NUTTIMER5))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM5)) = 1)) \
    :((timer) == (NUTTIMER6))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM6)) = 1)) \
    :((timer) == (NUTTIMER7))?((CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1Periph_TIM7)) = 1)) :-1

#define TIM_NVIC_EnableIRQ(timer)                            \
    ((timer) == (NUTTIMER2))?NVIC_EnableIRQ(TIM2_IRQn)       \
    :((timer) == (NUTTIMER3))?NVIC_EnableIRQ(TIM3_IRQn)      \
    :((timer) == (NUTTIMER4))?NVIC_EnableIRQ(TIM4_IRQn)      \
    :((timer) == (NUTTIMER5))?NVIC_EnableIRQ(TIM5_IRQn)      \
    :((timer) == (NUTTIMER6))?NVIC_EnableIRQ(TIM6_DAC_IRQn)  \
    :((timer) == (NUTTIMER7))?NVIC_EnableIRQ(TIM7_IRQn):0

#define TIM_NVIC_DisableIRQ(timer)                              \
    ((timer) == (NUTTIMER2))?NVIC_DisableIRQ(TIM2_IRQn)       \
    :((timer) == (NUTTIMER3))?NVIC_DisableIRQ(TIM3_IRQn)      \
    :((timer) == (NUTTIMER4))?NVIC_DisableIRQ(TIM4_IRQn)      \
    :((timer) == (NUTTIMER5))?NVIC_DisableIRQ(TIM5_IRQn)      \
    :((timer) == (NUTTIMER6))?NVIC_DisableIRQ(TIM6_DAC_IRQn)  \
    :((timer) == (NUTTIMER7))?NVIC_DisableIRQ(TIM7_IRQn):0

#define TIM_Cortex_RegisterInt(timer, pfnHandler) \
    ((timer) == (NUTTIMER2))?Cortex_RegisterInt(TIM2_IRQn, pfnHandler)         \
    :((timer) == (NUTTIMER3))?Cortex_RegisterInt(TIM3_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER4))?Cortex_RegisterInt(TIM4_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER5))?Cortex_RegisterInt(TIM5_IRQn, pfnHandler)        \
    :((timer) == (NUTTIMER6))?Cortex_RegisterInt(TIM6_DAC_IRQn, pfnHandler)    \
    :((timer) == (NUTTIMER7))?Cortex_RegisterInt(TIM7_IRQn, pfnHandler):0

#define TIM_ClockVal(timer) \
    ((timer) == (NUTTIMER2))?NutClockGet(NUT_HWCLK_PCLK1)         \
    :((timer) == (NUTTIMER3))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER4))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER5))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER6))?NutClockGet(NUT_HWCLK_PCLK1)        \
    :((timer) == (NUTTIMER7))?:0
#else
#warning "Unknown STM32 family"
#endif

#define TIM_AutoReloadValue( timer )       CM3REG(timer, TIM_TypeDef, ARR )
#define TIM_Prescaler( timer)              CM3REG(timer, TIM_TypeDef, PSC )
#define TIM_Compare1( timer)               CM3REG(timer, TIM_TypeDef, CCR1)
#define TIM_Compare2( timer)               CM3REG(timer, TIM_TypeDef, CCR2)
#define TIM_Compare3( timer)               CM3REG(timer, TIM_TypeDef, CCR3)
#define TIM_Compare4( timer)               CM3REG(timer, TIM_TypeDef, CCR4)
#define TIM_Counter( timer)                CM3REG(timer, TIM_TypeDef, CNT )
#define TIM_Clear(timer)                   {uint32_t len =  sizeof(TIM_TypeDef); while(len) {((uint32_t*)timer)[len] = 0; len -=4;} }
#define TIM_IRQEnable( timer )             CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_UIE  )) = 1
#define TIM_IRQEnable( timer )             CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_UIE  )) = 1
#define TIM_DISEnable( timer )             CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_UIE  )) = 0
#define TIM_C1IRQEnable( timer )           CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC1IE)) = 1
#define TIM_C1IRQDisable( timer )          CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC1IE)) = 0
#define TIM_C2IRQEnable( timer )           CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC2IE)) = 1
#define TIM_C2IRQDisable( timer )          CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC2IE)) = 0
#define TIM_C3IRQEnable( timer )           CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC3IE)) = 1
#define TIM_C3IRQDisable( timer )          CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC3IE)) = 0
#define TIM_C4IRQEnable( timer )           CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC4IE)) = 1
#define TIM_C4IRQDisable( timer )          CM3BBREG(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC4IE)) = 0
#define TIM_StartTimer( timer)             CM3BBREG(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_CEN   )) = 1
#define TIM_StopTimer( timer )             CM3BBREG(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_CEN   )) = 0
#define TIM_OnePulse( timer)               CM3BBREG(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_OPM   )) = 1
#define TIM_ContPulse( timer)              CM3BBREG(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_OPM   )) = 0
#define TIM_Update( timer )                CM3BBREG(timer, TIM_TypeDef, EGR , _BI16(TIM_EGR_UG    )) = 1
#define TIM_ClearInterruptFlag( timer)     CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_UIF    )) = 0
#define TIM_C1ClearInterruptFlag( timer )  CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC1IF  )) = 0
#define TIM_C2ClearInterruptFlag( timer )  CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC2IF  )) = 0
#define TIM_C3ClearInterruptFlag( timer )  CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC3IF  )) = 0
#define TIM_C4ClearInterruptFlag( timer )  CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC4IF  )) = 0
#define TIM_C1InterruptFlag( timer )      (CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC1IF  )) != 0)
#define TIM_C2InterruptFlag( timer )      (CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC2IF  )) != 0)
#define TIM_C3InterruptFlag( timer )      (CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC3IF  )) != 0)
#define TIM_C4InterruptFlag( timer )      (CM3BBREG(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC4IF  )) != 0)
