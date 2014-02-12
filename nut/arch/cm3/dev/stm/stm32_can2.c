/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: stm32_can2.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/can_dev.h>
#include <cfg/arch/gpio.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/canbus.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32xxxx_gpio.h>
#include <arch/cm3/stm/stm32xxxx_rcc.h>

#ifndef CANBUS2_REMAP_CAN
#define CANBUS2_REMAP_CAN 0
#endif

/*!
 * \brief CANBUS2 GPIO configuartion and assignment.
 */
#define CAN2_GPIO_PORT   NUTGPIO_PORTB
#if defined(MCU_STM32F1)
 #if (CANBUS2_REMAP_CAN == 1)
  #define CANBUS_REMAP       1
  #define CAN2RX_GPIO_PIN    5
  #define CAN2TX_GPIO_PIN    6
 #else
  #define CANBUS_REMAP       0
  #define CAN2RX_GPIO_PIN    12
  #define CAN2TX_GPIO_PIN    13
 #endif
#else /*L1/F2/F4*/
 #if !defined(CAN2_TX_PIN)
  #if (CANBUS2_REMAP_CAN == 0)
   #define CAN2TX_GPIO_PIN   13
  #elif (CANBUS2_REMAP_CAN == 1)
   #define CAN2TX_GPIO_PIN   6
  #else
   #error "Illegal CANBUS2_REMAP_CAN value"
  #endif
 #elif (CAN2_TX_PIN == 13)
  #define CAN2TX_GPIO_PIN   13
 #elif (CAN2_TX_PIN == 6)
  #define CAN2TX_GPIO_PIN    6
 #else
  #errror "Illegal CAN2 TX value"
 #endif
 #if !defined(CAN2_RX_PIN)
  #if (CANBUS2_REMAP_CAN == 0)
   #define CAN2RX_GPIO_PIN   12
  #elif (CANBUS2_REMAP_CAN == 1)
   #define CAN2RX_GPIO_PIN    5
  #else
   #error "Illegal CANBUS2_REMAP_CAN value"
  #endif
 #elif (CAN2_RX_PIN == 12)
  #define CAN2RX_GPIO_PIN   12
 #elif (CAN2_RX_PIN == 5)
  #define CAN2RX_GPIO_PIN    5
 #else
  #errror "Illegal CAN2 RX value"
 #endif
#endif

/*!
 * \brief Processor specific Hardware Initiliaization
 *
 */
int Stm32CanHw2Init(void)
{
    __IO uint32_t *rcc_bb = CM3BB_BASE(RCC_BASE);
#if defined (CAN2_ACCEPTANCE_FILTERS)
    uint32_t rc;
#endif

    if (!rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_CAN1EN))])
    {
        /* CAN1 has no clock, unconditionally clock CAN 1 */
        rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_CAN1EN))]=1;
        /* Reset CAN Bus 1 IP */
        rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN1RST))] = 1;
        rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN1RST))] = 0;
#if defined (CAN2_ACCEPTANCE_FILTERS)
        /* Set the CAN1/CAN2 Filter split */
        rc =CAN1->FMR;
        rc &= ~0x3f00;
        rc |= CAN2_ACCEPTANCE_FILTERS<<8;
        CAN1->FMR = rc;
#endif
    }

    /* Enable CAN Bus 2 peripheral clock. */
    rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_CAN2EN))]=1;

    /* Reset CAN Bus 2 IP */
    rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN2RST))]=1;
    rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN2RST))]=0;

    /* Setup Related GPIOs. */
    GpioPinConfigSet(CAN2_GPIO_PORT, CAN2RX_GPIO_PIN, GPIO_CFG_PULLUP|GPIO_CFG_PERIPHAL);
    GpioPinConfigSet(CAN2_GPIO_PORT, CAN2TX_GPIO_PIN, GPIO_CFG_OUTPUT|GPIO_CFG_PERIPHAL);

#if defined (MCU_STM32F1)
    CM3BBREG(AFIO_BASE, AFIO_TypeDef, MAPR, _BI32(AFIO_MAPR_CAN2_REMAP)) = CANBUS_REMAP;
#else
    GPIO_PinAFConfig((GPIO_TypeDef*) CAN2_GPIO_PORT, CAN2RX_GPIO_PIN, GPIO_AF_CAN2);
    GPIO_PinAFConfig((GPIO_TypeDef*) CAN2_GPIO_PORT, CAN2TX_GPIO_PIN, GPIO_AF_CAN2);
#endif

    return 0;
}

NUTCANBUS Stm32CanBus2 = {
    CAN2_BASE,
    CM3BB_BASE(CAN2_BASE),
    &sig_CAN2_RX0,
    &sig_CAN2_TX,
    &sig_CAN2_SCE,
    0,
    Stm32CanHw2Init,
};

NUTCANBUS Stm32CanBus2C = {
    CAN2_BASE,
    CM3BB_BASE(CAN2_BASE),
    &sig_CAN2_RX1,
    0,
    0,
    0,
    0,
};


