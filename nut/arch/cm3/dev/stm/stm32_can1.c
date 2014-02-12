/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012 Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de).
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
 * $Id: stm32_can1.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/can_dev.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/canbus.h>

#include <arch/cm3.h>
#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32xxxx_gpio.h>
#include <arch/cm3/stm/stm32xxxx_rcc.h>

#ifndef CANBUS1_REMAP_CAN
#define CANBUS1_REMAP_CAN 0
#endif

/*!
 * \brief CANBUS1 GPIO configuartion and assignment.
 */
#if defined(MCU_STM32F1)
 #if (CANBUS1_REMAP_CAN == 1)
  #define CANBUS_REMAP       AFIO_MAPR_CAN_REMAP_REMAP2
  #define CAN1RX_GPIO_PORT   NUTGPIO_PORTB
  #define CAN1TX_GPIO_PORT   NUTGPIO_PORTB
  #define CAN1RX_GPIO_PIN    8
  #define CAN1TX_GPIO_PIN    9

 #elif (CANBUS1_REMAP_CAN == 2)
  #define CANBUS_REMAP       AFIO_MAPR_CAN_REMAP_REMAP3
  #define CAN1RX_GPIO_PORT   NUTGPIO_PORTD
  #define CAN1TX_GPIO_PORT   NUTGPIO_PORTD
  #define CAN1RX_GPIO_PIN    0
  #define CAN1TX_GPIO_PIN    1
 #else
  #undef CANBUS_REMAP
  #define CANBUS_REMAP       AFIO_MAPR_CAN_REMAP_REMAP1
  #define CAN1RX_GPIO_PORT   NUTGPIO_PORTA
  #define CAN1TX_GPIO_PORT   NUTGPIO_PORTA
  #define CAN1RX_GPIO_PIN    11
  #define CAN1TX_GPIO_PIN    12
 #endif
#else /*L1/F2/F4*/
 #if !defined(CAN1_TX_PIN)
  #if (CANBUS1_REMAP_CAN == 0)
   #define CAN1TX_GPIO_PORT  NUTGPIO_PORTA
   #define CAN1TX_GPIO_PIN   12
  #elif (CANBUS1_REMAP_CAN == 1)
   #define CAN1TX_GPIO_PORT     NUTGPIO_PORTB
   #define CAN1TX_GPIO_PIN    9
  #elif (CANBUS1_REMAP_CAN == 2)
   #define CAN1TX_GPIO_PORT     NUTGPIO_PORTD
   #define CAN1TX_GPIO_PIN    1
  #else
   #error "Illegal CANBUS1_REMAP_CAN value"
  #endif
 #elif (CAN1_TX_PIN == 12)
  #define CAN1TX_GPIO_PORT  NUTGPIO_PORTA
  #define CAN1TX_GPIO_PIN   12
 #elif (CAN1_TX_PIN == 9)
  #define CAN1TX_GPIO_PORT     NUTGPIO_PORTB
  #define CAN1TX_GPIO_PIN    9
 #elif (CAN1_TX_PIN == 1)
  #define CAN1TX_GPIO_PORT     NUTGPIO_PORTD
  #define CAN1TX_GPIO_PIN    1
 #else
  #errror "Illegal CAN1 TX value"
 #endif
 #if !defined(CAN1_RX_PIN)
  #if (CANBUS1_REMAP_CAN == 0)
   #define CAN1RX_GPIO_PORT  NUTGPIO_PORTA
   #define CAN1RX_GPIO_PIN   11
  #elif (CANBUS1_REMAP_CAN == 1)
   #define CAN1RX_GPIO_PORT     NUTGPIO_PORTB
   #define CAN1RX_GPIO_PIN    8
  #elif (CANBUS1_REMAP_CAN == 2)
   #define CAN1RX_GPIO_PORT     NUTGPIO_PORTD
   #define CAN1RX_GPIO_PIN    0
  #else
   #error "Illegal CANBUS1_REMAP_CAN value"
  #endif
 #elif (CAN1_RX_PIN == 11)
  #define CAN1RX_GPIO_PORT  NUTGPIO_PORTA
  #define CAN1RX_GPIO_PIN   11
 #elif (CAN1_RX_PIN == 8)
  #define CAN1RX_GPIO_PORT     NUTGPIO_PORTB
  #define CAN1RX_GPIO_PIN    8
 #elif (CAN1_RX_PIN == 0)
  #define CAN1RX_GPIO_PORT     NUTGPIO_PORTD
  #define CAN1RX_GPIO_PIN    0
 #else
  #errror "Illegal CAN1 RX value"
 #endif
#endif

/*!
 * \brief Processor specific Hardware Initiliaization
 *
 */
int Stm32CanHw1Init(void)
{
    __IO uint32_t *rcc_bb = CM3BB_BASE(RCC_BASE);

#if defined (CAN2_ACCEPTANCE_FILTERS)
    uint32_t fmr;
#endif
    /* Enable CAN Bus 1 peripheral clock. */
    rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_CAN1EN))] = 1;

    /* Reset CAN Bus 1 IP */
    rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN1RST))] = 1;
    rcc_bb[CM3BB_OFFSET(RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN1RST))] = 0;

#if defined (CAN2_ACCEPTANCE_FILTERS)
    /* Set the CAN1/CAN2 Filter split */
    fmr = CAN1->FMR;
    fmr &= ~0x3f00;
    fmr |= CAN2_ACCEPTANCE_FILTERS<<8;
    CAN1->FMR = fmr;
#endif

    /* Setup Related GPIOs. */
    GpioPinConfigSet(CAN1RX_GPIO_PORT, CAN1RX_GPIO_PIN, GPIO_CFG_PULLUP|GPIO_CFG_PERIPHAL);
    GpioPinConfigSet(CAN1TX_GPIO_PORT, CAN1TX_GPIO_PIN, GPIO_CFG_OUTPUT|GPIO_CFG_PERIPHAL);

#if defined (MCU_STM32F1)
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
    AFIO->MAPR |=  CANBUS_REMAP;
#else
    GPIO_PinAFConfig((GPIO_TypeDef*) CAN1RX_GPIO_PORT, CAN1RX_GPIO_PIN, GPIO_AF_CAN1);
    GPIO_PinAFConfig((GPIO_TypeDef*) CAN1TX_GPIO_PORT, CAN1TX_GPIO_PIN, GPIO_AF_CAN1);
#endif

    return 0;
}

NUTCANBUS Stm32CanBus1 = {
    CAN1_BASE,
    CM3BB_BASE(CAN1_BASE),
    &sig_CAN1_RX0,
    &sig_CAN1_TX,
    &sig_CAN1_SCE,
    0,
    Stm32CanHw1Init,
};

NUTCANBUS Stm32CanBus1C = {
    CAN1_BASE,
    CM3BB_BASE(CAN1_BASE),
    &sig_CAN1_RX1,
    0,
    0,
    0,
    0,
};
