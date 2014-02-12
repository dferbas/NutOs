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
 * $Id: stm32_twi1.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

/*!
 * \file arch/cm3/dev/stm/stm32_twi1.c
 * \brief STM32F I2C bus 1 initialization
 */


#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/twi.h>
#include <cfg/arch/gpio.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/twif.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32xxxx_gpio.h>
#include <arch/cm3/stm/stm32xxxx_rcc.h>
#if defined(I2CBUS1_USE_DMA)
#if defined(MCU_STM32F1)
    #include <arch/cm3/stm/stm32f1_dma.h>
#else
#warning "Unhandled STM32 family"
#endif
#endif

#include <arch/cm3/stm/stm32_twi.h>

/*!
 * \brief I2CBUS1 GPIO configuartion and assignment.
 * F1/L1/F2/F4: SMBA PB5
 *              SCL  PB6 PB8
 *              SDA  PB7 PB9
 */
#define I2C_PORT    NUTGPIO_PORTB

#if defined(MCU_STM32F1)
 #if defined(I2CBUS1_REMAP_I2C)
  #define I2C_DOREMAP ENABLE
  #define I2CBUS1_SDA_PIN     9
  #define I2CBUS1_SCL_PIN     8
 #else /* I2CBUS1_REMAP_I2C */
  #define I2C_DOREMAP DISABLE
  #define I2CBUS1_SDA_PIN     7
  #define I2CBUS1_SCL_PIN     6
 #endif /* I2CBUS1_REMAP_I2C */
#else /*L1/F2/F4*/
 #if !defined(I2CBUS1_SDA_PIN)
  #if defined(I2CBUS1_REMAP_I2C)
   #define I2CBUS1_SDA_PIN     9
  #else
   #define I2CBUS1_SDA_PIN     7
  #endif
 #endif
 #if I2CBUS1_SDA_PIN != 7 && I2CBUS1_SDA_PIN != 9
  #warning "Illegal I2C1 SDA pin assignement"
 #endif
 #if !defined(I2CBUS1_SCL_PIN)
  #if defined(I2CBUS1_REMAP_I2C)
   #define I2CBUS1_SCL_PIN     8
  #else
   #define I2CBUS1_SCL_PIN     6
  #endif
 #endif
 #if I2CBUS1_SCL_PIN != 6 && I2CBUS1_SCL_PIN != 8
  #warning "Illegal I2C1 SCL pin assignement"
 #endif
#endif
#ifdef I2CBUS1_MODE_SMBUS
  #define SMBA_PIN  5
#endif

/*!
 * \brief Unlock a broken slave by clocking 8 SCL pulses manually.
 */
int Stm32I2cBus1Recover( void)
{
    uint_fast8_t i;

    /* Handle pins as GPIOs, set SCL low */
    GpioPortConfigSet( I2C_PORT, _BV(I2CBUS1_SDA_PIN) | _BV(I2CBUS1_SCL_PIN), GPIO_CFG_OUTPUT|GPIO_CFG_MULTIDRIVE);
    GpioPinSetLow( I2C_PORT, I2CBUS1_SDA_PIN);
    NutMicroDelay(10);

    /* Run sequence of 8 SCL clocks */
    for( i=0; i<9; i++) {
        GpioPinSetLow( I2C_PORT, I2CBUS1_SCL_PIN);
        NutMicroDelay(10);
        GpioPinSetHigh( I2C_PORT, I2CBUS1_SCL_PIN);
        NutMicroDelay(10);
    }

    /* Issue Stop condition on the bus */
    GpioPinSetHigh( I2C_PORT, I2CBUS1_SDA_PIN);
    NutMicroDelay(10);
    GpioPinSetHigh( I2C_PORT, I2CBUS1_SCL_PIN);

    GpioPortConfigSet(I2C_PORT, _BV(I2CBUS1_SDA_PIN) | _BV(I2CBUS1_SCL_PIN), GPIO_CFG_OUTPUT
                                                 | GPIO_CFG_PERIPHAL
                                                 | GPIO_CFG_MULTIDRIVE);

    return 0;
}

/*!
 * \brief Processor specific Hardware Initiliaization
 *
 */
int Stm32I2cBus1Init(void)
{
    uint16_t pins;

    /* Enable I2C Bus 1 peripheral clock. */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* Reset I2C Bus 1 IP */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    /* Setup Related GPIOs. */
    pins = _BV(I2CBUS1_SDA_PIN) | _BV(I2CBUS1_SCL_PIN);
#ifdef I2CBUS1_MODE_SMBUS
    pins |= _BV(SMBA_PIN);
    GPIO_PinAFConfig((GPIO_TypeDef*) I2C_PORT, SMBA_PIN, GPIO_AF_I2C1);
#endif
#if defined (MCU_STM32F1)
    /* Configure alternate configuration. */
    CM3BBREG(AFIO_BASE, AFIO_TypeDef, MAPR, _BI32(AFIO_MAPR_I2C1_REMAP)) = I2C_DOREMAP;
#elif defined (MCU_STM32L1) || defined (MCU_STM32F2) || defined (MCU_STM32F4)
    GPIO_PinAFConfig((GPIO_TypeDef*) I2C_PORT, I2CBUS1_SDA_PIN, GPIO_AF_I2C1);
    GPIO_PinAFConfig((GPIO_TypeDef*) I2C_PORT, I2CBUS1_SCL_PIN, GPIO_AF_I2C1);
#else
#warning "Unhandled STM32 family"
#endif
    GpioPortConfigSet( I2C_PORT, pins, GPIO_CFG_OUTPUT
                                     | GPIO_CFG_PERIPHAL
                                     | GPIO_CFG_MULTIDRIVE);
    NVIC_SetPriorityGrouping(4);
    NVIC_SetPriority( I2C1_EV_IRQn, 0);
    NVIC_SetPriority( I2C1_ER_IRQn, 1);

#if defined(I2CBUS1_USE_DMA)
#if defined (MCU_STM32F1)
    DMA_Init();
    DMA_Disable(I2C1_DMA_CHANNEL_TX);
    DMA_Disable(I2C1_DMA_CHANNEL_RX);
#else
#warning "Unhandled STM32 family"
#endif
#endif
    return 0;
}


/*!
 * \brief TWI/I2C bus structure.
 */
NUTTWIBUS Stm32TwiBus_1 = {
  /*.bus_base =    */  I2C1_BASE,              /* Bus base address. */
  /*.bus_sig_ev =  */ &sig_TWI1_EV,            /* Bus data and event interrupt handler. */
  /*.bus_sig_er =  */ &sig_TWI1_ER,            /* Bus error interrupt handler. */
  /*.bus_mutex =   */  NULL,                   /* Bus lock queue. */
  /*.bus_icb   =   */  NULL,                   /* Bus Runtime Data Pointer */
#if defined(I2CBUS1_USE_DMA)
  /*.bus_dma_tx =  */  I2C1_DMA_CHANNEL_TX,    /* DMA channel for TX direction. */
  /*.bus_dma_rx =  */  I2C1_DMA_CHANNEL_RX,    /* DMA channel for RX direction. */
#else
  /*.bus_dma_tx =  */  0,
  /*.bus_dma_rx =  */  0,
#endif
  /*.bus_initbus = */  Stm32I2cBus1Init,       /* Initialize bus controller. */
  /*.bus_recover = */  Stm32I2cBus1Recover,    /* Recover bus in case a slave hangs with SCL low */
};

