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
 * $Id: stm32_twi2.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

/*!
 * \file arch/cm3/dev/stm/stm32_twi2.c
 * \brief STM32F I2C bus 1 initialization
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/twi.h>
#include <cfg/arch/gpio.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/twif.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32xxxx_gpio.h>
#include <arch/cm3/stm/stm32xxxx_rcc.h>
#if defined(I2CBUS2_USE_DMA)
#if defined(MCU_STM32F1)
    #include <arch/cm3/stm/stm32f1_dma.h>
#else
#warning "Unhandled STM32 family"
#endif
#endif

#include <arch/cm3/stm/stm32_twi.h>

/*!
 * \brief I2CBUS2 GPIO configuartion and assignment.
 *
 * F1:     SMBA PB12
 *         SCL  PB10
 *         SDA  PB11
 * L1:     SMBA PB12
 *         SCL  PB10
 *         SDA  PB11
 * F2/F4L  SMBA PB12 PF2 PH6
 *         SCL  PB10 PF1 PH4
 *         SDA  PB11 PF0 PH5
 */
#if defined(MCU_STM32F1)
 #define I2CBUS2_SCL_PORT    NUTGPIO_PORTB
 #define I2CBUS2_SCL_PIN     10
 #define I2CBUS2_SDA_PORT    NUTGPIO_PORTB
 #define I2CBUS2_SDA_PIN     11
#else
 #if !defined(I2CBUS2_SCL_PIN)
  #define I2CBUS2_SCL_PORT    NUTGPIO_PORTB
  #define I2CBUS2_SCL_PIN     10
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F2)) && I2CBUS2_SCL_PIN == 1
  #define I2CBUS2_SCL_PORT    NUTGPIO_PORTF
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F2)) && I2CBUS2_SCL_PIN == 4
  #define I2CBUS2_SCL_PORT    NUTGPIO_PORTH
 #else
  #warning "Illegal I2C2 SCL pin assignement"
 #endif
 #if !defined(I2CBUS2_SDA_PIN)
  #define I2CBUS2_SDA_PORT    NUTGPIO_PORTB
  #define I2CBUS2_SDA_PIN     11
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F2)) && I2CBUS2_SDA_PIN == 0
  #define I2CBUS2_SDA_PORT    NUTGPIO_PORTF
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F2)) && I2CBUS2_SDA_PIN == 5
  #define I2CBUS2_SDA_PORT    NUTGPIO_PORTH
 #else
  #warning "Illegal I2C2 SCL pin assignement"
 #endif
#endif

#ifdef I2CBUS2_MODE_SMBUS
 #if defined(MCU_STM32F1)
  #define I2CBUS2_SMBA_PORT    NUTGPIO_PORTB
  #define I2CBUS2_SMBA_PIN     11
 #endif
#else
 #ifndef I2CBUS2_SMBA_PIN
  #define I2CBUS2_SMBA_PORT    NUTGPIO_PORTB
  #define I2CBUS2_SMBA_PIN     11
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F2)) && I2CBUS2_SMBA_PIN == 2
  #define I2CBUS2_SMBA_PORT    NUTGPIO_PORTF
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F2)) && I2CBUS2_SMBA_PIN == 6
 #define I2CBUS2_SMBA_PORT    NUTGPIO_PORTH
 #else
  #warning "Illegal I2C2 SMBA pin assignement"
 #endif
#endif

#ifdef I2CBUS2_DEFAULT_SPEED
#define I2C_DEFAULT_SPEED
#else
#endif

int Stm32I2cBus2Recover( void)
{
    uint_fast8_t i;

    /* Handle pins as GPIOs, set SCL low */
    GpioPinConfigSet( I2CBUS2_SCL_PORT, I2CBUS2_SCL_PIN, GPIO_CFG_OUTPUT|GPIO_CFG_MULTIDRIVE);
    GpioPinConfigSet( I2CBUS2_SDA_PORT, I2CBUS2_SDA_PIN, GPIO_CFG_OUTPUT|GPIO_CFG_MULTIDRIVE);
    GpioPinSetLow( I2CBUS2_SDA_PORT, I2CBUS2_SDA_PIN);
    NutMicroDelay(10);

    /* Run sequence of 8 SCL clocks */
    for( i=0; i<8; i++) {
        GpioPinSetLow( I2CBUS2_SCL_PORT, I2CBUS2_SCL_PIN);
        NutMicroDelay(10);
        GpioPinSetHigh( I2CBUS2_SCL_PORT, I2CBUS2_SCL_PIN);
        NutMicroDelay(10);
    }

    /* Issue Stop condition on the bus */
    GpioPinSetHigh( I2CBUS2_SDA_PORT, I2CBUS2_SDA_PIN);
    NutMicroDelay(10);
    GpioPinSetHigh( I2CBUS2_SCL_PORT, I2CBUS2_SCL_PIN);

    GpioPinConfigSet( I2CBUS2_SCL_PORT, I2CBUS2_SCL_PIN, GPIO_CFG_OUTPUT| GPIO_CFG_PERIPHAL|GPIO_CFG_MULTIDRIVE);
    GpioPinConfigSet( I2CBUS2_SDA_PORT, I2CBUS2_SDA_PIN, GPIO_CFG_OUTPUT| GPIO_CFG_PERIPHAL|GPIO_CFG_MULTIDRIVE);
    return 0;
}

/*!
 * \brief Processor specific Hardware Initiliaization
 *
 */
int Stm32I2cBus2Init(void)
{

    /* Enable I2C Bus 2 peripheral clock. */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    /* Reset I2C Bus 2 IP */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

    /* Setup Related GPIOs. */
#if defined (MCU_STM32L1) || defined (MCU_STM32F2) || defined (MCU_STM32F4)
    GPIO_PinAFConfig((GPIO_TypeDef*) I2CBUS2_SDA_PORT, I2CBUS2_SDA_PIN, GPIO_AF_I2C2);
    GPIO_PinAFConfig((GPIO_TypeDef*) I2CBUS2_SCL_PORT, I2CBUS2_SCL_PIN, GPIO_AF_I2C2);
#endif
    GpioPinConfigSet( I2CBUS2_SCL_PORT, I2CBUS2_SCL_PIN,
              GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_MULTIDRIVE);
    GpioPinConfigSet( I2CBUS2_SDA_PORT, I2CBUS2_SDA_PIN,
              GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_MULTIDRIVE);
#ifdef I2CBUS2_MODE_SMBUS
    GPIO_PinAFConfig((GPIO_TypeDef*) I2CBUS2_SMBA_PORT, I2CBUS2_SMBA_PIN, GPIO_AF_I2C2);
    GpioPinConfigSet( I2CBUS2_SMBA_PORT, I2CBUS2_SMBA_PIN,
              GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_MULTIDRIVE);
#endif
    NVIC_SetPriorityGrouping(4);
    NVIC_SetPriority( I2C2_EV_IRQn, 0);
    NVIC_SetPriority( I2C2_ER_IRQn, 1);

#if defined(I2CBUS2_USE_DMA)
    DMA_Init();
    DMA_Disable(I2C2_DMA_CHANNEL_TX);
    DMA_Disable(I2C2_DMA_CHANNEL_RX);
#endif

    return 0;
}

/*!
 * \brief TWI/I2C bus structure.
 */
NUTTWIBUS Stm32TwiBus_2 = {
  /*.bus_base =    */  I2C2_BASE,              /* Bus base address. */
  /*.bus_sig_ev =  */ &sig_TWI2_EV,            /* Bus data and event interrupt handler. */
  /*.bus_sig_er =  */ &sig_TWI2_ER,            /* Bus error interrupt handler. */
  /*.bus_mutex =   */  NULL,                   /* Bus lock queue. */
  /*.bus_icb   =   */  NULL,                   /* Bus Runtime Data Pointer */
#if defined(I2CBUS1_USE_DMA)
  /*.bus_dma_tx =  */  I2C2_DMA_CHANNEL_TX,    /* DMA channel for TX direction. */
  /*.bus_dma_rx =  */  I2C2_DMA_CHANNEL_RX,    /* DMA channel for RX direction. */
#else
  /*.bus_dma_tx =  */  0,
  /*.bus_dma_rx =  */  0,
#endif
  /*.bus_initbus = */  Stm32I2cBus2Init,       /* Initialize bus controller. */
  /*.bus_recover = */  Stm32I2cBus2Recover,    /* Recover bus in case a slave hangs with SCL low */
};

