/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2011/2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id$
 * \endverbatim
 */

/* GPIO Configuration for the GPIO of L1/F2/F4 */

#include <cfg/os.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/nutdebug.h>

#include <arch/cm3.h>
#include <dev/gpio.h>
#if defined (MCU_STM32L1)
#include <arch/cm3/stm/stm32l1xx_gpio.h>
#include <arch/cm3/stm/stm32l1xx_rcc.h>
#define GPIO_RCC_ENR AHBENR
#elif defined (MCU_STM32F2)
#include <arch/cm3/stm/stm32f2xx_gpio.h>
#include <arch/cm3/stm/stm32f2xx_rcc.h>
#define GPIO_RCC_ENR AHB1ENR
#elif defined (MCU_STM32F4)
#include <arch/cm3/stm/stm32f4xx_gpio.h>
#include <arch/cm3/stm/stm32f4xx_rcc.h>
#define GPIO_RCC_ENR AHB1ENR
#else
#warning "Unknown STM32 family"
#endif
#define NUTGPIOPORT_MAX NUTGPIO_PORTH+1


/*!
 * \brief Get pin configuration.
 *
 * Trying to set undefined ports must be avoided.
 * If NUTDEBUG is enabled an assertion will be rised.
 *
 * \param bank GPIO bank/port number.
 * \param bit  Bit number of the specified bank/port.
 *
 * \return Attribute flags of the pin.
 */
uint32_t GpioPinConfigGet(int bank, int bit)
{
    uint32_t rc = 0;
    GPIO_TypeDef *gpio = (GPIO_TypeDef *)bank;
    uint8_t mode = ((gpio->MODER) >> (bit *2)) & 0x3;
    uint8_t pull = ((gpio->PUPDR) >> (bit *2)) & 0x3;
    uint8_t dr_oc = ((gpio->OTYPER) >> bit ) & 0x1;
    uint8_t speed = ((gpio->OSPEEDR) >> (bit *2)) & 0x3;
    if (mode == GPIO_Mode_OUT)
    {
        rc = GPIO_CFG_OUTPUT;
        rc |= (dr_oc)? GPIO_CFG_MULTIDRIVE: 0;
        rc |= (pull == GPIO_PuPd_UP)?GPIO_CFG_PULLUP : 0;
    }
    else if (mode == GPIO_Mode_AF)
    {
        rc  = GPIO_CFG_PERIPHAL;
        rc |= (dr_oc)? GPIO_CFG_MULTIDRIVE: 0;
        rc |= (pull == GPIO_PuPd_UP)?GPIO_CFG_PULLUP : 0;
    }
    else if (mode == GPIO_Mode_AN)
    {
        rc  = GPIO_CFG_OUTPUT| GPIO_CFG_DISABLED;
    }
    else if(mode == GPIO_Mode_IN)
    {
        rc |= (pull == GPIO_PuPd_UP)?GPIO_CFG_PULLUP : 0;
    }
    switch (speed)
    {
    case 0:  rc |= GPIO_CFG_SPEED_SLOW; break;
    case 2:  rc |= GPIO_CFG_SPEED_HIGH; break;
    case 3:  rc |= GPIO_CFG_SPEED_FAST; break;
     }
    return rc;
}

/*!
 * \brief Set port wide pin configuration.
 *
 * \note This function does not check for undefined ports and pins or
 *       invalid attributes. If this is required, use GpioPinConfigSet().
 *       If NUTDEBUG is enabled accessing an undefined port will rise
 *       an assertion.
 *
 * \param bank  GPIO bank/port number.
 * \param mask  The given attributes are set for a specific pin, if the
 *              corresponding bit in this mask is 1.
 * \param flags Attribute flags to set.
 *
 * \return Always 0.
 */
int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags)
{
    int i;

    for( i=0; i<16; i++)
    {
        if (mask & 1)
            GpioPinConfigSet(bank, i, flags);
        mask >>= 1;
    }
    return 0;
}

int GpioPinConfigSet(int bank, int bit, uint32_t flags)
{
    NUTASSERT(IS_GPIO_ALL_PERIPH(bank));
    __IO uint32_t* gpio_bb = CM3BB_BASE(bank);

    CM3BBREG(RCC_BASE, RCC_TypeDef, GPIO_RCC_ENR, (bank-GPIOA_BASE)>>10) = 1;
    /* keep speed at slowest for now */
    if (flags & GPIO_CFG_PERIPHAL)
    {
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, MODER, ((bit << 1)    ))] = 0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, MODER, ((bit << 1) + 1))] = 1;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, OTYPER, bit)            ] = (flags & GPIO_CFG_MULTIDRIVE )?1:0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, PUPDR, ((bit << 1) + 1))] = 0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, PUPDR, ((bit << 1)    ))] = (flags & GPIO_CFG_PULLUP )?1:0;
    }
    else if (flags & GPIO_CFG_OUTPUT)
    {
        uint32_t speed_flags_lo =
         (((flags & GPIO_CFG_SPEED_FAST) == GPIO_CFG_SPEED_MED) |
              ((flags & GPIO_CFG_SPEED_FAST) == GPIO_CFG_SPEED_FAST))?1:0;
        uint32_t speed_flags_hi =
             (((flags & GPIO_CFG_SPEED_FAST) == GPIO_CFG_SPEED_HIGH) |
              ((flags & GPIO_CFG_SPEED_FAST) == GPIO_CFG_SPEED_FAST))?1:0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, OSPEEDR, ((bit << 1)    ))] = speed_flags_lo;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, OSPEEDR, ((bit << 1) +1 ))] = speed_flags_hi;
#if defined(SYSCFG_CMPCR_CMP_PD)
        if ((flags & GPIO_CFG_SPEED_HIGH) == GPIO_CFG_SPEED_HIGH)
        {
           /* On F4, if even one pin needs fastest (high) speed, we need to enable the SYSCFG clock
         and the IO compensation cell (whatever this compensation cell is ?)*/
      CM3BBREG(RCC_BASE, RCC_TypeDef, APB2ENR, _BI32(RCC_APB2ENR_SYSCFGEN)) = 1;
      CM3BBREG(SYSCFG_BASE, SYSCFG_TypeDef, CMPCR, _BI32(SYSCFG_CMPCR_CMP_PD)) = 1;
        /* FIXME: Do we need to check SYSCFG_CMPCR_READY ? */
    }
#endif
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, MODER, ((bit << 1) + 1))] = 0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, MODER, ((bit << 1)    ))] = 1;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, OTYPER, bit)            ] = (flags & GPIO_CFG_MULTIDRIVE )?1:0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, PUPDR, ((bit << 1) + 1))] = 0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, PUPDR, ((bit << 1)    ))] = (flags & GPIO_CFG_PULLUP )?1:0;

    }
    else if (flags & GPIO_CFG_DISABLED)
    {
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, MODER, ((bit << 1)    ))] = 0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, MODER, ((bit << 1) + 1))] = 0 ;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, PUPDR, ((bit << 1) + 1))] = 0;
        gpio_bb[CM3BB_OFFSET(GPIO_TypeDef, PUPDR, ((bit << 1)    ))] = (flags & GPIO_CFG_PULLUP )?1:0;
    }


    /* Check the result. */
    if( GpioPinConfigGet( bank, bit ) != flags ) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Register a GPIO pin interrupt handler.
 *
 * Generating interrupts on GPIO pin changes is not supported on all
 * platforms. In this case dedicated external interrupt pins may
 * be used with NutRegisterIrqHandler().
 *
 * Interrupts are triggered on rising and falling edges. Level triggering
 * or triggering on specific edges is not supported.
 *
 * After registering, interrupts are disabled. Calling GpioIrqEnable()
 * is required to activate the interrupt.
 *
 * The following code fragment registers an interrupt handler which is
 * called on each change of bit 4 of the first GPIO port:
 * \code
 * #include <dev/gpio.h>
 *
 * static void PinChange(void *arg)
 * {
 *     ...
 * }
 *
 * {
 *     ...
 *     GpioPinConfigSet(0, 4, GPIO_CFG_PULLUP);
 *     GpioRegisterIrqHandler(&sig_GPIO, 4, PinChange, NULL);
 *     GpioIrqEnable(&sig_GPIO, 4);
 *     ...
 * }
 * \endcode
 *
 * \param sig     Bank/port interrupt to be associated with this handler.
 * \param bit     Bit number of the specified bank/port.
 * \param handler This routine will be called by Nut/OS, when the specified
 *                pin changes its state.
 * \param arg     Argument to be passed to the interrupt handler routine.
 *
 * \return 0 on success, -1 otherwise.
 */
extern int GpioRegisterIrqHandler(GPIO_SIGNAL * sig, uint8_t bit, void (*handler) (void *), void *arg)
{
    int rc = 0;
#if 0
    if (bit<5) {
        NutRegisterIrqHandler( &sig_INTERRUPT0, void(* handler)(void *), void * arg)
    uint32_t afiob = bit/4;
    /* Select bank in AFIO_EXTIn */
    AFIO->EXTICR[afiob] |= bank<<(afiob*4);
    EXTI->
#endif
    return rc;
}


/*!
 * \brief Enable a specified GPIO interrupt.
 *
 * A related interrupt handler must have been registered before calling
 * this function. See GpioRegisterIrqHandler().
 *
 * \param sig Interrupt to enable.
 * \param bit Bit number of the specified bank/port.
 *
 * \return 0 on success, -1 otherwise.
 */
int GpioIrqEnable(GPIO_SIGNAL * sig, uint8_t bit)
{
  return -1;
}

/*!
 * \brief Disable a specified GPIO interrupt.
 *
 * \param sig Interrupt to disable.
 * \param bit Bit number of the specified bank/port.
 *
 * \return 0 on success, -1 otherwise.
 */
int GpioIrqDisable(GPIO_SIGNAL * sig, uint8_t bit)
{
  return 0;
}

/* Copied from STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c*/
/**
  * @}
  */

/** @defgroup GPIO_Group3 GPIO Alternate functions configuration function
 *  @brief   GPIO Alternate functions configuration function
 *
 ===============================================================================
               GPIO Alternate functions configuration function
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Changes the mapping of the specified pin.
  * @param  GPIOx: where x can be (A..I) to select the GPIO peripheral.
  * @param  GPIO_PinSource: specifies the pin for the Alternate function.
  *         This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @param  GPIO_AFSelection: selects the pin to used as Alternate function.
  *          This parameter can be one of the following values:
  *            @arg GPIO_AF_RTC_50Hz: Connect RTC_50Hz pin to AF0 (default after reset)
  *            @arg GPIO_AF_MCO: Connect MCO pin (MCO1 and MCO2) to AF0 (default after reset)
  *            @arg GPIO_AF_TAMPER: Connect TAMPER pins (TAMPER_1 and TAMPER_2) to AF0 (default after reset)
  *            @arg GPIO_AF_SWJ: Connect SWJ pins (SWD and JTAG)to AF0 (default after reset)
  *            @arg GPIO_AF_TRACE: Connect TRACE pins to AF0 (default after reset)
  *            @arg GPIO_AF_TIM1: Connect TIM1 pins to AF1
  *            @arg GPIO_AF_TIM2: Connect TIM2 pins to AF1
  *            @arg GPIO_AF_TIM3: Connect TIM3 pins to AF2
  *            @arg GPIO_AF_TIM4: Connect TIM4 pins to AF2
  *            @arg GPIO_AF_TIM5: Connect TIM5 pins to AF2
  *            @arg GPIO_AF_TIM8: Connect TIM8 pins to AF3
  *            @arg GPIO_AF_TIM9: Connect TIM9 pins to AF3
  *            @arg GPIO_AF_TIM10: Connect TIM10 pins to AF3
  *            @arg GPIO_AF_TIM11: Connect TIM11 pins to AF3
  *            @arg GPIO_AF_I2C1: Connect I2C1 pins to AF4
  *            @arg GPIO_AF_I2C2: Connect I2C2 pins to AF4
  *            @arg GPIO_AF_I2C3: Connect I2C3 pins to AF4
  *            @arg GPIO_AF_SPI1: Connect SPI1 pins to AF5
  *            @arg GPIO_AF_SPI2: Connect SPI2/I2S2 pins to AF5
  *            @arg GPIO_AF_SPI3: Connect SPI3/I2S3 pins to AF6
  *            @arg GPIO_AF_I2S3ext: Connect I2S3ext pins to AF7
  *            @arg GPIO_AF_USART1: Connect USART1 pins to AF7
  *            @arg GPIO_AF_USART2: Connect USART2 pins to AF7
  *            @arg GPIO_AF_USART3: Connect USART3 pins to AF7
  *            @arg GPIO_AF_UART4: Connect UART4 pins to AF8
  *            @arg GPIO_AF_UART5: Connect UART5 pins to AF8
  *            @arg GPIO_AF_USART6: Connect USART6 pins to AF8
  *            @arg GPIO_AF_CAN1: Connect CAN1 pins to AF9
  *            @arg GPIO_AF_CAN2: Connect CAN2 pins to AF9
  *            @arg GPIO_AF_TIM12: Connect TIM12 pins to AF9
  *            @arg GPIO_AF_TIM13: Connect TIM13 pins to AF9
  *            @arg GPIO_AF_TIM14: Connect TIM14 pins to AF9
  *            @arg GPIO_AF_OTG_FS: Connect OTG_FS pins to AF10
  *            @arg GPIO_AF_OTG_HS: Connect OTG_HS pins to AF10
  *            @arg GPIO_AF_ETH: Connect ETHERNET pins to AF11
  *            @arg GPIO_AF_FSMC: Connect FSMC pins to AF12
  *            @arg GPIO_AF_OTG_HS_FS: Connect OTG HS (configured in FS) pins to AF12
  *            @arg GPIO_AF_SDIO: Connect SDIO pins to AF12
  *            @arg GPIO_AF_DCMI: Connect DCMI pins to AF13
  *            @arg GPIO_AF_EVENTOUT: Connect EVENTOUT pins to AF15
  * @retval None
  */
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;

  /* Check the parameters */
  NUTASSERT(IS_GPIO_ALL_PERIPH(GPIOx));
  NUTASSERT(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
  NUTASSERT(IS_GPIO_AF(GPIO_AF));

  temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
}
