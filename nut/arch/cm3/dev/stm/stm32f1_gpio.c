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
 * $Id: stm32f1_gpio.c -1   $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/nutdebug.h>

#include <arch/cm3.h>
#include <dev/gpio.h>
#include <arch/cm3/stm/stm32f10x_rcc.h>

#define NUTGPIOPORT_MAX NUTGPIO_PORTG+1


static const uint32_t GPIO_RCCx[] = {
    RCC_APB2Periph_GPIOA,
    RCC_APB2Periph_GPIOB,
    RCC_APB2Periph_GPIOC,
    RCC_APB2Periph_GPIOD,
    RCC_APB2Periph_GPIOE,
    RCC_APB2Periph_GPIOF,
    RCC_APB2Periph_GPIOG,
};

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
    uint8_t cnf; */
uint32_t GpioPinConfigGet(int bank, int bit)
{
    uint32_t rc = 0;
    uint8_t mode;
    uint8_t cnf;
    uint8_t speed;
    GPIO_TypeDef *GPIOx = ((GPIO_TypeDef *)bank);

    if( bit < 8 ) {
        rc = GPIOx->CRL;
    }
    else {
        rc = GPIOx->CRH;
        bit -=8;
    }

    mode = ( rc >> ( bit * 4 ) );
    speed = mode & 0x3;
    cnf = ( mode >> 2 ) & 0x3;
    mode &= 0x3;

    if( mode == 0 ) {
        rc = 0;
        /* Pin is input */
        switch ( cnf ) {
        case 0:
            /* Input Analog */
            rc |= GPIO_CFG_DISABLED;
            break;
        case 1:
            /* Input Floating */
            rc = 0;
            break;        case 2:
            /* Input with Pullup */
            if (GPIOx->ODR & _BV(bit))
                rc |= GPIO_CFG_PULLUP;
            else
                rc |= GPIO_CFG_PULLDOWN;
            break;
        }
    }
    else {
        rc = GPIO_CFG_OUTPUT;

        /* Pin is output */
        switch ( cnf ) {
        case 1:
            /* Output Open-Drain */
            rc |= GPIO_CFG_MULTIDRIVE;
            break;
        case 2:
            /* Output Push-Pull Alternative Function */
            rc |= (GPIO_CFG_PERIPHAL | GPIO_CFG_DISABLED);
            break;
        case 3:
            /* Output Open-Drain Alternative Function */
            rc |= (GPIO_CFG_PERIPHAL | GPIO_CFG_DISABLED | GPIO_CFG_MULTIDRIVE);
            break;
        }
        switch ( speed) {
        case 2 :
            rc |= GPIO_CFG_SPEED_SLOW;
            break;
        case 3 :
            rc |= GPIO_CFG_SPEED_FAST;
            break;
        default :
            rc |= GPIO_CFG_SPEED_MED;
        }
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
 * \return Corrected flags
 */
int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags)
{
    uint32_t cxmx = 0;      /* Configuration mask */
    uint32_t clock = 0;     /* GPIO CLOCK and AFIO cache */
    uint32_t crl, crh, cln;
    uint8_t  msl, msh;
    int i;
    GPIO_TypeDef *GPIOx = ((GPIO_TypeDef *)bank);

    clock = GPIO_RCCx[(bank-GPIOA_BASE)/0x400];

    /* Pull-up/down can only be configured for input mode*/
    if (flags & (GPIO_CFG_OUTPUT|GPIO_CFG_DISABLED))
        if(flags & (GPIO_CFG_PULLUP |GPIO_CFG_PULLDOWN))
            flags &= ~(GPIO_CFG_PULLUP |GPIO_CFG_PULLDOWN);


    /*
     * cnf  mode  Meaning
     * 00   00    Input Analog
     * 01   00    Input Floating
     * 10   00    Input PullUp/Down
     * 11   00    Reserved

     * 00   xx    Output Push-Pull
     * 01   xx    Output Open-Drain
     * 10   xx    Output Alternate Push-Pull
     * 11   xx    Output Alternate Open-Drain
     *
     *      01    Output mode, max speed 10MHz
     *      10    Output mode, max speed 2MHz
     *      11    Output mode, max speed 50MHz
     */


    if( flags & GPIO_CFG_OUTPUT ) {
        switch (flags && GPIO_CFG_SPEED)
        {
        case GPIO_CFG_SPEED_SLOW: cxmx = 0x2; break;
        case GPIO_CFG_SPEED_FAST: cxmx = 0x3; break;
        default: cxmx = 0x1;
        }

        /* Configure Open-Drain */
        if( flags & GPIO_CFG_MULTIDRIVE )
            cxmx |= 0x4;

        /* Configure Alternate Function */
        if( flags & GPIO_CFG_DISABLED ) {
            cxmx |= 0x8;
            clock |= RCC_APB2Periph_AFIO;
        }

        /* Configure Alternate Function */
        if( flags & GPIO_CFG_PERIPHAL ) {
            cxmx |= 0x8;
            clock |= RCC_APB2Periph_AFIO;
        }
    }
    else {
        /* Configure pin as input floating */
        cxmx = 0x4;

        /* Configure pin as input PullUp/Down */
        if( flags & (GPIO_CFG_PULLUP |GPIO_CFG_PULLDOWN)) {
            cxmx = 0x8;
            if (flags & GPIO_CFG_PULLUP)
                GPIOx->ODR |= mask;
            else
                GPIOx->ODR &= ~mask;
        }
        /* Configure pin as analog input */
        if( flags & GPIO_CFG_DISABLED )
            cxmx = 0x0;

        /* Configure Alternate Function */
        if( flags & GPIO_CFG_PERIPHAL )
            clock |= RCC_APB2Periph_AFIO;
    }

    /* Important: Enable clock of port first
     * before trying to configure it!
     * Enable GPIO clock source */
    RCC->APB2ENR |= clock;

    msl = (uint8_t)(mask&0xff);
    msh = (uint8_t)(mask>>8);

    /* get actual config */
    crl = GPIOx->CRL;
    crh = GPIOx->CRH;

    /* shift in modified values */
    for( i = 0; i < 8; i++ ) {
        /* mask out actual nibble */
        cln = ~(0xf<<(i*4));

        /* check if pins 0..7 are affected */
        if( msl & _BV(i)) {
            crl &= cln;     /* mask nibble */
            crl |= cxmx;    /* write nibble */
        }
        /* check if pins 8..15 are affected */
        if( msh & _BV(i)) {
            crh &= cln;     /* mask nibble */
            crh |= cxmx;    /* write nibble */
        }
        /* shift left configuration for next bit */
        cxmx <<= 4;
    }

    /* write back new config */
    GPIOx->CRL = crl;
    GPIOx->CRH = crh;

    return flags;
}

/*!
 * \brief Set pin configuration.
 *
 * Applications may also use this function to make sure, that a specific
 * attribute is available for a specific pin.
 *
 * \note GPIO pins are typically initialized to a safe state after power
 *       up. This routine is not able to determine the consequences of
 *       changing pin configurations. In the worst case you may permanently
 *       damage your hardware by bad pin settings.
 *
 * \param bank  GPIO bank/port number.
 * \param bit   Bit number of the specified bank/port.
 * \param flags Attribute flags.
 *
 * \return 0 if all attributes had been set, -1 otherwise.
 */
int GpioPinConfigSet(int bank, int bit, uint32_t flags)
{
    NUTASSERT(bank<NUTGPIOPORT_MAX);

    if (GpioPortConfigSet( bank, _BV( bit ), flags ) !=
        GpioPinConfigGet( bank, bit )) {
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

