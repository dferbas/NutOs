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
 * $Id: stm32_can2.c 3108 2010-09-15 21:11:15Z Astralix $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <cfg/arch/gpio.h>
#include <dev/irqreg.h>
#include <arch/cm3/stm/stm32xxxx.h>

#define NUTGPIO_PORT    GPIOA_BASE
#define NUTGPIO_PORTA   GPIOA_BASE
#define NUTGPIO_PORTB   GPIOB_BASE
#define NUTGPIO_PORTC   GPIOC_BASE
#define NUTGPIO_PORTD   GPIOD_BASE
#define NUTGPIO_PORTE   GPIOE_BASE
#define NUTGPIO_PORTF   GPIOF_BASE
#define NUTGPIO_PORTG   GPIOG_BASE

#define NUTGPIO_EXTINT0     1
#define NUTGPIO_EXTINT1     2
#define NUTGPIO_EXTINT2     3
#define NUTGPIO_EXTINT3     4
#define NUTGPIO_EXTINT4     5

/*!
 * \brief GPIO input.
 *
 * Will configure the pin as input. This is the default state, when no other
 * config option is given.
 */

#define GPIO_CFG_INPUT      0x00000000

/*!
 * \brief GPIO disabled.
 *
 * STM32 Specific:
 * If combined with GPIO_CFG_OUTPUT it enables the alternate function.
 * If used without, it sets the analog input mode of the pin.
 */
#define GPIO_CFG_DISABLED   0x00000001

/*!
 * \brief GPIO output direction enabled.
 *
 * If set, the pin is configured as an output. Otherwise it is in
 * input mode or z-state.
 * For STM32F it is configured as output 50MHz.
 */
#define GPIO_CFG_OUTPUT     0x00000002

/*!
 * \brief GPIO pull-up enabled.
 */
#define GPIO_CFG_PULLUP     0x00000004

/*!
 * \brief GPIO pull-down enabled.
 */
#define GPIO_CFG_PULLDOWN   0x00000100

/*!
 * \brief GPIO open drain output feature enabled.
 *
 * If not set, the output will use push pull mode.
 */
#define GPIO_CFG_MULTIDRIVE 0x00000008

/*!
 * \brief GPIO input glitch filter enabled.
 *
 * Not supported with STM32F
 */
#define GPIO_CFG_DEBOUNCE   0x00000000

/*!
 * \brief GPIO set to alternate function.
 *
 * STM32F specific:
 * Enables alternate function of the pin.
 */
#define GPIO_CFG_PERIPHAL   0x00000020

/*!
 * \brief GPIO pin speed
 *
 * Speed L1      F2/F2
 * SLOW  400 kHz   2 MHz
 * MED     2 MHz  25 MHz
 * FAST   10 MHz  50 MHz
 * HIGH   40 MHz 100 MHz
 *
 * As default, we set GPIO_CFG_SPEED_MED
 */
#define GPIO_CFG_SPEED       0x000000C0
#define GPIO_CFG_SPEED_SLOW  0x00000040
#define GPIO_CFG_SPEED_MED   0x00000000
#define GPIO_CFG_SPEED_FAST  0x00000080
#define GPIO_CFG_SPEED_HIGH  0x000000C0

typedef struct {
    void (*iov_handler) (void *);
    void *iov_arg;
} GPIO_VECTOR;

typedef struct {
    IRQ_HANDLER *ios_sig;
    void (*ios_handler) (void *);
    int (*ios_ctl) (int cmd, void *param, int bit);
    GPIO_VECTOR *ios_vector;
} GPIO_SIGNAL;

#if defined(PIO_ISR)
extern GPIO_SIGNAL sig_GPIO;
#endif
#if defined(PIOA_ISR)
extern GPIO_SIGNAL sig_GPIO1;
#endif
#if defined(PIOB_ISR)
extern GPIO_SIGNAL sig_GPIO2;
#endif
#if defined(PIOC_ISR)
extern GPIO_SIGNAL sig_GPIO3;
#endif

extern uint32_t GpioPinConfigGet(int bank, int bit);
extern int GpioPinConfigSet(int bank, int bit, uint32_t flags);
extern int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags);

#define GpioPinGet(bank, bit)           CM3BBREG(bank, GPIO_TypeDef, IDR, bit)
#define GpioPinSet(bank, bit, value)    CM3BBREG(bank, GPIO_TypeDef, ODR, bit) = value
#if defined(MCU_STM32F1)
#define GpioPinSetHigh(bank, bit)   CM3BBREG(bank, GPIO_TypeDef, BSRR, bit) = 1
#define GpioPinSetLow(bank, bit)    CM3BBREG(bank, GPIO_TypeDef, BRR , bit) = 1
#else
#define GpioPinSetHigh(bank, bit)   CM3BBREG(bank, GPIO_TypeDef, BSRRL, bit) = 1
#define GpioPinSetLow(bank, bit)    CM3BBREG(bank, GPIO_TypeDef, BSRRH, bit) = 1
#endif

#define GpioPortGet(bank)       CM3REG(bank, GPIO_TypeDef, IDR )
#define GpioPortSet(bank, value)    CM3REG(bank, GPIO_TypeDef, ODR ) = value
#define GpioPortSetHigh(bank, mask) CM3REG(bank, GPIO_TypeDef, BSRR) = mask
#define GpioPortSetLow(bank, mask)  CM3REG(bank, GPIO_TypeDef, BRR ) = mask

extern int GpioRegisterIrqHandler(GPIO_SIGNAL * sig, uint8_t bit, void (*handler) (void *), void *arg);
extern int GpioIrqEnable(GPIO_SIGNAL * sig, uint8_t bit);
extern int GpioIrqDisable(GPIO_SIGNAL * sig, uint8_t bit);
