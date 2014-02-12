/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 * $Id: lpc176x_gpio.c $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/nutdebug.h>

#include <arch/cm3.h>
#include <arch/cm3/nxp/lpc176x_gpio.h>
#include <dev/gpio.h>

#include <string.h>
#include <stdlib.h>

/*!
 * \addtogroup xgNutArchCm3Lpc176xGpio
 */
/*@{*/

#define NUTGPIOPORT_MAX NUTGPIO_PORT4+1

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
    int      pin_offset;

    __IO uint32_t *PINSEL;
    __IO uint32_t *PINMODE;
    __IO uint32_t *PINMODE_OD;

    NUTASSERT(bank < NUTGPIOPORT_MAX);
    NUTASSERT(bit < 32);

    /*
     * PINSEL
     *
     * 00   GPIO
     * 01   Peripheral 1 AF
     * 10   Peripheral 2 AF
     * 11   Peripheral 3 AF
     *
     * PINMODE
     *
     * 00   Pullup enabled
     * 01   Repeater mode
     * 10   Neither pullup nor pulldown enabled
     * 11   Pulldown enabled
     */

    if (bit < 16) {
        PINSEL  = (uint32_t *)&LPC_PINCON->PINSEL0 + bank * 2;
        PINMODE = (uint32_t *)&LPC_PINCON->PINMODE0 + bank * 2;
        pin_offset = bit * 2;
    } else {
        PINSEL  = (uint32_t *)&LPC_PINCON->PINSEL0 + bank * 2 + 1;
        PINMODE = (uint32_t *)&LPC_PINCON->PINMODE0 + bank * 2 + 1;
        pin_offset = (bit - 16) * 2;
    }

    PINMODE_OD = (uint32_t *)&LPC_PINCON->PINMODE_OD0 + bank;

    if (CM3BBREG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, bit)) {
        rc |= GPIO_CFG_OUTPUT;
    }

    switch ((*PINSEL >> pin_offset) & 0x03) {
        case PINCON_PINSEL_AF0:
            /* GPIO mode selected */
            rc |= GPIO_CFG_PERIPHERAL0;
            break;
        case PINCON_PINSEL_AF1:
            /* AF1 selected */
            rc |= GPIO_CFG_PERIPHERAL1 | GPIO_CFG_DISABLED;
            break;
        case PINCON_PINSEL_AF2:
            /* AF1 selected */
            rc |= GPIO_CFG_PERIPHERAL2 | GPIO_CFG_DISABLED;
            break;
        case PINCON_PINSEL_AF3:
            /* AF1 selected */
            rc |= GPIO_CFG_PERIPHERAL3 | GPIO_CFG_DISABLED;
            break;
    }

    switch ((*PINMODE >> pin_offset) & 0x03) {
        case PINCON_PINMODE_PULLUP:
            /* Pullup mode selected */
            rc |= GPIO_CFG_PULLUP;
            break;
        case PINCON_PINMODE_REPEATER:
            /* Repeater mode selected */
            rc |= GPIO_CFG_REPEATER;
            break;
        case PINCON_PINMODE_NORMAL:
            /* Neither pullup not pulldown */
            break;
        case PINCON_PINMODE_PULLDOWN:
            /* AF1 selected */
            rc |= GPIO_CFG_PULLDOWN;
            break;
    }

    if (*PINMODE_OD & bit) {
        rc |= GPIO_CFG_MULTIDRIVE;
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
    int      i, j;
    uint32_t mode_l, mode_h;
    uint32_t sel_l, sel_h;

    __IO uint32_t *PINSEL_L;
    __IO uint32_t *PINSEL_H;
    __IO uint32_t *PINMODE_L;
    __IO uint32_t *PINMODE_H;
    __IO uint32_t *PINMODE_OD;

    NUTASSERT(bank < NUTGPIOPORT_MAX);

    /*
     * PINSEL
     *
     * 00   GPIO
     * 01   Peripheral 1 AF
     * 10   Peripheral 2 AF
     * 11   Peripheral 3 AF
     *
     * PINMODE
     *
     * 00   Pullup enabled
     * 01   Repeater mode
     * 10   Neither pullup nor pulldown enabled
     * 11   Pulldown enabled
     */

    PINSEL_L   = (uint32_t *)&LPC_PINCON->PINSEL0 + bank * 2;
    PINSEL_H   = PINSEL_L + 1;

    PINMODE_L  = (uint32_t *)&LPC_PINCON->PINMODE0 + bank * 2;
    PINMODE_H  = PINMODE_L + 1;

    PINMODE_OD = (uint32_t *)&LPC_PINCON->PINMODE_OD0 + bank;

    /* Read current settings */
    sel_l  = *PINSEL_L;
    sel_h  = *PINSEL_H;
    mode_l = *PINMODE_L;
    mode_h = *PINMODE_H;

    for (i = 0, j = 16; i < 16; i ++, j++) {
        if (mask & _BV(i)) {
            sel_l  &= ~(PINCON_PINSEL_MASK << (i * 2));
            mode_l &= ~(PINCON_PINMODE_MASK << (i * 2));

            /* skip flag GPIO_CFG_PERIPHERAL0 ==> PINCON_PINSEL_AF0 */
            if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL1) {
                sel_l |= PINCON_PINSEL_AF1 << (i * 2);
            } else
            if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL2) {
                sel_l |= PINCON_PINSEL_AF2 << (i * 2);
            } else
            if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL3) {
                sel_l |= PINCON_PINSEL_AF3 << (i * 2);
            }

            if (flags & GPIO_CFG_PULLUP) {
                mode_l |= PINCON_PINMODE_PULLUP << (i * 2);
            } else
            if (flags & GPIO_CFG_REPEATER) {
                mode_l |= PINCON_PINMODE_REPEATER << (i * 2);
            } else
            if (flags & GPIO_CFG_PULLDOWN) {
                mode_l |= PINCON_PINMODE_PULLDOWN << (i * 2);
            } else {
                mode_l |= PINCON_PINMODE_NORMAL << (i * 2);
            }
        }
        if (mask & _BV(j)) {
            sel_h  &= ~(PINCON_PINSEL_MASK << (i * 2));
            mode_h &= ~(PINCON_PINMODE_MASK << (i * 2));

            /* skip flag GPIO_CFG_PERIPHERAL0 ==> PINCON_PINSEL_AF0 */
            if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL1) {
                sel_h |= PINCON_PINSEL_AF1 << (i * 2);
            } else
            if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL2) {
                sel_h |= PINCON_PINSEL_AF2 << (i * 2);
            } else
            if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL3) {
                sel_h |= PINCON_PINSEL_AF3 << (i * 2);
            }

            if (flags & GPIO_CFG_PULLUP) {
                mode_h |= PINCON_PINMODE_PULLUP << (i * 2);
            } else
            if (flags & GPIO_CFG_REPEATER) {
                mode_h |= PINCON_PINMODE_REPEATER << (i * 2);
            } else
            if (flags & GPIO_CFG_PULLDOWN) {
                mode_h |= PINCON_PINMODE_PULLDOWN << (i * 2);
            } else {
                mode_h |= PINCON_PINMODE_NORMAL << (i * 2);
            }
        }
    }

    /* Write back the modified register values */
    *PINSEL_L  = sel_l;
    *PINSEL_H  = sel_h;
    *PINMODE_L = mode_l;
    *PINMODE_H = mode_h;

    if (flags & GPIO_CFG_OUTPUT) {
        CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR) |= mask;
    } else {
        CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR) &= ~mask;
    }

    if (flags & GPIO_CFG_MULTIDRIVE) {
        *PINMODE_OD |= mask;
    } else {
        *PINMODE_OD &= ~mask;
    }

    return 0;
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
    uint32_t mode;
    uint32_t sel;
    int      pin_offset;

    __IO uint32_t *PINSEL_L;
    __IO uint32_t *PINSEL_H;
    __IO uint32_t *PINMODE_L;
    __IO uint32_t *PINMODE_H;
    __IO uint32_t *PINMODE_OD;

    NUTASSERT(bank < NUTGPIOPORT_MAX);
    NUTASSERT(bit < 32);

    /*
     * PINSEL
     *
     * 00   GPIO
     * 01   Peripheral 1 AF
     * 10   Peripheral 2 AF
     * 11   Peripheral 3 AF
     *
     * PINMODE
     *
     * 00   Pullup enabled
     * 01   Repeater mode
     * 10   Neither pullup nor pulldown enabled
     * 11   Pulldown enabled
     */

    PINSEL_L   = (uint32_t *)&LPC_PINCON->PINSEL0 + bank * 2;
    PINSEL_H   = PINSEL_L + 1;

    PINMODE_L  = (uint32_t *)&LPC_PINCON->PINMODE0 + bank * 2;
    PINMODE_H  = PINMODE_L + 1;

    PINMODE_OD = (uint32_t *)&LPC_PINCON->PINMODE_OD0 + bank;

    /* Read current settings */
    if (bit < 16) {
        sel  = *PINSEL_L;
        mode = *PINMODE_L;
        pin_offset = bit * 2;
    } else {
        sel  = *PINSEL_H;
        mode = *PINMODE_H;
        pin_offset = (bit - 16) * 2;
    }

    sel  &= ~(PINCON_PINSEL_MASK << pin_offset);
    mode &= ~(PINCON_PINMODE_MASK << pin_offset);

    /* skip flag GPIO_CFG_PERIPHERAL0 */
    if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL1) {
        sel |= PINCON_PINSEL_AF1 << pin_offset;
    } else
    if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL2) {
        sel |= PINCON_PINSEL_AF3 << pin_offset;
    } else
    if ((flags & GPIO_CFG_PERIPHERAL_MASK) == GPIO_CFG_PERIPHERAL3) {
        sel |= PINCON_PINSEL_AF3 << pin_offset;
    }

    if (flags & GPIO_CFG_PULLUP) {
        mode |= PINCON_PINMODE_PULLUP << pin_offset;
    } else
    if (flags & GPIO_CFG_REPEATER) {
        mode |= PINCON_PINMODE_REPEATER << pin_offset;
    } else
    if (flags & GPIO_CFG_PULLDOWN) {
        mode |= PINCON_PINMODE_PULLDOWN << pin_offset;
    } else {
        mode |= PINCON_PINMODE_NORMAL << pin_offset;
    }

    /* Write back the modified register values */
    if (bit < 16) {
        *PINSEL_L  = sel;
        *PINMODE_L = mode;
    } else {
        *PINSEL_H  = sel;
        *PINMODE_H = mode;
    }

    if (flags & GPIO_CFG_OUTPUT) {
        CM3BBREG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, bit) = 1;
    } else {
        CM3BBREG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, bit) = 0;
    }

    if (flags & GPIO_CFG_MULTIDRIVE) {
        *PINMODE_OD |= _BV(bit);
    } else {
        *PINMODE_OD &= ~_BV(bit);
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
 * On the LPC17xx interrupts are triggered on rising, falling or both
 * edges. Level triggering is not supported.
 *
 * After registering, interrupts are disabled. Calling GpioIrqEnable()
 * is required to activate the interrupt.
 *
 * The following code fragment registers an interrupt handler which is
 * called on a rising edge of bit 4 of the first GPIO port:
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
 *     GpioRegisterIrqHandler(&sig_GPIO0, 4, PinChange, NULL);
 *     GpioIrqSetMode(&sig_GPIO0, 4, NUT_IRQMODE_RISINGEDGE);
 *     GpioIrqEnable(&sig_GPIO0, 4);
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
int GpioRegisterIrqHandler(GPIO_SIGNAL * sig, int bit, void (*handler) (void *), void *arg)
{
    int rc = 0;

    if (sig->ios_vector == 0) {
        /* This is the first call. Allocate the vector table. */
        sig->ios_vector = malloc(sizeof(GPIO_VECTOR) * 32);
        if (sig->ios_vector) {
            memset(sig->ios_vector, 0, sizeof(GPIO_VECTOR) * 32);
            /* Register our internal PIO interrupt service. */
            if (sig_PIO.ir_handler == NULL) {
                rc = NutRegisterIrqHandler(&sig_PIO, sig->ios_handler, NULL);
                if (rc == 0) {
                    /* Clear any pending interrupts */
                    LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
                    LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;

                    rc = NutIrqEnable(&sig_PIO);
                }
            }
        }
        else {
            return -1;
        }
    }
    sig->ios_vector[bit].iov_handler = handler;
    sig->ios_vector[bit].iov_arg = arg;

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
int GpioIrqEnable(GPIO_SIGNAL * sig, int bit)
{
    return (sig->ios_ctl) (sig, NUT_IRQCTL_ENABLE, NULL, bit);
}

/*!
 * \brief Disable a specified GPIO interrupt.
 *
 * \param sig Interrupt to disable.
 * \param bit Bit number of the specified bank/port.
 *
 * \return 0 on success, -1 otherwise.
 */
int GpioIrqDisable(GPIO_SIGNAL * sig, int bit)
{
    return (sig->ios_ctl) (sig, NUT_IRQCTL_DISABLE, NULL, bit);
}

/*!
 * \brief Query the status of a specified GPIO interrupt.
 *
 * A related interrupt handler must have been registered before calling
 * this function. See GpioRegisterIrqHandler().
 *
 * \param sig Interrupt to query
 * \param bit Bit number of the specified bank/port.
 *
 * \return 0 if interrupt is disabled, 1 of enabled
 */

int GpioIrqStatus(GPIO_SIGNAL * sig, int bit)
{
    uint32_t status;
    (sig->ios_ctl) (sig, NUT_IRQCTL_STATUS, &status, bit);

    return status;
}

/*!
 * \brief Set the GPIO interrupt mode for a pin
 *
 * \param sig Interrupt to configure.
 * \param bit Bit number of the specified bank/port.
 * \param mode one of the following modes:
 *          NUT_IRQMODE_RISINGEDGE,
 *          NUT_IRQMODE_FALLINGEDGE,
 *          NUT_IRQMODE_BOTHEDGE,
 *          NUT_IRQMODE_NONE,
 *
 * \return 0 on success, -1 otherwise.
 */
int GpioIrqSetMode(GPIO_SIGNAL * sig, int bit, int mode)
{
    return (sig->ios_ctl) (sig, NUT_IRQCTL_SETMODE, &mode, bit);
}

/*@}*/
