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
 * $Id: lpc177x_8x_gpio.c $
 * \endverbatim
 */

#include <arch/cm3.h>

#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#else
#warning "Unknown LPC family"
#endif

#include <dev/gpio.h>

/*!
 * \brief PIO interrupt service.
 */
static void Lpc17xxGpioIsr(void *arg)
{
    GPIO_VECTOR *vct;

    uint32_t isr = LPC_GPIOINT->IntStatus;
    uint32_t port_status;

    if (isr & _BV(NUTGPIO_PORT0)) {
        port_status = LPC_GPIOINT->IO0IntStatR | LPC_GPIOINT->IO0IntStatF;

        vct = sig_GPIO0.ios_vector;
        while (port_status) {
            if ((port_status & 1) != 0 && vct->iov_handler) {
                (vct->iov_handler) (vct->iov_arg);
            }
            port_status >>= 1;
            vct++;
        }

        LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
    }
    if (isr & _BV(NUTGPIO_PORT2)) {
        port_status = LPC_GPIOINT->IO2IntStatR | LPC_GPIOINT->IO2IntStatF;

        vct = sig_GPIO2.ios_vector;
        while (port_status) {
            if ((port_status & 1) != 0 && vct->iov_handler) {
                (vct->iov_handler) (vct->iov_arg);
            }
            port_status >>= 1;
            vct++;
        }

        LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
    }
}

/*!
 * \brief PIO interrupt control.
 */
static int Lpc17xxGpioCtrl(GPIO_SIGNAL * sig, int cmd, void *param, int bit)
{
    int rc = 0;
    uint32_t *ival = (uint32_t *) param;

    switch (cmd) {
        case NUT_IRQCTL_STATUS:
            if (sig->enabled & _BV(bit)) {
                *ival = 1;
            } else {
                *ival = 0;
            }
            break;

        case NUT_IRQCTL_ENABLE:
            sig->enabled |= _BV(bit);
            break;

        case NUT_IRQCTL_DISABLE:
            sig->enabled &= ~_BV(bit);
            break;

        case NUT_IRQCTL_GETMODE:
            if ((sig->mode_rising_enabled & _BV(bit)) && ((sig->mode_falling_enabled & _BV(bit)) == 0)) {
                *ival = NUT_IRQMODE_RISINGEDGE;
            } else
            if (((sig->mode_rising_enabled & _BV(bit)) == 0) && (sig->mode_falling_enabled & _BV(bit))) {
                *ival = NUT_IRQMODE_FALLINGEDGE;
            } else
            if ((sig->mode_rising_enabled & _BV(bit)) && (sig->mode_falling_enabled & _BV(bit))) {
                *ival = NUT_IRQMODE_BOTHEDGE;
            } else {
                *ival = NUT_IRQMODE_NONE;
            }
            break;

        case NUT_IRQCTL_SETMODE:
            switch (*ival) {
                case NUT_IRQMODE_RISINGEDGE:
                    sig->mode_rising_enabled  |= _BV(bit);
                    sig->mode_falling_enabled &= ~_BV(bit);
                    break;
                case NUT_IRQMODE_FALLINGEDGE:
                    sig->mode_rising_enabled  &= ~_BV(bit);
                    sig->mode_falling_enabled |= _BV(bit);
                    break;
                case NUT_IRQMODE_BOTHEDGE:
                    sig->mode_rising_enabled  |= _BV(bit);
                    sig->mode_falling_enabled |= _BV(bit);
                    break;
                case NUT_IRQMODE_NONE:
                    sig->mode_rising_enabled  &= ~_BV(bit);
                    sig->mode_falling_enabled &= ~_BV(bit);
                    break;
                default:
                    rc = -1;
            }
            break;


        default:
            rc = -1;
            break;
    }

    /* Enable / disable interrupt and set mode */

    switch (sig->ios_port) {
        case NUTGPIO_PORT0:
            LPC_GPIOINT->IO0IntEnR = sig->mode_rising_enabled & sig->enabled;
            LPC_GPIOINT->IO0IntEnF = sig->mode_falling_enabled & sig->enabled;
            break;

        case NUTGPIO_PORT2:
            LPC_GPIOINT->IO2IntEnR = sig->mode_rising_enabled & sig->enabled;
            LPC_GPIOINT->IO2IntEnF = sig->mode_falling_enabled & sig->enabled;
            break;
        default:
            rc = -1;
            break;
    }

    return rc;
}

GPIO_SIGNAL sig_GPIO0 = {
    NUTGPIO_PORT0,   /* ios_port */
    Lpc17xxGpioIsr,  /* ios_handler */
    Lpc17xxGpioCtrl, /* ios_ctl */
    NULL,            /* ios_vector */
    0,               /* mode_rising_enabled */
    0                /* mode_falling_enabled */
};

GPIO_SIGNAL sig_GPIO2 = {
    NUTGPIO_PORT2,   /* ios_port */
    Lpc17xxGpioIsr,  /* ios_handler */
    Lpc17xxGpioCtrl, /* ios_ctl */
    NULL,            /* ios_vector */
    0,               /* mode_rising_enabled */
    0                /* mode_falling_enabled */

};
