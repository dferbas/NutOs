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

/*!
 * \verbatim
 * $Id: ih_stm32_exti.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <dev/irqreg.h>
#include <dev/gpio.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32xxxx_gpio.h>

/*!
 * \brief External interrupt 0 control.
 *
 * \param cmd   Control command.
 *              - NUT_IRQCTL_INIT Initialize and disable interrupt.
 *              - NUT_IRQCTL_STATUS Query interrupt status.
 *              - NUT_IRQCTL_ENABLE Enable interrupt.
 *              - NUT_IRQCTL_DISABLE Disable interrupt.
 *              - NUT_IRQCTL_GETPRIO Query interrupt priority.
 *              - NUT_IRQCTL_GETCOUNT Query and clear interrupt counter.
 * \param param Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int InterruptCtl(int cmd, void *param)
{
    int rc = 0;
    unsigned int *ival = (unsigned int *)param;
    int_fast8_t enabled = NVIC_GetEnableIRQ(THIS_IRQn);

    /* Disable interrupt. */
    if (enabled) {
        NVIC_DisableIRQ(THIS_IRQn);
    }

    switch(cmd) {
    case NUT_IRQCTL_INIT:
        /* Set the vector. */
        Cortex_RegisterInt(THIS_IRQn, THIS_ENTRY);
        /* Initialize with defined priority. */
        NVIC_SetPriority(THIS_IRQn, THIS_IRQPRI);
        /* Clear interrupt */
        NVIC_ClearPendingIRQ(THIS_IRQn);
        break;
    case NUT_IRQCTL_STATUS:
        if (enabled) {
            *ival |= 1;
        }
        else {
            *ival &= ~1;
        }
        break;
    case NUT_IRQCTL_ENABLE:
        enabled = 1;
        break;
    case NUT_IRQCTL_DISABLE:
        enabled = 0;
        break;
/* This needs to be set at pin level configuration for the interrupt
    case NUT_IRQCTL_GETMODE:
        if ( EXTI->FTSR &= (1<<THIS_EXTI))
            *ival = NUT_IRQMODE_FALLINGEDGE;
        else if( EXTI->RTSR &= (1<<THIS_EXTI))
            *ival = NUT_IRQMODE_RISINGEDGE;
        break;
    case NUT_IRQCTL_SETMODE:
        if (*ival == NUT_IRQMODE_NONE) {
            EXTI->FTSR &= ~(1<<THIS_EXTI);
            EXTI->RTSR &= ~(1<<THIS_EXTI);
        }
        if (*ival == NUT_IRQMODE_FALLINGEDGE) {
            EXTI->FTSR |= (1<<THIS_EXTI);
        } else if (*ival == NUT_IRQMODE_RISINGEDGE) {
            EXTI->RTSR |= (1<<THIS_EXTI);
        } else  {
            rc = -1;
        }
        break;
*/
    case NUT_IRQCTL_GETPRIO:
        *ival = THIS_IRQPRI;
        break;
#ifdef NUT_PERFMON
    case NUT_IRQCTL_GETCOUNT:
        *ival = (unsigned int)THIS_SIG.ir_count;
        THIS_SIG.ir_count = 0;
        break;
#endif
    default:
        rc = -1;
        break;
    }

    /* Enable interrupt. */
    if (enabled) {
        NVIC_EnableIRQ(THIS_IRQn);
    }
    return rc;
}


