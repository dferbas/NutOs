/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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
 *
 */

/*!
 * \verbatim
 * $Id: ih_stm32_uart4.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>
#include <sys/device.h>

#ifndef NUT_IRQPRI_UART4
#define NUT_IRQPRI_UART4  4
#endif

extern NUTDEVICE devUartStm32_4;

static int Uart4IrqCtl(int cmd, void *param);

/*!
 * \breif IRQ Handler for UART4.
 */
IRQ_HANDLER sig_UART4 = {
#ifdef NUT_PERFMON
    0,                  /* Interrupt counter, ir_count. */
#endif
    &devUartStm32_4,   /* Passed argument, ir_arg. */
    NULL,               /* Handler subroutine, ir_handler. */
    Uart4IrqCtl         /* Interrupt control, ir_ctl. */
};

/*!
 * \brief UART4 interrupt entry.
 */
void Uart4IrqEntry(void *arg)
{
#ifdef NUT_PERFMON
    sig_UART4.ir_count++;
#endif
    if (sig_UART4.ir_handler) {
        (sig_UART4.ir_handler) (sig_UART4.ir_arg);
    }
}

/*!
 * \brief UART4 interrupt control.
 *
 * \param cmd   Control command.
 *              - NUT_IRQCTL_INIT Initialize and disable interrupt.
 *              - NUT_IRQCTL_STATUS Query interrupt status.
 *              - NUT_IRQCTL_ENABLE Enable interrupt.
 *              - NUT_IRQCTL_DISABLE Disable interrupt.
 *              - NUT_IRQCTL_GETMODE Query interrupt mode.
 *              - NUT_IRQCTL_SETMODE Set interrupt mode (NUT_IRQMODE_LEVEL or NUT_IRQMODE_EDGE).
 *              - NUT_IRQCTL_GETPRIO Query interrupt priority.
 *              - NUT_IRQCTL_SETPRIO Set interrupt priority.
 *              - NUT_IRQCTL_GETCOUNT Query and clear interrupt counter.
 * \param param Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Uart4IrqCtl(int cmd, void *param)
{
    int rc = 0;
    unsigned int *ival = (unsigned int *)param;
    int_fast8_t enabled = NVIC_GetEnableIRQ(UART4_IRQn);

    /* Disable interrupt. */
    if (enabled) {
        NVIC_DisableIRQ(UART4_IRQn);
    }

    switch(cmd) {
    case NUT_IRQCTL_INIT:
        /* Set the vector. */
        Cortex_RegisterInt(UART4_IRQn,Uart4IrqEntry);
        /* Initialize with defined priority. */
        NVIC_SetPriority(UART4_IRQn,NUT_IRQPRI_UART4);
        /* Clear interrupt */
        NVIC_ClearPendingIRQ(UART4_IRQn);
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
    case NUT_IRQCTL_GETMODE:
            *ival = NUT_IRQMODE_EDGE;
        break;
    case NUT_IRQCTL_SETMODE:
            rc = -1;
        break;
    case NUT_IRQCTL_GETPRIO:
        *ival = NVIC_GetPriority(UART4_IRQn);
        break;
    case NUT_IRQCTL_SETPRIO:
    NVIC_SetPriority(UART4_IRQn,*ival);
        break;
#ifdef NUT_PERFMON
    case NUT_IRQCTL_GETCOUNT:
        *ival = (unsigned int)sig_UART4.ir_count;
        sig_UART4.ir_count = 0;
        break;
#endif
    default:
        rc = -1;
        break;
    }

    /* Enable interrupt. */
    if (enabled) {
        NVIC_EnableIRQ(UART4_IRQn);
    }
    return rc;
}
