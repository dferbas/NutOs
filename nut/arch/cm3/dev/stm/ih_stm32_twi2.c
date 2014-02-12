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
 * THIS SOFTWARE IS PROVIDED BY EMBEDDED IT AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EMBEDDED IT
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

/*!
 *\verbatim
 * $Id: ih_stm32_twi2.c 4608 2012-09-14 13:14:15Z haraldkipp $
 *\endverbatim
 */

#include <arch/cm3.h>
#include <dev/irqreg.h>

#ifndef NUT_IRQPRI_TWI
/* According Errata Sheet 14574 Rev. 9 some I2C erratic behaviour can be overcome
 * by setting IRQ almost non interruptable.
 */
#define NUT_IRQPRI_TWI  0
#endif

#ifdef NUT_PERFMON
static int TwoWireIrqCtl(IRQn_Type IRQn, void(*ifunc)(void*), IRQ_HANDLER *ih, int cmd, void *param);
#else
static int TwoWireIrqCtl(IRQn_Type IRQn, void(*ifunc)(void*), int cmd, void *param);
#endif
static int TwoWireIrqCtl_Ev(int cmd, void *param);
static int TwoWireIrqCtl_Er(int cmd, void *param);

IRQ_HANDLER sig_TWI2_EV = {
#ifdef NUT_PERFMON
    0,                  /* Interrupt counter, ir_count. */
#endif
    NULL,               /* Passed argument, ir_arg. */
    NULL,               /* Handler subroutine, ir_handler. */
    TwoWireIrqCtl_Ev    /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_TWI2_ER = {
#ifdef NUT_PERFMON
    0,                  /* Interrupt counter, ir_count. */
#endif
    NULL,               /* Passed argument, ir_arg. */
    NULL,               /* Handler subroutine, ir_handler. */
    TwoWireIrqCtl_Er    /* Interrupt control, ir_ctl. */
};

/*!
 * \brief Two Wire Interface Event Interrupt Entry.
 */
static void TwoWireIrqEntry_Ev(void *arg)
{
#ifdef NUT_PERFMON
    sig_TWI2_EV.ir_count++;
#endif
    if (sig_TWI2_EV.ir_handler) {
        (sig_TWI2_EV.ir_handler) (sig_TWI2_EV.ir_arg);
    }
}

/*!
 * \brief Two Wire Interface Error Interrupt Entry.
 */
static void TwoWireIrqEntry_Er(void *arg)
{
#ifdef NUT_PERFMON
    sig_TWI2_ER.ir_count++;
#endif
    if (sig_TWI2_ER.ir_handler) {
        (sig_TWI2_ER.ir_handler) (sig_TWI2_ER.ir_arg);
    }
}

static int TwoWireIrqCtl_Ev(int cmd, void *param)
{
    return TwoWireIrqCtl(I2C2_EV_IRQn, TwoWireIrqEntry_Ev, cmd, param);
}

static int TwoWireIrqCtl_Er(int cmd, void *param)
{
    return TwoWireIrqCtl(I2C2_ER_IRQn, TwoWireIrqEntry_Er, cmd, param);
}

/*!
 * \brief Two Wire Interface interrupt control.
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
#ifdef NUT_PERFMON
static int TwoWireIrqCtl(IRQn_Type IRQn, void(*ifunc)(void*), IRQ_HANDLER *ih, int cmd, void *param)
#else
static int TwoWireIrqCtl(IRQn_Type IRQn, void(*ifunc)(void*), int cmd, void *param)
#endif
{
    int rc = 0;
    unsigned int *ival = (unsigned int *)param;
    uint_fast8_t enabled = NVIC_GetEnableIRQ(IRQn);

    /* Disable interrupt. */
    if (enabled) {
        NVIC_DisableIRQ(IRQn);
    }

    switch(cmd) {
    case NUT_IRQCTL_INIT:
        /* Set the vector. */
        Cortex_RegisterInt(IRQn, ifunc);
        /* Initialize Event IRQ with defined priority. */
        NVIC_SetPriority(IRQn, NUT_IRQPRI_TWI);
        /* Initialize Error IRQ with defined priority. */
        NVIC_SetPriority(IRQn+1, NUT_IRQPRI_TWI+1);
        /* Clear interrupt */
        NVIC_ClearPendingIRQ(IRQn);
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
        *ival = NUT_IRQMODE_LEVEL;
        break;
    case NUT_IRQCTL_SETMODE:
        break;
    case NUT_IRQCTL_GETPRIO:
        *ival = NVIC_GetPriority(IRQn);
        break;
    case NUT_IRQCTL_SETPRIO:
        NVIC_SetPriority(IRQn, *ival);
        break;
#ifdef NUT_PERFMON
    case NUT_IRQCTL_GETCOUNT:
        *ival = (unsigned int)ih->ir_count;
        ih->ir_count = 0;
        break;
#endif
    default:
        rc = -1;
        break;
    }

    /* Enable interrupt. */
    if (enabled) {
        NVIC_EnableIRQ(IRQn);
    }
    return rc;
}
