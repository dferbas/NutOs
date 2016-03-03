/*
 * Copyright 2012-2016 by Embedded Technologies s.r.o. All rights reserved.
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

#include <arch/m68k.h>
#include <dev/irqreg.h>

/*!
 * \addtogroup xgMcf51
 */
/*@{*/

static int IrqCtl1_Rx(int cmd, void *param);
static int IrqCtl1_Tx(int cmd, void *param);

static int IrqCtl2_Rx(int cmd, void *param);
static int IrqCtl2_Tx(int cmd, void *param);

static int IrqCtl3_Rx(int cmd, void *param);
static int IrqCtl3_Tx(int cmd, void *param);

IRQ_HANDLER sig_SCI1_RX = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1_Rx
    };

IRQ_HANDLER sig_SCI1_TX = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1_Tx
    };

IRQ_HANDLER sig_SCI2_RX = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl2_Rx
    };

IRQ_HANDLER sig_SCI2_TX = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl2_Tx
    };

IRQ_HANDLER sig_SCI3_RX = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl3_Rx
    };

IRQ_HANDLER sig_SCI3_TX = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl3_Tx
    };

/*!
 * \brief SCI1_RX interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl1_Rx(int cmd, void *param)
{
    return IrqCtlCommon(&sig_SCI1_RX, cmd, param, &MCF_SCI_C2(1), MCF_SCI_C2_RIE, 1);
}

/*!
 * \brief SCI1_TX interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl1_Tx(int cmd, void *param)
{
    return IrqCtlCommon(&sig_SCI1_TX, cmd, param, &MCF_SCI_C2(1), MCF_SCI_C2_TIE, 1);
}

/*!
 * \brief SCI2_RX interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl2_Rx(int cmd, void *param)
{
    return IrqCtlCommon(&sig_SCI2_RX, cmd, param, &MCF_SCI_C2(2), MCF_SCI_C2_RIE, 1);
}

/*!
 * \brief SCI2_TX interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl2_Tx(int cmd, void *param)
{
    return IrqCtlCommon(&sig_SCI2_TX, cmd, param, &MCF_SCI_C2(2), MCF_SCI_C2_TIE, 1);
}

/*!
 * \brief SCI3_RX interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl3_Rx(int cmd, void *param)
{
    return IrqCtlCommon(&sig_SCI3_RX, cmd, param, &MCF_SCI_C2(3), MCF_SCI_C2_RIE, 1);
}

/*!
 * \brief SCI3_TX interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl3_Tx(int cmd, void *param)
{
    return IrqCtlCommon(&sig_SCI2_TX, cmd, param, &MCF_SCI_C2(3), MCF_SCI_C2_TIE, 1);
}


SIGNAL(IH_SCI1_RX)
{
    /* Interrupt is cleared in driver by reading to SCI data register. */

    CallHandler(&sig_SCI1_RX);
}

SIGNAL(IH_SCI1_TX)
{
    /* Interrupt is cleared in driver by writing to SCI data register. */

    CallHandler(&sig_SCI1_TX);
}

SIGNAL(IH_SCI2_RX)
{
    /* Interrupt is cleared in driver by reading to SCI data register. */

    CallHandler(&sig_SCI2_RX);
}

SIGNAL(IH_SCI2_TX)
{
    /* Interrupt is cleared in driver by writing to SCI data register. */

    CallHandler(&sig_SCI2_TX);
}

SIGNAL(IH_SCI3_RX)
{
    /* Interrupt is cleared in driver by reading to SCI data register. */

    CallHandler(&sig_SCI3_RX);
}

SIGNAL(IH_SCI3_TX)
{
    /* Interrupt is cleared in driver by writing to SCI data register (SCIxD). */

    CallHandler(&sig_SCI3_TX);
}
