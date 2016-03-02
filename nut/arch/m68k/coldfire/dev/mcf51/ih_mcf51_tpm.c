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

static int IrqCtl1_ovfl(int cmd, void *param);
static int IrqCtl1_Ch0(int cmd, void *param);
static int IrqCtl1_Ch1(int cmd, void *param);
static int IrqCtl1_Ch2(int cmd, void *param);
static int IrqCtl2_ovfl(int cmd, void *param);
static int IrqCtl2_Ch0(int cmd, void *param);
static int IrqCtl2_Ch1(int cmd, void *param);
static int IrqCtl2_Ch2(int cmd, void *param);

IRQ_HANDLER sig_TPM1_OVFL = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1_ovfl
    };

IRQ_HANDLER sig_TPM1_CH0 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1_Ch0
    };

IRQ_HANDLER sig_TPM1_CH1 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1_Ch1
    };

IRQ_HANDLER sig_TPM1_CH2 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1_Ch2
    };

IRQ_HANDLER sig_TPM2_OVFL = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl2_ovfl
    };

IRQ_HANDLER sig_TPM2_CH0 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl2_Ch0
    };

IRQ_HANDLER sig_TPM2_CH1 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl2_Ch1
    };

IRQ_HANDLER sig_TPM2_CH2 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl2_Ch2
    };

/*! \brief TPM1_OVFL interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl1_ovfl(int cmd, void *param)
{
    return IrqCtlCommon(&sig_TPM1_OVFL, cmd, param, &MCF_TPM_SC(1), MCF_TPM_SC_TOIE, 1);
}


/*! \brief TPM1_CH0 interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl1_Ch0(int cmd, void *param)
{
    return IrqCtlCommon(&sig_TPM1_CH0, cmd, param, &MCF_TPM_CSC(1, 0), MCF_TPM_CSC_CHnIE, 1);
}


/*! \brief TPM1_CH1 interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl1_Ch1(int cmd, void *param)
{
    return IrqCtlCommon(&sig_TPM1_CH1, cmd, param, &MCF_TPM_CSC(1, 1), MCF_TPM_CSC_CHnIE, 1);
}


/*! \brief TPM1_CH2 interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl1_Ch2(int cmd, void *param)
{
    return IrqCtlCommon(&sig_TPM1_CH2, cmd, param, &MCF_TPM_CSC(1, 2), MCF_TPM_CSC_CHnIE, 1);
}


/*! \brief TPM2_OVFL interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl2_ovfl(int cmd, void *param)
{
    return IrqCtlCommon(&sig_TPM2_OVFL, cmd, param, &MCF_TPM_SC(2), MCF_TPM_SC_TOIE, 1);
}


/*! \brief TPM2_CH0 interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl2_Ch0(int cmd, void *param)
{
    return IrqCtlCommon(&sig_TPM2_CH0, cmd, param, &MCF_TPM_CSC(2, 0), MCF_TPM_CSC_CHnIE, 1);
}

/*! \brief TPM2_CH1 interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl2_Ch1(int cmd, void *param)
{
    return IrqCtlCommon(&sig_TPM2_CH1, cmd, param, &MCF_TPM_CSC(2, 1), MCF_TPM_CSC_CHnIE, 1);
}

/*! \brief TPM2_CH2 interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl2_Ch2(int cmd, void *param)
{
    return IrqCtlCommon(&sig_TPM2_CH2, cmd, param, &MCF_TPM_CSC(2, 2), MCF_TPM_CSC_CHnIE, 1);
}

SIGNAL(IH_TPM1_OVFL)
{
	/* Clear counter by writing to MCF_TPM_SC. */
	MCF_TPM_SC(1) &= ~MCF_TPM_SC_TOF;
    CallHandler(&sig_TPM1_OVFL);
}/* ?????*/


SIGNAL(IH_TPM1_CH0)
{
	/* Clear CHnF by reading TPMxCnSC while this bit is set and then writing a logic 0 to it. */
	MCF_TPM_CSC(1, 0) &= ~MCF_TPM_CSC_CHnF;
    CallHandler(&sig_TPM1_CH0);
}

SIGNAL(IH_TPM1_CH1)
{
	/* Clear CHnF by reading TPMxCnSC while this bit is set and then writing a logic 0 to it. */
	MCF_TPM_CSC(1, 1) &= ~MCF_TPM_CSC_CHnF;
    CallHandler(&sig_TPM1_CH1);
}

SIGNAL(IH_TPM1_CH2)
{
	/* Clear CHnF by reading TPMxCnSC while this bit is set and then writing a logic 0 to it. */
	MCF_TPM_CSC(1, 2) &= ~MCF_TPM_CSC_CHnF;
    CallHandler(&sig_TPM1_CH2);
}

SIGNAL(IH_TPM2_OVFL)
{
	/* Clear counter by writing to MCF_TPM_SC. */
	MCF_TPM_SC(2) &= ~MCF_TPM_SC_TOF;
    CallHandler(&sig_TPM2_OVFL);
}

SIGNAL(IH_TPM2_CH0)
{
	/* Clear CHnF by reading TPMxCnSC while this bit is set and then writing a logic 0 to it. */
	MCF_TPM_CSC(2, 0) &= ~MCF_TPM_CSC_CHnF;
    CallHandler(&sig_TPM2_CH0);
}

SIGNAL(IH_TPM2_CH1)
{
	/* Clear CHnF by reading TPMxCnSC while this bit is set and then writing a logic 0 to it. */
	MCF_TPM_CSC(2, 1) &= ~MCF_TPM_CSC_CHnF;
    CallHandler(&sig_TPM2_CH1);
}

SIGNAL(IH_TPM2_CH2)
{
	/* Clear CHnF by reading TPMxCnSC while this bit is set and then writing a logic 0 to it. */
	MCF_TPM_CSC(2, 2) &= ~MCF_TPM_CSC_CHnF;
    CallHandler(&sig_TPM2_CH2);
}
