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
//#include <arch/m68k/coldfire/mcf5225/mcf5225_intc.h>

/*!
 * \addtogroup xgMcfCommon
 */
/*@{*/

static int IrqCtlRb(int cmd, void *param);
static int IrqCtlRf(int cmd, void *param);
static int IrqCtlTb(int cmd, void *param);
static int IrqCtlTf(int cmd, void *param);

IRQ_HANDLER sig_FEC_RB = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlRb
    };

IRQ_HANDLER sig_FEC_RF = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlRf
    };

IRQ_HANDLER sig_FEC_TB = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlTb
    };

IRQ_HANDLER sig_FEC_TF = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlTf
    };

/*!
 * \brief Rb interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlRb(int cmd, void *param)
{
    return IrqCtlCommon(&sig_FEC_RB, cmd, param, &MCF_FEC_EIMR, MCF_FEC_EIMR_RXB, 4
#if defined (MCU_MCF5225)
    		, &MCF_INTC_IMRL(0), MCF_INTC_IMRL_INT_MASK28 | MCF_INTC_IMRL_MASKALL, &MCF_INTC_ICR28(0), IPL_FEC_RB
#endif
    		);
}


/*!
 * \brief Rf interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlRf(int cmd, void *param)
{
    return IrqCtlCommon(&sig_FEC_RF, cmd, param, &MCF_FEC_EIMR, MCF_FEC_EIMR_RXF, 4
#if defined (MCU_MCF5225)
    		, &MCF_INTC_IMRL(0), MCF_INTC_IMRL_INT_MASK27 | MCF_INTC_IMRL_MASKALL, &MCF_INTC_ICR27(0), IPL_FEC_RF
#endif
    		);
}

/*!
 * \brief Tb interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlTb(int cmd, void *param)
{
    return IrqCtlCommon(&sig_FEC_TB, cmd, param, &MCF_FEC_EIMR, MCF_FEC_EIMR_TXB, 4
#if defined (MCU_MCF5225)
    		, &MCF_INTC_IMRL(0), MCF_INTC_IMRL_INT_MASK24 | MCF_INTC_IMRL_MASKALL, &MCF_INTC_ICR24(0), IPL_FEC_TB
#endif
    		);
}

/*!
 * \brief Tf interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlTf(int cmd, void *param)
{
    return IrqCtlCommon(&sig_FEC_TF, cmd, param, &MCF_FEC_EIMR, MCF_FEC_EIMR_TXF, 4
#if defined (MCU_MCF5225)
    		, &MCF_INTC_IMRL(0), MCF_INTC_IMRL_INT_MASK23 | MCF_INTC_IMRL_MASKALL, &MCF_INTC_ICR23(0), IPL_FEC_TF
#endif
    		);
}

SIGNAL(IH_FEC_RB)
{
	MCF_FEC_EIR = MCF_FEC_EIR_RXB;
    CallHandler(&sig_FEC_RB);
}

SIGNAL(IH_FEC_RF)
{
	MCF_FEC_EIR = MCF_FEC_EIR_RXF;
    CallHandler(&sig_FEC_RF);
}

SIGNAL(IH_FEC_TB)
{
	MCF_FEC_EIR = MCF_FEC_EIR_TXB;
    CallHandler(&sig_FEC_TB);
}

SIGNAL(IH_FEC_TF)
{
	MCF_FEC_EIR = MCF_FEC_EIR_TXF;
    CallHandler(&sig_FEC_TF);
}
