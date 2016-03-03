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
 * \addtogroup xgMcf5225
 */
/*@{*/

static int IrqCtlPai(int cmd, void *param);
static int IrqCtlPaov(int cmd, void *param);
static int IrqCtlC0F(int cmd, void *param);
static int IrqCtlC1F(int cmd, void *param);
static int IrqCtlC2F(int cmd, void *param);
static int IrqCtlC3F(int cmd, void *param);

IRQ_HANDLER sig_GPT_PAI = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlPai
    };

IRQ_HANDLER sig_GPT_PAOV = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlPaov
    };

IRQ_HANDLER sig_GPT_C0F = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlC0F
    };

IRQ_HANDLER sig_GPT_C1F = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlC1F
    };

IRQ_HANDLER sig_GPT_C2F = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlC2F
    };

IRQ_HANDLER sig_GPT_C3F = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtlC3F
    };

/*!
 * \brief Pai interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlPai(int cmd, void *param)
{
    return IrqCtlCommon(&sig_GPT_PAI, cmd, param,
            &MCF_GPT_GPTPACTL, MCF_GPT_GPTPACTL_PAI, 1,
            &MCF_INTC_IMRH(0), MCF_INTC_IMRH_INT_MASK42,
            &MCF_INTC_ICR42(0), IPL_GPT_PAI);
}

/*!
 * \brief Paov interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlPaov(int cmd, void *param)
{
    return IrqCtlCommon(&sig_GPT_PAOV, cmd, param,
            &MCF_GPT_GPTPACTL, MCF_GPT_GPTPACTL_PAOVI, 1,
            &MCF_INTC_IMRH(0), MCF_INTC_IMRH_INT_MASK43,
            &MCF_INTC_ICR43(0), IPL_GPT_PAOV);
}

/*!
 * \brief C0F interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlC0F(int cmd, void *param)
{
    return IrqCtlCommon(&sig_GPT_C0F, cmd, param,
            &MCF_GPT_GPTIE, MCF_GPT_GPTIE_CI(MCF_GPT_CHANNEL0), 1,
            &MCF_INTC_IMRH(0), MCF_INTC_IMRH_INT_MASK44,
            &MCF_INTC_ICR44(0), IPL_GPT_C0F);
}

/*!
 * \brief C1F interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlC1F(int cmd, void *param)
{
    return IrqCtlCommon(&sig_GPT_C1F, cmd, param,
            &MCF_GPT_GPTIE, MCF_GPT_GPTIE_CI(MCF_GPT_CHANNEL1), 1,
            &MCF_INTC_IMRH(0), MCF_INTC_IMRH_INT_MASK45,
            &MCF_INTC_ICR45(0), IPL_GPT_C1F);
}

/*!
 * \brief C2F interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlC2F(int cmd, void *param)
{
    return IrqCtlCommon(&sig_GPT_C2F, cmd, param,
            &MCF_GPT_GPTIE, MCF_GPT_GPTIE_CI(MCF_GPT_CHANNEL2), 1,
            &MCF_INTC_IMRH(0), MCF_INTC_IMRH_INT_MASK46,
            &MCF_INTC_ICR46(0), IPL_GPT_C2F);
}

/*!
 * \brief C3F interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtlC3F(int cmd, void *param)
{
    return IrqCtlCommon(&sig_GPT_C3F, cmd, param,
            &MCF_GPT_GPTIE, MCF_GPT_GPTIE_CI(MCF_GPT_CHANNEL3), 1,
            &MCF_INTC_IMRH(0), MCF_INTC_IMRH_INT_MASK47,
            &MCF_INTC_ICR47(0), IPL_GPT_C3F);
}

/*
 * Following registers handle multiple interrupt events in 1 8-bit register.
 * Direct writing of only 1 bit clears the exact bit, other bits are untouched.
 *
 * This is a difference from other peripheral drivers, where is only 1 interrupt flag,
 * other bits are R/W, in that case or to the register is correct.
 * The only exception is the QSPI's QIR register, where are 3 interrupt bits and other R/W bits.
 */

SIGNAL(IH_GPT_PAI)
{
	MCF_GPT_GPTPAFLG = MCF_GPT_GPTPAFLG_PAIF;
    CallHandler(&sig_GPT_PAI);
}

SIGNAL(IH_GPT_PAOV)
{
    MCF_GPT_GPTPAFLG = MCF_GPT_GPTPAFLG_PAOVF;
    CallHandler(&sig_GPT_PAOV);
}

SIGNAL(IH_GPT_C0F)
{
    MCF_GPT_GPTFLG1 = MCF_GPT_GPTFLG1_CF(MCF_GPT_CHANNEL0);
    CallHandler(&sig_GPT_C0F);
}

SIGNAL(IH_GPT_C1F)
{
    MCF_GPT_GPTFLG1 = MCF_GPT_GPTFLG1_CF(MCF_GPT_CHANNEL1);
    CallHandler(&sig_GPT_C1F);
}

SIGNAL(IH_GPT_C2F)
{
    MCF_GPT_GPTFLG1 = MCF_GPT_GPTFLG1_CF(MCF_GPT_CHANNEL2);
    CallHandler(&sig_GPT_C2F);
}

SIGNAL(IH_GPT_C3F)
{
    MCF_GPT_GPTFLG1 = MCF_GPT_GPTFLG1_CF(MCF_GPT_CHANNEL3);
    CallHandler(&sig_GPT_C3F);
}
