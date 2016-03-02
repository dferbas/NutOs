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

static int IrqCtl1(int cmd, void *param);
static int IrqCtl2(int cmd, void *param);

IRQ_HANDLER sig_IIC1 =
{
#ifdef NUT_PERFMON
		0,
#endif
		NULL,
		NULL, IrqCtl1
};

IRQ_HANDLER sig_IIC2 =
{
#ifdef NUT_PERFMON
		0,
#endif
		NULL,
		NULL, IrqCtl2
};


/*! \brief IIC1 interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return IrqCtlCommon 0 on success, -1 otherwise.
 */
static int IrqCtl1(int cmd, void *param)
{
	return IrqCtlCommon(&sig_IIC1, cmd, param, &MCF_IIC_CR(0), MCF_IIC_CR_IICIE, 1);
}

/*! \brief IIC2 interrupt control.
 *
 * \param cmd 		Control command.
 * \param *param 	Pointer to optional parameter.
 *
 * \return IrqCtlCommon 0 on success, -1 otherwise.
 */
static int IrqCtl2(int cmd, void *param)
{
	return IrqCtlCommon(&sig_IIC2, cmd, param, &MCF_IIC_CR(1), MCF_IIC_CR_IICIE, 1);
}

#if defined (MCU_MCF51CN)
SIGNAL(IH_IIC1)
{
	MCF_IIC_SR(0) = MCF_IIC_SR_IICIF;
	CallHandler(&sig_IIC1);
}

SIGNAL(IH_IIC2)
{
	MCF_IIC_SR(1) = MCF_IIC_SR_IICIF;
	CallHandler(&sig_IIC2);
}
#elif defined (MCU_MCF51QE)
SIGNAL(IH_IIC_X)
{
	if (MCF_IIC_SR(0) & MCF_IIC_SR_IICIF)
	{
		MCF_IIC_SR(0) = MCF_IIC_SR_IICIF;
		CallHandler(&sig_IIC1);
	}

	if (MCF_IIC_SR(1) & MCF_IIC_SR_IICIF)
	{
		MCF_IIC_SR(1) = MCF_IIC_SR_IICIF;
		CallHandler(&sig_IIC2);
	}
}
#else
#warning "Unknown Coldfire V1 MCU Family defined"
#endif
