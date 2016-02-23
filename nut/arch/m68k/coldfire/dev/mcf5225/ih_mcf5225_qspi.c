/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

static int QspiIrqCtl(int cmd, void *param);

IRQ_HANDLER sig_QSPI_TF = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    QspiIrqCtl     /* Interrupt control, ir_ctl. */
};

static int QspiIrqCtl(int cmd, void *param)
{
    return IrqCtlCommon(&sig_QSPI_TF, cmd, param,
            &MCF_QSPI_QIR, MCF_QSPI_QIR_SPIFE | MCF_QSPI_QIR_ABRTE | MCF_QSPI_QIR_WCEFE | MCF_QSPI_QIR_ABRTL, 2,
            &MCF_INTC_IMRL(0), MCF_INTC_IMRL_INT_MASK18 | MCF_INTC_IMRL_MASKALL,
            &MCF_INTC_ICR18(0), IPL_QSPI_TF);
}

/*
 * The QIR register has 3 interrupt flags - SPIF, ABRT and WCEF.
 * If 1 bit is or-ed, all 3 flags are cleared - see also note in the GPT driver.
 * Writing only 1 bit is impossible for the QIR, where there are other R/W bits.
 */

SIGNAL(IH_QSPI_TF)
{
	if (MCF_QSPI_QIR & (MCF_QSPI_QIR_SPIF))
	{
	    MCF_QSPI_QIR |= MCF_QSPI_QIR_SPIF;			//clear finished flag (interrupt reason)

		CallHandler(&sig_QSPI_TF);
	}
}

