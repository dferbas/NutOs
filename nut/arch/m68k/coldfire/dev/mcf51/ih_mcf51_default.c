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

static int IrqCtl(int cmd, void *param);
static void IrqHandler(void *arg);

IRQ_HANDLER sig_DEFAULT =
{
#ifdef NUT_PERFMON
		0,
#endif
		NULL, IrqHandler, IrqCtl };

/*!
 * \brief Default interrupt entry.
 */
//NUTSIGNAL(IH_DEFAULT, sig_DEFAULT)
SIGNAL(IH_DEFAULT)
{
	CallHandler(&sig_DEFAULT);
}

/*!
 * \brief Default interrupt handler.
 */
static void IrqHandler(void *arg)
{
	//  void Put(char ch)
	//  {
	//      #define DEVNUM 1
	//
	//      /* Wait until the Tx register is empty */
	//      while ((MCF_SCI_S1(DEVNUM) & MCF_SCI_S1_TDRE) == 0)
	//          ;
	//
	//      /* Send the character */
	//      MCF_SCI_D(DEVNUM) = (uint8_t) ch;
	//  }
	//
	//  int Write(const void *buffer, int len)
	//  {
	//      int c = len;
	//      const char *cp = (const char *) buffer;
	//
	//      while (c--) {
	//          Put(*cp++);
	//      }
	//
	//      return len;
	//  }
	//
	//  Write("gogo", 4);

	while (1)
		;
}

/*!
 * \brief Default interrupt control.
 *
 * \param cmd   	Control command.
 *              	- NUT_IRQCTL_INIT Initialize and disable interrupt.
 * \param param 	Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int IrqCtl(int cmd, void *param)
{
	return (cmd == NUT_IRQCTL_INIT) ? 0 : -1;
}
