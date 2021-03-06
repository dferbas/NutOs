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

#include <dev/irqreg.h>
#include <sys/atom.h>
#include <sys/nutdebug.h>

/*!
 * \addtogroup xgMcf52
 */
/*@{*/

/*!
 * \brief Common interrupt control.
 *
 * \param cmd   Control command.
 *              - NUT_IRQCTL_INIT Initialize and disable interrupt.
 *              - NUT_IRQCTL_STATUS Query interrupt status.
 *              - NUT_IRQCTL_ENABLE Enable interrupt.
 *              - NUT_IRQCTL_DISABLE Disable interrupt.
 *              - NUT_IRQCTL_GETCOUNT Query and clear interrupt counter.
 * \param param Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
int IrqCtlCommon(IRQ_HANDLER *sig_handler, int cmd, void *param, volatile void *reg_imr,
		uint32_t imr_mask, uint8_t imr_size, volatile uint32_t *reg_imr_ic, uint32_t imr_mask_ic,
		volatile uint8_t *reg_icr, uint8_t ipl)
{
	/*
	 * Coldfire V2 Core Interrupt Subsystem
	 *
	 *  - Interrupts may be masked
	 *      - In peripheral registers
	 *      - In interrupt controller
	 *
	 *  - Interrupt levels and priorities
	 *      - All interrupts have ICR register which must be configured
	 *
	 *  NOTE: The purpose of interrupt handlers is to unify work with interrupts
	 *        on different MCU Families from devices point of view.
	 *
	 *        For example FEC device is implemented the same way in MCF51xxx and
	 *        MCF52xxx. There are following differences only:
	 *           - register address space        -> this is unified by MACROS in headers
	 *           - differen interrupts handling  -> this is unified by interrupt handler
	 *               - the difference is, that on MCF52xxx the interrupt must be
	 *                 enabled twice
	 *                      - in FEC control register (the same as MCF51xxx)
	 *                      - and in sven in Interrupt Controller Module (MCF52xxx only)
	 */

	/*
	 * MCF52259RM.pdf - 16.3.2 Interrupt Mask Registers (IMRHn, IMRLn)
	 *
	 * NOTE: Spurious Interrupts
	 *
	 * A spurious interrupt may occur if an interrupt source is being masked in the
	 * interrupt controller mask register (IMR) or a module�s interrupt mask
	 * register while the interrupt mask in the status register (SR[I]) is set to a value
	 * lower than the interrupt�s level. This is because by the time the status
	 * register acknowledges this interrupt, the interrupt has been masked. A
	 * spurious interrupt is generated because the CPU cannot determine the
	 * interrupt source.
	 * To avoid this situation for interrupts sources with levels 1�6, first write a
	 * higher level interrupt mask to the status register, before setting the mask in
	 * the IMR or the module�s interrupt mask register. After the mask is set, return
	 * the interrupt mask in the status register to its previous value. Because level
	 * 7 interrupts cannot be disabled in the status register prior to masking, use of
	 * the IMR or module interrupt mask registers to disable level 7 interrupts is
	 * not recommended.
	 */

	int rc = 0;
	unsigned int *ival = (unsigned int *) param;
	volatile uint8_t *reg8_imr = (volatile uint8_t *) reg_imr;
	volatile uint16_t *reg16_imr = (volatile uint16_t *) reg_imr;
	volatile uint32_t *reg32_imr = (volatile uint32_t *) reg_imr;
	uint32_t enabled = 0;

	NutUseCritical();
	NutEnterCritical(); /* Prevent spurious interrupts */

	/*
	 * Disable interrupt.
	 */
	switch (imr_size)
	{
		case 1:
			enabled = *reg8_imr & imr_mask;
			if (enabled)
			{
				*reg8_imr &= ~imr_mask;
			}
			break;
		case 2:
			enabled = *reg16_imr & imr_mask;
			if (enabled)
			{
				*reg16_imr &= ~imr_mask;
			}
			break;
		case 4:
			enabled = *reg32_imr & imr_mask;
			if (enabled)
			{
				*reg32_imr &= ~imr_mask;
			}
			break;
		default:
			NUTASSERT(0);
			break;
	}

	/*
	 * Process command.
	 */
	switch (cmd)
	{
		case NUT_IRQCTL_INIT:
			enabled = 0; /* Make sure the interrupt is disabled */
			*reg_icr = ipl; /* Configure interrupt level and priority */
			*reg_imr_ic &= ~imr_mask_ic; /* Unmask interrupt in Interrupt Controller Module */
			break;
		case NUT_IRQCTL_STATUS:
			if (enabled)
			{
				*ival |= 1;
			}
			else
			{
				*ival &= ~1;
			}
			break;
		case NUT_IRQCTL_ENABLE:
			enabled = 1;
			break;
		case NUT_IRQCTL_DISABLE:
			enabled = 0;
			break;
#ifdef NUT_PERFMON
			case NUT_IRQCTL_GETCOUNT:
			*ival = (unsigned int)sig_handler->ir_count;
			sig_handler->ir_count = 0;
			break;
#endif
		default:
			rc = -1;
			break;
	}

	/*
	 * Enable interrupt.
	 */
	if (enabled)
	{
		switch (imr_size)
		{
			case 1:
				*reg8_imr |= imr_mask;
				break;
			case 2:
				*reg16_imr |= imr_mask;
				break;
			case 4:
				*reg32_imr |= imr_mask;
				break;
			default:
				NUTASSERT(0);
				break;
		}
	}

	NutExitCritical();

	return rc;
}
