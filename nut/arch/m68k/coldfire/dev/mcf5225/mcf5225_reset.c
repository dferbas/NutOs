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
#include <dev/reset.h>

/*!
 * \addtogroup xgMcf5225
 */
/*@{*/


/*!
 * \brief Reset control registers
 *
 * \note SOFTRST - Allows software to request a reset. The reset caused by setting this bit clears this bit.
 *
 * 		1 	Software reset request
 *		0 	No software reset request
 */
void Mcf5225_Reset(void)
{
	MCF_RCM_RCR |= MCF_RCM_RCR_SOFTRST;
}

/*!
 * \brief Reset status register (RSR)
 *
 * \note The RSR contains a status bit for every reset source. RSR can be read at any time. Writing to RSR has no effect.
 */
int Mcf5225_ResetCause(void)
{
	// TODO: One or more status bits may be set at the same time.

	uint8_t rsr = MCF_RCM_RSR;

	if (rsr & MCF_RCM_RSR_POR)
		return NUT_RSTTYP_POWERUP;

	if (rsr & MCF_RCM_RSR_EXT)
		return NUT_RSTTYP_EXTERNAL;

	if (rsr & MCF_RCM_RSR_BWD)
		return NUT_RSTTYP_BACKUP_WATCHDOG;

	if (rsr & MCF_RCM_RSR_WDR)
		return NUT_RSTTYP_WATCHDOG;

	if (rsr & MCF_RCM_RSR_LVD)
		return NUT_RSTTYP_BROWNOUT;

	if (rsr & MCF_RCM_RSR_LOL)
		return NUT_RSTTYP_LOSS_OF_LOCK;

	if (rsr & MCF_RCM_RSR_LOC)
		return NUT_RSTTYP_LOSS_OF_CLOCK;

	if (rsr & MCF_RCM_RSR_SOFT)
		return NUT_RSTTYP_SOFTWARE;

	return NUT_RSTTYP_UNKNOWN;
}
