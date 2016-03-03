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

/*!
 * \addtogroup xgMcf51
 */
/*@{*/

/*!
 * \brief Not supported by CPU
 */
uint32_t Mcf51CopStart(uint32_t ms)
{
    /*
     * Not supported by CPU
     *
     * Watchdog enable/disable is possible using system option register (SOPT1),
     * which is controlled by different module
     *
     * In addition, the register is once-write only, co it cannot be modified.
     *
     * In default configuration, watchdog is started and it may be disabled only.
     * The default period is 1,024 ms.
     *
     */

    return 0;
}

/*!
 * \brief Not supported by CPU
 */
void Mcf51CopRestart(void)
{
#if defined(MCU_MCF51QE)
	/* The COP counter is reset by writing any value to the address of SRS.
	 * This write does not affect the data in the read-only SRS. */
	MCF_SRS = 0x00;
#elif defined(MCU_MCF51CN)
	/* The COP counter is reset by writing 0x55 and 0xAA (in this order)
	 * to the address of SRS during the selected time-out period.
	 * Writes do not affect the data in the read-only SRS. */
    MCF_SRS = 0x55;
    MCF_SRS = 0xAA;
#endif
}

/*!
 * \brief Not supported by CPU
 */
void Mcf51CopDisable(void)
{
    /*
     * Not supported by CPU
     *
     * Watchdog enable/disable is possible using system option register (SOPT1),
     * which is controlled by different module
     *
     * In addition, the register is once-write only, co it cannot be modified.
     */
}

/*!
 * \brief Not supported by CPU
 */
void Mcf51CopEnable(void)
{
    /*
     * Not supported by CPU
     *
     * Watchdog enable/disable is possible using system option register (SOPT1),
     * which is controlled by different module
     *
     * In addition, the register is once-write only, co it cannot be modified.
     */
}
