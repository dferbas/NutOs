/*
 * Copyright (C) 2006 by egnite Software GmbH. All rights reserved.
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
 *
 */

/*
 * $Log$
 * Revision 1.2  2008/08/11 06:59:42  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.1  2006/06/06 19:08:05  haraldkipp
 * First release
 *
 */

#include <sys/timer.h>
#include <dev/watchdog.h>

/*
 * Start the hardware watch dog timer.
 */
uint32_t NutWatchDogStart(uint32_t ms, uint32_t xmode)
{
#if defined(__AVR__)
    return AvrWatchDogStart(ms);
#elif defined(MCU_AT91R40008) || defined(MCU_AT91SAM7X) || defined(MCU_AT91SAM7S) || defined(MCU_AT91SAM7SE)
    return At91WatchDogStart(ms, xmode);
#elif defined(__AVR32__)
    return Avr32WatchDogStart(ms);
#elif defined(MCU_LPC17xx)
    return Lpc17xxWatchDogStart(ms, xmode);
#elif defined(MCU_MCF5225)
	return Mcf5225WatchDogStart(ms);
#elif defined(MCU_MCF51CN) || defined(MCU_MCF51QE)
	return Mcf51CopStart(ms);
#else
    return 0;
#endif
}

/*
 * Re-start the hardware watch dog timer.
 */
void NutWatchDogRestart(void)
{
#if defined(__AVR__)
    AvrWatchDogRestart();
#elif defined(MCU_AT91R40008) || defined(MCU_AT91SAM7X) || defined(MCU_AT91SAM7S) || defined(MCU_AT91SAM7SE)
    At91WatchDogRestart();
#elif defined(__AVR32__)
    Avr32WatchDogRestart();
#elif defined(MCU_LPC17xx)
    Lpc17xxWatchDogRestart();
#elif defined(MCU_MCF5225)
	Mcf5225WatchDogRestart();
#elif defined(MCU_MCF51CN) || defined(MCU_MCF51QE)
	Mcf51CopRestart();
#endif
}

/*
 * Disable the hardware watch dog timer.
 */
void NutWatchDogDisable(void)
{
#if defined(__AVR__)
    AvrWatchDogDisable();
#elif defined(MCU_AT91R40008) || defined(MCU_AT91SAM7X) || defined(MCU_AT91SAM7S) || defined(MCU_AT91SAM7SE)
    At91WatchDogDisable();
#elif defined(__AVR32__)
    Avr32WatchDogDisable();
#elif defined(MCU_LPC17xx)
    Lpc17xxWatchDogDisable();
#elif defined(MCU_MCF5225)
	Mcf5225WatchDogDisable();
#elif defined(MCU_MCF51CN) || defined(MCU_MCF51QE)
	Mcf51CopDisable();
#endif
}

/*
 * Enable the hardware watch dog timer.
 */
void NutWatchDogEnable(void)
{
#if defined(__AVR__)
    AvrWatchDogEnable();
#elif defined(MCU_AT91R40008) || defined(MCU_AT91SAM7X) || defined(MCU_AT91SAM7S) || defined(MCU_AT91SAM7SE)
    At91WatchDogEnable();
#elif defined(__AVR32__)
    Avr32WatchDogEnable();
#elif defined(MCU_LPC17xx)
    Lpc17xxWatchDogEnable();
#elif defined(MCU_MCF5225)
	Mcf5225WatchDogEnable();
#elif defined(MCU_MCF51CN) || defined(MCU_MCF51QE)
	Mcf51CopEnable();
#endif
}

