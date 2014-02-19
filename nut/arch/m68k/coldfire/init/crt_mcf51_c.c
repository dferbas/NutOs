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
#include <stdio.h>
#include <arch/m68k.h>
#include <cfg/clock.h>
#include <dev/watchdog.h>

void InitClock(void)
{
	MCF_SPMSC1 = 0x1C;
	MCF_SPMSC2 = 0x02;
	MCF_SPMSC3 &= ~0x38;
	  
#if defined(NUT_WDT_ENABLE)
    /* Default enable COP Watchdog Time-out with 1024ms period. For Restart
    Watchdog, write 0x55 and 0xAA values to the System Reset Status Register */
	NutWatchDogRestart();
#else
    /* For Disable Cop Watchdog, move 0x10 value to the System Options Register 1 */
	MCF_SOPT1 = 0x10;
#endif
	  
//#if defined(NUT_WDT_ENABLE)
    /* Default enable COP Watchdog Time-out with 1024ms period. For Restart
    Watchdog, write 0x55 and 0xAA values to the System Reset Status Register */
//	movel 0xFFFF8100,%d0
//	moveal %d0,%a0
//	moveb 0x55,%a0@
//	moveb 0xAA,%a0@
//#else
    /* For Disable Cop Watchdog, move 0x10 value to the System Options Register 1
	on address (0xFFFF8101). */
//	movel 0xFFFF8101,%d0
//	moveal %d0,%a0
//	moveb 0x10,%a0@
//#endif

	  /* Initialization of CPU registers */
	  /*lint -save  -e950 Disable MISRA rule (1.1) checking. */
//	  asm {
//	    /* VBR: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADDRESS=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
//	    clr.l d0
//	    movec d0,VBR
//	    /* CPUCR: ARD=0,IRD=0,IAE=0,IME=0,BWD=0,??=0,FSD=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
//	    clr.l d0
//	    movec d0,CPUCR
//	  }
	  /*lint -restore Enable MISRA rule (1.1) checking. */
	  /* PTDPF1: D4=3 */
	  MCF_PTDPF1 |= 0x03;
	  /* PTDPF1: D5=3 */
	  MCF_PTDPF1 |= 0x0C;

	Mcf51cnMcgInitClock();
}
