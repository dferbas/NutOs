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
#include <cfg/clock.h>
#include <dev/watchdog.h>
#include <stdio.h>

void InitSystem(void)
{
    uint8_t reg;

    /* Configure system options register 1 */
    /* SOPT1: COPE=1,COPT=1,STOPE=0,WAITE=1,RSTOPE=0,BKGDPE=1,RSTPE=0 */
    reg = 0xD2;
//#ifndef NUT_WDT_ENABLE
//    reg &= ~(MCF_SOPT1_COPE); // Disable COP (Watchdog timer)
//#endif
    /* All SOPT1 bit fields, except WAITE, are write-once. Therefore for
     * the write-once bits, only the first write after reset is honored. */
    MCF_SOPT1 = reg;

    /* SOPT2: COPCLKS=0,SPI1PS=1,ACIC2=0,IIC1PS=0,ACIC1=0 */
    MCF_SOPT2 = MCF_SOPT2_SPI1PS | MCF_SOPT2_COPCLKS;

    /* Configure System Power Management Status and Control 1 Register */
    /* SPMSC1: LVDF=0,LVDACK=0,LVDIE=0,LVDRE=1,LVDSE=1,LVDE=1,BGBE=0 */
    reg = 0x1c; // default
//#ifndef NUT_LVD_ENABLE
//    MCF_SPMSC1 &= ~(MCF_SPMSC1_LVDE);
//#endif
    MCF_SPMSC1 = reg;

    /* SPMSC2: LPR=0,LPRS=0,LPWUI=0,PPDF=0,PPDACK=0,PPDE=1,PPDC=0 */
    MCF_SPMSC2 = 0x02;

    /* SPMSC3: LVDV=0,LVWV=0,LVWIE=0 */
    MCF_SPMSC3 &= ~0x38;
}

void InitClock(void)
{
	Mcf51qeIcsInitClock();
}
