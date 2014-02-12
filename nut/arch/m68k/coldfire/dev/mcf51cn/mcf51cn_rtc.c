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

#include <sys/atom.h>

#include <string.h>

#include <dev/mcf51cn_rtc.h>


#define RTC_START_COUNTER	 MCF_RTC_SC = MCF_RTC_SC_RTIE \
										| MCF_RTC_SC_RTIF \
										| (0xE << MCF_RTC_SC_RTCPS_BITNUM) \
										| (0x1 << MCF_RTC_SC_RTCLKS_BITNUM) ; /* external clock 25 Mhz */
#define RTC_STOP_COUNTER	 MCF_RTC_SC = 0x0F;

static long timeInSeconds;

static void Mcf51RtcInterrupt(void *arg);

int Mcf51RtcInit(NUTRTC *rtc)
{
	MCF_RTC_SC = 0x00;
	// (100ms / 25) * 0xF7 = 1s
	MCF_RTC_MOD = 0xF9;
	MCF_RTC_MOD = MCF_RTC_MOD;

	if (NutRegisterIrqHandler(&sig_RTC, Mcf51RtcInterrupt, NULL)) {
		// We do not free buffer as this would cost ROM and is not likely
		return -1;
	    }
	NutIrqEnable(&sig_RTC);

	RTC_START_COUNTER

	return 0;
}

int Mcf51RtcSetClock(NUTRTC *rtc, const struct _tm *tm)
{
	time_t time;
	time = mktime((struct _tm *)tm);

	if (time) {
	   	timeInSeconds = time;
	}
	return 0;
}


int Mcf51RtcGetClock(NUTRTC *rtc, struct _tm *tm)
{
	time_t time = timeInSeconds;
	localtime_r(&time,tm);
	return 0;
}

static void Mcf51RtcInterrupt(void *arg)
{
	/* reset RTC request flag */
	MCF_RTC_SC |= 0x80;

	/* increment 1s */
	timeInSeconds ++;

}

NUTRTC rtcMcf51 = {
  /*.dcb           = */ NULL,               /*!< Driver control block */
  /*.rtc_init      = */ Mcf51RtcInit,       /*!< Hardware initializatiuon, rtc_init */
  /*.rtc_gettime   = */ Mcf51RtcGetClock,   /*!< Read date and time, rtc_gettime */
  /*.rtc_settime   = */ Mcf51RtcSetClock,   /*!< Set date and time, rtc_settime */
  /*.rtc_getalarm  = */ NULL,               /*!< Read alarm date and time, rtc_getalarm */
  /*.rtc_setalarm  = */ NULL,               /*!< Set alarm date and time, rtc_setalarm */
  /*.rtc_getstatus = */ NULL,               /*!< Read status flags, rtc_getstatus */
  /*.rtc_clrstatus = */ NULL,               /*!< Clear status flags, rtc_clrstatus */
  /*.alarm         = */ NULL,               /*!< Handle for alarm event queue, not supported right now */
};

