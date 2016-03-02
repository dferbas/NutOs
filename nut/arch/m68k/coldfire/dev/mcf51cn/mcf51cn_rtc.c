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
#include <dev/rtc.h>
#include <string.h>
#include <sys/atom.h>
#include <sys/heap.h>

/*!
 * \addtogroup xgMcf51cn
 */
/*@{*/

typedef struct _mcf51cn_rtc_dcb mcf51cn_rtc_dcb;
struct _mcf51cn_rtc_dcb
{
	int seconds;
};

/*! \brief Interrupt handler for RTC
 *
 */
static void Mcf51cnRtcInterrupt(void *arg)
{
	NUTRTC *rtc = (NUTRTC *) arg;
	((mcf51cn_rtc_dcb *) rtc->dcb)->seconds++;
}

/*! \brief Get date and time from an Mcf51cn hardware clock.
 *
 * \param tm Points to a structure that receives the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Mcf51cnRtcGetClock(NUTRTC *rtc, struct _tm *tm)
{
	time_t time = ((mcf51cn_rtc_dcb *) rtc->dcb)->seconds;
	localtime_r(&time, tm);
	return 0;
}

/*! \brief Set the Mcf51cn hardware clock.
 *
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Mcf51cnRtcSetClock(NUTRTC *rtc, const struct _tm *tm)
{
	time_t time;
	time = mktime((struct _tm *) tm);

	if (time)
	{
		((mcf51cn_rtc_dcb *) rtc->dcb)->seconds = time;
	}
	return 0;
}

/*! \brief Initialize the RTC in Mcf51cn controller
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
static int Mcf51cnRtcInit(NUTRTC *rtc)
{
	rtc->dcb = NutHeapAllocClear(sizeof(mcf51cn_rtc_dcb));
	if (rtc->dcb == NULL)
	{
		return -1;
	}

	if (NutRegisterIrqHandler(&sig_RTC, Mcf51cnRtcInterrupt, rtc))
	{
		NutHeapFree(rtc->dcb);
		return -1;
	}

	/* Disable interrupts */
	MCF_RTC_SC = 0;

	/* Use external 25MHz oscilator as clock source */
	MCF_RTC_SC |= 0x1 << MCF_RTC_SC_RTCLKS_BITNUM;

	/* Set prescaler to 100000 */
	MCF_RTC_SC |= 0xE << MCF_RTC_SC_RTCPS_BITNUM;

	/* Set modulo register to 250 */
	MCF_RTC_MOD = 0xFA;

	/* Start timer */
	NutIrqEnable(&sig_RTC);

	return 0;
}

NUTRTC rtcMcf51cn =
{
/*.dcb           = */NULL, /*!< Driver control block */
/*.rtc_init      = */Mcf51cnRtcInit, /*!< Hardware initializatiuon, rtc_init */
/*.rtc_gettime   = */Mcf51cnRtcGetClock, /*!< Read date and time, rtc_gettime */
/*.rtc_settime   = */Mcf51cnRtcSetClock, /*!< Set date and time, rtc_settime */
/*.rtc_getalarm  = */NULL, /*!< Read alarm date and time, rtc_getalarm */
/*.rtc_setalarm  = */NULL, /*!< Set alarm date and time, rtc_setalarm */
/*.rtc_getstatus = */NULL, /*!< Read status flags, rtc_getstatus */
/*.rtc_clrstatus = */NULL, /*!< Clear status flags, rtc_clrstatus */
/*.alarm         = */NULL, /*!< Handle for alarm event queue, not supported right now */
};
