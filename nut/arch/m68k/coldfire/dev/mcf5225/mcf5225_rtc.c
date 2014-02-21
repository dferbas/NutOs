/*
 * Copyright 2014 by Embedded Technologies s.r.o
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
#include <dev/rtc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timer.h>
#include <time.h>

/*!
 * \brief Get date and time from an MCF5225 hardware clock.
 *
 * \param tm Points to a structure that receives the date and time 
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int Mcf5225RtcGetClock(NUTRTC *rtc, struct _tm *tm)
{
	int rslt = -1;

    if (tm) {
    	MCF_RTC_RTCISR |= MCF_RTC_RTCISR_MIN;

		time_t timer = MCF_RTC_SECONDS
				+ (MCF_RTC_HOURMIN & 0x3F) * 60
				+ ((MCF_RTC_HOURMIN >> 8) & 0x1F) * 60 * 60
				+ MCF_RTC_DAYS * 24 * 60 * 60;

		if (MCF_RTC_RTCISR & MCF_RTC_RTCISR_MIN)
		{
			timer = MCF_RTC_SECONDS
				+ (MCF_RTC_HOURMIN & 0x3F) * 60
				+ ((MCF_RTC_HOURMIN >> 8) & 0x1F) * 60 * 60
				+ MCF_RTC_DAYS * 24 * 60 * 60;
		}

		gmtime_r(&timer, tm);

		rslt = 0;
    }
    return rslt;
}

/*!
 * \brief Set an MCF5225 hardware clock.
 *
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
static int Mcf5225RtcSetClock(NUTRTC *rtc, const struct _tm *tm)
{
	int rslt = -1;

    if (tm) {
        struct _tm  tm_tmp;
    	time_t      timer;

    	memcpy(&tm_tmp, tm, sizeof(tm_tmp));    // copy const to non-const
    	timer = _mkgmtime(&tm_tmp);

    	MCF_RTC_RTCCTL = 0;

        MCF_RTC_DAYS = MCF_RTC_DAYS_DAYS(timer / (60 * 60 * 24));
        MCF_RTC_HOURMIN = MCF_RTC_HOURMIN_MINUTES((timer / 60) % 60)
        				| MCF_RTC_HOURMIN_HOURS((timer / (60 * 60)) % 24);
    	MCF_RTC_SECONDS = timer % 60;

    	MCF_RTC_RTCCTL = MCF_RTC_RTCCTL_EN;

        rslt = 0;
    }
    return rslt;
}

/*!
 * \brief Initialize the RTC in Mcf5225 controller
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
static int Mcf5225RtcInit(NUTRTC *rtc)
{
	/* Disable RTC */
    MCF_RTC_RTCCTL = 0;

    /* Configure RTC clock source */
    MCF_CLOCK_RTCCR = MCF_CLOCK_RTCCR_EXTALEN | MCF_CLOCK_RTCCR_OSCEN | MCF_CLOCK_RTCCR_REFS | MCF_CLOCK_RTCCR_RTCSEL;

    /* Configure clock divider */
	MCF_RTC_RTCGOCL = 0x00002000; // We use 32.768 kHz oscilator
	MCF_RTC_RTCGOCU = 0;

	/* Enable RTC */
	MCF_RTC_RTCCTL = MCF_RTC_RTCCTL_EN;

    return 0;
}

NUTRTC rtcMcf5225 = {
  /*.dcb           = */ NULL,               /*!< Driver control block */
  /*.rtc_init      = */ Mcf5225RtcInit,     /*!< Hardware initializatiuon, rtc_init */
  /*.rtc_gettime   = */ Mcf5225RtcGetClock, /*!< Read date and time, rtc_gettime */
  /*.rtc_settime   = */ Mcf5225RtcSetClock, /*!< Set date and time, rtc_settime */
  /*.rtc_getalarm  = */ NULL,               /*!< Read alarm date and time, rtc_getalarm */
  /*.rtc_setalarm  = */ NULL,               /*!< Set alarm date and time, rtc_setalarm */
  /*.rtc_getstatus = */ NULL,               /*!< Read status flags, rtc_getstatus */
  /*.rtc_clrstatus = */ NULL,               /*!< Clear status flags, rtc_clrstatus */
  /*.alarm         = */ NULL,               /*!< Handle for alarm event queue, not supported right now */
};
