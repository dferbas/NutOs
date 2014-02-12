/*
 * Copyright (C) 2006 by egnite Software GmbH and Christian Welzel.
 * All rights reserved.
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

/*!
 * \file dev/ds1307rtc.c
 * \brief RTC and RAM routines for the Dallas DS1307 clock chip.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.6  2008/08/11 06:59:42  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.5  2007/09/11 17:32:30  haraldkipp
 * Last minute fix (bug #1786271)
 *
 * Revision 1.4  2006/10/08 16:39:33  haraldkipp
 * Corrected range check in DS1307RamRead(). Thanks to Helmut Holbach.
 *
 * Revision 1.3  2006/10/05 17:18:49  haraldkipp
 * Hardware independant RTC layer added.
 *
 * Revision 1.2  2006/08/25 13:42:55  olereinhardt
 * avr-gcc 3.4 does not understand binary representation of numbers,
 * changed to hex
 *
 * Revision 1.1  2006/06/30 22:05:51  christianwelzel
 * Initial check in.
 *
 *
 * \endverbatim
 */

#include <cfg/os.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <dev/rtc.h>
#include <arch/cm3/stm/stm32f10x_rtc.h>
#include <arch/cm3/stm/stm32f10x_rcc.h>
#include <arch/cm3/stm/stm32f10x_pwr.h>
#include <time.h>

#include <stdlib.h>
#include <string.h>

static void Stm32RtcRtoffPoll()
{
    while((RTC->CRL & RTC_FLAG_RTOFF)==0);
    return;
};

/*!
 * \brief Get date and time from an DS1307 hardware clock.
 *
 * \param tm Points to a structure that receives the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int Stm32RtcGetClock(struct _tm *tm)
{
    int rc=-1;
    time_t time;
    RTC->CRL&= ~(RTC_FLAG_RSF);
    while(!(RTC->CRL & RTC_FLAG_RSF));
    time=((RTC->CNTL|(RTC->CNTH<<16)));
    localtime_r(&time,tm);
    rc=0;
    return rc;
}

/*!
 * \brief Set the DS1307 hardware clock.
 *
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int Stm32RtcSetClock(const struct _tm *tm)
{
   time_t time;
   Stm32RtcRtoffPoll();
   RTC->CRL|= RTC_FLAG_CNF;
   time=mktime(tm);
   RTC->CNTL =time& 0xffffUL;
   RTC->CNTH =(time >> 16) &0xffffUL;
   RTC->CRL&= ~(RTC_FLAG_CNF);
   Stm32RtcRtoffPoll();
   return 0;
}


/*!
 * \brief Initialize the RTC in stm32f10x controller
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
int Stm32RtcInit(void)
{
    int rc=-1;
    uint32_t temp;


   RCC->APB1ENR |= RCC_APB1Periph_BKP|RCC_APB1Periph_PWR;
   PWR->CR |= PWR_CR_DBP;
   RCC->BDCR|= (1<<16);//Backup domain reset;
   RCC->BDCR&= ~(1<<16);//Backup domain reset;
   PWR->CR|= PWR_CR_DBP;

   temp=RCC->BDCR;
   temp&= ~((1<<9)|(1<<8));
   temp |= RCC_RTCCLKSource_HSE_Div128;//HSE/128
   temp |= 1<<15;
   RCC->BDCR=temp;
   RTC->CRL&= ~(RTC_FLAG_RSF);
   while(!(RTC->CRL & RTC_FLAG_RSF));

   Stm32RtcRtoffPoll();
   RTC->CRL|= RTC_FLAG_CNF;
   RTC->PRLH = 0;
   RTC->PRLL = (8000000ul/128)-1;//Для делителя на 128 от кварца 8МГц
   //Устанавливаем вменяемое время
   RTC->CNTL= 1;
   RTC->CNTH= 0;//1 january 1970
   RTC->CRL&= ~(RTC_FLAG_CNF);
   Stm32RtcRtoffPoll();
   rc=0;
   return rc;
}

NUTRTC rtcStm32 = {
    Stm32RtcInit,         /*!< Hardware initializatiuon, rtc_init */
    Stm32RtcGetClock,  /*!< Read date and time, rtc_gettime */
    Stm32RtcSetClock,  /*!< Set date and time, rtc_settime */
    NULL,               /*!< Read alarm date and time, rtc_getalarm */
    NULL,               /*!< Set alarm date and time, rtc_setalarm */
    NULL,               /*!< Read status flags, rtc_getstatus */
    NULL                /*!< Clear status flags, rtc_clrstatus */
};
