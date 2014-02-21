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

#ifndef _ARCH_M68K_H_
#error "Do not include this file directly. Use arch/m68k.h instead!"
#endif

/* RTC Registers */
#define MCF_RTC_HOURMIN                      (*(volatile uint32_t*)(0x40180000))
#define MCF_RTC_SECONDS                      (*(volatile uint32_t*)(0x40180004))
#define MCF_RTC_ALRM_HM                      (*(volatile uint32_t*)(0x40180008))
#define MCF_RTC_ALRM_SEC                     (*(volatile uint32_t*)(0x4018000C))
#define MCF_RTC_RTCCTL                       (*(volatile uint32_t*)(0x40180010))
#define MCF_RTC_RTCISR                       (*(volatile uint32_t*)(0x40180014))
#define MCF_RTC_RTCIENR                      (*(volatile uint32_t*)(0x40180018))
#define MCF_RTC_STPWCH                       (*(volatile uint32_t*)(0x4018001C))
#define MCF_RTC_DAYS                         (*(volatile uint32_t*)(0x40180020))
#define MCF_RTC_ALRM_DAY                     (*(volatile uint32_t*)(0x40180024))
#define MCF_RTC_RTCGOCU                      (*(volatile uint32_t*)(0x40180034))
#define MCF_RTC_RTCGOCL                      (*(volatile uint32_t*)(0x40180038))

/* MCF_RTC_HOURMIN */
#define MCF_RTC_HOURMIN_MINUTES(x)           (((x)&0x3F)<<0)
#define MCF_RTC_HOURMIN_HOURS(x)             (((x)&0x1F)<<0x8)

/* MCF_RTC_SECONDS */
#define MCF_RTC_SECONDS_SECONDS(x)           (((x)&0x3F)<<0)

/* MCF_RTC_ALRM_HM */
#define MCF_RTC_ALRM_HM_MINUTES(x)           (((x)&0x3F)<<0)
#define MCF_RTC_ALRM_HM_HOURS(x)             (((x)&0x1F)<<0x8)

/* MCF_RTC_ALRM_SEC */
#define MCF_RTC_ALRM_SEC_SECONDS(x)          (((x)&0x3F)<<0)

/* MCF_RTC_RTCCTL */
#define MCF_RTC_RTCCTL_SWR                   (0x1)
#define MCF_RTC_RTCCTL_EN                    (0x80)

/* MCF_RTC_RTCISR */
#define MCF_RTC_RTCISR_SW                    (0x1)
#define MCF_RTC_RTCISR_MIN                   (0x2)
#define MCF_RTC_RTCISR_ALM                   (0x4)
#define MCF_RTC_RTCISR_DAY                   (0x8)
#define MCF_RTC_RTCISR_1HZ                   (0x10)
#define MCF_RTC_RTCISR_HR                    (0x20)

/* MCF_RTC_RTCIENR */
#define MCF_RTC_RTCIENR_SW                   (0x1)
#define MCF_RTC_RTCIENR_MIN                  (0x2)
#define MCF_RTC_RTCIENR_ALM                  (0x4)
#define MCF_RTC_RTCIENR_DAY                  (0x8)
#define MCF_RTC_RTCIENR_1HZ                  (0x10)
#define MCF_RTC_RTCIENR_HR                   (0x20)
#define MCF_RTC_RTCIENR_ALL          		 (0x0000FFFF)

/* MCF_RTC_STPWCH */
#define MCF_RTC_STPWCH_CNT(x)                (((x)&0x3F)<<0)

/* MCF_RTC_DAYS */
#define MCF_RTC_DAYS_DAYS(x)                 (((x)&0xFFFF)<<0)

/* MCF_RTC_ALRM_DAY */
#define MCF_RTC_ALRM_DAY_DAYSAL(x)           (((x)&0xFFFF)<<0)

/* MCF_RTC_RTCGOCU */
#define MCF_RTC_RTCGOCU_RTCGOCNT(x)          (((x)&0xFFFF)<<0)

/* MCF_RTC_RTCGOCL */
#define MCF_RTC_RTCGOCL_RTCGOCNT(x)          (((x)&0xFFFF)<<0)
