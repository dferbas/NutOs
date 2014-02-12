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

#ifndef MCF51CN_RTC_GPIO_H_
#define MCF51CN_RTC_GPIO_H_


/* RTC Status and Control Register */
#define MCF_RTC_SC 							 (*(volatile uint8_t *)(0xFFFF82C0))
#define MCF_RTC_SC_RTIE                      0x10
#define MCF_RTC_SC_RTIF                      0x80
#define MCF_RTC_SC_RTCPS                     0x0F
#define MCF_RTC_SC_RTCPS_BITNUM              0x00
#define MCF_RTC_SC_RTCLKS                    0x60
#define MCF_RTC_SC_RTCLKS_BITNUM             0x05


/* RTC Counter Register */
#define MCF_RTC_CNT 						(*(volatile uint8_t *)(0xFFFF82C1))

/* RTC Modulo Register */
#define MCF_RTC_MOD 						(*(volatile uint8_t *)(0xFFFF82C2))


#endif /* MCF51CN_RTC_H_ */
