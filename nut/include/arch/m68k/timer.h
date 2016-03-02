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

#ifndef _SYS_TIMER_H
#error "Do not include this file directly. Use sys/timer.h instead!"
#endif

#if defined(MCU_COLDFIRE)
#if defined(MCU_MCF5225)
#define NutEnableTimerIrq()     NutIrqEnable(&sig_PIT0)
#define NutDisableTimerIrq()    NutIrqDisable(&sig_PIT0)
#elif defined(MCU_MCF51CN)
#define NutEnableTimerIrq()     NutIrqEnable(&sig_MTIM1)
#define NutDisableTimerIrq()    NutIrqDisable(&sig_MTIM1)
#elif defined(MCU_MCF51QE)
#define NutEnableTimerIrq()     NutIrqEnable(&sig_TPM1_CH0)
#define NutDisableTimerIrq()    NutIrqDisable(&sig_TPM1_CH0)
#else
#warning "Unknown Coldfire MCU Family defined"
#endif
#else
#warning "Unknown M68K MCU Family defined"
#endif
