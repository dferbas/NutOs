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

#ifndef _DEV_BOARD_H_
#error "Do not include this file directly. Use dev/board.h instead!"
#endif

/*
 * UART devices
 */
#include <dev/usartmcf5.h>
#define DEV_UART0       devUartOldMcf5_0
#define DEV_UART1       devUartOldMcf5_1
#define DEV_UART2       devUartOldMcf5_2

#ifndef NUTUART0
#define NUTUART0		DEV_UART0
#endif

#ifndef NUTUART1
#define NUTUART1		DEV_UART1
#endif

#ifndef NUTUART2
#define NUTUART2		DEV_UART2
#endif

/*
 * Debug device.
 */
#ifndef DEV_DEBUG
#define DEV_DEBUG		devDebug2
#endif

/*
 * RTC chip
 */
#include <dev/rtc.h>
extern NUTRTC rtcMcf5225;

#ifndef RTC_CHIP0
#define RTC_CHIP0 rtcMcf5225
#endif

/*
 * Ethernet device
 */
extern NUTDEVICE devMcf5Fec;

#ifndef DEV_ETHER
#define DEV_ETHER   devMcf5Fec
#endif

