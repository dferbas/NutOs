/*
 * Copyright (C) 2001-2004 by egnite Software GmbH
 *
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
 * \file net/tcputil.c
 * \brief TCP utility functions.
 *
 * \verbatim
 * $Id: tcputil.c 3686 2011-12-04 14:20:38Z haraldkipp $
 * \endverbatim
 */

#include <cfg/tcp.h>

#include <netinet/tcputil.h>
#include <sys/timer.h>
#include <stdio.h>

/*!
 * \addtogroup xgTCP
 */
/*@{*/

/* Limit retransmission timeout to 200ms as lower and 20secs as upper boundary */
#ifndef TCP_RTTO_MIN
#define TCP_RTTO_MIN    200
#endif
#ifndef TCP_RTTO_MAX
#define TCP_RTTO_MAX    20000
#endif

#define min(a,b) ((a>b)?b:a)
#define max(a,b) ((a>b)?a:b)

/*
 * Calculate round trip time.
 */
void NutTcpCalcRtt(TCPSOCKET * sock)
{
    uint16_t delta;

    if (sock->so_retran_time == 0)
        return;

    delta = (uint16_t) NutGetMillis() - (sock->so_retran_time & ~1);

    /* According to RFC793 (or STD007), page 41, we use 0.8 for ALPHA and 2.0 for BETA. */
    sock->so_rtto = min (TCP_RTTO_MAX, max(TCP_RTTO_MIN, (delta * 4 + sock->so_rtto * 8) / 10));
    //@@@printf ("[%04X] new retran timeout: %u, delta: %u\n", (u_short) sock, sock->so_rtto, delta);
}

/*@}*/
