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
 * -
 */

/*!
 * \file net/pppsm.c
 * \brief PPP state machine.
 *
 * \verbatim
 * $Id: pppsm.c 3686 2011-12-04 14:20:38Z haraldkipp $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/ppp.h>

#include <string.h>
#include <io.h>
#include <fcntl.h>
#include <dev/uart.h>

#include <sys/thread.h>
#include <sys/heap.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <net/if_var.h>
#include <dev/ppp.h>

//#define NUTDEBUG
#include <netinet/if_ppp.h>
#include <netinet/ppp_fsm.h>

#ifdef NUTDEBUG
#include <net/netdebug.h>
#endif

#ifndef NUT_THREAD_PPPSMSTACK
#define NUT_THREAD_PPPSMSTACK   512
#endif

#define	DCB_RETRIES_INIT	9
#define	DCB_RETRIES_LIMIT	0
#define PPP_SM_PERIOD		2000	//3000 unreliable, 5000 often unable to connect with O2 operator


/*!
 * \addtogroup xgPPP
 */
/*@{*/

uint32_t new_magic = 0x12345678;
static HANDLE pppThread;

/*
 * Message handling (from other threads)
 * (currently only 1 message pending, static variable used instead of a queue)
 */
//#include "d:/eclipse_etech/workspace/sim2/queue/queue.h"

typedef enum {
	pscSynchronize,
	pscHdlcExit,
	pscProcess
} EPppSmEvent;

typedef struct {
//	q		queue;
	EPppSmEvent	event;				// requested event
} TPppSmEvent;

//static TQueue		PppSmEventQueue;
static TPppSmEvent	PppSmEvent;

/*
 * Echo request/reply declarations
 */
static int echo_enable = 0;
static int echo_timeout = 0;


/*! \fn NutPppSm(void *arg)
 * \brief PPP state machine timeout thread.
 *
 * Handles timeouts for LCP and IPCP.
 */
THREAD(NutPppSm, arg)
{
    NUTDEVICE *dev = arg;
    PPPDCB *dcb = dev->dev_dcb;
    uint_fast8_t retries, lcp_echo_counter = 0;
    int echo_test = 0;

//    Queue_Init(&PppSmEventQueue, 5);	// max 5 events pending

    for (;;) {
#if 0
        NutSleep(5000);
#else
        if (!NutEventWait(&dcb->dcb_timer_event, PPP_SM_PERIOD))
        {
//        	TPppSmEvent	*event = Queue_RemoveItem(&PppSmEventQueue);
        	if (PppSmEvent.event == pscSynchronize)
        		continue;		//event occurred, restart timer
        	else if (PppSmEvent.event == pscHdlcExit)
        	{
        		NUTDEVICE *dev_null = NULL;

        		//Signal hdlc thread its termination and wait up to 2 sec for its exit and LCP_LOWERDOWN
        		_ioctl(dcb->dcb_fd, HDLC_SETIFNET, &dev_null);
        	}
//        	else if (PppSmEvent.event == pscProcess)
//        	{}	// proceed immediately
        }
#endif
        new_magic++;

        retries = dcb->dcb_retries;

#ifdef NUTDEBUG
		if (__ppp_trf && dcb->dcb_ipcp_state < PPPS_OPENED) {
			fprintf(__ppp_trs, "ppp_sm(%u)-%u", dcb->dcb_ipcp_state, retries);
		}
#endif

        /*
         * LCP timeouts.
         */
        switch (dcb->dcb_lcp_state) {
        case PPPS_CLOSING:
        case PPPS_STOPPING:
            if (retries > DCB_RETRIES_LIMIT) {
                NutLcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
                dcb->dcb_retries = retries - 1;
            } else
            {
            	dcb->dcb_lcp_state = (dcb->dcb_lcp_state == PPPS_CLOSING) ? PPPS_CLOSED : PPPS_STOPPED;
            	LcpTlf(dev);
            }
            break;

        case PPPS_ACKRCVD:
            dcb->dcb_lcp_state = PPPS_REQSENT;
        case PPPS_REQSENT:
        case PPPS_ACKSENT:
            if (retries > DCB_RETRIES_LIMIT) {
                LcpTxConfReq(dev, dcb->dcb_reqid, 0);
                dcb->dcb_retries = retries - 1;
            } else
            {
                dcb->dcb_lcp_state = PPPS_STOPPED;
            	LcpTlf(dev);
            }
            break;
       }

        /*
         * Authentication timeouts.
         */
        if (dcb->dcb_auth_state == PAPCS_AUTHREQ) {
            if (retries > DCB_RETRIES_LIMIT) {
                PapTxAuthReq(dev, dcb->dcb_reqid);
                dcb->dcb_retries = retries - 1;
            } else
                dcb->dcb_lcp_state = PPPS_STOPPED;
        }

        /*
         * IPCP timeouts.
         */
        switch (dcb->dcb_ipcp_state) {
        case PPPS_CLOSING:
        case PPPS_STOPPING:
            if (retries > DCB_RETRIES_LIMIT) {
                NutIpcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
                dcb->dcb_retries = retries - 1;
            }
            else
            {
                dcb->dcb_ipcp_state = (dcb->dcb_ipcp_state == PPPS_CLOSING) ? PPPS_CLOSED : PPPS_STOPPED;
            	IpcpTlf(dev);
            }
            break;

        case PPPS_ACKRCVD:
            dcb->dcb_ipcp_state = PPPS_REQSENT;
        case PPPS_REQSENT:
        case PPPS_ACKSENT:
            if (retries > DCB_RETRIES_LIMIT)
            {
            	IpcpTxConfReq(dev, dcb->dcb_reqid);
            	dcb->dcb_retries = retries - 1;
            }
            else
            {
                dcb->dcb_ipcp_state = PPPS_STOPPED;
            	IpcpTlf(dev);
            }
            break;

        case PPPS_OPENED:
			if (echo_enable)
			{
				if (echo_test)
				{
					if (dcb->dcb_echo_req_pending)
					{
						LcpClose(dev);		// close ppp connection
						echo_timeout = 1;	// timeout occurred
					}
					else
						echo_timeout = 0;	// echo reply received

					echo_test = 0;
				}
				else if (++lcp_echo_counter >= echo_enable)
				{
					lcp_echo_counter = 0;
					echo_test = LcpTxEchoReq(dev);	// echo_test = 1 if packet was sent
				}
			}
			break;
         }
    }
}

/*!
 * \brief Initialize the PPP state machine.
 *
 * Start the PPP timer thread, if not already running.
 *
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutPppInitStateMachine(NUTDEVICE * dev)
{
    if (pppThread == 0 && (pppThread = NutThreadCreate("pppsm", NutPppSm, dev,
        (NUT_THREAD_PPPSMSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD)) == 0) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Process automaton immediately.
 * Do not wait until PPP_SM_PERIOD elapsed.
 */
void PppSmProcessImmediately(PPPDCB * dcb)
{
	PppSmEvent.event = pscProcess;
//	Queue_AddItem(&PppSmEventQueue, &PppSmEvent);
	NutEventPost/*Async*/(&dcb->dcb_timer_event);
}

/*!
 * \brief Request hdlc exit (works also from other thread).
 */
static inline void PppRequestHdlcExit(PPPDCB * dcb)
{
	PppSmEvent.event = pscHdlcExit;
//	Queue_AddItem(&PppSmEventQueue, &PppSmEvent);
	NutEventPostAsync(&dcb->dcb_timer_event);
}

static void PppRetriesTimerSet(PPPDCB * dcb, uint8_t retries)
{
	dcb->dcb_retries = retries;
	PppSmEvent.event = pscSynchronize;
//	Queue_AddItem(&PppSmEventQueue, &PppSmEvent);
	NutEventPost(&dcb->dcb_timer_event);
}

/*!
 * \brief irc - as per RFC1661
 */
void PppRetriesTimerReset(PPPDCB * dcb)
{
	PppRetriesTimerSet(dcb, DCB_RETRIES_INIT);
}

/*!
 * \brief zrc - as per RFC1661
 */
void PppRetriesTimerStop(PPPDCB * dcb)
{
	PppRetriesTimerSet(dcb, DCB_RETRIES_LIMIT);
}

/*!
 * \brief enable/disable LCP echo sending.
 *
 */
void SetLcpEchoEnable(int period)
{
	echo_enable = period / (PPP_SM_PERIOD / 1000);
}

/*!
 * \brief get LCP echo timeout state.
 *
 */
int GetLcpEchoState(void)
{
#if 0
	int rc = echo_timeout;
	echo_timeout = 0;

	return rc;
#else
	return echo_timeout;
#endif
}

/*!
 * \brief LCP This-Layer-Started
 * Sets LCP parameters for next PPP session.
 *
 */
static void LcpTls(NUTDEVICE *dev)
{
	PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
    	fprintf(__ppp_trs, "\n[lcp-tls](%u)", dcb->dcb_lcp_state);
    }
#endif

	dcb->dcb_reqid = dcb->dcb_rejid = dcb->dcb_rejects = dcb->dcb_auth_state = 0;

	/*
	 * No need to signal HDLC we started.
	 */
}

/*!
 * \brief LCP This-Layer-Finished
 *
 */
void LcpTlf(NUTDEVICE *dev)
{
	PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
    	fprintf(__ppp_trs, "\n[lcp-tlf](%u)", dcb->dcb_lcp_state);
    }
#endif

    /*
     * If we requested LCP close, ioctl to exit HDLC is issued from LcpClose.
     * If we get here because of STOPPING event from remote site, we need to request HDLC exit.
     */
	PppRequestHdlcExit(dcb);
}

/*
 * Common functions
 */

static void NcpUp(NUTDEVICE * dev)
{
	IpcpOpen(dev);
	IpcpLowerUp(dev);
}

static void NcpDown(NUTDEVICE * dev)
{
	IpcpLowerDown(dev);
    IpcpClose(dev);
}

/*!
 * \brief LCP This-Layer-Up.
 *
 */
void LcpTlu(NUTDEVICE * dev)
{
	PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
    	fprintf(__ppp_trs, "\n[lcp-tlu](%u)", dcb->dcb_lcp_state);
    }
#endif

    //use negotiated character map
    _ioctl(dcb->dcb_fd, HDLC_SETTXACCM, &(dcb->dcb_accm) );

    if (dcb->dcb_auth == PPP_PAP)
        PapTxAuthReq(dev, ++dcb->dcb_reqid);
    else
    	NcpUp(dev);
}

/*!
 * \brief LCP This-Layer-Down.
 *
 */
void LcpTld(NUTDEVICE * dev)
{
#ifdef NUTDEBUG
    if (__ppp_trf) {
    	fprintf(__ppp_trs, "\n[lcp-tld](%u)", ((PPPDCB *)dev->dev_dcb)->dcb_lcp_state);
    }
#endif

    NcpDown(dev);
}

/*!
 * \brief PAP This-Layer-Up.
 *
 */
void PapTlu(NUTDEVICE * dev)
{
#ifdef NUTDEBUG
    if (__ppp_trf) {
    	fprintf(__ppp_trs, "\n[pap-tlu](%u)", ((PPPDCB *)dev->dev_dcb)->dcb_auth_state);
    }
#endif

	NcpUp(dev);
}

/*!
 * \brief PAP This-Layer-Down.
 *
 */
void PapTld(NUTDEVICE * dev)
{
#ifdef NUTDEBUG
    if (__ppp_trf) {
    	fprintf(__ppp_trs, "\n[pap-tld](%u)", ((PPPDCB *)dev->dev_dcb)->dcb_auth_state);
    }
#endif

    NcpDown(dev);
}

/*!
 * \brief IPCP This-Layer-Up.
 *
 *	Signal to upper layer (application) we are up.
 */
void IpcpTlu(NUTDEVICE * dev)
{
	PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
    	fprintf(__ppp_trs, "\n[ipcp-tlu](%u)", dcb->dcb_ipcp_state);
    }
#endif

	NutEventPost(&dcb->dcb_state_chg);

	/*
	 * Signal application, it can start using established PPP.
	 */
	if (dcb->dcb_callback)
		(*dcb->dcb_callback)(dcb, PPP_EVENT_IPCP_UP);
}

/*!
 * \brief IPCP This-Layer-Down.
 *
 *	Signal to upper layer (application) we are down.
 */
void IpcpTld(NUTDEVICE * dev)
{
	PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
    	fprintf(__ppp_trs, "\n[ipcp-tld](%u)", dcb->dcb_ipcp_state);
    }
#endif

	/*
	 * Signal application, PPP is no more available.
	 */
	if (dcb->dcb_callback)
		(void)(*dcb->dcb_callback)(dcb, PPP_EVENT_IPCP_DOWN);
}

#ifdef NUTDEBUG
/*!
 * \brief IPCP This-Layer-Finish.
 *
 *	IPCP layer signalizuje dolu LCP, ze ma hotovo a az budou vsechny sitove protokoly finish, tak se LCP muze take zavrit
 */

void IpcpTlf(NUTDEVICE * dev)
{
	PPPDCB *dcb = dev->dev_dcb;

    if (__ppp_trf)
    	fprintf(__ppp_trs, "\n[ipcp-tlf](%u)", dcb->dcb_ipcp_state);
}

/*!
 * \brief IPCP This-Layer-Started.
 *
 *	IPCP layer signalizuje dolu LCP
 */

void IpcpTls(NUTDEVICE * dev)
{
	PPPDCB *dcb = dev->dev_dcb;

    if (__ppp_trf)
    	fprintf(__ppp_trs, "\n[ipcp-tls](%u)", dcb->dcb_ipcp_state);
}
#endif

/*!
 * \brief Trigger LCP open event.
 *
 * Enable the link to come up. Typically triggered by the upper layer,
 * when it is enabled.
 *
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 *
 */
void LcpOpen(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;
#ifdef NUTDEBUG
    if (__ppp_trf) {
//        fputs("\n[LCP-OPEN]", __ppp_trs);
    	fprintf(__ppp_trs, "\n[LCP-OPEN](%u)", dcb->dcb_lcp_state);
    }
#endif

    switch (dcb->dcb_lcp_state) {
    case PPPS_INITIAL:
        /*
         * The LCP layer and the lower layer are down. Enable the LCP
         * layer. Link negotiation will start as soon as the lower
         * layer comes up.
         */
        dcb->dcb_lcp_state = PPPS_STARTING;
        LcpTls(dev);
        break;

    case PPPS_CLOSED:
        /*
         * The LCP layer is down and the lower layer is up. Start
         * link negotiation by sending out a request.
         */
    	PppRetriesTimerReset(dcb);
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        dcb->dcb_lcp_state = PPPS_REQSENT;
        break;

    case PPPS_CLOSING:
        /*
         * The LCP layer is going down.
         */
        dcb->dcb_lcp_state = PPPS_STOPPING;
        break;
    }
}

/*!
 * \brief Trigger LCP close event.
 *
 * Disable the link.
 *
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 */
void LcpClose(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;
    NUTDEVICE *dev_null = NULL;

#ifdef NUTDEBUG
    if (__ppp_trf) {
//        fputs("\n[LCP-CLOSE]", __ppp_trs);
    	fprintf(__ppp_trs, "\n[LCP-CLOSE](%u)", dcb->dcb_lcp_state);
    }
#endif

    switch (dcb->dcb_lcp_state) {
    case PPPS_STARTING:
        /*
         * The LCP layer has been enabled, but the lower layer is still
         * down. Disable the link layer.
         */
        dcb->dcb_lcp_state = PPPS_INITIAL;
    	LcpTlf(dev);
        break;

    case PPPS_STOPPED:
        dcb->dcb_lcp_state = PPPS_CLOSED;
        break;

    case PPPS_STOPPING:
        dcb->dcb_lcp_state = PPPS_CLOSING;
        break;

    case PPPS_REQSENT:
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
    	PppRetriesTimerReset(dcb);
    	NutLcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
    	dcb->dcb_lcp_state = PPPS_CLOSING;
    	break;

    case PPPS_OPENED:
        /*
         * The LCP layer and the lower layer are up. Inform the upper
         * layer that we are going down and send out a termination
         * request.
         */
        dcb->dcb_lcp_state = PPPS_CLOSING;
        LcpTld(dev);
        PppRetriesTimerReset(dcb);
        NutLcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);

        /*
         * Wait until termination action ends.
         */
        NutEventWait(&dcb->dcb_state_chg, 5000); // df proposal: 10s instead of 5s - verify, 19.9.2017 - changed back to 5s (sufficient)

    	/*
    	 * Signal hdlc thread its termination and wait up to 2 sec for its exit.
    	 */
        _ioctl(dcb->dcb_fd, HDLC_SETIFNET, &dev_null);
//        /*
//         * Wait until hdlc thread exits.
//         */
//        NutEventWait(&dcb->dcb_state_chg, 5000);

//        dcb->dcb_lcp_state = PPPS_INITIAL;
        break;
    }
}

/*!
 * \brief Trigger LCP lower up event.
 *
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 *
 */
void LcpLowerUp(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
//        fputs("\n[LCP-LOWERUP]", __ppp_trs);
    	fprintf(__ppp_trs, "\n[LCP-LOWERUP](%u)", dcb->dcb_lcp_state);
    }
#endif

	/*
	 * Signal application, PPP initialization started (hdlc started).
	 */
//	if (dcb->dcb_callback)
//		(void)(*dcb->dcb_callback)(dcb, PPP_EVENT_HDLC_UP);

   switch (dcb->dcb_lcp_state) {
    case PPPS_INITIAL:
        /*
         * The LCP layer is still disabled.
         */
        dcb->dcb_lcp_state = PPPS_CLOSED;
        break;

    case PPPS_STARTING:
        /*
         * The LCP layer is enabled. Send a configuration request.
         */
    	PppRetriesTimerReset(dcb);
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        dcb->dcb_lcp_state = PPPS_REQSENT;
        break;
    }
}

/*!
 * \brief Trigger LCP lower down event.
 *
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 *
 */
void LcpLowerDown(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
//        fputs("\n[LCP-LOWERDOWN]", __ppp_trs);
    	fprintf(__ppp_trs, "\n[LCP-LOWERDOWN](%u)", dcb->dcb_lcp_state);
    }
#endif

	/*
	 * Signal application, PPP is no more available (hdlc exited).
	 */
	if (dcb->dcb_callback)
		(void)(*dcb->dcb_callback)(dcb, PPP_EVENT_HDLC_DOWN);

    switch (dcb->dcb_lcp_state) {
    case PPPS_CLOSED:
		/*
		 * Here we comes (hdlc thread context) when TERM ACK was received to a TERM REQ, issued from LcpClose (application context).
		 * This happens when we actively close PPP via ioctl(LCP_CLOSE) from application.
		 */

        /*
         * Wake up the LCP_CLOSE ioctl (application thread) and return back to hdlc thread.
         */
    	dcb->dcb_lcp_state = PPPS_INITIAL;
        NutEventPostAsync(&dcb->dcb_state_chg);
        break;

    case PPPS_STOPPED:
    	dcb->dcb_lcp_state = PPPS_STARTING;
    	LcpTls(dev);
        break;

    case PPPS_CLOSING:
        dcb->dcb_lcp_state = PPPS_INITIAL;
        break;

    case PPPS_STOPPING:
		/*
		 * We will arrive here when hdlc thread is exiting (hdlc thread context).
		 * This happens when NO CARRIER was detected and hdlc is now terminating itself.
		 */

//        /*
//         * Wake up the LCP_CLOSE ioctl (application thread) and proceed with hdlc thread exit.
//         * (see above)
//         */
//        NutEventPostAsync(&dcb->dcb_state_chg); // flk: the event is expected; it has to stay uncommented df proposal: comment on

    case PPPS_REQSENT:
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
        dcb->dcb_lcp_state = PPPS_STARTING;
        break;

    case PPPS_OPENED:
        dcb->dcb_lcp_state = PPPS_STARTING;
    	LcpTld(dev);
        break;
    }
}

/*!
 * \brief Trigger IPCP open event.
 *
 * Link is allowed to come up.
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 *
 */
void IpcpOpen(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
//        fputs("\n[IPCP-OPEN]", __ppp_trs);
    	fprintf(__ppp_trs, "\n[IPCP-OPEN](%u)", dcb->dcb_ipcp_state);
    }
#endif

    switch (dcb->dcb_ipcp_state) {
    case PPPS_INITIAL:
        /*
         * The IPCP layer and the lower layer are down. Enable the
         * IPCP layer and the lower layer.
         */
        dcb->dcb_ipcp_state = PPPS_STARTING;
        IpcpTls(dev);
//        LcpOpen(dev);
        break;

    case PPPS_CLOSED:
    	PppRetriesTimerReset(dcb);
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;

    case PPPS_CLOSING:
        dcb->dcb_ipcp_state = PPPS_STOPPING;
        break;
    }
}

/*!
 * \brief Trigger IPCP close event.
 *
 * Disable the link.
 *
 * Cancel timeouts and either initiate close or possibly go directly to
 * the PPPS_CLOSED state.
 *
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 */
void IpcpClose(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
//        fputs("\n[IPCP-CLOSE]", __ppp_trs);
    	fprintf(__ppp_trs, "\n[IPCP-CLOSE](%u)", dcb->dcb_ipcp_state);
    }
#endif

    switch (dcb->dcb_ipcp_state) {
    case PPPS_STARTING:
        /*
         * The IPCP layer has been enabled, but the lower layer is still
         * down. Disable the network layer.
         */
        dcb->dcb_ipcp_state = PPPS_INITIAL;
    	IpcpTlf(dev);
        break;

    case PPPS_STOPPED:
        dcb->dcb_ipcp_state = PPPS_CLOSED;
        break;

    case PPPS_STOPPING:
        dcb->dcb_ipcp_state = PPPS_CLOSING;
        break;

    case PPPS_REQSENT:
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
    	PppRetriesTimerReset(dcb);
        NutIpcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
        dcb->dcb_ipcp_state = PPPS_CLOSING;
        break;
    case PPPS_OPENED:
        /*
         * The IPCP layer and the lower layer are up. Inform the upper
         * layer that we are going down and send out a termination
         * request.
         */
        dcb->dcb_ipcp_state = PPPS_CLOSING;
        IpcpTld(dev);
    	PppRetriesTimerReset(dcb);
        NutIpcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
//        NutEventPost(&dcb->dcb_state_chg);
        break;
    }
}

/*
 * The lower layer is up.
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 *
 */
void IpcpLowerUp(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;

#ifdef NUTDEBUG
    if (__ppp_trf) {
//        fputs("\n[IPCP-LOWERUP]", __ppp_trs);
        fprintf(__ppp_trs, "\n[IPCP-LOWERUP](%u)", dcb->dcb_ipcp_state);
    }
#endif

    switch (dcb->dcb_ipcp_state) {
    case PPPS_INITIAL:
        dcb->dcb_ipcp_state = PPPS_CLOSED;
        break;

    case PPPS_STARTING:
    	PppRetriesTimerReset(dcb);
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;
    }
}

/*
 * The link layer is down.
 *
 * Cancel all timeouts and inform upper layers.
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 *
 */
void IpcpLowerDown(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;
//    NUTDEVICE *dev_null = NULL;

#ifdef NUTDEBUG
    if (__ppp_trf) {
//        fputs("\n[IPCP-LOWERDOWN]", __ppp_trs);
        fprintf(__ppp_trs, "\n[IPCP-LOWERDOWN](%u)", dcb->dcb_ipcp_state);
    }
#endif

    switch (dcb->dcb_ipcp_state) {
    case PPPS_CLOSED:
        dcb->dcb_ipcp_state = PPPS_INITIAL;
//        _ioctl(dcb->dcb_fd, HDLC_SETIFNET, &dev_null);
        break;

    case PPPS_STOPPED:
        dcb->dcb_ipcp_state = PPPS_STARTING;
        IpcpTls(dev);
        break;

    case PPPS_CLOSING:
        dcb->dcb_ipcp_state = PPPS_INITIAL;
        break;

    case PPPS_STOPPING:
    case PPPS_REQSENT:
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
        dcb->dcb_ipcp_state = PPPS_STARTING;
        break;

    case PPPS_OPENED:
        dcb->dcb_ipcp_state = PPPS_STARTING;	//to be properly reopened (IPCP has no thread for reading, which should be terminated as HDLC)
        IpcpTld(dev);
//        NutEventPost(&dcb->dcb_state_chg);
        break;
    }
}

/*@}*/
