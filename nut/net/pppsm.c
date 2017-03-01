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

#include <netinet/if_ppp.h>
#include <netinet/ppp_fsm.h>

#ifdef NUTDEBUG
#include <net/netdebug.h>
#endif

#ifndef NUT_THREAD_PPPSMSTACK
#define NUT_THREAD_PPPSMSTACK   512
#endif

/*
 * Echo request/reply declarations
 */
static int echo_enable = 0;
static int echo_timeout = 0;

/*!
 * \brief set enable/disable LCP echo sending.
 *
 */
void SetLcpEchoEnable(int state)
{
	echo_enable = state;
}

/*!
 * \brief get LCP echo timeout state.
 *
 */
int GetLcpEchoState(void)
{
	int rc = echo_timeout;
	echo_timeout = 0;

	return rc;
}


/*!
 * \brief Trigger LCP echo request event.
 *
 * Enable the link to come up. Typically triggered by the upper layer,
 * when it is enabled.
 *
 * \param dev Pointer to the NUTDEVICE structure of the PPP device.
 *
 */
void LcpEchoRq(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;
    NETBUF *nb;
    int nb_len;
    uint8_t id;

#ifdef NUTDEBUG
    if (__ppp_trf) {
        fputs("\n[LCP-ERQ]", __ppp_trs);
    }
#endif

    if (dcb->dcb_lcp_state == PPPS_OPENED)
    {
    	/* id,nb init */
    	id = 255;
    	nb_len = sizeof(uint32_t);
    	nb = NutNetBufAlloc(NULL, NBAF_APPLICATION, nb_len);
    	/* Use local magic number. */
        memcpy(nb->nb_ap.vp, &dcb->dcb_neg_magic, nb_len);
        NutLcpOutput(dev, LCP_ERQ, id, nb);

        NutNetBufFree(nb);
    }
}

/*!
 * \addtogroup xgPPP
 */
/*@{*/

uint32_t new_magic = 0x12345678;
static HANDLE pppThread;


/*! \fn NutPppSm(void *arg)
 * \brief PPP state machine timeout thread.
 *
 * Handles timeouts for LCP and IPCP.
 */
THREAD(NutPppSm, arg)
{
    NUTDEVICE *dev = arg;
    PPPDCB *dcb = dev->dev_dcb;
    uint_fast8_t retries;

    for (;;) {
        NutSleep(5000);
        new_magic++;

        retries = dcb->dcb_retries;

        /*
         * LCP timeouts.
         */
        switch (dcb->dcb_lcp_state) {
        case PPPS_CLOSING:
        case PPPS_STOPPING:
            if (retries < 9) {
                if (retries) {
                    NutLcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
                }
                dcb->dcb_retries = retries + 1;
            } else
                dcb->dcb_lcp_state = (dcb->dcb_lcp_state == PPPS_CLOSING) ? PPPS_CLOSED : PPPS_STOPPED;
            break;

        case PPPS_REQSENT:
        case PPPS_ACKSENT:
            if (retries < 9) {
                if (retries)
                    LcpTxConfReq(dev, dcb->dcb_reqid, 0);
                dcb->dcb_retries = retries + 1;
            } else
                dcb->dcb_lcp_state = PPPS_STOPPED;
            break;
       }

        /*
         * Authentication timeouts.
         */
        if (dcb->dcb_auth_state == PAPCS_AUTHREQ) {
            if (retries < 9) {
                if (retries)
                    PapTxAuthReq(dev, dcb->dcb_reqid);
                dcb->dcb_retries = retries + 1;
            } else
                dcb->dcb_lcp_state = PPPS_STOPPED;
        }

        /*
         * IPCP timeouts.
         */
        switch (dcb->dcb_ipcp_state) {
        case PPPS_CLOSING:
        case PPPS_STOPPING:
            if (retries < 9) {
                if (retries)
                    NutIpcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
                dcb->dcb_retries = retries + 1;
            } else
                dcb->dcb_ipcp_state = (dcb->dcb_ipcp_state == PPPS_CLOSING) ? PPPS_CLOSED : PPPS_STOPPED;
            break;

        case PPPS_REQSENT:
        case PPPS_ACKSENT:
            if (retries < 9) {
                if (retries)
                    IpcpTxConfReq(dev, dcb->dcb_reqid);
                dcb->dcb_retries = retries + 1;
            } else
                dcb->dcb_ipcp_state = PPPS_STOPPED;
            break;

        case PPPS_OPENED:
        	if (echo_enable && ++retries > echo_enable)
        	{
        		LcpEchoRq(dev);
        		dcb->dcb_retries = 0;

        		if (NutEventWaitNext(&dcb->dcb_echo_reply, 20))
        			echo_timeout = 1; // timeout occurred
        		else
        			echo_timeout = 0; // echo reply received
        	}
        	else
                dcb->dcb_retries = retries + 1;
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
        fputs("\n[LCP-OPEN]", __ppp_trs);
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

    case PPPS_STARTING:
        dcb->dcb_reqid = dcb->dcb_rejid = dcb->dcb_rejects = dcb->dcb_auth_state = 0;
        break;

    case PPPS_CLOSED:
        /*
         * The LCP layer is down and the lower layer is up. Start
         * link negotiation by sending out a request.
         */
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
        fputs("\n[LCP-CLOSE]", __ppp_trs);
    }
#endif

    switch (dcb->dcb_lcp_state) {
    case PPPS_STARTING:
        /*
         * The LCP layer has been enabled, but the lower layer is still
         * down. Disable the link layer.
         */
        dcb->dcb_lcp_state = PPPS_INITIAL;
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
    case PPPS_OPENED:
        /*
         * The LCP layer and the lower layer are up. Inform the upper
         * layer that we are going down and send out a termination
         * request.
         */
        dcb->dcb_lcp_state = PPPS_CLOSING;
        IpcpLowerDown(dev);
        NutLcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);

        /*
         * Wait until termination action ends.
         */
        NutEventWait(&dcb->dcb_state_chg, 5000); // df proposal: 10s instead of 5s - verify

    	/*
    	 * Signal hdlc thread its termination and wait up to 2 sec for its exit.
    	 */
        _ioctl(dcb->dcb_fd, HDLC_SETIFNET, &dev_null);
        /*
         * Wait until hdlc thread exits.
         */
//***   NutEventWait(&dcb->dcb_state_chg, 5000);

        dcb->dcb_lcp_state = PPPS_INITIAL;
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
        fputs("\n[LCP-LOWERUP]", __ppp_trs);
    }
#endif

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
        fputs("\n[LCP-LOWERDOWN]", __ppp_trs);
    }
#endif

    switch (dcb->dcb_lcp_state) {
    case PPPS_CLOSED:
		/*
		 * Here we comes (hdlc thread context) when TERM ACK was received to a TERM REQ, issued from LcpClose (application context).
		 */
//df    	dcb->dcb_lcp_state = PPPS_STOPPING;

        /*
         * Wake up the LCP_CLOSE ioctl (application thread) and return back to hdlc thread.
         */
        NutEventPostAsync(&dcb->dcb_state_chg);
        break;

    case PPPS_STOPPED:
        dcb->dcb_lcp_state = PPPS_STARTING;
        break;

    case PPPS_CLOSING:
        dcb->dcb_lcp_state = PPPS_INITIAL;
        break;

    case PPPS_STOPPING:
		/*
		 * We will arrive here when hdlc thread is exiting (hdlc thread context).
		 * This happens when we actively close PPP via ioctl(LCP_CLOSE) from application
		 *  or when NO CARRIER was detected and hdlc is now terminating itself.
		 */
//df        dcb->dcb_lcp_state = PPPS_STOPPED; // df proposal: comment on

        /*
         * Wake up the LCP_CLOSE ioctl (application thread) and proceed with hdlc thread exit.
         * (see above)
         */
        NutEventPostAsync(&dcb->dcb_state_chg); // flk: the event is expected; it has to stay uncommented df proposal: comment on
//df        break; // df proposal: comment on

    case PPPS_REQSENT:
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
        dcb->dcb_lcp_state = PPPS_STARTING;
        break;

    case PPPS_OPENED:
        IpcpLowerDown(dev);
        dcb->dcb_lcp_state = PPPS_STARTING;
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
        fputs("\n[IPCP-OPEN]", __ppp_trs);
    }
#endif

    switch (dcb->dcb_ipcp_state) {
    case PPPS_INITIAL:
        /*
         * The IPCP layer and the lower layer are down. Enable the
         * IPCP layer and the lower layer.
         */
        dcb->dcb_ipcp_state = PPPS_STARTING;
        LcpOpen(dev);
        break;

    case PPPS_CLOSED:
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
        fputs("\n[IPCP-CLOSE]", __ppp_trs);
    }
#endif

    switch (dcb->dcb_ipcp_state) {
    case PPPS_STARTING:
        /*
         * The IPCP layer has been enabled, but the lower layer is still
         * down. Disable the network layer.
         */
        dcb->dcb_ipcp_state = PPPS_INITIAL;
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
    case PPPS_OPENED:
        /*
         * The IPCP layer and the lower layer are up. Inform the upper
         * layer that we are going down and send out a termination
         * request.
         */
        NutIpcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
        dcb->dcb_ipcp_state = PPPS_CLOSING;
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
        fputs("\n[IPCP-LOWERUP]", __ppp_trs);
    }
#endif

    switch (dcb->dcb_ipcp_state) {
    case PPPS_INITIAL:
        dcb->dcb_ipcp_state = PPPS_CLOSED;
        break;

    case PPPS_STARTING:
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
        fputs("\n[IPCP-LOWERDOWN]", __ppp_trs);
    }
#endif

    switch (dcb->dcb_ipcp_state) {
    case PPPS_CLOSED:
        dcb->dcb_ipcp_state = PPPS_INITIAL;
//        _ioctl(dcb->dcb_fd, HDLC_SETIFNET, &dev_null);
        break;

    case PPPS_STOPPED:
        dcb->dcb_ipcp_state = PPPS_STARTING;
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
//        NutEventPost(&dcb->dcb_state_chg);
        break;
    }
}

/*@}*/
