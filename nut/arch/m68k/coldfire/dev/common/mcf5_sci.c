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

#include <string.h>
#include <arch/m68k.h>
#include <dev/gpio.h>
#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

/* \brief ASCII code for software flow control, starts transmitter. */
#define ASCII_XON   0x11
/* \brief ASCII code for software flow control, stops transmitter. */
#define ASCII_XOFF  0x13

/* \brief XON transmit pending flag. */
#define XON_PENDING     0x10
/* \brief XOFF transmit pending flag. */
#define XOFF_PENDING    0x20
/* \brief XOFF sent flag. */
#define XOFF_SENT       0x40
/* \brief XOFF received flag. */
#define XOFF_RCVD       0x80

/*!
 * \brief Enables software flow control if not equal zero.
 */
static ureg_t flow_control;

/*!
 * \brief Receiver error flags.
 */
static uint8_t rx_errors;
static uint8_t tx_errors;


/*
 * \brief Scin transmit data register empty interrupt handler.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Mcf5SciTxReady(void *arg)
{
    register RINGBUF *rbf = &((USARTDCB *)arg)->dcb_tx_rbf;;
    register uint8_t *cp;
    uint_fast8_t usr;

#ifndef UART_NO_SW_FLOWCONTROL

    /*
     * Process pending software flow controls first.
     */
    if (flow_control & (XON_PENDING | XOFF_PENDING)) {
        if (flow_control & XOFF_PENDING) {
        	MCF_SCI_D(BASE) = ASCII_XOFF;
            flow_control |= XOFF_SENT;
        } else {
        	MCF_SCI_D(BASE) = ASCII_XON;
            flow_control &= ~XOFF_SENT;
        }
        flow_control &= ~(XON_PENDING | XOFF_PENDING);
        return;
    }

    if (flow_control & XOFF_RCVD) {
        /*
         * If XOFF has been received, we disable the transmit interrupts
         * and return without sending anything.
         */
    	MCF_SCI_C2(BASE) &= ~(MCF_SCI_C2_TIE);
        return;
    }
#endif /* UART_NO_SW_FLOWCONTROL */

    /*
     * Prepare next character for transmitting
     */
    if (rbf->rbf_cnt) {

        /*
         * Previous character transmitted successfully
         */
        rbf->rbf_cnt--;

        /*
         * Start transmission of the next character and clear TXRDY bit
         * in USR register.
         */
        cp = rbf->rbf_tail;

        /*
		 * To clear TDRE, read SCIxS1 with TDRE set and then read the SCI data register (SCIxD).
		 */
		usr = MCF_SCI_S1(BASE);

		/* Record SCI Errors  */
		tx_errors |= usr;

        MCF_SCI_D(BASE) = *cp;

        /*
         * Wrap around the buffer pointer if we reached its end.
         */
        if (++cp == rbf->rbf_last) {
            cp = rbf->rbf_start;
        }
        rbf->rbf_tail = cp;

        /*
         * Wakeup waiting thread when tx buffer low watermark is reached
         */
        if (rbf->rbf_cnt == rbf->rbf_lwm) {
            NutEventPostFromIrq(&rbf->rbf_que);
        }
    }

    if (rbf->rbf_cnt == 0){

//    	if (MCF_SCI_S1(BASE) & MCF_SCI_S1_TC)
		{
			// If transmit complete, disable transmitter and interrupts
			MCF_SCI_C2(BASE) &= ~(MCF_SCI_C2_TIE); //MCF_SCI_C2_TCIE |

			/*
			 * Nothing left to transmit, wakeup waiting thread
			 */
			NutEventPostFromIrq(&rbf->rbf_que);
    	}

    }
}

/*
 * \brief Scin receive complete interrupt handler.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Mcf5SciRxComplete(void *arg) {
    register RINGBUF *rbf = &((USARTDCB *)arg)->dcb_rx_rbf;
    register size_t cnt;
    register uint8_t ch;
    uint_fast8_t usr;
    uint_fast8_t postEvent = 0;

    /*
     * Receive all bytes from RxFIFO
     */
    do {
        /*
         * To clear RDRF, read SCIxS1 with RDRF set and then read the SCI data register (SCIxD)
         */
        usr = MCF_SCI_S1(BASE);

       /* Record SCI Errors  */
        rx_errors |= usr;

       /* Receive char from Rx FIFO */
        ch = MCF_SCI_D(BASE);

#ifndef UART_NO_SW_FLOWCONTROL
        /*
         * Handle software handshake. We have to do this before checking the
         * buffer, because flow control must work in write-only mode, where
         * there is no receive buffer.
         */
        if (flow_control) {
            /* XOFF character disables transmit interrupts. */
            if (ch == ASCII_XOFF) {
            	NutIrqDisable(&sig_sci_tx);
                flow_control |= XOFF_RCVD;
                return;
            }
            /* XON enables transmit interrupts. */
            else if (ch == ASCII_XON) {
            	NutIrqEnable(&sig_sci_tx);
                flow_control &= ~XOFF_RCVD;
                return;
            }
        }
#endif

        /*
         * Check buffer overflow.
         */
        cnt = rbf->rbf_cnt;
        if (cnt >= rbf->rbf_siz) {
            rx_errors |= MCF_SCI_S1_OR;
            return;
        }

        /*
         * Wake up waiting threads if this is the first byte in the buffer.
         */
        if (cnt++ == 0){
            postEvent = 1;
        }

#ifndef UART_NO_SW_FLOWCONTROL

        /*
         * Check the high watermark for software handshake. If the number of
         * buffered bytes is above this mark, then send XOFF.
         */
        else if (flow_control) {
            if(cnt >= rbf->rbf_hwm) {
                if((flow_control & XOFF_SENT) == 0) {
                    if (MCF_SCI_C2(BASE) & MCF_SCI_C2_TIE) {
                    	MCF_SCI_D(BASE) = ASCII_XOFF;
                        flow_control |= XOFF_SENT;
                        flow_control &= ~XOFF_PENDING;
                    } else {
                        flow_control |= XOFF_PENDING;
                    }
                }
            }
        }
#endif

        /*
         * Store the character and increment and the ring buffer pointer.
         */
        *rbf->rbf_head++ = ch;
        if (rbf->rbf_head == rbf->rbf_last) {
            rbf->rbf_head = rbf->rbf_start;
        }

        /*
         * Update the ring buffer counter.
         */
        rbf->rbf_cnt = cnt;

    } while ( MCF_SCI_S1(BASE) & MCF_SCI_S1_RDRF );

    /*
     *  Wakeup waiting threads
     */
    if (postEvent)
        NutEventPostFromIrq(&rbf->rbf_que);
}

/*!
 * \brief Carefully enable Sci hardware functions.
 *
 * Always enable transmitter and receiver, even on read-only or
 * write-only mode. So we can support software flow control.
 */
static void Mcf5SciEnable(void)
{
	/*
	 * Enable Sci receive.
	 */
    MCF_SCI_C2(BASE) = MCF_SCI_C2_RE | MCF_SCI_C2_TE;
}

/*!
 * \brief Carefully disable Sci hardware functions.
 */
/*static void Mcf5SciDisable(void)
{
	NutIrqDisable(&sig_sci_rx);
	NutIrqDisable(&sig_sci_tx);


     * Allow incoming or outgoing character to finish.

    NutDelay(10);


     * Disable Sci transmit and receive.

    MCF_SCI_C2(BASE) &= ~MCF_SCI_C2_RE;
}*/

/*!
 * \brief Query the Sci hardware for the selected speed.
 *
 * This function is called by ioctl function of the upper level Sci
 * driver through the USARTDCB jump table.
 *
 * \return The currently selected baudrate.
 */
static uint32_t Mcf5SciGetSpeed(void)
{
    return -1;
}

/*!
 * \brief Set the Sci hardware bit rate.
 *
 * This function is called by ioctl function of the upper level Sci
 * driver through the USARTDCB jump table.
 *
 * \param rate Number of bits per second.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5SciSetSpeed(uint32_t rate)
{
	register uint16_t ubgs;
	uint32_t sysclk = NutGetCpuClock();

	/* Calculate baud settings */
	ubgs = (uint16_t)(sysclk / (rate * 16));

	MCF_SCI_BD(BASE) = ubgs;

    return 0;
}

/*!
 * \brief Query the Sci hardware for the number of data bits.
 *
 * This function is called by ioctl function of the upper level Sci
 * driver through the USARTDCB jump table.
 *
 * \return The number of data bits set.
 */
static uint8_t Mcf5SciGetDataBits(void)
{
    return -1;
}

/*!
 * \brief Set the Sci hardware to the number of data bits.
 *
 * This function is called by ioctl function of the upper level Sci
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5SciSetDataBits(uint8_t bits)
{
    return -1;
}

/*!
 * \brief Query the Sci hardware for the parity mode.
 *
 * This routine is called by ioctl function of the upper level Sci
 * driver through the USARTDCB jump table.
 *
 * \return Parity mode, either 0 (disabled), 1 (odd) or 2 (even).
 */
static uint8_t Mcf5SciGetParity(void)
{
    return -1;
}

/*!
 * \brief Set the Sci hardware to the specified parity mode.
 *
 * This routine is called by ioctl function of the upper level Sci
 * driver through the USARTDCB jump table.
 *
 * \param mode 0 (none), 1 (odd) or 2 (even).
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5SciSetParity(uint8_t mode)
{
    return -1;
}

/*!
 * \brief Query the Sci hardware for the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level Sci
 * driver through the USARTDCB jump table.
 *
 * \return The number of stop bits set, either 1 or 2.
 */
static uint8_t Mcf5SciGetStopBits(void)
{
    return -1;
}

/*!
 * \brief Set the Sci hardware to the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level Sci
 * driver through the USARTDCB jump table.
 *
 * \param stop bits 1, 2 or 15 (1,5).
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5SciSetStopBits(uint8_t bits)
{
    return -1;
}

/*!
 * \brief Query the Sci hardware status.
 *
 * \return Status flags.
 */
static uint32_t Mcf5SciGetStatus(void)
{
	uint32_t rc = 0;

	/*
	 * Determine software handshake status. The flow control status may
	 * change during interrupt, but this doesn't really hurt us.
	 */
	if (flow_control) {
		if (flow_control & XOFF_SENT) {
			rc |= UART_RXDISABLED;
		}
		if (flow_control & XOFF_RCVD) {
			rc |= UART_TXDISABLED;
		}
	}

	return rc;
}

/*!
 * \brief Set the Sci hardware status.
 *
 * \param flags Status flags.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5SciSetStatus(uint32_t flags)
{
    /*
     * Process software handshake control.
     */
    if (flow_control) {

        /* Access to the flow control status must be atomic. */
        NutEnterCritical();

        /*
         * Enabling or disabling the receiver means to behave like
         * having sent a XON or XOFF character resp.
         */
        if (flags & UART_RXENABLED) {
            flow_control &= ~XOFF_SENT;
        } else if (flags & UART_RXDISABLED) {
            flow_control |= XOFF_SENT;
        }

        /*
         * Enabling or disabling the transmitter means to behave like
         * having received a XON or XOFF character resp.
         */
        if (flags & UART_TXENABLED) {
            flow_control &= ~XOFF_RCVD;
        } else if (flags & UART_TXDISABLED) {
            flow_control |= XOFF_RCVD;
        }
        NutExitCritical();
    }

    /*
     * Verify the result.
     */
    if (Mcf5SciGetStatus() != flags) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Query flow control mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return See UsartIOCtl().
 */
static uint32_t Mcf5SciGetFlowControl(void)
{
    uint32_t rc = 0;

    if (flow_control) {
        rc |= USART_MF_XONXOFF;
    } else {
        rc &= ~USART_MF_XONXOFF;
    }

    return rc;
}

/*!
 * \brief Set flow control mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 * Nove pridan parametr USART_MF_HALFDUPLEX_YZ, ktery se vklada spolecne
 * s parametrem USART_MF_HALFDUPLEX. Pokud USART_MF_HALFDUPLEX_YZ je
 * zadan komonikuje se pres porty XY, pokud neni tak pres AB.
 *
 * \param flags See UsartIOCtl().
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5SciSetFlowControl(uint32_t flags)
{
    /*
     * Set software handshake mode.
     */
    if (flags & USART_MF_XONXOFF) {
        if(flow_control == 0) {
            NutEnterCritical();
            flow_control = 1 | XOFF_SENT;  /* force XON to be sent on next read */
            NutExitCritical();
        }
    } else {

        NutEnterCritical();
        flow_control = 0;
        NutExitCritical();
    }

    /*
     * Verify the result.
     */
    if (Mcf5SciGetFlowControl() != flags) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Start the Sci transmitter hardware.
 *
 * The upper level Sci driver will call this function through the
 * USARTDCB jump table each time it added one or more bytes to the
 * transmit buffer.
 */
static void Mcf5SciTxStart(void)
{
    /* Enable Transmit to activate interrupt TxReady */
	NutIrqEnable(&sig_sci_tx);
}

/*!
 * \brief Start the Sci receiver hardware.
 *
 * The upper level Sci driver will call this function through the
 * USARTDCB jump table each time it removed enough bytes from the
 * receive buffer. Enough means, that the number of bytes left in
 * the buffer is below the low watermark.
 */
static void Mcf5SciRxStart(void)
{
    /*
     * Enable receive interrupt. It could be disabled by flow control.
     */
    NutIrqEnable(&sig_sci_rx);

	/*
	 * Do any required software flow control.
	 */
	if (flow_control && (flow_control & XOFF_SENT) != 0) {
		NutEnterCritical();
		if (MCF_SCI_C2(BASE) & MCF_SCI_C2_TIE) {
			MCF_SCI_D(BASE) = ASCII_XON;
			flow_control &= ~XON_PENDING;
		} else {
			flow_control |= XON_PENDING;
		}
		flow_control &= ~(XOFF_SENT | XOFF_PENDING);
		NutExitCritical();
	}
}

/*
 * \brief Initialize the Sci hardware driver.
 *
 * This function is called during device registration by the upper level
 * Sci driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5SciInit(void)
{

    int result = 0;

	MCF_SCI_C1(BASE) = 0x0;              /* Configure the SCI */
	MCF_SCI_C3(BASE) = 0x0;              /* Disable error interrupts */
	MCF_SCI_C2(BASE) = 0x0;             /* Disable all interrupts */
	MCF_SCI_S2(BASE) = 0x0;

    /*
     *  GPIO Configuration
     */
    result |= GpioPinConfigSet(TXD_PORT, TXD_PIN, TXD_PERIPHERAL | GPIO_CFG_PULLUP);
    result |= GpioPinConfigSet(RXD_PORT, RXD_PIN, RXD_PERIPHERAL | GPIO_CFG_PULLUP);

    /*
     * SCI Configuration
     */

//    result |= Mcf5SciSetDataBits(8);
//    result |= Mcf5SciSetStopBits(1);
//    result |= Mcf5SciSetParity(0);
    result |= Mcf5SciSetSpeed(USART_INITSPEED);


    /*
     * Register and enable Interrupt handler
     */
    if (result
     || NutRegisterIrqHandler(&sig_sci_rx, Mcf5SciRxComplete, &dcb_sci)
     || NutRegisterIrqHandler(&sig_sci_tx, Mcf5SciTxReady, &dcb_sci))
        return -1;

    /*
     * Start receiving immediately
     * NOTE: Transmitting is started too, but it is stopped after while in Mcf5SciTxReady()
     */
    Mcf5SciEnable();

    return 0;
}

/*
 * \brief Deinitialize the Sci hardware driver.
 *
 * This function is called during device deregistration by the upper
 * level Sci driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5SciDeinit(void)
{
    /* Deregister receive and transmit interrupts. */
    NutIrqDisable(&sig_sci_rx);
    NutIrqDisable(&sig_sci_tx);

    /* Deregister receive and transmit interrupts. */
    NutRegisterIrqHandler(&sig_sci_rx, 0, 0);
    NutRegisterIrqHandler(&sig_sci_tx, 0, 0);

    return 0;
}

/*@}*/
