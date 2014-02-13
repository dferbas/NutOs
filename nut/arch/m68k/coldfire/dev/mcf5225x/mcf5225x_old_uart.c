
//#include <arch/m68k.h>
//#include <cfg/arch.h>
//#include <dev/irqreg.h>
#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#define NutEnterCriticalLevel(...)   NutEnterCritical()        // TODO JS .. upravit podle SolarMonitoru

#undef NUTTRACER    // TODO .. not portet yet
//#define UART_NO_SW_FLOWCONTROL

/* IOCtrl Halfduplex flag, ktery urci komunkaci pres porty AB, XY */
#define USART_MF_HALFDUPLEX_YZ	0x1000

/* Priznak, ktery urci jaky z portu AB, XY je vybran. */
#define HDX_CONTROL_YZ			0x10

#ifdef NUTTRACER
#include <sys/tracer.h>
#endif

/* Enable Transmit Ready Interrupt. */
#define SET_TXRDY_INTERRUPT() \
    {   \
        usartControlRegister.uimr |= MCF_UART_UIMR_TXRDY;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UARTn_UIMR = usartControlRegister.uimr;)\
    }

/* Disable Transmit Ready Interrupt. */
#define CLR_TXRDY_INTERRUPT() \
    {   \
        usartControlRegister.uimr &= ~MCF_UART_UIMR_TXRDY;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UARTn_UIMR = usartControlRegister.uimr;)\
    }


/* Enable Receive Ready Interrupt. */
#define SET_RXRDY_INTERRUPT() \
    {   \
        usartControlRegister.uimr |= MCF_UART_UIMR_FFULL_RXRDY;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UARTn_UIMR = usartControlRegister.uimr;)\
    }

/* Disable All Interrupt. */
#define CLR_ALL_INTERRUPT() \
    {   \
        usartControlRegister.uimr = 0;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UARTn_UIMR = 0;)\
    }

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
 * \brief Receiver error flags.
 */
static ureg_t rx_errors;

/*!
 * \brief Receiver error flags.
 */
#define UART_RECEIVEBREAK    0x08

/*!
 * \brief Enables software flow control if not equal zero.
 */
static ureg_t flow_control;

#if defined(UART_HDX_BIT) || defined(UART_HDB_FDX_BIT)
    /* define in cfg/modem.h */
    #ifdef UART_HDX_FLIP_BIT    /* same as RTS toggle by Windows NT driver */
        #define UART_HDX_TX     cbi
        #define UART_HDX_RX     sbi
    #else                       /* previous usage by Ethernut */
        #define UART_HDX_TX     sbi
        #define UART_HDX_RX     cbi
    #endif
#endif


#if defined(UART_HDX_BIT) || defined(UART_HDB_FDX_BIT)
/*!
 * \brief Enables half duplex control if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to switch between receive and transmit mode.
 */
static ureg_t hdx_control;
#endif

#ifdef UART_RTS_BIT
/*!
 * \brief Enables RTS control if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to control the RTS signal.
 */
static ureg_t rts_control;
#endif

#ifdef UART_CTS_BIT
/*!
 * \brief Enables CTS sense if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to sense the CTS signal.
 */
static ureg_t cts_sense;
#endif


#if defined(UART_HDX_BIT) || defined(UART_HDB_FDX_BIT)
/*
 * \brief USARTn transmit complete interrupt handler.
 *
 * Used with half duplex communication to switch from tranmit to receive
 * mode after the last character has been transmitted.
 *
 * This routine exists only if the hardware configuration defines a
 * port bit to switch between receive and transmit mode.
 *
 * \param arg Pointer to the transmitter ring buffer.
 */
static void McfUsartTxEmpty(void *arg) {

#if (PLATFORM_SUB == REV_D && defined(UART_HDB_FDX_BIT))
	if (hdx_control & HDX_CONTROL_YZ) {

		/* Set Half duplex on second chip*/
		/* Enable receiver, Transceiver DE2 Enable */
		MCF_GPIO_PORT_CHIP2 &= ~(MCF_GPIO_PORT_RE2 | MCF_GPIO_PORT_DE2);
	}
	else
#endif
	{
		/* Last byte from shift register was sent. */
		/* Enable receiver, Transceiver DE2 Enable */
		MCF_GPIO_PORT_CHIP1 &= ~(MCF_GPIO_PORT_RE1 | MCF_GPIO_PORT_DE1);
	}

    /* Disable USART transmit interrupt.  */
    CLR_TXRDY_INTERRUPT();
}
#endif

/*
 * \brief USARTn transmit data register empty interrupt handler.
 *
 * \param arg Pointer to the transmitter ring buffer.
 */
static void McfUsartTxReady(void *arg) {
    register RINGBUF *rbf = (RINGBUF *) arg;

    register uint8_t *cp = rbf->rbf_tail;
    uint8_t postEvent = 0;


#ifdef NUTTRACER
    TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_ENTER,TRACE_INT_UART_TXEMPTY);
#endif

#ifndef UART_NO_SW_FLOWCONTROL

    /*
     * Process pending software flow controls first.
     */
    if (flow_control & (XON_PENDING | XOFF_PENDING)) {
        if (flow_control & XOFF_PENDING) {
            MCF_UARTn_UTB = ASCII_XOFF;
            flow_control |= XOFF_SENT;
        } else {
            MCF_UARTn_UTB = ASCII_XON;
            flow_control &= ~XOFF_SENT;
        }
        flow_control &= ~(XON_PENDING | XOFF_PENDING);
#ifdef NUTTRACER
        TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_EXIT,TRACE_INT_UART_TXEMPTY);
#endif
        return;
    }

    if (flow_control & XOFF_RCVD) {
        /*
         * If XOFF has been received, we disable the transmit interrupts
         * and return without sending anything.
         */
        CLR_TXRDY_INTERRUPT();
#ifdef NUTTRACER
        TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_EXIT,TRACE_INT_UART_TXEMPTY);
#endif
        return;
    }
#endif /* UART_NO_SW_FLOWCONTROL */

    if (rbf->rbf_cnt) {


        /* rbf_cnt != 0, test before call this function */
        rbf->rbf_cnt--;

        /*
         * Start transmission of the next character and clear TXRDY bit
           * in USR register.
        */
        MCF_UARTn_UTB = *cp;

        /*
         * Wrap around the buffer pointer if we reached its end.
         */
        if (++cp == rbf->rbf_last) {
            cp = rbf->rbf_start;
        }
        rbf->rbf_tail = cp;
        if (rbf->rbf_cnt == rbf->rbf_lwm) {
            postEvent = 1;
        }
    }
    if (rbf->rbf_cnt == 0){

#if defined(UART_HDX_BIT) || defined(UART_HDB_FDX_BIT)
        if (!hdx_control)
#endif
        {
            /* Disable USART transmit interrupt. Send last byte in shift register. */
            CLR_TXRDY_INTERRUPT();
        }

        postEvent = 1;
    }
    if (postEvent)
        NutEventPostFromIrq(&rbf->rbf_que);

#ifdef NUTTRACER
    TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_EXIT,TRACE_INT_UART_TXEMPTY);
#endif
}


/*
 * \brief USARTn receive complete interrupt handler.
 *
 *
 * \param arg Pointer to the receiver ring buffer.
 */
static void McfUsartRxComplete(void *arg) {
    register RINGBUF *rbf = (RINGBUF *) arg;

    register size_t cnt;
    register uint8_t ch;


#ifdef NUTTRACER
    TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_ENTER,TRACE_INT_UART_RXCOMPL);
#endif

    register uint8_t postEvent = 0;
    do {
        /*
         * We read the received character as early as possible to avoid overflows
         * caused by interrupt latency. However, reading the error flags must come
         * first, because reading the ATmega128 data register clears the status.
         */
        rx_errors |= MCF_UARTn_USR;
        ch = MCF_UARTn_URB;

#ifndef UART_NO_SW_FLOWCONTROL
        /*
         * Handle software handshake. We have to do this before checking the
         * buffer, because flow control must work in write-only mode, where
         * there is no receive buffer.
         */
        if (flow_control) {
            /* XOFF character disables transmit interrupts. */
            if (ch == ASCII_XOFF) {
                CLR_TXRDY_INTERRUPT();
                flow_control |= XOFF_RCVD;
#ifdef NUTTRACER
                TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_EXIT,TRACE_INT_UART_RXCOMPL);
#endif
                return;
            }
            /* XON enables transmit interrupts. */
            else if (ch == ASCII_XON) {
                SET_TXRDY_INTERRUPT();
                flow_control &= ~XOFF_RCVD;
#ifdef NUTTRACER
                TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_EXIT,TRACE_INT_UART_RXCOMPL);
#endif
                return;
            }
        }
#endif

        /*
         * Check buffer overflow.
         */
        cnt = rbf->rbf_cnt;
        if (cnt >= rbf->rbf_siz) {
            rx_errors |= MCF_UART_USR_OE; // same flag as FIFO Overrun
#ifdef NUTTRACER
            TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_EXIT,TRACE_INT_UART_RXCOMPL);
#endif
            return;
        }

        /* Wake up waiting threads if this is the first byte in the buffer. */
        if (cnt++ == 0){
            // we do this later, to get the other bytes in time..
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
                    if (MCF_UARTn_USR & MCF_UART_USR_TXRDY) {
                        MCF_UARTn_UTB = ASCII_XOFF;
                        flow_control |= XOFF_SENT;
                        flow_control &= ~XOFF_PENDING;
                    } else {
                        flow_control |= XOFF_PENDING;
                    }
                }
            }
        }
#endif


#ifdef UART_RTS_BIT
        /*
         * Check the high watermark for hardware handshake. If the number of
         * buffered bytes is above this mark, then disable RTS.
         */
        else if (rts_control && cnt >= rbf->rbf_hwm) {
        	MCF_GPIO_RCTS_PORT |= MCF_GPIO_RCTS_PORT_RTS;
        }
#endif

        /*
         * Store the character and increment and the ring buffer pointer.
         */
        *rbf->rbf_head++ = ch;
        if (rbf->rbf_head == rbf->rbf_last) {
            rbf->rbf_head = rbf->rbf_start;
        }

        /* Update the ring buffer counter. */
        rbf->rbf_cnt = cnt;

    } while ( MCF_UARTn_USR & MCF_UART_USR_RXRDY ); // byte in buffer?

    // Eventually post event to wake thread
    if (postEvent)
        NutEventPostFromIrq(&rbf->rbf_que);

#ifdef NUTTRACER
    TRACE_ADD_ITEM(TRACE_TAG_INTERRUPT_EXIT,TRACE_INT_UART_RXCOMPL);
#endif
    
}

static void McfUsartInterrupts(void *arg) {
    register USARTDCB *p_dcb = (USARTDCB *) arg;

    if(MCF_UARTn_USR & MCF_UART_USR_RXRDY){
        McfUsartRxComplete(&p_dcb->dcb_rx_rbf);
    }

    if(MCF_UARTn_USR & MCF_UART_USR_TXRDY){
        McfUsartTxReady(&p_dcb->dcb_tx_rbf);
    }

#if defined(UART_HDX_BIT) || defined(UART_HDB_FDX_BIT)
    if(hdx_control && (p_dcb->dcb_tx_rbf.rbf_cnt == 0) && (MCF_UARTn_USR & MCF_UART_USR_TXEMP)){
        McfUsartTxEmpty(&p_dcb->dcb_tx_rbf);
    }
#endif

}

/*!
 * \brief Carefully enable USART hardware functions.
 *
 * Always enable transmitter and receiver, even on read-only or
 * write-only mode. So we can support software flow control.
 */
static void McfUsartEnable(void)
{
    NutEnterCriticalLevel(IH_USART_LEVEL);

    MCF_UARTn_UCR = MCF_UART_UCR_TX_ENABLED;
    MCF_UARTn_UCR = MCF_UART_UCR_RX_ENABLED;

    SET_TXRDY_INTERRUPT();
    SET_RXRDY_INTERRUPT();

    NutExitCritical();
}

/*!
 * \brief Carefully disable USART hardware functions.
 */
static void McfUsartDisable(void)
{
    CLR_ALL_INTERRUPT();
    /*
     * Allow incoming or outgoing character to finish.
     */
    NutDelay(10);

    /*
     * Disable USART transmit and receive.
     */
    MCF_UARTn_UCR = MCF_UART_UCR_TX_DISABLED;
    MCF_UARTn_UCR = MCF_UART_UCR_RX_DISABLED;
}

/*!
 * \brief Query the USART hardware for the selected speed.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The currently selected baudrate.
 */
static uint32_t McfUsartGetSpeed(void)
{
    uint16_t sv;

    sv = usartControlRegister.ubg1 << 8 | usartControlRegister.ubg2;
    return NutGetCpuClock() / (32 * sv);
}

/*!
 * \brief Set the USART hardware bit rate.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param rate Number of bits per second.
 *
 * \return 0 on success, -1 otherwise.
 */
static int McfUsartSetSpeed(uint32_t rate)
{
    uint16_t sv;

    McfUsartDisable();

    sv = (uint16_t) (NutGetCpuClock() / (rate * 32));

    usartControlRegister.ubg1 = (uint8_t) ((sv & 0xFF00) >> 8);
    MCF_UARTn_UBG1 = usartControlRegister.ubg1;
    usartControlRegister.ubg2 = (uint8_t) (sv & 0x00FF);
    MCF_UARTn_UBG2 = usartControlRegister.ubg2;

    McfUsartEnable();

    return 0;
}

/*!
 * \brief Query the USART hardware for the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The number of data bits set.
 */
static uint8_t McfUsartGetDataBits(void)
{
    return (usartControlRegister.umr1 & 0x3) + 5;
}

/*!
 * \brief Set the USART hardware to the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int McfUsartSetDataBits(uint8_t bits)
{
    /* Clear register data bits. */
    usartControlRegister.umr1 &= ~MCF_UART_UMR_BC(0x3);

    switch (bits) {
    case 5:
        usartControlRegister.umr1 |= MCF_UART_UMR_BC_5;
        break;
    case 6:
        usartControlRegister.umr1 |= MCF_UART_UMR_BC_6;
        break;
    case 7:
        usartControlRegister.umr1 |= MCF_UART_UMR_BC_7;
        break;
    case 8:
        usartControlRegister.umr1 |= MCF_UART_UMR_BC_8;
        break;
    }

    McfUsartDisable();

    /* Reset Mode Register */
    MCF_UARTn_UCR = MCF_UART_UCR_RESET_MR;

    MCF_UARTn_UMR1 = usartControlRegister.umr1;

    McfUsartEnable();

    /*
     * Verify the result.
     */
    if (McfUsartGetDataBits() != bits) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Query the USART hardware for the parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return Parity mode, either 0 (disabled), 1 (odd) or 2 (even).
 */
static uint8_t McfUsartGetParity(void)
{
    if((usartControlRegister.umr1 & 0x1C) == MCF_UART_UMR_PM_NONE){
        return 0;
    }
    if((usartControlRegister.umr1 & 0x1C) == MCF_UART_UMR_PM_ODD){
        return 1;
    }
    if ((usartControlRegister.umr1 & 0x1C) == MCF_UART_UMR_PM_EVEN) {
        return 2;
    }
    return -1;
}

/*!
 * \brief Set the USART hardware to the specified parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param mode 0 (none), 1 (odd) or 2 (even).
 *
 * \return 0 on success, -1 otherwise.
 */
static int McfUsartSetParity(uint8_t mode)
{
    /* Clear register parity. */
    usartControlRegister.umr1 &= ~MCF_UART_UMR_PM(0x3);

    switch (mode) {
    case 0:
        usartControlRegister.umr1 |= MCF_UART_UMR_PM_NONE;
        break;
    case 1:
        usartControlRegister.umr1 |= MCF_UART_UMR_PM_ODD;
        break;
    case 2:
        usartControlRegister.umr1 |= MCF_UART_UMR_PM_EVEN;
        break;
    }
    McfUsartDisable();

    /* Reset Mode Register */
    MCF_UARTn_UCR = MCF_UART_UCR_RESET_MR;

    MCF_UARTn_UMR1 = usartControlRegister.umr1;
    McfUsartEnable();

    /*
     * Verify the result.
     */
    if (McfUsartGetParity() != mode) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Query the USART hardware for the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The number of stop bits set, either 1 or 2.
 */
static uint8_t McfUsartGetStopBits(void)
{
    if ((usartControlRegister.umr2 & 0xF) == MCF_UART_UMR_SB_STOP_BITS_1) {
        return 1;
    }
    if ((usartControlRegister.umr2 & 0xF) == MCF_UART_UMR_SB_STOP_BITS_2) {
        return 2;
    }
    if ((usartControlRegister.umr2 & 0xF) == MCF_UART_UMR_SB_STOP_BITS_15) {
        return 15;
    }
    return -1;
}

/*!
 * \brief Set the USART hardware to the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param stop bits 1, 2 or 15 (1,5).
 *
 * \return 0 on success, -1 otherwise.
 */
static int McfUsartSetStopBits(uint8_t bits)
{
    usartControlRegister.umr2 &= ~0xF;

    if (bits == 1) {
        usartControlRegister.umr2 |= MCF_UART_UMR_SB_STOP_BITS_1;
    } else if (bits == 2) {
        usartControlRegister.umr2 |= MCF_UART_UMR_SB_STOP_BITS_2;
    } else if (bits == 15) {
        usartControlRegister.umr2 |= MCF_UART_UMR_SB_STOP_BITS_15;
    }
    McfUsartDisable();
    MCF_UARTn_UMR2 = usartControlRegister.umr2;
    McfUsartEnable();

    /*
     * Verify the result.
     */
    if (McfUsartGetStopBits() != bits) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Query the USART hardware status.
 *
 * \return Status flags.
 */
static uint32_t McfUsartGetStatus(void)
{
    uint32_t rc = 0;

    /*
     * Set receiver error flags.
     */
    if ((rx_errors & MCF_UART_USR_FE) != 0) {
        rc |= UART_FRAMINGERROR;
    }
    if ((rx_errors & MCF_UART_USR_OE) != 0) {
        rc |= UART_OVERRUNERROR;
    }
    if ((rx_errors & MCF_UART_USR_PE) != 0) {
        rc |= UART_PARITYERROR;
    }
    if ((rx_errors & MCF_UART_USR_RB) != 0) {
        rc |= UART_RECEIVEBREAK;
    }

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
    // todo: RTS, CTS, ...

    return rc;
}

/*!
 * \brief Set the USART hardware status.
 *
 * \param flags Status flags.
 *
 * \return 0 on success, -1 otherwise.
 */
static int McfUsartSetStatus(uint32_t flags)
{
    /*
     * Process software handshake control.
     */
    if (flow_control) {

        /* Access to the flow control status must be atomic. */
        NutEnterCriticalLevel(IH_USART_LEVEL);

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
    // todo: RTS, CTS, ...

    /*
     * Clear UART receive errors.
     */
    if (flags & UART_FRAMINGERROR) {
        rx_errors &= ~MCF_UART_USR_FE;
    }
    if (flags & UART_OVERRUNERROR) {
        MCF_UARTn_UCR = MCF_UART_UCR_RESET_ERROR; //reset all errors
        rx_errors &= ~MCF_UART_USR_OE;
    }
    if (flags & UART_PARITYERROR) {
        rx_errors &= ~MCF_UART_USR_PE;
    }
    if (flags & UART_RECEIVEBREAK) {
        rx_errors &= ~MCF_UART_USR_RB;
    }

    /*
     * Verify the result.
     */
    if ((McfUsartGetStatus() & ~UART_ERRORS) != flags) {
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
static uint32_t McfUsartGetFlowControl(void)
{
    uint32_t rc = 0;

    if (flow_control) {
        rc |= USART_MF_XONXOFF;
    } else {
        rc &= ~USART_MF_XONXOFF;
    }

#ifdef UART_RTS_BIT
    if (rts_control) {
        rc |= USART_MF_RTSCONTROL;
    } else {
        rc &= ~USART_MF_RTSCONTROL;
    }
#endif

#ifdef UART_CTS_BIT
    if (cts_sense) {
        rc |= USART_MF_CTSSENSE;
    } else {
        rc &= ~USART_MF_CTSSENSE;
    }
#endif

#if defined(UART_HDB_FDX_BIT)
    if (hdx_control) {
        rc |= USART_MF_HALFDUPLEX;
    } else {
        rc &= ~USART_MF_HALFDUPLEX;
    }
#endif

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
static int McfUsartSetFlowControl(uint32_t flags)
{
    /*
     * Set software handshake mode.
     */
    if (flags & USART_MF_XONXOFF) {
        if(flow_control == 0) {
            NutEnterCriticalLevel(IH_USART_LEVEL);
            flow_control = 1 | XOFF_SENT;  /* force XON to be sent on next read */
            NutExitCritical();
        }
    } else {

        NutEnterCriticalLevel(IH_USART_LEVEL);
        flow_control = 0;
        NutExitCritical();
    }

#ifdef UART_RTS_BIT
    /*
     * Set RTS control mode.
     */
    if (flags & USART_MF_RTSCONTROL) {
        /* Set GPIO function for RTS - PUC2 */
    	MCF_GPIO_RCTS_PAR &= ~MCF_GPIO_RCTS_PAR_RTS(0x03);

        /* Set as an output */
    	MCF_GPIO_RCTS_DDR |= MCF_GPIO_RCTS_DDR_RTS;

        /* Set output enable RTS */
    	MCF_GPIO_RCTS_PORT &= ~MCF_GPIO_RCTS_PORT_RTS;

        rts_control = 1;
    } else if (rts_control) {
        rts_control = 0;
    }
#endif

#ifdef UART_CTS_BIT
    /*
     * Set CTS sense mode.
     */
    if (flags & USART_MF_CTSSENSE) {

        /* Set primary function (usart) for CTS */
    	MCF_GPIO_RCTS_PAR |= MCF_GPIO_RCTS_PAR_CTS;
        /* Set CTS hardware flow control, pointer points after read to UMR2 register after init */
        usartControlRegister.umr2 |= MCF_UART_UMR_TXCTS;
        MCF_UARTn_UMR2 = usartControlRegister.umr2;

        cts_sense = 1;
    } else if (cts_sense) {
        /* Clear CTS hardware flow control, pointer points after read to UMR2 register after init */
        usartControlRegister.umr2 &= ~MCF_UART_UMR_TXCTS;
        MCF_UARTn_UMR2 = usartControlRegister.umr2;
        cts_sense = 0;
    }
#endif

#ifdef UART_HDB_FDX_BIT

	/*
	 * Set half duplex mode.
	 */
	if (flags & USART_MF_HALFDUPLEX) {
	
		/* Half duplex Komunikace pres porty YZ */
		if(flags & USART_MF_HALFDUPLEX_YZ){

			// Disable Half duplex on first chip
			/* Disable Transmitter DE1 = 0 */
			MCF_GPIO_PORT_CHIP1 &= ~MCF_GPIO_PORT_DE1;
			/* Disable Receiver RE1 = 1 */
			MCF_GPIO_PORT_CHIP1 |= MCF_GPIO_PORT_RE1;

		#if PLATFORM_SUB == REV_C

			/* H/F Duplex  - 1 Half duplex*/
			MCF_GPIO_PORTAN |= MCF_GPIO_PORTAN_PORTAN5;
		#elif PLATFORM_SUB == REV_D

			/* Enable Half duplex on second chip*/
			/* RE2 = 0 Enabled */
			MCF_GPIO_PORT_CHIP2 &= ~(MCF_GPIO_PORT_RE2 | MCF_GPIO_PORT_DE2);
		#else
			#error "Please define User Platform Macro PLATFORM_SUB in Nut/OS Configurator."
		#endif
			hdx_control = HDX_CONTROL_YZ | 1;
		}
		/* Half duplex Komunikace pres porty AB */
		else{
		
			/* Enable Receiver RE1 = 0, Disable Transmitter DE1 = 1 */
    		MCF_GPIO_PORT_CHIP1 &= ~(MCF_GPIO_PORT_RE1 | MCF_GPIO_PORT_DE1);

		#if PLATFORM_SUB == REV_C

			/* H/F Duplex  - 1 Half duplex*/
			MCF_GPIO_PORTAN |= MCF_GPIO_PORTAN_PORTAN5;
		#elif PLATFORM_SUB == REV_D

			/* Disable Transmitter and Receiver on second chip*/
			/* RE2 = 1 Disabled */
			MCF_GPIO_PORT_CHIP2 |= MCF_GPIO_PORT_RE2;

			/* DE2 = 0 Disabled*/
			MCF_GPIO_PORT_CHIP2 &= ~MCF_GPIO_PORT_DE2;
		#else
			#error "Please define User Platform Macro PLATFORM_SUB in Nut/OS Configurator."
		#endif
			hdx_control = 1;
		}
	/* Full duplex */
	} else if (hdx_control) {
		hdx_control = 0;
		
		/* Enable Receiver RE1 = 0 */
    	MCF_GPIO_PORT_CHIP1 &= ~MCF_GPIO_PORT_RE1;
	#if PLATFORM_SUB == REV_C
		/* DE - transmit enable: 1 enabled */
		MCF_GPIO_PORT_CHIP1 |= MCF_GPIO_PORT_DE1;

		/* H/F Duplex  - 0 Full duplex*/
		MCF_GPIO_PORTAN &= ~MCF_GPIO_PORTAN_PORTAN5;
	#elif PLATFORM_SUB == REV_D

		/* Transmitter */
		/* RE2 = 1 Disabled, DE2 = 1 Enabled */
		MCF_GPIO_PORT_CHIP2 |= MCF_GPIO_PORT_RE2 | MCF_GPIO_PORT_DE2;
	#endif
	}

#endif

    /*
     * Verify the result.
     */
    if (McfUsartGetFlowControl() != flags) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Start the USART transmitter hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it added one or more bytes to the
 * transmit buffer.
 */
static void McfUsartTxStart(void)
{
#if defined(UART_HDX_BIT) || defined(UART_HDB_FDX_BIT)
    if (hdx_control) {
	#if (PLATFORM_SUB == REV_D && defined(UART_HDB_FDX_BIT))
    	if (hdx_control & HDX_CONTROL_YZ){

			/* Set Half duplex on second chip*/
			/* Receiver RE2 Disable, Enable Transceiver */
    		MCF_GPIO_PORT_CHIP2 |= MCF_GPIO_PORT_RE2 | MCF_GPIO_PORT_DE2;
    	}
    	else
	#endif
    	{
    		/* Disable receiver, Enable Transceiver  */
    		MCF_GPIO_PORT_CHIP1 |= MCF_GPIO_PORT_RE1 | MCF_GPIO_PORT_DE1;
    	}
    }
#endif
    /* Enable Transmit Ready Interrupt */
    SET_TXRDY_INTERRUPT();
}

/*!
 * \brief Start the USART receiver hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it removed enough bytes from the
 * receive buffer. Enough means, that the number of bytes left in
 * the buffer is below the low watermark.
 */
static void McfUsartRxStart(void)
{
    /* Enable Receive Ready Interrupt */
    SET_RXRDY_INTERRUPT();

    /*
     * Do any required software flow control.
     */
    if (flow_control && (flow_control & XOFF_SENT) != 0) {
        NutEnterCriticalLevel(IH_USART_LEVEL);
        if (MCF_UARTn_USR & MCF_UART_USR_TXRDY) {
            MCF_UARTn_UTB = ASCII_XON;
            flow_control &= ~XON_PENDING;
        } else {
            flow_control |= XON_PENDING;
        }
        flow_control &= ~(XOFF_SENT | XOFF_PENDING);
        NutExitCritical();
    }
#ifdef UART_RTS_BIT
    if (rts_control)
        /* Enable RTS. Manually asserted first time, automatically negated if overrun occurs */
    	MCF_GPIO_RCTS_PORT &= ~MCF_GPIO_RCTS_PORT_RTS;
#endif
}

/*
 * \brief Initialize the USART hardware driver.
 *
 * This function is called during device registration by the upper level
 * USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int McfUsartInit(void)
{

    /* Enable UART Port on GPIO */
    MCF_GPIO_PUnPAR = 0 | MCF_GPIO_PUnPAR_URXDn
                | MCF_GPIO_PUnPAR_UTXDn;

    /* Reset Transmitter */
    MCF_UARTn_UCR = MCF_UART_UCR_RESET_TX;

    /* Reset Receiver */
    MCF_UARTn_UCR = MCF_UART_UCR_RESET_RX;

    /* Reset Mode Register */
    MCF_UARTn_UCR = MCF_UART_UCR_RESET_MR;

    /*
     * Initialize input enable control, UACR
     * trigger interrupt when /CTS has change of state
     */
    MCF_UARTn_UACR = MCF_UART_UACR_IEC;

    /*
     * No parity, 8-bits per character, RXRDY is source for interrupt or DMA request
     * enable receiver request-to-send
     */
    usartControlRegister.umr1 = MCF_UART_UMR_PM_NONE // No parity
            | MCF_UART_UMR_BC_8; // 8 bits per character.
    MCF_UARTn_UMR1 = usartControlRegister.umr1;

    /*
     * Normal mode(No echo or loopback), 1 stop bit
     * enable transmitter clear-to-send
     */
    usartControlRegister.umr2 = MCF_UART_UMR_CM_NORMAL
            | MCF_UART_UMR_SB_STOP_BITS_1;
    MCF_UARTn_UMR2 = usartControlRegister.umr2;

    /* Use internal bus clock as the clock source for Rx and Tx */
    MCF_UARTn_UCSR = MCF_UART_UCSR_RCS_SYS_CLK | MCF_UART_UCSR_TCS_SYS_CLK;

    /* Disable all interrupts */
    CLR_ALL_INTERRUPT();

    if (NutRegisterIrqHandler(&sig_UART, McfUsartInterrupts, &dcb_usart))
        return -1;


#if defined(UART_HDX_BIT) || defined(UART_HDB_FDX_BIT)
    /* Set GPIO function for RE - reset enable PUA2 and for DE - data enable PUA3 */
    MCF_GPIO_PAR_CHIP1 &= ~(MCF_GPIO_PAR_RE1(0x03) | MCF_GPIO_PAR_DE1(0x03));

    /* Set as an output */
    MCF_GPIO_DDR_CHIP1 |= MCF_GPIO_DDR_RE1 | MCF_GPIO_DDR_DE1;

    /* Enable Receiver / Half duplex */
    /* Enable RE1 = 0 */
    MCF_GPIO_PORT_CHIP1 &= ~(MCF_GPIO_PORT_RE1 | MCF_GPIO_PORT_DE1);
#endif

#ifdef UART_HDX_BIT
     /* Half duplex default */
    #ifndef UART_HDB_FDX_BIT
        hdx_control = 1;
    #endif
#endif

/* Full duplex default */
#ifdef UART_HDB_FDX_BIT

	#if PLATFORM_SUB == REV_C
        /* Enable Transceiver = 1 */
        MCF_GPIO_PORT_CHIP1 |= MCF_GPIO_PORT_DE1;

        /* Set GPIO function for RE - reset enable PUA2 and for DE - data enable PUA3 */
        MCF_GPIO_PANPAR &= ~MCF_GPIO_PANPAR_PANPAR5;

        /* Set as an output */
        MCF_GPIO_DDRAN |= MCF_GPIO_DDRAN_DDRAN5;

        /* H/F Duplex  - 0 Full duplex*/
        MCF_GPIO_PORTAN &= ~MCF_GPIO_PORTAN_PORTAN5;
	#elif PLATFORM_SUB == REV_D
        /* Usart1 enable full duplex */
        /* Set GPIO function for RE2, DE2 */
        MCF_GPIO_PAR_CHIP2 &= ~(MCF_GPIO_PAR_RE2 | MCF_GPIO_PAR_DE2);

        /* Set as an output */
        MCF_GPIO_DDR_CHIP2 |= MCF_GPIO_DDR_RE2 | MCF_GPIO_PAR_DE2;

        /* Enable Transmitter */
        /* RE2 = 1 Disabled, DE2 = 1 Enabled */
        MCF_GPIO_PORT_CHIP2 |= MCF_GPIO_PORT_RE2 | MCF_GPIO_PORT_DE2;
	#endif
#endif

    return 0;
}

/*
 * \brief Deinitialize the USART hardware driver.
 *
 * This function is called during device deregistration by the upper
 * level USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int McfUsartDeinit(void)
{
    /* Deregister receive and transmit interrupts. */
    NutRegisterIrqHandler(&sig_UART, 0, 0);

    /*
     * Disabling flow control shouldn't be required here, because it's up
     * to the upper level to do this on the last close or during
     * deregistration.
     */
#ifdef UART_HDB_FDX_BIT

    if (hdx_control) {
        hdx_control = 0;
    }
#endif

#ifdef UART_CTS_BIT
    if (cts_sense) {
        cts_sense = 0;
        /* Deregister CTS sense interrupt. */
    }
#endif

#ifdef UART_RTS_BIT
    if (rts_control) {
        rts_control = 0;
    }
#endif

    return 0;
}

/*@}*/
