/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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
 */

/*
 * \verbatim
 * $Id: stm32_usart.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <cfg/clock.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/usart.h>
#include <arch/cm3/stm/stm32xxxx_rcc.h>
#include <arch/cm3/stm/stm32xxxx_usart.h>
#include <arch/cm3/stm/stm32_usart.h>
#include <arch/cm3/stm/stm32_dma.h>

#if defined(UART_DMA_TXCHANNEL) || defined(UART_DMA_RXCHANNEL)
#include <arch/cm3/stm/stm32_dma.h>
#endif

/* Some function renaming for IRQ handling on uarts
 * that do not have a separate IRQ instance but a
 * merged one with others like DBU on SYS-IRQ or
 * external uarts on GPIO-IRQs
 */
#define NutUartIrqEnable()  USARTn->CR1 |= (USART_CR1_TXEIE | USART_CR1_RXNEIE)
#define NutUartIrqDisable() USARTn->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_RXNEIE)

/*!
 * \addtogroup xgNutArchArmStm32Us
 */
/*@{*/

#if defined(UART_XONXOFF_CONTROL)

/* \brief ASCII code for software flow control, starts transmitter. */
#define ASCII_XON       0x11
/* \brief ASCII code for software flow control, stops transmitter. */
#define ASCII_XOFF      0x13

/* \brief XON transmit pending flag. */
#define XON_PENDING     0x10
/* \brief XOFF transmit pending flag. */
#define XOFF_PENDING    0x20
/* \brief XOFF sent flag. */
#define XOFF_SENT       0x40
/* \brief XOFF received flag. */
#define XOFF_RCVD       0x80
#endif

/* USART default speed if not preset by nutconfig */
#ifndef USART_INIT_BAUTRATE
#define USART_INIT_BAUTRATE USART_INITSPEED
#endif

/*!
 * \brief Receiver error flags.
 */
static uint16_t rx_errors;

/*!
 * \brief Transmit address frame, if not zero.
 */
static uint_fast8_t tx_aframe;

#if defined(US_MODE_HWHANDSHAKE)
/*!
 * \brief Enables RTS control if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to control the RTS signal.
 */
static uint_fast8_t rts_control;

/*!
 * \brief Enables CTS sense if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to sense the CTS signal.
 */
// TODO: CTS Handling
// static uint_fast8_t cts_sense;
#endif

#if defined(UART_XONXOFF_CONTROL)
/*!
 * \brief Handles local software flow control.
 *
 * This variable exists only if the hardware configuration defines a
 * software flow control support.
 */
static uint_fast8_t flow_control;
#endif

/*!
 * \brief Handles local half-duplex flow control.
 *
 */
static uint_fast8_t hdpx_control;

#ifdef USART_485_CTRL
/*!
 * \brief Handles local echo for RS485.
 *
 */
static uint_fast8_t rs485_control;
#endif /* #ifdef USART_485_CTRL */


#if defined(DE_GPIO_PORT) && defined(DE_GPIO_PIN)
/*!
 * \brief Definition of RS485 DE switching functions.
 */
#ifdef USART_4485DE_INV
#define Rs485DE_L() GpioPinSetHigh(DE_GPIO_PORT, DE_GPIO_PIN)
#define Rs485DE_H() GpioPinSetLow(DE_GPIO_PORT, DE_GPIO_PIN)
#else
#define Rs485DE_L() GpioPinSetLow(DE_GPIO_PORT, DE_GPIO_PIN)
#define Rs485DE_H() GpioPinSetHigh(DE_GPIO_PORT, DE_GPIO_PIN)
#endif
#else
#define Rs485DE_L()
#define Rs485DE_H()
#endif

#if defined(NRE_GPIO_PORT) && defined(NRE_GPIO_PIN)
/*!
 * \brief Definition of RS485 NRE switching functions.
 */
// TODO: Figure out fastest way for switching signals
//#define Rs485NRE_L() NRE_GPIO_BASE->BRR=_BV(NRE_GPIO_PIN)
//#define Rs485NRE_H() NRE_GPIO_BASE->BSRR=_BV(NRE_GPIO_PIN)
#ifdef USART_4485RE_INV
#define Rs485NRE_L() GpioPinSetHigh(NRE_GPIO_PORT, NRE_GPIO_PIN)
#define Rs485NRE_H() if( rs485_control == 0) \
    GpioPinSetLow(NRE_GPIO_PORT, NRE_GPIO_PIN)
#else
#define Rs485NRE_L() GpioPinSetLow(NRE_GPIO_PORT, NRE_GPIO_PIN)
#define Rs485NRE_H() if( rs485_control == 0) \
    GpioPinSetHigh(NRE_GPIO_PORT, NRE_GPIO_PIN)
#endif
#else
#define Rs485NRE_L()
#define Rs485NRE_H()
#endif

#if defined(UART_DMA_TXCHANNEL) || defined(UART_DMA_RXCHANNEL)
/*!
 * \brief Handles local DMA block read/write control.
 *
 */
static uint_fast8_t block_control;

#define BC_RD_EN 0x01
#define BC_WR_EN 0x02

#endif

#if 0
#ifdef UART_DMA_TXCHANNEL
static void Stm32UsartDmaTxIrq(void* arg)
{
    NutEventPost(arg);
}
#endif

#ifdef UART_DMA_RXCHANNEL
static void Stm32UsartDmaRxIrq(void* arg)
{
    NutEventPost(arg);
}
#endif
#endif

/*
 * \brief USARTn transmitter ready interrupt handler.
 *
 * \param rbf Pointer to the transmitter ring buffer.
 */
static void Stm32UsartTxReady(RINGBUF * rbf)
{
    register uint8_t *cp = rbf->rbf_tail;

#ifdef UART_DMA_TXCHANNEL
    if (block_control & BC_WR_EN) {
        /* Disable TX-Interrupt */
        CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TXEIE)) = 0;

        /* Setup Transfer */
        DMA_Setup(UART_DMA_TXCHANNEL, UART_DR_PTR, rbf->rbf_blockptr, rbf->rbf_blockcnt,
            DMA_MINC);

        /* Enable TxComplete Interrupt */
        CM3BBREG(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_DMAT)) = 1;
        CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TCIE)) = 1;

        /* Get DMA running */
        DMA_Enable(UART_DMA_TXCHANNEL);

        /* Now wait for the DMA to finish */
        return;
    }
#endif

#if defined(UART_XONXOFF_CONTROL)
    /*
     * Process pending software flow controls first.
     */
    if (flow_control & (XON_PENDING | XOFF_PENDING)) {
        if (flow_control & XON_PENDING) {
            USARTn->DR=ASCII_XOFF;
            flow_control |= XOFF_SENT;
        } else {
            USARTn->DR=ASCII_XON;
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
        CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TXEIE)) = 0;
        USARTn->SR;
        return;
    }
#endif

    /*
     * Check if we have more bytes to transmit.
     */
    if (rbf->rbf_cnt) {
        /*
         * If CTS has been disabled, we disable the transmit interrupts,
         * enable CTS interrupts and return without sending anything.
         */
        // TODO: CTS handling in here

        /* Start transmission of the next character. */
        USARTn->DR=*cp;

        /* Decrement the number of available bytes in the buffer. */
        rbf->rbf_cnt--;

        /* Wrap around the buffer pointer if we reached its end. */
        if (++cp == rbf->rbf_last) {
            cp = rbf->rbf_start;
        }
        rbf->rbf_tail = cp;

        /* Send an event if we reached the low watermark. */
        if (!hdpx_control && (rbf->rbf_cnt == rbf->rbf_lwm)) {
            NutEventPostFromIrq(&rbf->rbf_que);
        }
    }

    if ( rbf->rbf_cnt==0) {
        /*
         * Nothing left to transmit:
         */

        /* Enable transmit complete interrupt */
        CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TCIE))= 1;
        /* Disable transmit interrupts. */
        CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TXEIE)) = 0;
        if( !hdpx_control) {
            /* if half-duplex post the waiting thread after all bits are out.
             * Otherwise he might read back his own echo */
            NutEventPostFromIrq(&rbf->rbf_que);
        }

    }
}


/*
 * \brief USARTn receiver ready interrupt handler.
 *
 *
 * \param rbf Pointer to the receiver ring buffer.
 */
static void Stm32UsartRxReady(RINGBUF * rbf)
{
    register size_t cnt;
    register uint8_t ch;

#ifdef UART_DMA_RXCHANNEL
    if (block_control & BC_RD_EN) {
        /* Disable TX-Interrupt */
        CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RXNEIE)) = 0;

        /* Setup Transfer */
        DMA_Setup(UART_DMA_RXCHANNEL, rbf->rbf_blockptr, UART_DR_PTR, rbf->rbf_blockcnt,
            DMA_MINC);

        /* Enable TxComplete Interrupt */
        CM3BBREG(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_DMAR))= 1;

        /* Get DMA running */
        DMA_Enable(UART_DMA_RXCHANNEL);

        /* Now wait for the DMA to finish */
        return;
    }
#endif

    /*
     * We read the received character as early as possible to avoid overflows
     * caused by interrupt latency.
     */
    ch = USARTn->DR;

    /* Collect receiver errors. */
    rx_errors |= USARTn->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);

#if defined(UART_XONXOFF_CONTROL)
    /*
     * Handle software handshake. We have to do this before checking the
     * buffer, because flow control must work in write-only mode, where
     * there is no receive buffer.
     */
    if (flow_control) {
        /* XOFF character disables transmit interrupts. */
        if (ch == ASCII_XOFF) {
            CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TE))= 0;
            flow_control |= XOFF_RCVD;
            return;
        }
        /* XON enables transmit interrupts. */
        else if (ch == ASCII_XON) {
            CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TE))= 1;
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
        rx_errors |= USART_FLAG_ORE;
        return;
    }

    /* Wake up waiting threads if this is the first byte in the buffer. */
    if (cnt++ == 0){
        NutEventPostFromIrq(&rbf->rbf_que);
    }

#if defined(UART_XONXOFF_CONTROL)
    /*
     * Check the high watermark for software handshake. If the number of
     * buffered bytes is equal or above this mark, then send XOFF.
     */
    else if (flow_control) {
        if(cnt >= rbf->rbf_hwm) {
            if((flow_control & XOFF_SENT) == 0) {
                if (CM3BBREG(USARTnBase, USART_TypeDef, SR, _BI32(USART_CR1_TE))) {
                    USARTn->DR= ASCII_XOFF;
                    flow_control |= XOFF_SENT;
                    flow_control &= ~XOFF_PENDING;
                } else {
                    flow_control |= XOFF_PENDING;
                }
            }
        }
    }
#endif

#ifdef US_MODE_HWHANDSHAKE
    /*
     * Check the high watermark for hardware handshake. If the number of
     * buffered bytes is above this mark, then disable RTS.
     */
    else if (rts_control && cnt >= rbf->rbf_hwm) {
        CM3BBREG(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_RTSE)) = 1;
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
}

/*
 * \brief USARTn transmitter empty interrupt handler.
 *
 * Used with half duplex communication to switch from tranmit to receive
 * mode after the last character has been transmitted.
 *
 * This routine exists only if the hardware configuration defines
 * RS485 control.
 *
 * \param arg Pointer to the transmitter ring buffer.
 */
static void Stm32UsartTxComplete(RINGBUF * rbf)
{
    /*
     * Check if half duplex mode has been enabled and if all characters
     * had been sent out.
     */
#ifdef UART_DMA_TXCHANNEL
    if (block_control & BC_WR_EN) {
        /* Clear TX and TX-Complete interrupt */
        USARTn->CR1 &= ~(USART_CR1_TCIE|USART_CR1_TXEIE);
        /* Disable DMA transfer */
        CM3BBREG(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_DMAT)) = 0;

        if( hdpx_control) {
            /* In case of half-duplex, enable receiver again */
            CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RE)) = 1;
        }

        /* Reset Counter */
        rbf->rbf_blockcnt = 0;
        rbf->rbf_cnt = 0;

        /* Switch to receiver mode. */
        Rs485DE_L();    /* Disable Sender */
        Rs485NRE_L();   /* Enable Receiver */
        NutEventPostFromIrq(&rbf->rbf_que);
        return;
    }
#endif

    if( rbf->rbf_cnt == 0) {
        /* Clear TX and TX-Complete interrupt */
        USARTn->CR1 &= ~(USART_CR1_TCIE|USART_CR1_TXEIE);

        /* Switch to receiver mode. */
        Rs485DE_L();    /* Disable Sender */
        Rs485NRE_L();   /* Enable Receiver */

        /* Clear pending TX-Complete Status */
        CM3BBREG(USARTnBase, USART_TypeDef, SR, _BI32(USART_SR_TC)) = 0;

        if( hdpx_control) {
            /* In case of half-duplex, enable receiver again */
            CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RE))= 1;
        }
        /* Send an event to inform the upper level. */
        NutEventPostFromIrq(&rbf->rbf_que);
    }
}

/*!
 * \brief USART interrupt handler.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Stm32UsartInterrupt(void *arg)
{
    USARTDCB *dcb = (USARTDCB *) arg;
    uint16_t csr = USARTn->SR;

    /* Test for byte received */
    if (csr & USART_SR_RXNE) {
        Stm32UsartRxReady(&dcb->dcb_rx_rbf);
    }
    /* Test for next byte can be transmitted
     * At end of DMA_TX Transfer, both TC and TXE are set.
     * Do not reinitialize transfer in that case!
     */
    if ((csr & USART_SR_TXE) && !CM3BBREG(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_DMAT))) {
        Stm32UsartTxReady(&dcb->dcb_tx_rbf);
    }
    /* Last byte has been sent completely. */
    if (csr & USART_SR_TC) {
        Stm32UsartTxComplete(&dcb->dcb_tx_rbf);
    }
}

/*!
 * \brief Carefully enable USART hardware functions.
 *
 * Always enabale transmitter and receiver, even on read-only or
 * write-only mode. So we can support software flow control.
 */
static void Stm32UsartEnable(void)
{
    /* Enable UART, receiver and transmitter. */
    USARTn->CR1 |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;

    /* Enable Usart Interrupts */
    NutUartIrqEnable();
}

/*!
 * \brief Carefully disable USART hardware functions.
 *
 * This routine is called before changing cruical settings like
 * baudrate, frame format etc.
 *
 * The previous version uses a 10ms delay to make sure, that any
 * incoming or outgoing character is processed. However, this time
 * depends on the baudrate.
 *
 * In fact we do not need to take care of incoming characters,
 * when changing such settings.
 *
 * For outgoing characters however, settings may be changed on
 * the fly and we should wait, until the last character transmitted
 * with the old settings has left the shift register. While TXRDY
 * is set when the holding register is empty, TXEMPTY is set when the
 * shift register is empty. The bad news is, that both are zero, if
 * the transmitter is disabled. We are not able to determine this
 * state. So we check TXRDY first and, if set, wait for any character
 * currently in the shift register.
 */
static void Stm32UsartDisable(void)
{
    /* Disable Usart Interrupts*/
    NutUartIrqDisable();
    /* Wait until all bits had been shifted out. */
    while (CM3BBREG(USARTnBase, USART_TypeDef, SR, _BI32(USART_SR_TXE)) == 0);
    /* Disable USART. */
    USARTn->CR1 &= ~(USART_CR1_TE|USART_CR1_RE);
}

/*!
 * \brief Query the USART hardware for the selected speed.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The currently selected baudrate.
 */
static uint32_t Stm32UsartGetSpeed(void)
{
    uint32_t clk, frac_div = USARTn->BRR;
    RCC_ClocksTypeDef RCC_ClocksStatus;

    RCC_GetClocksFreq(&RCC_ClocksStatus);
#if USARTclk == NUT_HWCLK_PCLK1
    clk = RCC_ClocksStatus.PCLK2_Frequency;
#else
    clk = RCC_ClocksStatus.PCLK1_Frequency;
#endif
    if (CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_OVER8)))
    {
        uint32_t  frac = frac_div & 7;
        frac_div >>= 1;
        frac_div &= ~0x7;
        frac_div |= frac;
    }
    return clk / frac_div;
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
static int Stm32UsartSetSpeed(uint32_t rate)
{
    uint32_t tmpreg = 0x00;
    uint32_t apbclock = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;
    RCC_ClocksTypeDef RCC_ClocksStatus;

    Stm32UsartDisable();

    RCC_GetClocksFreq(&RCC_ClocksStatus);
#if USARTclk == NUT_HWCLK_PCLK1
    apbclock = RCC_ClocksStatus.PCLK2_Frequency;
#else
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;
#endif

    /* Determine the integer part */
    if (CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_OVER8)))
    {
        /* In case Oversampling mode is 8 Samples */
        integerdivider = ((25 * apbclock) / (2 * rate));
        if (integerdivider > 200)
        {
            /* switch back to 16 Sample Oversampling when possible*/
            CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_OVER8)) = 0;
            integerdivider = ((25 * apbclock) / (4 * rate));
        }
    }
    else
    {
        /* In case Oversampling mode is 16 Samples */
        integerdivider = ((25 * apbclock) / (4 * rate));
        if (integerdivider < 100)
        {
            /* switch back to 8 Sample Oversampling*/
            CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_OVER8)) = 1;
            integerdivider = ((25 * apbclock) / (2 * rate));
        }
    }
    /* Cap rate to highest possible */
    if (integerdivider <100)
        integerdivider = 100;
    tmpreg = (integerdivider / 100) << 4;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

    if (CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_OVER8)))
    {
        /* In case Oversampling mode is 8 Samples */
        tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
    }
    else
    {
        /* In case Oversampling mode is 16 Samples */
        tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
    }

    /* Write to USART BRR */
    USARTn->BRR = (uint16_t)tmpreg;

    Stm32UsartEnable();
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
static uint8_t Stm32UsartGetDataBits(void)
{
    uint16_t val = USARTn->CR1 & USART_CR1_M;

    if (val) {
        val = 9;
    } else {
        val = 8;
    }
    return (uint8_t)val;
}

/*!
 * \brief Set the USART hardware to the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetDataBits(uint8_t bits)
{
    int rc = 0;
    uint16_t val = USARTn->CR1;

    switch( bits) {
        case 8:
            val &= ~USART_CR1_M;
            break;
        case 9:
            val |= USART_CR1_M;
            break;
        default:
            return -1;
    }
    Stm32UsartDisable();
    USARTn->CR1=val;
    Stm32UsartEnable();

    /*
     * Verify the result.
     */
    if (Stm32UsartGetDataBits() != bits) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware for the parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return Parity mode, either 0 (disabled), 1 (odd), 2 (even) or 9 (multidrop).
 */
static uint8_t Stm32UsartGetParity(void)
{
    uint16_t val = USARTn->CR1 & (USART_CR1_PCE|USART_CR1_PS);

    if (val == (USART_CR1_PCE|USART_CR1_PS))
        val = 1;
    else if (val == USART_CR1_PCE)
        val = 2;
    else
        val = 0;

    return (uint8_t)val;
}

/*!
 * \brief Set the USART hardware to the specified parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param mode 0 (disabled), 1 (odd) or 2 (even)
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetParity(uint8_t mode)
{
    int rc = 0;
    uint16_t val = USARTn->CR1 & ~(USART_CR1_PCE|USART_CR1_PS);

    switch (mode) {
        case 0:
            break;
        case 1:
            val |= USART_CR1_PCE|USART_CR1_PS;
            break;
        case 2:
            val |= USART_CR1_PCE;
            break;
        default:
            return -1;
            break;
    }
    Stm32UsartDisable();
    USARTn->CR1=val;
    Stm32UsartEnable();

    /*
     * Verify the result.
     */
    if (Stm32UsartGetParity() != mode) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware for the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The number of stop bits set, either 1, 2 or 3 (1.5 bits).
 */
static uint8_t Stm32UsartGetStopBits(void)
{
    uint16_t val = USARTn->CR2 & USART_CR2_STOP;

    if (val == 0) {
        val = 1;
    }
    else if (val == USART_CR2_STOP_1) {
        val = 2;
    }
    else if (val == USART_CR2_STOP){
        val = 3;
    }
    return (uint8_t)val;
}

/*!
 * \brief Set the USART hardware to the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
int Stm32UsartSetStopBits(uint8_t bits)
{
    int rc = 0;
    uint16_t val = USARTn->CR2 & ~USART_CR2_STOP;

    switch(bits) {
        case 1:
            break;
        case 2:
            val |= USART_CR2_STOP_1;
            break;
        case 3:
            val |= USART_CR2_STOP;
            break;
        default:
            return -1;
            break;
    }
    Stm32UsartDisable();
    USARTn->CR2 = val;
    Stm32UsartEnable();

    /*
     * Verify the result.
     */
    if (Stm32UsartGetStopBits() != bits) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware status.
 *
 * \return Status flags.
 */
static uint32_t Stm32UsartGetStatus(void)
{
    uint32_t rc = 0;
#if defined(US_MODE_HWHANDSHAKE)
    uint16_t csr = USARTn->SR;
#endif

    /*
     * Set receiver error flags.
     */
    if ((rx_errors & USART_SR_FE) != 0) {
        rc |= UART_FRAMINGERROR;
    }
    if ((rx_errors & USART_SR_ORE) != 0) {
        rc |= UART_OVERRUNERROR;
    }
    if ((rx_errors & USART_SR_PE) != 0) {
        rc |= UART_PARITYERROR;
    }

#if defined(UART_XONXOFF_CONTROL)
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
#endif

    /*
     * Determine hardware handshake control status.
     */
#if defined(US_MODE_HWHANDSHAKE)
    // TODO: reflect RTS state
#endif

    /*
     * Determine hardware handshake sense status.
     */
#if defined(US_MODE_HWHANDSHAKE)
    // TODO: reflect CTS state
#endif

    /*
     * If transmitter and receiver haven't been detected disabled by any
     * of the checks above, then they are probably enabled.
     */
    if ((rc & UART_RXDISABLED) == 0) {
        rc |= UART_RXENABLED;
    }
    if ((rc & UART_TXDISABLED) == 0) {
        rc |= UART_TXENABLED;
    }

    /*
     * Process multidrop setting.
     */
    if (tx_aframe) {
        rc |= UART_TXADDRFRAME;
    } else {
        rc |= UART_TXNORMFRAME;
    }
    return rc;
}

/*!
 * \brief Set the USART hardware status.
 *
 * \param flags Status flags.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetStatus(uint32_t flags)
{
    int rc = 0;
    /*
     * Process software handshake control.
     */

#if defined(UART_XONXOFF_CONTROL)
    if (flow_control) {

        /* Access to the flow control status must be atomic. */
        NutUartIrqDisable();

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
        NutUartIrqEnable();
    }
#endif

    /*
     * Process multidrop setting.
     */
    if (flags & UART_TXADDRFRAME) {
        tx_aframe = 1;
    }
    if (flags & UART_TXNORMFRAME) {
        tx_aframe = 0;
    }

    /*
     * Clear USART receive errors.
     */
    if (flags & UART_ERRORS) {
        USARTn->SR;
        USARTn->DR;
    }

    /*
     * Verify the result.
     */
    if ((Stm32UsartGetStatus() & ~UART_ERRORS) != flags) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware for synchronous mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * Not implemented for the STM32 USART. Always returns 0.
 *
 * \return Or-ed combination of \ref UART_SYNC, \ref UART_MASTER,
 *         \ref UART_NCLOCK and \ref UART_HIGHSPEED.
 */
uint8_t Stm32UsartGetClockMode(void)
{
    return 0;
}

/*!
 * \brief Set asynchronous or synchronous mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * Not implemented for the STM32 USART. Always returns -1.
 *
 * \param mode Must be an or-ed combination of USART_SYNC, USART_MASTER,
 *             USART_NCLOCK and USART_HIGHSPEED.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetClockMode(uint8_t mode)
{
    return -1;
}

/*!
 * \brief Query flow control mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return See UsartIOCtl().
 */
static uint32_t Stm32UsartGetFlowControl(void)
{
    uint32_t rc = 0;

#if defined(UART_XONXOFF_CONTROL)
    if (flow_control) {
        rc |= USART_MF_XONXOFF;
    } else {
        rc &= ~USART_MF_XONXOFF;
    }
#endif

    if( hdpx_control) {
        rc |= USART_MF_HALFDUPLEX;
    } else {
        rc &= ~USART_MF_HALFDUPLEX;
    }

#if defined( USART_485_CTRL)
    if (rs485_control) {
        rc |= USART_MF_LOCALECHO;
    } else {
        rc &= ~USART_MF_LOCALECHO;
    }
#endif

#ifdef UART_DMA_RXCHANNEL
    if (block_control & BC_RD_EN)
        rc |= USART_MF_BLOCKREAD;
    else
        rc &= ~USART_MF_BLOCKREAD;
#endif

#ifdef UART_DMA_TXCHANNEL
    if (block_control & BC_WR_EN)
        rc |= USART_MF_BLOCKWRITE;
    else
        rc &= ~USART_MF_BLOCKWRITE;
#endif

    return rc;
}

/*!
 * \brief Set flow control mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param flags See UsartIOCtl().
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetFlowControl(uint32_t flags)
{
#if defined(UART_XONXOFF_CONTROL)
    NutUartIrqDisable();
    /*
     * Set software handshake mode.
     */
    if (flags & USART_MF_XONXOFF) {
        if(flow_control == 0) {
            flow_control = 1 | XOFF_SENT;  /* force XON to be sent on next read */
        }
    } else {
        flow_control = 0;
    }
    NutUartIrqEnable();
#endif

    /* Setup half-duplex mode */
    if (flags & USART_MF_HALFDUPLEX) {
        hdpx_control = 1;
    }
    else if( hdpx_control == 1) {
        hdpx_control = 0;
    }

#if defined( USART_485_CTRL)
    /* Control local echo mode:
     * Established by keeping /RE low while DE high. */
    if (flags & USART_MF_LOCALECHO) {
        rs485_control = 1;
        Rs485NRE_L();
    }
    else if (rs485_control) {
        rs485_control = 0;
    }
#endif

#ifdef UART_DMA_RXCHANNEL
    /* Setup block mode */
    if( flags & USART_MF_BLOCKREAD) {
        block_control |= BC_RD_EN;
    }
    else {
        block_control &= ~BC_RD_EN;
    }
#endif

#ifdef UART_DMA_TXCHANNEL
    if( flags & USART_MF_BLOCKWRITE) {
        block_control |= BC_WR_EN;
    }
    else {
        block_control &= ~BC_WR_EN;
    }
#endif

    /*
     * Verify the result.
     */
    if ((Stm32UsartGetFlowControl() & USART_MF_FLOWMASK) !=
        (flags & USART_MF_FLOWMASK)) {
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
static void Stm32UsartTxStart(void)
{
    Rs485DE_H();    /* Enable Sender */
    Rs485NRE_H();   /* Disable Receiver */

    if( hdpx_control) {
        /* Disable Receiver if half-duplex */
        CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RE)) = 0;
    }

    /* Clear Transmit Complete flag */
    CM3BBREG(USARTnBase, USART_TypeDef, SR, _BI32(USART_SR_TC)) = 0;
    /* Enable transmit interrupts. */
    CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TXEIE)) = 1;
}

/*!
 * \brief Start the USART receiver hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it removed enough bytes from the
 * receive buffer. Enough means, that the number of bytes left in
 * the buffer is below the low watermark.
 */
static void Stm32UsartRxStart(void)
{
#if defined(UART_XONXOFF_CONTROL)
    /*
     * Do any required software flow control.
     */
    if (flow_control && (flow_control & XOFF_SENT) != 0) {
        NutUartIrqDisable();
        if (CM3BBREG(USARTnBase, USART_TypeDef, SR, _BI32(USART_SR_TXE))) {
            USARTn->DR=ASCII_XON;
            flow_control &= ~XON_PENDING;
        } else {
            flow_control |= XON_PENDING;
        }
        flow_control &= ~(XOFF_SENT | XOFF_PENDING);
        NutUartIrqEnable();
    }
#endif
    /* Enable receive interrupts. */
    CM3BBREG(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RXNEIE)) = 1;
}

/*
 * \brief Initialize the USART hardware driver.
 *
 * This function is called during device registration by the upper level
 * USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartInit(void)
{
    uint16_t cr3 = 0;

    /*
     * Register receive and transmit interrupts.
     */
    if (NutRegisterIrqHandler(&SigUSART, Stm32UsartInterrupt, &DcbUSART)) {
        return -1;
    }

    /* Enable UART clock */
#if USARTclk == NUT_HWCLK_PCLK1
        RCC_APB2PeriphClockCmd(STM_USART_CLK, ENABLE);
        /* Reset USART IP */
        RCC_APB2PeriphResetCmd(STM_USART_CLK, ENABLE);
        RCC_APB2PeriphResetCmd(STM_USART_CLK, DISABLE);
#else
        RCC_APB1PeriphClockCmd(STM_USART_CLK, ENABLE);
        /* Reset USART IP */
        RCC_APB1PeriphResetCmd(STM_USART_CLK, ENABLE);
        RCC_APB1PeriphResetCmd(STM_USART_CLK, DISABLE);
#endif

    /* Configure USART Tx as alternate function push-pull */
    GpioPinConfigSet( TX_GPIO_PORT, TX_GPIO_PIN, GPIO_CFG_OUTPUT|GPIO_CFG_PERIPHAL);
    /* Configure USART Rx as input floating */
    GpioPinConfigSet( RX_GPIO_PORT, RX_GPIO_PIN, GPIO_CFG_PERIPHAL);

#if defined(RTS_GPIO_PORT) && defined(RTS_GPIO_PIN)
    /* Configure USART RTS as alternate function push-pull */
    GpioPinConfigSet( RTS_GPIO_PORT, RTS_GPIO_PIN, GPIO_CFG_PERIPHAL);
#endif
#if defined(CTS_GPIO_PORT) && defined(CTS_GPIO_PIN)
    /* Configure USART CTS as input floating */
    GpioPinConfigSet( CTS_GPIO_PORT, CTS_GPIO_PIN, GPIO_CFG_PERIPHAL);
#endif

#if defined(DE_GPIO_PORT) && defined(DE_GPIO_PIN)
   /* Configure RS485 direction switching pin */
    GpioPinConfigSet( DE_GPIO_PORT, DE_GPIO_PIN, GPIO_CFG_OUTPUT);
#endif
#if defined(NRE_GPIO_PORT) && defined(NRE_GPIO_PIN)
    /* Configure RS485 direction switching pin */
    GpioPinConfigSet( NRE_GPIO_PORT, NRE_GPIO_PIN, GPIO_CFG_OUTPUT);
#endif

    /* USART Related GPIO Init */
#if defined (MCU_STM32F1)
 #ifdef STM_USART_REMAP_MASK
    AFIO->MAPR &= ~STM_USART_REMAP_MASK;
    AFIO->MAPR |=  STM_USART_REMAP_VALUE;
 #endif
#else
    GPIO_PinAFConfig((GPIO_TypeDef*) TX_GPIO_PORT, TX_GPIO_PIN,  STM_USART_REMAP);
    GPIO_PinAFConfig((GPIO_TypeDef*) RX_GPIO_PORT, RX_GPIO_PIN,  STM_USART_REMAP);
#if defined(RTS_GPIO_PORT) && defined(RTS_GPIO_PIN)
    GPIO_PinAFConfig((GPIO_TypeDef*) RTS_GPIO_PORT, RTS_GPIO_PIN,  STM_USART_REMAP);
#endif
#if defined(CTS_GPIO_PORT) && defined(CTS_GPIO_PIN)
    GPIO_PinAFConfig((GPIO_TypeDef*) CTS_GPIO_PORT, CTS_GPIO_PIN,  STM_USART_REMAP);
#endif
#endif/*F1 */

    /*
     *   USART Communication Init
     */

    /* USART configuration */
    Stm32UsartSetSpeed(USART_INIT_BAUTRATE);
    Stm32UsartSetDataBits(8);
    Stm32UsartSetStopBits(2);
    Stm32UsartSetParity(0);

    /* Enable additional features */
#ifdef USART_HWFLOWCTRL
    /* Enable hardware handshake options */
#if defined(RTS_GPIO_PORT) && defined(RTS_GPIO_PIN)
    cr3 |= USART_CR3_RTSE;
#endif
#if defined(CTS_GPIO_PORT) && defined(CTS_GPIO_PIN)
    cr3 |= USART_CR3_CTSE;
#endif
#endif

#ifdef USART_MODE_IRDA
    cr3 |= USART_CR3_IREN;
#endif

#ifdef USART_HARDWARE_HDX
    cr3 |= USART_CR3_HDSEL;
#endif

    USARTn->CR3 = cr3;
    /* Disable transmitter interrupts as they will
     * be enabled by the TxStart routine */
    USARTn->CR1 &= ~(USART_CR1_TXEIE|USART_CR1_TCIE|USART_CR1_RXNEIE);
    /* Enable USART */
    USARTn->CR1 |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;

#if defined(UART_DMA_TXCHANNEL) || defined(UART_DMA_RXCHANNEL)
    DMA_Init();
#endif
    NutIrqEnable(&SigUSART);
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
static int Stm32UsartDeinit(void)
{
    /* Disable interrupts */
    NutIrqDisable(&SigUSART);
    USARTn->CR1 = 0;

    /* Deregister receive and transmit interrupts. */
    NutRegisterIrqHandler(&SigUSART, 0, 0);

    /* Reset UART. */
#if USARTclk == NUT_HWCLK_PCLK1
    RCC_APB2PeriphResetCmd(STM_USART_CLK, ENABLE);
    RCC_APB2PeriphResetCmd(STM_USART_CLK, DISABLE);
#else
    RCC_APB1PeriphResetCmd(STM_USART_CLK, ENABLE);
    RCC_APB1PeriphResetCmd(STM_USART_CLK, DISABLE);
#endif

    return 0;
}

/*@}*/
