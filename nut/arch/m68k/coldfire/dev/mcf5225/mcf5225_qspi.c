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

#include <stdint.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <sys/event.h>
#include <dev/irqreg.h>
#include <dev/board.h>
#include <dev/mcf5225_qspi.h>
#include <sys/timer.h>
#include <arch/m68k.h>

#include <sys/atom.h>

/*!
 * \addtogroup xgMcf5225
 */
/*@{*/

typedef int t_spi_ret;

static uint32_t tx_counter, rx_counter;
static uint8_t *p_tx_data, *p_rx_data;
static uint32_t size_data;

static HANDLE mutex_tran_start, mutex_tran_end;

void QspiInterrupt(void *arg);

//TODO: params desc
/*! \brief Auxiliary common functions
 *
 * \param *p_src
 * \param len
 */
static inline void FillBuff(uint8_t *p_src, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		MCF_QSPI_QAR = QSPI_COMMAND_ADDRESS + i;
		MCF_QSPI_QDR = MCF_QSPI_QDR_DATA(MCF_QSPI_QDR_BITSE | MCF_QSPI_QDR_CONT);
		//fill data
		MCF_QSPI_QAR = QSPI_TRANSMIT_ADDRESS + i;

		MCF_QSPI_QDR = (p_src == NULL) ? 0xFF : p_src[i];
		tx_counter++;
	}
}

static inline void SetupXmit(uint32_t len)
{
	//setup start and end queue pointers
	MCF_QSPI_QWR = (MCF_QSPI_QWR & MCF_QSPI_QWR_CSIV) | MCF_QSPI_QWR_NEWQP(0)
			| MCF_QSPI_QWR_ENDQP(len - 1);
}

static inline void StartXmit(void)
{
	// start transmit data
	MCF_QSPI_QDLYR |= MCF_QSPI_QDLYR_SPE;
}

/*! \brief qspi_tx1 - Transmit 1 byte
 *
 * \param uid 	unit ID
 * \param val 	value to send
 *
 * \return QSPI_SUCCESS
 */
t_spi_ret qspi_tx1(uint8_t uid, uint8_t val)
{
	// wait on mutex 'start transmit'
	NutEventWait(&mutex_tran_start, NUT_WAIT_INFINITE);

	tx_counter = 0;
	rx_counter = 0;

	FillBuff(&val, 1);

	MCF_QSPI_QIR |= MCF_QSPI_QIR_SPIF;
	MCF_QSPI_QIR &= ~MCF_QSPI_QIR_SPIFE;

	SetupXmit(1);
	StartXmit();

	while (!(MCF_QSPI_QIR & MCF_QSPI_QIR_SPIF))
		// polling
		;/* Empty Body */

//    MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);

	// release mutex 'start transmit'
	NutEventPost(&mutex_tran_start);

	return QSPI_SUCCESS;
}

/*! \brief Transmit 'len' number of bytes
 *
 * \param uid		unit ID
 * \param p_src		pointer to source buffer
 * \param len		number of bytes to transmit
 *
 * \return QSPI_SUCCESS
 */
t_spi_ret qspi_tx(uint8_t uid, uint8_t * p_src, uint32_t len)
{
	// wait on mutex 'start transmit'
	NutEventWait(&mutex_tran_start, NUT_WAIT_INFINITE);

	tx_counter = 0;
	rx_counter = 0;

	if (len > 16) // interrupt method
	{
		// save source data pointer to interrupt TX buffer
		p_tx_data = p_src;
		// save destination data pointer to interrupt RX buffer
		p_rx_data = NULL;
		// save length of receive data to interrupt data size
		size_data = len;

		// initialization of interrupt 'end of queue'
		MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);

		// limited transmit data length
		len = (len > 16) ? 16 : len;

		// transmit only first 16 bytes of TX data
		FillBuff(p_src, len);

		SetupXmit(len);
		// start transmit data
		StartXmit();

		// wait on mutex 'end transmit'
		NutEventWait(&mutex_tran_end, NUT_WAIT_INFINITE);

	}
	else // polling method
	{
		FillBuff(p_src, len);

		MCF_QSPI_QIR |= MCF_QSPI_QIR_SPIF;
		MCF_QSPI_QIR &= ~MCF_QSPI_QIR_SPIFE;

		SetupXmit(len);
		StartXmit();

		while (!(MCF_QSPI_QIR & MCF_QSPI_QIR_SPIF))
			; /* Empty Body */

//        MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);
	}
	// release mutex 'start transmit'
	NutEventPost(&mutex_tran_start);

	return QSPI_SUCCESS;
}

/*! \brief Receive 'len' number of bytes
 *
 * \param uid		unit ID
 * \param p_dst		pointer to destination buffer
 * \param len		number of bytes to receive
 *
 * \return QSPI_SUCCESS
 */
t_spi_ret qspi_rx(uint8_t uid, uint8_t * p_dst, uint32_t len)
{
	uint8_t j;

	// wait on mutex 'start transmit'
	NutEventWait(&mutex_tran_start, NUT_WAIT_INFINITE);

	tx_counter = 0;
	rx_counter = 0;

	if (len > 16) // interrupt method
	{

		// save source data pointer to interrupt TX buffer
		p_tx_data = NULL;
		// save destination data pointer to interrupt RX buffer
		p_rx_data = p_dst;
		// save length of receive data to interrupt data size
		size_data = len;

		// initialization of interrupt 'end of queue'
		MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);

		// limited transmit data length
		len = (len > 16) ? 16 : len;

		FillBuff(NULL, len);

		SetupXmit(len);
		// start transmit data
		StartXmit();

		// wait on mutex 'end transmit'
		NutEventWait(&mutex_tran_end, NUT_WAIT_INFINITE);
	}
	else // polling method
	{
		FillBuff(NULL, len);

		MCF_QSPI_QIR |= MCF_QSPI_QIR_SPIF;
		MCF_QSPI_QIR &= ~MCF_QSPI_QIR_SPIFE;

		SetupXmit(len);
		StartXmit();

		while (!(MCF_QSPI_QIR & MCF_QSPI_QIR_SPIF))
			; /* Empty Body */

		MCF_QSPI_QAR = QSPI_RECEIVE_ADDRESS;
		for (j = 0; j < len; j++)
		{
			p_dst[j] = MCF_QSPI_QDR;
		}

		MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);
	}
	// release mutex 'start transmit'
	NutEventPost(&mutex_tran_start);

	return QSPI_SUCCESS;
}

/*! \brief Set chip select low.
 *
 * \param uid	Unit ID
 */
void qspi_cs_lo(uint8_t uid)
{
	MCF_GPIO_PORTQS &= ~(MCF_GPIO_PORTQS_PORTQS3);
}

/*! \brief Set chip select high.
 *
 * \param uid	Unit ID
 */
void qspi_cs_hi(uint8_t uid)
{
	MCF_GPIO_PORTQS |= MCF_GPIO_PORTQS_PORTQS3;
}

/*! \brief Lock the SPI for the specific unit.
 *
 * \note This can be useful if multiple units are attached to the same SPI bus.
 *
 * \param uid	Unit ID
 */
void qspi_lock(uint8_t uid)
{
	return;
}

/*! \brief Unlock the SPI for the specific unit.
 *
 * \note This can be useful if multiple units are attached to the same SPI bus.
 *
 * \param uid	Unit ID
 */
void qspi_unlock(uint8_t uid)
{
	return;
}

/*! \brief Set baudrate.
 *
 * \param uid	Unit ID
 * \param br	Baudrate in Hz
 *
 * \return QSPI_SUCCESS or QSPI_ERROR
 */
t_spi_ret qspi_set_baudrate(uint8_t uid, uint32_t br)
{
	uint16_t baudRate;

	baudRate = (NutGetCpuClock() / (2 * br)); /* 80M/(2*BaudRate) */
	baudRate = (baudRate < 2) ? 2 : baudRate;		// set min possible baudrate
	baudRate = (baudRate > 255) ? 255 : baudRate;	// set max possible baudrate
	if ((baudRate > 1) && (baudRate < 256))
	{
		MCF_QSPI_QMR = (MCF_QSPI_QMR & ~MCF_QSPI_QMR_BAUD(0xFF))
				| MCF_QSPI_QMR_BAUD((uint8_t) baudRate);

		return QSPI_SUCCESS;
	}
	else
		return QSPI_ERROR;
}

/*! \brief Get baudrate.
 *
 * \param uid	Unit ID
 * \param p_br	Pointer to baudrate in Hz
 *
 * \return QSPI_SUCCESS
 */
t_spi_ret qspi_get_baudrate(uint8_t uid, uint32_t * p_br)
{
	uint8_t baud;

	baud = MCF_QSPI_QMR & 0xFF;
	*p_br = (NutGetCpuClock() / (2 * baud));

	return QSPI_SUCCESS;
}


/*! \brief Init SPI port.
 *
 * \param uid	Init ID
 *
 * \return ret QSPI_SUCCESS or baud rate
 */
t_spi_ret qspi_init(uint8_t uid)
{
	t_spi_ret ret = QSPI_SUCCESS;

	// CS0 to high
	MCF_GPIO_PORTQS |= MCF_GPIO_PORTQS_PORTQS3;

	MCF_GPIO_PQSPAR |= MCF_GPIO_PQSPAR_QSPI_DOUT_DOUT | MCF_GPIO_PQSPAR_QSPI_DIN_DIN
			| MCF_GPIO_PQSPAR_QSPI_CLK_CLK
			/*| MCF_GPIO_PQSPAR_QSPI_CS0_CS0*/;

	// Port QS Data Direction pin 3 (QSPI_PCS0) as an output
	MCF_GPIO_DDRQS |= MCF_GPIO_DDRQS_DDRQS3;

	// Set as a Master always and set CPOL & CPHA
	MCF_QSPI_QMR = MCF_QSPI_QMR_MSTR /*| MCF_QSPI_QMR_CPHA | MCF_QSPI_QMR_CPOL */;
	// Set number of bits to be transferred for each entry in the queue
	MCF_QSPI_QMR |= MCF_QSPI_QMR_BITS(QSPI_TRANS_SIZE);

	// Set baud rate
	ret |= qspi_set_baudrate(0, QSPI_BAUD_RATE);

	// Use active low as default
	MCF_QSPI_QWR = MCF_QSPI_QWR_CSIV;

	// Set interrupt mode
	MCF_QSPI_QIR = (MCF_QSPI_QIR_WCEFB | MCF_QSPI_QIR_ABRTB | MCF_QSPI_QIR_ABRTL
			| MCF_QSPI_QIR_SPIFE | MCF_QSPI_QIR_WCEF | MCF_QSPI_QIR_ABRT | MCF_QSPI_QIR_SPIF);

	// Register interrupt handler
	NutRegisterIrqHandler(&sig_QSPI_TF, QspiInterrupt, NULL);
	NutIrqEnable(&sig_QSPI_TF);

	// put mutex 'start transmit' into signaled state
	NutEventPost(&mutex_tran_start);

	return ret;
}

/*! \brief Start SPI port.
 *
 * \param uid	Unit ID
 *
 * \return QSPI_SUCCESS
 */
t_spi_ret qspi_start(uint8_t uid)
{
	return QSPI_SUCCESS;
}

/*! \brief Stop SPI port.
 *
 * \param uid	Unit ID
 *
 * \return QSPI_SUCCESS
 */
t_spi_ret qspi_stop(uint8_t uid)
{
	return QSPI_SUCCESS;
}

/*! \brief Delete SPI port.
 *
 * \param uid	Unit ID
 *
 * \return QSPI_SUCCESS
 */
t_spi_ret qspi_delete(uint8_t uid)
{
	return QSPI_SUCCESS;
}

/*! \brief QSPI ISR
 *
 * \note TODO: param?, \brief
 */
void QspiInterrupt(void *arg)
{
	uint32_t i;

	MCF_QSPI_QAR = QSPI_RECEIVE_ADDRESS;
	for (i = rx_counter; i < tx_counter; i++)
	{
		if (p_rx_data != NULL)
		{
			p_rx_data[rx_counter] = MCF_QSPI_QDR;
		}
		rx_counter++;
	}

	FillBuff((p_tx_data != NULL) ? &p_tx_data[tx_counter] : NULL,
			(size_data - tx_counter) > 16 ? 16 : (size_data - tx_counter));

	if (tx_counter == size_data)
		SetupXmit((size_data) % 16);

	if (rx_counter == size_data)
	{
		// release mutex 'end transmit'
		NutEventPostFromIrq(&mutex_tran_end);
		return;
	}

	StartXmit();
}
