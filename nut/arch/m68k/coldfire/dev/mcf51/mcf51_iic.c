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

#include <string.h>

#include <dev/irqreg.h>

#include <sys/event.h>
#include <sys/atom.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/heap.h>
#include <stdio.h>
#include <dev/twif.h>

#include <arch/m68k.h>
#include <arch/m68k/coldfire/mcf51_iic.h>

/*!
 * \addtogroup xgMcf51
 */
/*@{*/

#define NUT_THREAD_TWI_TRANSACT_STACK   	1000			// TODO .. tune it

typedef struct
{
	uint8_t dcb_base;
	HANDLE tw_mm_mutex; /* Exclusive master access. */
	HANDLE tw_mm_que; /* Threads waiting for master transfer done. */
	volatile uint8_t tw_mm_err; /* Current master mode error. */
	uint8_t tw_mm_error; /* Last master mode error. */

	uint32_t AddrLenM; /* Length of input bufer's content */
	uint32_t InpLenM; /* Length of input bufer's content */
	uint32_t OutLenM; /* Length of output bufer's content */
	uint32_t tw_mr_cnt; /* Number of received bytes */
	uint8_t *DataPtrM; /* Pointer to data buffer for Master mode */
	uint8_t *AddrPtrM; /* Pointer to output buffer for Master mode */
	volatile uint8_t tw_if_bsy; /* Bus busy flag */
	uint8_t tw_rptst_rqst; /* Request for repeated start */
	uint8_t tw_mm_sla; /* Destination slave address. */
} TWIDCB;

static TWIDCB dcb_twi[2];

/*
 * TWI interrupt handler.
 */
static void TwInterrupt(void *arg)
{
	uint8_t Status; /* Safe status register */
	TWIDCB *dev = arg;

//	MCF_IIC_SR(dev->dcb_base) = MCF_IIC_SR_IIF; /* Clear interrupt flag if set (by means of writing 1 to its bit) */

	Status = MCF_IIC_SR(dev->dcb_base);
	if (MCF_IIC_CR(dev->dcb_base) & MCF_IIC_CR_MST)
	{ /* Is device in master mode? */
		if (MCF_IIC_CR(dev->dcb_base) & MCF_IIC_CR_TX)
		{ /* Is device in Tx mode? */
			if (Status & MCF_IIC_SR_RXAK)
			{ /* NACK received? */
				MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_MST; /* Switch device to slave mode (stop signal sent) */
				MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_TX; /* Switch to Rx mode */
				dev->AddrLenM = 0U;
				dev->OutLenM = 0U; /* No character for sending */
				dev->InpLenM = 0U; /* No character for reception */
				dev->tw_if_bsy = 0;
			}
			else
			{
				if (dev->AddrLenM)
				{ /* Is any char. for transmitting? */
					dev->AddrLenM--; /* Decrease number of chars for the transmit */
					MCF_IIC_DR(dev->dcb_base) = *(dev->AddrPtrM)++; /* Send character */
				}
				else if (dev->OutLenM)
				{ /* Is any char. for transmitting? */
					dev->OutLenM--; /* Decrease number of chars for the transmit */
					MCF_IIC_DR(dev->dcb_base) = *(dev->DataPtrM)++; /* Send character */
				}
				else
				{
					if (dev->InpLenM)
					{ /* Is any char. for reception? */
						if (dev->tw_rptst_rqst)
						{
							MCF_IIC_CR(dev->dcb_base) |= MCF_IIC_CR_RSTA; /* Resend Start */
							MCF_IIC_DR(dev->dcb_base) = (uint8_t) (dev->tw_mm_sla | 0x01); /* device id to read */
							dev->tw_rptst_rqst = 0;
						}
						else
						{
							if (dev->InpLenM == 1U)
							{ /* If only one char to receive */
								MCF_IIC_CR(dev->dcb_base) |= MCF_IIC_CR_TXAK; /* then transmit ACK disable */
							}
							else
							{
								MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_TXAK; /* else transmit ACK enable */
							}
							MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_TX; /* Switch to Rx mode */
							(void) MCF_IIC_DR(dev->dcb_base); /* Dummy read character */
						}
					}
					else
					{
						dev->tw_if_bsy = 0;
						MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_MST; /* Switch device to slave mode (stop signal sent) */
						MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_TX; /* Switch to Rx mode */
						NutEventPostFromIrq(&dev->tw_mm_que);
					}
				}
			}
		}
		else
		{
			dev->InpLenM--; /* Decrease number of chars for the receive */
			if (dev->InpLenM)
			{ /* Is any char. for reception? */
				if (dev->InpLenM == 1U)
				{
					MCF_IIC_CR(dev->dcb_base) |= MCF_IIC_CR_TXAK; /* Transmit ACK disable */
				}
			}
			else
			{
				MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_MST; /* If no, switch device to slave mode (stop signal sent) */
				MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_TXAK; /* Transmit ACK enable */
			}
			*(dev->DataPtrM)++ = MCF_IIC_DR(dev->dcb_base); /* Receive character */
			dev->tw_mr_cnt++;
			if (!dev->InpLenM)
			{ /* Is any char. for reception? */
				NutEventPostFromIrq(&dev->tw_mm_que);
			}
		}
	}
	else
	{
		if (Status & MCF_IIC_SR_ARBL)
		{ /* Arbitration lost? */
			dev->AddrLenM = 0U; /* No address character for sending */
			dev->OutLenM = 0U; /* No character for sending */
			dev->InpLenM = 0U; /* No character for reception */
			dev->tw_if_bsy = 0; /* No character for sending or reception*/
			MCF_IIC_CR(dev->dcb_base) &= ~MCF_IIC_CR_TX; /* Switch to Rx mode */
			dev->tw_mm_err = TWERR_BUS;
		}
	}
}

static void reenableDevice(int dcbBase)
{
	/* clear control register */
	MCF_IIC_CR (dcbBase) = 0;

	/* enable module and send a START condition*/
	MCF_IIC_CR (dcbBase) = MCF_IIC_CR_IICEN | MCF_IIC_CR_MST;

	/* dummy read */

	(void) MCF_IIC_DR(dcbBase);

	/* clear status register (IAAS=x, ARBL=1, IICIF=1) */
	MCF_IIC_SR (dcbBase) = 0xFF;

	/* clear control register */
	MCF_IIC_CR (dcbBase) = 0;

	/* enable the module again */
	MCF_IIC_CR (dcbBase) = MCF_IIC_CR_IICEN;
}

//TODO check \param
/*!
 * \brief Transmit and/or receive data as a master.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \note This function is only available on ATmega128 systems.
 *
 * \param sla    	Slave address of the destination. This slave address
 *               	must be specified as a 7-bit address. For example, the
 *               	PCF8574A may be configured to slave addresses from 0x38
 *               	to 0x3F.
 * \param *addr		Pointer to address for transmit.
 * \param addrsiz	Length of data.
 * \param *data		Pointer to data for transmit or receive.
 * \param siz       Length of data to transmit.
 * \param tmo    	Timeout in milliseconds. To disable timeout, set this
 *               	parameter to NUT_WAIT_INFINITE.
 * \param write		1 = data to transmit, otherwise = data to receive
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int TwMasterCommon(uint8_t sla, const void *addr, uint16_t addrsiz, void *data, uint16_t siz,
		uint32_t tmo, uint8_t write)
{
	int rc = -1;
	TWIDCB *dev = &dcb_twi[DCB_BASE(sla)];

	/* This routine is marked reentrant, so lock the interface. */
	if (NutEventWait(&dev->tw_mm_mutex, 500))
	{
		dev->tw_mm_err = TWERR_IF_LOCKED;
		return -1;
	}

	/*
	 TODO: DF - commented out, check
	 uint8_t control = MCF_IIC_CR(dev->dcb_base);
	 control++;
	 */
	/* Set all parameters for master mode. */
	dev->tw_mm_err = 0;
	dev->tw_mr_cnt = 0;
	dev->tw_mm_sla = (uint8_t) (sla << 1); /* Set slave address */
	uint8_t slave_addr = (uint8_t) (dev->tw_mm_sla); /* Prepare slave address (default = write) */

	dev->AddrPtrM = (uint8_t *) addr; /* Save pointer to address for transmit */
	dev->AddrLenM = addrsiz; /* Set length of data */

	dev->DataPtrM = (uint8_t *) data; /* Save pointer to data for transmit or receive */

	if (write)
	{
		dev->OutLenM = siz; /* Set length of data to transmit */
		dev->InpLenM = 0;
	}
	else
	{
		dev->OutLenM = 0;
		dev->InpLenM = siz; /* Set length of data to receive */

		if (dev->InpLenM && dev->AddrLenM)
			dev->tw_rptst_rqst = 1;
		else
			dev->tw_rptst_rqst = 0;

		if (dev->AddrLenM == 0)
			slave_addr |= 0x01; /* -> read */
	}

	if ((MCF_IIC_SR(dev->dcb_base) & MCF_IIC_SR_BUSY) || dev->tw_if_bsy)
	{ /* Is the bus busy */
		reenableDevice(dev->dcb_base);

		if (dev->dcb_base == 0)
		{
			NutIrqEnable(&sig_IIC1);
		}
		else
		{
			NutIrqEnable(&sig_IIC2);
		}
	}

	/* Clear the queue. */
	//*broken?! NutEventBroadcastAsync(&tw_mm_que);
	if (dev->tw_mm_que == SIGNALED)
	{
		dev->tw_mm_que = 0;
	}

	NutEnterCritical(); /* Enter the critical section */

	MCF_IIC_CR(dev->dcb_base) |= MCF_IIC_CR_TX; /* Set TX mode */
	if (MCF_IIC_CR(dev->dcb_base) & MCF_IIC_CR_MST)
	{ /* Is device in master mode? */
		MCF_IIC_CR(dev->dcb_base) |= MCF_IIC_CR_RSTA; /* If yes then repeat start cycle generated */
	}
	else
	{
		MCF_IIC_CR(dev->dcb_base) |= MCF_IIC_CR_MST; /* If no then start signal generated */
	}
	MCF_IIC_DR(dev->dcb_base) = slave_addr;

	NutExitCritical(); /* Exit the critical section */

	/* Wait for transact complete. */
	if (NutEventWait(&dev->tw_mm_que, tmo))
	{
		dev->tw_mm_error = TWERR_TIMEOUT;
	}
	else
	{
		NutEnterCritical();
		if (dev->tw_mm_err)
		{
			dev->tw_mm_error = dev->tw_mm_err;
		}
		else
		{
			rc = dev->tw_mr_cnt;
		}
		NutExitCritical();
	}

	/* Release the interface. */
	NutEventPost(&dev->tw_mm_mutex);

	return rc; /* Dummy number of really received chars */
}
int TwMasterTransact(uint8_t sla, const void *txdata, uint16_t txlen, void *rxdata, uint16_t rxsiz,
		uint32_t tmo)
{
	return TwMasterCommon(sla, txdata, txlen, rxdata, rxsiz, tmo, 0);
}

int TwMasterRead(uint8_t sla, const void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz,
		uint32_t tmo)
{
	return TwMasterCommon(sla, addr, addrlen, rxdata, rxsiz, tmo, 0);
}

int TwMasterWrite(uint8_t sla, const void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz,
		uint32_t tmo)
{
	return TwMasterCommon(sla, addr, addrlen, txdata, txsiz, tmo, 1);
}

/*!
 * \brief Get last master mode error only from IIC1!!!.
 *
 * You may call this function to determine the specific cause
 * of an error after TwMasterTransact() failed.
 *
 * \note This function is only available on ATmega128 systems.
 *
 */
int TwMasterError(void)
{
	// TwMasterError receive only error from IIC1
	TWIDCB *dev = &dcb_twi[0];

	int rc = (int) dev->tw_mm_error;
	dev->tw_mm_error = 0;
	return rc;
}

/*!
 * \brief Perform TWI control functions.
 *
 * This function is only available on mcf5xxxx systems.
 *
 * \param req  Requested control function. May be set to one of the
 *	       following constants:
 *	       - TWI_SETSPEED + DCB_BASE_MASK_TWIn, if conf points to an uint32_t value speed in Hz.
 *	       - TWI_GETSPEED + DCB_BASE_MASK_TWIn, if conf points to an uint32_t value receiving the current speed in Hz.
 * \param conf Points to a buffer that contains any data required for
 *	       the given control function or receives data from that
 *	       function.
 * \return 0 on success, -1 otherwise.
 *
 * \note Timeout is limited to the granularity of the system timer.
 *
 */
int TwIOCtl(int req, void *p_conf)
{
	// chose which TWI control
	uint8_t dcbBase = DCB_BASE(req);
	req &= ~DCB_BASE_MASK;

#define IC_SIZE 64
	uint8_t rc = 0, ic = IC_SIZE - 1, i;

	uint16_t dividerTable[IC_SIZE] =
	{ 20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56, 68, 48, 56, 64, 72, 80, 88, 104,
			128, 80, 96, 112, 128, 144, 160, 192, 240, 160, 192, 224, 256, 288, 320, 384, 480, 320,
			384, 448, 512, 576, 640, 768, 960, 640, 768, 896, 1024, 1152, 1280, 1536, 1920, 1280,
			1536, 1792, 2048, 2304, 2560, 3072, 3840 };

	uint32_t *speedHz = (uint32_t *) p_conf;
	uint16_t selectedDivider = dividerTable[ic];		//maximal divider
	uint16_t countedDivider = NutGetCpuClock() / (*speedHz);

	switch (req)
	{

		case TWI_SETSPEED:
			for (i = 0; i < IC_SIZE; i++)
			{
				if (dividerTable[i] >= countedDivider && dividerTable[i] < selectedDivider)
				{
					selectedDivider = dividerTable[i];
					ic = i;
				}
			}
			MCF_IIC_FDR (dcbBase) = MCF_IIC_FDR_ICR(ic);
			break;
		case TWI_GETSPEED:
			selectedDivider = dividerTable[MCF_IIC_FDR(dcbBase)];
			*speedHz = NutGetCpuClock() / selectedDivider;
			break;
		default:
			rc = -1;
			break;
	}
	return rc;
}

/*!
 * \brief Initialize TWI interface.
 *
 * The specified slave address is used only, if the local system
 * is running as a slave. Anyway, care must be taken that it doesn't
 * conflict with another connected device.
 *
 * \note This function is only available on mcf5xxxx systems.
 *
 * \param sla Slave address contains 0x80 bit which decide which TWI(0-1)
 *            are used. Other 7-bit are slave address.
 */
int TwInit(uint8_t sla)
{
	TWIDCB *dev = &dcb_twi[DCB_BASE(sla)];
	dev->dcb_base = DCB_BASE(sla);
	dev->tw_if_bsy = 0; /* Clear busy flag */

	dev->InpLenM = 0U; /* No data to be received */

	uint32_t speed = 100000;

	/* Enable the IIC signals */
	if (dev->dcb_base == 0)
	{
		MCF_SCGC1 |= MCF_SCGC1_IIC1; // enable system clock
	}
	else
	{
		MCF_SCGC1 |= MCF_SCGC1_IIC2;
	}

	/* set the frequency near 100 000Hz, see MCF5QE128RM table for details */
	TwIOCtl(TWI_SETSPEED | (sla & DCB_BASE_MASK), &speed);
	TwIOCtl(TWI_GETSPEED | (sla & DCB_BASE_MASK), &speed);

	reenableDevice(dev->dcb_base);

	if (dev->dcb_base == 0)
	{
		NutRegisterIrqHandler(&sig_IIC1, TwInterrupt, dev);
		NutIrqEnable(&sig_IIC1);
	}
	else
	{
		NutRegisterIrqHandler(&sig_IIC2, TwInterrupt, dev);
		NutIrqEnable(&sig_IIC2);
	}

	/* Release the interface. */
	NutEventPost(&dev->tw_mm_mutex);

	return 0;
}

