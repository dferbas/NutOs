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

#include <arch/m68k.h>
#include <arch/m68k/coldfire/mcf51cn/spi_mcf51cn.h>
#include <cfg/spi.h>
#include <dev/irqreg.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/atom.h>
#include <sys/event.h>
#include <sys/mutex.h>
#include <sys/nutdebug.h>

static MUTEX 	spi_bus_mutex;		// waiting for spi bus
static HANDLE 	spi_transfer_handler;	// waiting for spi transfer finished

#define CHIP_SELECT_LO	0	// slave activated
#define CHIP_SELECT_HI	1	// slave deactivated

#define SPI_CHANNEL		2

/*!
 * \brief Set the specified chip select to a given level.
 */
static int Mcf51cnSpiChipSelect(uint_fast8_t cs, uint_fast8_t level)
{
    int rc = 0;

    switch (cs) {
    case NODE_CS_PTE2:
    	GpioPinSet(PORTE, 2, level);
    	break;
    case NODE_CS_PTB2:
        GpioPinSet(PORTB, 2, level);
        break;
    default:
        errno = EIO;
        rc = -1;
        break;
    }
    return rc;
}

/*! \brief Select a device on the first SPI bus.
 *
 * Locks and activates the bus for the specified node.
 *
 * \param node Specifies the SPI bus node.
 * \param tmo  Timeout in milliseconds. To disable timeout, set this
 *             parameter to NUT_WAIT_INFINITE.
 *
 * \return 0 on success. In case of an error, -1 is returned and the bus
 *         is not locked.
 */
int Mcf51cnSpiSelect(uint_fast8_t node_cs) //, uint32_t tmo)
{
    int rc;

    /* Allocate the bus. */
    NutMutexLock(&spi_bus_mutex); // if timeout need to set, use NutEventWait with spi_bus_handle
//    rc = NutEventWait(&spi_bus_mutex, tmo);
//    if (rc) {
//        errno = EIO;
//    } else {
        /* Enable SPI peripherals and clock. */

        /* Finally activate the node's chip select. */
        rc = Mcf51cnSpiChipSelect(node_cs, CHIP_SELECT_LO);
        if (rc) {
        	errno = ENXIO;
            /* Release the bus in case of an error. */
        	NutMutexUnlock(&spi_bus_mutex);
//            NutEventPost(&spi_bus_mutex);
        }
//    }
    return rc;

}

/*! \brief Deselect a device on the first SPI bus.
 *
 * Deactivates the chip select and unlocks the bus.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
int Mcf51cnSpiDeselect(uint_fast8_t node_cs)
{
    /* Deactivate the node's chip select. */
	Mcf51cnSpiChipSelect(node_cs, CHIP_SELECT_HI);

    /* Release the bus. */
	NutMutexUnlock(&spi_bus_mutex);
//    NutEventPost(&spi_bus_mutex);

    return 0;
}

static uint8_t * volatile spi0_txp;
static uint8_t * volatile spi0_rxp;
static volatile size_t spi0_xc;

static void Mcf51cnSpiInterrupt(void *arg)
{
    uint8_t data;
    uint8_t status;

    status = MCF_SPI_S(SPI_CHANNEL);
    data = MCF_SPI_D(SPI_CHANNEL);

    if (status & MCF_SPI_S_SPRF)
    {
		if (spi0_xc)
		{
			if (spi0_rxp) {
				*spi0_rxp++ = data;
			}
			spi0_xc--;
			if (spi0_xc)
			{
				NUTASSERT(status & MCF_SPI_S_SPTEF);
				if (spi0_txp)
				{
					data = *spi0_txp++;
				} else
				{
					data = 0xFF; // write dummy char to generate clock for reading
				}
				MCF_SPI_D(SPI_CHANNEL) = data;
			} else
			{
				NutEventPostFromIrq(&spi_transfer_handler);
			}
		}
    }
}

/*!
 * \brief Transfer data on the SPI bus using single buffered interrupt mode.
 *
 * A device must have been selected by calling Mcf51SpiSelect().
 *
 * \param txbuf Pointer to the transmit buffer. If NULL, undetermined
 *              byte values are transmitted.
 * \param rxbuf Pointer to the receive buffer. If NULL, then incoming
 *              data is discarded.
 * \param xlen  Number of bytes to transfer.
 *
 * \return Always 0, -1 on timeout.
 */
int Mcf51cnSpiTransfer(const void *txbuf, void *rxbuf, int xlen)
{
    uint8_t data;
    int rc;
    // Mutex .. spolecny s SpiSelect
    rc = NutMutexTrylock(&spi_bus_mutex);
    NUTASSERT(!rc);

    if (rc == 0 && xlen) {
        spi0_txp = (uint8_t *) txbuf;
        spi0_rxp = (uint8_t *) rxbuf;
        spi0_xc = (size_t) xlen;
        if (spi0_txp) {
            data = *spi0_txp++;
        }
        else {
        	data = 0xff;
        }
        /* Enable and kick interrupts. */
        NutEnterCritical();

        (void) MCF_SPI_S(SPI_CHANNEL); // read status register to clear flags
        MCF_SPI_D(SPI_CHANNEL) =  data; // start transmission

        NutExitCritical();

        /* Wait until transfer has finished. */
        rc = NutEventWait(&spi_transfer_handler, 5000); /* if -1 on timeout, data are broken */

    }

    NutMutexUnlock(&spi_bus_mutex);
    return rc;
}


/*!
 * \brief Initialize an SPI bus node.
 */
void Mcf51cnSpiInit(void)
{

#if	SPI_CHANNEL == 2
	MCF_SCGC2 |= MCF_SCGC2_SPI2;
    GpioPinConfigSet(PORTD, 7, GPIO_CFG_ALT2); // set pin assignment for SCK2
    GpioPinConfigSet(PORTE, 0, GPIO_CFG_ALT2); // set pin assignment for MISO2
    GpioPinConfigSet(PORTE, 1, GPIO_CFG_ALT2); // set pin assignment for MOSI2
#else
	MCF_SCGC2 |= MCF_SCGC2_SPI1;
    GpioPinConfigSet(PORTC, 7, GPIO_CFG_ALT2); // set pin assignment for SCK1
    GpioPinConfigSet(PORTC, 6, GPIO_CFG_ALT2); // set pin assignment for MISO1
    GpioPinConfigSet(PORTC, 5, GPIO_CFG_ALT2); // set pin assignment for MOSI1
#endif

//    NutEventPost(&spi_bus_mutex);
    NutMutexInit(&spi_bus_mutex);

	/* Init registers */
	(void) MCF_SPI_S(SPI_CHANNEL); 		/* Read the status register */
	(void) MCF_SPI_D(SPI_CHANNEL); 		/* Read the data register */
	/* Set the baud rate register 0x41 for speed 1,25MHz */
	/* Set the baud rate register 0x72 for speed 375kHz */
	MCF_SPI_BR(SPI_CHANNEL) = 0x72; 	// TODO: pokud zadam nejvyzsi rychlost 12,5Mhz tak to zamrzne po dokonceni dlouheho zapisu nebo cteni
	MCF_SPI_C2(SPI_CHANNEL) = 0x00; 	/* Configure the SPI port - control register 2 */
	MCF_SPI_C1(SPI_CHANNEL) = MCF_SPI_C1_MSTR; /* Master */

	/*
	 * SPI Interrupt Enable (for SPRF and MODF)
	 *  — This is the interrupt enable for SPI receive buffer full (SPRF) and mode fault (MODF) events.
	 *  SPI Transmit Interrupt Enable is not used
	 */
#if	SPI_CHANNEL == 2
    NutRegisterIrqHandler(&sig_SPI2, Mcf51cnSpiInterrupt, 0);
	NutIrqEnable(&sig_SPI2);
#else
	NutRegisterIrqHandler(&sig_SPI1, Mcf51cnSpiInterrupt, 0);
	NutIrqEnable(&sig_SPI1);
#endif

	/* Enable device */
	MCF_SPI_C1(SPI_CHANNEL) |= MCF_SPI_C1_SPE;

}

