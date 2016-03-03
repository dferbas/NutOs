/*
 * Copyright (C) 2008-2009 by egnite GmbH
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

#include <dev/board.h>
#include <cfg/spi.h>
#include <dev/gpio.h>
#include <dev/spibus.h>

#include <dev/irqreg.h>
#include <sys/event.h>
#include <sys/nutdebug.h>
#include <sys/atom.h>

#include <errno.h>
#include <stdlib.h>
#include <memdebug.h>

/*!
 * \addtogroup xgMcf51
 */
/*@{*/

#define SPI_CHANNEL		1

/*!
 * \brief Set the specified chip select to a given level.
 */
static int Mcf51Spi1ChipSelect(uint_fast8_t cs, uint_fast8_t hi)
{
    int rc = 0;

    switch (cs) {
    case NODE_SS1_PTE3:
        GpioPinSet(PORTE, 3, hi);
        GpioPinConfigSet(PORTE, 3, GPIO_CFG_OUTPUT);
        break;
    case NODE_SS1_PTB5:
    	GpioPinSet(PORTB, 5, hi);
    	GpioPinConfigSet(PORTB, 5, GPIO_CFG_OUTPUT);
    	break;
    default:
        errno = EIO;
        rc = -1;
        break;
    }
    return rc;
}

static uint8_t * volatile spi1_txp;
static uint8_t * volatile spi1_rxp;

static HANDLE spi1_que;
static volatile size_t spi1_xc;

/*!
 * \brief MCF51 SPI interrupt handler.
 */
static void Mcf51Spi1Interrupt(void *arg)
{
    uint8_t data;
    uint8_t status;

    status = MCF_SPI_S(SPI_CHANNEL);
    data = MCF_SPI_D(SPI_CHANNEL);

    if (status & MCF_SPI_S_SPRF)
    {
		if (spi1_xc)
		{
			if (spi1_rxp) {
				*spi1_rxp++ = data;
			}
			spi1_xc--;
			if (spi1_xc)
			{
				NUTASSERT(status & MCF_SPI_S_SPTEF);
				if (spi1_txp)
				{
					data = *spi1_txp++;
				} else
				{
					data = 0xFF; // write dummy char to generate clock for reading
				}
				MCF_SPI_D(SPI_CHANNEL) = data;
			} else
			{
				NutEventPostFromIrq(&spi1_que);
			}
		}
    }
}

/*!
 * \brief Transfer data on the SPI bus.
 *
 * A device must have been selected by calling Mcf51Spi1Select().
 *
 * Depending on the configuration, this routine implemets polling or
 * interrupt mode. For the latter either single or double buffering
 * may have been selected.
 *
 * When using double buffered interrupt mode, then the transfer may
 * be still in progress when returning from this function.
 *
 * \param node Specifies the SPI bus node.
 * \param txbuf Pointer to the transmit buffer. If NULL, undetermined
 *              byte values are transmitted.
 * \param rxbuf Pointer to the receive buffer. If NULL, then incoming
 *              data is discarded.
 * \param xlen  Number of bytes to transfer.
 *
 * \return Always 0.
 */
int Mcf51SpiBus1Transfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen)
{
    uint8_t data;

    /* Sanity check. */
    NUTASSERT(node != NULL);

    if (xlen) {
		spi1_txp = (uint8_t *) txbuf;
		spi1_rxp = (uint8_t *) rxbuf;
		spi1_xc = (size_t) xlen;
		if (spi1_txp) {
			data = *spi1_txp++;
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
		NutEventWait(&spi1_que, NUT_WAIT_INFINITE); /* if -1 on timeout, data are broken */

	}
    return 0;
}

/*!
 * \brief Initialize an SPI bus node.
 *
 * This routine is called for each SPI node, which is registered via
 * NutRegisterSpiDevice().
 *
 * \param node Specifies the SPI bus node.
 *
 * \return 0 on success or -1 if there is no valid chip select.
 */
int Mcf51SpiBus1NodeInit(NUTSPINODE * node)
{
    int rc;

    /* Sanity check. */
    NUTASSERT(node != NULL);

    /* Try to deactivate the node's chip select. */
    rc = Mcf51Spi1ChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) == 0);
    /* It should not hurt us being called more than once. Thus, we
       ** check wether any initialization had been taken place already. */
    if (rc == 0 && node->node_stat == NULL) {
        /* Allocate our shadow registers. */
        node->node_stat = malloc(sizeof(uint8_t));
        if (node->node_stat) {

        	MCF_SCGC2 |= MCF_SCGC2_SPI1; // enable system clock

			/* Init registers */
			(void) MCF_SPI_S(SPI_CHANNEL); 		/* Read the status register */
			(void) MCF_SPI_D(SPI_CHANNEL); 		/* Read the data register */
			/* Set the baud rate register, speed 1,25MHz */
			/* Set the baud rate register 0x72 for speed 375kHz */
			MCF_SPI_BR(SPI_CHANNEL) = 0x72; 	// TODO: When I set highest speed 12,5Mhz then it freezes if long reading or writing is complete.
			MCF_SPI_C2(SPI_CHANNEL) = 0x00; 	/* Configure the SPI port - control register 2 */
			MCF_SPI_C1(SPI_CHANNEL) = MCF_SPI_C1_MSTR; /* Master */

			/*
			 * SPI Interrupt Enable (for SPRF and MODF)
			 *  — This is the interrupt enable for SPI receive buffer full (SPRF) and mode fault (MODF) events.
			 *  SPI Transmit Interrupt Enable is not used
			 */
            /* Register and enable SPI interrupt handler. */
            NutRegisterIrqHandler(node->node_bus->bus_sig, Mcf51Spi1Interrupt, NULL);
			NutIrqEnable(node->node_bus->bus_sig);

			/* Enable device */
			MCF_SPI_C1(SPI_CHANNEL) |= MCF_SPI_C1_SPE;
        } else {
            /* Out of memory? */
            rc = -1;
        }
    }
    return rc;
} /* ????? */

/*!
 * \brief Select a device on the SPI bus.
 *
 * Locks and activates the bus for the specified node.
 *
 * \param node Specifies the SPI bus node.
 * \param tmo Timeout in milliseconds. To disable timeout, set this
 *            parameter to NUT_WAIT_INFINITE.
 *
 * \return 0 on success. In case of an error, -1 is returned and the bus
 *         is not locked.
 */
int Mcf51SpiBus1Select(NUTSPINODE * node, uint32_t tmo)
{
    int rc;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_stat != NULL);

    /* Allocate the bus. */
    rc = NutEventWait(&node->node_bus->bus_mutex, tmo);
    if (rc) {
        errno = EIO;
    } else {

        /* Finally activate the node's chip select. */
        rc = Mcf51Spi1ChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) != 0);
        if (rc) {
            /* Release the bus in case of an error. */
            NutEventPost(&node->node_bus->bus_mutex);
        }
    }
    return rc;
}

/*!
 * \brief Deselect a device on the SPI bus.
 *
 * Deactivates the chip select and unlocks the bus.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
int Mcf51SpiBus1Deselect(NUTSPINODE * node)
{
    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    /* Deactivate the node's chip select. */
    Mcf51Spi1ChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) == 0);

    /* Release the bus. */
    NutEventPost(&node->node_bus->bus_mutex);

    return 0;
}

/*!
 * \brief AVR SPI bus driver implementation structure.
 */
NUTSPIBUS spiBus1Mcf51 = {
    NULL,                       /*!< Bus mutex semaphore (bus_mutex). */
    NULL,                       /*!< Bus ready signal (bus_ready). */
    0,                          /*!< Unused bus base address (bus_base). */
    &sig_SPI1,                   /*!< Bus interrupt handler (bus_sig). */
    Mcf51SpiBus1NodeInit,         /*!< Initialize the bus (bus_initnode). */
    Mcf51SpiBus1Select,           /*!< Select the specified device (bus_alloc). */
    Mcf51SpiBus1Deselect,         /*!< Deselect the specified device (bus_release). */
    Mcf51SpiBus1Transfer,         /*!< Transfer data to and from a specified device (bus_transfer). */
    NutSpiBusWait,              /*!< Wait for bus transfer ready (bus_wait). */
    NutSpiBusSetMode,           /*!< Set SPI mode of a specified device (bus_set_mode). */
    NutSpiBusSetRate,           /*!< Set clock rate of a specified device (bus_set_rate). */
    NutSpiBusSetBits            /*!< Set number of data bits of a specified device (bus_set_bits). */
};
