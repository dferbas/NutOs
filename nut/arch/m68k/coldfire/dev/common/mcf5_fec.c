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
#include <stdio.h>
#include <errno.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <sys/heap.h>
#include <sys/timer.h>
#include <sys/nutdebug.h>
#include <net/ether.h>
#include <net/if.h>
#include <netinet/if_ether.h>
#include <dev/irqreg.h>
#include <dev/phy_generic.h>
#include <arch/m68k.h>
#include <dev/gpio.h>
#include <dev/phy.h>

/*!
 * \addtogroup xgMcfCommon
 */
/*@{*/

//#define DBG(...)
#define DBG	printf

#if defined (MCU_MCF5225)
# define TX_PACKET_ASSEMBLE		// SM2-MU defined
# define BD_IN_INTRAM			// SM2-MU defined
#endif

#ifdef TX_PACKET_ASSEMBLE
# ifdef BD_IN_INTRAM
#  define	TX_BUF_FREE		NutStackFree
# else
#  define	TX_BUF_FREE		NutHeapFree
# endif
#else
# define	TX_BUF_FREE		NutNetBufFree
#endif

#define ALIGN(x,pow)	(((x) + ((pow)-1)) & ~((pow)-1))
#define RX_BUFFER_ALIGNMENT   16
#define TX_BUFFER_ALIGNMENT   4
#define BD_ALIGNMENT          16 // may be 4.

#define MAX_FL						(ETHER_MAX_LEN)	// maximum frame length
#define PHY_RESET_TIMEOUT			200				// e.g. Power Up Stabilization of DP83848C takes 176ms
#define PHY_AUTONEGO_TIMEOUT		6000			// Parallel detection and Auto-Negotiation take approximately 2-3 seconds to complete. In addition, Auto-Negotiation with next page should take approximately 2-3 seconds to complete, depending on the number of next pages sent. Refer to Clause 28 of the IEEE 802.3u standard for a full description of the individual timers related to Auto-Negotiation.
#define PHY_LINK_TIMEOUT			3000			// e.g. using DP83848C takes 200ms after autonego "fails"
													// DP83848C: 1000 too small, 2000 sometimes does not work for 10 Mb
//#if defined (MCU_MCF5225)
//# define FEC_WAIT_FOR_LINK_TIMEOUT	10000		// if this macro is defined, the FEC will wait for link after startup
//#endif

#ifndef NUT_THREAD_NICRXSTACK
#if defined (MCU_MCF5225)
#  define NUT_THREAD_NICRXSTACK   	1000		// SM2-MU 1000
# else
#  define NUT_THREAD_NICRXSTACK   	660			// SM2-RM 660
# endif
#endif

#ifndef NIC_PHY_ADDR
# define NIC_PHY_ADDR	        	1
#endif

//#define NIC_PHY_UID 				0x20005C90 // MCF52259
//#define NIC_PHY_UID 				0x00221556 // MCF51CN

/*
 * Buffer descriptors config
 */
#ifdef TX_PACKET_ASSEMBLE
# if defined (MCU_MCF5225)
#  define FEC_TX_BUFFERS			(2*1 + 1)		// SM2-MU no fragments (one buffer only)
# else
#  define FEC_TX_BUFFERS			(1*1 + 1)		// no fragments (one buffer only)
# endif
#else
# define FEC_TX_BUFFERS				(2*4 + 1)		// 2*4 netbuf fragments
#endif
#define FEC_RX_BUFSIZ				256				// to minimize bus utilization (descriptor fetches), set this field to 256 or larger.
// DCH TODO otestovat zmenseni pameti na 2 ze 4
// INFO: nesmi se stat, ze se zaplni vsechny BD jednim paketem, proto musi byt vzdy o jeden RxBD vic
// TODO: muze se stat, ze kvuli chybe na siti prijde delsi paket? .. Osetruje to nejak HW?
#if defined (MCU_MCF5225)
# define FEC_RX_BUFFERS				(4 * ((ETHER_MAX_LEN/FEC_RX_BUFSIZ) + 1)) // SM2-MU
#else
# define FEC_RX_BUFFERS				((1 * ((ETHER_MAX_LEN/FEC_RX_BUFSIZ) + 1)) + 1) // SM2-RM
#endif


#if (FEC_TX_BUFFERS < FEC_TX_BUFFERS_MIN)
# error("FEC_TX_BUFFERS must be grater or equal to FEC_TX_BUFFERS_MIN")
#endif

#if ((FEC_RX_BUFFERS * FEC_RX_BUFSIZ) < MAX_FL)
# error("FEC_RX_BUFFERS or FEC_RX_BUFSIZ too small")
#endif

// Switch on red led
uint8_t alarmFec;

uint8_t resetPHY = 0;

static TMWDTSetVariableFN	EthMWDTSetVariableFN = NULL;
/*
 * Minimal number of free BDs required for transmitting maximal packet.
 * One free BD must left all times - HW requirement (more info in datasheet)
 */
#ifdef TX_PACKET_ASSEMBLE
# define FEC_TX_BUFFERS_MIN		(1 + 1)
#else
# define FEC_TX_BUFFERS_MIN		(4 + 1)			// 4 netbuf fragments
#endif

/*
 * TX Queue limits helps to prevent heap exhaustion
 */
#define FEC_TX_QUEUE_MAX_SIZE	(4 * ETHER_MAX_LEN)				// maximum size of all packets waiting in tx queue
#define FEC_TX_QUEUE_TIMEOUT	500								// maximum time [ms] the packet spent in tx queue


/* FEC buffer descriptor */
typedef volatile struct {
   uint16_t	flags;
   uint16_t	length;		// buffer length
   char		*buffer;	// Rx/Tx data buffer pointer
} FECBD;

/*!
 * \brief Network interface controller information structure.
 */
typedef struct {
	// NOTE: rx_bd_ring, tx_bd_ring and rx_buffers MUST be first (or 128 bit aligned)
    FECBD	rx_bd_fec[FEC_RX_BUFFERS];
    FECBD	tx_bd_fec[FEC_TX_BUFFERS];

    char	*rx_buf_unaligned;
    char	*rx_buf[FEC_RX_BUFFERS];

#ifdef TX_PACKET_ASSEMBLE
    char	*tx_buf[FEC_TX_BUFFERS];
#else
    NETBUF	*tx_buf[FEC_TX_BUFFERS];
#endif

    int		rx_pos_frame_start;
    int		rx_pos;

    int		tx_pos_push;
    int		tx_pos_pop;
    int		tx_bd_free;

    int		initialized;

    HANDLE volatile rx_rdy;  			/*!< Receiver event queue. */
    HANDLE volatile tx_rdy; 			/*!< Transmitter event queue. */
    HANDLE volatile tx_mutex;			/*!< Transmitter mutex. */
    HANDLE volatile ioctl_rdy;			/*!< IOCTL event queue. */

    uint32_t	tx_queue_size;

#ifdef NUT_PERFMON
    uint32_t ni_rx_packets;       /*!< Number of packets received. */
    // TODO
#endif
} FECINFO;

#define INC_RX_BD_POS(pos) (pos = ((pos) + 1) % FEC_RX_BUFFERS)
#define INC_TX_BD_POS(pos) (pos = ((pos) + 1) % FEC_TX_BUFFERS)
#define DEC_RX_BD_POS(pos) (pos = ((pos) - 1) % FEC_RX_BUFFERS)
#define DEC_TX_BD_POS(pos) (pos = ((pos) - 1) % FEC_TX_BUFFERS)

/* --------------------------------------------------------------------------
 * PHY & FEC Configuration Handling Functions
 * -------------------------------------------------------------------------- */
static uint32_t phy_addr = NIC_PHY_ADDR;

static uint16_t PhyRead(uint8_t reg_addr)
{
    /* Clear the MII interrupt bit */
    MCF_FEC_EIR = MCF_FEC_EIR_MII;

    /* Write to the MII Management Frame Register to kick-off the MII read */
    MCF_FEC_MMFR = 0
        | MCF_FEC_MMFR_ST_01
        | MCF_FEC_MMFR_OP_READ
        | MCF_FEC_MMFR_PA(phy_addr)
        | MCF_FEC_MMFR_RA(reg_addr)
        | MCF_FEC_MMFR_TA_10;

    /* Poll for the MII interrupt */
    while (!(MCF_FEC_EIR & MCF_FEC_EIR_MII))
    	;

    /* Clear the MII interrupt bit */
    MCF_FEC_EIR = MCF_FEC_EIR_MII;

    return (uint16_t)(MCF_FEC_MMFR & 0x0000FFFF);
}

static void PhyWrite(uint8_t reg_addr, uint16_t data)
{
    /* Clear the MII interrupt bit */
    MCF_FEC_EIR = MCF_FEC_EIR_MII;

    /* Write to the MII Management Frame Register to kick-off the MII write */
    MCF_FEC_MMFR = 0
        | MCF_FEC_MMFR_ST_01
        | MCF_FEC_MMFR_OP_WRITE
        | MCF_FEC_MMFR_PA(phy_addr)
        | MCF_FEC_MMFR_RA(reg_addr)
        | MCF_FEC_MMFR_TA_10
        | MCF_FEC_MMFR_DATA(data);

    /* Poll for the MII interrupt (interrupt should be masked) */
    while (!(MCF_FEC_EIR & MCF_FEC_EIR_MII))
    	;

    /* Clear the MII interrupt bit */
    MCF_FEC_EIR = MCF_FEC_EIR_MII;
}

/*! \brief Configure hardware pins, wait for end of hw reset.
 *
 * Do this only once after device registration.
 */
static inline void FecPowerUp(void)
{
#if defined (MCU_MCF5225)
	/* Configure MII port for MCF52259 GPIO: */
//	GpioPortConfigSet(PORTTI, 0xFF, GPIO_CFG_PERIPHERAL0);	// primary function
//	GpioPortConfigSet(PORTTJ, 0xFF, GPIO_CFG_PERIPHERAL0);	// primary function
//	GpioPortConfigSet(PORTNQ, 0x28, GPIO_CFG_PERIPHERAL1);	// FEC_MDIO + FEC_MDC (alt 1)
	MCF_GPIO_PTIPAR = 0xFF;
	MCF_GPIO_PTJPAR = 0xFF;
	MCF_GPIO_PNQPAR = (MCF_GPIO_PNQPAR & ~((3 << 6) | (3 << 10))) | (MCF_GPIO_PNQPAR_IRQ3_FEC_MDIO | MCF_GPIO_PNQPAR_IRQ5_FEC_MDC);
#else // MCU_MCF51CN
	/* Set Reset Phy pin to output */
	GpioPinSetHigh(PORTC, 3);
	GpioPinConfigSet(PORTC, 3, GPIO_CFG_ALT1 | GPIO_CFG_OUTPUT);

	MCF_SOPT3 = MCF_SOPT3_PCS_OSCOUT << MCF_SOPT3_PCS_BITNUM;

	/* MII_TX_ER pin P0RB2 used as gpio chip select in SM2-RM SPI FRAM, for ETH_mod not used*/
	GpioPortConfigSet(PORTB, 0xFB, GPIO_CFG_ALT1);
	GpioPortConfigSet(PORTC, 0x07, GPIO_CFG_ALT1);
	/* Configure MII port for MCF51CN */
	GpioPortConfigSet(PORTA, 0xFF, GPIO_CFG_ALT1);

	/* Waiting for charge condenser on PHY reset. It takes 50ms charge to 1,85V. See printscreen from oscilloscope */
	NutSleep(50);
#endif
}

static void ConfigureDuplex(int duplex)
{
	if (duplex)
	{
		/* Enable full duplex & disable receive on transmit */
		MCF_FEC_TCR |= MCF_FEC_TCR_FDEN;
		//MCF_FEC_RCR &= ~MCF_FEC_RCR_DRT;				//default after FEC reset
	}
	else
	{
		/* Disable full duplex & enable receive on transmit */
		MCF_FEC_TCR &= ~MCF_FEC_TCR_FDEN;
		MCF_FEC_RCR |= MCF_FEC_RCR_DRT;
	}
}

/*! \brief Reset the Ethernet controller and configure according to requested flags.
 *
 * Do this every time, new configuration is applied.
 *
 * NutPhyCtl value parameter should fit into uin16_t (see casting in NutPhyCtl).
 *
 * \return 0 on success, -1 otherwise.
 */
static int FecConfigure(IFNET *nif, int do_reset)
{
	int rc = 0;
	int do_linktest = 0;
	uint32_t regvalue, regvalue_lpbck;

	/* ECR[ETHER_EN] is cleared (initialization time) */
	MCF_FEC_ECR = 0;

	/* Reset the controller & wait for the reset sequence to complete */
	MCF_FEC_ECR = MCF_FEC_ECR_RESET;
	NutMicroDelay(1);	// The reset sequence takes approximately eight internal bus clock cycles
	if (MCF_FEC_ECR & MCF_FEC_ECR_RESET)
	{
		DBG("FEC: reset failure\n");
		errno = EIO;
		return -1;
	}

	/* Setup MII speed */
#if defined (MCU_MCF5225)
	MCF_FEC_MSCR = MCF_FEC_MSCR_MII_SPEED(0x08);		//Internal FEC freq = 2,5 MHz, MCF52259 set 0x08
#else //if defined (MCU_MCF51CN)
	MCF_FEC_MSCR = MCF_FEC_MSCR_MII_SPEED(0x05);		//Internal FEC freq = 2,5 MHz, MCF51CN set 0x05
#endif

	/* Set MII mode */
	MCF_FEC_RCR |= MCF_FEC_RCR_MII_MODE;

	/* Set promiscuous mode */
	if (nif->if_flags & IFF_PROMISC)
	{
		/* Receive all packets enable. */
		MCF_FEC_RCR |= MCF_FEC_RCR_PROM;
	}
	else
	{
		/* Receive all packets disable. */
		MCF_FEC_RCR &= ~MCF_FEC_RCR_PROM;				//default after FEC reset
	}

	/* Set FEC loopback */
	if (nif->if_flags & IFF_LOOPBACK_MAC)
	{
		//DBG("FEC: loopback = ON\n");

		/* Enable MAC loopback */
		MCF_FEC_RCR |= MCF_FEC_RCR_LOOP;				//default after FEC reset

		/* Enable full duplex */
		ConfigureDuplex(1);

		return 0;
	}

	/* Disable loopback */
	MCF_FEC_RCR &= ~MCF_FEC_RCR_LOOP;

	/* Register PHY - this must be called after FEC is reset (does not work if called from FecPowerUp) */
	NutRegisterPhy(1, PhyWrite, PhyRead);

#if defined (MCU_MCF51CN)
	/* If PHY does not respond, restart PHY. It can happen that PHY is not properly started (when power source is applied and voltage oscillates). */
	if (PhyRead(PHY_REG_BMSR) == 0xFFFF)
	{
		int wait;

		DBG("PHY RESTART\n");
		GpioPinSetLow(PORTC, 3);
		NutSleep(10); 				// PHY reset time 10ms
		GpioPinSetHigh(PORTC, 3);

		for (wait = 25;; wait--)
		{
			// wait until PHY starts
			NutSleep(100);
			if (PhyRead(PHY_REG_BMSR) != 0xFFFF)
			{
				break;
			}
			if (wait == 0)
			{
				DBG("PHY NOT STARTED!\n");
				return -1;
			}
		}
	}
#endif

	/* Activate reset, wait for completion. */
	if (do_reset)
	{
		regvalue = 1;
		rc = NutPhyCtl(PHY_CTL_RESET, &regvalue);
		//TODO: check if error
	}

	/* Enable power down mode. */
#if 0	//not implemented in phy.c
	//regvalue = 1;
	rc = NutPhyCtl(PHY_CTL_POWERSAVE, &regvalue);
	//TODO: "do phy.c"
#define PHY_CTR2        0x1F    /* PHY Control 2 */
#define PHY_BMCR_PSAV   0x0400  /* 1: Power Saving Enabled. */
	uint16_t bmcr2 = phyr(PHY_CTR2);
	bmcr2 |= PHY_BMCR_PSAV;
	phyw( PHY_CTR2, bmcr2);
#else
#define PHY_CTR2        0x1F    /* PHY Control 2 */
#define PHY_BMCR_PSAV   0x0400  /* 1: Power Saving Enabled. */
	uint16_t bmcr2 = PhyRead(PHY_CTR2);
	bmcr2 |= PHY_BMCR_PSAV;
	PhyWrite(PHY_CTR2, bmcr2);
#endif

	/* Disable Loopback if driver changed from loopback phy mode */
	regvalue_lpbck = (nif->if_flags & IFF_LOOPBACK_PHY) != 0;
	(void)NutPhyCtl(PHY_CTL_LOOPBACK, &regvalue_lpbck);

	/* Set autonego */
	regvalue = (nif->if_flags & IFF_AUTONEGO_ENABLE) != 0 && !regvalue_lpbck;
	(void)NutPhyCtl(PHY_CTL_AUTONEG, &regvalue);

	/* if autonego enabled and loopback disabled, do autonego */
	if (regvalue)
	{
		/* Restart auto negotiation. */
		rc = NutPhyCtl(PHY_CTL_AUTONEG_RE, &phy_addr); //PhyProbe();

		/* Wait for auto negotiation completed and link established. */
		uint32_t timeout = NutGetMillis() + PHY_AUTONEGO_TIMEOUT;
		while (1)
		{
			(void)NutPhyCtl(PHY_GET_STATUS, &regvalue);
			if (regvalue & PHY_STATUS_AUTONEG_OK)
			{
				do_linktest = !(regvalue & PHY_STATUS_HAS_LINK);
				break;
			}

			if (NutGetMillis() > timeout)
			{
				DBG("NO AUTONEGO!\n");
				errno = ENETDOWN;
				return -1;
			}
			NutThreadYield();
		}

		ConfigureDuplex(regvalue & PHY_STATUS_FULLDUPLEX);
	}
	else
	{
		/* No autonego or PHY loopback */

		/* Set duplex */
		regvalue = (nif->if_flags & IFF_FULL_DUPLEX) != 0;
		(void)NutPhyCtl(PHY_CTL_DUPLEX, &regvalue);
		ConfigureDuplex(regvalue);

		/* Set Phy speed */
		if (nif->if_flags & IFF_SPEED_100)
		{
			regvalue = 100;
		}
		else
		{
			/* 10 megabit */
			regvalue = 10;
		}
		(void)NutPhyCtl(PHY_CTL_SPEED, &regvalue);

		/* Do link test only for non loopback configurations. */
		do_linktest = !regvalue_lpbck;
	}

	if (do_linktest)
	{
		/* Wait for link established. */
		uint32_t timeout = NutGetMillis() + PHY_LINK_TIMEOUT;
		while (1)
		{
			(void)NutPhyCtl(PHY_GET_STATUS, &regvalue);
			if (regvalue & PHY_STATUS_HAS_LINK)
			{
				break;
			}

			if (NutGetMillis() > timeout)
			{
				DBG("NO LINK!\n");
				errno = ENETDOWN;
				return -1;
			}
			NutThreadYield();
		}
	}

	/* Now in all cases regvalue contains Phy status */
	if (regvalue & PHY_STATUS_HAS_LINK)
		nif->if_flags |= IFF_LINK0;
	else
		nif->if_flags &= ~IFF_LINK0;

	return rc;
}

/*!
 * \brief Link detection.
 *
 * Polls PHY to get link state.
 *
 * \return	PHY status
 */
int PhyLinkTest(NUTDEVICE *dev)
{
	static uint32_t last_status = 0xffffffff;
	IFNET *nif = (IFNET *)dev->dev_icb;
	uint32_t status;

	NutPhyCtl(PHY_GET_STATUS, &status);

	if (status == last_status)
	{
		// no state change
		return last_status;
	}
	last_status = status;

	if (status & PHY_STATUS_HAS_LINK)
	{
		/* Link is now on. */
		if (status & PHY_STATUS_FULLDUPLEX)
		{
			/* Enable full duplex & disable receive on transmit */
			MCF_FEC_TCR |= MCF_FEC_TCR_FDEN;
			MCF_FEC_RCR &= ~MCF_FEC_RCR_DRT;
		}
		else
		{
			/* Disable full duplex & receive on transmit */
			MCF_FEC_TCR &= ~MCF_FEC_TCR_FDEN;
			//TODO: both Rx and Tx are transmitted on same wire pair?
			MCF_FEC_RCR |= MCF_FEC_RCR_DRT;
		}

		// reflect state into flags
		nif->if_flags |= IFF_LINK0;
	}
	else
	{
		/* Link is now off. */
		nif->if_flags &= ~IFF_LINK0;
	}

//	printf("%s, %s, %s, %x\r\n", 	value & (1 << 0) ? "Link ON" : "Link Off",
//		   							value & (1 << 1) ? "Speed 10" : "Speed 100",
//									value & (1 << 2) ? "Full Duplex" : "Half Duplex",
//									value );

	return last_status;
}

#if defined (MCU_MCF51CN)
int Mcf5FecIsLinkedUp(void)
{
	uint16_t reg_val;
	reg_val = PhyRead(PHY_REG_BMSR);
	if (reg_val == 0xFFFF)
		return 0; /* PHY does not respond, no link */
	else
		return reg_val & PHY_REG_BMSR_LINK_STATUS;
}
#endif

/*!
 * \brief Buffer descriptors (BD) init.
 */
static int FecBdInit(FECINFO *ni)
{
	FECBD		*bd;
	int			i;

	/* Allocate alligned Rx buffers */
	if (!ni->rx_buf_unaligned)
	{
#ifdef BD_IN_INTRAM
		ni->rx_buf_unaligned = NutStackAlloc(FEC_RX_BUFFERS * FEC_RX_BUFSIZ + 16);
#else
		ni->rx_buf_unaligned = NutHeapAlloc(FEC_RX_BUFFERS * FEC_RX_BUFSIZ + 16);
#endif
		if (!ni->rx_buf_unaligned)
			return -1;

		ni->rx_buf[0] = (char *)ALIGN((int)ni->rx_buf_unaligned, RX_BUFFER_ALIGNMENT);

		for (i = 1; i < FEC_RX_BUFFERS; i++)
			ni->rx_buf[i] = ni->rx_buf[0] + i * FEC_RX_BUFSIZ;
	}

	/* Initialize Rx buffer descriptors */
	for (i = 0; i < FEC_RX_BUFFERS; i++)
	{
		bd = &ni->rx_bd_fec[i];
		bd->buffer = ni->rx_buf[i];
		bd->length = 0;
		bd->flags = MCF_FEC_RX_BD_EMPTY;
	}
	/* Set wrap bit in the last bd */
		bd->flags |= MCF_FEC_RX_BD_WRAP;

	/* Initialize Tx buffer descriptors */
	for (i = 0; i < FEC_TX_BUFFERS; i++)
	{
		bd = &ni->tx_bd_fec[i];
		bd->length = 0;
		bd->flags = 0;
	}
		bd->flags |= MCF_FEC_TX_BD_WRAP;

	/*  Erase TX buffers (NETBUF) if present (required only when re-initializing) */
	for (i = 0; i < FEC_TX_BUFFERS; i++)
	{
		if (ni->tx_buf[i])
		{
			TX_BUF_FREE(ni->tx_buf[i]);
			ni->tx_buf[i] = NULL;
		}
	}

	/* Initialize positions & counters*/
	ni->rx_pos = ni->rx_pos_frame_start = 0;
	ni->tx_pos_push = ni->tx_pos_pop = 0;
	ni->tx_bd_free = FEC_TX_BUFFERS;

	return 0;
}

/*!
 * \brief Start FEC peripheral.
 *
 * Do this after successful PHY configuration
 */
static int FecStart(FECINFO *ni, uint8_t *mac)
{
	/* Set MAC address */
	MCF_FEC_PALR = MCF_FEC_PALR_PADDR1(*(uint32_t *)(mac + 0));
	MCF_FEC_PAUR = MCF_FEC_PAUR_PADDR2(*(uint16_t *)(mac + 4));

	/* Increase fx FIFO watermark */
	MCF_FEC_TFWR = MCF_FEC_TFWR_X_WMRK_64; //MCF_FEC_TFWR_X_WMRK_192

	/* Clear the individual hash table registers */
	MCF_FEC_IAUR = 0;
	MCF_FEC_IALR = 0;

	/* Clear the group hash table registers */
	MCF_FEC_GAUR = 0;
	MCF_FEC_GALR = 0;

	//TODO - otestovat, toto pridano 17.2. do nut 5.0. Pak se prijimaly vsechny pakety!
	/* Enable all packets receiving */
//	MCF_FEC_RCR |= MCF_FEC_RCR_PROM;

	/* Maximum size of frame */
	MCF_FEC_RCR &= ~MCF_FEC_RCR_MAX_FL(0xFFFFFFFF);
	MCF_FEC_RCR |=  MCF_FEC_RCR_MAX_FL(MAX_FL);

	/* Maximum size of all receive buffers */
	MCF_FEC_EMRBR = FEC_RX_BUFSIZ;

	/* Initialize buffer descriptors */
	if (FecBdInit(ni))
		return -1;

	/* Receive Descriptor Ring Start Register point to the start of the circular Rx buffer descriptor queue */
	MCF_FEC_ERDSR = (uint32_t) ni->rx_bd_fec;

	/* Transmit Buffer Descriptor Ring Start Registers point to the start of the circular Tx buffer descriptor queue */
	MCF_FEC_ETSDR = (uint32_t) ni->tx_bd_fec;

	/* Clear MIB Counters */
/*	MCF_FEC_MIBC |= MCF_FEC_MIBC_MIB_DISABLE;
	memset((void *)(0x40001200), 0x00000000, 0xFF);
	MCF_FEC_MIBC &= ~MCF_FEC_MIBC_MIB_DISABLE;*/

	/* Enable interrupts */
	NutIrqEnable(&sig_FEC_RF);
	NutIrqEnable(&sig_FEC_TF);

	/* Enable Fec */
	MCF_FEC_ECR = MCF_FEC_ECR_ETHER_EN;

	/* Start receiving */
	MCF_FEC_RDAR = MCF_FEC_RDAR_R_DES_ACTIVE;

	/* Start/Enable transmitting */
	NutEventPostAsync(&ni->tx_mutex);

	//DBG("FEC: I'm UP\n");
    return 0;
}


/* --------------------------------------------------------------------------
 * Buffer Handling Functions
 * -------------------------------------------------------------------------- */

/*!
 * \brief FEC Buffer I/O functions
 */
static void FecTxBdCleanup(FECINFO * ni)
{
	for ( ; ni->tx_pos_pop != ni->tx_pos_push; INC_TX_BD_POS(ni->tx_pos_pop), ni->tx_bd_free++)
	{
		FECBD	*bd = &ni->tx_bd_fec[ni->tx_pos_pop];

		if (bd->flags & MCF_FEC_TX_BD_READY)
			return;

		if (bd->flags & MCF_FEC_TX_BD_LAST_IN_FRAME)
		{
			/* last frame fragment was processed, free frame's NETBUF */
			NUTASSERT(ni->tx_buf[ni->tx_pos_pop] != NULL);
			TX_BUF_FREE(ni->tx_buf[ni->tx_pos_pop]);
			ni->tx_buf[ni->tx_pos_pop] = NULL;
		}
	}
}

static void FecRxFrameReceive(FECINFO * ni, uint8_t *buffer, int frame_length)
{
	int 	length;
	FECBD	*bd;

	for (; ni->rx_pos_frame_start != ni->rx_pos; INC_RX_BD_POS(ni->rx_pos_frame_start))
	{
		bd = &ni->rx_bd_fec[ni->rx_pos_frame_start];

		if (buffer)
		{
			if (bd->flags & MCF_FEC_RX_BD_LAST_IN_FRAME)
				length = frame_length;
			else
				length = bd->length;

			if (length > 0)
			{
				memcpy(buffer, bd->buffer, length);
				buffer += length;
				frame_length -= length;
			}
			else
			{
				/*
				 * This occurs when terminating CRC does not fit the rx buffer.
				 * Example:
				 *  - FEC_RX_BUFSIZ = 256;
				 *  - frame_length = 509
				 *  - in this case, 3 buffer descriptors are used, because 509 + 4 (= sizeof(crc)) does not fit into two buffer descriptors
				 */
				NUTASSERT(length >= -3);
			}
		}
		memset(bd->buffer, 0, FEC_RX_BUFSIZ);

		bd->flags &= MCF_FEC_RX_BD_WRAP;
		bd->flags |= MCF_FEC_RX_BD_EMPTY;
	}

	/* Notify FEC there is are new empty BDs */
	MCF_FEC_RDAR = MCF_FEC_RDAR_R_DES_ACTIVE;
}

static inline void FecRxFrameErase(FECINFO * ni)
{
	FecRxFrameReceive(ni, NULL, 0);
}

/*!
 * \brief Fetch the next packet out of the receive buffers.
 *
 * \return 0 on success, -1 otherwise.
 */
static int FecGetPacket(FECINFO * ni, NETBUF ** nbp)
{
	FECBD		*bd_curr;
	uint16_t	bd_flags;
	int			frame_length;

	while(1)
	{
		bd_curr = &ni->rx_bd_fec[ni->rx_pos];
		bd_flags = bd_curr->flags;

		/* Check if there is some frame part waiting for processing */
		if (bd_flags & MCF_FEC_RX_BD_EMPTY)
			return -1;

		/* Process BD */
		INC_RX_BD_POS(ni->rx_pos);

		/* Check if frame part is valid */
		if (bd_flags & MCF_FEC_RX_BD_TRUNCATED)
		{
			FecRxFrameErase(ni);
			continue;
		}

		/* Check if this is the last frame part */
		if (!(bd_flags & MCF_FEC_RX_BD_LAST_IN_FRAME))
			continue;

		/* Check if the frame is valid */
		if (bd_flags & (MCF_FEC_RX_BD_LENGTH_VIOLATION |
						MCF_FEC_RX_BD_NON_OCTET_ALIGNED |
						MCF_FEC_RX_BD_RX_CRC_ERROR |
						MCF_FEC_RX_BD_OVERRUN))
		{
			FecRxFrameErase(ni);
			continue;
		}

		/*
		 * Allocate NETBUF and copy received data into them.
		 *
		 * NOTE: I tried to remove this data copying but I did not succeed.
		 *
		 *       Nut/OS uses NETBUF structure. After it finishes packet processing,
		 *       it calls NutNetBufFree() so it does NOT returns the NETBUF to FEC driver.
		 *
		 *       There is an idea to allocate NETBUFs in FEC driver using NutNetBufAlloc,
		 *       use them as buffers in buffer descriptors, let the PHY to fill them
		 *       and pass them up to the Nut/OS without any packet copying.
		 *
		 *       Unfortunatelly this idea is bad, becauese RX buffers must
		 *       be 128 bit aligned (16) so we cannot create them dynamically using
		 *       NutNetBufAlloc() function.
		 */

		frame_length = bd_curr->length - 4;	// remove checksum

		if ((*nbp = NutNetBufAlloc(NULL, NBAF_DATALINK, frame_length)) == NULL)
		{
			FecRxFrameErase(ni);
			continue;
		}

		/* Copy fragments to the big buffer */
		FecRxFrameReceive(ni, (*nbp)->nb_dl.vp, frame_length);

		return 0;
	}

	return -1;
}

/*!
 * \brief Load a packet into the transmit ring buffer.
 */
#ifdef TX_PACKET_ASSEMBLE
static void FecPutPacket(FECINFO * ni, char *p_packet, int size)
{
	FECBD			*bd;

	NUTASSERT(size >= FEC_TX_BUFFERS_MIN);

	//if (size & 1) //DF test - frame with odd and even size should work in the same way
	//	size++;

	/* Get first empty buffer descriptor */
	bd = &ni->tx_bd_fec[ni->tx_pos_push];

	/* Save netbuf pointer. The packet's memory will be released after send. */
	ni->tx_buf[ni->tx_pos_push] = p_packet;

	/* Set first BD position for next frame */
	INC_TX_BD_POS(ni->tx_pos_push);
	ni->tx_bd_free--;

	/* Fill buffer descriptor */
	bd->length = size;
	bd->buffer = p_packet;
	bd->flags &= MCF_FEC_TX_BD_WRAP;
	bd->flags |= MCF_FEC_TX_BD_READY | MCF_FEC_TX_BD_LAST_IN_FRAME | MCF_FEC_TX_BD_TRANSMIT_CRC;

	/* Notify FEC about new READY BDs */
	MCF_FEC_TDAR |= MCF_FEC_TDAR_X_DES_ACTIVE;
}
#else

static void FecPutPacketNetbuf(FECINFO * ni, NETBUF *nb)
{
	struct _NBDATA	*fragment = &nb->nb_dl;
	FECBD			*bd_first = &ni->tx_bd_fec[ni->tx_pos_push];;
	FECBD			*bd;
	int				frame_size = 0;

	NUTASSERT(ni->tx_bd_free >= FEC_TX_BUFFERS_MIN);

    while(1)
    {
    	bd = &ni->tx_bd_fec[ni->tx_pos_push];

    	frame_size += fragment->sz;
		bd->length = fragment->sz;
		bd->buffer = fragment->vp;
		bd->flags &= MCF_FEC_TX_BD_WRAP;

		/* Do not set READY flag in the first BD. It will be set below as a last step. */
		if (bd != bd_first)
			bd->flags |= MCF_FEC_TX_BD_READY;

		if ((fragment == &nb->nb_ap) || !(fragment + 1)->sz)
		{
			/* Set last-in-frame flags */
			bd->flags |= MCF_FEC_TX_BD_LAST_IN_FRAME | MCF_FEC_TX_BD_TRANSMIT_CRC;

		    /* Align frame size due to CRC alignment */

			//if (frame_size & 1) //DF test - frame with odd and even size should work in the same way
		    //	bd->length++;

		    /* Save netbuf pointer. It will be freed after send. */
			ni->tx_buf[ni->tx_pos_push] = nb;

			/* Set first BD position for next frame */
			INC_TX_BD_POS(ni->tx_pos_push);
			ni->tx_bd_free--;

			/* Set the first BD READY as a last step */
			bd_first->flags |= MCF_FEC_TX_BD_READY;

			/* Notify FEC about new READY BDs */
			MCF_FEC_TDAR |= MCF_FEC_TDAR_X_DES_ACTIVE;

			break;
		}

		fragment++;
		INC_TX_BD_POS(ni->tx_pos_push);
		ni->tx_bd_free--;
    }
}
#endif

/*!
 * \brief Send Ethernet packet.
 *
 * \param dev 	Identifies the device to use.
 * \param nb  	Network buffer structure containing the packet to be sent.
 *            	The structure must have been allocated by a previous
 *            	call NutNetBufAlloc().
 *
 * \return 0 on success, -1 in case of any errors.
 */
static int FecOutput(NUTDEVICE * dev, NETBUF * nb)
{
	FECINFO 	*ni = (FECINFO *) dev->dev_dcb;
	uint32_t	size;
#ifdef TX_PACKET_ASSEMBLE
	char		*p_packet;
#else
	NETBUF		*nb_send;
#endif

	/* Drop packet if interface is not UP */
	if (!ni->initialized)
	{
		errno = ENETDOWN;
		return -1;
	}

    /* Check max packet size */
	size = nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz;
    if (size > ETHERMTU)
    {
    	errno = EMSGSIZE;
        return -1;
    }

	/* Check if there is enough space in tx queue (Heap Guard) */
	size += nb->nb_dl.sz + sizeof(NETBUF);
	if (ni->tx_queue_size + size > FEC_TX_QUEUE_MAX_SIZE)
	{
		errno = ENOBUFS;
		return -1;
	}

	/* Wait for tx mutex */
	if (NutEventWait(&ni->tx_mutex, FEC_TX_QUEUE_TIMEOUT))
	{
		errno = EBUSY;
		return -1;	// timeout
	}

#ifdef TX_PACKET_ASSEMBLE
#ifdef BD_IN_INTRAM
	if ((p_packet = NutStackAlloc(size)) == NULL)
#else
	if ((p_packet = NutHeapAlloc(size)) == NULL)
#endif
	{
		/* Unlock tx mutex */
		NutEventPost(&ni->tx_mutex);
		errno = ENOMEM;
		return -1;
	}

	size = 0;
	memcpy(&p_packet[size], nb->nb_dl.vp, nb->nb_dl.sz); size += nb->nb_dl.sz;
	memcpy(&p_packet[size], nb->nb_nw.vp, nb->nb_nw.sz); size += nb->nb_nw.sz;
	memcpy(&p_packet[size], nb->nb_tp.vp, nb->nb_tp.sz); size += nb->nb_tp.sz;
	memcpy(&p_packet[size], nb->nb_ap.vp, nb->nb_ap.sz); size += nb->nb_ap.sz;

	/* Write packet to BDs */
	FecPutPacket(ni, p_packet, size);
#else
	/*
	 * Increment NETBUF's reference counter fo prevent erasing the netbuf
	 * after this function returns.
	 *
	 * When NutNetBufClonePart() is called with zero 'inserts' parameter,
	 * then only the NETBUF structure is allocated and all NBDATA are
	 * referenced only (no data copying).
	 *
	 * The clone will be erased later in FecTxBdCleanup() after FEC confirms
	 * the packet was send.
	 */
	if ((nb_send = NutNetBufClonePart(nb, 0)) == NULL)
	{
		/* Unlock tx mutex */
		NutEventPost(&ni->tx_mutex);
		return -1;
	}

	/* Write packet to BDs */
	FecPutPacketNetbuf(ni, nb_send);
#endif

	/*
	 * Do TxBD's cleanup until there are at least FEC_TX_BUFFERS_MIN empty TxBDs.
	 *
	 * NOTE: Cleanup cannot be done immediatelly when tx interrupt occurs,
	 *       because calling Nut/OS API function from interrupt is disabled.
	 */
	FecTxBdCleanup(ni);

	while (ni->tx_bd_free < FEC_TX_BUFFERS_MIN)
	{
		NutEventWait(&ni->tx_rdy, NUT_WAIT_INFINITE);
		FecTxBdCleanup(ni);
	}

    /* Unlock tx mutex */
    NutEventPost(&ni->tx_mutex);

    return 0;
}

/* --------------------------------------------------------------------------
 * Interrupts, Rx thread, IOCTL & Init
 * -------------------------------------------------------------------------- */
/*
 * FEC Interrupt handlers
 */
static void FecIntTxF(FECINFO * ni)
{
	/* Wakeup task waiting for empty TxBD */
	NutEventPostFromIrq(&ni->tx_rdy);
}

static void FecIntRxF(FECINFO * ni)
{
	/* Wakeup receive thread */
	NutEventPostFromIrq(&ni->rx_rdy);
}


/*! \fn FecRxThread(void *arg)
 * \brief FEC receiver thread.
 *
 */
THREAD(FecRxThread, arg)
{
	NUTDEVICE *dev = arg;
	FECINFO *ni = (FECINFO *)dev->dev_dcb;
	IFNET *nif = (IFNET *)dev->dev_icb;
	NETBUF *nb;

	/* Run at high priority. */
    NutThreadSetPriority(9);

	while (1)
	{
		/* Disable Watchdog */
		if (EthMWDTSetVariableFN != NULL)
			EthMWDTSetVariableFN(0); //stop time guarding (prepare for wait state)

		/*
		 * Wait for Link (Auto-nego) & Initialize FEC
		 */
        while (!ni->initialized)
        {
        	if (nif->if_flags & IFF_UP)
        	{
				int	do_reset = 0;			//1st try w/o reset, if fails, reset is done in next cycle

				//if 1st call fails, successive calls do PHY sw reset
				do_reset = FecConfigure(nif, do_reset);
				if (do_reset == 0 && FecStart(ni, nif->if_mac) == 0)
				{
					/* FEC & PHY are initialized */
					ni->initialized = 1;
					/* wakeup application thread - reached init state */
					NutEventPostAsync(&ni->ioctl_rdy);
				}
				else
				{
#if 0
					if(runningThread->td_next)
						NutThreadSetPriority(((NUTTHREADINFO*)(runningThread->td_next))->td_priority);
					NutThreadYield();
#else
					NutSleep(500);
#endif
				}
        	}
        	else
        	{
				/* wakeup application thread - reached idle state */
        		NutEventPostAsync(&ni->ioctl_rdy);

				//wait for configuration wakeup
        		NutEventWait(&ni->rx_rdy, NUT_WAIT_INFINITE);
        	}
        }

		/*
         * Wait for the arrival of new packets
         */
//		NutEventWait(&ni->rx_rdy, NUT_WAIT_INFINITE);
		if (NutEventWait(&ni->rx_rdy, 1000) < 0)
		{
			//TODO: announce link lost event -> alarm?
			// Timeout, time for Link test
			(void)PhyLinkTest(dev);
			continue;
		}
		else
		{
			//packet arrived, does it make sense to check the link state :-)?
//			(void)PhyLinkTest(dev);
			//check if we were woken up by ioctl API fn, which disabled us
			if (!ni->initialized)
				continue;
		}

		/* Enable watchdog, wait for receive and pass the received packet to the upper layer. */
		if (EthMWDTSetVariableFN != NULL)
			EthMWDTSetVariableFN(5 * 60); //enable time guarding
		/*
		 * Fetch all packets from the NIC's internal buffer and pass
		 * them to the registered handler.
		 */
	    while (FecGetPacket(ni, &nb) == 0)
	    {
			/* Discard short packets. */
	        if (nb->nb_dl.sz < 60)
	        {
	        	NutNetBufFree(nb);
	        	//TODO counter
	        	continue;
	        }

	        /* Pass the received packet to the upper layer */
	        (*nif->if_recv) (dev, nb);
	    }
	}
}

/*!
 * \brief Ioctl Functions
 *
 * SIOCGIFADDR - Get MAC address
 * SIOCSIFADDR - Set MAC address
 * SIOCGIFFLAGS - Get flags
 * SIOCSIFFLAGS - Set Flags
 *
 * \param req	Required action.
 * \param conf	Desired value.
 */
static int FecIOCtl(NUTDEVICE * dev, int req, void *conf)
{
	int rc = 0;
	uint32_t *lvp = (uint32_t *)conf;
	IFNET *nif = (IFNET *)dev->dev_icb;
	FECINFO *ni = (FECINFO *)dev->dev_dcb;

	switch (req)
	{
		case SIOCSIFFLAGS:
			//TODO: check if new flags differ, post config event only in this case?

			/* Set interface flags. */
			if (!(*lvp & IFF_UP))
			{
				/* Stop interface. */
				ni->initialized = 0;
			}
			nif->if_flags = *lvp;

			/* Release FEC Rx thread to proceed with initialization
			 *  - either from waiting for config point
			 *  - or from wait for packet point
			 */
			NutEventPost(&ni->rx_rdy);

			//wait for configuration wakeup in case FEC Rx task did not yet finished its config phase
			rc = NutEventWait(&ni->ioctl_rdy, 10000);

			break;

		case SIOCGIFFLAGS:
			/* Get interface flags. */

			if (MCF_FEC_RCR & MCF_FEC_RCR_PROM)
			{
				nif->if_flags |= IFF_PROMISC;
			}
			else
			{
				nif->if_flags &= ~IFF_PROMISC;
			}

			*lvp = nif->if_flags;
			break;

		case SIOCSIFADDR:
			/* Set interface hardware address. */
			memcpy(nif->if_mac, conf, sizeof(nif->if_mac));

			/* Set the Ethernet MAC Address registers */
			MCF_FEC_PALR =	(((unsigned long)nif->if_mac[0]) << 24) |
							(((unsigned long)nif->if_mac[1]) << 16) |
							(((unsigned long)nif->if_mac[2]) <<  8) |
							(((unsigned long)nif->if_mac[3]) <<  0);
			MCF_FEC_PAUR = 	(((unsigned long)nif->if_mac[4]) << 24) |
							(((unsigned long)nif->if_mac[5]) << 16);
			break;

		case SIOCGIFADDR:
			/* Get interface hardware address. */
			memcpy(conf, nif->if_mac, sizeof(nif->if_mac));
			break;

		default:
			rc = -1;
			break;
	}
	return rc;
}

/*!
 * \brief Initialize Ethernet hardware.
 *
 * Applications should do not directly call this function. It is
 * automatically executed during device registration by
 * NutRegisterDevice().
 *
 * \param dev Identifies the device to initialize.
 */
static int FecInit(NUTDEVICE * dev)
{
	FECINFO		*ni = (FECINFO *) dev->dev_dcb;
#ifdef FEC_WAIT_FOR_LINK_TIMEOUT
	IFNET 		*nif = (IFNET *) dev->dev_icb;
	uint32_t	timeout = FEC_WAIT_FOR_LINK_TIMEOUT + NutGetMillis();
#endif

	/* Register interrupt handlers */
	typedef void (*ih) (void *);
	NutRegisterIrqHandler(&sig_FEC_RF, (ih)FecIntRxF, ni);
	NutRegisterIrqHandler(&sig_FEC_TF, (ih)FecIntTxF, ni);

    /* Configure pins, wait for end of reset */
    FecPowerUp();

#ifdef FEC_WAIT_FOR_LINK_TIMEOUT
	/*
	 * Wait for Link (Auto-nego) & Initialize FEC.
	 * If the initialization fails (e.g. due to link down), then
	 * the FEC will wait for link inside FecRxThread.
	 */
	while (1)
	{
		if (!FecConfigure(nif))
		{
			if (FecStart(ni, nif->if_mac))
				return -1;

			break;
		}

		if (NutGetMillis() > timeout)
		{
			if (errno == ENETDOWN)
				break;
			else
			{
				puts("FEC: initialization error");
				alarmFec = 1;
				return -1;
			}
		}

		NutThreadYield();
	}
#endif

	/* Start the receiver thread. */
	if (NutThreadCreate("eth0rx", FecRxThread, dev, NUT_THREAD_NICRXSTACK) == NULL)
		return -1;

	return 0;
}

/* Set MultiWatchDog set reset function */
void Mcf5FecEthMWDTSetVariableFN(TMWDTSetVariableFN VariableFN)
{
	EthMWDTSetVariableFN = VariableFN;
}

static FECINFO dcb_eth0  __attribute__ ((aligned (16)));

/*!
 * \brief Network interface information structure.
 *
 * Used to call.
 */
static IFNET ifn_eth0 = {
    IFT_ETHER,                  /*!< \brief Interface type, if_type. */
    IFF_AUTONEGO_ENABLE,        /*!< \brief Interface flags, if_flags. */
    {0, 0, 0, 0, 0, 0},         /*!< \brief Hardware net address, if_mac. */
    0,                          /*!< \brief IP address, if_local_ip. */
    0,                          /*!< \brief Remote IP address for point to point, if_remote_ip. */
    0,                          /*!< \brief IP network mask, if_mask. */
    ETHERMTU,                   /*!< \brief Maximum size of a transmission unit, if_mtu. */
    0,                          /*!< \brief Packet identifier, if_pkt_id. */
    0,                          /*!< \brief Linked list of arp entries, arpTable. */
    0,                          /*!< \brief Linked list of multicast address entries, if_mcast. */
    NutEtherInput,              /*!< \brief Routine to pass received data to, if_recv(). */
    FecOutput,                  /*!< \brief Driver output routine, if_send(). */
    NutEtherOutput              /*!< \brief Media output routine, if_output(). */
};

/*!
 * \brief Device information structure.
 *
 * A pointer to this structure must be passed to NutRegisterDevice()
 * to bind this Ethernet device driver to the Nut/OS kernel.
 * An application may then call NutNetIfConfig() with the name \em eth0
 * of this driver to initialize the network interface.
 *
 */
NUTDEVICE devMcf5Fec = {
    0,                          /*!< \brief Pointer to next device. */
    {'e', 't', 'h', '0', 0, 0, 0, 0, 0},        /*!< \brief Unique device name. */
    IFTYP_NET,                  /*!< \brief Type of device. */
    0,                          /*!< \brief Base address. */
    0,                          /*!< \brief First interrupt number. */
    &ifn_eth0,                  /*!< \brief Interface control block. */
    &dcb_eth0,                  /*!< \brief Driver control block. */
    FecInit,                    /*!< \brief Driver initialization routine. */
    FecIOCtl,                   /*!< \brief Driver specific control function. */
    0,                          /*!< \brief Read from device. */
    0,                          /*!< \brief Write to device. */
#ifdef __HARVARD_ARCH__
    0,                          /*!< \brief Write from program space data to device. */
#endif
    0,                          /*!< \brief Open a device or file. */
    0,                          /*!< \brief Close a device or file. */
    0                           /*!< \brief Request file size. */
};

/*@}*/
