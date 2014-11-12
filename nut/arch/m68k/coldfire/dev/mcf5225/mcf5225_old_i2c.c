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

#define NUT_THREAD_TWI_TRANSACT_STACK   	1000			// TODO .. tune it

typedef struct {
	uint8_t	dcb_base;
	HANDLE tw_mm_mutex;      /* Exclusive master access. */
	HANDLE tw_mm_que;        /* Threads waiting for master transfer done. */
	volatile uint8_t tw_mm_err;   /* Current master mode error. */
	uint8_t tw_mm_error;          /* Last master mode error. */

	uint8_t I2C1_SlaveAddr;            /* Variable for Slave address */
	uint32_t AddrLenM;                   /* Length of input bufer's content */
	uint32_t InpLenM;                   /* Length of input bufer's content */
	uint32_t OutLenM;                   /* Length of output bufer's content */
	uint32_t tw_mr_cnt;                   /* Number of received bytes */
	uint8_t *DataPtrM;                  /* Pointer to data buffer for Master mode */
	uint8_t *AddrPtrM;                  /* Pointer to output buffer for Master mode */
	//uint32_t I2C1_SndRcvTemp;                  /* Temporary variable for SendChar (RecvChar) when they call SendBlock (RecvBlock) */
	//uint8_t ChrTemp;                   /* Temporary variable for SendChar method */
	volatile uint8_t tw_if_bsy;   	/* Bus busy flag */
	uint8_t tw_rptst_rqst;   			/* Request for repeated start */
	uint8_t tw_mm_sla;            /* Destination slave address. */
} TWIDCB;

static TWIDCB dcb_twi[2];

/*
 * TWI interrupt handler.
 */
static void TwInterrupt(void *arg)
{
	uint8_t Status; /* Safe status register */
	TWIDCB *dev = arg;

//	MCF_I2C_I2SR(dev->dcb_base) &= ~MCF_I2C_I2SR_IIF; /* Clear interrupt flag and ARBL flag if set (by means of the read modify write effect) */

	Status = MCF_I2C_I2SR(dev->dcb_base);
	if (MCF_I2C_I2CR(dev->dcb_base) & MCF_I2C_I2CR_MSTA) { /* Is device in master mode? */
		if (MCF_I2C_I2CR(dev->dcb_base) & MCF_I2C_I2CR_MTX) { /* Is device in Tx mode? */
			if (Status & MCF_I2C_I2SR_RXAK) { /* NACK received? */
				MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_MSTA; /* Switch device to slave mode (stop signal sent) */
				MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_MTX; /* Switch to Rx mode */
				dev->AddrLenM = 0U;
				dev->OutLenM = 0U; /* No character for sending */
				dev->InpLenM = 0U; /* No character for reception */
				dev->tw_if_bsy = 0;
			} else {
				if (dev->AddrLenM) { /* Is any char. for transmitting? */
					dev->AddrLenM--; /* Decrease number of chars for the transmit */
					MCF_I2C_I2DR(dev->dcb_base) = *(dev->AddrPtrM)++; /* Send character */
				}
				else if (dev->OutLenM) { /* Is any char. for transmitting? */
					dev->OutLenM--; /* Decrease number of chars for the transmit */
					MCF_I2C_I2DR(dev->dcb_base) = *(dev->DataPtrM)++; /* Send character */
				} else {
					if (dev->InpLenM) { /* Is any char. for reception? */
						if (dev->tw_rptst_rqst) {
							MCF_I2C_I2CR(dev->dcb_base) |= MCF_I2C_I2CR_RSTA; /* Resend Start */
							MCF_I2C_I2DR(dev->dcb_base) = (uint8_t) (dev->tw_mm_sla
									| 0x01); /* device id to read */
							dev->tw_rptst_rqst = 0;
						} else {
							if (dev->InpLenM == 1U) { /* If only one char to receive */
								MCF_I2C_I2CR(dev->dcb_base) |= MCF_I2C_I2CR_TXAK; /* then transmit ACK disable */
							} else {
								MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_TXAK; /* else transmit ACK enable */
							}
							MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_MTX; /* Switch to Rx mode */
							(void) MCF_I2C_I2DR(dev->dcb_base); /* Dummy read character */
						}
					} else {
						dev->tw_if_bsy = 0;
						MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_MSTA; /* Switch device to slave mode (stop signal sent) */
						MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_MTX; /* Switch to Rx mode */
						NutEventPostFromIrq(&dev->tw_mm_que);
					}
				}
			}
		} else {
			dev->InpLenM--; /* Decrease number of chars for the receive */
			if (dev->InpLenM) { /* Is any char. for reception? */
				if (dev->InpLenM == 1U) {
					MCF_I2C_I2CR(dev->dcb_base) |= MCF_I2C_I2CR_TXAK; /* Transmit ACK disable */
				}
			} else {
				MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_MSTA; /* If no, switch device to slave mode (stop signal sent) */
				MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_TXAK; /* Transmit ACK enable */
			}
			*(dev->DataPtrM)++ = MCF_I2C_I2DR(dev->dcb_base); /* Receive character */
			dev->tw_mr_cnt++;
			if (!dev->InpLenM) { /* Is any char. for reception? */
				NutEventPostFromIrq(&dev->tw_mm_que);
			}
		}
	} else {
		if (Status & MCF_I2C_I2SR_IAL) { /* Arbitration lost? */
			dev->AddrLenM = 0U; /* No address character for sending */
			dev->OutLenM = 0U; /* No character for sending */
			dev->InpLenM = 0U; /* No character for reception */
			dev->tw_if_bsy = 0; /* No character for sending or reception*/
			MCF_I2C_I2CR(dev->dcb_base) &= ~MCF_I2C_I2CR_MTX; /* Switch to Rx mode */
			dev->tw_mm_err = TWERR_BUS;
		}
	}
}

static void reanableDevice(int dcbBase){
	/* clear control register */
	MCF_I2C_I2CR(dcbBase) = 0;

	/* enable module and send a START condition*/
	MCF_I2C_I2CR(dcbBase) =
	MCF_I2C_I2CR_IEN | MCF_I2C_I2CR_MSTA;

	/* dummy read */

	(void) MCF_I2C_I2DR(dcbBase);

	/* clear status register */
	MCF_I2C_I2SR(dcbBase) = 0;

	/* clear control register */
	MCF_I2C_I2CR(dcbBase) = 0;

	/* enable the module again */
	MCF_I2C_I2CR(dcbBase) = MCF_I2C_I2CR_IEN;
}

/*!
 * \brief Transmit and/or receive data as a master.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \note This function is only available on ATmega128 systems.
 *
 * \param sla    Slave address of the destination. This slave address
 *               must be specified as a 7-bit address. For example, the
 *               PCF8574A may be configured to slave addresses from 0x38
 *               to 0x3F.
 * \param txdata Points to the data to transmit. Ignored, if the number
 *               of data bytes to transmit is zero.
 * \param txlen  Number of data bytes to transmit. If zero, then the
 *               interface will not send any data to the slave device
 *               and will directly enter the master receive mode.
 * \param rxdata Points to a buffer, where the received data will be
 *               stored. Ignored, if the maximum number of bytes to
 *               receive is zero.
 * \param rxsiz  Maximum number of bytes to receive. Set to zero, if
 *               no bytes are expected from the slave device.
 * \param tmo    Timeout in milliseconds. To disable timeout, set this
 *               parameter to NUT_WAIT_INFINITE.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int TwMasterCommon(uint8_t sla, const void *addr, uint16_t addrsiz, void *data, uint16_t siz, uint32_t tmo, uint8_t write)
{
    int rc = -1;
    TWIDCB *dev = &dcb_twi[DCB_BASE(sla)];

	/* This routine is marked reentrant, so lock the interface. */
	if (NutEventWait(&dev->tw_mm_mutex, 500)) {
		dev->tw_mm_err = TWERR_IF_LOCKED;
		return -1;
	}

    /* Set all parameters for master mode. */
	dev->tw_mm_err = 0;
	dev->tw_mr_cnt = 0;
	dev->tw_mm_sla = (uint8_t) (sla << 1); /* Set slave address */
	uint8_t slave_addr = (uint8_t) (dev->tw_mm_sla); /* Prepare slave address */
    if (write)
    {
    	dev->AddrLenM = addrsiz; /* Set lenght of data */
    	dev->AddrPtrM = (uint8_t *) addr; /* Save pointer to data for transmit */

    	dev->OutLenM = siz; /* Set lenght of data */
    	dev->DataPtrM = (uint8_t *) data; /* Save pointer to data for transmit */

    	dev->InpLenM = 0; /* Set lenght of data */
    }
    else
    {
    	dev->AddrLenM = addrsiz; /* Set lenght of data */
    	dev->AddrPtrM = (uint8_t *) addr; /* Save pointer to data for transmit */

    	dev->OutLenM = 0; 	/* Set lenght of data */

    	dev->InpLenM = siz; /* Set lenght of data */
    	dev->DataPtrM = (uint8_t *) data; /* Save pointer to data for reception */

    	if (dev->InpLenM && dev->AddrLenM)
    		dev->tw_rptst_rqst = 1;
    	else
    		dev->tw_rptst_rqst = 0;

    	if (dev->AddrLenM == 0)
    		slave_addr |= 0x01;
    }

	if((MCF_I2C_I2SR(dev->dcb_base) & MCF_I2C_I2SR_IBB) || dev->tw_if_bsy) { /* Is the bus busy */
		reanableDevice(dev->dcb_base);

		if(dev->dcb_base == 0){
			NutIrqEnable(&sig_I2C0);
		}
		else{
			NutIrqEnable(&sig_I2C1);
		}
	}

    /* Clear the queue. */
    //*broken?! NutEventBroadcastAsync(&tw_mm_que);
    if (dev->tw_mm_que == SIGNALED) {
    	dev->tw_mm_que = 0;
    }

    NutEnterCriticalLevel(IH_I2C_LEVEL); /* Enter the critical section */

	MCF_I2C_I2CR(dev->dcb_base) |= MCF_I2C_I2CR_MTX; /* Set TX mode */
	if (MCF_I2C_I2CR(dev->dcb_base) & MCF_I2C_I2CR_MSTA) { /* Is device in master mode? */
		MCF_I2C_I2CR(dev->dcb_base) |= MCF_I2C_I2CR_RSTA; /* If yes then repeat start cycle generated */
	} else {
		MCF_I2C_I2CR(dev->dcb_base) |= MCF_I2C_I2CR_MSTA; /* If no then start signal generated */
	}
	MCF_I2C_I2DR(dev->dcb_base) = slave_addr;

	NutExitCritical(); /* Exit the critical section */

	/* Wait for transact complete. */
    if (NutEventWait(&dev->tw_mm_que, tmo)) {
    	dev->tw_mm_error = TWERR_TIMEOUT;
    } else {
    	NutEnterCriticalLevel(IH_I2C_LEVEL);
        if (dev->tw_mm_err) {
        	dev->tw_mm_error = dev->tw_mm_err;
        } else {
            rc = dev->tw_mr_cnt;
        }
        NutExitCritical();
    }

	/* Release the interface. */
	NutEventPost(&dev->tw_mm_mutex);

	return rc; /* Dummy number of really received chars */
}
int TwMasterTransact(uint8_t sla, const void *txdata, uint16_t txlen, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
	return TwMasterCommon(sla, txdata, txlen, rxdata, rxsiz, tmo, 0);
}

int TwMasterRead(uint8_t sla, const void *addr, uint8_t addrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
	return TwMasterCommon(sla, addr, addrlen, rxdata, rxsiz, tmo, 0);
}

int TwMasterWrite(uint8_t sla, const void *addr, uint8_t addrlen, void *txdata, uint16_t txsiz, uint32_t tmo)
{
	return TwMasterCommon(sla, addr, addrlen, txdata, txsiz, tmo, 1);
}

/*!
 * \brief Get last master mode error only from I2C0!!!.
 *
 * You may call this function to determine the specific cause
 * of an error after TwMasterTransact() failed.
 *
 * \note This function is only available on ATmega128 systems.
 *
 */
int TwMasterError(void)
{
	// TwMasterError receive only error from I2C0
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
int TwIOCtl(int req, void *conf)
{
	// chose which TWI control
	uint8_t dbcBase = DCB_BASE(req);
	req &= ~DCB_BASE_MASK;

#define IC_SIZE 64
	uint8_t rc = 0, ic = 0x1F, i;
    uint16_t divaderTable[IC_SIZE] = {28, 30, 34, 40, 44, 48, 56, 68, 80, 88, 104, 128, 144, 160, 192, 240,
        		288, 320, 384, 480, 576, 640, 768, 960, 1152, 1280, 1536, 1920, 2304, 2560, 3072, 3840,
        		20, 22, 24, 26, 28, 32, 36, 40, 48, 56, 64, 72, 80, 96, 112, 128,
        		160, 192, 224, 256, 320, 384, 448, 512, 640, 768, 896, 1024, 1280, 1536, 1792, 2048};
    uint32_t *speedHz = (uint32_t *) conf;
    uint16_t selectedDivader = 0xFFFF;
    uint16_t countedDivader = NutGetCpuClock() / (*speedHz);

    switch (req) {

    case TWI_SETSPEED:
    	for (i = 0; i < IC_SIZE; ++i) {
			if (divaderTable[i] >= countedDivader
					&& divaderTable[i] < selectedDivader) {
				selectedDivader = divaderTable[i];
				ic = i;
			}
		}
		MCF_I2C_I2FDR(dbcBase) = MCF_I2C_I2FDR_IC(ic);
        break;
    case TWI_GETSPEED:
    	selectedDivader = divaderTable[MCF_I2C_I2FDR(dbcBase)];
    	*speedHz = NutGetCpuClock() / (selectedDivader);
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
	dev->I2C1_SlaveAddr = 0x10U; /* Set variable for slave address */
	dev->InpLenM = 0U; /* No data to be received */
	 uint32_t speed = 100000;
	 /* Enable the I2C signals */
	 if(dev->dcb_base == 0){
		 MCF_GPIO_PASPAR |= MCF_GPIO_PASPAR_SDA0_SDA0 | MCF_GPIO_PASPAR_SCL0_SCL0;
	 }
	 else{
		 MCF_GPIO_PUCPAR |= MCF_GPIO_PUCPAR_URTS2_SDA1 | MCF_GPIO_PUCPAR_UCTS2_SCL1;
	 }

	/* set the frequency near 100 000Hz, see MCF52223RM table for details */
	TwIOCtl(TWI_SETSPEED + (sla & DCB_BASE_MASK), &speed);
	TwIOCtl(TWI_GETSPEED + (sla & DCB_BASE_MASK), &speed);

	reanableDevice(dev->dcb_base);


	if(dev->dcb_base == 0){
		NutRegisterIrqHandler(&sig_I2C0, TwInterrupt, dev);
		NutIrqEnable(&sig_I2C0);
	}
	else{
		NutRegisterIrqHandler(&sig_I2C1, TwInterrupt, dev);
		NutIrqEnable(&sig_I2C1);
	}


	/* Release the interface. */
	NutEventPost(&dev->tw_mm_mutex);

    return 0;
}

