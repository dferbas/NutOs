
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

#define PREVENT_SPURIOUS_INTERRUPT(code) {NutEnterCritical();{code;} NutExitCritical();}

typedef int  t_spi_ret;

uint32_t tx_counter, rx_counter;
uint8_t *p_tx_data, *p_rx_data;
uint32_t size_data;

static HANDLE mutex_tran_start, mutex_tran_end;

static void QspiIntSPIFE(void);
void QspiInterrupt(void *arg);

/*
** psp_spi_tx1
**
** Transmit 1 byte
**
** Input:
**   uid   - unit ID
**   val   - value to send
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_tx1 ( uint8_t uid, uint8_t val )
{
	MCF_QSPI_QAR = QSPI_COMMAND_ADDRESS;
    MCF_QSPI_QDR = MCF_QSPI_QDR_DATA(MCF_QSPI_QDR_BITSE);
    MCF_QSPI_QAR = QSPI_TRANSMIT_ADDRESS;
    MCF_QSPI_QDR = val;

    MCF_QSPI_QIR |= MCF_QSPI_QIR_SPIF;

    PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR &= ~MCF_QSPI_QIR_SPIFE;)

    MCF_QSPI_QWR =  (MCF_QSPI_QWR&MCF_QSPI_QWR_CSIV)|MCF_QSPI_QWR_ENDQP(0x00)|MCF_QSPI_QWR_NEWQP(0x00);
    MCF_QSPI_QDLYR |= MCF_QSPI_QDLYR_SPE;

    while (!(MCF_QSPI_QIR & MCF_QSPI_QIR_SPIF))	// polling
    	;/* Empty Body */

    PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);)

    return PSP_SPI_SUCCESS;
}

/*
** psp_spi_tx
**
** Transmit 'len' number of bytes
**
** Input:
**   uid   - unit ID
**   p_src - pointer to source buffer
**   len   - number of bytes to transmit
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_tx ( uint8_t uid, uint8_t * p_src, uint32_t len )
{
	uint8_t j;

    if (len > 16) // interrupt method
    {
    	// wait on mutex 'start transmit'
    	NutEventWait(&mutex_tran_start, NUT_WAIT_INFINITE);

        // save source data pointer to interrupt TX buffer
        p_tx_data = p_src;
        // save destination data pointer to interrupt RX buffer
        p_rx_data = NULL;
        // save length of receive data to interrupt data size
        size_data = len;

        // initialization of interrupt 'end of queue'
        PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);)

       	// limited transmit data length
        len = (len > 16) ? 16 : len;

        rx_counter = 0;
        tx_counter = 0;

        // transmit only first 16 bytes of TX data
        for (j=0; j < len; j++) {
        	MCF_QSPI_QAR = QSPI_COMMAND_ADDRESS+j;
            MCF_QSPI_QDR = MCF_QSPI_QDR_DATA(MCF_QSPI_QDR_BITSE);
            MCF_QSPI_QAR = QSPI_TRANSMIT_ADDRESS+j;
            MCF_QSPI_QDR = p_src[tx_counter++];
        }

        MCF_QSPI_QWR = (MCF_QSPI_QWR&MCF_QSPI_QWR_CSIV) |
                       	   MCF_QSPI_QWR_ENDQP(len-1) | MCF_QSPI_QWR_NEWQP(0);

        // start transmit data
        MCF_QSPI_QDLYR |= MCF_QSPI_QDLYR_SPE;

        // wait on mutex 'end transmit'
        NutEventWait(&mutex_tran_end, NUT_WAIT_INFINITE);

        // release mutex 'start transmit'
        NutEventPost(&mutex_tran_start);
    }
    else // polling method
    {
        for (j=0; j < len; j++) {
        	MCF_QSPI_QAR = QSPI_COMMAND_ADDRESS+j;
            MCF_QSPI_QDR = MCF_QSPI_QDR_DATA(MCF_QSPI_QDR_BITSE);
            MCF_QSPI_QAR = QSPI_TRANSMIT_ADDRESS+j;
            MCF_QSPI_QDR = p_src[j];
        }

        MCF_QSPI_QIR |= MCF_QSPI_QIR_SPIF;
        PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR &= ~MCF_QSPI_QIR_SPIFE;)

        MCF_QSPI_QWR = (MCF_QSPI_QWR&MCF_QSPI_QWR_CSIV) |
                        MCF_QSPI_QWR_ENDQP(len-1) | MCF_QSPI_QWR_NEWQP(0);
        MCF_QSPI_QDLYR |= MCF_QSPI_QDLYR_SPE;


        while (!(MCF_QSPI_QIR & MCF_QSPI_QIR_SPIF))
            ; /* Empty Body */

        PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);)
    }

    return PSP_SPI_SUCCESS;
}

/*
** psp_spi_rx
**
** receive 'len' number of bytes
**
** Input:
**   uid   - unit ID
**   p_dst - pointer to destination buffer
**   len   - number of bytes to receive
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_rx ( uint8_t uid, uint8_t * p_dst, uint32_t len )
{
	uint8_t j;

    if (len > 16) // interrupt method
    {
    	// wait on mutex 'start transmit'
    	NutEventWait(&mutex_tran_start, NUT_WAIT_INFINITE);

        // save source data pointer to interrupt TX buffer
        p_tx_data = NULL;
        // save destination data pointer to interrupt RX buffer
        p_rx_data = p_dst;
        // save length of receive data to interrupt data size
        size_data = len;

        // initialization of interrupt 'end of queue'
        PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);)

       	// limited transmit data length
        len = (len > 16) ? 16 : len;

        rx_counter = 0;
        tx_counter = 0;

        // transmit only first 16 bytes of TX data
        for (j=0; j < len; j++) {
        	MCF_QSPI_QAR = QSPI_COMMAND_ADDRESS+j;
            MCF_QSPI_QDR = MCF_QSPI_QDR_DATA(MCF_QSPI_QDR_BITSE);
            MCF_QSPI_QAR = QSPI_TRANSMIT_ADDRESS+j;
            MCF_QSPI_QDR = 0xFF;
            tx_counter++;
        }

        MCF_QSPI_QWR = (MCF_QSPI_QWR&MCF_QSPI_QWR_CSIV) |
                       	   MCF_QSPI_QWR_ENDQP(len-1) | MCF_QSPI_QWR_NEWQP(0);

        // start transmit data
        MCF_QSPI_QDLYR |= MCF_QSPI_QDLYR_SPE;

        // wait on mutex 'end transmit'
        NutEventWait(&mutex_tran_end, NUT_WAIT_INFINITE);

        // release mutex 'start transmit'
        NutEventPost(&mutex_tran_start);

    }
    else // polling method
    {
        for (j=0; j < len; j++) {
        	MCF_QSPI_QAR = QSPI_COMMAND_ADDRESS+j;
            MCF_QSPI_QDR = MCF_QSPI_QDR_DATA(MCF_QSPI_QDR_BITSE);
            MCF_QSPI_QAR = QSPI_TRANSMIT_ADDRESS+j;
            MCF_QSPI_QDR = 0xFF;
        }

        MCF_QSPI_QIR |= MCF_QSPI_QIR_SPIF;
        PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR &= ~MCF_QSPI_QIR_SPIFE;)

        MCF_QSPI_QWR = (MCF_QSPI_QWR&MCF_QSPI_QWR_CSIV) |
                        MCF_QSPI_QWR_ENDQP(len-1) | MCF_QSPI_QWR_NEWQP(0);
        MCF_QSPI_QDLYR |= MCF_QSPI_QDLYR_SPE;

        while (!(MCF_QSPI_QIR & MCF_QSPI_QIR_SPIF))
            ; /* Empty Body */

        MCF_QSPI_QAR = QSPI_RECEIVE_ADDRESS;
        for (j=0; j < len; j++) {
            p_dst[j] = MCF_QSPI_QDR;
        }

        PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR |= (MCF_QSPI_QIR_SPIF | MCF_QSPI_QIR_SPIFE);)
    }

    return PSP_SPI_SUCCESS;
}

/*
** psp_spi_cs_lo
**
** Set chip select low.
**
** Input:
**   uid   - unit ID
*/
void psp_spi_cs_lo ( uint8_t uid )
{
	MCF_GPIO_PORTQS &= ~(MCF_GPIO_PORTQS_PORTQS3);
}


/*
** psp_spi_cs_hi
**
** Set chip select high.
**
** Input:
**   uid   - unit ID
*/
void psp_spi_cs_hi ( uint8_t uid )
{
	MCF_GPIO_PORTQS |= MCF_GPIO_PORTQS_PORTQS3;
}

/*
** psp_spi_lock
**
** Lock the SPI for the specific unit. This can
** be useful if multiple units are attached to the
** same SPI bus.
**
** Input:
**   uid   - unit ID
*/
void psp_spi_lock ( uint8_t uid )
{
	return;
}


/*
** psp_spi_unlock
**
** Unlock the SPI for the specific unit. This can
** be useful if multiple units are attached to the
** same SPI bus.
**
** Input:
**   uid   - unit ID
*/
void psp_spi_unlock ( uint8_t uid )
{
	return;
}


/*
** psp_spi_set_baudrate
**
** Set baudrate.
**
** Input:
**   uid   - unit ID
**   br    - baudrate in Hz
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_set_baudrate ( uint8_t uid, uint32_t br )
{
	uint16_t baudRate;

	baudRate = ( NutGetCpuClock() / (2 * br) );	/* 80M/(2*BaudRate) */
	baudRate = (baudRate < 2) ? 2 : baudRate;		// set min possible baudrate
	baudRate = (baudRate > 255) ? 255 : baudRate;	// set max possible baudrate
    if ((baudRate > 1 ) && (baudRate < 256)){
        MCF_QSPI_QMR &= ~MCF_QSPI_QMR_BAUD(0xFF);
        MCF_QSPI_QMR |= MCF_QSPI_QMR_BAUD((uint8_t)baudRate);

        return PSP_SPI_SUCCESS;
    }
    else
        return PSP_SPI_ERROR;
}

/*
** psp_spi_get_baudrate
**
** Get baudrate.
**
** Input:
**   uid   - unit ID
** Output:
**   p_br  - baudrate in Hz
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_get_baudrate ( uint8_t uid, uint32_t * p_br )
{
	uint8_t baud;

	baud = MCF_QSPI_QMR & 0xFF;
	*p_br = ( NutGetCpuClock() / (2 * baud) );

	return PSP_SPI_SUCCESS;
}

/*
** psp_spi_init
**
** Init SPI port
**
** Input:
**   uid   - unit ID
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_init ( uint8_t uid )
{
	t_spi_ret ret = PSP_SPI_SUCCESS;

	// CS0 to high
	MCF_GPIO_PORTQS |= MCF_GPIO_PORTQS_PORTQS3;

	MCF_GPIO_PQSPAR |= MCF_GPIO_PQSPAR_QSPI_DOUT_DOUT |
                          MCF_GPIO_PQSPAR_QSPI_DIN_DIN |
                          MCF_GPIO_PQSPAR_QSPI_CLK_CLK
                          /*| MCF_GPIO_PQSPAR_QSPI_CS0_CS0*/;

    // Port QS Data Direction pin 3 (QSPI_PCS0) as an output
    MCF_GPIO_DDRQS |= MCF_GPIO_DDRQS_DDRQS3;

    // Set as a Master always and set CPOL & CPHA
    MCF_QSPI_QMR = MCF_QSPI_QMR_MSTR /*| MCF_QSPI_QMR_CPHA | MCF_QSPI_QMR_CPOL */;
    // Set number of bits to be transferred for each entry in the queue
    MCF_QSPI_QMR |= MCF_QSPI_QMR_BITS(QSPI_TRANS_SIZE);

    // Set baud rate
    ret |= psp_spi_set_baudrate(0, QSPI_BAUD_RATE);

    // Use active low as default
    MCF_QSPI_QWR = MCF_QSPI_QWR_CSIV;

    // Set interrupt flags and errors
    PREVENT_SPURIOUS_INTERRUPT(MCF_QSPI_QIR = (MCF_QSPI_QIR_WCEFB | 
    				  MCF_QSPI_QIR_ABRTB | MCF_QSPI_QIR_WCEF | 
    				  MCF_QSPI_QIR_ABRT | MCF_QSPI_QIR_SPIF);)

    // Register interrupt handler
    NutRegisterIrqHandler(&sig_QSPI_TF, QspiInterrupt, NULL);

    /* Enable Interrupts */
   	NutIrqEnable(&sig_QSPI_TF);

   	// put mutex 'start transmit' into signaled state
   	NutEventPost(&mutex_tran_start);

    return ret;
}

/*
** psp_spi_start
**
** Start SPI port
**
** Input:
**   uid   - unit ID
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_start ( uint8_t uid )
{
  return PSP_SPI_SUCCESS;
}

/*
** psp_spi_stop
**
** Stop SPI port
**
** Input:
**   uid   - unit ID
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_stop ( uint8_t uid )
{
  return PSP_SPI_SUCCESS;
}

/*
** psp_spi_delete
**
** Delete SPI port
**
** Input:
**   uid   - unit ID
** Return:
**   PSP_SPI_...
*/
t_spi_ret psp_spi_delete ( uint8_t uid )
{
  return PSP_SPI_SUCCESS;
}

/*
 * interrupt callback on SPIFE
 */
void QspiIntSPIFE(void)
{
	uint32_t i, j;

   	MCF_QSPI_QAR = QSPI_RECEIVE_ADDRESS;
   	for (i=rx_counter; i<tx_counter; i++ ) {
   		if (p_rx_data != NULL) {
   			p_rx_data[rx_counter] = MCF_QSPI_QDR;
   		}
   		rx_counter++;
   	}

    j = 0;
    for (i=tx_counter; i<size_data; i++ ) {
    	MCF_QSPI_QAR = QSPI_COMMAND_ADDRESS+j;
        MCF_QSPI_QDR = MCF_QSPI_QDR_DATA(MCF_QSPI_QDR_BITSE);
        MCF_QSPI_QAR = QSPI_TRANSMIT_ADDRESS+j;
        MCF_QSPI_QDR = (p_tx_data != NULL) ? p_tx_data[tx_counter] : 0xFF;
       	tx_counter++;

        if (tx_counter == size_data)
        	MCF_QSPI_QWR = (MCF_QSPI_QWR&MCF_QSPI_QWR_CSIV) |
                        MCF_QSPI_QWR_ENDQP(((size_data)%16)-1) | MCF_QSPI_QWR_NEWQP(0);

        if (j < 15)
        	j++;
        else
        	break;
    }

    if (rx_counter == size_data) {
    	// release mutex 'end transmit'
    	NutEventPostFromIrq(&mutex_tran_end);
    	return;
    }

    MCF_QSPI_QDLYR |= MCF_QSPI_QDLYR_SPE;
}

/*
 * QSPI ISR
 */
void QspiInterrupt(void *arg)
{
   QspiIntSPIFE();
}
