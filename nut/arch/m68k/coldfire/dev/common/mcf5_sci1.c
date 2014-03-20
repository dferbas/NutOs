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

#include <cfg/arch.h>
#include <cfg/uart.h>
#include <arch/m68k.h>
#include <dev/usart.h>

#define BASE    1

/*
 * Function prototypes
 */
static uint32_t Mcf5SciGetSpeed(void);
static int Mcf5SciSetSpeed(uint32_t rate);
static uint8_t Mcf5SciGetDataBits(void);
static int Mcf5SciSetDataBits(uint8_t bits);
static uint8_t Mcf5SciGetParity(void);
static int Mcf5SciSetParity(uint8_t mode);
static uint8_t Mcf5SciGetStopBits(void);
static int Mcf5SciSetStopBits(uint8_t bits);
static uint32_t Mcf5SciGetStatus(void);
static int Mcf5SciSetStatus(uint32_t flags);
static void Mcf5SciTxStart(void);
static void Mcf5SciRxStart(void);
static int Mcf5SciSetFlowControl(uint32_t flags);
static uint32_t Mcf5SciGetFlowControl(void);
static int Mcf5SciInit(void);
static int Mcf5SciDeinit(void);

/*!
 * \brief SCI1 device control block structure.
 */
static USARTDCB dcb_sci1 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    {0, 0, 0, 0, 0, 0, 0, 0},   /* dcb_tx_rbf */
    {0, 0, 0, 0, 0, 0, 0, 0},   /* dcb_rx_rbf */
    0,                          /* dbc_last_eol */
    Mcf5SciInit,              /* dcb_init */
    Mcf5SciDeinit,            /* dcb_deinit */
    Mcf5SciTxStart,           /* dcb_tx_start */
    Mcf5SciRxStart,           /* dcb_rx_start */
    Mcf5SciSetFlowControl,    /* dcb_set_flow_control */
    Mcf5SciGetFlowControl,    /* dcb_get_flow_control */
    Mcf5SciSetSpeed,          /* dcb_set_speed */
    Mcf5SciGetSpeed,          /* dcb_get_speed */
    Mcf5SciSetDataBits,       /* dcb_set_data_bits */
    Mcf5SciGetDataBits,       /* dcb_get_data_bits */
    Mcf5SciSetParity,         /* dcb_set_parity */
    Mcf5SciGetParity,         /* dcb_get_parity */
    Mcf5SciSetStopBits,       /* dcb_set_stop_bits */
    Mcf5SciGetStopBits,       /* dcb_get_stop_bits */
    Mcf5SciSetStatus,         /* dcb_set_status */
    Mcf5SciGetStatus,         /* dcb_get_status */
    0,                          /* dcb_set_clock_mode */
    0,                          /* dcb_get_clock_mode */
};

/*!
 * \name Coldfire SCI1 Device
 */
/*@{*/
/*!
 * \brief Coldfire device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the Coldfire's on-chip SCI1.
 *
 * The device is named \b sci1.
 *
 * \showinitializer
 */
NUTDEVICE devSciMcf5_1 = {
    0,                          /* Pointer to next device, dev_next. */
    {'s', 'c', 'i', '1', 0, 0, 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    BASE,                       /* Base address, dev_base. */
    0,                          /* First interrupt number, dev_irq (not used). */
    0,                          /* Interface control block, dev_icb (not used). */
    &dcb_sci1,                 /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize                   /* Request file size, dev_size. */
};

/*@}*/

/*
 * Peripheral GPIO Configuration
 */
#define TXD_PORT        SCI1_TXD_PORT
#define TXD_PIN         SCI1_TXD_PIN
#define TXD_PERIPHERAL  SCI1_TXD_PERIPHERAL

#define RXD_PORT        SCI1_RXD_PORT
#define RXD_PIN         SCI1_RXD_PIN
#define RXD_PERIPHERAL  SCI1_RXD_PERIPHERAL


/*
 * Global Variables
 */
#define sig_sci_rx	sig_SCI1_RX
#define sig_sci_tx	sig_SCI1_TX
#define dcb_sci		dcb_sci1
#define reg_sci		reg_sci1

#include "mcf5_sci.c"
