
#include <cfg/os.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>

#include <dev/mcf5xxxx_usart.h>

/*
 * Local function prototypes.
 */
static uint32_t McfUsartGetSpeed(void);
static int McfUsartSetSpeed(uint32_t rate);
static uint8_t McfUsartGetDataBits(void);
static int McfUsartSetDataBits(uint8_t bits);
static uint8_t McfUsartGetParity(void);
static int McfUsartSetParity(uint8_t mode);
static uint8_t McfUsartGetStopBits(void);
static int McfUsartSetStopBits(uint8_t bits);
static uint32_t McfUsartGetStatus(void);
static int McfUsartSetStatus(uint32_t flags);
static uint32_t McfUsartGetFlowControl(void);
static int McfUsartSetFlowControl(uint32_t flags);
static void McfUsartTxStart(void);
static void McfUsartRxStart(void);
static int McfUsartInit(void);
static int McfUsartDeinit(void);

/*
 * brief USART control structure used for write only registers.
 */
static USART_CONTROL_REGISTER usartControlRegister;

/*!
 * \brief USART1 device control block structure.
 */
static USARTDCB dcb_usart1 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    {0, 0, 0, 0, 0, 0, 0, 0},   /* dcb_tx_rbf */
    {0, 0, 0, 0, 0, 0, 0, 0},   /* dcb_rx_rbf */
    0,                          /* dbc_last_eol */
    McfUsartInit,               /* dcb_init */
    McfUsartDeinit,             /* dcb_deinit */
    McfUsartTxStart,            /* dcb_tx_start */
    McfUsartRxStart,            /* dcb_rx_start */
    McfUsartSetFlowControl,     /* dcb_set_flow_control */
    McfUsartGetFlowControl,     /* dcb_get_flow_control */
    McfUsartSetSpeed,           /* dcb_set_speed */
    McfUsartGetSpeed,           /* dcb_get_speed */
    McfUsartSetDataBits,        /* dcb_set_data_bits */
    McfUsartGetDataBits,        /* dcb_get_data_bits */
    McfUsartSetParity,          /* dcb_set_parity */
    McfUsartGetParity,          /* dcb_get_parity */
    McfUsartSetStopBits,        /* dcb_set_stop_bits */
    McfUsartGetStopBits,        /* dcb_get_stop_bits */
    McfUsartSetStatus,          /* dcb_set_status */
    McfUsartGetStatus,          /* dcb_get_status */
    0,       					/* dcb_set_clock_mode */
    0,       					/* dcb_get_clock_mode */
};

/*!
 * \name mcf5xxxx USART1 Device
 */
/*@{*/
/*!
 * \brief mcf5xxxx device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the AVR's on-chip USART1.
 *
 * The device is named \b uart1.
 *
 * \showinitializer
 */
NUTDEVICE devUsartOldMcf1 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 'a', 'r', 't', '1', 0, 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    1,                          /* Base address, dev_base. */
    0,                          /* First interrupt number, dev_irq (not used). */
    0,                          /* Interface control block, dev_icb (not used). */
    &dcb_usart1,                /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    //UsartWrite_P,               /* Write data from program space to device, dev_write_P. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize                   /* Request file size, dev_size. */
};


/* USART1 Registers */
#define MCF_UARTn_UMR1   MCF_UART1_UMR1
#define MCF_UARTn_UMR2   MCF_UART1_UMR2
#define MCF_UARTn_USR    MCF_UART1_USR
#define MCF_UARTn_UCSR   MCF_UART1_UCSR
#define MCF_UARTn_UCR    MCF_UART1_UCR
#define MCF_UARTn_URB    MCF_UART1_URB
#define MCF_UARTn_UTB    MCF_UART1_UTB
#define MCF_UARTn_UIPCR  MCF_UART1_UIPCR
#define MCF_UARTn_UACR   MCF_UART1_UACR
#define MCF_UARTn_UIMR   MCF_UART1_UIMR
#define MCF_UARTn_UISR   MCF_UART1_UISR
#define MCF_UARTn_UBG1   MCF_UART1_UBG1
#define MCF_UARTn_UBG2   MCF_UART1_UBG2
#define MCF_UARTn_UIP    MCF_UART1_UIP
#define MCF_UARTn_UOP1   MCF_UART1_UOP1
#define MCF_UARTn_UOP0   MCF_UART1_UOP0

/* USART1 Interrupt Handler */
#define sig_USART	    sig_USART1

/* Define Interrupt Level */
#define IH_USART_LEVEL 	IH_USART1_LEVEL

#define SIG_UART_RECV   SIG_UART1_RECV
#define SIG_UART_DATA   SIG_UART1_DATA
#define SIG_UART_TRANS  SIG_UART1_TRANS

/* USART1 device control structure */
#define dcb_usart   dcb_usart1

/* GPIO */
#define MCF_GPIO_PUnPAR		  MCF_GPIO_PUBPAR
#define MCF_GPIO_PUnPAR_URXDn MCF_GPIO_PUBPAR_URXD1_URXD1
#define MCF_GPIO_PUnPAR_UTXDn MCF_GPIO_PUBPAR_UTXD1_UTXD1

/* 485 chip 1 */
#define MCF_GPIO_PAR_CHIP1	  	MCF_GPIO_PUBPAR
#define MCF_GPIO_DDR_CHIP1      MCF_GPIO_DDRUB
#define MCF_GPIO_PORT_CHIP1     MCF_GPIO_PORTUB

#define MCF_GPIO_PAR_RE1    	MCF_GPIO_PUBPAR_PUBPAR2
#define MCF_GPIO_PAR_DE1     	MCF_GPIO_PUBPAR_PUBPAR3
#define MCF_GPIO_DDR_RE1       	MCF_GPIO_DDRUB_DDRUB2
#define MCF_GPIO_DDR_DE1       	MCF_GPIO_DDRUB_DDRUB3
#define MCF_GPIO_PORT_RE1     	MCF_GPIO_PORTUB_PORTUB2
#define MCF_GPIO_PORT_DE1     	MCF_GPIO_PORTUB_PORTUB3

#ifdef NUTTRACER
#define TRACE_INT_UART_CTS TRACE_INT_UART1_CTS
#define TRACE_INT_UART_RXCOMPL TRACE_INT_UART1_RXCOMPL
#define TRACE_INT_UART_TXEMPTY TRACE_INT_UART1_TXEMPTY
#endif

#ifdef UART1_READMULTIBYTE
#define UART_READMULTIBYTE
#endif

#ifdef USE_USART1
#define USE_USART
#endif

#ifdef UART1_NO_SW_FLOWCONTROL
#define UART_NO_SW_FLOWCONTROL
#endif

#ifndef UART_HDX_BIT
#define UART_HDX_BIT
#endif

#include "mcf5_old_uart.c"

