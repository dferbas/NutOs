
#include <cfg/arch.h>
#include <dev/usart.h>
#include <dev/gpio.h>

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
static struct _USART_CONTROL_REGISTER {
    uint8_t uimr;   /* UISR/UIMR register, write from one, red from other => write only */
    uint8_t ubg1;   /* Write only */
    uint8_t ubg2;   /* Write only */
    uint8_t umr1;   /* Write/ Read after RESET MODE REGISTER POINTER */
    uint8_t umr2;   /* Write/ Read after Write/ Read from UMR1 register */

} usartControlRegister ;

/*!
 * \brief USART0 device control block structure.
 */
static USARTDCB dcb_usart0 = {
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
    McfUsartSetParity,         	/* dcb_set_parity */
    McfUsartGetParity,          /* dcb_get_parity */
    McfUsartSetStopBits,        /* dcb_set_stop_bits */
    McfUsartGetStopBits,        /* dcb_get_stop_bits */
    McfUsartSetStatus,          /* dcb_set_status */
    McfUsartGetStatus,          /* dcb_get_status */
    0,       					/* dcb_set_clock_mode */
    0,       					/* dcb_get_clock_mode */
};

/*!
 * \name mcf5xxxx USART0 Device
 */
/*@{*/
/*!
 * \brief mcf5xxxx device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the AVR's on-chip USART0.
 *
 * The device is named \b uart0.
 *
 * \showinitializer
 */
NUTDEVICE devUartOldMcf5_0 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 'a', 'r', 't', '0', 0, 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    0,                          /* Base address, dev_base. */
    0,                          /* First interrupt number, dev_irq (not used). */
    0,                          /* Interface control block, dev_icb (not used). */
    &dcb_usart0,                /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    //UsartWrite_P,               /* Write data from program space to device, dev_write_P. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize                   /* Request file size, dev_size. */
};

/* USART0 Registers */
#define MCF_UARTn_UMR1   MCF_UART_UMR(0)
#define MCF_UARTn_UMR2   MCF_UART_UMR(0)
#define MCF_UARTn_USR    MCF_UART_USR(0)
#define MCF_UARTn_UCSR   MCF_UART_UCSR(0)
#define MCF_UARTn_UCR    MCF_UART_UCR(0)
#define MCF_UARTn_URB    MCF_UART_URB(0)
#define MCF_UARTn_UTB    MCF_UART_UTB(0)
#define MCF_UARTn_UIPCR  MCF_UART_UIPCR(0)
#define MCF_UARTn_UACR   MCF_UART_UACR(0)
#define MCF_UARTn_UIMR   MCF_UART_UIMR(0)
#define MCF_UARTn_UISR   MCF_UART_UISR(0)
#define MCF_UARTn_UBG1   MCF_UART_UBG1(0)
#define MCF_UARTn_UBG2   MCF_UART_UBG2(0)
#define MCF_UARTn_UIP    MCF_UART_UIP(0)
#define MCF_UARTn_UOP1   MCF_UART_UOP1(0)
#define MCF_UARTn_UOP0   MCF_UART_UOP0(0)

/* USART1 Interrupt Handler */
#define sig_UART	    sig_UART0

/* Define Interrupt Level */
#define IH_USART_LEVEL 	IH_USART0_LEVEL

#define SIG_UART_RECV   SIG_UART0_RECV
#define SIG_UART_DATA   SIG_UART0_DATA
#define SIG_UART_TRANS  SIG_UART0_TRANS

/* USART0 device control structure */
#define dcb_usart   dcb_usart0

#define MCF_GPIO_PUAPAR_URXD0_URXD0          (0x4)
#define MCF_GPIO_PUAPAR_UTXD0_UTXD0          (0x1)
#define MCF_GPIO_PUAPAR_PUAPAR2(x)           (((x)&0x3)<<0x4)
#define MCF_GPIO_PUAPAR_PUAPAR3(x)           (((x)&0x3)<<0x6)
#define MCF_GPIO_DDRUA_DDRUA2                (0x4)
#define MCF_GPIO_DDRUA_DDRUA3                (0x8)
#define MCF_GPIO_PORTUA_PORTUA2              (0x4)
#define MCF_GPIO_PORTUA_PORTUA3              (0x8)
#define MCF_GPIO_PANPAR_PANPAR1              (0x2)
#define MCF_GPIO_PANPAR_PANPAR2              (0x4)
#define MCF_GPIO_DDRAN_DDRAN1                (0x2)
#define MCF_GPIO_DDRAN_DDRAN2                (0x4)
#define MCF_GPIO_PORTAN_PORTAN1              (0x2)
#define MCF_GPIO_PORTAN_PORTAN2              (0x4)
#define MCF_GPIO_PUAPAR_UCTS0_UCTS0          (0x40)

/* GPIO */
#define MCF_GPIO_PUnPAR		  		MCF_GPIO_PAR8(PORTUA)
#define MCF_GPIO_PUnPAR_URXDn 		MCF_GPIO_PUAPAR_URXD0_URXD0
#define MCF_GPIO_PUnPAR_UTXDn 		MCF_GPIO_PUAPAR_UTXD0_UTXD0

#if PLATFORM == SM2_MU
	/* 485 chip 1 */
	#define MCF_GPIO_PAR_CHIP1	  		MCF_GPIO_PAR8(PORTUA)
	#define MCF_GPIO_DDR_CHIP1      	MCF_GPIO_DDR(PORTUA)
	#define MCF_GPIO_PORT_CHIP1     	MCF_GPIO_PORT(PORTUA)

	#define MCF_GPIO_PAR_RE1		    MCF_GPIO_PUAPAR_PUAPAR2
	#define MCF_GPIO_PAR_DE1		    MCF_GPIO_PUAPAR_PUAPAR3
	#define MCF_GPIO_DDR_RE1		    MCF_GPIO_DDRUA_DDRUA2
	#define MCF_GPIO_DDR_DE1		    MCF_GPIO_DDRUA_DDRUA3
	#define MCF_GPIO_PORT_RE1		    MCF_GPIO_PORTUA_PORTUA2
	#define MCF_GPIO_PORT_DE1     		MCF_GPIO_PORTUA_PORTUA3

	/* 485 chip 2 */
	#define MCF_GPIO_PAR_CHIP2      	MCF_GPIO_PAR8(PORTAN)
	#define MCF_GPIO_DDR_CHIP2      	MCF_GPIO_DDR(PORTAN)
	#define MCF_GPIO_PORT_CHIP2     	MCF_GPIO_PORT(PORTAN)

	#define MCF_GPIO_PAR_RE2		    MCF_GPIO_PANPAR_PANPAR1
	#define MCF_GPIO_PAR_DE2		    MCF_GPIO_PANPAR_PANPAR2
	#define MCF_GPIO_DDR_RE2		    MCF_GPIO_DDRAN_DDRAN1
	#define MCF_GPIO_DDR_DE2		    MCF_GPIO_DDRAN_DDRAN2
	#define MCF_GPIO_PORT_RE2		    MCF_GPIO_PORTAN_PORTAN1
	#define MCF_GPIO_PORT_DE2     		MCF_GPIO_PORTAN_PORTAN2

#endif

/* RTS, CTS flow control */
#define MCF_GPIO_RCTS_PAR		MCF_GPIO_PAR8(PORTUA)
#define MCF_GPIO_RCTS_DDR		MCF_GPIO_DDR(PORTUA)
#define MCF_GPIO_RCTS_PORT		MCF_GPIO_PORT(PORTUA)

#define MCF_GPIO_RCTS_PAR_RTS	MCF_GPIO_PUAPAR_PUAPAR2
#define MCF_GPIO_RCTS_DDR_RTS	MCF_GPIO_DDRUA_DDRUA2
#define MCF_GPIO_RCTS_PORT_RTS	MCF_GPIO_PORTUA_PORTUA2
#define MCF_GPIO_RCTS_PAR_CTS	MCF_GPIO_PUAPAR_UCTS0_UCTS0

#ifdef NUTTRACER
#define TRACE_INT_UART_CTS TRACE_INT_UART0_CTS
#define TRACE_INT_UART_RXCOMPL TRACE_INT_UART0_RXCOMPL
#define TRACE_INT_UART_TXEMPTY TRACE_INT_UART0_TXEMPTY
#endif

#ifdef UART0_READMULTIBYTE
#define UART_READMULTIBYTE
#endif

#ifdef USE_USART0
#define USE_USART
#endif

#ifdef UART0_NO_SW_FLOWCONTROL
#define UART_NO_SW_FLOWCONTROL
#endif

#if PLATFORM == SM2_MU
	#ifndef UART_HDX_BIT
	#define UART_HDX_BIT
	#endif

	#ifndef UART_HDB_FDX_BIT
	#define UART_HDB_FDX_BIT
	#endif
#elif PLATFORM == POSEIDON
	#define UART_RTS_BIT
	#define UART_CTS_BIT
#endif

#include "mcf5225_old_uart.c"

int McfUsartResetRx0(void)
{
	MCF_UARTn_UCR = MCF_UART_UCR_RESET_RX;

	return 0;
}

