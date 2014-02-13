
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
 * \brief USART2 device control block structure.
 */
static USARTDCB dcb_usart2 = {
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
 * \name mcf5xxxx USART2 Device
 */
/*@{*/
/*!
 * \brief mcf5xxxx device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the AVR's on-chip USART2.
 *
 * The device is named \b uart2.
 *
 * \showinitializer
 */
NUTDEVICE devUsartOldMcf2 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 'a', 'r', 't', '2', 0, 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    2,                          /* Base address, dev_base. */
    0,                          /* First interrupt number, dev_irq (not used). */
    0,                          /* Interface control block, dev_icb (not used). */
    &dcb_usart2,                /* Driver control block, dev_dcb. */
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
#define MCF_UARTn_UMR1   MCF_UART_UMR(2)
#define MCF_UARTn_UMR2   MCF_UART_UMR(2)
#define MCF_UARTn_USR    MCF_UART_USR(2)
#define MCF_UARTn_UCSR   MCF_UART_UCSR(2)
#define MCF_UARTn_UCR    MCF_UART_UCR(2)
#define MCF_UARTn_URB    MCF_UART_URB(2)
#define MCF_UARTn_UTB    MCF_UART_UTB(2)
#define MCF_UARTn_UIPCR  MCF_UART_UIPCR(2)
#define MCF_UARTn_UACR   MCF_UART_UACR(2)
#define MCF_UARTn_UIMR   MCF_UART_UIMR(2)
#define MCF_UARTn_UISR   MCF_UART_UISR(2)
#define MCF_UARTn_UBG1   MCF_UART_UBG1(2)
#define MCF_UARTn_UBG2   MCF_UART_UBG2(2)
#define MCF_UARTn_UIP    MCF_UART_UIP(2)
#define MCF_UARTn_UOP1   MCF_UART_UOP1(2)
#define MCF_UARTn_UOP0   MCF_UART_UOP0(2)

/* USART2 Interrupt Handler */
#define sig_UART	    sig_UART2

/* Define Interrupt Level */
#define IH_USART_LEVEL 	IH_USART2_LEVEL

#define SIG_UART_RECV   SIG_UART2_RECV
#define SIG_UART_DATA   SIG_UART2_DATA
#define SIG_UART_TRANS  SIG_UART2_TRANS

/* USART2 device control structure */
#define dcb_usart   dcb_usart2

#define MCF_GPIO_PUCPAR_URXD2_URXD2          (0x4)
#define MCF_GPIO_PUCPAR_UTXD2_UTXD2          (0x1)
#define MCF_GPIO_DDRUC_DDRUC2                (0x4)
#define MCF_GPIO_PORTUC_PORTUC2              (0x4)
#define MCF_GPIO_PUCPAR_UCTS2_UCTS2          (0x40)

/* GPIO */
#define MCF_GPIO_PUnPAR		  MCF_GPIO_PAR8(PORTUC)
#define MCF_GPIO_PUnPAR_URXDn MCF_GPIO_PUCPAR_URXD2_URXD2
#define MCF_GPIO_PUnPAR_UTXDn MCF_GPIO_PUCPAR_UTXD2_UTXD2
#define MCF_GPIO_PUCPAR_PUCPAR2(x)           (((x)&0x3)<<0x4)

#define UART_RTS_BIT
#define UART_CTS_BIT

#define MCF_GPIO_RCTS_PAR		MCF_GPIO_PAR8(PORTUC)
#define MCF_GPIO_RCTS_DDR		MCF_GPIO_DDR(PORTUC)
#define MCF_GPIO_RCTS_PORT		MCF_GPIO_PORT(PORTUC)

#define MCF_GPIO_RCTS_PAR_RTS	MCF_GPIO_PUCPAR_PUCPAR2
#define MCF_GPIO_RCTS_DDR_RTS	MCF_GPIO_DDRUC_DDRUC2
#define MCF_GPIO_RCTS_PORT_RTS	MCF_GPIO_PORTUC_PORTUC2
#define MCF_GPIO_RCTS_PAR_CTS	MCF_GPIO_PUCPAR_UCTS2_UCTS2

#ifdef NUTTRACER
#define TRACE_INT_UART_CTS TRACE_INT_UART2_CTS
#define TRACE_INT_UART_RXCOMPL TRACE_INT_UART2_RXCOMPL
#define TRACE_INT_UART_TXEMPTY TRACE_INT_UART2_TXEMPTY
#endif

#ifdef UART2_READMULTIBYTE
#define UART_READMULTIBYTE
#endif

#ifdef USE_USART2
#define USE_USART
#endif

#ifdef UART2_NO_SW_FLOWCONTROL
#define UART_NO_SW_FLOWCONTROL
#endif

#ifdef UART_HDX_BIT
#define UART_HDX_BIT
#endif

/* defined variables and macros for this include */
#include "mcf5225x_old_uart.c"

