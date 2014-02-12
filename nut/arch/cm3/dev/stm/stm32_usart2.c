/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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
 *
 */

/*!
 *\verbatim
 * $Id: stm32_usart2.c 4608 2012-09-14 13:14:15Z haraldkipp $
 *\endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/uart.h>
#include <cfg/arch/gpio.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/usart.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32xxxx_gpio.h>
#include <arch/cm3/stm/stm32xxxx_rcc.h>
#include <arch/cm3/stm/stm32_usart.h>

/*!
 * \addtogroup xgNutArchCm3Stm32
 */
/*@{*/

/*!
 * \brief USART2 device control block structure.
 */
static USARTDCB dcb_usart2 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    { 0,0,0,0,0,0,0,0,0},       /* dcb_tx_rbf */
    { 0,0,0,0,0,0,0,0,0},       /* dcb_rx_rbf */
    0,                          /* dcb_last_eol */
    Stm32UsartInit,             /* dcb_init */
    Stm32UsartDeinit,           /* dcb_deinit */
    Stm32UsartTxStart,          /* dcb_tx_start */
    Stm32UsartRxStart,          /* dcb_rx_start */
    Stm32UsartSetFlowControl,   /* dcb_set_flow_control */
    Stm32UsartGetFlowControl,   /* dcb_get_flow_control */
    Stm32UsartSetSpeed,         /* dcb_set_speed */
    Stm32UsartGetSpeed,         /* dcb_get_speed */
    Stm32UsartSetDataBits,      /* dcb_set_data_bits */
    Stm32UsartGetDataBits,      /* dcb_get_data_bits */
    Stm32UsartSetParity,        /* dcb_set_parity */
    Stm32UsartGetParity,        /* dcb_get_parity */
    Stm32UsartSetStopBits,      /* dcb_set_stop_bits */
    Stm32UsartGetStopBits,      /* dcb_get_stop_bits */
    Stm32UsartSetStatus,        /* dcb_set_status */
    Stm32UsartGetStatus,        /* dcb_get_status */
    Stm32UsartSetClockMode,     /* dcb_set_clock_mode */
    Stm32UsartGetClockMode,     /* dcb_get_clock_mode */
};

/*!
 * \name STM32 USART2 Device
 */
/*@{*/
/*!
 * \brief USART2 device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the STM32s on-chip USART2.
 *
 * The device is named \b uart2.
 *
 * \showinitializer
 */
NUTDEVICE devUsartStm32_2 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 's', 'a', 'r', 't', '2', 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    USART2_BASE,                /* Base address, dev_base. */
    USART2_IRQn,                /* First interrupt number, dev_irq. */
    NULL,                       /* Interface control block, dev_icb. */
    &dcb_usart2,                /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize                   /* Request file size, dev_size. */
};

/*@}*/

/*!
 * \brief USART2 GPIO configuartion and assignment.
 */
/* F1  NOREMAP REMAP
 * L1/F2/F4
 * CTS PA0     PD3
 * RTS PA1     PD4
 * TX  PA2     PD5
 * RX  PA3     PD6
 * CK  PA4     PD7
 */


#if defined(MCU_STM32F1)
 #define STM_USART_REMAP_MASK AFIO_MAPR_USART2_REMAP
 #if defined(USART2_REMAP_USART)
  #define STM_USART_REMAP_VALUE AFIO_MAPR_USART2_REMAP
  #define TX_GPIO_PORT  NUTGPIO_PORTD
  #define TX_GPIO_PIN   5
  #define RX_GPIO_PORT  NUTGPIO_PORTD
  #define RX_GPIO_PIN   6
  #ifdef USART2_HARDWARE_HANDSHAKE
   #define RTS_GPIO_PORT NUTGPIO_PORTD
   #define RTS_GPIO_PIN  4
   #define CTS_GPIO_PORT NUTGPIO_PORTD
   #define CTS_GPIO_PIN  3
  #endif /* USART2_HARDWARE_HANDSHAKE */
 #else
  #define STM_USART_REMAP_VALUE DISABLE
  #define TX_GPIO_PORT  NUTGPIO_PORTA
  #define TX_GPIO_PIN   2
  #define RX_GPIO_PORT  NUTGPIO_PORTA
  #define RX_GPIO_PIN   3
  #ifdef USART2_HARDWARE_HANDSHAKE
   #define RTS_GPIO_PORT NUTGPIO_PORTA
   #define RTS_GPIO_PIN  1
   #define CTS_GPIO_PORT NUTGPIO_PORTA
   #define CTS_GPIO_PIN  0
  #endif /* USART2_HARDWARE_HANDSHAKE */
 #endif /* USART2_REMAP_USART */
#else /*L1/F2/F4*/
 #define STM_USART_REMAP  GPIO_AF_USART2
 #if !defined(USART2_TX_PIN)
  #if defined(USART2_REMAP_USART)
   #define TX_GPIO_PORT  NUTGPIO_PORTD
   #define TX_GPIO_PIN   5
  #else
   #define TX_GPIO_PORT  NUTGPIO_PORTA
   #define TX_GPIO_PIN   2
  #endif
 #elif USART2_TX_PIN == 5
  #define TX_GPIO_PORT  NUTGPIO_PORTD
  #define TX_GPIO_PIN   5
 #elif USART2_TX_PIN == 2
  #define TX_GPIO_PORT  NUTGPIO_PORTA
  #define TX_GPIO_PIN   2
 #else
  #warning "Illegal USART2 TX pin assignement"
 #endif
 #if !defined(USART2_RX_PIN)
  #if defined(USART2_REMAP_USART)
   #define RX_GPIO_PORT  NUTGPIO_PORTD
   #define RX_GPIO_PIN   6
  #else
   #define RX_GPIO_PORT  NUTGPIO_PORTA
   #define RX_GPIO_PIN   3
  #endif
 #elif USART2_RX_PIN == 6
  #define RX_GPIO_PORT  NUTGPIO_PORTD
  #define RX_GPIO_PIN   6
 #elif USART2_RX_PIN == 3
  #define RX_GPIO_PORT  NUTGPIO_PORTA
  #define RX_GPIO_PIN   3
 #else
  #warning "Illegal USART2 RX pin assignement"
 #endif
 #ifdef USART2_HARDWARE_HANDSHAKE
  #if !defined(USART2_CTS_PIN)
   #if defined(USART2_REMAP_USART)
    #define CTS_GPIO_PORT  NUTGPIO_PORTD
    #define CTS_GPIO_PIN   3
   #else
    #define CTS_GPIO_PORT  NUTGPIO_PORTA
    #define CTS_GPIO_PIN   0
   #endif
  #elif USART2_CTS_PIN == 3
   #define CTS_GPIO_PORT  NUTGPIO_PORTD
   #define CTS_GPIO_PIN   3
  #elif USART2_CTS_PIN == 0
    #define CTS_GPIO_PORT  NUTGPIO_PORTA
    #define CTS_GPIO_PIN   0
  #else
   #warning "Illegal USART2 CTS pin assignement"
  #endif
  #if !defined(USART2_RTS_PIN)
   #if defined(USART2_REMAP_USART)
    #define RTS_GPIO_PORT  NUTGPIO_PORTD
    #define RTS_GPIO_PIN   4
   #else
    #define RTS_GPIO_PORT  NUTGPIO_PORTA
    #define RTS_GPIO_PIN   1
   #endif
  #elif USART2_CTS_PIN == 4
   #define RTS_GPIO_PORT  NUTGPIO_PORTD
   #define RTS_GPIO_PIN   4
  #elif USART2_CTS_PIN == 1
   #define RTS_GPIO_PORT  NUTGPIO_PORTA
   #define RTS_GPIO_PIN   1
  #else
   #warning "Illegal USART2 RTS pin assignement"
  #endif
 #endif
#endif

#ifdef USART2_RS485_CTRL
#define USART_485_CTRL
#ifdef USART2_485RE_INV
#define USART_4485RE_INV
#endif
#ifdef USART2_485DE_INV
#define USART_4485DE_INV
#endif
#if defined(USART2_485DE_PORT) && defined(USART2_485DE_PIN)
#define DE_GPIO_BASE  GPIO_ID2GPIO(USART2_485DE_PORT)
#define DE_GPIO_PORT  USART2_485DE_PORT
#define DE_GPIO_PIN   USART2_485DE_PIN
#endif
#if defined(USART2_485RE_PORT) && defined(USART2_485RE_PIN)
#define NRE_GPIO_BASE  GPIO_ID2GPIO(USART2_485RE_PORT)
#define NRE_GPIO_PORT  USART2_485RE_PORT
#define NRE_GPIO_PIN   USART2_485RE_PIN
#endif
#endif /* USART2_RS485_CTRL */

/*!
 * \brief USART2 base configuration.
 */
#define STM_USART_CLK   RCC_APB1Periph_USART2

#ifdef USART2_INIT_BAUDRATE
#define USART_INIT_BAUTRATE USART2_INIT_BAUDRATE
#endif

#ifdef USART2_HARDWARE_HANDSHAKE
  #define USART_HWFLOWCTRL USART_HardwareFlowControl_RTS_CTS
  #define US_MODE_HWHANDSHAKE
#else
  #define USART_HWFLOWCTRL USART_HardwareFlowControl_None
  #undef  US_MODE_HWHANDSHAKE
#endif

#ifdef USART2_XONXOFF_CONTROL
#define USART_XONXOFF_CONTROL
#else
#undef  USART_XONXOFF_CONTROL
#endif

#ifdef USART2_MODE_IRDA
#define USART_MODE_IRDA
#else
#undef  USART_MODE_IRDA
#endif

#ifdef USART2_HARDWARE_HDX
#define USART_HARDWARE_HDX
#else
#undef USART_HARDWARE_HDX
#endif

#ifdef USART2_SUPPORT_DMA
 #if defined(MCU_STM32F1)||defined(MCU_STM32L1)
  #define UART_DMA_TXCHANNEL  DMA1_C7
  #define UART_DMA_RXCHANNEL  DMA1_C6
 #elif  defined(MCU_STM32F2)||defined(MCU_STM32F4)
  #define UART_DMA_TXCHANNEL  DMA_CONTROL0 | DMA_STREAM6 | DMA_CHANNEL4
  #define UART_DMA_RXCHANNEL  DMA_CONTROL0 | DMA_STREAM5 | DMA_CHANNEL4
 #else
  #warning "STM32 family has no implemented DMA"
 #endif
#else
#undef UART_DMA_TXCHANNEL
#undef UART_DMA_RXCHANNEL
#endif


#define USARTn      USART2
#define USARTnBase  USART2_BASE
#define USARTirqn   USART2_IRQn
#define USARTclk    NUT_HWCLK_PCLK2
#define UART_DR_PTR (uint32_t*)(USARTnBase+4)

#define SigUSART sig_USART2
#define DcbUSART dcb_usart2

/*@}*/
#include "stm32_usart.c"
