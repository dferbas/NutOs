/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
 * Copyright (C) 2012 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de
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
#ifdef STM32_SPI2
/*
 * \verbatim
 * $Id: stm32_spi2.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <sys/timer.h>
#include <cfg/spi.h>
#include <cfg/arch/gpio.h>
#include <dev/spibus.h>
#include <dev/gpio.h>

#include <arch/cm3/stm/stm32xxxx_rcc.h>
#include <arch/cm3/stm/stm32xxxx_gpio.h>
#if defined(MCU_STM32F1)
#include <arch/cm3/stm/stm32f1_dma.h>
#endif
#include <arch/cm3/stm/stm32_spi.h>
#include <dev/irqreg.h>
#include <sys/event.h>
#include <sys/nutdebug.h>

#include <stdlib.h>
#include <errno.h>

/* Handle the PIN remap possibilities
 * F1:    NSS:  PB12
 *        SCK:  PB13
 *        MISO: PB14
 *        MOSI: PB15
 *  L1:   NSS:  PB12/PD0
 *        SCK:  PB13/PD1
 *        MISO: PB14/PD3
 *        MOSI: PB15/PD4
 * F2/F4: NSS:  PB12/PB9/PI0
 *        SCK:  PB13/PB10/PI1
 *        MISO: PB14/PC2/PI2
 *        MOSI: PB15/PC3/PI3
 *
 * For Chip select, we use NSS pin as default or any other pin as pure GPIO
*/

#if !defined(SPIBUS2_CS_PORT)
 #define SPIBUS_CS_PORT NUTGPIO_PORTB
#else
 #define SPIBUS_CS_PORT SPIBUS2_CS_PORT
#endif
#if !defined(SPIBUS2_CS_PIN)
 #define SPIBUS_CS_PIN 12
#else
 #define SPIBUS_CS_PIN SPIBUS2_CS_PIN
#endif
#if defined(MCU_STM32F1)
 #undef SPIBUS_REMAP_BB
 #define SPIBUS_SCK_PIN 13
 #define SPIBUS_SCK_PORT NUTGPIO_PORTB
 #define SPIBUS_MISO_PIN 14
 #define SPIBUS_MISO_PORT NUTGPIO_PORTB
 #define SPIBUS_MOSI_PIN 15
 #define SPIBUS_MOSI_PORT NUTGPIO_PORTB
#else
 #if !defined(SPIBUS2_SCK_PIN)
  #define SPIBUS_SCK_PORT NUTGPIO_PORTB
  #define SPIBUS_SCK_PIN 13
 #elif SPIBUS2_SCK_PIN == 13
  #define SPIBUS_SCK_PORT NUTGPIO_PORTB
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F4)) && SPIBUS2_SCK_PIN == 10
  #define SPIBUS_SCK_PORT NUTGPIO_PORTB
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F4)) && SPIBUS2_SCK_PIN == 1
  #define SPIBUS_SCK_PORT NUTGPIO_PORTD
 #elif defined(MCU_STM32L1) && SPIBUS2_SCK_PIN == 1
  #define SPIBUS_SCK_PORT NUTGPIO_PORTD
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F4)) && SPIBUS2_SCK_PIN == 901
  #undef SPIBUS2_SCK_PIN
  #define SPIBUS_SCK_PIN 1
  #define SPIBUS_SCK_PORT NUTGPIO_PORTI
 #else
  #warning "Illegal I2C2 SCK pin assignement"
 #endif
 #if !defined(SPIBUS2_MISO_PIN)
  #define SPIBUS_MISO_PIN 14
  #define SPIBUS_MISO_PORT NUTGPIO_PORTB
 #elif  SPIBUS2_MISO_PIN == 14
  #define SPIBUS_MISO_PORT NUTGPIO_PORTB
 #elif defined(MCU_STM32L1) && SPIBUS2_MISO_PIN == 3
  #define SPIBUS_MISO_PORT NUTGPIO_PORTD
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F4)) && SPIBUS2_MISO_PIN == 2
  #define SPIBUS_MISO_PORT NUTGPIO_PORTC
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F4)) && SPIBUS2_MISO_PIN == 902
  #undef SPIBUS2_MISO_PIN
  #define SPIBUS_MISO_PIN 2
  #define SPIBUS_MISO_PORT NUTGPIO_PORTI
 #else
  #warning "Illegal I2C2 MISO pin assignement"
 #endif
 #if !defined(SPIBUS2_MOSI_PIN)
  #define SPIBUS_MOSI_PIN 15
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTB
 #elif SPIBUS2_MOSI_PIN == 15
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTB
 #elif defined(MCU_STM32L1) && SPIBUS2_MOSI_PIN == 4
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTD
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F4)) && SPIBUS2_MOSI_PIN == 3
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTC
 #elif (defined(MCU_STM32F2) || defined(MCU_STM32F4)) && SPIBUS2_MOSI_PIN == 903
  #undef SPIBUS2_MOSI_PIN
  #define SPIBUS_MOSI_PIN 3
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTI
 #else
  #warning "Illegal I2C2 MOSI pin assignement"
 #endif
#endif
#define SPI_DEV 2
#define SPI_GPIO_AF GPIO_AF_SPI2
#define SPI_ENABLE_CLK (RCC->APB1ENR |= RCC_APB1Periph_SPI2)


/*Dma Channels
  * DMA1.2 - spi1_rx        DMA1.3 - spi1_tx
  * DMA1.4 - spi2_rx (I2c2_tx)  DMA1.5 - spi2_tx (i2c2_rx)
  * DMA1.6 - i2c1_tx        DMA1.7 - i2c1_rx
  * DMA2.1 - spi3_rx        DMA2.2 - spi3_tx
  */
//static HANDLE spi2_que;

//#define SPI_QUE spi2_que

NUTSPIBUS spiBus1Stm32 = {
    NULL,                       /*!< Bus mutex semaphore (bus_mutex). */
    NULL,                       /*!< Bus ready signal (bus_ready). */
    SPI2_BASE,                  /*!< Bus base address (bus_base). */
    NULL,                       /*!< Bus interrupt handler (bus_sig). */
    Stm32SpiBusNodeInit,         /*!< Initialize the bus (bus_initnode). */
    Stm32SpiBusSelect,          /*!< Select the specified device (bus_alloc). */
    Stm32SpiBusDeselect,        /*!< Deselect the specified device (bus_release). */
    Stm32SpiBusTransfer,
    NutSpiBusWait,
    NutSpiBusSetMode,           /*!< Set SPI mode of a specified device (bus_set_mode). */
    NutSpiBusSetRate,           /*!< Set clock rate of a specified device (bus_set_rate). */
    NutSpiBusSetBits            /*!< Set number of data bits of a specified device (bus_set_bits). */
};

#include "stm32_spi.c"
#endif /* STM32_SPI2 */
