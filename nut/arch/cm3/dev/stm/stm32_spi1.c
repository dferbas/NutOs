/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
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

/*
 * \verbatim
 * $Id: stm32_spi1.c 4608 2012-09-14 13:14:15Z haraldkipp $
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
 * F1:    NSS:  PA4/PA15
 *        SCK:  PA5/PB3
 *        MISO: PA6/PB4
 *        MOSI: PA7/PB5
 *  L1:   NSS:  PA4/PA15/PE12
 *        SCK:  PA5/PB3/PE13
 *        MISO: PA6/PA11/PB4/PE14
 *        MOSI: PA7/PA12/PB5/PE15
 * F2/F4: NSS:  PA4/PA15
 *        SCK:  PA5/PB3
 *        MISO: PA6/PB4
 *        MOSI: PA7/PB5
 *
 * For Chip select, we use NSS pin as default or any other pin as pure GPIO
*/

#if !defined(SPIBUS1_CS_PORT)
 #define SPIBUS_CS_PORT NUTGPIO_PORTA
#else
 #define SPIBUS_CS_PORT SPIBUS1_CS_PORT
#endif
#if !defined(SPIBUS1_CS_PIN)
 #if defined(SPIBUS1_REMAP_SPI)
  #define SPIBUS_CS_PIN 15
 #else
  #define SPIBUS_CS_PIN 4
 #endif
#else
#define SPIBUS_CS_PIN SPIBUS1_CS_PIN
#endif
#if defined(MCU_STM32F1)
#define SPIBUS_REMAP_BB CM3BBREG(AFIO_BASE, AFIO_TypeDef, MAPR, _BI32(AFIO_MAPR_SPI1_REMAP))
 #if defined (SPIBUS1_REMAP_SPI)
  #define SPI_DOREMAP ENABLE
  #define SPIBUS_SCK_PIN 3
  #define SPIBUS_SCK_PORT NUTGPIO_PORTB
  #define SPIBUS_MISO_PIN 4
  #define SPIBUS_MISO_PORT NUTGPIO_PORTB
  #define SPIBUS_MOSI_PIN 5
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTB
 #else
  #define SPI_DOREMAP DISABLE
  #define SPIBUS_SCK_PIN 5
  #define SPIBUS_SCK_PORT NUTGPIO_PORTA
  #define SPIBUS_MISO_PIN 6
  #define SPIBUS_MISO_PORT NUTGPIO_PORTA
  #define SPIBUS_MOSI_PIN 7
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTA
 #endif
#else
 #if !defined(SPIBUS1_SCK_PIN)
  #define SPIBUS_SCK_PIN 5
  #define SPIBUS_SCK_PORT NUTGPIO_PORTA
 #elif SPIBUS1_SCK_PIN == 5
  #define SPIBUS_SCK_PORT NUTGPIO_PORTA
 #elif SPIBUS1_SCK_PIN == 3
  #define SPIBUS_SCK_PORT NUTGPIO_PORTB
 #elif defined(MCU_STM32L1) && SPIBUS1_SCK_PIN == 13
  #define SPIBUS_SCK_PORT NUTGPIO_PORTE
 #else
  #warning "Illegal SPI1 SCK pin assignement"
 #endif

 #if !defined(SPIBUS1_MISO_PIN)
  #define SPIBUS_MISO_PIN 6
  #define SPIBUS_MISO_PORT NUTGPIO_PORTA
 #elif SPIBUS1_MISO_PIN == 6
  #define SPIBUS_MISO_PORT NUTGPIO_PORTA
 #elif SPIBUS1_MISO_PIN == 11
  #define SPIBUS_MISO_PORT NUTGPIO_PORTA
 #elif SPIBUS1_MISO_PIN == 4
  #define SPIBUS_MISO_PORT NUTGPIO_PORTB
 #elif defined(MCU_STM32L1) && SPIBUS1_MISO_PIN == 14
  #define SPIBUS_MISO_PORT NUTGPIO_PORTE
 #else
  #warning "Illegal SPI1 MISO pin assignement"
 #endif

 #if !defined(SPIBUS1_MOSI_PIN)
  #define SPIBUS_MOSI_PIN 7
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTA
 #elif SPIBUS1_MOSI_PIN == 7
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTA
 #elif SPIBUS1_MOSI_PIN == 12
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTA
 #elif SPIBUS1_MOSI_PIN == 5
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTB
 #elif defined(MCU_STM32L1) && SSPIBUS1_MOSI_PIN == 15
  #define SPIBUS_MOSI_PORT NUTGPIO_PORTE
 #else
  #warning "Illegal SPI1 MOSI pin assignement"
 #endif
#endif
#define SPI_DEV 1
#define SPI_REMAP GPIO_Remap_SPI1
#define SPI_GPIO_AF GPIO_AF_SPI1
#define SPI_ENABLE_CLK (RCC->APB2ENR |= RCC_APB2Periph_SPI1)

/*Dma Channels
  * DMA1.2 - spi1_rx        DMA1.3 - spi1_tx
  * DMA1.4 - spi2_rx (I2c2_tx)  DMA1.5 - spi2_tx (i2c2_rx)
  * DMA1.6 - i2c1_tx        DMA1.7 - i2c1_rx
  * DMA2.1 - spi3_rx        DMA2.2 - spi3_tx
  */
//static HANDLE spi1_que;

NUTSPIBUS spiBus0Stm32 = {
    NULL,                       /*!< Bus mutex semaphore (bus_mutex). */
    NULL,                       /*!< Bus ready signal (bus_ready). */
    SPI1_BASE,                  /*!< Bus base address (bus_base). */
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
