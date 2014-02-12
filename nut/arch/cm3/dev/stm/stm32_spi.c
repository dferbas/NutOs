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
 * $Id: stm32_spi.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <sys/timer.h>
#include <cfg/spi.h>
#include <cfg/arch/gpio.h>
#include <dev/spibus.h>
#include <dev/gpio.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32xxxx_gpio.h>
#include <arch/cm3/stm/stm32xxxx_rcc.h>
#if defined(MCU_STM32F1)
#include <arch/cm3/stm/stm32f1_dma.h>
#endif
#include <arch/cm3/stm/stm32_spi.h>
#include <dev/irqreg.h>
#include <sys/event.h>
#include <sys/nutdebug.h>

#include <stdlib.h>
#include <errno.h>

/*!
 * \brief Set the specified chip select to a given level.
 */
static int Stm32SpiChipSelect(uint_fast8_t cs, uint_fast8_t hi)
{
    /* Fixme: Check for cs != 0 */
    GpioPinConfigSet(SPIBUS_CS_PORT, SPIBUS_CS_PIN, GPIO_CFG_OUTPUT);
    GpioPinSet(SPIBUS_CS_PORT, SPIBUS_CS_PIN, hi);
    return 0;
}

/*! \brief Deselect a device on the SPI bus.
 *
 * Deactivates the chip select and unlocks the bus.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
static int Stm32SpiBusDeselect(NUTSPINODE * node)
{
    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    NutSpiBusWait(node, NUT_WAIT_INFINITE);
    /* Deactivate the node's chip select. */
    Stm32SpiChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) == 0);

    /* Release the bus. */
    NutEventPostAsync(&node->node_bus->bus_mutex);

    return 0;
}

/*! \brief Select a device on the SPI bus.
 *
 * Locks and activates the bus for the specified node.
 *
 * \param node Specifies the SPI bus node.
 * \param tmo  Timeout in milliseconds. To disable timeout, set this
 *             parameter to NUT_WAIT_INFINITE.
 *
 * \return 0 on success. In case of an error, -1 is returned and the bus
 *         is not locked.
 */
static int Stm32SpiBusSelect(NUTSPINODE * node, uint32_t tmo)
{
    int rc;
    SPI_TypeDef* base;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_stat != NULL);

    base=(SPI_TypeDef*)(node->node_bus->bus_base);

    /* Allocate the bus. */
    rc = NutEventWait(&node->node_bus->bus_mutex, tmo);
    if (rc) {
        errno = EIO;
    } else {
        SPI_TypeDef *spireg = node->node_stat;

        SPI_ENABLE_CLK;
        /* Activate the IO Pins to avoid glitches*/
    GpioPinConfigSet(SPIBUS_SCK_PORT,  SPIBUS_SCK_PIN,  GPIO_CFG_DISABLED);//SCK
    GpioPinConfigSet(SPIBUS_MISO_PORT, SPIBUS_MISO_PIN, GPIO_CFG_DISABLED );//MISO
    GpioPinConfigSet(SPIBUS_MOSI_PORT, SPIBUS_MOSI_PIN, GPIO_CFG_DISABLED);//MOSI

        /* If the mode update bit is set, then update our shadow registers. */
        if (node->node_mode & SPI_MODE_UPDATE) {
            Stm32SpiSetup(node);
        }

        /* Set SPI mode. */
        base->CR1 = spireg->CR1;
        base->CR1 |= SPI_CR1_SSI|SPI_CR1_MSTR;
        base->CR2=spireg->CR2;
#if !defined(STM32L1XX_MD)
        base->I2SCFGR=spireg->I2SCFGR;
        base->I2SPR=spireg->I2SPR;
#endif
        //No enable - set it only during transfer

#if defined(STM32F10X_CL)
 #if defined(SPIBUS_REMAP_BB)
        SPIBUS_REMAP_BB = SPI_DOREMAP;
 #endif
#elif defined (MCU_STM32L1) || defined (MCU_STM32F2) || defined (MCU_STM32F4)
    GPIO_PinAFConfig((GPIO_TypeDef*)SPIBUS_SCK_PORT,  SPIBUS_SCK_PIN,  SPI_GPIO_AF);
    GPIO_PinAFConfig((GPIO_TypeDef*)SPIBUS_MISO_PORT, SPIBUS_MISO_PIN, SPI_GPIO_AF);
    GPIO_PinAFConfig((GPIO_TypeDef*)SPIBUS_MOSI_PORT, SPIBUS_MOSI_PIN, SPI_GPIO_AF);
#endif
    GpioPinConfigSet(SPIBUS_SCK_PORT,  SPIBUS_SCK_PIN,  GPIO_CFG_OUTPUT|GPIO_CFG_PERIPHAL);//SCK
    GpioPinConfigSet(SPIBUS_MISO_PORT, SPIBUS_MISO_PIN,                 GPIO_CFG_PERIPHAL);//MISO
    GpioPinConfigSet(SPIBUS_MOSI_PORT, SPIBUS_MOSI_PIN, GPIO_CFG_OUTPUT|GPIO_CFG_PERIPHAL);//MOSI
        /* Finally activate the node's chip select. */
        rc = Stm32SpiChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) != 0);
        if (rc) {
            /* Release the bus in case of an error. */
            NutEventPost(&node->node_bus->bus_mutex);
        }
    }
    return rc;
}


/*Dma Channels
  * DMA1.2 - spi1_rx        DMA1.3 - spi1_tx
  * DMA1.4 - spi2_rx (I2c2_tx)  DMA1.5 - spi2_tx (i2c2_rx)
  * DMA1.6 - i2c1_tx        DMA1.7 - i2c1_rx
  * DMA2.1 - spi3_rx        DMA2.2 - spi3_tx
  */

static void Stm32SpiEnable(SPI_TypeDef* SPIx){
    SPIx->CR1 |= SPI_CR1_SPE;
};

static void Stm32SpiDisable(SPI_TypeDef* SPIx){
    SPIx->CR1&= ~(SPI_CR1_SPE);
};


/*!
 * \brief Update SPI shadow registers.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
static int Stm32SpiSetup(NUTSPINODE * node)
{
    uint32_t clk;
    uint8_t i;
    uint32_t clkdiv;
    SPI_TypeDef *spireg;

    NUTASSERT(node != NULL);
    NUTASSERT(node->node_stat != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_bus->bus_base != 0);
    spireg = node->node_stat;

    spireg->CR1 &= ~(SPI_CR1_DFF | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR);
    switch(node->node_bits){
        case 8:
            spireg->CR1 &= ~(SPI_CR1_DFF);
            break;
        case 16:
            spireg->CR1 |= SPI_CR1_DFF;
            break;
        default:
            break;
    };
    if (node->node_mode & SPI_MODE_CPOL) {
        spireg->CR1 |= SPI_CR1_CPOL;
    }
    if (node->node_mode & SPI_MODE_CPHA) {
        spireg->CR1 |= SPI_CR1_CPHA;
    }

    /* Query peripheral clock. */
    if(node->node_bus->bus_base == SPI1_BASE){
        //SPI1
        clk = NutClockGet(NUT_HWCLK_PCLK2);
    }else{
        //SPI2,SPI3
        clk = NutClockGet(NUT_HWCLK_PCLK1);
    };
    /* Calculate the SPI clock divider. Avoid rounding errors. */
    clkdiv = (clk + node->node_rate - 1) / node->node_rate;
    /* The divider value minimum is 1. */
    if (clkdiv < 2) {
        clkdiv=2;
    }
    /* The divider value maximum is 255. */
    else if (clkdiv > 256) {
        clkdiv = 256;
    }

    for(i=8;i>=1;i--){
        if(clkdiv& (1<<i)){
            clkdiv=i-1;
            break;
        };
    };
    spireg->CR1 |= clkdiv << 3;//FIXME: write real code

    /* Update interface parameters. */
    node->node_rate = clk / clkdiv;
    node->node_mode &= ~SPI_MODE_UPDATE;

    return 0;
}


/*!
 * \brief Initialize an SPI bus node.
 *
 * This routine is called for each SPI node, which is registered via
 * NutRegisterSpiDevice().
 *
 * \param node Specifies the SPI bus node.
 *
 * \return 0 on success or -1 if there is no valid chip select.
 */
static int Stm32SpiBusNodeInit(NUTSPINODE * node)
{
    int rc = -1;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    /* Try to deactivate the node's chip select. */
    rc = Stm32SpiChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) == 0);
    /* It should not hurt us being called more than once. Thus, we
       ** check wether any initialization had been taken place already. */
    if (rc == 0 && node->node_stat == NULL)
    {
        /* Allocate and set our shadow registers. */
        SPI_TypeDef *spireg = malloc(sizeof(SPI_TypeDef));

        if (spireg) {
            /* Set interface defaults. */
            spireg->CR1 = SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_MSTR;
            /* Set Prescaler divide by 4 */
            spireg->CR1 |= SPI_CR1_BR_0;

#if !defined(STM32L1XX_MD)
//          spireg->CR2 = SPI_I2S_DMAReq_Tx|SPI_I2S_DMAReq_Rx;//FIXME:
//          commented out for debugging
            spireg->I2SCFGR=0;
            spireg->I2SPR=2; //FIXME: end of
#endif
            /* Update with node's defaults. */
            node->node_stat = (void *)spireg;
            Stm32SpiSetup(node);
        }
        else {
            /* Out of memory? */
            rc = -1;
        }
    }
    return rc;
}

/*!
 * \brief Transfer data on the SPI bus using single buffered interrupt mode.
 *
 * A device must have been selected by calling At91SpiSelect().
 *
 * \param node  Specifies the SPI bus node.
 * \param txbuf Pointer to the transmit buffer. If NULL, undetermined
 *              byte values are transmitted.
 * \param rxbuf Pointer to the receive buffer. If NULL, then incoming
 *              data is discarded.
 * \param xlen  Number of bytes to transfer.
 *
 * \return Always 0.
 */
static int Stm32SpiBusTransfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen)
{
    SPI_TypeDef* base;
  //  DMA_Channel_TypeDef* channel_rx,*channel_tx;


    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_bus->bus_base != 0);
    base = (SPI_TypeDef*)node->node_bus->bus_base;

   /* if(base == SPI1_BASE){
        channel_rx=DMA1_Channel2;
        channel_tx=DMA1_Channel3;
    }else if(base == SPI2_BASE){
        channel_rx=DMA1_Channel4;
        channel_tx=DMA1_Channel5;
    }else if(base == SPI3_BASE){
        channel_rx=DMA2_Channel1;
        channel_tx=DMA2_Channel2;
    };*/
//    DMA_Setup(channel_rx,(void*)rxbuf,(void*)(&(base->DR)),xlen,DMA_DIR_PeripheralSRC|DMA_MemoryDataSize_Byte|DMA_PeripheralDataSize_Byte|DMA_Priority_VeryHigh|DMA_IT_TC);//rx
//    DMA_Setup(channel_tx,(void*)txbuf,(void*)(&(base->DR)),xlen,DMA_DIR_PeripheralDST|DMA_MemoryDataSize_Byte|DMA_PeripheralDataSize_Byte|DMA_Priority_VeryHigh|DMA_IT_TC);//tx
//    DMA_Enable(channel_rx);DMA_Enable(channel_tx);

//    DMA_Register_Interrupt(channel_rx,&SPI_QUE);

    Stm32SpiEnable(base);

    unsigned char * tx = (unsigned char*)txbuf;
    unsigned char * rx = (unsigned char*)rxbuf;

    while( xlen-- > 0){
        unsigned char b = tx ? (* tx ++) : 0xff;

        base->DR = b;
        /* wait until receive buffer no longer empty */
        while ( ( base->SR & SPI_SR_RXNE ) == 0 ) {
    }

    b = base->DR;

    if( rx ) {
      * rx ++ = b;
    }
  }

//    NutEventWait(&SPI_QUE,0);
    Stm32SpiDisable(base);
    return 0;

}

