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
 * $Id: stm32f1_dma.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>

#if defined(MCU_STM32F1)
#include <arch/cm3/stm/stm32f10x.h>
#include <arch/cm3/stm/stm32f10x_rcc.h>
#elif defined(MCU_STM32L1)
#include <arch/cm3/stm/stm32l1xx.h>
#include <arch/cm3/stm/stm32l1xx_rcc.h>
#else
#warning "STM32 family has no F1/L1 compatible DMA"
#endif
#include <arch/cm3/stm/stm32_dma.h>

/*!
 * \brief Table to align channels and interrupts for simpler access
 */
const DMATAB DmaTab[] = {
    { DMA1_BASE,  0, DMA1_Channel1 },
    { DMA1_BASE,  4, DMA1_Channel2 },
    { DMA1_BASE,  8, DMA1_Channel3 },
    { DMA1_BASE, 12, DMA1_Channel4 },
    { DMA1_BASE, 16, DMA1_Channel5 },
    { DMA1_BASE, 20, DMA1_Channel6 },
    { DMA1_BASE, 24, DMA1_Channel7 },
#if defined(STM32F10X_HD) || defined(STM32F10X_XL)
    { DMA1_BASE,  0, DMA2_Channel1 },
    { DMA1_BASE,  4, DMA2_Channel2 },
    { DMA1_BASE,  8, DMA2_Channel3 },
      /* These chips have one IRQ for ch. 4 & 5 */
    { DMA1_BASE, 12, DMA2_Channel4 },
    { DMA1_BASE, 16, DMA2_Channel4 },
#endif
#if defined(STM32F10X_CL)
    { DMA1_BASE,  0, DMA2_Channel1 },
    { DMA1_BASE,  4, DMA2_Channel2 },
    { DMA1_BASE,  8, DMA2_Channel3 },
    { DMA1_BASE, 12, DMA2_Channel4 },
    { DMA1_BASE, 16, DMA2_Channel5 },
#endif
};

const uint8_t DMACount = sizeof(DmaTab)/sizeof(DMATAB);
/*
 * Tricky but simple, small and fast
 */
const DMATAB *DmaTab1 = &DmaTab[0];
#ifdef STM_HAS_DMA2
const DMATAB *DmaTab2 = &DmaTab[STM_HAS_DMA1];
#endif

/*
 * \brief DMA Transfer Setup Function.
 *
 * This function sets up a DMA transfer. This function automatically
 * detects the transfer direction and if sources or targets are memory
 * or peripherals and sets up the right flags.
 *
 * \param channel One of the available channels of the STM32. If used
 *        with peripherals, check to use the correct channel for the
 *        desired peripheral unit.
 * \param dst Destination where to send the data to. Can be memory or peripheral.
 * \param src Source to take the data from. Can be memory or peripheral.
 * \param flags Option flags for priority and autoincrement of memory or peripheral.
 *
 */
void DMA_Setup( uint8_t ch, void* dst, void* src, uint16_t length, uint32_t flags)
{
    DMA_Channel_TypeDef* channel = DmaTab[ch].dma_ch;
    uint32_t cc = flags & ~(DMA_CCR1_MEM2MEM|DMA_CCR2_DIR|DMA_CCR1_EN);
    uint32_t cp;
    uint32_t cm;

    /* Stop DMA channel */
    channel->CCR = cc;

    /* Detect transfer type and set Registers accordingly */
    if ((uint32_t)src & PERIPH_BASE) {
        /* Peripheral to Memory */
        cp=(uint32_t)src;
        cm=(uint32_t)dst;
    }
    else if ((uint32_t)dst & PERIPH_BASE) {
        /* Memory to Peripheral */
        cc |= DMA_CCR2_DIR;
        cp=(uint32_t)dst;
        cm=(uint32_t)src;
    }
    else {
        /* Memory to Memory Transfer */
        cc |= DMA_CCR1_MEM2MEM | DMA_CCR1_DIR;
        cp =(uint32_t)dst;
        cm =(uint32_t)dst;
    }

    channel->CCR=cc;
    channel->CNDTR=length;
    channel->CPAR=cp;
    channel->CMAR=cm;

};

/*
 * \brief Enable DMA Transfer.
 */
void DMA_Enable(uint8_t ch)
{
    DMA_Channel_TypeDef *channel = (DMA_Channel_TypeDef*)DmaTab[ch].dma_ch;
    channel->CCR |= DMA_CCR1_EN;
}

/*
 * \brief Disable DMA Transfer.
 */
void DMA_Disable(uint8_t ch)
{
    DMA_Channel_TypeDef *channel = (DMA_Channel_TypeDef*)DmaTab[ch].dma_ch;
    channel->CCR &= ~DMA_CCR1_EN;
}


/*!
 * \brief DMA System Initialization
 *
 * Register all DMA interrupt handlers.
 */
void DMA_Init(void)
{
    uint8_t i;
    DMA_Channel_TypeDef *channel;

    /* Enable DMA clocks */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
/* FIXME: The ST defined headers don't define RCC_AHBENR_DMA2EN for XL
 * devices,  but RM0008 says that XL devices have DMA2
 * Assume no DMA2 for XL devices for now
 */
#if defined(STM_HAS_DMA2) && defined(RCC_AHBENR_DMA2EN)
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
#endif

    /* Clear pending interrupts in DMA 1 ISR */
    DMA1->IFCR = 0xFFFFFFFF;
    /* Clear interrupt related flags in channels */
    for(i=0; i<STM_HAS_DMA1; i++) {
        channel = (DMA_Channel_TypeDef*)DmaTab1[i].dma_ch;
        channel->CCR = 0;
        DMA_ClearFlag(i,0xf);
     }

#ifdef STM_HAS_DMA2
    /* Clear pending interrupts in DMA 2 ISR */
    DMA2->IFCR = 0xFFFFFFFF;
    /* Clear interrupt related flags in channels */
    for(i=0;i<STM_HAS_DMA2;i++) {
        channel = (DMA_Channel_TypeDef*)DmaTab2[i].dma_ch;
        channel->CCR = 0;
        DMA_ClearFlag(i,0xf);
    }
#endif
}

/*!
 * \brief Control DMA channel interrupt masks
 *
 * Setup interrupt mask on given channel. Channel numbers are
 * from 0..n while documentation numbers them from 1..n.
 * For that please use the defines DMAx_Cn.
 *
 * \param ch   Channel number to set interrupt mask.
 * \param mask Mask to set on the designated channel interrupts.
 * \param ena  Enable (1) or Disable (0) the bits in the mask.
 */
void DMA_IrqMask( uint8_t ch, uint32_t mask, uint8_t ena)
{
    DMA_Channel_TypeDef *channel = (DMA_Channel_TypeDef*)DmaTab[ch].dma_ch;
    uint32_t ccr = channel->CCR;
    DMA_TypeDef *dma = (DMA_TypeDef*)DmaTab[ch].dma;
    uint32_t msk = mask & DMA_FLAGMASK;

    if( ena) {
        /* Enable some/all of the DMA channels interrupts */
        ccr |= msk;
    }
    else {
        /* Disable some/all of the DMA channels interrupts */
        ccr &= ~msk;
    }

    /* Set channel configuration */
    channel->CCR = ccr;

    /* Clear already pending flags */
    dma->IFCR = (mask << (DmaTab[ch].fofs));
};

/*!
 * \brief Clear DMA channel flags.
 *
 * Setup interrupt mask on given channel. Channel numbers are
 * from 0..n while documentation numbers them from 1..n.
 * For that please use the defines DMAx_Cn.
 *
 * \param ch    Channel number.
 * \param flags Mask of flags to clear.
 */
void DMA_ClearFlag( uint8_t ch, uint32_t flags)
{
    volatile DMA_TypeDef *dma = (DMA_TypeDef*)DmaTab[ch].dma;
    uint32_t msk = (flags&DMA_FLAGMASK)<<DmaTab[ch].fofs;

    dma->IFCR = msk;
}

/*!
 * \brief Get DMA channel flags.
 *
 * Get interrupt / status flags of given channel.
 *
 * \param ch    Channel number to set interrupt mask.
 *
 * \return flags Mask of flags to clear.
 */
uint32_t DMA_GetFlag( uint8_t ch)
{
    volatile DMA_TypeDef *dma = (DMA_TypeDef*)DmaTab[ch].dma;
    uint32_t flags = (dma->ISR>>DmaTab[ch].fofs)&DMA_FLAGMASK;
    return flags;
}

