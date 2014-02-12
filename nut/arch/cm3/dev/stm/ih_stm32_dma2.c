/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 * THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>
#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_dma.h>

#ifndef NUT_IRQPRI_DMA2
#define NUT_IRQPRI_DMA2  4
#endif

#ifdef NUT_PERFMON
static int Dma2IrqCtl(IRQn_Type IRQn, void(*irqfn)(void*), IRQ_HANDLER* IRQh, int cmd, void *param);
#else
static int Dma2IrqCtl(IRQn_Type IRQn, void(*irqfn)(void*), int cmd, void *param);
#endif

static int Dma2c1IrqCtl(int cmd, void *param);
static int Dma2c2IrqCtl(int cmd, void *param);
static int Dma2c3IrqCtl(int cmd, void *param);
static int Dma2c4IrqCtl(int cmd, void *param);
#if defined(STM32F10X_CL)  || defined(MCU_STM32F2) || defined(MCU_STM32F4)
static int Dma2c5IrqCtl(int cmd, void *param);
#endif
#if defined(MCU_STM32F2) || defined(MCU_STM32F4)
static int Dma2c6IrqCtl(int cmd, void *param);
static int Dma2c7IrqCtl(int cmd, void *param);
static int Dma2c8IrqCtl(int cmd, void *param);
#endif

/*!
 * \brief DMA 2 Channel Interrupt Signals
 */
IRQ_HANDLER sig_DMA2_CH1 = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    Dma2c1IrqCtl    /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_DMA2_CH2 = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    Dma2c2IrqCtl    /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_DMA2_CH3 = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    Dma2c3IrqCtl    /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_DMA2_CH4 = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    Dma2c4IrqCtl    /* Interrupt control, ir_ctl. */
};

#if defined(STM32F10X_CL) || defined(MCU_STM32F2) || defined(MCU_STM32F4)
IRQ_HANDLER sig_DMA2_CH5 = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    Dma2c5IrqCtl    /* Interrupt control, ir_ctl. */
};
#endif

#if defined(MCU_STM32F2) || defined(MCU_STM32F4)
IRQ_HANDLER sig_DMA2_CH6 = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    Dma2c6IrqCtl    /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_DMA2_CH7 = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    Dma2c7IrqCtl    /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_DMA2_CH8 = {
#ifdef NUT_PERFMON
    0,              /* Interrupt counter, ir_count. */
#endif
    NULL,           /* Passed argument, ir_arg. */
    NULL,           /* Handler subroutine, ir_handler. */
    Dma2c8IrqCtl    /* Interrupt control, ir_ctl. */
};

#endif
/*!
 * \brief DMA 2 Channel interrupt entries.
 */
void Dma2IrqEntry_Ch1(void *arg)
{
#ifdef NUT_PERFMON
    sig_DMA2_CH1.ir_count++;
#endif
    if (sig_DMA2_CH1.ir_handler) {
        (sig_DMA2_CH1.ir_handler) (sig_DMA2_CH1.ir_arg);
    }
}

void Dma2IrqEntry_Ch2(void *arg)
{
#ifdef NUT_PERFMON
    sig_DMA2_CH2.ir_count++;
#endif
    if (sig_DMA2_CH2.ir_handler) {
        (sig_DMA2_CH2.ir_handler) (sig_DMA2_CH2.ir_arg);
    }
}

void Dma2IrqEntry_Ch3(void *arg)
{
#ifdef NUT_PERFMON
    sig_DMA2_CH3.ir_count++;
#endif
    if (sig_DMA2_CH3.ir_handler) {
        (sig_DMA2_CH3.ir_handler) (sig_DMA2_CH3.ir_arg);
    }
}

void Dma2IrqEntry_Ch4(void *arg)
{
#ifdef NUT_PERFMON
    sig_DMA2_CH4.ir_count++;
#endif
    if (sig_DMA2_CH4.ir_handler) {
        (sig_DMA2_CH4.ir_handler) (sig_DMA2_CH4.ir_arg);
    }
}

#if defined(STM32F10X_CL) || defined(MCU_STM32F2) || defined(MCU_STM32F4)
void Dma2IrqEntry_Ch5(void *arg)
{
#ifdef NUT_PERFMON
    sig_DMA2_CH5.ir_count++;
#endif
    if (sig_DMA2_CH5.ir_handler) {
        (sig_DMA2_CH5.ir_handler) (sig_DMA2_CH5.ir_arg);
    }
}
#endif

#if defined(MCU_STM32F2) || defined(MCU_STM32F4)
void Dma2IrqEntry_Ch6(void *arg)
{
#ifdef NUT_PERFMON
    sig_DMA2_CH6.ir_count++;
#endif
    if (sig_DMA2_CH6.ir_handler) {
        (sig_DMA2_CH6.ir_handler) (sig_DMA2_CH6.ir_arg);
    }
}

void Dma2IrqEntry_Ch7(void *arg)
{
#ifdef NUT_PERFMON
    sig_DMA2_CH7.ir_count++;
#endif
    if (sig_DMA2_CH7.ir_handler) {
        (sig_DMA2_CH7.ir_handler) (sig_DMA2_CH7.ir_arg);
    }
}

void Dma2IrqEntry_Ch8(void *arg)
{
#ifdef NUT_PERFMON
    sig_DMA2_CH8.ir_count++;
#endif
    if (sig_DMA2_CH8.ir_handler) {
        (sig_DMA2_CH8.ir_handler) (sig_DMA2_CH8.ir_arg);
    }
}
#endif

/*!
 * DMA 2 Channel Interrupt Controllers
 */
static int Dma2c1IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel1_IRQn, Dma2IrqEntry_Ch1, sig_DMA2_CH1, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel1_IRQn, Dma2IrqEntry_Ch1, cmd, param);
#endif
}
static int Dma2c2IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel2_IRQn, Dma2IrqEntry_Ch2, sig_DMA2_CH2, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel2_IRQn, Dma2IrqEntry_Ch2, cmd, param);
#endif
}
static int Dma2c3IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel3_IRQn, Dma2IrqEntry_Ch3, sig_DMA2_CH3, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel3_IRQn, Dma2IrqEntry_Ch3, cmd, param);
#endif
}

#if defined(STM32F10X_HD) || defined(STM32F10X_XL)
static int Dma2c4IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel4_5_IRQn, Dma2IrqEntry_Ch4, sig_DMA2_CH4, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel4_5_IRQn, Dma2IrqEntry_Ch4, cmd, param);
#endif
}

#elif defined(STM32F10X_CL) || defined(MCU_STM32F2) || defined(MCU_STM32F4)

static int Dma2c4IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel4_IRQn, Dma2IrqEntry_Ch4, sig_DMA2_CH4, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel4_IRQn, Dma2IrqEntry_Ch4, cmd, param);
#endif
}
static int Dma2c5IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel5_IRQn, Dma2IrqEntry_Ch5, sig_DMA2_CH5, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel5_IRQn, Dma2IrqEntry_Ch5, cmd, param);
#endif
}
#if defined(MCU_STM32F2) || defined(MCU_STM32F4)
static int Dma2c6IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel6_IRQn, Dma2IrqEntry_Ch6, sig_DMA2_CH6, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel6_IRQn, Dma2IrqEntry_Ch6, cmd, param);
#endif
}
static int Dma2c7IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel7_IRQn, Dma2IrqEntry_Ch7, sig_DMA2_CH7, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel7_IRQn, Dma2IrqEntry_Ch7, cmd, param);
#endif
}
static int Dma2c8IrqCtl(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return Dma2IrqCtl(DMA2_Channel8_IRQn, Dma2IrqEntry_Ch8, sig_DMA2_CH8, cmd, param);
#else
    return Dma2IrqCtl(DMA2_Channel8_IRQn, Dma2IrqEntry_Ch8, cmd, param);
#endif
}
#endif
#endif



/*!
 * \brief DMA 2 interrupt control.
 *
 * \param cmd   Control command.
 *              - NUT_IRQCTL_INIT Initialize and disable interrupt.
 *              - NUT_IRQCTL_STATUS Query interrupt status.
 *              - NUT_IRQCTL_ENABLE Enable interrupt.
 *              - NUT_IRQCTL_DISABLE Disable interrupt.
 *              - NUT_IRQCTL_GETMODE Query interrupt mode.
 *              - NUT_IRQCTL_SETMODE Set interrupt mode (NUT_IRQMODE_LEVEL or NUT_IRQMODE_EDGE).
 *              - NUT_IRQCTL_GETPRIO Query interrupt priority.
 *              - NUT_IRQCTL_SETPRIO Set interrupt priority.
 *              - NUT_IRQCTL_GETCOUNT Query and clear interrupt counter.
 * \param param Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
#ifdef NUT_PERFMON
static int Dma2IrqCtl(IRQn_Type IRQn, void(*irqfn)(void*), IRQ_HANDLER* IRQh, int cmd, void *param)
#else
static int Dma2IrqCtl(IRQn_Type IRQn, void(*irqfn)(void*), int cmd, void *param)
#endif
{
    int rc = 0;
    unsigned int *ival = (unsigned int *)param;
    int_fast8_t enabled = NVIC_GetEnableIRQ(IRQn);

    /* Disable interrupt. */
    if (enabled) {
        NVIC_DisableIRQ(IRQn);
    }

    switch(cmd) {
    case NUT_IRQCTL_INIT:
        /* Set the vector. */
        Cortex_RegisterInt(IRQn, irqfn);
        /* Initialize with defined priority. */
        NVIC_SetPriority(IRQn, NUT_IRQPRI_DMA2);
        /* Clear interrupt */
        NVIC_ClearPendingIRQ(IRQn);
        break;
    case NUT_IRQCTL_STATUS:
        if (enabled) {
            *ival |= 1;
        }
        else {
            *ival &= ~1;
        }
        break;
    case NUT_IRQCTL_ENABLE:
        enabled = 1;
        break;
    case NUT_IRQCTL_DISABLE:
        enabled = 0;
        break;
    case NUT_IRQCTL_GETMODE:
            *ival = NUT_IRQMODE_EDGE;
        break;
    case NUT_IRQCTL_SETMODE:
            rc = -1;
        break;
    case NUT_IRQCTL_GETPRIO:
        *ival = NVIC_GetPriority(IRQn);
        break;
    case NUT_IRQCTL_SETPRIO:
        NVIC_SetPriority(IRQn,*ival);
        break;
#ifdef NUT_PERFMON
    case NUT_IRQCTL_GETCOUNT:
        *ival = (unsigned int)IRQh.ir_count;
        IRQh.ir_count = 0;
        break;
#endif
    default:
        rc = -1;
        break;
    }

    /* Enable interrupt. */
    if (enabled) {
        NVIC_EnableIRQ(IRQn);
    }
    return rc;
}

