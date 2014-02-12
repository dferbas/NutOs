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
 * THIS SOFTWARE IS PROVIDED BY EMBEDDED IT AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EMBEDDED IT
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

/*!
 * \verbatim
 * $Id: ih_stm32_can1.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <dev/irqreg.h>

#ifndef NUT_IRQPRI_CAN1
#define NUT_IRQPRI_CAN1  4
#endif

#if defined(STM32F10X_LD)|| defined(STM32F10X_MD)|| defined(STM32F10X_HD)|| defined(STM32F10X_XL)
#define CAN_TX_IRQn  USB_HP_CAN1_TX_IRQn
#define CAN_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#define CAN_RX1_IRQn CAN1_RX1_IRQn
#define CAN_SCE_IRQn CAN1_SCE_IRQn
#else
#define CAN_TX_IRQn  CAN1_TX_IRQn
#define CAN_RX0_IRQn CAN1_RX0_IRQn
#define CAN_RX1_IRQn CAN1_RX1_IRQn
#define CAN_SCE_IRQn CAN1_SCE_IRQn

#endif

#ifdef NUT_PERFMON
static int CanIrqCtl(IRQn_Type IRQn, void(*ifunc)(void*), IRQ_HANDLER *ih, int cmd, void *param);
#else
static int CanIrqCtl(IRQn_Type IRQn, void(*ifunc)(void*), int cmd, void *param);
#endif
static int CanIrqCtl_Tx(int cmd, void *param);
static int CanIrqCtl_Rx0(int cmd, void *param);
static int CanIrqCtl_Rx1(int cmd, void *param);
static int CanIrqCtl_Sce(int cmd, void *param);

IRQ_HANDLER sig_CAN1_TX = {
#ifdef NUT_PERFMON
    0,                  /* Interrupt counter, ir_count. */
#endif
    NULL,               /* Passed argument, ir_arg. */
    NULL,               /* Handler subroutine, ir_handler. */
    CanIrqCtl_Tx        /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_CAN1_RX0 = {
#ifdef NUT_PERFMON
    0,                  /* Interrupt counter, ir_count. */
#endif
    NULL,               /* Passed argument, ir_arg. */
    NULL,               /* Handler subroutine, ir_handler. */
    CanIrqCtl_Rx0       /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_CAN1_RX1 = {
#ifdef NUT_PERFMON
    0,                  /* Interrupt counter, ir_count. */
#endif
    NULL,               /* Passed argument, ir_arg. */
    NULL,               /* Handler subroutine, ir_handler. */
    CanIrqCtl_Rx1       /* Interrupt control, ir_ctl. */
};

IRQ_HANDLER sig_CAN1_SCE = {
#ifdef NUT_PERFMON
    0,                  /* Interrupt counter, ir_count. */
#endif
    NULL,               /* Passed argument, ir_arg. */
    NULL,               /* Handler subroutine, ir_handler. */
    CanIrqCtl_Sce       /* Interrupt control, ir_ctl. */
};

/*!
 * \brief CAN Interface TX Interrupt Entry.
 */
static void CanIrqEntry_Tx(void *arg)
{
#ifdef NUT_PERFMON
    sig_CAN1_TX.ir_count++;
#endif
    if (sig_CAN1_TX.ir_handler) {
        (sig_CAN1_TX.ir_handler) (sig_CAN1_TX.ir_arg);
    }
}

/*!
 * \brief CAN Interface RX0 Interrupt Entry.
 */
static void CanIrqEntry_Rx0(void *arg)
{
#ifdef NUT_PERFMON
    sig_CAN1_RX0.ir_count++;
#endif
    if (sig_CAN1_RX0.ir_handler) {
        (sig_CAN1_RX0.ir_handler) (sig_CAN1_RX0.ir_arg);
    }
}

/*!
 * \brief CAN Interface RX1 Interrupt Entry.
 */
static void CanIrqEntry_Rx1(void *arg)
{
#ifdef NUT_PERFMON
    sig_CAN1_RX1.ir_count++;
#endif
    if (sig_CAN1_RX1.ir_handler) {
        (sig_CAN1_RX1.ir_handler) (sig_CAN1_RX1.ir_arg);
    }
}

/*!
 * \brief CAN Interface SCE Interrupt Entry.
 */
static void CanIrqEntry_Sce(void *arg)
{
#ifdef NUT_PERFMON
    sig_CAN1_SCE.ir_count++;
#endif
    if (sig_CAN1_SCE.ir_handler) {
        (sig_CAN1_SCE.ir_handler) (sig_CAN1_SCE.ir_arg);
    }
}

static int CanIrqCtl_Tx(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return CanIrqCtl(CAN_TX_IRQn, CanIrqEntry_Tx, &sig_CAN1_TX, cmd, param);
#else
    return CanIrqCtl(CAN_TX_IRQn, CanIrqEntry_Tx, cmd, param);
#endif
}

static int CanIrqCtl_Rx0(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return CanIrqCtl(CAN_RX0_IRQn, CanIrqEntry_Rx0, &sig_CAN1_RX0, cmd, param);
#else
    return CanIrqCtl(CAN_RX0_IRQn, CanIrqEntry_Rx0, cmd, param);
#endif
}

static int CanIrqCtl_Rx1(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return CanIrqCtl(CAN_RX1_IRQn, CanIrqEntry_Rx1, &sig_CAN1_RX1, cmd, param);
#else
    return CanIrqCtl(CAN_RX1_IRQn, CanIrqEntry_Rx1, cmd, param);
#endif
}

static int CanIrqCtl_Sce(int cmd, void *param)
{
#ifdef NUT_PERFMON
    return CanIrqCtl(CAN_SCE_IRQn, CanIrqEntry_Sce, &sig_CAN1_SCE, cmd, param);
#else
    return CanIrqCtl(CAN_SCE_IRQn, CanIrqEntry_Sce, cmd, param);
#endif
}

/*!
 * \brief Two Wire Interface interrupt control.
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
static int CanIrqCtl(IRQn_Type IRQn, void(*ifunc)(void*), IRQ_HANDLER *ih, int cmd, void *param)
#else
static int CanIrqCtl(IRQn_Type IRQn, void(*ifunc)(void*), int cmd, void *param)
#endif
{
    int rc = 0;
    unsigned int *ival = (unsigned int *)param;
    uint_fast8_t enabled = NVIC_GetEnableIRQ(IRQn);

    /* Disable interrupt. */
    if (enabled) {
        NVIC_DisableIRQ(IRQn);
    }

    switch(cmd) {
    case NUT_IRQCTL_INIT:
        /* Set the vector. */
        Cortex_RegisterInt(IRQn, ifunc);
        /* Initialize with defined priority. */
        NVIC_SetPriority(IRQn, NUT_IRQPRI_CAN1);
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
        *ival = NUT_IRQMODE_LEVEL;
        break;
    case NUT_IRQCTL_SETMODE:
        break;
    case NUT_IRQCTL_GETPRIO:
        *ival = NVIC_GetPriority(IRQn);
        break;
    case NUT_IRQCTL_SETPRIO:
        NVIC_SetPriority(IRQn, *ival);
        break;
#ifdef NUT_PERFMON
    case NUT_IRQCTL_GETCOUNT:
        *ival = (unsigned int)ih->ir_count;
        ih->ir_count = 0;
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

