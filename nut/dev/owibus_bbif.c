/*
 * Copyright (C) 2012 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
 *
 * All rights reserved.
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
 */

/*!
 * \file dev/owibus_timerif.c
 * \brief Implementation of One-Wire primitives with Bitbanging.
 *
 * \verbatim
 * $Id: owibus_bbif.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <string.h>
#include <stdint.h>
#include <sys/timer.h>
#include <dev/gpio.h>
#include <stdlib.h>

#include <dev/owibus.h>
#include <dev/owibus_bbif.h>

/*!
 * \addtogroup xgOwibusBb
 */
/*@{*/

/*!
 * \brief Perform One-Wire transaction.
 *
 * \param bus     Specifies the One-Wire bus.
 * \param command Either OWI_CMD_RESET or OWI_CMD_RWBIT.
 * \param value   The value to send.
 *
 * \return The value read on success, a negative value otherwise.
 */
static int BB_OwiTransaction(NUTOWIBUS *bus, int_fast8_t command, int_fast8_t value)
{
    int res;
    NUTOWIINFO_BB *owcb = (NUTOWIINFO_BB *) (bus->owibus_info);
    int16_t delay1 =
        (owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SYNC_PULSE] -
         owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SETUP]) >> 2;
    int16_t delay2 =
        (owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_RW] -
         owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_SYNC_PULSE]) >> 2;
    int16_t delay3 =
        (owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_RELEASE] -
         owi_timervalues_250ns[bus->mode & OWI_OVERDRIVE][command][OWI_PHASE_RW]) >> 2;

    /* Be nice! Allow other thing to happen now before we block
     * cooperative multitasking for up to 480 us
     */
    NutSleep(0);
    GpioPinSetLow(owcb->txrx_port, owcb->txrx_pin);
    NutMicroDelay(delay1);
    if (value == 0)
        GpioPinSetLow(owcb->txrx_port, owcb->txrx_pin);
    else
        GpioPinSetHigh(owcb->txrx_port, owcb->txrx_pin);
    NutMicroDelay(delay2);
    res = GpioPinGet(owcb->txrx_port, owcb->txrx_pin);
    if (value)
        /* If the TXRX line is allready pull up, we only need to wait,
         * but no time sensitive action must be performed. We block for
         * up to 410 us now. So be nice again!
         */
        NutSleep(0);
    NutMicroDelay(delay3);
    GpioPinSetHigh(owcb->txrx_port, owcb->txrx_pin);
    return res;
}

/*!
 * \brief Reset the One-Wire bus and check if device(s) present.
 *
 * \param bus Specifies the One-Wire bus.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int BB_OwiTouchReset(NUTOWIBUS *bus)
{
    return BB_OwiTransaction(bus, OWI_CMD_RESET, 1);
}

/*!
 * \brief Exchange one bit on the One-Wire bus.
 *
 * \param bus Specifies the One-Wire bus.
 * \param bit Value for the bit to send.
 *
 * \return The bus state at the read slot on success, a negative value
 *         otherwise.
 */
static int OwiRWBit(NUTOWIBUS *bus, uint_fast8_t bit)
{
    return BB_OwiTransaction(bus, OWI_CMD_RWBIT, bit);
}

/*!
 * \brief Write a block of data bits to the One-Wire bus.
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data bits to send.
 * \param len  Number of bits to send.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int BB_OwiWriteBlock(NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len)
{
    int res;
    int i;

    for (i = 0; i < len; i++) {
        res = OwiRWBit(bus, data[i >> 3] & (1 << (i & 0x7)));
        if (res < 0)
            return OWI_HW_ERROR;
    }
    return OWI_SUCCESS;
}

/*!
 * \brief Read a block of data bits from the One-Wire bus.
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data bits received.
 * \param len  Number of bits to read.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int BB_OwiReadBlock(NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len)
{
    int res;
    int i;

    memset(data, 0, (len >> 3) + 1);
    for (i = 0; i < len; i++) {
        res = OwiRWBit(bus, 1);
        if (res < 0)
            return OWI_HW_ERROR;
        data[i >> 3] |= (res << (i & 0x7));
    }
    return OWI_SUCCESS;
}

/*!
 * \brief Register the One-Wire bus.
 *
 * \param bus         The returned NUTOWIBUS.
 * \param txrx_port   The port to use for the One_Wire bus.
 * \param txrx_pin    The pin to use for the One_Wire bus.
 * \param pullup_port If given, port to control strong pull-up for
 *                    parasitic powered devices.
 * \param pullup_pin  The pin to control strong pull-up for parasitic
 *                    powered devices.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int NutRegisterOwiBus_BB(NUTOWIBUS *bus, int txrx_port, uint_fast8_t txrx_pin, int pullup_port, uint_fast8_t pullup_pin)
{
    int res;
    NUTOWIINFO_BB *owcb;

    owcb = calloc(1, sizeof(*owcb));
    if (owcb == NULL) {
        return OWI_OUT_OF_MEM;
    }

    if (GpioPinConfigSet(txrx_port, txrx_pin, GPIO_CFG_PULLUP | GPIO_CFG_OUTPUT | GPIO_CFG_MULTIDRIVE)) {
        res = OWI_INVALID_HW;
        goto free_all;
    }
    if (pullup_pin) {
        if (GpioPinConfigSet(pullup_pin, pullup_pin, GPIO_CFG_PULLUP | GPIO_CFG_OUTPUT)) {
            res = OWI_INVALID_HW;
            goto free_all;
        }
        GpioPinSetHigh(pullup_pin, pullup_pin);
        GpioPinConfigSet(pullup_pin, pullup_pin, GPIO_CFG_PULLUP);
    }

    owcb->txrx_port = txrx_port;
    owcb->txrx_pin = txrx_pin;
    owcb->pp_port = pullup_pin;
    owcb->pp_pin = pullup_pin;
    bus->owibus_info = (uintptr_t) owcb;
    bus->OwiTouchReset = BB_OwiTouchReset;
    bus->OwiReadBlock = BB_OwiReadBlock;
    bus->OwiWriteBlock = BB_OwiWriteBlock;
    bus->mode = 0;
    return OWI_SUCCESS;

  free_all:
    free(owcb);
    return res;
}

/*@}*/
