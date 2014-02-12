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
 * \file dev/owibus_uartif.c
 * \brief Implementation of the One-Wire via Uart.
 *
 * \verbatim
 * $Id: owibus_uartif.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <stdint.h>
#include <io.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/timer.h>
#include <dev/uart.h>
#include <dev/gpio.h>
#include <dev/owibus.h>
#include <dev/owibus_uartif.h>
#include <stdlib.h>

/*!
 * \addtogroup xgOwibusUart
 */
/*@{*/

/*!
 * \brief Reset the One-Wire bus and check if device(s) present.
 *
 * \param bus Specifies the One-Wire bus.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
static int Uart_OwiTouchReset(NUTOWIBUS *bus)
{
    NUTOWIINFO_UART *owcb = (NUTOWIINFO_UART *) (bus->owibus_info);
    uint8_t send_data[1] = { OWI_UART_WRITE_RST };
    uint8_t rec_data[1] = { 0 };
    uint32_t baud_presence = OWI_UART_BAUD_RESET, baud_owi_rwbit = OWI_UART_BAUD_RWBIT;

    _ioctl(owcb->uart_fd, UART_SETSPEED, &baud_presence);
    _write(owcb->uart_fd, send_data, 1);
    NutSleep(2);
    if (_read(owcb->uart_fd, rec_data, 1) == -1) {
        return OWI_HW_ERROR;
    }
    _ioctl(owcb->uart_fd, UART_SETSPEED, &baud_owi_rwbit);
    if ((rec_data[0] != 0xf0) && !(rec_data[0] & 0x10) && (rec_data[0] != 0)) {
        return OWI_SUCCESS;
    }
    return OWI_PRESENCE_ERR;
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
static int Uart_OwiRWBit(NUTOWIBUS *bus, uint_fast8_t bit)
{
    NUTOWIINFO_UART *owcb = (NUTOWIINFO_UART *) (bus->owibus_info);
    uint8_t send_data[1] = { (bit) ? OWI_UART_WRITE_ONE : OWI_UART_WRITE_ZERO };
    uint8_t rec_data[1] = { 0 };

    _write(owcb->uart_fd, send_data, 1);
    if (_read(owcb->uart_fd, rec_data, 1) == -1) {
        return OWI_HW_ERROR;
    }
    if (rec_data[0] & OWI_UART_READ_ONE) {
        return 1;
    }
    return 0;
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
static int Uart_OwiWriteBlock(NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len)
{
    int res;
    int i;

    for (i = 0; i < len; i++) {
        res = Uart_OwiRWBit(bus, data[i >> 3] & (1 << (i & 0x7)));
        if (res < 0) {
            return OWI_HW_ERROR;
        }
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
static int Uart_OwiReadBlock(NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len)
{
    int res;
    int i;

    memset(data, 0, (len >> 3) + 1);
    for (i = 0; i < len; i++) {
        res = Uart_OwiRWBit(bus, 1);
        if (res < 0) {
            return OWI_HW_ERROR;
        }
        data[i >> 3] |= (res << (i & 0x7));
    }
    return OWI_SUCCESS;
}

/*!
 * \brief Register the One-Wire bus.
 *
 * \param bus         The returned NUTOWIBUS.
 * \param uart        The UART device to use.
 * \param pullup_port If given, port to control strong pull-up for
 *                    parasitic powered devices.
 * \param pullup_pin  The pin to control strong pull-up for parasitic
 *                    powered devices.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int NutRegisterOwiBus_Uart(NUTOWIBUS *bus, NUTDEVICE *uart, int pullup_port, uint_fast8_t pullup_pin)
{
    int uart_fd;
    uint32_t timeout = 2;
    uint32_t stopbits = 2;
    NUTOWIINFO_UART *owcb;

    if (NutRegisterDevice(uart, 0, 0)) {
        return OWI_INVALID_HW;
    }
    uart_fd = _open(uart->dev_name, _O_RDWR | _O_BINARY);
    if (uart_fd == -1) {
        return OWI_INVALID_HW;
    }
    _ioctl(uart_fd, UART_SETREADTIMEOUT, &timeout);
    _ioctl(uart_fd, UART_SETSTOPBITS, &stopbits);

    owcb = calloc(1, sizeof(*owcb));
    if (owcb == NULL) {
        return OWI_OUT_OF_MEM;
    }
    owcb->uart_fd = uart_fd;
    bus->owibus_info = (uintptr_t) owcb;
    bus->OwiTouchReset = Uart_OwiTouchReset;
    bus->OwiReadBlock = Uart_OwiReadBlock;
    bus->OwiWriteBlock = Uart_OwiWriteBlock;
    bus->mode = 0;

    return OWI_SUCCESS;
}

/*@}*/
