/*!
 * Copyright (C) 2001-2003 by egnite Software GmbH
 *           (C) 2011-2012 Uwe Bonnes
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

/* Connect a DS18B20 Onewire Device to OWI_PORT/OWI_PIN with a Pullup of
 * about 5 k.
 * On startup the unique Hardware ID of the device is  printed and on
 * return (CR) from the UART the measuered temperature
 */

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <arch/arm/cm3.h>
#include <dev/gpio.h>
#include <dev/hwtimer_stm32.h>
#include <dev/owi.h>

static char inbuf[128];

uint64_t hid  = 0;
uint8_t res = 1;
volatile int xcelsius;

THREAD(OneWire, arg)
{
    int i = 0;
    int16_t raw;
    for(;;)
    {
        if (w1_command( CONVERT_T, NULL))
            printf("w1_command convert_t error\n");
        NutSleep(750);
        while (!(OWReadBit()) && i < 100)
        {
            NutSleep(10);
            i++;
        }
        if(i > 100)
        {
            raw = TEMP_GENERAL_ERROR;
        }
        else
        {
            if (i)
            {
                printf(
                          "Conversion took additional %d poll cycles\n",
                          i);
                i = 0;
            }
            if (w1_command(  READ, NULL))
                printf("w1_command read error\n");
             raw = OWReadByte();
            raw |= OWReadByte() <<8;
        }
        xcelsius = raw * 625;
    }
}

#if defined(LED2_PORT) && defined(LED2_PIN)
THREAD(LedBlink, arg)
{
    for(;;)
    {
        NutSleep(100);
        GpioPinSetHigh(LED2_PORT, LED2_PIN);
        NutSleep(100);
        GpioPinSetLow(LED2_PORT, LED2_PIN);
    }
}
#endif

/*
 * Test for a connected OWI DSA18B20 using GPIO and a timer
 *
 */
int main(void)
{
    uint32_t baud = 9600;

    /*
     * Each device must be registered. We do this by referencing the
     * device structure of the driver. The advantage is, that only
     * those device drivers are included in our flash code, which we
     * really need.
     *
     * The uart0 device is the first one on the ATmega chip. So it
     * has no configurable base address or interrupt and we set both
     * parameters to zero.
     */
    NutRegisterDevice(&DEV_UART, 0, 0);

    /*
     * Now, as the device is registered, we can open it. The fopen()
     * function returns a pointer to a FILE structure, which we use
     * for subsequent reading and writing.
     */
    freopen(DEV_UART_NAME, "w", stdout);
    freopen(DEV_UART_NAME, "r", stdin);

    /*
     * Before doing the first read or write, we set the baudrate.
     * This low level function doesn't know about FILE structures
     * and we use _fileno() to get the low level file descriptor
     * of the stream.
     *
     * The short sleep allows the UART to settle after the baudrate
     * change.
     */
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    /*
     * Stream devices can use low level read and write functions.
     * Writing program space data is supported too.
     */
    printf("\nNut/OS OWI Sample2 " __DATE__ " " __TIME__
            "\nPress any key...");

#if defined(LED1_PORT) && defined(LED1_PIN)
    GpioPinConfigSet( LED1_PORT, LED1_PIN, GPIO_CFG_OUTPUT);
#endif
    OWInit();
    res = OWRomSearch(SEARCH_FIRST, &hid);
    if(res)
        printf("OWRomSearch failed\n");
    else
    {
        printf("Hid is 0x%08lx%08lx\n", (uint32_t)(hid>>32), (uint32_t)(hid & 0xffffffffL));
        if (NutThreadCreate("t1", OneWire, 0, 512)== 0)
            printf("Can't create owi thread\n");
        else
            printf("Owi thread started\n");
    }
#if defined(LED2_PORT) && defined(LED2_PIN)
    GpioPinConfigSet( LED2_PORT, LED2_PIN, GPIO_CFG_OUTPUT);
    if (NutThreadCreate("t2", LedBlink, 0, 512)== 0)
        printf("Can't create LED thread\n");
    else
        printf("LED thread started\n");
#endif
    /*
     * Nut/OS never expects a thread to return. So we enter an
     * endless loop here.
     */
    for (;;) {
#if defined(LED1_PORT) && defined(LED1_PIN)
        GpioPinSet(LED1_PORT, LED1_PIN,
                   !(GpioPinGet(LED1_PORT, LED1_PIN)));
#endif
        /*
         * A bit more advanced input routine is able to read a string
         * up to and including the first newline character or until a
         * specified maximum number of characters, whichever comes first.
         */
        printf("\nType enter to get temperature value: ");
        fgets(inbuf, sizeof(inbuf), stdinq);

        printf("Temp %d.%04d\n",
                xcelsius/10000, xcelsius%10000);

    }
    return 0;
}
