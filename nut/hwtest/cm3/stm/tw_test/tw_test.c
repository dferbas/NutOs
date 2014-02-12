/*!
 * Copyright (C) 2001-2003 by egnite Software GmbH
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

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <dev/twif.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <dev/gpio.h>


static char *banner = "\nNut/OS TW Sample " __DATE__ " " __TIME__ "\n";
static char *pgm_ptr = "\nHello stranger and our/my friends!\n";

static char inbuf[128];

#if defined(LED2_PORT) && defined( LED2_PIN)
THREAD(LedBlink, arg)
{
    GpioPinConfigSet( LED2_PORT, LED2_PIN, GPIO_CFG_OUTPUT);
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
 * TWI sample: Scan the TWI Bus and look for connected devices
 *
 * Some functions do not work with ICCAVR.
 */
int main(void)
{
    int res;
    char *cp;
    uint32_t baud = 9600;
    uint8_t lbuf[1];
    lbuf[0]=0; // dummy
    uint8_t a;

    res = NutRegisterDevice(&DEV_UART, 0, 0);
    while(res != 0){};

    freopen(DEV_UART_NAME, "w", stdout);
    freopen(DEV_UART_NAME, "r", stdin);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    printf(banner);

#if defined(LED1_PORT) && defined( LED1_PIN)
    GpioPinConfigSet( LED1_PORT, LED1_PIN, GPIO_CFG_OUTPUT);
#endif

#if defined(LED2_PORT) && defined( LED2_PIN)
    if (NutThreadCreate("t2", LedBlink, 0, 512)== 0)
        printf("Can't create LED thread\n");
    else
        printf("LED thread started\n");
#endif
#if defined(F4_DISCOVERY)
/* Set all CS43L22 I2C relates pins to s safe state*/
    GpioPinConfigSet(NUTGPIO_PORTE, 3, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTE, 3);
    GpioPinConfigSet(NUTGPIO_PORTC, 7, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 7);
    GpioPinConfigSet(NUTGPIO_PORTC, 10, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 10);
    GpioPinConfigSet(NUTGPIO_PORTC, 12, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 12);
    GpioPinConfigSet(NUTGPIO_PORTC, 14, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 14);
    GpioPinConfigSet(AUDIO_RST_PORT, AUDIO_RST_PIN, GPIO_CFG_OUTPUT);
    GpioPinSetLow(AUDIO_RST_PORT, AUDIO_RST_PIN);
    NutSleep(11);
    GpioPinSetHigh(AUDIO_RST_PORT, AUDIO_RST_PIN);
    NutSleep(10);
#endif

    res = NutRegisterTwiBus( &DEF_TWIBUS, 0);
    if (res !=0)
    {
        printf("NutRegisterTwiBus failed\n");
    }
    else
    {
        printf("NutRegisterTwiBus success\n");
    }

    baud = 10000;
    res = NutTwiIOCtl(&DEF_TWIBUS, TWI_SETSPEED, &baud);
    if (res !=0)
    {
        printf("Can't set speed TWI\n");
    }
    else
    {
        NutTwiIOCtl(&DEF_TWIBUS, TWI_GETSPEED, &baud);
        printf("TWI speed is %ld\n", baud );
    }

    /* Don't a a general call (a == 0) */
    for(a=2; a < 0x80; a++)
    {
        res =TwMasterTransact(a, lbuf, 1, NULL, 0, 5);
        if ((res == 1) && a)
        {
            printf("Found I2C device at 0x%02x\n", a);
        }
    }

    /*
     * Nut/OS never expects a thread to return. So we enter an
     * endless loop here.
     */
    for (;;) {
#if defined(LED1_PORT) && defined( LED1_PIN)
        GpioPinSet(LED1_PORT, LED1_PIN,
                   !(GpioPinGet(LED1_PORT, LED1_PIN)));
#endif
        /*
         * A bit more advanced input routine is able to read a string
         * up to and including the first newline character or until a
         * specified maximum number of characters, whichever comes first.
         */
        puts("\nEnter your name: ");
        fflush(stdout);
        fgets(inbuf, sizeof(inbuf), stdin);

        /*
         * Chop off trailing linefeed.
         */
        cp = strchr(inbuf, '\n');
        if (cp)
            *cp = 0;

        /*
         * Streams support formatted output as well as printing strings
         * from program space.
         */
        if (inbuf[0])
            printf("\nHello %s!\n", inbuf);
        else {
            puts(pgm_ptr);
        }

    }
    return 0;
}
