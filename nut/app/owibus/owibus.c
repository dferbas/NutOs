
#include <string.h>
#include <stdio.h>
#include <io.h>

#include <cfg/arch.h>
#include <stdint.h>
#include <sys/timer.h>
#include <dev/board.h>
#include <dev/gpio.h>
#include <dev/owibus.h>

#define USE_UART
//#define USE_BB

static char *banner = "\nNut/OS OWI Bus "__DATE__ " " __TIME__"\n";
/*
 * UART sample.
 *
 */
int main(void)
{
#if defined (USE_BB) && (!defined(OWI_PORT) || !defined (OWI_PIN))
    puts("Please defined the Port/Pin to use for the One-Wire Bus for your board");
#elif defined(USE_UART) && !defined(OWI_UART)
    puts("Please defined the UART to use for the One-Wire Bus for your board");
#else

    uint32_t baud = 115200;
    FILE *uart;
    int res, i = 0;
    uint64_t hid  = 0;
    int xcelsius;
    int run =0;
    uint8_t raw[2];
    NUTOWIBUS bus_container, *bus=&bus_container;
    uint8_t diff;

    NutRegisterDevice(&DEV_CONSOLE, 0, 0);

    uart = fopen(DEV_CONSOLE_NAME, "r+");

    _ioctl(_fileno(uart), UART_SETSPEED, &baud);

    freopen(DEV_CONSOLE_NAME, "w", stdout);
    fprintf(stdout, banner);

#if defined(USE_BB)
    res= NutRegisterOwiBus_BB(bus, OWI_PORT, OWI_PIN, 0, 0);
    fprintf(stdout, "Using Bitbang\n");
#else
    res= NutRegisterOwiBus_Uart(bus, &devUsartStm32_1, 0, 0);
    fprintf(stdout, "Using UART");
    /* Switch to Open Drain */
 #if defined(MCU_STM32) && defined(OWI_PORT) && defined(OWI_PIN)
    /* Switch to Open Drain */
  #if defined(MCU_STM32F1)
    ((__IO uint32_t*)(CM3BB_BASE(OWI_PORT)))[CM3BB_OFFSET(GPIO_TypeDef, CRL, 4*(OWI_PIN*4))] = 1;
  #else
    CM3BB_BASE(OWI_PORT)[CM3BB_OFFSET(GPIO_TypeDef, OTYPER, OWI_PIN)] = 1;
  #endif
/*   Switch to Half Duplex */
    CM3BBREG((devUsartStm32_1.dev_base), USART_TypeDef, CR3, _BI32(USART_CR3_HDSEL))=1;
    fprintf(stdout, " with RX connected internal to TX and TX configured as Open Drain\n");
 #else
    fprintf(stdout, "Make sure TX drives the OWI device as Open Drain and RX is connected to OWI\n");
 #endif

#endif

    if (res)
    {
        fprintf(stdout, "NutRegisterOwiBus_Timer failed %d\n", res);
        while(1)
            NutSleep(100);
    }
    diff = OWI_SEARCH_FIRST;
    res = OwiRomSearch(bus, &diff, &hid);
    if(res)
    {
        printf("OwiRomSearch failed\n");
        while(1)
            NutSleep(10);
    }
    fprintf(stdout, "hid %08lx%08lx\n", (uint32_t)(hid>>32), (uint32_t)(hid &0xffffffff));
    while(1)
    {
        res = OwiCommand( bus, OWI_CONVERT_T, NULL);
        if (res)
            printf("OwiCommand convert_t error %d\n", res);
        NutSleep(750);
        while (!(res = OwiReadBlock(bus, &diff, 1)) && !diff && i < 100)
        {
            NutSleep(10);
            i++;
        }
        if (i)
        {
            printf(
                "Conversion took additional %d poll cycles\n",
                i);
            i = 0;
        }
        res = OwiCommand( bus, OWI_READ, NULL);
        if (res)
            printf("OwiCommand read error %d\n", res);
        res = OwiReadBlock(bus, raw, 16);
        if (res)
            printf("OwiReadBlock error %d\n", res);
        xcelsius = (raw[1]<<8 | raw[0]) * 625;
        fprintf(stdout,"Run %3d: Temp %d.%04d\r",
                run++, xcelsius/10000, xcelsius%10000);
    }
#endif
    return 0;
}
