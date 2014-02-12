#include <string.h>
#include <stdio.h>

#include <sys/event.h>
#include <sys/timer.h>
#include <dev/gpio.h>
#include <dev/board.h>
#include <dev/hwtimer_stm32.h>
#include "owi.h"

HANDLE owi_irq_done;

volatile int owi_sample;
volatile int bit_value;
int OWInit(void)
{
    uint32_t val = TIM_ClockVal(OWI_TIMER);
    GpioPinConfigSet( OWI_PORT, OWI_PIN, GPIO_CFG_OUTPUT| GPIO_CFG_MULTIDRIVE);
    TIM_Init(OWI_TIMER);
    TIM_Clear(OWI_TIMER);
    TIM_IntEnable(OWI_TIMER);
    /* let us work in steps of 0.5 us */
    TIM_Prescaler(OWI_TIMER) = ((TIM_ClockVal(OWI_TIMER))/2000000L)-1;
    printf(" Clock %08lx, PSC soll %08lx ist %04x\n", val,
            (val/2000000)-1, TIM_Prescaler(OWI_TIMER));
    TIM_OnePulse( OWI_TIMER);
    return 1;
}

static void irq_ow_reset(void *arg)
{
    if TIM_C1InterruptFlag(OWI_TIMER)
    {
        GpioPinSetLow(OWI_PORT, OWI_PIN);
        TIM_C1ClearInterruptFlag(OWI_TIMER);
    }
    else if TIM_C2InterruptFlag(OWI_TIMER)
    {
        GpioPinSetHigh(OWI_PORT, OWI_PIN);
        TIM_C2ClearInterruptFlag(OWI_TIMER);
     }
    else if TIM_C3InterruptFlag(OWI_TIMER)
    {
        owi_sample = GpioPinGet(OWI_PORT, OWI_PIN);
        TIM_C3ClearInterruptFlag(OWI_TIMER);
   }

    else if TIM_C4InterruptFlag(OWI_TIMER)
    {
        TIM_C4ClearInterruptFlag(OWI_TIMER);
        NutEventPostFromIrq(&owi_irq_done);
    }
    else /* Just in case*/
    {
        TIM_ClearInterruptFlag( OWI_TIMER );
    }
}

int OWTouchReset(void)
{
    TIM_Compare1(OWI_TIMER)  = 2*(3);
    TIM_Compare2(OWI_TIMER)  = 2*(3 + 480);
    TIM_Compare3(OWI_TIMER)  = 2*(3 + 480 +70);
    TIM_Compare4(OWI_TIMER)  = 2*(3 + 480 +70 +410);
    TIM_C1IRQEnable( OWI_TIMER);
    TIM_C2IRQEnable( OWI_TIMER);
    TIM_C3IRQEnable( OWI_TIMER);
    TIM_C4IRQEnable( OWI_TIMER);
    TIM_IntRegister(OWI_TIMER, irq_ow_reset);
    TIM_Counter(OWI_TIMER) =  0;
    /* Reload the values (Counter and Prescaler*/
    TIM_Update( OWI_TIMER);
    TIM_StartTimer( OWI_TIMER );
    NutEventWait(&owi_irq_done, 10);
    TIM_StopTimer( OWI_TIMER);
    GpioPinSetHigh(OWI_PORT, OWI_PIN);
    return owi_sample;
}

static void irq_ow_bit(void *arg)
{
    if TIM_C1InterruptFlag(OWI_TIMER)
    {
        GpioPinSetLow(OWI_PORT, OWI_PIN);
        TIM_C1ClearInterruptFlag(OWI_TIMER);
    }
    else if TIM_C2InterruptFlag(OWI_TIMER)
    {
        if(bit_value == 0)
            GpioPinSetLow(OWI_PORT, OWI_PIN);
        else
            GpioPinSetHigh(OWI_PORT, OWI_PIN);
    TIM_C2ClearInterruptFlag(OWI_TIMER);
    }
    else if TIM_C3InterruptFlag(OWI_TIMER)
    {
        owi_sample = GpioPinGet(OWI_PORT, OWI_PIN);
    TIM_C3ClearInterruptFlag(OWI_TIMER);
    }

    else if TIM_C4InterruptFlag(OWI_TIMER)
    {
        GpioPinSetHigh(OWI_PORT, OWI_PIN);
    TIM_C4ClearInterruptFlag(OWI_TIMER);
        NutEventPostFromIrq(&owi_irq_done);
    }
   else
    {
        TIM_ClearInterruptFlag( OWI_TIMER );
    }
}

static int OWRWBit(int bit)
{
    bit_value = bit;
    TIM_Compare1(OWI_TIMER) = 2*(3);
    TIM_Compare2(OWI_TIMER) = 2*(3 +6);
    TIM_Compare3(OWI_TIMER) = 2*(3 +6 +9);
    TIM_Compare4(OWI_TIMER) = 2*(3 +6 +9 +51);
/* TIM_Update will cause a Update Interrupt, so we can't use the rollover
   with the Auto reload Value as final signal*/
    CM3BBREG(OWI_TIMER, TIM_TypeDef, DIER, _BI32(TIM_DIER_CC1IE)) = 1;
    CM3BBREG(OWI_TIMER, TIM_TypeDef, DIER, _BI32(TIM_DIER_CC2IE)) = 1;
    CM3BBREG(OWI_TIMER, TIM_TypeDef, DIER, _BI32(TIM_DIER_CC3IE)) = 1;
    CM3BBREG(OWI_TIMER, TIM_TypeDef, DIER, _BI32(TIM_DIER_CC4IE)) = 1;
    TIM_C1IRQEnable( OWI_TIMER);
    TIM_C2IRQEnable( OWI_TIMER);
    TIM_C3IRQEnable( OWI_TIMER);
    TIM_C4IRQEnable( OWI_TIMER);
    TIM_IntRegister(OWI_TIMER, irq_ow_bit);
    TIM_Counter(OWI_TIMER) =  0;
    TIM_Update( OWI_TIMER); /* Reload the values*/
    TIM_StartTimer( OWI_TIMER );
    NutEventWait(&owi_irq_done, 10);
    while (TIM_Counter( OWI_TIMER) < 10 + 6*4 + 9*4 + 51*4){};
    TIM_StopTimer( OWI_TIMER);
    return owi_sample;
}

void OWWriteBit(int b)
{
    OWRWBit(b);
}

int OWReadBit(void)
{
    return OWRWBit(1);
}

void OWWriteByte(uint8_t byte)
{
    int i;
    for(i=0; i<8; i++)
        OWRWBit(byte & 1<<i);
}

uint8_t OWReadByte(void)
{
    int i;
    uint8_t ret = 0;
    for(i=0; i<8; i++)
        ret |= (OWRWBit(1)<<i);
    return ret;
}

/* from u2c_owi.c*/
uint8_t OWRomSearch(uint8_t diff, uint64_t *hid)
{
    uint8_t i,j, next_diff;
    uint8_t b;
    uint8_t *id = (uint8_t *)hid;

    if (OWTouchReset())
        return  PRESENCE_ERR;
    OWWriteByte(SEARCH_ROM);

    next_diff = LAST_DEVICE;                      // unchanged on last device
    i = 8 * 8;                                    // 8 bytes
    do{
        j = 8;                                      // 8 bits
        do{
            b = OWReadBit();                       // read bit
            if( OWReadBit() ){                     // read complement bit
                if( b )                                 // 11
                    return DATA_ERR;                      // data error
            }else{
                if( !b ){                               // 00 = 2 devices
                    if( diff > i ||
                        ((*id & 1) && diff != i) ){
                        b = 1;                              // now 1
                        next_diff = i;                      // next pass 0
                    }
                }
            }
            OWWriteBit(b );                           // write bit
            *id >>= 1;
            if( b )                                   // store bit
                *id |= 0x80;
            i--;
        }while( --j );
        id++;                                       // next byte
    }while( i );
    return next_diff;                             // to continue search
}

int w1_command( uint8_t command, uint64_t *hid )
{
    int i;

    if (OWTouchReset())
    {
        return PRESENCE_ERR;
    }
    if (hid)
    {
        uint8_t *id = (uint8_t *)hid;

        OWWriteByte( MATCH_ROM );                    // to a single device
        for (i=0; i<8; i++)
            OWWriteByte( id[i] );
    }
    else {
        OWWriteByte( SKIP_ROM );                     // to all devices
    }
    OWWriteByte( command );
    return 0;
}
