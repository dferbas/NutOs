/*
 * Copyright 2012-2016 by Embedded Technologies s.r.o. All rights reserved.
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

#include <cfg/arch.h>
#include <arch/m68k.h>
#include <dev/gpio.h>
#include <dev/board.h>

/*!
 * \brief Early poseidon hardware initialization.
 *
 * Add code here to configure board for all applications.
 *
 * \note It is called if NUT_INIT_BOARD macro is defined (using Nut/OS Configurator).
 */
void NutBoardInit(void)
{
#if PLATFORM_SUB == REV_C
	/* Set GPIO function to enable Usart2 (RS-232). Set pin FORCEON. */
	GpioPinConfigSet(PORTAN, 7, GPIO_CFG_OUTPUT);
	GpioPinSetHigh(PORTAN, 7);

	/*
		BUT   .. PORTTA .. 0x0001

		IN 1  .. PORTTA .. 0x0008
		IN 2  .. PORTNQ .. 0x0002
		IN 3  .. PORTNQ .. 0x0080

		LED 1 .. PORTAN .. 0x0002
		LED 2 .. PORTAN .. 0x0004
		LED 3 .. PORTAN .. 0x0008
		LED 4 .. PORTAN .. 0x0010

		OUT   .. PORTAN .. 0x0001
	 */

	/*
	 * PORT AN
	 */
	/* After start, Relay is switched off */
	MCF_GPIO_CLRAN = ~0x01;

#if 0	//this doubles clearing the port through CLR register
	/* PORTAN: PORTAN4=0,PORTAN3=0,PORTAN2=0,PORTAN1=0,PORTAN0=0 */
	MCF_GPIO_PORTAN &= ~0x1E;
#endif

	/* DDRAN: DDRAN4=1,DDRAN3=1,DDRAN2=1,DDRAN1=1,DDRAN0=1 */
	MCF_GPIO_DDRAN |= 0x1F;

	/* PANPAR: PANPAR4=0,PANPAR3=0,PANPAR2=0,PANPAR1=0,PANPAR0=0 */
	MCF_GPIO_PANPAR &= ~0x1F;

	/*
	 * PORT TA
	 */
	/* DDRTA: DDRTA3=0,DDRTA0=0 */
	MCF_GPIO_DDRTA &= ~0x09;

	/* PTAPAR: PTAPAR3=0,PTAPAR0=0 */
	MCF_GPIO_PTAPAR &= ~0xC3;

	/*
	 * PORT NQ
	 */
	/* DDRNQ: DDRNQ7=0,DDRNQ1=0 */
	MCF_GPIO_DDRNQ &= ~0x82;

	/* PNQPAR: PNQPAR7=0,PNQPAR1=0 */
	MCF_GPIO_PNQPAR &= ~0xC00C;

#elif PLATFORM_SUB == REV_D || PLATFORM_SUB == REV_F
	/*
		NEW:
		BUT   .. PORTTA .. 0x0001 (0)

		IN1   .. PORTTA .. 0x0002 (1)
		IN2   .. PORTTA .. 0x0004 (2)
		IN3   .. PORTTA .. 0x0008 (3)

		LED 1 .. PORTTC .. 0x0001 (0)
		LED 2 .. PORTTC .. 0x0002 (1)
		LED 3 .. PORTTC .. 0x0004 (2)
		LED 4 .. PORTTC .. 0x0008 (3)

		OUT   .. PORTAN .. 0x0001 (0)
	 232 EN-  .. PORTAN .. 0x0008 (3)
	 232 SD   .. PORTAN .. 0x0010 (4)
	 232 DTR  .. PORTAN .. 0x0020 (5)
	 */

	/*
	 * Set GPIO function to enable Usart2 (HBUS RS-232).
	 * Set pins \EN and SD (shutdown).
	 * relay OFF (OUT)
	 */
	GpioPortSetLow(PORTAN, _BV(4) | _BV(3) | _BV(0));
	GpioPinSetHigh(PORTAN, 5);

	GpioPortConfigSet(PORTAN, _BV(5) | _BV(4) | _BV(3) | _BV(0), GPIO_CFG_OUTPUT);

#if PLATFORM_SUB == REV_F
	/* Set GPIO function to check 485/232 piggy back. */
	GpioPinConfigSet(PORTQS, 6, GPIO_CFG_INPUT);
#endif

	/*
	 * PORT TA - button, IN1-3 (inputs)
	 */
	GpioPortConfigSet(PORTTA, _BV(3) | _BV(2) | _BV(1) | _BV(0), GPIO_CFG_INPUT);	//button, IN1-3

	/*
	 * PORT TC - LEDs (output)
	 */
	GpioPortSetLow(PORTTC, _BV(3) | _BV(2) | _BV(1) | _BV(0));							//LEDs OFF
	GpioPortConfigSet(PORTTC, _BV(3) | _BV(2) | _BV(1) | _BV(0), GPIO_CFG_OUTPUT);		//LEDs 1-4

#else
	#error "Please define User Platform Macro PLATFORM_SUB in Nut/OS Configurator."
#endif
}

/*
 * Application API
 */
void SetDtrState(int state)
{
	/*
	 * For control signals (ie. RTS, CTS, DTR, DSR, ...):
	 *  logical 0: -3 V .. -15 V
	 *  logical 1: +3 V .. +15 V.
	 * DTE signals its ready state with logical 1 on this pin.
	 * We set log. 0 for DTR by setting the pin to log. 1.
	 */
	GpioPinSet(HBUS232_DTR_OUT_PORT, HBUS232_DTR_OUT_PIN, 1 - state);
}

EPiggyBackState GetPiggyBackState(void)
{
	if (GpioPinGet(PIGGYBACK232_IN_PORT, PIGGYBACK232_IN_PIN))
		return YZ_HALFDUP_485;

	return YZ_FULLDUP_232;
}

//------------------------------------------------------------------------------
void BoardInitExtram(void)
{
    extern void *__extram_start;
    extern void *__extram_size;

	MCF_GPIO_PTHPAR = 0x5555;	// Enable Data Lines D0-D7
	MCF_GPIO_PTEPAR = 0xFF;		// Enable Address Lines A0-A7
	MCF_GPIO_PTFPAR = 0xFF;		// Enable Address Lines A8-A15
	MCF_GPIO_PTGPAR = MCF_GPIO_PTGPAR_MB_A16_MB_A16
					| MCF_GPIO_PTGPAR_MB_A17_MB_A17
					| MCF_GPIO_PTGPAR_MB_A18_MB_A18
					| MCF_GPIO_PTGPAR_MB_A19_MB_A19
					| MCF_GPIO_PTGPAR_MB_CS0_MB_CS0
					| MCF_GPIO_PTGPAR_MB_OE_MB_OE
					| MCF_GPIO_PTGPAR_MB_RW_MB_RW;

	/* Pro inicializaci externi pameti nelze pouzit GPIO driver, ktery pouziva externi pamet */
//    GpioPortConfigSet(PORTTH, 0xFF, GPIO_CFG_PERIPHERAL0);  /* Enable Data Lines D0-D7 */
//    GpioPortConfigSet(PORTTE, 0xFF, GPIO_CFG_PERIPHERAL0);  /* Enable Address Lines A0-A7 */
//    GpioPortConfigSet(PORTTF, 0xFF, GPIO_CFG_PERIPHERAL0);  /* Enable Address Lines A8-A15 */
//    GpioPortConfigSet(PORTTG, 0xEF, GPIO_CFG_PERIPHERAL0);

    MCF_FBCS_CSAR(0) = MCF_FBCS_CSAR_BA(((uint32_t) &__extram_start));
    MCF_FBCS_CSMR(0) = ((((uint32_t) &__extram_size) - 1) & 0xFFFF0000) | MCF_FBCS_CSMR_V;
    MCF_FBCS_CSCR(0) = MCF_FBCS_CSCR_AA | MCF_FBCS_CSCR_PS_8;
}


//------------------------------------------------------------------------------
void BackDoorEnable(void)
{
	extern void *__rambar;

	/* Enable on-chip modules (FEC, ... ) to access internal SRAM */
	MCF_SCM_RAMBAR = (0
			| MCF_SCM_RAMBAR_BA((uint32_t)&__rambar)
			| MCF_SCM_RAMBAR_BDE);
}
