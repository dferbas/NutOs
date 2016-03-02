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

#include <cfg/clock.h>
#include <arch/m68k.h>
#include <sys/nutdebug.h>

/*!
 * \addtogroup xgMcf51cn
 */
/*@{*/

#define MCG_INTERNAL_REF_CLOCK 			32768

#define MCG_MAX_CPU_FREQ 				50000000
#define MCG_MAX_INT_REF_FREQ 			39062
#define MCG_MIN_INT_REF_FREQ 			31250
#define MCG_MAX_EXT_REF_FREQ 			2000000
#define MCG_MIN_EXT_REF_FREQ 			1000000

#ifndef MCG_EXT_REF_DIVIDE_FACTOR
#define MCG_EXT_REF_DIVIDE_FACTOR       1
#endif

#ifndef MCG_FLL_DCO_MODE
#define MCG_FLL_DCO_MODE_VALUE			0
#else
#define MCG_FLL_DCO_MODE_VALUE			1
#endif

#ifndef MCG_FLL_MULTIPLY_FACTOR
#define MCG_FLL_MULTIPLY_FACTOR       	512
#endif

#ifndef MCG_PLL_MULTIPLY_FACTOR
#define MCG_PLL_MULTIPLY_FACTOR       	4
#endif

#if !(defined(MCG_MODE_FEI) || defined(MCG_MODE_FEE) || defined(MCG_MODE_FBI)\
	|| defined(MCG_MODE_FBE) || defined(MCG_MODE_PEE) || defined(MCG_MODE_PBE)\
	|| defined(MCG_MODE_BLPI) || defined(MCG_MODE_BLPE))
#define MCG_MODE_FEI
#endif

void Mcf51cnMcgRunPLL(uint8_t divider, uint8_t multiply);
void Mcf51cnMcgRunFLL(uint8_t divider, uint8_t dco, uint16_t multiply);

void Mcf51cnMcgInit(void);

void Mcf51cnMcgFEItoFEE(void);
void Mcf51cnMcgFEItoFBI(void);
void Mcf51cnMcgFEItoFBE(void);
void Mcf51cnMcgFBEtoPBE(void);
void Mcf51cnMcgFBEtoBLPE(void);
void Mcf51cnMcgBLPEtoPBE(void);
void Mcf51cnMcgPBEtoPEE(void);
void Mcf51cnMcgFBItoBLPI(void);

void Mcf51cnMcgInitToFEI(void);
void Mcf51cnMcgInitToPEE(void);
void Mcf51cnMcgInitToFEE(void);
void Mcf51cnMcgInitToFBI(void);
void Mcf51cnMcgInitToFBE(void);
void Mcf51cnMcgInitToPBE(void);
void Mcf51cnMcgInitToBLPI(void);
void Mcf51cnMcgInitToBLPE(void);

void Mcf51cnMcgInitClock(void)
{
#if defined(MCG_MODE_FEI)
	Mcf51cnMcgInitToFEI();
#elif defined(MCG_MODE_FEE)
	Mcf51cnMcgInitToFEE(); // TODO test it
#elif defined(MCG_MODE_FBI)
	Mcf51cnMcgInitToFBI(); // TODO test it
#elif defined(MCG_MODE_FBE)
	Mcf51cnMcgInitToFBE(); // TODO test it
#elif defined(MCG_MODE_PEE)
	Mcf51cnMcgInitToPEE();
#elif defined(MCG_MODE_PBE)
	Mcf51cnMcgInitToPBE(); // TODO test it
#elif defined(MCG_MODE_BLPI)
	Mcf51cnMcgInitToBLPI(); // TODO test it
#elif defined(MCG_MODE_BLPE)
	Mcf51cnMcgInitToBLPE(); // TODO test it
#endif
}

uint32_t Mcf51cnMcgGetFFCLK(void)
{
	uint8_t value_RDIV;
	uint32_t extFreq;

	if (MCF_MCG_C1 & MCF_MCG_C1_IREFS)
	{
		/* Fixed-frequency clock is internal reference clock. */
		return MCG_INTERNAL_REF_CLOCK;
	}
	else
	{
		/* Fixed-frequency clock is external reference clock. */
		extFreq = MCG_EXT_REF_FREQ;
		if (MCF_MCG_C3 & MCF_MCG_C3_DIV32)
		{
			/* Divide frequency by DIV32 */
			extFreq /= 32;
		}

		/* Divide frequency by RDIV */
		value_RDIV = (MCF_MCG_C1 & MCF_MCG_C1_RDIV) >> MCF_MCG_C1_RDIV_BITNUM;

		return (extFreq >> value_RDIV);
	}

}

void Mcf51cnMcgInitToFEI(void)
{
	Mcf51cnMcgInit();
	Mcf51cnMcgRunFLL(MCG_EXT_REF_DIVIDE_FACTOR, MCG_FLL_DCO_MODE_VALUE, MCG_FLL_MULTIPLY_FACTOR);
}

void Mcf51cnMcgInitToPEE(void)
{
	Mcf51cnMcgInit();
	Mcf51cnMcgFEItoFBE();
	Mcf51cnMcgFBEtoBLPE();
	Mcf51cnMcgRunPLL(MCG_EXT_REF_DIVIDE_FACTOR, MCG_PLL_MULTIPLY_FACTOR);
	Mcf51cnMcgBLPEtoPBE();
	Mcf51cnMcgPBEtoPEE();
}

void Mcf51cnMcgInitToFEE(void)
{
	Mcf51cnMcgInit();
	Mcf51cnMcgFEItoFEE();
	Mcf51cnMcgRunFLL(MCG_EXT_REF_DIVIDE_FACTOR, MCG_FLL_DCO_MODE_VALUE, MCG_FLL_MULTIPLY_FACTOR);
}
void Mcf51cnMcgInitToFBI(void)
{
	Mcf51cnMcgInit();
	Mcf51cnMcgFEItoFBI();
	Mcf51cnMcgRunFLL(MCG_EXT_REF_DIVIDE_FACTOR, MCG_FLL_DCO_MODE_VALUE, MCG_FLL_MULTIPLY_FACTOR);
}
void Mcf51cnMcgInitToFBE(void)
{
	Mcf51cnMcgInit();
	Mcf51cnMcgFEItoFBE();
	Mcf51cnMcgRunFLL(MCG_EXT_REF_DIVIDE_FACTOR, MCG_FLL_DCO_MODE_VALUE, MCG_FLL_MULTIPLY_FACTOR);

}
void Mcf51cnMcgInitToPBE(void)
{
	Mcf51cnMcgInit();
	Mcf51cnMcgFEItoFBE();
	Mcf51cnMcgFBEtoBLPE();
	Mcf51cnMcgRunPLL(MCG_EXT_REF_DIVIDE_FACTOR, MCG_PLL_MULTIPLY_FACTOR);
	Mcf51cnMcgBLPEtoPBE();
}
void Mcf51cnMcgInitToBLPI(void)
{
	Mcf51cnMcgInit();
	Mcf51cnMcgFEItoFBI();
	Mcf51cnMcgFBItoBLPI();
}
void Mcf51cnMcgInitToBLPE(void)
{
	Mcf51cnMcgInit();
	Mcf51cnMcgFEItoFBE();
	Mcf51cnMcgFBEtoBLPE();
}

void Mcf51cnMcgRunPLL(uint8_t divider, uint8_t multiply)
{
	/*PLLS (bit 6) set to 1, selects the PLL. At this time, with an RDIV value of %011, the FLL
	 reference divider of 256 is switched to the PLL reference divider of 8 (see Table 6-3),
	 resulting in a reference frequency of 8 MHz/ 8 = 1 MHz. In BLPE mode,changing the PLLS
	 bit only prepares the MCG for PLL usage in PBE mode
	 – DIV32 (bit 4) remains set at 1. Because the MCG is in a PLL mode, the DIV32 bit is ignored.
	 Keeping it set at 1 makes transitions back into an FLL external mode easier.
	 – VDIV (bits 3-0) set to %1000, or multiply-by-32 because 1 MHz reference * 32= 32MHz.
	 In BLPE mode, the configuration of the VDIV bits does not matter because the PLL is
	 disabled. Changing them only sets up the multiply value for PLL usage in PBE mode*/
	uint8_t value_RDIV;

#ifdef MCG_EXT_REF_FREQ
	/*	PEE, PBE Mode: F.ext / 'Ref. Clock Divider' must be in the range of 1 MHz to 2 MHz*/
	uint32_t result_freq = ((uint32_t)MCG_EXT_REF_FREQ) / divider;

	NUTASSERT((MCG_MIN_EXT_REF_FREQ < result_freq) && (result_freq < MCG_MAX_EXT_REF_FREQ));

	/*	CPUCLK must be lower than 50 MHz*/
	result_freq *= multiply;

	NUTASSERT(result_freq <= (MCG_MAX_CPU_FREQ + 1));

#endif

	for (value_RDIV = 0; value_RDIV < 8; ++value_RDIV)
	{
		divider >>= 1;
		if (divider == 0)
		{
			break;
		}
	}
	/*
	 External Reference Divider — Selects the amount to divide down the external reference clock. If the PLL is selected, the
	 resulting frequency must be in the range 1 MHz to 2 MHz. See Table 6-2 and Table 6-3 for the divide-by factors.
	 */
	MCF_MCG_C1 = (MCF_MCG_C1 & ~MCF_MCG_C1_RDIV) | (value_RDIV << MCF_MCG_C1_RDIV_BITNUM);

	/*
	 VCO Divider — Selects the amount to divide down the VCO output of PLL. The VDIV bits establish the
	 multiplication factor (M) applied to the reference clock frequency
	 */
	MCF_MCG_C3 = (multiply / 4) << MCF_MCG_C3_VDIV_BITNUM | MCF_MCG_C3_DIV32;

	/*
	 Controls whether the PLL or FLL is selected. If the PLLS bit is clear, the PLL is disabled in all
	 modes. If the PLLS is set, the FLL is disabled in all modes. This bit should only be written in bypassed external
	 modes when CLKST=10.
	 */
	MCF_MCG_C3 |= MCF_MCG_C3_PLLS;
}

void Mcf51cnMcgRunFLL(uint8_t divider, uint8_t dco, uint16_t multiply)
{
	uint8_t value_RDIV;

	/*	Write to the MCF_MCG_C4 register to determine the DCO output (MCF_MCG_OUT) frequency range. Make
	 sure that the resulting bus clock frequency does not exceed the maximum specified bus clock
	 frequency of the device.
	 — By default, with DMX32 cleared to 0, the FLL multiplier for the DCO output is 512. For greater
	 flexibility, if a mid-range FLL multiplier of 1024 is desired instead, set the DRS[1:0] bits to
	 %01 for a DCO output frequency of 33.55 MHz. If a high-range FLL multiplier of 1536 is
	 desired instead, set the DRS[1:0] bits to %10 for a DCO output frequency of 50.33 MHz.
	 — When using a 32.768 kHz external reference, if the maximum low-range DCO frequency that
	 can be achieved with a 32.768 kHz reference is desired, set the DRS[1:0] bits to %00 and set
	 the DMX32 bit to 1. The resulting DCO output (MCF_MCG_OUT) frequency with the new multiplier
	 of 608 is 19.92 MHz.
	 — When using a 32.768 kHz external reference, if the maximum mid-range DCO frequency that
	 can be achieved with a 32.768 kHz reference is desired, set the DRS[1:0] bits to %01 and set
	 the DMX32 bit to 1. The resulting DCO output (MCF_MCG_OUT) frequency with the new multiplier
	 of 1216 is 39.85 MHz.
	 — When using a 32.768 kHz external reference, if the maximum high-range DCO frequency that
	 can be achieved with a 32.768 kHz reference is desired, set the DRS[1:0] bits to %10 and set
	 the DMX32 bit to 1. The resulting DCO output (MCF_MCG_OUT) frequency with the new multiplier
	 of 1824 is 59.77 MHz.*/

#if defined(MCG_EXT_REF_FREQ) && (defined(MCG_MODE_FEE) || defined(MCG_MODE_FBE))
	/*	FEE, FBE Mode: F.ext / 'Ref. Clock Divider' must be in the range of 31.25 kHz to 39.0625 kHz */
	uint32_t result_freq = ((uint32_t)MCG_EXT_REF_FREQ) / divider;

	NUTASSERT((MCG_MIN_INT_REF_FREQ < result_freq) || (result_freq > MCG_MAX_INT_REF_FREQ));

//	CPUCLK must be lower than 50 MHz
	result_freq *= multiply;

	NUTASSERT(result_freq > MCG_MAX_CPU_FREQ);

#endif

	/*Divide-by-32 Enable — Controls an additional divide-by-32 factor to the external reference clock for the FLL
	 when RANGE bit is set. When the RANGE bit is 0, this bit has no effect. Writes to this bit are ignored if PLLS bit is set.*/
	if (divider >= 512)
	{
		divider /= 512;
		MCF_MCG_C3 |= MCF_MCG_C3_DIV32;
	}
	else
	{
		MCF_MCG_C3 &= ~MCF_MCG_C3_DIV32;
	}

	for (value_RDIV = 0; value_RDIV < 8; ++value_RDIV)
	{
		divider >>= 1;
		if (divider == 0)
		{
			break;
		}
	}

	/* External Reference Divider — If the FLL is selected, the resulting frequency must
	 * be in the range 31.25 kHz to 39.0625 kHz. See Table 6-2 and Table 6-3 for the divide-by factors.*/
	uint8_t mcg1 = (MCF_MCG_C1 & ~MCF_MCG_C1_RDIV) | (value_RDIV << MCF_MCG_C1_RDIV_BITNUM)
			| MCF_MCG_C1_IREFS	//Internal reference clock selected
			| MCF_MCG_C1_IRCLKEN; //MCGIRCLK active

	MCF_MCG_C1 = mcg1;

	uint8_t mcg4;

	switch (multiply)
	{
		case 1216: // 1024, 1216 Mid range
		case 1024:
			mcg4 = 1 << MCF_MCG_C4_DRST_DRS_BITNUM | (dco ? MCF_MCG_C4_DMX32 : 0);
			break;
		case 1536: // 1536, 1824 High range
		case 1824:
			mcg4 = 2 << MCF_MCG_C4_DRST_DRS_BITNUM | (dco ? MCF_MCG_C4_DMX32 : 0);
			break;
		default: // 512, 608 for Low range.
			mcg4 = (dco ? MCF_MCG_C4_DMX32 : 0);
			break;
	}
	MCF_MCG_C4 = mcg4;

	/*Wait for the LOCK bit in MCF_MCG_SC to become set, indicating that the FLL has locked to the new
	 multiplier value designated by the DRS and DMX32 bits.*/
	while ((MCF_MCG_SC & MCF_MCG_SC_LOCK) == 0)
	{
	}
}

void Mcf51cnMcgInit(void)
{
	uint8_t mask_MCGC2 = 0; //initialize MCF_MCG_C2
	uint8_t mask_MCGC3; 	//initialize MCF_MCG_C3

#if MCG_BUS_DIV <= 2 // Values 1, 2, 4
//	Selects the amount to divide down the clock source selected by 'clock mode' below.
	mask_MCGC2 |= ((MCG_BUS_DIV >> 1) << MCF_MCG_C2_BDIV_BITNUM);
#else	// Value 8
	mask_MCGC2 |= (3 << MCF_MCG_C2_BDIV_BITNUM);
#endif

#ifdef MCG_INT_REF_CLKEN
//	MCF_MCG_IRCLK may be used by several peripherals as a clock source instead of BUSCLK.
//	Enables the external reference clock for use as MCF_MCG_ERCLK.
	mask_MCGC2 |= MCF_MCG_C2_ERCLKEN;
#endif

#ifdef MCG_EXT_REF_HIGH_GAIN_OSC //if
	/*	Use this mode for Crystal Oscilator
	 This option provides a higher amplitude output for the improved noise immunity
	 Supports following frequency ranges:
	 - 32 kHz to 100 kHz
	 - 1 MHz to 25 MHz*/
	mask_MCGC2 |= MCF_MCG_C2_HGO | MCF_MCG_C2_EREFS;
#endif
#ifdef MCG_EXT_REF_LOW_PWR_OSC //else if
	/*	Use this mode for Crystal Oscilator
	 This option provides the lower power consumption.
	 Supports following frequency ranges:
	 - 32 kHz to 100 kHz
	 - 1 MHz to 8 MHz*/
	mask_MCGC2 |= MCF_MCG_C2_EREFS;	//MCF_MCG_C2_HGO = 0
#endif
#ifdef MCG_EXT_REF_EXT_CLOCK //else if
	/*	Use this mode for External Clock Source
	 MCF_MCG_C2_HGO = 0 and MCF_MCG_C2_EREFS = 0*/
#endif

#ifdef MCG_EXT_REF_HIGH_FREQ_RANGE
	/*	Select for the high frequency external clock source (1 MHz to 40 MHz).
	 Deselect for the low frequency external clock source (32 kHz to 1 MHz).*/
	mask_MCGC2 |= MCF_MCG_C2_RANGE;
#endif

#ifdef MCG_EXT_REF_CLKEN
	/*Enables the external reference clock for use as MCF_MCG_ERCLK.
	 MCF_MCG_ERCLK may be used by several peripherals as a clock source instead of BUSCLK.*/
	mask_MCGC2 |= MCF_MCG_C2_ERCLKEN;
#endif

	MCF_MCG_C2 = mask_MCGC2;	// end initialize MCF_MCG_C2

#if defined(MCG_EXT_REF_HIGH_GAIN_OSC) || defined(MCG_EXT_REF_LOW_PWR_OSC)
	/*	wait here for the OSCINIT bit to become set indicating that the
	 external clock source has finished its initialization cycles and stabilized.*/
	while((MCF_MCG_SC & MCF_MCG_SC_OSCINIT) == 0U)
	{
	}
#endif

	mask_MCGC3 = (0x1 << MCF_MCG_C3_VDIV_BITNUM); //default value, lowest value
#ifdef MCG_EXT_REF_HIGH_FREQ_RANGE
			/* If the RANGE bit (bit 5) in MCF_MCG_C2 is set, set DIV32 in MCF_MCG_C3 to allow access to the proper RDIV values.*/
			mask_MCGC3 |= MCF_MCG_C3_DIV32;
#endif
#ifdef MCG_EXT_REF_CLK_MONITOR
	// The clock monitor issues reset after a loss of external clock is detected.
	mask_MCGC3 |= MCF_MCG_C3_CME;
#endif

	/* Determines if an interrupt request is made following a loss of lock indication.
	 * Use as a prevent from remain in wait for a lock */
	if ((mask_MCGC2 & MCF_MCG_C2_EREFS) && !(mask_MCGC2 & MCF_MCG_C2_BDIV))
	{
		mask_MCGC3 |= MCF_MCG_C3_LOLIE;
	}

	MCF_MCG_C3 = mask_MCGC3; // end initialize MCF_MCG_C3
}

void Mcf51cnMcgFEItoFEE(void)
{
	/*	If entering FEE mode, set RDIV appropriately, clear the IREFS bit to switch to the external
	 reference, and leave the CLKS bits at %00 so that the output of the FLL is selected as the
	 system clock source.*/

	MCF_MCG_C1 = (5 << MCF_MCG_C1_RDIV_BITNUM) //MCF_MCG_SC_CLKST = 0
	| MCF_MCG_C1_IRCLKEN;

	while (MCF_MCG_SC_IREFST != 0U)
	{
	}
}

void Mcf51cnMcgFEItoFBI(void)
{
	/*	To change from FEI clock mode to FBI clock mode, follow this procedure:
	 1. Change the CLKS bits in MCF_MCG_C1 to %01 so that the internal reference clock is selected as the
	 system clock source.
	 2. Wait for the CLKST bits in the MCF_MCG_SC register to change to %01, indicating that the internal
	 reference clock has been appropriately selected.*/

	MCF_MCG_C1 = (1 << MCF_MCG_C1_CLKS_BITNUM) | MCF_MCG_C1_IRCLKEN | MCF_MCG_C1_IREFS;

	while ((MCF_MCG_SC & MCF_MCG_SC_CLKST) != (1 << MCF_MCG_SC_CLKST_BITNUM))
	{
	}

}

void Mcf51cnMcgFEItoFBE(void)
{
	/*	If entering FBE, clear the IREFS bit to switch to the external reference and change the CLKS
	 bits to %10 so that the external reference clock is selected as the system clock source. The
	 RDIV bits should also be set appropriately here according to the external reference frequency
	 because although the FLL is bypassed, it remains on in FBE mode.*/

	MCF_MCG_C1 = (2 << MCF_MCG_C1_CLKS_BITNUM)
	/*	The internal reference can optionally be kept running by setting the IRCLKEN bit. This is
	 useful if the application switches back and forth between internal and external modes. For
	 minimum power consumption, leave the internal reference disabled while in an external clock
	 mode.*/
	| MCF_MCG_C1_IRCLKEN | (5 << MCF_MCG_C1_RDIV_BITNUM);

	/*	Loop until IREFST (bit 4) in MCF_MCG_SC is 0, indicating the external reference is the current
	 source for the reference clock*/
	while ((MCF_MCG_SC & MCF_MCG_SC_IREFST) != 0)
		;
	/*	Loop until CLKST (bits 3 and 2) in MCF_MCG_SC is %10, indicating that the external reference
	 clock is selected to feed MCF_MCG_OUT*/
	while ((MCF_MCG_SC & MCF_MCG_SC_CLKST) != (2 << MCF_MCG_SC_CLKST_BITNUM))
		;
}

void Mcf51cnMcgFBEtoPBE(void)
{
	/* !! Call RunPLL function before call this function !! */

	/*	Loop until PLLST (bit 5) in MCF_MCG_SC is set, indicating that the current source for the
	 PLLS clock is the PLL*/
	while ((MCF_MCG_SC & MCF_MCG_SC_PLLST) == 0)
		;

	/*Then loop until LOCK (bit 6) in MCF_MCG_SC is set, indicating that the PLL has acquired lock*/
	while ((MCF_MCG_SC & MCF_MCG_SC_LOCK) == 0)
		;
}

void Mcf51cnMcgFBEtoBLPE(void)
{
	/*If a transition through BLPE mode is desired, first set LP (bit 3) in MCGC2 to 1.*/
	MCF_MCG_C2 |= MCF_MCG_C2_LP;
}

void Mcf51cnMcgBLPEtoPBE(void)
{
	/* !! Call RunPLL function before call this function !! */

	/* If transitioning through BLPE mode, clear LP (bit 3) in MCF_MCG_C2 to 0 here to switch to
	 * PBE mode*/
	MCF_MCG_C2 &= ~MCF_MCG_C2_LP; // MCF_MCG_C2 = 0;

	Mcf51cnMcgFBEtoPBE();
}

void Mcf51cnMcgPBEtoPEE(void)
{
	/*	CLKS (bits7 and 6) in MCF_MCG_SC1 set to %00 to select the output of the PLL as the system
	 clock source*/
	MCF_MCG_C1 &= ~MCF_MCG_C1_CLKS;

	/*	Loop until CLKST (bits 3 and 2) in MCF_MCG_SC are %11, indicating that the PLL output is
	 selected to feed MCF_MCG_OUT in the current clock mode*/

	/* Wait until PLL clock is selected as a bus clock reference */
	while ((MCF_MCG_SC & MCF_MCG_SC_CLKST) != (3 << MCF_MCG_SC_CLKST_BITNUM))
		;
}

void Mcf51cnMcgFBItoBLPI(void)
{
	MCF_MCG_C2 |= MCF_MCG_C2_LP;
}
