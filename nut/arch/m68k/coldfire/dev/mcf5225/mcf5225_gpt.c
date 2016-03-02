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

#include <arch/m68k.h>
#include <arch/m68k/coldfire/mcf5225/gpt_mcf5225.h>
#include <dev/irqreg.h>
#include <string.h>
#include <sys/event.h>
#include <sys/types.h>
#include <dev/gpio.h>
#include <stdlib.h>

/*!
 * \addtogroup xgMcf5225
 */
/*@{*/

static HANDLE *GptPAEHandler = NULL;


/*! \brief GPT Pulse Accumulator Event Interrupt
 *
 * \param *arg
 */
static void IntHandlerPAEvent(void *arg)
{
	if (GptPAEHandler != NULL)
		NutEventPostFromIrq(GptPAEHandler);

	//TODO zakomentovat, je v IH
	MCF_GPT_GPTPAFLG |= MCF_GPT_GPTPAFLG_PAIF;
}

/*! \brief Initialize GPT as a pulse accumulator counter
 *
 * \param *pae_handler Function used from interrupt
 */
void Mcf5225GptInitPA(HANDLE *pae_handler)
{
	//GptInit();
	MCF_GPT_GPTIOS &= ~MCF_GPT_GPTIOS_IOS3;
	//MCF_GPT_GPTSCR1 = 0; //MCF_GPT_GPTSCR1_GPTEN; // Enable GPT
	MCF_GPT_GPTCTL1 &= ~(MCF_GPT_GPTCTL1_OL3 | MCF_GPT_GPTCTL1_OM3);
	MCF_GPT_GPTCTL2 = 0x00;

	MCF_GPT_GPTPACTL = 0x00;
	///MCF_GPT_GPTPACNT = 65000; // Clear PA Counter

// JS TODO - predelat na gpio
//	MCF_GPIO_PTAPAR &= ~(MCF_GPIO_PTAPAR_PTAPAR3(3));
//	MCF_GPIO_PTAPAR |= (MCF_GPIO_PTAPAR_PTAPAR3(1));

	/* Enables the pulse accumulator. */
	///MCF_GPT_GPTPACTL |= MCF_GPT_GPTPACTL_PAE; // Enable PA if not MCF_GPT_GPTSCR1_GPTEN

	/* Save pae_handler into global variable used from interrupt */
	GptPAEHandler = pae_handler;

	NutRegisterIrqHandler(&sig_GPT_PAI, IntHandlerPAEvent, NULL);
	NutIrqEnable(&sig_GPT_PAI);

	Mcf5225GptClearPACounter();
	Mcf5225GptStartPA();
}

/*! \brief Enable Pulse accumulator.
 *	If enabled MCF_GPT_GPTSCR1_GPTEN, this function have not effect.
 *
 */
void Mcf5225GptStartPA(void)
{
    MCF_GPT_GPTPACTL |= MCF_GPT_GPTPACTL_PAE; // Enable PA
}

/*! \brief Disable Pulse accumulator.
 *	If enabled MCF_GPT_GPTSCR1_GPTEN, this function have not effect.
 *
 */
void Mcf5225GptStopPA(void)
{
	MCF_GPT_GPTPACTL &= ~MCF_GPT_GPTPACTL_PAE; // Disable PA
}

/*! \brief Clear Pa
 *
 */
void Mcf5225GptClearPACounter(void)
{
	MCF_GPT_GPTPACNT = 0; // Clear PA Counter
}

/*! \brief Get value of pulse accumulator
 *
 * \return MCF_GPT_GPTPACNT
 */
uint16_t Mcf5225GptGetPACounter(void)
{
	return MCF_GPT_GPTPACNT;
}

/* ******************************************************
 * Counter implementation
 * ******************************************************
 * Firstly, you need to use init function, then you can use control functions (Enable,Disable,Clear,Get,Start,Stop)
 */

#define	STABLE_COUNT		5000	//ochrana proti zakmitum na meridlu => 26.667ms pri prescaleru 7 (128)  = (1 / 24000000) * 128 * 1000 * 5000

//#define	DEBUG_INT_EVENTS
#define	INT_EVENTS_MAX		200

typedef struct
{
	int			counter;
	HANDLE		*handler;
	uint16_t	captured_count;
#ifdef	DEBUG_INT_EVENTS
	uint16_t	captured_count_prev;
#endif
} GptCounterS;

static	volatile GptCounterS	GptCounter[MCF_GPT_CHANNEL_COUNT];
static	int						Gptcounter_GPCTL2_mask = 0;	//TODO: initialized only after power up

#ifdef	DEBUG_INT_EVENTS
typedef struct
{
	uint8_t	channel;				//use uint8_t instead of int with faster access to save space
	int		timer_count;
	int     timer_count_handled;
} InterruptEventS;

static	InterruptEventS	int_events[INT_EVENTS_MAX];
static	int int_event_count = 0;
#endif


/*! \brief Fce called in interrupt event
 *
 * \param *arg 	Channel Index (1-3)
 */
static void IntHandlerCaptureEvent(void *arg)
{
	int						channel = (int)arg;
	volatile GptCounterS	*p_gptCounter = &GptCounter[channel];
	uint16_t				captured_count;
	int 					tmp;

	//ignore flickers
	captured_count = MCF_GPT_GPTC(channel);
#ifdef	DEBUG_INT_EVENTS
	if(int_event_count < INT_EVENTS_MAX)
	{
		int_events[int_event_count].channel = channel;
		int_events[int_event_count].timer_count = captured_count;
		int_events[int_event_count].timer_count_handled = MCF_GPT_GPTCNT;
		int_event_count++;
	}
#endif
	if ((tmp = abs(captured_count - p_gptCounter->captured_count)) > STABLE_COUNT)
	{
#ifdef	DEBUG_INT_EVENTS
		p_gptCounter->captured_count_prev = p_gptCounter->captured_count;	//save previous value
#endif
		p_gptCounter->captured_count = captured_count;		//save for next capture event
		p_gptCounter->counter++;							//pulse detected, accumulate

		if (p_gptCounter->handler != NULL)
			NutEventPostFromIrq(p_gptCounter->handler);		//signal application
	}
#ifdef	DEBUG_INT_EVENTS
	else
		tmp = captured_count;		//just for debug

	if (MCF_GPT_GPTFLG1 & MCF_GPT_GPTFLG1_CF(channel))
	{
		//another event occured during this interrupt processing
		tmp = captured_count;		//just for debug
	}
#endif
}

/*! \brief Start GPT Counting on channel (n)
 *
 * \param channel 			Channel Index (1-3)
 * \param *counter_handler 	Function used from interrupt
 */
void Mcf5225GptCounterInit(int channel, HANDLE *counter_handler)
{
	static IRQ_HANDLER	*sig_GPT[MCF_GPT_CHANNEL_COUNT] = {
			&sig_GPT_C0F,
			&sig_GPT_C1F,
			&sig_GPT_C2F,
			&sig_GPT_C3F
	};

	//following operations result in setting zeros, which are default after power up, so save code space
#if 0
	MCF_GPT_GPTIOS &= ~MCF_GPT_GPTIOS_IOS(channel);
	MCF_GPT_GPTSCR1 &= ~MCF_GPT_GPTSCR1_GPTEN; // Disable GPT

	MCF_GPT_GPTCTL1 &= ~MCF_GPT_GPTCTL1_OUTPUT_MASK(channel);
	MCF_GPT_GPTCTL1 |=  MCF_GPT_GPTCTL1_OUTPUT_NOTHING(channel);

	MCF_GPT_GPTCTL2 &= ~MCF_GPT_GPTCTL2_INPUT_MASK(channel);
#endif

	/* Save counter_handler into global variable used from interrupt */
	GptCounter[channel].handler = counter_handler;

	MCF_GPT_GPTSCR2 = MCF_GPT_GPTSCR2_PR(7);

	NutRegisterIrqHandler(sig_GPT[channel], IntHandlerCaptureEvent, (void *)channel);
	NutIrqEnable(sig_GPT[channel]);

	Mcf5225GptCounterClear(channel);
}

/*! \brief Start GPT Counting on channel (n)
 *
 * \param channel 	Channel Index (1-3)
 */
void Mcf5225GptCounterStart(int channel)
{
	Gptcounter_GPCTL2_mask |=  MCF_GPT_GPTCTL2_INPUT_RISING(channel);
	GpioPinConfigSet(PORTTA, channel, GPIO_CFG_PERIPHERAL0 | GPIO_CFG_INPUT);	//set PIN functionality to GPT
}

/*! \brief Stop GPT Counting on channel (n)
 *
 * \param channel 	Channel Index (1-3)
 */
void Mcf5225GptCounterStop(int channel)
{
	GpioPinConfigSet(PORTTA, channel, GPIO_CFG_INPUT);							//return pin functionality to GPIO
	Gptcounter_GPCTL2_mask &= ~MCF_GPT_GPTCTL2_INPUT_MASK(channel);
}

/*! \brief Start GPT Counting input capture events
 *
 */
void Mcf5225GptCountersEnable(void)
{
	MCF_GPT_GPTCTL2 |=  Gptcounter_GPCTL2_mask;
	MCF_GPT_GPTSCR1 |=  MCF_GPT_GPTSCR1_GPTEN; // Enable GPT
}

/*! \brief Stop GPT Counting
 *
 */
void Mcf5225GptCountersDisable(void)
{
	MCF_GPT_GPTSCR1 &= ~MCF_GPT_GPTSCR1_GPTEN; // Disable GPT
	MCF_GPT_GPTCTL2 &= ~Gptcounter_GPCTL2_mask;
}


/*! \brief Clear GPT Channel Counter
 *
 * \param channel 	Channel Index (1-3)
 */
void Mcf5225GptCounterClear(int channel)
{
	GptCounter[channel].counter = 0;
}


/*! \brief Get GPT Channel Counter
 *
 * \param channel 	Channel Index (1-3)
 *
 * \return GptCounter[channel].counter
 */
int Mcf5225GptCounterGet(int channel)
{
	return GptCounter[channel].counter;
}
