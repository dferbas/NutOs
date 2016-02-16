#include <arch/m68k.h>
#include <arch/m68k/coldfire/mcf5225/gpt_mcf5225.h>
#include <dev/irqreg.h>
#include <string.h>
#include <sys/event.h>
#include <sys/types.h>
#include <dev/gpio.h>
#include <stdlib.h>

static HANDLE *GptPAEHandler = NULL;


/*
 * GPT Pulse Accumulator Event Interrupt
 */
static void IntHandlerPAEvent(void *arg)
{
	if (GptPAEHandler != NULL)
		NutEventPostFromIrq(GptPAEHandler);

	//TODO zakomentovat, je v IH
	MCF_GPT_GPTPAFLG |= MCF_GPT_GPTPAFLG_PAIF;
}

/*
 * Initialize GPT as a pulse accumulator counter
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

/*
 * Enable Pulse accumulator. If enabled MCF_GPT_GPTSCR1_GPTEN,
 * this function have not effect.
 */
void Mcf5225GptStartPA(void)
{
    MCF_GPT_GPTPACTL |= MCF_GPT_GPTPACTL_PAE; // Enable PA
}

/*
 * Disable Pulse accumulator. If enabled MCF_GPT_GPTSCR1_GPTEN,
 * this function have not effect.
 */
void Mcf5225GptStopPA(void)
{
	MCF_GPT_GPTPACTL &= ~MCF_GPT_GPTPACTL_PAE; // Disable PA
}

/*
 * Clear Pa
 */
void Mcf5225GptClearPACounter(void)
{
    MCF_GPT_GPTPACNT = 0; // Clear PA Counter
}

/*
 * Get value of pulse accumulator
 */
uint16_t Mcf5225GptGetPACounter(void)
{
    return MCF_GPT_GPTPACNT;
}

/* ******************************************************
 * Counter implementation
 * ******************************************************
 * Firstly you need to use init function, then you can use control functions (Enable,Disable,Clear,Get)
 */

#define STABLE_PERIOD     20    //Tohle je na pokus omyl, teda pokud nespocitame, jak rychle tomu citaci bezi hodiny

typedef struct
{
	int			counter;
	HANDLE		*handler;
	uint16_t	captured_count;
	uint16_t	captured_count_prev;
} GptCounterS;

static	GptCounterS	GptCounter[MCF_GPT_CHANNEL_COUNT];
static	int	Gptcounter_GPCTL2_mask = 0;	//TODO: initialized only after power up

static void IntHandlerCaptureEvent(void *arg)
{
	int			channel = (int)arg;
	GptCounterS	*p_gptCounter = &GptCounter[channel];
	uint16_t	captured_count;
	int tmp;

	//ignore flickers
	captured_count = MCF_GPT_GPTC(channel);
	if ((tmp = abs(captured_count - p_gptCounter->captured_count)) > STABLE_PERIOD)
	{
	    p_gptCounter->captured_count_prev = p_gptCounter->captured_count;    //save previous value
	    p_gptCounter->captured_count = captured_count;    //save for next capture event
	    p_gptCounter->counter++;                          //pulse detected, accumulate

	    if (p_gptCounter->handler != NULL)
	        NutEventPostFromIrq(p_gptCounter->handler);   //signal application
	}
	else
		tmp = captured_count;		//just for debug

	if (MCF_GPT_GPTFLG1 & MCF_GPT_GPTFLG1_CF(channel))
	{
		//another event occured during this interrupt processing
		tmp = captured_count;		//just for debug
	}
}

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
	//Gptcounter_GPCTL2_mask |=  MCF_GPT_GPTCTL2_INPUT_RISING(channel);
//	Gptcounter_GPCTL2_mask |=  MCF_GPT_GPTCTL2_INPUT_FALLING(channel);

	/* Save counter_handler into global variable used from interrupt */
	GptCounter[channel].handler = counter_handler;

	NutRegisterIrqHandler(sig_GPT[channel], IntHandlerCaptureEvent, (void *)channel);
	NutIrqEnable(sig_GPT[channel]);

	Mcf5225GptCounterClear(channel);
}

/*
 * Start GPT Counting on channel (n)
 */
void Mcf5225GptCounterStart(int channel)
{
//	MCF_GPT_GPTCTL2 |=  MCF_GPT_GPTCTL2_INPUT_RISING(channel);
	Gptcounter_GPCTL2_mask |=  MCF_GPT_GPTCTL2_INPUT_FALLING(channel);

	GpioPinConfigSet(PORTTA, channel, GPIO_CFG_PERIPHERAL0 | GPIO_CFG_INPUT);	//set PIN functionality to GPT
}

/*
 * Stop GPT Counting on channel (n)
 */
void Mcf5225GptCounterStop(int channel)
{
	GpioPinConfigSet(PORTTA, channel, GPIO_CFG_INPUT);							//return pin functionality to GPIO

//	MCF_GPT_GPTCTL2 &= ~MCF_GPT_GPTCTL2_INPUT_MASK(channel);
	Gptcounter_GPCTL2_mask &= ~MCF_GPT_GPTCTL2_INPUT_MASK(channel);
}

/*
 * Start GPT Counting input capture events
 */
void Mcf5225GptCountersEnable(void)
{
	MCF_GPT_GPTCTL2 |=  Gptcounter_GPCTL2_mask;
	MCF_GPT_GPTSCR1 |=  MCF_GPT_GPTSCR1_GPTEN; // Enable GPT
}

/*
 * Stop GPT Counting
 */
void Mcf5225GptCountersDisable(void)
{
	MCF_GPT_GPTSCR1 &= ~MCF_GPT_GPTSCR1_GPTEN; // Disable GPT
	MCF_GPT_GPTCTL2 &= ~Gptcounter_GPCTL2_mask;
}

/*
 * Clear GPT Channel Counter
 */
void Mcf5225GptCounterClear(int channel)
{
	GptCounter[channel].counter = 0;
}

/*
 * Get GPT Channel Counter
 */
int Mcf5225GptCounterGet(int channel)
{
	return GptCounter[channel].counter;
}

