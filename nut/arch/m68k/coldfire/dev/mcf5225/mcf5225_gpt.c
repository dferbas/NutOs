#include <arch/m68k.h>
#include <arch/m68k/coldfire/mcf5225/gpt_mcf5225.h>
#include <dev/irqreg.h>
#include <string.h>
#include <sys/event.h>
#include <sys/types.h>

static HANDLE *GptPAEHandler = NULL;

/*
 * GPT Pulse Accumulator Event Interrupt
 */
static void IntHandlerPAEvent(void *arg)
{
	if (GptPAEHandler != NULL)
		NutEventPostFromIrq(GptPAEHandler);

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
	MCF_GPT_GPTPACNT = 65000; // Clear PA Counter
	
// JS TODO - predelat na gpio
//	MCF_GPIO_PTAPAR &= ~(MCF_GPIO_PTAPAR_PTAPAR3(3));
//	MCF_GPIO_PTAPAR |= (MCF_GPIO_PTAPAR_PTAPAR3(1));
	
	/* Enables the pulse accumulator. */
	MCF_GPT_GPTPACTL |= MCF_GPT_GPTPACTL_PAE; // Enable PA if not MCF_GPT_GPTSCR1_GPTEN

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
