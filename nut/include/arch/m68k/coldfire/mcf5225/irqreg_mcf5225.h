/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

#ifndef _DEV_IRQREG_H_
#error "Do not include this file directly. Use dev/irqreg.h instead!"
#endif

#define IH_FEC_LEVEL			 	5

// Usart0 (RS485) has more priority than Usart1 and Usart2(Debug)
#define IH_USART0_LEVEL			 	3
#define IH_USART1_LEVEL			 	3
#define IH_USART2_LEVEL			 	3

#define IH_QSPI_LEVEL			 	2

#define IH_I2C_LEVEL			 	3

/*
 * Interrupt level & priority setup
 *
 * IMPORTANT: Interrupt level and priority combination MUST be unique
 */
#define IPL_QSPI_TF     (MCF_INTC_ICR_IL(IH_QSPI_LEVEL) | MCF_INTC_ICR_IP(4))
#define IPL_UART0       (MCF_INTC_ICR_IL(IH_USART0_LEVEL) | MCF_INTC_ICR_IP(0))
#define IPL_UART1       (MCF_INTC_ICR_IL(IH_USART1_LEVEL) | MCF_INTC_ICR_IP(1))
#define IPL_UART2       (MCF_INTC_ICR_IL(IH_USART2_LEVEL) | MCF_INTC_ICR_IP(2))
#define IPL_I2C0        (MCF_INTC_ICR_IL(IH_I2C_LEVEL) | MCF_INTC_ICR_IP(3))
#define IPL_I2C1        (MCF_INTC_ICR_IL(IH_I2C_LEVEL) | MCF_INTC_ICR_IP(4))
#define IPL_PIT0	    (MCF_INTC_ICR_IL(4) | MCF_INTC_ICR_IP(0))
#define IPL_PIT1	    (MCF_INTC_ICR_IL(4) | MCF_INTC_ICR_IP(1))
#define IPL_FEC_RB		(MCF_INTC_ICR_IL(IH_FEC_LEVEL) | MCF_INTC_ICR_IP(5))
#define IPL_FEC_RF		(MCF_INTC_ICR_IL(IH_FEC_LEVEL) | MCF_INTC_ICR_IP(4))
#define IPL_FEC_TB		(MCF_INTC_ICR_IL(IH_FEC_LEVEL) | MCF_INTC_ICR_IP(3))
#define IPL_FEC_TF		(MCF_INTC_ICR_IL(IH_FEC_LEVEL) | MCF_INTC_ICR_IP(2))
#define IPL_GPT_PAOV    (MCF_INTC_ICR_IL(7) | MCF_INTC_ICR_IP(3))
#define IPL_GPT_PAI     (MCF_INTC_ICR_IL(7) | MCF_INTC_ICR_IP(4))
#define IPL_CWD         (MCF_INTC_ICR_IL(7) | MCF_INTC_ICR_IP(7))

/*
 * Interrupt handlers
 */
extern IRQ_HANDLER sig_CWD;
extern IRQ_HANDLER sig_I2C0;
extern IRQ_HANDLER sig_I2C1;
extern IRQ_HANDLER sig_PIT0;
extern IRQ_HANDLER sig_PIT1;
extern IRQ_HANDLER sig_UART0;
extern IRQ_HANDLER sig_UART1;
extern IRQ_HANDLER sig_UART2;
extern IRQ_HANDLER sig_GPT_PAOV;
extern IRQ_HANDLER sig_GPT_PAI;
extern IRQ_HANDLER sig_FEC_RB;
extern IRQ_HANDLER sig_FEC_RF;
extern IRQ_HANDLER sig_FEC_TB;
extern IRQ_HANDLER sig_FEC_TF;
extern IRQ_HANDLER sig_QSPI_TF;

/*
 * Common Interrupt control
 */
extern int IrqCtlCommon(IRQ_HANDLER *sig_handler, int cmd, void *param,
        volatile void *reg_imr, uint32_t imr_mask, uint8_t imr_size,
        volatile uint32_t *reg_imr_ic, uint32_t imr_mask_ic,
        volatile uint8_t *reg_icr, uint8_t ipl);
