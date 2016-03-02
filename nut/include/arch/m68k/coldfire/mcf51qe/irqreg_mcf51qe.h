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

#ifndef _DEV_IRQREG_H_
#error "Do not include this file directly. Use dev/irqreg.h instead!"
#endif

/*
 * Interrupt handlers
 */
extern IRQ_HANDLER sig_IIC1;
extern IRQ_HANDLER sig_IIC2;
//extern IRQ_HANDLER sig_MTIM1;
//extern IRQ_HANDLER sig_MTIM2;
extern IRQ_HANDLER sig_TPM1_OVFL;
extern IRQ_HANDLER sig_TPM1_CH0;
extern IRQ_HANDLER sig_TPM1_CH1;
extern IRQ_HANDLER sig_TPM1_CH2;
extern IRQ_HANDLER sig_TPM2_OVFL;
extern IRQ_HANDLER sig_TPM2_CH0;
extern IRQ_HANDLER sig_TPM2_CH1;
extern IRQ_HANDLER sig_TPM2_CH2;
extern IRQ_HANDLER sig_SCI1_RX;
extern IRQ_HANDLER sig_SCI1_TX;
extern IRQ_HANDLER sig_SCI2_RX;
extern IRQ_HANDLER sig_SCI2_TX;
//extern IRQ_HANDLER sig_SCI3_RX;
//extern IRQ_HANDLER sig_SCI3_TX;
extern IRQ_HANDLER sig_ADC;
//extern IRQ_HANDLER sig_RTC;
extern IRQ_HANDLER sig_SPI1;
extern IRQ_HANDLER sig_SPI2;

/*
 * Common Interrupt control
 */
extern int IrqCtlCommon(IRQ_HANDLER *sig_handler, int cmd, void *param, volatile void *reg_imr, uint32_t imr_mask, uint8_t imr_size);

