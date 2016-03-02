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

//------------------------------------------------------------------------------
// Peripheral vectors
//------------------------------------------------------------------------------
.section .vectors,"ax",%progbits

vector40:   .long   IH_DEFAULT                  // 0x100 IRQ_pin
vector41:   .long   IH_DEFAULT                  // 0x104 Low_voltage_detect
vector42:   .long   IH_TPM1_CH0                 // 0x108 TPM1_ch0
vector43:   .long   IH_TPM1_CH1                 // 0x10C TPM1_ch1
vector44:   .long   IH_TPM1_CH2                 // 0x110 TPM1_ch2
vector45:   .long   IH_TPM1_OVFL                // 0x114 TPM1_ovfl
vector46:   .long   IH_TPM2_CH0                 // 0x118 TPM2_ch0
vector47:   .long   IH_TPM2_CH1                 // 0x11C TPM2_ch1
vector48:   .long   IH_TPM2_CH2                 // 0x120 TPM2_ch2
vector49:   .long   IH_TPM2_OVFL                // 0x124 TPM2_ovfl
vector4A:   .long   IH_SPI2                 	// 0x128 SPI2
vector4B:   .long   IH_SPI1		               	// 0x12C SPI1
vector4C:   .long   IH_DEFAULT                  // 0x130 SCI1_err
vector4D:   .long   IH_SCI1_RX                  // 0x134 SCI1_rx
vector4E:   .long   IH_SCI1_TX                  // 0x138 SCI1_tx
vector4F:   .long   IH_IIC_X                  	// 0x13C IICx
vector50:   .long   IH_DEFAULT                  // 0x140 KBIx
vector51:   .long   IH_ADC                 		// 0x144 ADC
vector52:   .long   IH_DEFAULT                  // 0x148 ACMPx
vector53:   .long   IH_DEFAULT                  // 0x14C SCI2_err
vector54:   .long   IH_SCI2_RX                  // 0x150 SCI2_rx
vector55:   .long   IH_SCI2_TX                  // 0x154 SCI2_tx
vector56:   .long   IH_DEFAULT                 	// 0x158 RTC
vector57:   .long   IH_DEFAULT                  // 0x15C TPM3_ch0 IH_TPM3_CH0
vector58:   .long   IH_DEFAULT                  // 0x160 TPM3_ch1 IH_TPM3_CH1
vector59:   .long   IH_DEFAULT                  // 0x164 TPM3_ch2 IH_TPM3_CH2
vector5A:   .long   IH_DEFAULT                  // 0x168 TPM3_ch3 IH_TPM3_CH3
vector5B:   .long   IH_DEFAULT                  // 0x16C TPM3_ch4 IH_TPM3_CH4
vector5C:   .long   IH_DEFAULT                  // 0x170 TPM3_ch5 IH_TPM3_CH5
vector5D:   .long   IH_DEFAULT                  // 0x174 TPM3_ovfl IH_TPM3_OVFL
//vector5E:   .long   IH_DEFAULT                // 0x178 Reserved; unused for V1
//vector5F:   .long   IH_DEFAULT                // 0x17C Reserved; unused for V1
vector60:   .long   IH_DEFAULT                  // 0x180 Level 7
vector61:   .long   IH_DEFAULT                  // 0x184 Level 6
vector62:   .long   IH_DEFAULT                  // 0x188 Level 5
vector63:   .long   IH_DEFAULT                  // 0x18C Level 4
vector64:   .long   IH_DEFAULT                  // 0x190 Level 3
vector65:   .long   IH_DEFAULT                  // 0x194 Level 2
vector66:   .long   IH_DEFAULT                  // 0x198 Level 1
//vector67-FF:   .long   IH_DEFAULT             // 0x19C-0x3FC Reserved; unused for V1

//------------------------------------------------------------------------------
// Section 0 - Basic CPU initialization (family dependant code)
//           - The code starts here after reset
//           - After this section, C stack memory must be initialized
//------------------------------------------------------------------------------
	.section .init0,"ax",%progbits

//------------------------------------------------------------------------------
// Section 10 - CPU and Peripherals Initialization (family dependant code)
//------------------------------------------------------------------------------
    .section .init10,"ax",%progbits
    jsr InitSystem
    jsr InitClock
