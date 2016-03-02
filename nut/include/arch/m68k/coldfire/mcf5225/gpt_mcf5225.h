/*
 * Copyright 2014-2016 by Embedded Technologies s.r.o. All rights reserved.
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

#ifndef GPT_MCF5225_H_
#define GPT_MCF5225_H_

#include <stdint.h>
#include <sys/types.h>

void Mcf5225GptInitPA(HANDLE *pae_handler);
void Mcf5225GptStartPA(void);
void Mcf5225GptStopPA(void);
void Mcf5225GptClearPACounter(void);
uint16_t Mcf5225GptGetPACounter(void);

void Mcf5225GptCounterInit(int channel, HANDLE *counter_handler);
void Mcf5225GptCountersEnable(void);
void Mcf5225GptCountersDisable(void);
void Mcf5225GptCountersStart(int channel);
void Mcf5225GptCountersStop(int channel);
void Mcf5225GptCounterClear(int channel);
int Mcf5225GptCounterGet(int channel);
void Mcf5225GptCounterStart(int channel);
void Mcf5225GptCounterStop(int channel);

#endif  /* GPT_MCF5225_H_ */
