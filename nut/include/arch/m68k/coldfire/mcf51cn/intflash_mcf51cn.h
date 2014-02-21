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

#ifndef INTFLASH_MCF51CN_H_
#define INTFLASH_MCF51CN_H_

#include <stdint.h>

/* address in memory map to write in FLASH */
#define FLASH_MIN_ADDRESS	 				0x00000000

/* End address for region of flash */
#define FLASH_MAX_ADDRESS					0x0001FFFF

/* Flash erase page size 1024 Bytes */
#define FLASH_PAGE_ERASE_SIZE		1024 // size of sector to erase

/* Flash protected page size 4096 Bytes */
#define FLASH_PAGE_PROTECT_SIZE		0x1000 // size of protected sector

int Mcf51cnIntFlashInit(void);
uint8_t Mcf51cnIntFlashProtectRegister(void);
int Mcf51cnIntFlashRead(uint32_t dst, uint32_t *data, uint32_t size);
int Mcf51cnIntFlashWrite(uint32_t dst, uint32_t *data, uint32_t size);
int Mcf51cnIntFlashSectorErase(uint32_t addr);
int Mcf51cnIntFlashMassErase(void);

#endif /* INTFLASH_MCF51CN_H_ */
