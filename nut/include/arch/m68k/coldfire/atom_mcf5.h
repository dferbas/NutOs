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

#ifndef _SYS_ATOM_H_
#error "Do not include this file directly. Use sys/atom.h instead!"
#endif

/*
 * This is because of Nut/OS usage - no declaration for a temporary variable,
 * NutJumpOutCritical() is used to exit critical section from inside a block.
 * Result of this is allocated space for more _savpri variables
 * if NutEnterCritical() / NutExitCritical() function pair is used more times in a function.
 */

/*	this code can not use, due to small amount of registers
 *  Error: operands mismatch -- statement `move.w %sr,%a1' ignored
 */

/*
 * savpri - variable used to save SR register
 * 			following also works
	uint16_t register _savpri asm ("d7"); \
 * "d" constraint forces dn register to be used, because move.w sr,... can be only to/from data register
 * {it is sufficient to have this constraint only in NutJumpOutCritical, in NutEnterCritical can be "=r")
 * compilator knows that register d0 is used, because "d0" is listed in clobbered regs

 */
#define NutUseCritical() \
	uint16_t register _savpri;

#define NutEnterCritical() \
	asm volatile ( \
		"move.w	%%sr, %%d0\n" \
		"move.w	%%d0,%[savpri]\n" \
		"ori.l #0x700, %%d0\n" \
		"move.w %%d0, %%sr\n" : [savpri] "=d" (_savpri) :: "d0");

#define NutEnterCriticalLevel(level) \
	asm volatile ( \
		"move.w	%%sr, %%d0\n" \
		"move.w	%%d0,%[savpri]\n" \
		"andi.l #~0x700, %d0\n"\
		"ori.l %[lvl], %%d0\n"\
		"move.w %%d0, %%sr\n" : [savpri] "=d" (_savpri) : [lvl] "i" (level * 0x100) : "d0");

#define NutJumpOutCritical() \
	asm volatile ( \
		"move.w %[savpri], %%sr\n" :: [savpri] "d" (_savpri));

#define NutExitCritical() NutJumpOutCritical()

