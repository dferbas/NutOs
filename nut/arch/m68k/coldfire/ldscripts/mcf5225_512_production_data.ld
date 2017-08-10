/*
 * Copyright 2013-2017 by Solar Monitor s.r.o
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

        /* Set MAC info */
        . = 0x380;
        LONG (0xbeaf0001)
        LONG (0x00010080)
        LONG (0x70B59911)	/* MAC */
        LONG (0x00020000)	/* MAC */
        LONG (0x31303034)	/* SN */
        LONG (0x35303030)	/* SN */
        LONG (0x30320000)	/* SN */
        LONG (0x00000000)	/* za poslednim znakem SN musi byt ukoncovaci 0x00, vypisuje se jako retezec */

        /* Emit a CFM Key */
        . = 0x400;          /* JS TODO .. musi byt vzdy ve FLASH .. stejne jako vektory, kdyby nekdo dal .text do RAM */ 
        LONG (0x98563182) /* Backdoor Key */
        LONG (0x87519645) /* Backdoor Key */
        LONG (0xffffffff) /* Reserved */
        LONG (0x00FF00F2) /* Flash protection, only last sector unprotected 81, back door key disabled, security enabled F2 */
        /* LONG (0x008100FF) */ /* For TEST HW - Flash protection, only last sector unprotected 81, back door key disabled, security disabled FF */
        LONG (0x701011C0) /* Data Access */
        LONG (0x82A070C2) /* Security Word */

