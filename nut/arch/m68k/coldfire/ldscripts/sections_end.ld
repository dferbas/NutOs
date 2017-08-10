/*
 * Copyright 2012-2017 by Embedded Technologies s.r.o. All rights reserved.
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

     .rodata : ALIGN (4)
    {
        *(.rodata)
        *(.rodata.*)
        *(.gnu.linkonce.r.*)
    } > REGION_RODATA AT > REGION_LOAD

    .dataint : ALIGN(4)
    {
        PROVIDE (__dataint_start = .) ;
        *(.dataint)
        *(.dataint.*)
        . = ALIGN(4);
        PROVIDE (__dataint_end = .) ;
    } > REGION_DATA_INT AT > REGION_LOAD
    PROVIDE (__dataint_size = SIZEOF(.dataint));
    PROVIDE (__dataint_load_start = LOADADDR(.dataint));
    
    .data : ALIGN(4)
    {
        PROVIDE (__data_start = .) ;
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        . = ALIGN(4);
        PROVIDE (__data_end = .) ;
    }  > REGION_DATA AT > REGION_LOAD
    PROVIDE (__data_size = SIZEOF(.data));
    PROVIDE (__data_load_start = LOADADDR(.data));

    .bssint : ALIGN (4)
    {
        PROVIDE (__bssint_start = .) ;
        *(.bssint)
        *(.bssint.*)
        PROVIDE (__bssint_end = .) ;
    } > REGION_BSS_INT AT > REGION_LOAD
    PROVIDE (__bssint_size = SIZEOF(.bssint));

    .bss : ALIGN (4)
    {
        PROVIDE (__bss_start = .) ;
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        PROVIDE (__bss_end = .) ;
    } > REGION_BSS AT > REGION_LOAD
    PROVIDE (__bss_size = SIZEOF(.bss));
