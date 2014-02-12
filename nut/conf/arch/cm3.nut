--
-- Copyright (C) 2004-2007 by egnite Software GmbH. All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
--
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
-- 3. Neither the name of the copyright holders nor the names of
--    contributors may be used to endorse or promote products derived
--    from this software without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-- ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-- LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
-- FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
-- COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
-- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-- BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
-- OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
-- AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
-- OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
-- THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.
--
-- For additional information see http://www.ethernut.de/
--

-- ARM CortexM3 Architecture
--
-- $Log$
-- Revision 1.0  2010/09/10 13:24:58  ulrichprinz
-- STM32F Initial Port
--

nutarch_cm3 =
{
    --
    -- MCU Family CortexM3
    --
    {
        name = "nutarch_cm3_family",
        brief = "MCU Family",
        options =
        {
            {
                macro = "MCU_CM3",
                brief = "CortexM3 Family",
                type = "integer",
                default = 1,
                requires = { "HW_MCU_CM3" },
                file = "include/cfg/arch.h"
            }
        }
    },

    --
    -- CortexM3 Core Functions
    --
    {
        name = "nutarch_cm3_init",
        brief = "Initialization and interrupt registration(CortexM3)",
        description = "Contains spurious interrupt handler.",
        requires = { "HW_MCU_CM3" },
	provides = { "DEV_IRQ_CM3" },
        sources = { "cm3/cmsis/core_cm3.c",
                    "cm3/cmsis/cortex_init.c",
                  },
        options =
        {
            {
                macro = "NUT_BOOT_FUNCTION",
                brief = "Boot Function",
                description = "This function is given for the reset entry vecor.\n"..
                              "Default is NutInit\n"..
                              "The function must be declarated as int function(void).",
                flavor = "booldata",
                file = "include/cfg/arch.h"
            },
            {
                macro = "MSP_STACK_SIZE",
                brief = "Main Stack Size",
                description = "Number of bytes reserved for interrupt stack\n"..
                              "Default is 128 words.\n\n"..
                              "This stack is used in flat mode, i.e. the thread mode the CPU uses with Nut/OS.\n"..
                              "The stack is needed for any operation and function call while Nut/OS startup.\n",
                flavor = "booldata",
                file = "include/cfg/memory.h"
            },
            {
                macro = "PSP_STACK_SIZE",
                brief = "Process Stack Size",
                description = "Number of bytes reserved for process stack\n"..
                              "Default is 32 words.\n\n"..
                              "The process stack is actually not used in Nut/OS but may be used for faster task switching "..
                              "in a later release. So beware of modifications.",
                flavor = "booldata",
                file = "include/cfg/memory.h"
            }
        }
    },

    --
    -- CortexM3 SysTick Timer
    --
    {
        name = "nutarch_cm3_ostimer",
        brief = "System Timer (CortexM3)",
        requires = { "LICENSE_MCD_ST_LIBERTY", "LICENSE_ST_GUIDANCE_ONLY", "HW_MCU_CM3" },
        provides = { "NUT_OSTIMER_DEV" },
        sources = { "cm3/cmsis/ostimer_cortex.c" },
    },

    --
    -- CortexM3 Context Switching
    --
    {
        name = "nutarch_cm3_context",
        brief = "Context Switching (CortexM3)",
        provides = { "NUT_CONTEXT_SWITCH" },
        requires = { "HW_MCU_CM3", "TOOL_GCC" },
        sources = { "cm3/os/context.c" },
    },

    --
    -- CortexM3 Reset Controller
    --
    {
        name = "nutarch_cm3_reset",
        brief = "Cortex Reset Controller support",
        requires = { "HW_MCU_CM3" },
        sources =
        {
            "cm3/cmsis/cortex_reset.c",
        },
    },

    --
    -- Board Initialization
    --
    {
        name = "nutarch_cm3_bs",
        brief = "Board Support",
        sources =
            function()
                return { "cm3/board/"..string.lower(c_macro_edit("PLATFORM"))..".c" };
            end,
        requires = { "HW_BOARD_SUPPORT" },
    },

    --
    -- Cortex Based Cpu Directory
    --
    {
        name = "nutarch_cm3_stm32_family",
        brief = "STM32 Family",
        requires = { "HW_MCU_STM32" },
        description = "ST Microelectronics STM32 Series",
        script = "arch/cm3/stm32fam.nut"
    },
    {
        name = "nutarch_cm3_sam3",
        brief = "SAM3",
        requires = { "HW_MCU_SAM3" },
        description = "ATMEL AT91SAM3 Series",
        script = "arch/cm3/sam3u.nut"
    },
    {
        name = "nutarch_cm3_lm3",
        brief = "LM3",
        requires = { "HW_MCU_LM3" },
        description = "Luminary Micro LM3 Series",
        script = "arch/cm3/lm3.nut"
    },
    {
        name = "nutarch_cm3_lpc17xx_family",
        brief = "LPC17xx",
        requires = { "HW_MCU_LPC17xx" },
        description = "NXP LPC17xx Series",
        script = "arch/cm3/lpc17xxfam.nut"
    }
}

