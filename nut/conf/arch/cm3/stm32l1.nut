--
--  * Copyright (C) 2011 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
-- STMicroelectronics STM32L Family Devices
--
--
stm32_memory_l1 = { "32", "64", "128" }
stm32l1_device_class = { "STML1XX_MD" }

-- *****************************************************************************
-- STM32L1 Family
-- *****************************************************************************
--

nutarch_cm3_stm32l1 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32L1_family",
        brief = "MCU L1 Family",
        requires = { "HW_MCU_STM32L1XX" },
        options =
        {
            {
                macro = "MCU_STM32L1",
                brief = "STM32L1",
                type = "integer",
                default = 1,
                requires = { "HW_MCU_CM3" },
                provides =
                {
                    "HW_PLL_STM32L1",
                    "HW_RCC_STM32",
                    "HW_GPIO_STM32V2",
                    "HW_CRC32_STM32",
                    "DEV_IRQ_STM32",
                    "HW_DMA1_STM32F1",
                    "HW_EXTI04_STM32",
                    "HW_EXTI95_STM32",
                    "HW_EXTI1510_STM32",
                    "HW_I2C1_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_UART1_STM32",
                    "HW_UART2_STM32",
                    "HW_UART3_STM32",
                },
                file = "include/cfg/arch.h"
            }
        }
    },
    --
    -- STM32L1 MCU Classes
    --
    {
        name = "nutarch_cm3_stm32l1_class",
        brief = "STM32L1 Device Classes",
        requires = { "HW_MCU_STM32" },
        options =
        {
            {
                macro = "STM32L1XX_MD",
                brief = "STM32L1 MD Series",
                description = "STM32L1 Medium Density devices.",
                flavor = "booldata",
                exclusivity = stm32l1_device_class,
                makedefs = { "HWDEF+=-DSTM32L1XX_MD" },
                provides = { "STM32L1XX_MD" },
                file = "include/cfg/arch.h"
            },
        }
    },
    {
        name = "nutarch_cm3_stm32l1_memory",
        brief = "STM32L1 Device Memory",
        requires = { "HW_MCU_STM32" },
        options =
        {
            {
                macro = "MCU_STM32L15X",
                brief = "STM32L1xx memory",
                description = "Select your devices memory by the marked alphanumeric code on the chip:\n"..
                              "STM32L1xx>Y<zz where Y is one of the list below.\n\n"..
                              "6 =   32 kbytes Flash\n"..
                              "8 =   64 kbytes Flash\n"..
                              "B =  128 kbytes Flash\n",

                requires = { "HW_MCU_STM32L1XX" },
                type = "enumerated",
--                choices = stm32_memory_l1xx,
                file = "include/cfg/arch.h"
            },
        }
    },
    --
    -- STM32L1 PLL Configuration
    --
    {
        name = "nutarch_cm3_stm32L1_pll",
        brief = "STM32L1 PLL Setup",
        sources = { "cm3/dev/stm/system_stm32l1xx.c",
                    "cm3/dev/stm/stm32l1_clk.c"
                  },
        requires = { "LICENSE_MCD_ST_LIBERTY", "LICENSE_ST_GUIDANCE_ONLY", "HW_MCU_STM32", "TOOL_CC_CM3", "TOOL_GCC" },
        options =
        {
            {
                macro = "SYSCLK_SOURCE",
                brief = "SYSCLK Source",
                description = "Select where SYSCLK should get its clock from.\n\n"..
                              "SYSCLK_HSI is the internal 8MHz clock.\n"..
                              "SYSCLK_PLL is the internal PLL output. Select the source for the PLL in the next option.\n"..
                              "SYSCLK_HSE is the external oscillator or crystal input.\n",
                requires = { "HW_PLL_STM32L1" },
                type = "enumerated",
                choices = { "SYSCLK_HSI", "SYSCLK_PLL", "SYSCLK_HSE" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLLCLK_SOURCE",
                brief = "PLL Clock Source",
                description = "Select where the PLL should get its clock from.\n\n"..
                              "SYSCLK_HSI is the internal 8MHz clock. PLL is fed with SYSCLK_HSI/2.\n"..
                              "SYSCLK_HSE is the external oscillator or crystal input.\n",
                requires = { "HW_PLL_STM32L1" },
                type = "enumerated",
                choices = { "PLLCLK_HSI", "PLLCLK_HSE" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_CLOCK_DIV",
                brief = "PLL Clock Prescaler",
                description = "Select this to force the HSE clock beeing divided by 2.",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_CLOCK_MUL",
                brief = "PLL Multiplier",
                description = "Set this value to override automatic setup of the PLL.\n"..
                              "Respect that the PLL output has to be at maximum 72MHz.\n"..
                              "If USB is used the ouput must provide a frequency of 72MHz or 48MHz.\n"..
                              "This clock is used for audio I2S interface directly.\n",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "HSE_VALUE",
                brief = "External Oszillator Frequency",
                description = "Value of the external oscillator in Herz.\n"..
                              "Typical Values are:\n"..
                              "STM32F Conectivity Line Devices: 25MHz\n"..
                              "STM32F Value Line devices asr limited to 24MHz and do not have a PLL."..
                              "Other devices: 8MHz.",
                requires = { "HW_PLL_STM32L1" },
                flavor = "booldata",
                type = "long",
                default = "8000000",
                file = "include/cfg/clock.h"
            },
            {
                macro = "SYSCLK_FREQ",
                brief = "CM3 System Clock",
                description = "System clock (SYSCLK) target frequency after PLL setup.\n\n"..
                              "24MHz USB not supported\n"..
                              "36MHz USB not supported\n"..
                              "48MHz USB supported\n"..
                              "56MHz USB not supported\n"..
                              "72MHz USB supported\n\n",
                requires = { "HW_PLL_STM32L1" },
                type = "enumerated",
                choices = stm32_syclk_frequencies,
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_HCLK",
                brief = "AHB Prescaler",
                description = "This is the divider for the AHB bus. It is supplied by the SYSCLK and must not exceed 72MHz.\n"..
                              "The AHB clocks the core, memory, DMA and all other busses.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n"..
                              "Respect that usage of ethernet or USB requires at least 25MHz on this bus.\n",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_APB1",
                brief = "APB1 Prescaler",
                description = "This is the divider for the slow peripheral bus (APB1). It is upplied by the AHB clock and "..
                              "it must not exceed 36MHz.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n"..
                              "specific BoardInit() function.\n\n",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_APB2",
                brief = "APB2 Prescaler",
                description = "This is the divider for the fast peripheral bus (APB2). It is upplied by the AHB clock and "..
                              "it must not exceed 36MHz.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n"..
                              "specific BoardInit() function.\n\n",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
        }
    },

    -- ***********************************
    --
    -- STM32L Device Drivers
    --
    -- ***********************************

    --
    --
    -- STM32L RCC Interface
    --
    {
        name = "nutarch_cm3_stm32l1_rcc",
        brief = "STM32L RCC",
        description = "STM32L Clock Controller.",
        requires = { "LICENSE_MCD_ST_LIBERTY", "LICENSE_ST_GUIDANCE_ONLY", "HW_RCC_STM32" },
        sources = { "cm3/dev/stm/stm32l1xx_rcc.c" }
    },
}
