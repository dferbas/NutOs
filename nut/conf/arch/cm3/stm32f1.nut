--
--  * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
-- STMicroelectronics STM32F1 Family Devices
--
-- $Id: stm32f1.nut 4608 2012-09-14 13:14:15Z haraldkipp $
--
--

stm32_memory_f10x = { "16", "32", "64", "128", "256", "384", "512", "768", "1024" }
stm32_device_class = { "STM32F10X_LD", "STM32F10X_LD_VL",
                       "STM32F10X_MD", "STM32F10X_MD_VL",
                       "STM32F10X_HD", "STM32F10X_CL", "STM32F10X_XL" }

stm32_syclk_frequencies =
    { " ", "24000000", "36000000", "48000000", "56000000", "72000000" }

--
-- ********************************************************************************
-- STM32F1 Family
-- ********************************************************************************
--

nutarch_cm3_stm32f1 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32f1_family",
        brief = "MCU F1 Family",
        options =
        {
            {
                macro = "MCU_STM32F1",
                brief = "STM32F1",
                type = "integer",
                default = 1,
                requires = { "HW_MCU_CM3" },
                provides = {
                        "HW_PLL_STM32",
                        "HW_RCC_STM32",
                        "HW_RTC_STM32F1",
                         "HW_FLASH_STM32F1",
                        "HW_GPIO_STM32V1",
                        "HW_CRC32_STM32",
                        "DEV_IRQ_STM32",
                        "HW_ADC12_STM32F1",
                        "HW_EXTI04_STM32",
                        "HW_WWDG_STM32",
                        "HW_DMA1_STM32F1",
                        "HW_EXTI95_STM32",
                        "HW_TIM1_STM32",
                        "HW_TIM2_STM32",
                        "HW_TIM3_STM32",
                        "HW_I2C1_STM32",
                        "HW_SPI1_STM32",
                        "HW_UART1_STM32",
                        "HW_UART2_STM32",
                        "HW_EXTI1510_STM32",
                },
                file = "include/cfg/arch.h"
            }
        }
    },

    --
    -- STM32F1 MCU Classes
    --
    {
        name = "nutarch_cm3_stm32f_class",
        brief = "STM32F1 Device Classes",
        requires = { "HW_MCU_STM32" },
        options =
        {
            {
                macro = "STM32F10X_LD",
                brief = "STM32F LD Series",
                description = "STM32F Low Density devices.",
                flavor = "booldata",
                exclusivity = stm32_device_class,
                makedefs = { "HWDEF+=-DSTM32F10X_LD" },
                provides = {
                        "STM32F10X_LD",
                        "HW_USB_STM32",
                        "HW_CAN1_STM32",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F10X_LD_VL",
                brief = "STM32F LD-VL Series",
                description = "STM32F Low Density devices.",
                flavor = "booldata",
                exclusivity = stm32_device_class,
                makedefs = { "HWDEF+=-DSTM32F10X_LD_VL" },
                provides = {
                        "STM32F10X_LD_VL",
                        "HW_CEC_STM32",
                        "HW_TIM6_STM32",
                        "HW_TIM7_STM32",
                     },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F10X_MD",
                brief = "STM32F MD Series",
                description = "STM32F Medium Density devices.",
                flavor = "booldata",
                exclusivity = stm32_device_class,
                makedefs = { "HWDEF+=-DSTM32F10X_MD" },
                provides = {
                        "STM32F10X_MD",
                        "HW_USB_STM32",
                        "HW_CAN1_STM32",
                        "HW_TIM4_STM32",
                        "HW_I2C2_STM32",
                        "HW_SPI2_STM32",
                        "HW_UART3_STM32",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F10X_MD_VL",
                brief = "STM32F MD-VL Series",
                description = "STM32F Medium Density devices.",
                flavor = "booldata",
                exclusivity = stm32_device_class,
                makedefs = { "HWDEF+=-DSTM32F10X_MD_VL" },
                provides = {
                        "STM32F10X_MD_VL",
                        "HW_TIM4_STM32",
                        "HW_I2C2_STM32",
                        "HW_SPI2_STM32",
                        "HW_UART3_STM32",
                        "HW_CEC_STM32",
                        "HW_TIM6_STM32",
                        "HW_TIM7_STM32",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F10X_HD",
                brief = "STM32F HD Series",
                description = "STM32F HD-Series devices.",
                flavor = "booldata",
                exclusivity = stm32_device_class,
                makedefs = { "HWDEF+=-DSTM32F10X_HD" },
                provides = {
                        "STM32F10X_HD",
                        "HW_USB_STM32",
                        "HW_CAN1_STM32",
                        "HW_TIM4_STM32",
                        "HW_I2C2_STM32",
                        "HW_SPI2_STM32",
                        "HW_UART3_STM32",
                        "HW_TIM8_STM32",
                        "HW_ADC3_STM32",
                        "HW_FSMC_STM32",
                        "HW_SDIO_STM32",
                        "HW_TIM5_STM32",
                        "HW_SPI3_STM32",
                        "HW_UART4_STM32",
                        "HW_UART5_STM32",
                        "HW_TIM6_STM32",
                        "HW_TIM7_STM32",
                        "HW_DMA2_STM32F1",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F10X_XL",
                brief = "STM32F XL Devices",
                description = "STM32F XL-Series devices.",
                flavor = "booldata",
                exclusivity = stm32_device_class,
                makedefs = { "HWDEF+=-DSTM32F10X_XL" },
                provides = {
                        "STM32F10X_XL",
                        "HW_USB_STM32",
                        "HW_CAN1_STM32",
                        "HW_TIM4_STM32",
                        "HW_I2C2_STM32",
                        "HW_SPI2_STM32",
                        "HW_UART3_STM32",
                        "HW_TIM8_STM32",
                        "HW_ADC3_STM32",
                        "HW_FSMC_STM32",
                        "HW_SDIO_STM32",
                        "HW_TIM5_STM32",
                        "HW_SPI3_STM32",
                        "HW_UART4_STM32",
                        "HW_UART5_STM32",
                        "HW_TIM6_STM32",
                        "HW_TIM7_STM32",
                        "HW_DMA2_STM32F1",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F10X_CL",
                brief = "STM32F CL Series",
                description = "STM32F Connectivity line devices.",
                flavor = "booldata",
                exclusivity = stm32_device_class,
                makedefs = { "HWDEF+=-DSTM32F10X_CL" },
                provides = {
                        "STM32F10X_CL",
                        "HW_USB_STM32",
                "HW_OTG_STM32",
                        "HW_CAN1_STM32",
                        "HW_TIM4_STM32",
                        "HW_I2C2_STM32",
                        "HW_SPI2_STM32",
                        "HW_UART3_STM32",
                        "HW_OTG1_STM32",
                        "HW_TIM5_STM32",
                        "HW_SPI3_STM32",
                        "HW_UART4_STM32",
                        "HW_UART5_STM32",
                        "HW_TIM6_STM32",
                        "HW_TIM7_STM32",
                        "HW_EMAC_STM32",
                        "HW_CAN2_STM32",
                        "HW_DMA2_STM32F1",
                    },
                file = "include/cfg/arch.h"
            },
        }
    },

    --
    -- STM32F Memory Configuration
    --
    {
        name = "nutarch_cm3_stm32f_memory",
        brief = "STM32F Device Memory",
        requires = { "HW_MCU_STM32" },
        options =
        {
            {
                macro = "MCU_STM32F10X",
                brief = "STM32F10x memory",
                description = "Select your devices memory by the marked alphanumeric code on the chip:\n"..
                              "STM32F10x>Y<zz where Y is one of the list below.\n\n"..
                              "4 =   16 kbytes Flash\n"..
                              "6 =   32 kbytes Flash\n"..
                              "7 =   48 kbytes Flash\n"..
                              "8 =   60/64 kbytes Flash\n"..
                              "9 =   72 kbytes Flash\n"..
                              "A =   96 kbytes Flash\n"..
                              "B =  128 kbytes Flash\n"..
                              "C =  256 kbytes Flash\n"..
                              "D =  384 kbytes Flash\n"..
                              "E =  512 kbytes Flash\n"..
                              "Z = 1024 kbytes Flash\n",

                requires = { "HW_MCU_STM32F10X" },
                type = "enumerated",
                choices = stm32_memory_f10x,
                file = "include/cfg/arch.h"
            },
        }
    },

    --
    -- STM32F Flexible Static Memory Controller
    --
    {
        name = "nutarch_cm3_stm32f_fsmc",
        brief = "STM32F Static Memory Controller",
        description = "Routines for configuration of the FSMC in STM32F controllers.\n"..
                      "The FSMC access access to SRAM, NAND- and NOR-Flash and PC-Memory Cards.",
        requires = { "LICENSE_ST_GUIDANCE_ONLY", "HW_FSMC_STM32" },
--        sources = { "cm3/dev/stm/stm32_fsmc.c" },
    },

    --
    -- STM32F Flash Memory Controller
    --
    {
        name = "nutarch_cm3_stm32f_flash",
        brief = "STM32F Flash Memory Controller",
        description = "Routines for setup and programming STM32F series internal FLASH.\n",
        requires = { "LICENSE_ST_GUIDANCE_ONLY", "HW_FLASHF1_STM32" },
        sources = { "cm3/dev/stm/stm32f1_flash.c" },
    },

    --
    -- STM32F PLL Configuration
    --
    {
        name = "nutarch_cm3_stm32f_pll",
        brief = "STM32F PLL Setup",
        sources = { "cm3/dev/stm/system_stm32f10x.c",
                    "cm3/dev/stm/stm32_clk.c"
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
                requires = { "HW_PLL_STM32" },
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
                requires = { "HW_PLL_STM32" },
                type = "enumerated",
                choices = { "PLLCLK_HSI", "PLLCLK_HSE" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_CLOCK_DIV",
                brief = "PLL Clock Prescaler",
                description = "Select this to force the HSE clock beeing divided by 2.",
                requires = { "HW_PLL_STM32", "DISABLED" },
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
                requires = { "HW_PLL_STM32", "DISABLED" },
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
                requires = { "HW_PLL_STM32" },
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
                requires = { "HW_PLL_STM32" },
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
                requires = { "HW_PLL_STM32", "DISABLED" },
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
                requires = { "HW_PLL_STM32", "DISABLED" },
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
                requires = { "HW_PLL_STM32", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
        },
    },

    -- ***********************************
    --
    -- STM32F Device Drivers
    --
    -- ***********************************

    --
    --
    -- STM32F RCC Interface
    --
    {
        name = "nutarch_cm3_stm32f_rcc",
        brief = "STM32F RCC",
        description = "STM32F Clock Controller.",
        requires = { "LICENSE_ST_GUIDANCE_ONLY", "HW_RCC_STM32" },
        sources = { "cm3/dev/stm/stm32f10x_rcc.c" }
    },

    --
    -- STM32F1 GPIO Interface
    --
    {
        name = "nutarch_cm3_stm32f1_gpio",
        brief = "STM32F1 GPIO",
        description = "Generic port I/O API.",
        requires = { "LICENSE_ST_GUIDANCE_ONLY", "HW_MCU_STM32", "HW_GPIO_STM32V1" },
        sources = { "cm3/dev/stm/stm32f1_gpio.c"}
    },
}

