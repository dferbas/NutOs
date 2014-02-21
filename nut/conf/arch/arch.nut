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

-- Operating system functions
--
-- $Log$
-- Revision 1.38  2009/02/06 15:51:38  haraldkipp
-- Added NutReset support for SAM9260.
-- Removed stack size defaults.
--
-- Revision 1.37  2008/10/23 08:50:43  haraldkipp
-- Prepared AT91 UART hardware handshake.
--
-- Revision 1.36  2008/10/05 17:01:15  haraldkipp
-- Added Helix MP3 decoder and Hermite re-sampler. Do not miss to read
-- the license texts.
--
-- Revision 1.35  2008/10/03 11:31:27  haraldkipp
-- Added TWI support for the AT91SAM9260.
--
-- Revision 1.34  2008/09/23 07:24:34  haraldkipp
-- Added support for remaining AT91SAM7 familiy members.
-- Added support for AT91 SDRAM and reset controller.
--
-- Revision 1.33  2008/08/11 11:51:19  thiagocorrea
-- Preliminary Atmega2560 compile options, but not yet supported.
-- It builds, but doesn't seam to run properly at this time.
--
-- Revision 1.32  2008/08/06 12:51:08  haraldkipp
-- Added support for Ethernut 5 (AT91SAM9XE reference design).
--
-- Revision 1.31  2008/04/18 13:24:57  haraldkipp
-- Added Szemzo Andras' RS485 patch.
--
-- Revision 1.30  2008/02/15 17:05:53  haraldkipp
-- AT91SAM7SE512 support added. Global MCU list simplifies exclusivity
-- attribute. HW_EBI_AT91 added for MCUs with external bus interface.
--
-- Revision 1.29  2007/10/04 21:06:11  olereinhardt
-- Support for SAM7S256 added
--
-- Revision 1.28  2007/09/06 19:41:30  olereinhardt
-- Added HW_PDC_AT91 to AT91SAM7X256 platform
--
-- Revision 1.27  2007/04/12 09:20:00  haraldkipp
-- ATmega2561 no longer bound to ICCAVR.
--
-- Revision 1.26  2007/02/15 16:24:34  haraldkipp
-- Now the periodic interrupt timer may become the system clock.
--
-- Revision 1.25  2006/10/05 17:16:04  haraldkipp
-- Added exclusivity attribute.
--
-- Revision 1.24  2006/09/29 12:40:36  haraldkipp
-- PDC provision added for AT91 devices.
--
-- Revision 1.23  2006/09/05 12:29:59  haraldkipp
-- SPI and MCI support added for SAM9260.
--
-- Revision 1.22  2006/08/31 19:04:08  haraldkipp
-- Added support for the AT91SAM9260 and Atmel's AT91SAM9260 Evaluation Kit.
--
-- Revision 1.21  2006/07/26 11:19:06  haraldkipp
-- Defining AT91_PLL_MAINCK will automatically determine SAM7X clock by
-- reading PLL settings.
-- Added MMC/SD-Card support for AT91SAM7X Evaluation Kit.
--
-- Revision 1.20  2006/07/05 08:01:56  haraldkipp
-- SAM7 EMAC attribute added.
--
-- Revision 1.19  2006/06/28 17:22:34  haraldkipp
-- Make it compile for AT91SAM7X256.
--
-- Revision 1.18  2006/02/23 15:40:59  haraldkipp
-- GPIO provision added for all MCUs except Gameboy Advance.
--
-- Revision 1.17  2006/02/08 15:20:21  haraldkipp
-- ATmega2561 Support
--
-- Revision 1.16  2006/01/23 19:50:48  haraldkipp
-- Dummy NVMEM configuration for GBA added.
--
-- Revision 1.15  2006/01/23 17:28:30  haraldkipp
-- HW_NVMEM is now provided by specific modules.
--
-- Revision 1.14  2006/01/05 16:45:04  haraldkipp
-- ATmega2561 CPU added, but not yet supported.
--
-- Revision 1.13  2005/10/07 22:09:23  hwmaier
-- Changed HWDEF= to HWDEF+= so more defs can be added to this macros from other LUA scripts.
--
-- Revision 1.12  2005/10/04 05:47:26  hwmaier
-- Added CAN driver for AT90CAN128
--
-- Revision 1.11  2005/07/26 15:41:05  haraldkipp
-- All target dependent code is has been moved to a new library named
-- libnutarch. Each platform got its own script.
--
-- Revision 1.10  2005/02/19 22:48:12  hwmaier
-- no message
--
-- Revision 1.9  2005/01/22 19:20:21  haraldkipp
-- Fixes Ethernut 1.3G memory bug
--
-- Revision 1.8  2004/10/03 18:37:39  haraldkipp
-- GBA support
--
-- Revision 1.7  2004/09/22 08:19:42  haraldkipp
-- No ATmega103 with LAN91C111
--
-- Revision 1.6  2004/09/19 15:17:37  haraldkipp
-- ICCAVR initialization added to build
--
-- Revision 1.5  2004/09/08 10:18:54  haraldkipp
-- C startup on ARM platforms only
--
-- Revision 1.4  2004/09/07 19:07:45  haraldkipp
-- MCU type in NutConf.mk and cstartup added.
--
-- Revision 1.3  2004/08/18 16:05:26  haraldkipp
-- Use consistent directory structure
--
-- Revision 1.2  2004/08/18 13:46:08  haraldkipp
-- Fine with avr-gcc
--
-- Revision 1.1  2004/08/03 15:09:30  haraldkipp
-- Another change of everything
--
-- Revision 1.1  2004/06/07 16:32:45  haraldkipp
-- First release
--
--

nutarch =
{
    {
        name = "nutarch_mcu",
        brief = "Target CPU",
        description = "Select one only.",
        options =
        {
            {
                macro = "MCU_ATMEGA103",
                brief = "Atmel ATmega 103",
                description = "8-bit RISC microcontroller with 128K bytes flash, 4K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, UART, 3 timers, "..
                              "8-channel ADC and SPI.",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_GPIO"
                },
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                makedefs = { "MCU=$(MCU_ATMEGA103)", "HWDEF+=-D__HARVARD_ARCH__" }
            },
            {
                macro = "MCU_ATMEGA128",
                brief = "Atmel ATmega 128",
                description = "8-bit RISC microcontroller with 128K bytes flash, 4K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 2 USARTs, 4 timers, "..
                              "8-channel ADC, SPI and TWI.",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_MCU_ATMEGA128",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_GPIO"
                },
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                makedefs = { "MCU=$(MCU_ATMEGA128)", "HWDEF+=-D__HARVARD_ARCH__" }
            },
            {
                macro = "MCU_AT90CAN128",
                brief = "Atmel AT90CAN128",
                description = "8-bit RISC microcontroller with 128K bytes flash, 4K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 2 USARTs, 4 timers, "..
                              "8-channel ADC, SPI, TWI and CAN controller.\n"..
                              "\n"..
                              "Please configure appropriate stack settings in category "..
                              "Memory Management. "..
                              "Due to a bug for silicon revisions C of the AT90CAN128 MCU, "..
                              "the code stack must reside in internal RAM. Refer to "..
                              "AT90CAN128 Datasheet Rev. 4250F–CAN–04/05 - Errata Rev C ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_CAN_AVR",
                    "HW_GPIO"
                },
                makedefs = { "MCU=at90can128", "HWDEF+=-D__HARVARD_ARCH__" },
                options =
                {
                }
            },
            {
                macro = "MCU_ATMEGA2560",
                brief = "Atmel ATmega 2560",
                description = "8-bit RISC microcontroller with 256K bytes flash, 8K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 4 USARTs, 6 timers, "..
                              "8-channel ADC, SPI and TWI.",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_MCU_ATMEGA2560",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
--                    "HW_AVR_HAVE_UART2",
--                    "HW_AVR_HAVE_UART3",
                    "HW_GPIO"
                },
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                makedefs =
                {
                    "MCU = $(MCU_ATMEGA2560)",
                    "HWDEF += -D__HARVARD_ARCH__",
                    "HWDEF += -DATMega2560",
                    "HWDEF += -D__AVR_3_BYTE_PC__"
                }
            },
            {
                macro = "MCU_ATMEGA2561",
                brief = "Atmel ATmega 2561",
                description = "8-bit RISC microcontroller with 256K bytes flash, 8K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 2 USARTs, 6 timers, "..
                              "8-channel ADC, SPI and TWI.",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_MCU_ATMEGA2561",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_GPIO"
                },
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                makedefs =
                {
                    "MCU = $(MCU_ATMEGA2561)",
                    "HWDEF += -D__HARVARD_ARCH__",
                    "HWDEF += -DATMega2561",
                    "HWDEF += -D__AVR_3_BYTE_PC__"
                }
            },
            {
                macro = "MCU_AT91SAM9260",
                brief = "Atmel AT91SAM9260",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9260",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_MCI_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_EXT_CALYPSO"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM9G45",
                brief = "Atmel AT91SAM9G45",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9G45",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_PIT_AT91"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM9XE512",
                brief = "Atmel AT91SAM9XE512",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n"..
                              "Experimental port.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9XE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_MCI_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_PIT_AT91"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM7X128",
                brief = "Atmel AT91SAM7X128",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 128K bytes flash, "..
                              "32K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7X256",
                brief = "Atmel AT91SAM7X256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                              "64K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_EXT_CALYPSO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7X512",
                brief = "Atmel AT91SAM7X512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                              "128K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S16",
                brief = "Atmel AT91SAM7S16",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 16K bytes flash, "..
                    "4K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S32",
                brief = "Atmel AT91SAM7S32",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 32K bytes flash, "..
                    "8K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S64",
                brief = "Atmel AT91SAM7S64",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 64K bytes flash, "..
                    "16K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S128",
                brief = "Atmel AT91SAM7S128",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 128K bytes flash, "..
                    "32K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S256",
                brief = "Atmel AT91SAM7S256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                    "64K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S512",
                brief = "Atmel AT91SAM7S512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                    "64K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE32",
                brief = "Atmel AT91SAM7SE32",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 32K bytes flash, "..
                    "8K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE256",
                brief = "Atmel AT91SAM7SE256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                    "32K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE512",
                brief = "Atmel AT91SAM7SE512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                    "32K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91R40008",
                brief = "Atmel AT91R40008",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes RAM, "..
                              "64M bytes address space, 2 USARTs and 3 timers. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91R40008",
                    "HW_TIMER_AT91",
                    "HW_UART_AT91",
                    "HW_MCU_SWTWI",
                    "HW_WDOG_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_GBA",
                brief = "Nintendo GBA",
                description = "ARM7TDMI 16/32-bit RISC microcontroller",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_GBA",
                    "HW_TIMER_GBA",
                    "HW_LCD_GBA",
                    "DEV_NVMEM"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            --
            -- STM STM32F10X SERIES CONTROLLER
            --
            {
                macro = "MCU_STM32F100",
                brief = "STM STM32F100",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F100",
                    "MCU_STM32F100"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_STM32F101",
                brief = "STM STM32F101",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F101",
                    "MCU_STM32F101",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_STM32F102",
                brief = "STM STM32F102",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F102",
                    "MCU_STM32F102",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_STM32F103",
                brief = "STM STM32F103",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F103",
                    "MCU_STM32F103",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_STM32F105",
                brief = "STM STM32F105",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F105",
                    "MCU_STM32F105",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" }
            },
            {
                macro = "MCU_STM32F107",
                brief = "STM STM32F107",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F107",
                    "MCU_STM32F107",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" }
            },
            {
                macro = "MCU_STM32L151",
                brief = "STM STM32L151",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L1XX",
                    "HW_MCU_STM32L151",
                    "MCU_STM32L151",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" }
            },
            {
                macro = "MCU_STM32F405",
                brief = "STM STM32F405",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F405",
                    "MCU_STM32F405",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m4", "MARCH=cm4" }
            },
            {
                macro = "MCU_STM32F205",
                brief = "STM STM32F205",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F2XX",
                    "HW_MCU_STM32F205",
                    "MCU_STM32F205",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cmc" }
            },
            {
                macro = "MCU_STM32F207",
                brief = "STM STM32F207",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F2XX",
                    "HW_MCU_STM32F207",
                    "MCU_STM32F207",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cmc" }
            },
            --
            -- TI LM3S SERIES CONTROLLER
            --
            {
                macro = "MCU_LM3S9B96",
                brief = "Luminary LM3S9B96",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LM3 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LM3",
                    "HW_MCU_LM3S9B96",
                    "MCU_LM3S9B96",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            --
            -- NXP LPC17xx CONTROLLER
            --
            {
                macro = "MCU_LPC175x",
                brief = "NXP LCP175x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC175x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC175x",
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_LPC176x",
                brief = "NXP LCP176x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC176x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC176x",
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_LPC177x_8x",
                brief = "NXP LCP177x_8x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC177x_8x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC177x_8x",
                },
                makedefs = { "MCU=cortex-m3", "MARCH=cm3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            --
            -- UNIX EMULATION FOR NUT/OS
            --
            {
                macro = "MCU_LINUX_EMU",
                brief = "Linux Emulator",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_LINUX" },
                provides = { "HW_TARGET", "DEV_NVMEM", "HW_EMU_LINUX" }
            },
            {
                macro = "MCU_H8_3068",
                brief = "Renesas H8/3068",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_H8300" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_H8300",
                    "H83068_TIMER",
                    "H83068_UART"
                }
            },
            {
                macro = "MCU_S3C4510B",
                brief = "Samsung S3C4510B",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with Ethernet MAC,"..
                              "HDLC protocol, 64M bytes address space, I2C, 2 UARTs and "..
                              "2 timers.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_S3C45",
                    "HW_TIMER_S3C45",
                    "HW_UART_S3C45"
                }
            },

            --
            -- AVR32
            --
            {
                macro = "MCU_AVR32UC3A0512",
                brief = "Atmel AT32UC3A0512",
                description = "32-bit AVR UC3 RISC microcontroller with 512K flash, 64K SRAM,\n"..
                              "10/100 Ethernet MAC, Full-Speed + OTG USB,"..
                              "I2C, 4 UARTs and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_MACB_AVR32",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a0512" }
            },
            {
                macro = "MCU_AVR32UC3A0512ES",
                brief = "Atmel AT32UC3A0512-ES",
                description = "*** Engineering Sample, do not use this CPU for new designs ***\n\n"..
                              "32-bit AVR UC3 RISC microcontroller with 512K flash, 64K SRAM,\n"..
                              "10/100 Ethernet MAC, Full-Speed + OTG USB,"..
                              "I2C, 4 UARTs and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_MACB_AVR32",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a0512es" }
            },
            {
                macro = "MCU_AVR32UC3A3256",
                brief = "Atmel AT32UC3A3256",
                description = "32-bit AVR UC3 RISC microcontroller with 256k Flash, 64k SRAM,\n"..
                              "High-Speed + OTG USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a3256" }
            },
            {
                macro = "MCU_AVR32UC3B0256",
                brief = "Atmel AT32UC3B0256",
                description = "32-bit AVR UC3 RISC microcontroller with 256k Flash, 32k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3b0256" }
            },
            {
                macro = "MCU_AVR32UC3B164",
                brief = "Atmel AT32UC3B164",
                description = "32-bit AVR UC3 RISC microcontroller with 64k Flash, 16k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3b164" }
            },
            {
                macro = "MCU_AVR32UC3L064",
                brief = "Atmel AT32UC3L064",
                description = "32-bit AVR UC3 RISC microcontroller with 64k Flash, 16k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                   -- "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                   -- "HW_PLL_AVR32",
                    "HW_DFLL_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3l064" }
            },
            {
                macro = "MCU_MCF51CN128",
                brief = "Freescale MCF51CN128",
                description = "32-bit RISC microcontroller, V1 Coldfire Core, 128K flash, 24K SRAM"..
                              "10/100 Ethernet MAC, MII, 3xSCI(UART), 2xIIC, 2xSPI, ADC, RTC, 2xModulo Timer, 2xTimer/PWM."..
                              "Mini FlexBUS, Keyboard Interrupts, 1xExternal Interrupt, Rapid GPIO",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_M68K" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_M68K",
                    "HW_MCU_COLDFIRE",
                    "HW_MCU_COLDFIRE_V1",
                    "HW_MCU_MCF51CN",
                    "HW_MCU_MCF51CN128",
                },
                makedefs = { "MCU=51cn" }
            },
            {
                macro = "MCU_MCF52259",
                brief = "Freescale MCF52259",
                description = "32-bit RISC microcontroller, V2 Coldfire Core, 10/100 Ethernet MAC"..
                              "512K flash, 64K SRAM, 3xUART, 2xI2C, ADC, QSPI, DMA, FlexCAN,"..
                              "Mini FlexBUS, USB, RTC, Random Number Generator, Cryptographic Accelerator.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_M68K" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_M68K",
                    "HW_MCU_COLDFIRE",
                    "HW_MCU_COLDFIRE_V2",
                    "HW_MCU_MCF5225",
                    "HW_MCU_MCF52259",
                },
                makedefs = { "MCU=52259" }
            },

            --
            -- Imaginary Zero CPU
            --
            {
                macro = "MCU_ZERO",
                brief = "Zero Dummy CPU",
                description = "Imaginary ARM9 CPU, useful as a porting template.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_TIMER_ZERO",
                    "HW_UART_ZERO"
                },
                makedefs = { "MCU=arm9" }
            }
        }
    },

    --
    -- Architecture Dependent Implementations
    --
    {
        name = "nutarch_arm",
        brief = "ARM",
        requires = { "HW_MCU_ARM" },
        script = "arch/arm.nut"
    },
    {
        name = "nutarch_cm3",
        brief = "CM3",
        requires = { "HW_MCU_CM3" },
        script = "arch/cm3.nut"
    },
    {
        name = "nutarch_avr",
        brief = "AVR",
        requires = { "HW_MCU_AVR" },
        script = "arch/avr.nut"
    },
    {
        name = "nutarch_avr32",
        brief = "AVR32",
        requires = { "HW_MCU_AVR32" },
        script = "arch/avr32.nut"
    },
    {
        name = "nutarch_h8300h",
        brief = "H8/300H",
        requires = { "HW_MCU_H8300" },
        script = "arch/h8300h.nut"
    },
    {
        name = "nutarch_m68k",
        brief = "M68K",
        requires = { "HW_MCU_M68K" },
        script = "arch/m68k.nut"
    },
    {
        name = "nutarch_unix",
        brief = "Linux Emulator",
        requires = { "HW_EMU_LINUX" },
        script = "arch/unix.nut"
    },

}
