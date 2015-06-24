#ifndef _DEV_BOARD_H_
#define _DEV_BOARD_H_

/*
 * Copyright (C) 2001-2007 by egnite Software GmbH
 * Copyright (C) 2012 by egnite GmbH
 *
 * All rights reserved.
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

/*!
 * \file dev/board.h
 * \brief Default board configuration.
 *
 * The purpose of this file is to help creating Nut/OS applications,
 * which will run on many (or all) supported target boards without
 * I/O-device related modification, by simply adding
 * \code
 * #include <dev/board>
 * \endcode
 *
 * The file is intended for inclusion in application code only. If
 * it seems to be required in a library file, then something else
 * is probably broken. Ignoring this advice may lead to unforeseen
 * header file hell. As an exception, it may be included into certain
 * initialization files like nutinit.c.
 *
 * Roughly speaking, this file provides a number of default devices,
 * which are evaluated in three steps:
 *
 * - Board specific settings are defined first.
 * - All defaults, which had not been declared for the board, may
 *   be specified based on the target CPU.
 * - Finally all defaults are set, which were neither based on the
 *   board nor on the target CPU.
 *
 * Instead of referring to platform specific device drivers, like
 * \code
 * NutRegisterDevice(&devDebug0, 0, 0);
 * freopen("uart0", "w", stdout);
 * \endcode
 * applications can use globally assigned macros, like
 * \code
 * NutRegisterDevice(&DEV_DEBUG, 0, 0);
 * freopen(DEV_DEBUG_NAME, "w", stdout);
 * \endcode
 * which will work for all target boards.
 *
 * \verbatim
 * $Id: board.h 4692 2012-10-03 12:50:37Z shindy2m $
 * \endverbatim
 */

/*
 * \todo Not sure if we should include them here. Possibly yes, but in
 *       this case we should add all device related configuration files.
 *       However, simply adding the complete cfg directory without sense
 *       will significantly increase compile times during distcheck
 *       and make it harder to solve conflicts.
 */
#include <cfg/arch.h>
#include <cfg/uart.h>
#include <compiler.h>

/*
 * Include individual board header files.
 *
 * These files can be used to override any of the default settings,
 * which are following below. Each supported board should have this,
 * even if all defaults are fine. So users may modify the settings
 * without having to touch this global file. The selection is
 * based on the PLATFORM entry in the board configuration file.
 * Some boards, like the MMNET family, may share the same header file
 * or, like the USPS family, may even share the same platform, if they
 * are based on the same hardware design.
 *
 * Individual headers must not define any settings, which are
 * configured by default anyway. Keep the maintenance effort for
 * your board to a minimum. Developers are not be able to check
 * every individual board when adding global modifications.
 *
 * In order to allow modifications via Userconf.mk, all settings should
 * make sure, that they were previously undefined, .e.g:
 * \code
 * #ifndef SAMPLE_MACRO
 * #define SAMPLE_MACRO 1
 * #endif
 * \endcode
 *
 * All individual headers must contain the following preprocessor
 * check to make sure, that it is included via dev/board.h only:
 * \code
 * #ifndef _DEV_BOARD_H_
 * #error "Do not include this file directly. Use dev/board.h instead!"
 * #endif
 * \endcode
 * Otherwise compilation may fail after new items are added here.
 *
 * Please keep this list sorted alphabetically.
 */
#if defined(ARTHERNET1)
#include <arch/avr/board/arthernet1.h>
#elif defined(AT91SAM7X_EK)
#include <arch/arm/board/at91sam7x_ek.h>
#elif defined(AT91SAM9260_EK)
#include <arch/arm/board/at91sam9260_ek.h>
#elif defined(CHARON2)
#include <arch/avr/board/charon2.h>
#elif defined(ELEKTOR_IR1)
#include <arch/arm/board/elektor_ir1.h>
#elif defined(ENET_SAM7X)
#include <arch/arm/board/enet_sam7x.h>
#elif defined(ETHERNUT1)
#include <arch/avr/board/ethernut1.h>
#elif defined(ETHERNUT2)
#include <arch/avr/board/ethernut2.h>
#elif defined(ETHERNUT3)
#include <arch/arm/board/ethernut3.h>
#elif defined(ETHERNUT5)
#include <arch/arm/board/ethernut5.h>
#elif defined(EVK1100)
#include <arch/avr32/board/evk1100.h>
#elif defined(EVK1101)
#include <arch/avr32/board/evk1101.h>
#elif defined(EVK1104)
#include <arch/avr32/board/evk1104.h>
#elif defined(EVK1105)
#include <arch/avr32/board/evk1105.h>
#elif defined(F4_DISCOVERY)
#include <arch/cm3/board/f4_discovery.h>
#elif defined(FLECX1)
#include <arch/cm3/board/flecx1.h>
#elif defined(GBAXPORT2)
#include <arch/cm3/board/gba_xport2.h>
#elif defined(KSK_LPC1788_SK)
#include <arch/cm3/board/ksk_lpc1788_sk.h>
#elif defined(LISA)
#include <arch/cm3/board/lisa.h>
#elif defined(MMNET01) || defined(MMNET02) || defined(MMNET101) || defined(MMNET102)
#include <arch/avr/board/mmnet.h>
#elif defined(MORPHOQ1)
#include <arch/arm/board/morphoq1.h>
#elif defined(SAM7ETH)
#include <arch/arm/board/sam7eth.h>
#elif defined(STM3210C_EVAL)
#include <arch/cm3/board/stm3210c_eval.h>
#elif defined(STM3210E_EVAL)
#include <arch/cm3/board/stm3210e_eval.h>
#elif defined(STEVAL_PCC010V2)
#include <arch/cm3/board/steval_pcc010v2.h>
#elif defined(STM32_CAN)
#include <arch/cm3/board/stm32_can.h>
#elif defined(STM32_COMSTICK)
#include <arch/cm3/board/stm32_comstick.h>
#elif defined(USPS)
#include <arch/cm3/board/usps.h>
#elif defined(OLIMEX_LPCE2294)
#include <arch/arm/board/olimex_lpce2294.h>
#elif defined(XNUT_100) || defined(XNUT_105)
#include <arch/avr/board/xnut.h>
#elif defined(SM2_MU)
#include <arch/m68k/board/sm2_mu.h>
#elif defined(POWERNET_MOD)
#include <arch/m68k/board/powernet_mod.h>
#elif defined(SM2_AD_PC)
#include <arch/m68k/board/sm2_ad_pc.h>
#elif defined(POSEIDON)
#include <arch/m68k/board/poseidon.h>
#elif defined(ZERO_EK)
#include <arch/arm/board/zero_ek.h>
#endif


/*
 * Define CPU specific defaults.
 *
 * After each board claims its individual settings, we will try
 * to figure out remaining defaults based on the CPU or CPU family.
 */

#if defined(__AVR__)
/*
 * UART devices for Atmel's AVR 8-bit family.
 */
#include <dev/usartavr.h>

/* We only need to define the devices, their names are system wide
   defaults. Note, that we use the advanced drivers here. Boards
   with scarce memory resources or running a slow clocks may use
   devUart0 and devUart1 instead. */
#ifndef DEV_UART0
#define DEV_UART0       devUsartAvr0
#endif
#ifndef DEV_UART1
#define DEV_UART1       devUsartAvr1
#endif

/*
 * SPI bus controller for Atmel's AVR 8-bit family.
 */
#include <dev/spibus_avr.h>
#ifndef DEV_SPIBUS0
#define DEV_SPIBUS0     spiBus0Avr
#endif

#elif defined(MCU_AT91)

/*
 * Debug device for Atmel's ARM based AT91 family.
 *
 * All chips with DBGU interface will most likely use it for debug
 * output. If such an interface is not available, than we rely on
 * the system wide default, which is declared further down.
 *
 * \note We assume, that the system default debug driver devDebug0
 *       is always available on devices without DBGU.
 */
#ifdef DBGU_BASE
#ifndef DEV_DEBUG
#define DEV_DEBUG       devDebug
#endif
#ifndef DEV_DEBUG_NAME
#define DEV_DEBUG_NAME  "dbgu"
#endif
#endif

/*
 * UART devices for Atmel's ARM based AT91 family.
 *
 * Note, that DEV_UARTD is special. It uses the DGBU interface, but
 * as an interrupt driven driver, it shouldn't be used for debugging.
 */
#include <dev/usartat91.h>

#ifndef DEV_UART0
#define DEV_UART0       devUsartAt910
#endif
#ifndef DEV_UART1
#define DEV_UART1       devUsartAt911
#endif
#if !defined(DEV_UARTD) && defined(DBGU_BASE)
#define DEV_UARTD       devDbguAt91
#endif

/*
 * SPI bus controller for Atmel's ARM based AT91 family.
 */
#if !defined(DEV_SPIBUS0) && defined(SPI0_BASE)
#include <dev/spibus_at91.h>
#define DEV_SPIBUS0     spiBus0At91
#endif

/*
 * TWI / I2C bus controller for Atmel's ARM based AT91 family.
 *
 * \todo Somewhere is a problem with bit banging. If the CPU has a TWI
 * hardware interface, we cannot define another controller based on bit
 * banging.
 */
#include <cfg/twi.h>
#include <dev/twif.h>
#if defined(MCU_AT91R40008)
#include <dev/twibus_bbif.h>
#define DEF_TWIBUS      TwBbifBus
#else
#include <dev/twibus_at91.h>
#define DEF_TWIBUS      At91TwiBus
#endif

#elif defined(MCU_LPC17xx)
/*
 * UART devices for the ARM based LPC17xx family.
 *
 * \note Instead of providing literal names, references to the
 *       driver structure are used here. Therefore all names are
 *       given here, even it they use the system wide default
 *       name. Furthermore, all possible devices seem to have
 *       been added here, which makes it hard for applications
 *       to figure out, if they are available at all on the
 *       target hardware.
 */
#include <dev/usart_lpc17xx.h>

#if defined(MCU_LPC176x)

#ifndef DEV_UART0
#define DEV_UART0       devUsartLpc176x_0
#endif
#ifndef DEV_UART0_NAME
#define DEV_UART0_NAME  "usart0"
#endif

#else

#ifndef DEV_UART0
#define DEV_UART0       devUsartLpc17xx_0
#endif
#ifndef DEV_UART0_NAME
#define DEV_UART0_NAME  DEV_UART0.dev_name
#endif

#ifndef DEV_UART1
#define DEV_UART1       devUsartLpc17xx_1
#endif
#ifndef DEV_UART1_NAME
#define DEV_UART1_NAME  DEV_UART1.dev_name
#endif

#ifndef DEV_UART2
#define DEV_UART2       devUsartLpc17xx_2
#endif
#ifndef DEV_UART2_NAME
#define DEV_UART2_NAME  DEV_UART2.dev_name
#endif

#ifndef DEV_UART3
#define DEV_UART3       devUsartLpc17xx_3
#endif
#ifndef DEV_UART3_NAME
#define DEV_UART3_NAME  DEV_UART3.dev_name
#endif

#endif

#elif defined(__AVR32__)
/*
 * UART devices for Atmel's AVR 32-bit family.
 *
 * Clear and simple, that's how we like it. But wait, where is the
 * debug driver?
 */
#include <dev/usartavr32.h>

#ifndef DEV_UART0
#define DEV_UART0       devUsartAvr320
#endif
#ifndef DEV_UART1
#define DEV_UART1       devUsartAvr321
#endif
#ifndef DEV_UART2
#define DEV_UART2       devUsartAvr322
#endif

/*
 * SPI bus controllers for Atmel's AVR 32-bit family.
 */
#include <dev/spibus_avr32.h>
#define DEV_SPIBUS0     spiBus0Avr32
#define DEV_SPIBUS1     spiBus1Avr32

#elif defined(MCU_STM32)
/*
 * Debug devices for the ARM based STM32 family.
 *
 * \todo Looks like this debug driver is using a normal, possibly
 *       interrupt driven UART driver, which makes its use quite
 *       limited. Better do not use it within interrupt routines
 *       and do not rely on synchronicity. Output may be intermixed
 *       or even scrambled during debugging.
 */
#ifndef DEV_DEBUG
#define DEV_DEBUG       DEV_UART1
#endif
#ifndef DEV_DEBUG_NAME
#define DEV_DEBUG_NAME  DEV_UART1_NAME
#endif

/*
 * UART devices for the ARM based STM32 family.
 *
 * \note Instead of providing literal names, references to the
 *       driver structure are used here. Therefore all names are
 *       given here, even it they use the system wide default
 *       name. Furthermore, all possible devices seem to have
 *       been added here, which makes it hard for applications
 *       to figure out, if they are available at all on the
 *       target hardware.
 */
#include <dev/usartstm32.h>

#ifndef DEV_UART1
#define DEV_UART1       devUsartStm32_1
#endif
#ifndef DEV_UART1_NAME
#define DEV_UART1_NAME  DEV_UART1.dev_name
#endif

#ifndef DEV_UART2
#define DEV_UART2       devUsartStm32_2
#endif
#ifndef DEV_UART2_NAME
#define DEV_UART2_NAME  DEV_UART2.dev_name
#endif

#ifndef DEV_UART3
#define DEV_UART3       devUsartStm32_3
#endif
#ifndef DEV_UART3_NAME
#define DEV_UART3_NAME  DEV_UART3.dev_name
#endif

#ifndef DEV_UART4
#define DEV_UART4       devUartStm32_4
#endif
#ifndef DEV_UART4_NAME
#define DEV_UART4_NAME  DEV_UART4.dev_name
#endif

#ifndef DEV_UART5
#define DEV_UART5       devUartStm32_5
#endif
#ifndef DEV_UART5_NAME
#define DEV_UART5_NAME  DEV_UART5.dev_name
#endif

#ifndef DEV_UART6
#define DEV_UART6       devUsartStm32_6
#endif
#ifndef DEV_UART6_NAME
#define DEV_UART6_NAME  DEV_USART6.dev_name
#endif

/*
 * System default UART0 is not available on STM32 boards. Thus,
 * we declare UART1 as the default.
 */
#ifndef DEV_UART
#define DEV_UART        DEV_UART1
#define DEV_UART_NAME   DEV_UART1_NAME
#endif

/*
 * TWI / I2C bus controller for the ARM based STM32 family.
 */
#include <cfg/twi.h>
#include <dev/twif.h>
#include <arch/cm3/stm/stm32_twi.h>

#elif defined(__NUT_EMULATION__)
/*
 * UART devices for the UNIX emulation.
 */
#include <dev/usartavr.h>

#ifndef DEV_UART0
#define DEV_UART0       devUsartAvr0
#endif

#ifndef DEV_UART1
#define DEV_UART1       devUsartAvr1
#endif

#define DEV_UART1_NAME  "uart1"

#endif

/*
 * Define system wide defaults.
 *
 * After each board and each target CPU claimed their individual settings,
 * we will set all remaining defaults.
 *
 * Use most likely defaults here, to keep board and CPU defaults to a
 * minimum. For example, the UART0 device is named "uart0" by almost
 * all drivers and there is rarely any reason other than to confuse
 * people for choosing a different name. Sticking to the default
 * reduces code lines.
 */

/*
 * UART device defaults.
 *
 * We have all kind of UART drivers. Some platforms, like the AVR, do
 * have four(!) of them, all running on the same physical interface:
 * One simple polling driver, one fast interrupt driven driver with
 * limited buffer and handshake capabilities, one full fledged driver
 * with all bells and whistles and one special driver optimized for
 * AHDLC.
 *
 * Furthermore all drivers use different structure variable names.
 * It doesn't make much sense to define any default here.
 *
 * Device names, however, are the same for most implementations.
 * We will also select the UART with the lowest index as our default
 * DEV_UART.
 */
#ifdef DEV_UART0
#ifndef DEV_UART0_NAME
#define DEV_UART0_NAME  "uart0"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UART0
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UART0_NAME
#endif
#endif

#ifdef DEV_UART1
#ifndef DEV_UART1_NAME
#define DEV_UART1_NAME  "uart1"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UART1
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UART1_NAME
#endif
#endif

#ifdef DEV_UART2
#ifndef DEV_UART2_NAME
#define DEV_UART2_NAME  "uart2"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UART2
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UART2_NAME
#endif
#endif

#ifdef DEV_UART3
#ifndef DEV_UART3_NAME
#define DEV_UART3_NAME  "uart3"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UART3
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UART3_NAME
#endif
#endif

#ifdef DEV_UART4
#ifndef DEV_UART4_NAME
#define DEV_UART4_NAME  "uart4"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UART4
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UART4_NAME
#endif
#endif

#ifdef DEV_UART5
#ifndef DEV_UART5_NAME
#define DEV_UART5_NAME  "uart5"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UART5
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UART5_NAME
#endif
#endif

#ifdef DEV_UART6
#ifndef DEV_UART6_NAME
#define DEV_UART6_NAME  "uart6"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UART6
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UART6_NAME
#endif
#endif

#ifdef DEV_UART7
#ifndef DEV_UART7_NAME
#define DEV_UART7_NAME  "uart7"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UART7
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UART7_NAME
#endif
#endif

#ifdef DEV_UARTD
#ifndef DEV_UARTD_NAME
#define DEV_UARTD_NAME  "uartd"
#endif
#ifndef DEV_UART
#define DEV_UART        DEV_UARTD
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME   DEV_UARTD_NAME
#endif
#endif

/*
 * Debug device defaults.
 *
 * Due to its hardware dependency, there is no such thing like a global
 * debug device. However, in reality devDebug0 is available on most
 * platforms. Not sure, if it had been a good idea giving them all the same
 * structure name. Probably they were created by copying and pasting
 * the original AVR driver. The good news is, that they all share the same
 * header file.
 *
 * In most cases the driver will be implemented on the default UART. So
 * we can use that name as the most likely default.
 *
 * Note, that Nut/OS device names reflect the hardware they are attached to.
 * If more than one driver exists for the same hardware, they will all have
 * the same name. In most cases this prevents applications from accidently
 * using more than one driver on a specific hardware interface, because
 * NutRegisterDevice() expects a unique name for each registered device.
 */
#include <dev/debug.h>

#ifndef DEV_DEBUG
#define DEV_DEBUG       devDebug0
#endif
#ifndef DEV_DEBUG_NAME
#define DEV_DEBUG_NAME  DEV_UART_NAME
#endif

/*
 * Console device defaults.
 *
 * The console device is used for stdin and stdout, attached to the
 * main serial port of the board. If more than one serial port is
 * available, then the same port that is attached to the debug device
 * is selected. But it's not that easy really.
 *
 * In earlier releases Nut/OS provides the debug device for progress
 * reports. At that time the debug driver was write-only to keep it
 * small and simple. When user input was required, the interrupt
 * driven UART0 driver was used instead. As long as both drivers
 * referred to the same physical interface, this was no problem.
 *
 * The later supported AT91 family offers a unique DGBU interface,
 * which was naturally used for the debug driver. When running existing
 * applications, users of this hardware had to switch between DBGU
 * and UART0, either by changing jumper settings or changing connectors.
 * Can you imagine, how often users were desperately waiting for output
 * until they recognized, that the application requires to switch the
 * physical interface?
 *
 * To solve this, some debug drivers had been expanded with input
 * capabilities, allowing to use same interface for write-only and
 * read/write applications. But the increase of code size is not
 * bearable by all targets, and quite senseless, if they already have
 * a full UART driver at the same physical interface. If defined,
 * the configuration item NUT_DEV_DEBUG_READ enables the input
 * capability and is used here, to determine, whether the debug
 * device is able to support both, stdout and stdin.
 */
#ifndef DEV_CONSOLE
#ifdef NUT_DEV_DEBUG_READ
/* Debug device is read/write, use it. */
#define DEV_CONSOLE      DEV_DEBUG
#ifndef DEV_CONSOLE_NAME
#define DEV_CONSOLE_NAME DEV_DEBUG_NAME
#endif
#else
/* Debug device is write-only, use default UART. */
#define DEV_CONSOLE      DEV_UART
#ifndef DEV_CONSOLE_NAME
#define DEV_CONSOLE_NAME DEV_UART_NAME
#endif
#endif
#endif

/*
 * Ethernet device defaults.
 *
 * Boards may not offer any Ethernet interface. However, many of our
 * sample applications are network based. It would require a lot of
 * ugly preprocessor ifdefs to make them compile and avoid crashing
 * them on hardware without a network interface. Therefore a so called
 * Null-driver will be used if no real interface had been specified
 * above.
 */
#ifndef DEV_ETHER
#include <dev/null_ether.h>
#endif
#ifndef DEV_ETHER_NAME
#define DEV_ETHER_NAME  "eth0"
#endif

/*
 * SPI bus controller defaults.
 *
 * Select the interface with the lowest index as our default.
 */
#ifndef DEV_SPIBUS
#if defined(DEV_SPIBUS0)
#define DEV_SPIBUS      DEV_SPIBUS0
#elif defined(DEV_SPIBUS1)
#define DEV_SPIBUS      DEV_SPIBUS1
#endif
#endif

/*
 * No TWI / I2C bus controller defaults.
 */

/*
 * Memory card driver defaults.
 *
 * We expect the board or CPU configuration to specify DEV_MMCARD0,
 * which will be used for the default interface DEV_MMCARD.
 */
#if defined(DEV_MMCARD0)
#ifndef DEV_MMCARD
#define DEV_MMCARD          DEV_MMCARD0
#endif
#ifndef DEV_MMCARD0_NAME
#define DEV_MMCARD0_NAME    "MMC0"
#endif
#ifndef DEV_MMCARD_NAME
#define DEV_MMCARD_NAME     DEV_MMCARD0_NAME
#endif
#endif

/*
 * Real time clock defaults.
 *
 * We expect the board or CPU configuration to specify RTC_CHIP0,
 * which will be used for the default interface RTC_CHIP.
 *
 * This effort looks a little bit funny, because boards will rarely have
 * two of this. But who knows...
 */
#ifdef RTC_CHIP0
#ifndef RTC_CHIP
#define RTC_CHIP RTC_CHIP0
#endif
#endif


extern void NutBoardInit(void);
extern void NutIdleInit(void);
extern void NutMainInit(void);

#endif

