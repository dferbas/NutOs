
mcf51cn_sci1_txd_pins = { "PTD0" }
mcf51cn_sci1_rxd_pins = { "PTD1" }
mcf51cn_sci1_txd_pins_descr = "Choose SCI1 TXD Pin:\n\tPTD0 (RGPIO0)"
mcf51cn_sci1_rxd_pins_descr = "Choose SCI1 RXD Pin:\n\tPTD1 (RGPIO1)"

mcf51cn_sci2_txd_pins = { "PTD2" }
mcf51cn_sci2_rxd_pins = { "PTD3" }
mcf51cn_sci2_txd_pins_descr = "Choose SCI2 TXD Pin:\n\tPTD2 (RGPIO2)"
mcf51cn_sci2_rxd_pins_descr = "Choose SCI2 RXD Pin:\n\tPTD3 (RGPIO3)"

mcf51cn_sci3_txd_pins = { "PTE6", "PTA3" }
mcf51cn_sci3_rxd_pins = { "PTE7", "PTA4" }
mcf51cn_sci3_txd_pins_descr = "Choose SCI3 TXD Pin:\n\tPTE6 (KBI2P6)\n\tPTA3"
mcf51cn_sci3_rxd_pins_descr = "Choose SCI3 RXD Pin:\n\tPTE7 (KBI2P7)\n\tPTA4"

nutarch_m68k_coldfire_mcf51cn =
{
   	--
    -- MCU Family
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_family",
        brief = "MCU Family",
        provides = {
                "HW_ADC12_COLDFIRE",
                "HW_SCI_COLDFIRE",
        },
        options =
        {
            {
                macro = "MCU_MCF51CN",
                brief = "MCF51CN",
                description = "MCF51CN Coldfire Family",
                type = "integer",
                default = 1,
                file = "include/cfg/arch.h"
            },
            {
                macro = "SCI1",
                type = "integer",
                default = 1,
                provides = { "HW_SCI1" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "SCI2",
                type = "integer",
                default = 1,
                provides = { "HW_SCI2" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "SCI3",
                type = "integer",
                default = 1,
                provides = { "HW_SCI3" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "FEC",
                type = "integer",
                default = 1,
                provides = { "HW_FEC" },
                file = "include/cfg/peripherals.h"
            },
        }
    },
    
    --
    -- Runtime Initialization
    --
    {
        name = "nutarch_m68k_coldfire_mcf51_init",
        brief = "Initialization",
        description = "System startup code for Coldfire V1 Core MCUs.",
        sources = { 
                    "m68k/coldfire/init/crt_$(LDNAME).S",
                    "m68k/coldfire/init/crt_common.S",
                    "m68k/coldfire/init/crt_common_c.c", 
                    "m68k/coldfire/init/crt_mcf51cn.S",
                    "m68k/coldfire/init/crt_mcf51cn_c.c",
                  },
        targets = { 
                    "m68k/coldfire/init/crt_$(LDNAME).o",
                    "m68k/coldfire/init/crt_common.o",
                    "m68k/coldfire/init/crt_common_c.o", 
                    "m68k/coldfire/init/crt_mcf51cn.o", 
                    "m68k/coldfire/init/crt_mcf51cn_c.o",
                  },
        requires = { "TOOL_CC_M68K", "TOOL_GCC"},
        options =
        {
            {
                macro = "NUT_WDT_ENABLE",
                brief = "Watchdog",
                description = "If enabled, the watchdog timer will be started during system initialization.\n\n"..
                              "Once the watchdog is configured, it is not possible to re-configure the watchdog.\n"..
                              "If enabled, NutWatchDogRestart() functiom must be called periodically from application.",
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "NUT_LVD_ENABLE",
                brief = "Low Voltage Detection",
                description = "If enabled, the Low Voltage Detection module resets MCU after low voltage is detected",
                flavor = "booldata",
                file = "include/cfg/clock.h"
            }
        }
    },    

    --
    -- GPIO Interface
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_gpio",
        brief = "GPIO",
        description = "Generic port I/O API.",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_gpio.c"}
    },
    
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_ihndlr",
        brief = "Interrupt Handler",
        description = "Peripheral interrupt handlers for MCF51CN family.",
        provides = { 
                    "DEV_IRQ_ADC",
                    "DEV_IRQ_SCI",
                    "DEV_IRQ_FEC",
                   },
        sources = { 
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_mtim.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_rtc.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_spi.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_sci.c",
          			"m68k/coldfire/dev/common/ih_mcf5_fec.c",
        		  },
    },

    -- 
    -- Multipurpose Clock Generator
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mcg",
        brief = "Clock Setup (MCG)",
        description = "Multipurpose Clock Generator",
        script = "arch/coldfire/mcf51cn_mcg.nut"
    },
    
    --
    -- Modulo Timer
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mtim",
        brief = "Modulo Timer (MTIM)",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_mtim.c" },
    },
    
    --
    -- Timer/PWM Module
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_tpm",
        brief = "Timer/PWM Module (TPM)",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_tpm.c" },
    },

    --
    -- System Timer
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_ostimer",
        brief = "System Timer",
        provides = { "NUT_OSTIMER_DEV" },
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_ostimer.c" },
    },
    
    --
    -- Real Time Counter
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_rtc",
        brief = "Real Time Counter (RTC)",
        description = "WARNING! This is not a real time CLOCK, but a real time COUNTER only.\n\n"..
                      "It is only a timer/counter which uses different clock souce than the system "..
                      "timer, so it runs even if system clock is halted.",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_rtc.c" },
    },
    
    --
    -- Serial Peripheral Interface
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_spi",
        brief = "Serial Peripheral Interface (SPI)",
        description = "Deprecated, non-standard SPI driver created during rapid developement. \n"..
                      "It is used only by FRAM FM25L04B and by SPI Serial Flash SST25VF020B. \n\n"..
                      "TODO: This driver must be recoded to use standard SPIBUS framework",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_spi.c" },
        provides = { "DEV_SPI_MCF51CN_DEPRECATED" },
    },
    
    --
    -- Analog to Digital Converter
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_adc",
        brief = "A/D Converter (ADC12)",
        description = "12-bit analog-to-digital converter.",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_adc.c" },
    },   
    
    --
    -- Fast Ethernet Controller
    --
    {
        name = "nutarch_m68k_coldfire_mcf5_fec",
        brief = "Fast Ethernet Controller (FEC)",
        sources = { "m68k/coldfire/dev/common/mcf5_fec.c" },
    }, 
}
