
nutarch_m68k_coldfire_mcf51qe =
{
   	--
    -- MCU Family
    --
    {
        name = "nutarch_m68k_coldfire_mcf51qe_family",
        brief = "MCU Family",
        provides = {
                "HW_ADC12_COLDFIRE",
                "HW_SCI_COLDFIRE",
        },
        options =
        {
            {
                macro = "MCU_MCF51QE",
                brief = "MCF51QE",
                description = "MCF51QE Coldfire Family",
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
                    "m68k/coldfire/init/crt_mcf51qe.S",
                    "m68k/coldfire/init/crt_mcf51qe_c.c",
                  },
        targets = { 
                    "m68k/coldfire/init/crt_$(LDNAME).o",
                    "m68k/coldfire/init/crt_common.o",
                    "m68k/coldfire/init/crt_common_c.o", 
                    "m68k/coldfire/init/crt_mcf51qe.o", 
                    "m68k/coldfire/init/crt_mcf51qe_c.o",
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
        name = "nutarch_m68k_coldfire_mcf51qe_gpio",
        brief = "GPIO",
        description = "Generic port I/O API.",
        sources = { "m68k/coldfire/dev/mcf51qe/mcf51qe_gpio.c"}
    },
    
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_mcf51qe_ihndlr",
        brief = "Interrupt Handler",
        description = "Peripheral interrupt handlers for MCF51QE family.",
        provides = { 
                    "DEV_IRQ_SCI",
                    "DEV_IRQ_SPI",
                    "DEV_IRQ_IIC",
                   },
        sources = { 
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_sci.c",
          			"m68k/coldfire/dev/mcf51/ih_mcf51_spi.c",
          			"m68k/coldfire/dev/common/ih_mcf5_iic.c",
        		  },
    },

    -- 
    -- Internal Clock Source
    --
    {
        name = "nutarch_m68k_coldfire_mcf51qe_ics",
        brief = "Clock Setup (ICS)",
        description = "Internal Clock Source",
        sources = { "m68k/coldfire/dev/mcf51qe/mcf51qe_ics.c" },
    },
    
    --
    -- System Timer
    --
    {
        name = "nutarch_m68k_coldfire_mcf51qe_ostimer",
        brief = "System Timer",
        provides = { "NUT_OSTIMER_DEV" },
        sources = { "m68k/coldfire/dev/mcf51qe/mcf51qe_ostimer.c" },
    },
    
    --
    -- Timer/PWM Module
    --
    {
        name = "nutarch_m68k_coldfire_mcf51qe_tpm",
        brief = "Timer/PWM Module (TPM)",
        sources = { "m68k/coldfire/dev/mcf51qe/mcf51qe_tpm.c" },
    },

    --
    --  Serial Peripheral Interface
    --    
    {
        name = "nutarch__m68k_coldfire_mcf51_spi",
        brief = "Serial Peripheral Interface",
        description = "Serial Peripheral Interface",
        provides = { "SPIBUS_CONTROLLER" },
        sources =
        {
        	"m68k/coldfire/dev/mcf51/mcf51_spi1.c",
            "m68k/coldfire/dev/mcf51/mcf51_spi2.c",
        },
    },
    
    --
    -- IIC - Inter-Integrated Circuit
    --
    {
        name = "nutarch_mcf5_iic",
        brief = "Inter-Integrated Circuit",
        description = "IIC for MCF5 family.",
        requires = {"DEV_IRQ_IIC"},
        provides = { "DEV_TWI" },
        sources = { "m68k/coldfire/dev/common/mcf5_iic.c" }
    },
    
    --
    -- Analog to Digital Converter
    --
    {
        name = "nutarch_m68k_coldfire_mcf51_adc",
        brief = "Analog to Digital Converter (ADC)",
        sources = { "m68k/coldfire/dev/mcf51qe/mcf51qe_adc.c" },
    },
    
    
}
