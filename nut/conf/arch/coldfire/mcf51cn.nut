
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
--              "HW_MTIM_COLDFIRE",
--              "HW_IIC_COLDFIRE",
--              "HW_GPIO_COLDFIRE",
--              "HW_COP_COLDFIRE",
--              "HW_FEC_COLDFIRE",
--              "HW_RTC_COLDFIRE",
--              "HW_SPI_COLDFIRE",
--              "HW_TMP_COLDFIRE",
--              "HW_KBI_COLDFIRE",
--              "HW_MFB_COLDFIRE",
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
--          {
--              macro = "IIC1",
--              type = "integer",
--              default = 1,
--              provides = { "HW_IIC1" },
--              file = "include/cfg/peripherals.h"
--          },
--          {
--              macro = "IIC2",
--              type = "integer",
--              default = 1,
--              provides = { "HW_IIC2" },
--              file = "include/cfg/peripherals.h"
--          },
--          {
--              macro = "MTIM1",
--              type = "integer",
--              default = 1,
--              provides = { "HW_MTIM1" },
--              file = "include/cfg/peripherals.h"
--          },
--          {
--              macro = "MTIM2",
--              type = "integer",
--              default = 1,
--              provides = { "HW_MTIM2" },
--              file = "include/cfg/peripherals.h"
--          },
--          
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
--                  "DEV_IRQ_MTIM",
--                  "DEV_IRQ_IIC",
--                  "DEV_IRQ_TPM",
--                  "DEV_IRQ_RTC",
--                  "DEV_IRQ_SPI",
--                  "DEV_IRQ_FEC",
                   },
        sources = { 
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_mtim.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_tpm.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_adc.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_rtc.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_spi.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_fec.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_sci.c",
        		  },
    },

    --
    -- Internal Flash
    --
    {
        name = "nutarch_m68k_coldfire_devices_intflash",
        brief = "Internal Flash",
        description = "Code snippet for modiffying internal flash memory.",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_intflash.c" },
    },

    -- 
    -- Multipurpose Clock Generator
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mcg",
        brief = "Clock Setup",
        description = "Multipurpose Clock Generator",
        script = "arch/coldfire/mcf51cn_mcg.nut"
    },
    
    --
    -- Modulo Timer
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mtim",
        brief = "Modulo Timer",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_mtim.c" },
    },
    
    --
    -- Timer/PWM Module (TPM)
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
    -- Reset Controller
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_reset",
        brief = "Reset Controller",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_reset.c" },
    },
    
    --
    -- Real-Time Counter (RTC)
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_rtc",
        brief = "Real-Time Counter (RTC)",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_rtc.c" },
    },
    
    --
    -- Serial Peripheral Interface (SPI)
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
    -- Fast Ethernet Controller (FEC)
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_fec",
        brief = "Fast Ethernet Controller (FEC)",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_fec.c" },
    },
}
