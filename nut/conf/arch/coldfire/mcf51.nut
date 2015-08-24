nutarch_m68k_coldfire_mcf51 =
{
    {
        name = "nutarch_m68k_coldfire_mcf51_mcf51cn",
        brief = "MCF51CN Family",
        requires = { "HW_MCU_MCF51CN" },
        description = "MCF51CN family",
        script = "arch/coldfire/mcf51cn.nut"
    },
    {
        name = "nutarch_m68k_coldfire_mcf51_mcf51qe",
        brief = "MCF51QE Family",
        requires = { "HW_MCU_MCF51QE" },
        description = "MCF51QE family",
        script = "arch/coldfire/mcf51qe.nut"
    },
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_mcf51_ihndlr",
        brief = "Interrupt Handler",
        description = "Common part of interrupt handlers for Coldfire V1 Core MCUs.",
        provides = { 
                    "DEV_IRQ_SCI",
                    "DEV_IRQ_TPM",
                    "DEV_IRQ_ADC",
                    "DEV_IRQ_IIC",
                    "DEV_IRQ_SPI",
                   },
        sources = { 
                    "m68k/coldfire/dev/mcf51/ih_mcf51_common.c",
                    "m68k/coldfire/dev/mcf51/ih_mcf51_default.c",
                    "m68k/coldfire/dev/mcf51/ih_mcf51_sci.c",
                    "m68k/coldfire/dev/mcf51/ih_mcf51_tpm.c",
                    "m68k/coldfire/dev/mcf51/ih_mcf51_adc.c",
                    "m68k/coldfire/dev/mcf51/ih_mcf51_iic.c",
                    "m68k/coldfire/dev/mcf51/ih_mcf51_spi.c",
                  },
    },
    
    --
    -- Reset Controller
    --
    {
        name = "nutarch_m68k_coldfire_mcf51_reset",
        brief = "Reset Controller",
        sources = { "m68k/coldfire/dev/mcf51/mcf51_reset.c" },
    },
    
    --
    -- Watchdog Timer
    --
    {
        name = "nutarch_m68k_coldfire_mcf51_cop",
        brief = "Watchdog (COP)",
        sources = { "m68k/coldfire/dev/mcf51/mcf51_cop.c" },
    },

    --
    -- Internal Flash
    --
    {
        name = "nutarch_m68k_coldfire_mcf51_intflash",
        brief = "Internal Flash",
        description = "Code snippet for modiffying internal flash memory.",
        sources = { "m68k/coldfire/dev/mcf51/mcf51_intflash.c" },
    },

    --
    -- IIC - Inter-Integrated Circuit
    --
    {
        name = "nutarch__m68k_coldfire_mcf51_iic",
        brief = "Inter-Integrated Circuit",
        description = "IIC for MCF5 family.",
        requires = {"DEV_IRQ_IIC"},
        provides = { "DEV_TWI" },
        sources = { "m68k/coldfire/dev/mcf51/mcf51_iic.c" },
        requires = { "DEV_IRQ_IIC"},
    },

    --
    -- SPI - Serial Peripheral Interface
    -- TODO: Temporarily fixed only to MCF51QE - AD_PC board
    -- TODO: modify board related code (pin setup, chip selects) 
    --    
    {
        name = "nutarch__m68k_coldfire_mcf51_spi",
        brief = "Serial Peripheral Interface",
        description = "Serial Peripheral Interface\n\n"..
                      "Temporarily fixed to MCF51QE - AD_PC board only",
        requires = { "DEV_IRQ_SPI", "SPI_BUS_DRIVER_AD_PC_ONLY"},
        provides = { "SPIBUS_CONTROLLER" },
        sources =
        {
            "m68k/coldfire/dev/mcf51/mcf51_spi1.c",
            "m68k/coldfire/dev/mcf51/mcf51_spi2.c",
        },
    },    
}
