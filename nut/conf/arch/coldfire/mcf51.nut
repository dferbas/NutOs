nutarch_m68k_coldfire_mcf51 =
{
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_mcf51_ihndlr",
        brief = "Interrupt Handler",
        description = "Common part of interrupt handlers for for Coldfire V1 Core MCUs.",
        sources = { 
                    "m68k/coldfire/dev/mcf51/ih_mcf51_common.c",
                    "m68k/coldfire/dev/mcf51/ih_mcf51_default.c",
          			"m68k/coldfire/dev/mcf51/ih_mcf51_tpm.c",
          			"m68k/coldfire/dev/mcf51/ih_mcf51_adc.c",
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
        name = "nutarch_m68k_coldfire_devices_intflash",
        brief = "Internal Flash",
        description = "Code snippet for modiffying internal flash memory.",
        sources = { "m68k/coldfire/dev/mcf51/mcf51_intflash.c" },
    },
}
