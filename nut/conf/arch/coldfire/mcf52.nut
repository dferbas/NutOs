nutarch_m68k_coldfire_mcf52 =
{
    {
        name = "nutarch_m68k_coldfire_mcf52_mcf5225",
        brief = "MCF5225 Family",
        requires = { "HW_MCU_MCF5225" },
        description = "MCF5225 family",
        script = "arch/coldfire/mcf5225.nut"
    },
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_mcf52_ihndlr",
        brief = "Interrupt Handler",
        description = "Common part of interrupt handlers for for Coldfire V1 Core MCUs.",
        sources = { 
                    "m68k/coldfire/dev/mcf52/ih_mcf52_common.c",
                    "m68k/coldfire/dev/mcf52/ih_mcf52_default.c",
                  },
    },
}
