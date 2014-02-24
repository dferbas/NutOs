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
                  },
    },
}
