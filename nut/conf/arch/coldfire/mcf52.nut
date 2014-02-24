nutarch_m68k_coldfire_mcf52 =
{
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
