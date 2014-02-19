nutarch_m68k_coldfire_mcf52 =
{
    --
    -- Runtime Initialization
    --
    {
        name = "nutarch_m68k_coldfire_mcf52_init",
        brief = "Initialization",
        description = "System startup code for Coldfire V2 Core MCUs.",
        sources = { 
                    "m68k/coldfire/init/crt_$(LDNAME).S",
                    "m68k/coldfire/init/crt_common.S",
                    "m68k/coldfire/init/crt_common_c.c", 
                    "m68k/coldfire/init/crt_mcf52.S",
                    "m68k/coldfire/init/crt_mcf52_c.c",
                  },
        targets = { 
                    "m68k/coldfire/init/crt_$(LDNAME).o",
                    "m68k/coldfire/init/crt_common.o",
                    "m68k/coldfire/init/crt_common_c.o",
                    "m68k/coldfire/init/crt_mcf52.o", 
                    "m68k/coldfire/init/crt_mcf52_c.o", 
                  },
        requires = { "TOOL_CC_M68K", "TOOL_GCC"},
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
