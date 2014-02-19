nutarch_m68k_coldfire_mcf51 =
{
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
                    "m68k/coldfire/init/crt_mcf51.S",
                    "m68k/coldfire/init/crt_mcf51_c.c",
                  },
        targets = { 
                    "m68k/coldfire/init/crt_$(LDNAME).o",
                    "m68k/coldfire/init/crt_common.o",
                    "m68k/coldfire/init/crt_common_c.o", 
                    "m68k/coldfire/init/crt_mcf51.o", 
                    "m68k/coldfire/init/crt_mcf51_c.o",
                  },
        requires = { "TOOL_CC_M68K", "TOOL_GCC"},
    },    

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
    
    --
    -- Watchdog Timer(COP)
    --
    {
        name = "nutarch_m68k_coldfire_mcf51_cop",
        brief = "Watchdog (COP)",
        sources = { "m68k/coldfire/dev/mcf51/mcf51_cop.c" },
        options =
        {
            {
                macro = "NUT_WDT_ENABLE",
                brief = "Enable watchdog",
                description = "If enabled, the watchdog timer will be started during system initialization.\n\n"..
                              "Once the watchdog is configured, it is not possible to re-configure the watchdog.\n"..
                              "If enabled, NutWatchDogRestart() functiom must be called periodically from application.",
                flavor = "booldata",
                file = "include/cfg/clock.h"
            }
        }
    },
}
