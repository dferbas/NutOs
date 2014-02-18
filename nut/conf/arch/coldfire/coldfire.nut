nutarch_m68k_coldfire =
{
    --
    -- MCU Family
    --
    {
        name = "nutarch_m68k_coldfire_family",
        brief = "Family",
        options =
        {
            {
                macro = "MCU_COLDFIRE",
                brief = "COLDFIRE",
                type = "integer",
                default = 1,
                file = "include/cfg/arch.h"
            }
        }
    },
    
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_ihndlr",
        brief = "Interrupt Handler",
        description = "CPU Interrupt handlers common for all Coldfire families.\n\n"..
                      "In addition, this package contains the default interrupt handler, which may be used by application to handle spurious interrupts.",
        sources = { "m68k/coldfire/dev/common/ih_mcf5_default.c" }
    },

    --
    -- Internal Memory
    --
    {
        name = "nutarch_m68k_coldfire_int_mem",
        brief = "Internal Memory",
        description = "Internal memory (RAM)",
        options =
        {
            {
                macro = "NUTMEM_SPLIT_FAST",
                brief = "Use internal RAM as 'fast heap'",
                description = "If this option is enabled, free internal memory is used as a heap.\n\n"..
                              "Some devices (e.g. FEC) are able to work only with internal memory.\n"..
                              "They might use NutHeapFastMemAlloc() for allocating this memory.",
                provides = { "NUTMEM_SPLIT_FAST" },
                flavor = "booldata",
                file = "include/cfg/memory.h"
            },
        }        
    },

    --
    -- Context Switching
    --
    {
        name = "nutarch_m68k_coldfire_context",
        brief = "Context Switching",
        description = "Context Switching (GCC)",
        provides = { "NUT_CONTEXT_SWITCH" },
        requires = { "TOOL_GCC" },
        sources = { "m68k/coldfire/os/context.c" },
        options =
        {
            {
                macro = "NUTMEM_STACKHEAP",
                brief = "Stack in 'fast heap'",
                description = "This option enables use of a 'fast heap' for stack.\n\n"..
                              "When a thread is created with this option enabled, "..
                              "it's stack is allocated on a 'fast heap' which is kept "..
                              "in faster internal memory \ninstead of using the 'standard heap' ".. 
                              "which is typically located in slower external memory.",
                requires = { "NUTMEM_SPLIT_FAST" },
                provides = { "NUTMEM_STACKHEAP" },
                flavor = "booldata",
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTMEM_STACKHEAP_ALLOW_EXTRAM",
                brief = "Stack in 'slow heap'",
                description = "This option enables use of 'slow heap' located in external "..
                              "memory for stack after the internal 'fast heap' is exhousted.\n\n",
                requires = { "NUTMEM_STACKHEAP" },
                flavor = "booldata",
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTMEM_STACKHEAP_LIMIT",
                brief = "Fast heap stack limit",
                description = "This option disables to use all the 'fast heap' for stack.\n"..
                              "It is usefull if there are other modules which also uses 'fast heap'\n\n."..
                              "Set this number to amount of memory you want to protect. If there will "..
                              "be less than 'Fast heap stack limit' free bytes in the 'fast heap', "..
                              "then the allocation fails, or 'slow heap' will be used.",  
                requires = { "NUTMEM_STACKHEAP" },
                flavor = "integer",
                file = "include/cfg/memory.h"
            },
        }
    },
    
    --
    -- Coldfire MCU Directory
    --
	{
        name = "nutarch_m68k_coldfire_mcf5225x",
        brief = "MCF5225X Family",
        requires = { "HW_MCU_MCF5225X" },
        description = "MCF5225X family",
        script = "arch/coldfire/mcf5225x.nut"
    },
	{
        name = "nutarch_m68k_coldfire_mcf51cn",
        brief = "MCF51CN Family",
        requires = { "HW_MCU_MCF51CN" },
        description = "MCF51CN family",
        script = "arch/coldfire/mcf51cn.nut"
    },
    
    --
    -- Coldfire Common Devices
    --
    {
        name = "nutarch_m68k_coldfire_devices",
        brief = "Common devices",
        description = "Common devices to the Cipfire families",
        script = "arch/coldfire/coldfire_dev.nut",
    },
}
