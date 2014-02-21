nutarch_m68k_coldfire =
{
    --
    -- MCU Family
    --
    {
        name = "nutarch_m68k_coldfire_family",
        brief = "MCU Family",
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
                description = "This option enables to use of 'slow heap' for thread stack allocation in case there is not enough memory in the 'fast heap'.\n\n",
                requires = { "NUTMEM_STACKHEAP" },
                flavor = "booldata",
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTMEM_STACKHEAP_LIMIT",
                brief = "Fast heap stack limit",
                description = "This option disables to use whole 'fast heap' for stack.\n"..
                              "This migh be useful if there are other modules which also uses 'fast heap'.\n\n"..
                              "Set this number to amount of memory you want to protect. If there will "..
                              "be less than 'Fast heap stack limit' free bytes in the 'fast heap', "..
                              "then the allocation fails, or 'slow heap' will be used if it is enabled.",  
                requires = { "NUTMEM_STACKHEAP" },
                flavor = "integer",
                file = "include/cfg/memory.h"
            },
        }
    },
    
    --
    -- Coldfire CORE Directory
    --
    {
        name = "nutarch_m68k_coldfire_mcf51",
        brief = "Coldfire V1 Core",
        requires = { "HW_MCU_COLDFIRE_V1" },
        description = "Coldfire V1 Core",
        script = "arch/coldfire/mcf51.nut"
    },
    {
        name = "nutarch_m68k_coldfire_mcf52",
        brief = "Coldfire V2 Core",
        requires = { "HW_MCU_COLDFIRE_V2" },
        description = "Coldfire V2 Core",
        script = "arch/coldfire/mcf52.nut"
    },

    --
    -- Coldfire MCU Directory
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn",
        brief = "MCF51CN Family",
        requires = { "HW_MCU_MCF51CN" },
        description = "MCF51CN family",
        script = "arch/coldfire/mcf51cn.nut"
    },
	{
        name = "nutarch_m68k_coldfire_mcf5225",
        brief = "MCF5225 Family",
        requires = { "HW_MCU_MCF5225" },
        description = "MCF5225 family",
        script = "arch/coldfire/mcf5225.nut"
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
