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
        brief = "Interrupt Handlers",
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
        description = "Code snippet for modifying internal flash memory.",
        sources = { "m68k/coldfire/dev/mcf51/mcf51_intflash.c" },
    },

    --
    -- SCI Debug Output
    --
    {
        name = "nutarch_m68k_coldfire_devices_sci_debug",
        brief = "SCI Debug Output",
        description = "In current implementation the SCI (Serial Communication Interface) is used as an UART device only.\n\n"..
                      "This simple SCI output driver uses polling instead of interrupts "..
                      "and can be used within interrupt routines.\n"..
                      "It is mainly used for debugging and tracing.\n\n"..
                      "This driver uses only RXD and TXD pins which are configured in SCIn Drivers below.\n\n"..
                      "Call one of following functions:\n"..
                      "\tNutRegisterDevice(&devDebug1, 0, 0) for SCI1\n"..
                      "\tNutRegisterDevice(&devDebug2, 0, 0) for SCI2\n"..
                      "\t...\n"..
                      "\tNutRegisterDevice(&devDebugn, 0, 0) for SCIn\n\n"..
                      "Then you can use any of the stdio functions to open device sci0, sci1, ..., scin.\n".. 
                      "",
        requires = { "HW_SCI_COLDFIRE" },
        provides = { "DEV_UART", "DEV_FILE", "DEV_WRITE" },
        sources = { "m68k/coldfire/dev/mcf51/mcf5_sci_debug.c"},
    },

    --
    -- SCI1 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_sci1",
        brief = "SCI1 Driver",
        description = "In current implementation the SCI (Serial Communication Interface) is used as an UART device only.\n\n"..
                      "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic USART driver framework.",
        requires = { "HW_SCI_COLDFIRE", "DEV_IRQ_SCI", "HW_SCI1" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/mcf51/mcf5_sci1.c" },
        options =
        {
            {
                macro = "SCI1_TXD_PORTPIN",
                brief = "SCI1 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci1_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci1_txd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
            {
                macro = "SCI1_RXD_PORTPIN",
                brief = "SCI1 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci1_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci1_rxd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
        },
    },
        
    --
    -- SCI2 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_sci2",
        brief = "SCI2 Driver",
        description = "In current implementation the SCI (Serial Communication Interface) is used as an UART device only.\n\n"..
                      "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic USART driver framework.",
        requires = { "HW_SCI_COLDFIRE", "DEV_IRQ_SCI", "HW_SCI2" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/mcf51/mcf5_sci2.c" },
        options =
        {
            {
                macro = "SCI2_TXD_PORTPIN",
                brief = "SCI2 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci2_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci2_txd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
            {
                macro = "SCI2_RXD_PORTPIN",
                brief = "SCI2 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci2_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci2_rxd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
        },
    },
        
    --
    -- SCI3 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_sci3",
        brief = "SCI3 Driver",
        description = "In current implementation the SCI (Serial Communication Interface) is used as an UART device only.\n\n"..
                      "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic USART driver framework.",
        requires = { "HW_SCI_COLDFIRE", "DEV_IRQ_SCI", "HW_SCI3" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/mcf51/mcf5_sci3.c" },
        options =
        {
            {
                macro = "SCI3_TXD_PORTPIN",
                brief = "SCI3 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci3_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci3_txd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
            {
                macro = "SCI3_RXD_PORTPIN",
                brief = "SCI3 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci3_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci3_rxd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
        },
    },
        
    --
    -- IIC - Inter-Integrated Circuit
    --
    {
        name = "nutarch__m68k_coldfire_mcf51_iic",
        brief = "Inter-Integrated Circuit (IIC)",
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
        brief = "Serial Peripheral Interface (SPI)",
        description = "Serial Peripheral Interface\n\n"..
                      "Temporarily fixed to MCF51QE - AD_PC board only",
        requires = { "SPI_BUS_DRIVER_AD_PC_ONLY", "DEV_IRQ_SPI"},
        provides = { "SPIBUS_CONTROLLER" },
        sources =
        {
            "m68k/coldfire/dev/mcf51/mcf51_spi1.c",
            "m68k/coldfire/dev/mcf51/mcf51_spi2.c",
        },
    },    
}
