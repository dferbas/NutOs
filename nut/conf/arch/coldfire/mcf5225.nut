            
mcf5225_i2c0_scl_pins = { "PAS0", "PQS2" }
mcf5225_i2c0_sda_pins = { "PAS1", "PQS3" }

mcf5225_i2c0_scl_pins_descr = "Choose I2C 0 SCL pin:\n\tPAS0 (I2C_SCL0)\n\tPQS2 (QSPI_CLK)"
mcf5225_i2c0_sda_pins_descr = "Choose I2C 0 SDA pin:\n\tPAS1 (I2C_SDA0)\n\tPQS3 (QSPI_CS0)"

mcf5225_i2c1_scl_pins = { "PQS0", "PUB0", "PUC3", "PTH3" }
mcf5225_i2c1_sda_pins = { "PQS1", "PUB1", "PUC2", "PTH2" }

mcf5225_i2c1_scl_pins_descr = "Choose I2C 1 SCL pin:\n\tPQS0 (QSPI_DOUT)\n\tPUB0 (UTXD1)\n\tPUC3 (UCTS2)\n\tPTH3 (FB_D5)"
mcf5225_i2c1_sda_pins_descr = "Choose I2C 1 SDL pin:\n\tPQS1 (QSPI_IN)  \n\tPUB1 (URXD1)\n\tPUC2 (URTS2)\n\tPTH2 (FB_D4)"

mcf5225_uart0_txd_pins = { "PUA0" }
mcf5225_uart0_rxd_pins = { "PUA1" }
mcf5225_uart0_rts_pins = { "PUA2" }
mcf5225_uart0_cts_pins = { "PUA3" }
mcf5225_uart0_txd_pins_descr = "Choose UART0 TXD Pin:\n\tPUA0 (UTXD0)"
mcf5225_uart0_rxd_pins_descr = "Choose UART0 RXD Pin:\n\tPUA1 (URXD0)"
mcf5225_uart0_rts_pins_descr = "Choose UART0 RTS Pin:\n\tPUA2 (URTS0)"
mcf5225_uart0_cts_pins_descr = "Choose UART0 CTS Pin:\n\tPUA3 (UCTS0)"

mcf5225_uart1_txd_pins = { "PUB0", "PQS0" }
mcf5225_uart1_rxd_pins = { "PUB1", "PQS1" }
mcf5225_uart1_rts_pins = { "PUB2", "PQS2" }
mcf5225_uart1_cts_pins = { "PUB3", "PQS3" }
mcf5225_uart1_txd_pins_descr = "Choose UART1 TXD Pin:\n\tPUB0 (UTXD1)\n\tPQS0 (QSPI_DOUT)"
mcf5225_uart1_rxd_pins_descr = "Choose UART1 RXD Pin:\n\tPUB1 (URXD1)\n\tPQS1 (QSPI_DIN)"
mcf5225_uart1_rts_pins_descr = "Choose UART1 RTS Pin:\n\tPUB2 (URTS1)\n\tPQS2 (QSPI_CLK)"
mcf5225_uart1_cts_pins_descr = "Choose UART1 CTS Pin:\n\tPUB3 (UCTS1)\n\tPQS3 (QSPI_CS0)"

mcf5225_uart2_txd_pins = { "PUC0", "PUB2", "PAS0" }
mcf5225_uart2_rxd_pins = { "PUC1", "PUB3", "PAS1" }
mcf5225_uart2_rts_pins = { "PUC2" }
mcf5225_uart2_cts_pins = { "PUC3" }
mcf5225_uart2_txd_pins_descr = "Choose UART2 TXD Pin:\n\tPUC0 (UTXD2)\n\tPUB2 (URTS1)\n\tPAS0 (I2C_SCL0)"
mcf5225_uart2_rxd_pins_descr = "Choose UART2 RXD Pin:\n\tPUC1 (URXD2)\n\tPUB3 (UCTS1)\n\tPAS1 (I2C_SDA0)"
mcf5225_uart2_rts_pins_descr = "Choose UART2 RTS Pin:\n\tPUC2 (URTS2)"
mcf5225_uart2_cts_pins_descr = "Choose UART2 CTS Pin:\n\tPUC3 (UCTS2)"

nutarch_m68k_coldfire_mcf5225 =
{
   	--
    -- MCU Family
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_family",
        brief = "MCU Family",
        provides = {
                "HW_I2C_COLDFIRE",
                "HW_UART_COLDFIRE",
        },
        options =
        {
            {
                macro = "MCU_MCF5225",
                brief = "MCF5225",
                description = "MCF5225 Coldfire Family",
                type = "integer",
                default = 1,
                file = "include/cfg/arch.h"
            },
            {
                macro = "UART0",
                type = "integer",
                default = 1,
                provides = { "HW_UART0" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "UART1",
                type = "integer",
                default = 1,
                provides = { "HW_UART1" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "UART2",
                type = "integer",
                default = 1,
                provides = { "HW_UART2" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "I2C0",
                type = "integer",
                default = 1,
                provides = { "HW_I2C0" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "I2C1",
                type = "integer",
                default = 1,
                provides = { "HW_I2C1" },
                file = "include/cfg/peripherals.h"
            },
        }
    },
    
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
                    "m68k/coldfire/init/crt_mcf5225.S",
                    "m68k/coldfire/init/crt_mcf5225_c.c",
                  },
        targets = { 
                    "m68k/coldfire/init/crt_$(LDNAME).o",
                    "m68k/coldfire/init/crt_common.o",
                    "m68k/coldfire/init/crt_common_c.o",
                    "m68k/coldfire/init/crt_mcf5225.o", 
                    "m68k/coldfire/init/crt_mcf5225_c.o", 
                  },
        requires = { "TOOL_CC_M68K", "TOOL_GCC"},
    },    

    --
    -- GPIO Interface
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_gpio",
        brief = "GPIO",
        description = "Generic port I/O API.",
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_gpio.c"}
    },
    
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_ihndlr",
        brief = "Interrupt Handler",
        description = "Peripheral interrupt handlers for MCF5225 family.",
        provides = { 
                    "DEV_IRQ_I2C",
                    "DEV_IRQ_UART",
                    "DEV_IRQ_QSPI",
                   },
        sources = { 
                    "m68k/coldfire/dev/mcf5225/ih_mcf5225_pit.c",
                    "m68k/coldfire/dev/mcf5225/ih_mcf5225_cwt.c",
                    "m68k/coldfire/dev/mcf5225/ih_mcf5225_i2c.c",
                    "m68k/coldfire/dev/mcf5225/ih_mcf5225_uart.c", 
                    "m68k/coldfire/dev/mcf5225/ih_mcf5225_gpt.c",
                    "m68k/coldfire/dev/mcf5225/ih_mcf5225_qspi.c",
                  },
    },

    --
    -- System Timer Hardware
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_ostimer",
        brief = "System Timer",
        provides = { "NUT_OSTIMER_DEV" },
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_ostimer.c" },
    },
    
    --
    -- Reset Controller
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_reset",
        brief = "Reset Controller",
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_reset.c" },
    },
    
    --
    -- Core Watchdog Timer
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_cwt",
        brief = "Core Watchdog Timer (CWT)",
        description = "Core watchdog timer for MCF5225 family.",
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_cwt.c" }
    },

    --
    -- Old Usart0 Interface
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_old_uart0",
        brief = "UART0 Driver Old",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.\n\n"..
                      "DO NOT USE THIS DRIVER. It is used only for SolarMonitor and Posedon products and all GPIO ping are hard wired",
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_old_uart0.c" }
    },
    
    --
    -- Old Usart1 Interface
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_old_uart1",
        brief = "UART1 Driver Old",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.\n\n"..
                      "DO NOT USE THIS DRIVER. It is used only for SolarMonitor and Posedon products and all GPIO ping are hard wired",
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_old_uart1.c" }
    },
    
    --
    -- Old Usart2 Interface
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_old_uart2",
        brief = "UART2 Driver Old",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.\n\n"..
                      "DO NOT USE THIS DRIVER. It is used only for SolarMonitor and Posedon products and all GPIO ping are hard wired",
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_old_uart2.c" }
    },
    
    --
    -- General Purpose Timer / Pulse Accumulator
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_gpt",
        brief = "Timer / Pulse Accumulator (GPT)",
        description = "General Purpose Timer / Pulse Accumulator\n\n"..
                      "Not fully implemented. The module contains only several functions which controls Pulse Acumulator.",
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_gpt.c" }
    }, 
    
    --
    -- Real Time Clock
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225_rtc",
        brief = "Real Time Clock (RTC)",
        description = "Mcf5225 RTC driver.",
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_rtc.c" }
    },

    --
    -- QSPI
    --
    {
        name = "nutarch_mcf5225_qspi",
        brief = "QSPI Driver",
        description = "Hardware specific QSPI driver. Implements hardware ",
        requires = {"HW_MCU_M68K", "NUT_OSTIMER_DEV", "NUT_EVENT", "DEV_IRQ_QSPI" },
        provides = {"DEV_SPI" },
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_qspi.c" }
    },
    
    --
    -- I2c
    --
    {
        name = "nutarch_mcf5225_old_i2c",
        brief = "I2C (MCF5225)",
        description = "Old I2C for MCF5225 family.",
        requires = {"HW_I2C_COLDFIRE", "DEV_IRQ_I2C", "HW_I2C0" },
        provides = { "DEV_TWI" },
        sources = { "m68k/coldfire/dev/mcf5225/mcf5225_old_i2c.c" }
    },

}
