nutarch_cm3_lpc17xx_devices =
{
    -- ***********************************
    --
    -- LPC17xx Device Drivers
    --
    -- ***********************************

    {
        name = "nutarch_cm3_lpc1768_debug",
        brief = "UART Debug Output (LPC1768)",
        description = "Polling UART driver\n"..
                      "The debug output entry above uses the interrupt driven UART "..
                      "driver, which cannot be used in interrupt context."..
                      "But that's exactly what I need right now.\n\n"..
                      "I've currently no idea, which CPU variants this polling "..
                      "driver may support. For the time being this is limited "..
                      "to the LPC1768.",
        requires = { "HW_MCU_LPC1768" },
        provides = { "DEV_UART", "DEV_FILE", "DEV_WRITE" },
        sources = { "cm3/dev/nxp/lpc_debug.c", "cm3/dev/nxp/lpc176x_debug0.c" },
    },

    --
    -- LPC17xx RTC
    --
    {
        name = "nutarch_cm3_lpc17xx_rtc",
        brief = "LPC17xx RTC Driver",
        description = "LPC17xx RTC driver.",
        requires = { "HW_RTC_LPC17xx" },
        provides = { "DEV_RTC" },
        sources = { "cm3/dev/nxp/lpc17xx_rtc.c", "cm3/dev/nxp/ih_lpc17xx_rtc.c" },
    },
    --
    -- LPC17xx Watchdog Timer
    --
    {
        name = "nutarch_cm3_lpc17xx_wdt",
        brief = "LPC17xx Watchdog Timer",
        requires = { "HW_WDT_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_wdt.c", "cm3/dev/nxp/ih_lpc17xx_wdt.c"}
    },
    --
    -- LPC17xx Flash Memory Controller
    --
    {
        name = "nutarch_cm3_lpc17xx_iap",
        brief = "LPC17xx In Application Flash Programming API (IAP)",
        description = "Routines for setup and programming LPC17x series internal FLASH.\n",
        requires = { "HW_FLASH_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_iap.c" }
    },
    --
    -- LPC17xx General purpose DMA Controller
    --
    {
        name = "nutarch_cm3_lpc17xx_gpdma",
        brief = "LPC17xx General purpose DMA Controller API",
        description = "Routines for setup and programming LPC17x series GPDMA controller.\n",
        requires = { "HW_GPDMA_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_gpdma.c" }
    },
    --
    -- LPC17xx General purpose DMA Controller
    --
    {
        name = "nutarch_cm3_lpc17xx_emac",
        brief = "LPC17xx EMAC Driver",
        description = "LAN driver for LPC176x, LPC177x_8x etc.",
        requires = { "HW_EMAC_LPC17xx", "NUT_EVENT", "NUT_TIMER" },
        provides = { "NET_MAC" },
        sources = { "cm3/dev/nxp/lpc17xx_emac.c", "cm3/dev/nxp/ih_lpc17xx_emac.c" },
        options =
        {
            {
                macro = "NUT_THREAD_NICRXSTACK",
                brief = "Receiver Thread Stack",
                description = "Number of bytes to be allocated for the stack of the NIC receive thread.",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_RX_BUFFERS",
                brief = "Receive Buffers",
                description = "Number of 128 byte receive buffers.\n"..
                              "Increase to handle high traffic situations.\n"..
                              "Decrease to handle low memory situations.\n"..
                              "Default is 32.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_TX_BUFSIZ",
                brief = "Transmit Buffer Size",
                description = "The driver will allocate two transmit buffers.\n"..
                              "Can be decreased in low memory situations. Be aware, "..
                              "that this may break your network application. Do not "..
                              "change this without exactly knowing the consequences.\n"..
                              "Default is 1536.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_LINK_LOOPS",
                brief = "Link Polling Loops",
                description = "This simple implementation runs a dumb polling loop "..
                              "while waiting for the Ethernet link status.\n"..
                              "If you experience link problems, increasing this value "..
                              "may help.\n"..
                              "Default is 10000.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
        }
    }
}

