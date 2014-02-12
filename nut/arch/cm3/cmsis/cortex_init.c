/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2012 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*
 * \verbatim
 * $Id: cortex_init.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/memory.h>

#include <arch/cm3.h>
#include <dev/rtc.h>
#include <sys/heap.h>
#include <sys/nutdebug.h>
#include <sys/atom.h>

#ifndef MSP_STACK_SIZE
#define MSP_STACK_SIZE 128
#endif

#ifndef PSP_STACK_SIZE
#define PSP_STACK_SIZE 32
#endif

#ifndef NUT_BOOT_FUNCTION
#define NUT_BOOT_FUNCTION NutInit
#endif

extern void NUT_BOOT_FUNCTION(void *);

/*!
 * \brief CortexM3 MSP processor stack.
 */
__attribute__ ((section(".stack")))
volatile static uint32_t mspStack[MSP_STACK_SIZE];

/*!
 * \brief CortexM3 PSP processor stack.
 */
__attribute__ ((section(".psp_stack")))
volatile static uint32_t pspStack[PSP_STACK_SIZE];


static void IntDefaultHandler(void *arg) __attribute__ ((naked));
static void IntNmiHandler(void *arg) __attribute__ ((naked));
static void IntHardfaultHandler(void *arg) __attribute__ ((naked));
static void IntMemfaultHandler(void *arg) __attribute__ ((naked));
static void IntBusfaultHandler(void *arg) __attribute__ ((naked));
static void IntUsagefaultHandler(void *arg) __attribute__ ((naked));

/*!
 * \brief CortexM3 memory pointers
 *
 * These pointers are provided by the linker, controlled by the
 * linker script. The list is shrinked to the two basic
 * entries of the initial stack pointer address and the
 * reset vector. All other interrupt vectors are set up dynamically
 * later after system start.
 */
__attribute__ ((section(".isr_vector")))
#if defined(NUTDEBUG_RAM)
void (* g_pfnVectors[NUM_INTERRUPTS])(void*) =
#else
void (* const g_pfnVectors[])(void *) =
#endif
{
    (void (*)(void *))((uint32_t)mspStack + sizeof(mspStack)), /* Initial Stack Pointer */
    NUT_BOOT_FUNCTION,   /* Reset_Handler */
    IntNmiHandler,       /* NMI_Handler */
    IntHardfaultHandler, /* HardFault_Handler */
    IntMemfaultHandler,  /* MemManage_Handler */
    IntBusfaultHandler,  /* BusFault_Handler */
    IntUsagefaultHandler,/* UsageFault_Handler */
    0,                   /* Reserved */
    0,                   /* Reserved */
    0,                   /* Reserved */
    0,                   /* Reserved */
};


/*!
 * \brief Dynamic interrupt vector table in RAM
 *
 * Copy of the interrupt vector table in RAM. The vector table layout is
 * defined by the hardware. The list CM3 CPUs can handle a maximum of 256
 * interrupt vectors. The maximum number of handlers used by a specific CPU
 * implementation is defined in NUM_INTERRUPTS which should be defined in each
 * CPU header as the last entry in IRQn_Type enum.
 */

#if defined(NUTDEBUG_RAM)
/* Code is just running in RAM, so re-use the g_pfnVectors */
#define g_pfnRAMVectors g_pfnVectors
#else
static __attribute__((section(".vtable")))
void (*g_pfnRAMVectors[NUM_INTERRUPTS])(void*);
#endif


/*!
 * \brief CortexM3 memory pointers
 *
 * These pointers are provided by the linker, controlled by the
 * linker script.
 */
extern void * _etext;           /* Start of constants in FLASH */
extern void * _sidata;          /* Start of variables in FLASH */
extern void * _sdata;           /* Start of variables in RAM */
extern void * _edata;           /* End of variables in RAM */
extern void * _sbss;            /* Start of unset variables in RAM */
extern void * _ebss;            /* End of unset variables in RAM */

/* Remember: following stack grow from end address to start */
extern void * _pspstack_start;  /* Process stack start address */
extern void * _pspstack_end;    /* Process stack end address */
extern void * _stack_start;     /* Main stack start address */
extern void * _stack_end;       /* Main stack end address */

/* Default interrupt handler */
static void IntDefaultHandler(void *arg)
{
    for (;;);
}

/* Below the default exception handler follow */

/*!
 * \brief Non mascable interrupt handler
 */
static void IntNmiHandler(void *arg)
{
    for (;;);
}

/*!
 * \brief Hard fault handler
 */
static void IntHardfaultHandler(void *arg)
{
    for (;;);
}

/*!
 * \brief Mem fault handler
 */
static void IntMemfaultHandler(void *arg)
{
    for (;;);
}

/*!
 * \brief Bus fault handler
 */
static void IntBusfaultHandler(void *arg)
{
    for (;;);
}

/*!
 * \brief Usage fault handler
 */
static void IntUsagefaultHandler(void *arg)
{
    for (;;);
}


/*!
 * \brief Register interrupt handler in RAM vector table
 *
 * \param int_id    Specifies interrupt ID to register
 *
 * \param pfnHandler Interrupt handler function to be called
 */
void Cortex_RegisterInt(IRQn_Type int_id, void (*pfnHandler)(void*))
{
    uint16_t idx = int_id + 16;

    /* Check for valid interrupt number */
    NUTASSERT(idx < NUM_INTERRUPTS);

    /* Make sure that the RAM vector table is correctly aligned. */
    NUTASSERT(((uint32_t)g_pfnRAMVectors & 0x000003ff) == 0);

    if (pfnHandler != NULL) {
        /* Save the interrupt handler. */
        g_pfnRAMVectors[idx] = pfnHandler;
    } else {
        /* Reset to default interrupt handler */
        g_pfnRAMVectors[idx] = &IntDefaultHandler;
    }
}


/*!
 * \brief CortexM3 Initialization.
 *
 * This function copies over the data segment from flash to ram
 * and fills the bss segment with 0.
 */
static void Cortex_MemInit(void)
{
    register uint32_t *src, *dst, *end;
    register uint32_t fill = 0;

    /*
     * Plain C-Code Cortex Init
     */

    /* Copy the data segment initializers from flash to SRAM. */
    src = (uint32_t*)&_etext;
    end = (uint32_t*)&_edata;
    for( dst = (uint32_t*)&_sdata; dst < end;)
    {
        *dst++ = *src++;
    }

    /* Fill the bss segment with 0x00 */
    end = (uint32_t*)&_ebss;
    for( dst = (uint32_t*)&_sbss; dst < end; )
    {
        *dst++ = fill;
    }

    __set_PSP((uint32_t)&_pspstack_end);
}

static void Cortex_IntInit(void)
{
    int int_id;

    /* Disable exceptions */
    SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk |
                    SCB_SHCSR_BUSFAULTENA_Msk |
                    SCB_SHCSR_MEMFAULTENA_Msk);

    /* Disable SysTick interrupt */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

#ifndef NUTDEBUG_RAM
    /* Copy Reset vector to RAM vector table */
    g_pfnRAMVectors[0] = (void(*)(void*))g_pfnVectors[0];

    /* Copy Stackpointer to RAM vector table */
    g_pfnRAMVectors[1] = (void(*)(void*))g_pfnVectors[1];
#endif

    /* Set the exception handler */
    g_pfnRAMVectors[2] = &IntNmiHandler;
    g_pfnRAMVectors[3] = &IntHardfaultHandler;
    g_pfnRAMVectors[4] = &IntMemfaultHandler;
    g_pfnRAMVectors[5] = &IntBusfaultHandler;
    g_pfnRAMVectors[6] = &IntUsagefaultHandler;

    /* Init reserved vectors with NULL */
    g_pfnRAMVectors[7]  = NULL;
    g_pfnRAMVectors[8]  = NULL;
    g_pfnRAMVectors[9]  = NULL;
    g_pfnRAMVectors[10] = NULL;
    g_pfnRAMVectors[12] = NULL;
    g_pfnRAMVectors[13] = NULL;

    /* Init all other exception handler and system interrupts with the
       default handler
     */
    g_pfnRAMVectors[10] = &IntDefaultHandler;
    g_pfnRAMVectors[11] = &IntDefaultHandler;
    g_pfnRAMVectors[14] = &IntDefaultHandler;
    g_pfnRAMVectors[15] = &IntDefaultHandler;

    /* Enable exceptions again. The NMI and Hard-Fault handler are always enabled */
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_MEMFAULTENA_Msk;

    for (int_id = 0; int_id < NUM_INTERRUPTS - 16; int_id ++) {
        /* Make sure the interrupt is disabled */
        NVIC_DisableIRQ(int_id);
        g_pfnRAMVectors[int_id] = &IntDefaultHandler;
    }

    /* Basic interrupt system setup */
    //SCB->AIRCR = (0x05fa0000|SCB_AIRCR_PRIGROUP0);
    /* Clear pending bits for SysTick interrupt, PendSV exception and ISRs */
    SCB->ICSR |= (SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_ISRPENDING_Msk);

    /* Point NVIC at the RAM vector table. */
    SCB->VTOR = (uint32_t)g_pfnRAMVectors;
    __enable_fault_irq();
    __enable_irq();
}


/*!
 * \brief CortexM3 Startup.
 *
 * This function runs the basic cpu initialization.
 */
void Cortex_Start(void)
{
    Cortex_MemInit();
    Cortex_IntInit();
}
