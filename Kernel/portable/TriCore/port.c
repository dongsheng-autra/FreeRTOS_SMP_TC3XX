/*
 * Copyright (c) 2023 Autra.Tech. All rights reserved.
 *
 * THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
 * ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
 * IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
 * PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
 */

/* Std includes */
#include <string.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* IFX include */
#include <Src/Std/IfxSrc.h>
#include <Stm/Timer/IfxStm_Timer.h>

#include "UART_VCOM.h"

/* Prgoram status word macros */
#define portINITIAL_SYSTEM_PSW          ( 0x000008FFUL ) /* Supervisor Mode, MPU Register Set 0 and Call Depth Counting disabled. */
#define portINITIAL_PRIVILEGED_PSW      ( 0x000014FFUL ) /* IO Level 1, MPU Register Set 1 and Call Depth Counting disabled. */
#define portINITIAL_UNPRIVILEGED_PSW    ( 0x000010FFUL ) /* IO Level 0, MPU Register Set 1 and Call Depth Counting disabled. */
#define portRESTORE_PSW_MASK            ( ~( 0x000000FFUL ) )

/* Context save area macros */
#define portCSA_TO_ADDRESS( pCSA )                            \
    ( ( unsigned long * ) ( ( ( pCSA & 0x000F0000 ) << 12 ) | \
                            ( ( pCSA & 0x0000FFFF ) << 6 ) ) )
#define portADDRESS_TO_CSA( pAddress )                                            \
    ( ( unsigned long ) ( ( ( ( unsigned long ) pAddress & 0xF0000000 ) >> 12 ) | \
                          ( ( ( unsigned long ) pAddress & 0x003FFFC0 ) >> 6 ) ) )
#define portCSA_FCX_MASK        ( 0x000FFFFFUL )
#define portINITIAL_LOWER_PCXI  ( 0x00300000UL ) /* Set UL to upper and PIE to 1 */
#define portINITIAL_UPPER_PCXI  ( 0x00200000UL ) /* Set UL to lower and PIE to 1 */
#define portNUM_WORDS_IN_CSA    ( 16 )

/* This reference is required by the save/restore context macros. */
extern volatile unsigned long *pxCurrentTCBs[];

static IfxStm_Timer g_timer[configNUM_CORES];
static Ifx_STM *g_system_timer[configNUM_CORES] = {&MODULE_STM0, &MODULE_STM1};
volatile Ifx_SRC_SRCR *g_GPSR[configNUM_CORES] = {&SRC_GPSR00, &SRC_GPSR10};
volatile unsigned port_xSchedulerRunning[configNUM_CORES] = {0};

#define configSTM_CLOCK_HZ		(configCPU_CLOCK_HZ/3)

#define portTICK_COUNT    ( configSTM_CLOCK_HZ / configTICK_RATE_HZ )

#define configSYSTEM_INTERRUPT_PRIORITY   (configKERNEL_INTERRUPT_PRIORITY + configNUM_CORES)
static void vPortInitTickTimer()
{
	IfxStm_Timer_Config timerConfig;
    int cpu_id = portGET_CORE_ID();
    console_printf("Current CPU is %d\n", cpu_id);

	IfxStm_Timer_initConfig(&timerConfig, g_system_timer[cpu_id]);
	timerConfig.base.isrPriority = configSYSTEM_INTERRUPT_PRIORITY + cpu_id;
	timerConfig.base.frequency = configTICK_RATE_HZ;
    timerConfig.base.isrProvider = IfxCpu_Irq_getTos(cpu_id);

	IfxStm_Timer_init(&g_timer[cpu_id], &timerConfig);
	IfxStm_Timer_run(&g_timer[cpu_id]);
}

static void vPortInitContextSrc()
{
    int cpu_id = portGET_CORE_ID();
    IfxSrc_init( g_GPSR[cpu_id], IfxCpu_Irq_getTos(cpu_id), configKERNEL_INTERRUPT_PRIORITY + cpu_id);
    IfxSrc_enable( g_GPSR[cpu_id] );
}

void vPortYield()
{
    int cpu_id = portGET_CORE_ID();
    console_printf("Yield CPU is %d\n", cpu_id);
    if( IfxSrc_isRequested( g_GPSR[cpu_id] ) != TRUE )
    {
        IfxSrc_setRequest( g_GPSR[cpu_id] );
    }

    __dsync();
    __isync();
}

void vPortYieldCore(BaseType_t xCoreID)
{
    int cpu_id = portGET_CORE_ID();
    if (cpu_id == 1) {
        console_printf("Yield Core CPU is %d\n", cpu_id);
    }
}

#define portINITIAL_SYSCON (0x00000000UL)							/* MPU Disable. */

static void vPortStartFirstTask()
{
    unsigned long ulMFCR = 0UL;
    unsigned long ** ppxTopOfStack;
    unsigned long uxLowerCSA;

    int cpu_id = portGET_CORE_ID();
    console_printf("vPortStartFirstTask CPU is %d, 0x%x\n", cpu_id, pxCurrentTCBs[cpu_id]);

    __disable();

    /* Load the initial SYSCON. */
    __mtcr(CPU_SYSCON, portINITIAL_SYSCON);
    __isync();

	/* ENDINIT has already been applied in the 'cstart.c' code. */

	/* Clear the PSW.CDC to enable the use of an RFE without it generating an
	exception because this code is not genuinely in an exception. */
	ulMFCR = __mfcr(CPU_PSW);
	ulMFCR &= portRESTORE_PSW_MASK;
	__dsync();
	__mtcr(CPU_PSW, ulMFCR);
	__isync();

	/* Finally, perform the equivalent of a portRESTORE_CONTEXT() */
    /* Load the new CSA id from the stack and update the stack pointer */
    ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[cpu_id];
    uxLowerCSA = **ppxTopOfStack;
	( *ppxTopOfStack )++;
	__dsync();
	__mtcr(CPU_PCXI, uxLowerCSA);
	__isync();
	__nop();
	__rslcx();
	__nop();

	/* Return to the first task selected to execute. */
	__asm volatile("rfe");
}

StackType_t *pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                    TaskFunction_t pxCode,
                                    void * pvParameters )
{
    unsigned long * pulUpperCSA = NULL;
    unsigned long * pulLowerCSA = NULL;

    /* Have to disable interrupts here because the CSAs are going to be
     * manipulated. */
    __disable();
    {
        /* DSync to ensure that buffering is not a problem. */
        __dsync();

        /* Consume two free CSAs. */
        pulLowerCSA = portCSA_TO_ADDRESS( __mfcr( CPU_FCX ) );

        if( NULL != pulLowerCSA )
        {
            /* The Lower Links to the Upper. */
            pulUpperCSA = portCSA_TO_ADDRESS( pulLowerCSA[ 0 ] );
        }

        /* Check that we have successfully reserved two CSAs. */
        if( ( NULL != pulLowerCSA ) && ( NULL != pulUpperCSA ) )
        {
            /* Remove the two consumed CSAs from the free CSA list. */
            __mtcr( CPU_FCX, pulUpperCSA[ 0 ] );
            #ifndef __TASKING__
                __isync();
            #endif
        }
        else
        {
            /* Simply trigger a context list depletion trap. */
            __svlcx();
        }
    }
    __enable();

    /* Upper Context. */
    memset( pulUpperCSA, 0, portNUM_WORDS_IN_CSA * sizeof( unsigned long ) );
    pulUpperCSA[ 2 ] = ( unsigned long ) pxTopOfStack; /* A10;    Stack Return aka Stack Pointer */
    pulUpperCSA[ 1 ] = portINITIAL_SYSTEM_PSW;         /* PSW    */
    pulUpperCSA[ 0 ] = portINITIAL_UPPER_PCXI;

    /* Lower Context. */
    memset( pulLowerCSA, 0, portNUM_WORDS_IN_CSA * sizeof( unsigned long ) );
    pulLowerCSA[ 8 ] = ( unsigned long ) pvParameters; /* A4;    Address Type Parameter Register    */
    pulLowerCSA[ 1 ] = ( unsigned long ) pxCode;       /* A11;    Return Address aka RA */
    pulLowerCSA[ 0 ] = ( portINITIAL_LOWER_PCXI | ( unsigned long ) portADDRESS_TO_CSA( pulUpperCSA ) ); /* PCXI pointing to the Upper context. */

    /* Save the link to the CSA to the top of stack. */
    pxTopOfStack--;
    *pxTopOfStack = portADDRESS_TO_CSA( pulLowerCSA );

    return pxTopOfStack;
}

BaseType_t xPortStartScheduler( void )
{
    vPortInitTickTimer();
    vPortInitContextSrc();
    vPortStartFirstTask();

    return 0;
}

void vPortEndScheduler()
{
}

IFX_INTERRUPT( vPortSystemContextHandler, 0, configKERNEL_INTERRUPT_PRIORITY )
{
    unsigned long ** ppxTopOfStack;
    unsigned long uxLowerCSA;
    unsigned long * pxLowerCSA, * pxUpperCSA;

    __disable();
    {
        __dsync();
        /* Load the current top of stack pointer and csa info */
        ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[0];
        uxLowerCSA = __mfcr( CPU_PCXI );
        pxLowerCSA = portCSA_TO_ADDRESS( uxLowerCSA );
        pxUpperCSA = portCSA_TO_ADDRESS( pxLowerCSA[ 0 ] );
        /* Update the stack info in the TCB */
        *ppxTopOfStack = ( unsigned long * ) pxUpperCSA[ 2 ];
        /* Place the lower CSA id on the stack */
        ( *ppxTopOfStack )--;
        **ppxTopOfStack = uxLowerCSA;

        vTaskSwitchContext(0);

        /* Load the new CSA id from the new stack and update the stack pointer */
        ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[0];
        uxLowerCSA = **ppxTopOfStack;
        ( *ppxTopOfStack )++;
        /* Update the link register */
        __mtcr( CPU_PCXI, uxLowerCSA );
        __isync();
    }
    __enable();
}

IFX_INTERRUPT( vPortSystemContextHandlerCore, 1, configKERNEL_INTERRUPT_PRIORITY + 1 )
{
    unsigned long ** ppxTopOfStack;
    unsigned long uxLowerCSA;
    unsigned long * pxLowerCSA, * pxUpperCSA;

    console_printf("vPortSystemContextHandlerCore: 0x%x\n", pxCurrentTCBs[1]);

    __disable();
    {
        __dsync();
        /* Load the current top of stack pointer and csa info */
        ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[1];
        uxLowerCSA = __mfcr( CPU_PCXI );
        pxLowerCSA = portCSA_TO_ADDRESS( uxLowerCSA );
        pxUpperCSA = portCSA_TO_ADDRESS( pxLowerCSA[ 0 ] );
        /* Update the stack info in the TCB */
        *ppxTopOfStack = ( unsigned long * ) pxUpperCSA[ 2 ];
        /* Place the lower CSA id on the stack */
        ( *ppxTopOfStack )--;
        **ppxTopOfStack = uxLowerCSA;

        vTaskSwitchContext(1);

        /* Load the new CSA id from the new stack and update the stack pointer */
        ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[1];
        uxLowerCSA = **ppxTopOfStack;
        ( *ppxTopOfStack )++;
        /* Update the link register */
        __mtcr( CPU_PCXI, uxLowerCSA );
        __isync();
    }
    __enable();
}

IFX_INTERRUPT( vPortSystemTickHandler, 0, configSYSTEM_INTERRUPT_PRIORITY )
{
    unsigned long ulSavedInterruptMask;
    long lYieldRequired;

    IfxStm_Timer_acknowledgeTimerIrq(&g_timer[0]);

    /* Kernel API calls require Critical Sections. */
    ulSavedInterruptMask = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        /* Increment the Tick. */
        lYieldRequired = xTaskIncrementTick();
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulSavedInterruptMask );

    unsigned long ** ppxTopOfStack;
    unsigned long uxLowerCSA;
    unsigned long * pxLowerCSA, * pxUpperCSA;

    __disable();
    {
        __dsync();
        /* Load the current top of stack pointer and csa info */
        ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[0];
        uxLowerCSA = __mfcr( CPU_PCXI );
        pxLowerCSA = portCSA_TO_ADDRESS( uxLowerCSA );
        pxUpperCSA = portCSA_TO_ADDRESS( pxLowerCSA[ 0 ] );
        /* Update the stack info in the TCB */
        *ppxTopOfStack = ( unsigned long * ) pxUpperCSA[ 2 ];
        /* Place the lower CSA id on the stack */
        ( *ppxTopOfStack )--;
        **ppxTopOfStack = uxLowerCSA;

        vTaskSwitchContext(0);

        /* Load the new CSA id from the new stack and update the stack pointer */
        ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[0];
        uxLowerCSA = **ppxTopOfStack;
        ( *ppxTopOfStack )++;
        /* Update the link register */
        __mtcr( CPU_PCXI, uxLowerCSA );
        __isync();
    }
    __enable();
}

IFX_INTERRUPT( vPortSystemTickHandlerCore1, 1, configSYSTEM_INTERRUPT_PRIORITY + 1)
{
    IfxStm_Timer_acknowledgeTimerIrq(&g_timer[1]);

    unsigned long ** ppxTopOfStack;
    unsigned long uxLowerCSA;
    unsigned long * pxLowerCSA, * pxUpperCSA;

    __disable();
    {
        __dsync();
        /* Load the current top of stack pointer and csa info */
        ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[1];
        uxLowerCSA = __mfcr( CPU_PCXI );
        pxLowerCSA = portCSA_TO_ADDRESS( uxLowerCSA );
        pxUpperCSA = portCSA_TO_ADDRESS( pxLowerCSA[ 0 ] );
        /* Update the stack info in the TCB */
        *ppxTopOfStack = ( unsigned long * ) pxUpperCSA[ 2 ];
        /* Place the lower CSA id on the stack */
        ( *ppxTopOfStack )--;
        **ppxTopOfStack = uxLowerCSA;

        vTaskSwitchContext(1);

        /* Load the new CSA id from the new stack and update the stack pointer */
        ppxTopOfStack = ( unsigned long ** ) pxCurrentTCBs[1];
        uxLowerCSA = **ppxTopOfStack;
        ( *ppxTopOfStack )++;
        /* Update the link register */
        __mtcr( CPU_PCXI, uxLowerCSA );
        __isync();
    }
    __enable();
}

/*-----------------------------------------------------------*/

__attribute__( ( __noreturn__ ) ) void vPortLoopForever( void )
{
    while( 1 )
    {
    }
}

void vPortConfigureTimeForRunTimeStats()
{
    //Nothing to do... STM is used
}

unsigned long vPortGetRunTimeCounterValue()
{
    int cpu_id = portGET_CORE_ID();
    return (unsigned long) (IfxStm_get(g_system_timer[cpu_id])/1000);
}

UBaseType_t ulPortSetInterruptMask(void)
{
    unsigned long ulICR;
    __disable();
    ulICR = __mfcr( CPU_ICR );      /* Get current ICR value. */
    ulICR &= ~portCCPN_MASK;                       /* Clear down mask bits. */
    ulICR |= configMAX_SYSCALL_INTERRUPT_PRIORITY; /* Set mask bits to required priority mask. */
    __mtcr( CPU_ICR, ulICR );       /* Write back updated ICR. */
    __isync();
    __enable();
    return ulICR;
}

unsigned long uxPortSetInterruptMaskFromISR( void )
{
    unsigned long uxReturn = 0UL;

    __disable();
    uxReturn = __mfcr( CPU_ICR );
    __mtcr( CPU_ICR, ( ( uxReturn & ~portCCPN_MASK ) | configMAX_SYSCALL_INTERRUPT_PRIORITY ) );
    __isync();
    __enable();

    /* Return just the interrupt mask bits. */
    return( uxReturn & portCCPN_MASK );
}

void vPortAssertIfInISR( void )
{
    configASSERT( ( __mfcr( CPU_PSW ) & ( 1U << 9U ) ) == 0x00000000U );
}

BaseType_t xPortCheckIfInISR(void)
{
    return pdFALSE;
}
#include "Cpu/Std/IfxCpu.h"
static IfxCpu_spinLock spinlock_task;
static IfxCpu_spinLock spinlock_isr;
void spinlock_take_task()
{
    IfxCpu_setSpinLock(&spinlock_task, 0xFFFF);
}
void spinlock_release_task()
{
    IfxCpu_resetSpinLock(&spinlock_task);
}
void spinlock_take_isr()
{
    IfxCpu_setSpinLock(&spinlock_isr, 0xFFFF);
}
void spinlock_release_isr()
{
    IfxCpu_resetSpinLock(&spinlock_isr);
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    /* Idle task control block and stack */
    static StaticTask_t Idle_TCB;
    static StackType_t  Idle_Stack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer   = &Idle_TCB;
    *ppxIdleTaskStackBuffer = &Idle_Stack[0];
    *pulIdleTaskStackSize   = (uint32_t)configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    /* Timer task control block and stack */
    static StaticTask_t Timer_TCB;
    static StackType_t  Timer_Stack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer   = &Timer_TCB;
    *ppxTimerTaskStackBuffer = &Timer_Stack[0];
    *pulTimerTaskStackSize   = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}
#endif
