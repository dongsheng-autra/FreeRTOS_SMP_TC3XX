/*
 * Copyright (c) 2023 Autra.Tech. All rights reserved.
 *
 * THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
 * ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
 * IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
 * PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
 */

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
    extern "C" {
#endif

/* System Includes. */
#include <stdint.h>

#include "IfxCpu.h"
#include "IfxCpu_reg.h"

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR          char
#define portFLOAT         float
#define portDOUBLE        double
#define portLONG          long
#define portSHORT         short
#define portSTACK_TYPE    unsigned long
#define portBASE_TYPE     long

typedef portSTACK_TYPE   StackType_t;
typedef long             BaseType_t;
typedef unsigned long    UBaseType_t;

#if ( configUSE_16_BIT_TICKS == 1 )
    typedef unsigned     TickType_t;
    #define portMAX_DELAY              ( TickType_t ) 0xffff
#else
    typedef unsigned     TickType_t;
    #define portMAX_DELAY              ( TickType_t ) 0xffffffffUL

    /* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
     * not need to be guarded with a critical section. */
    #define portTICK_TYPE_IS_ATOMIC    1
#endif
/*---------------------------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH               ( -1 )
#define portTICK_PERIOD_MS             ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT             4
extern void vPortAssertIfInISR( void );
#define portASSERT_IF_IN_ISR()    vPortAssertIfInISR()
#define portNOP()                 TriCore__nop()
#define portCRITICAL_NESTING_IN_TCB    1

extern void vTaskEnterCritical( void );
extern void vTaskExitCritical( void );
extern __attribute__( ( __noreturn__ ) ) void vPortLoopForever( void );
#define portENTER_CRITICAL()    vTaskEnterCritical()
#define portEXIT_CRITICAL()     vTaskExitCritical()

extern void vPortCoreEmitSyncEvent( void );
#define portCORE_EMIT_SYNC()             vPortCoreEmitSyncEvent()

extern BaseType_t vPortCoreWaitSyncEvent( void );
#define portCORE_WAIT_SYNC()             vPortCoreWaitSyncEvent()

extern void vPortYield( void );
#define portYIELD()             vPortYield()

extern void vPortConfigureTimeForRunTimeStats();
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vPortConfigureTimeForRunTimeStats()
extern unsigned long vPortGetRunTimeCounterValue();
#define portGET_RUN_TIME_COUNTER_VALUE()    vPortGetRunTimeCounterValue()

/* Syscall IDs */
#define portSYSCALL_TASK_YIELD    0

/* Critical section management. */
#define portCCPN_MASK             ( 0x000000FFUL )
/* Set ICR.CCPN to configMAX_SYSCALL_INTERRUPT_PRIORITY. */
extern UBaseType_t ulPortSetInterruptMask(void);
#define portDISABLE_INTERRUPTS()     ulPortSetInterruptMask()

/* Clear ICR.CCPN to allow all interrupt priorities. */
#define portENABLE_INTERRUPTS()                                                 \
    {                                                                           \
        unsigned long ulICR;                                                    \
        __disable();                                                     \
        ulICR = __mfcr( CPU_ICR ); /* Get current ICR value. */  \
        ulICR &= ~portCCPN_MASK;                  /* Clear down mask bits. */   \
        __mtcr( CPU_ICR, ulICR );  /* Write back updated ICR. */ \
        __isync();                                                       \
        __enable();                                                      \
    }

#define portRESTORE_INTERRUPTS(x)    portENABLE_INTERRUPTS()

/* Set ICR.CCPN to uxSavedMaskValue. */
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedMaskValue )                                         \
    {                                                                                                 \
        unsigned long ulICR;                                                                          \
        __disable();                                                                           \
        ulICR = __mfcr( CPU_ICR ); /* Get current ICR value. */                        \
        ulICR &= ~portCCPN_MASK;                  /* Clear down mask bits. */                         \
        ulICR |= uxSavedMaskValue;                /* Set mask bits to previously saved mask value. */ \
        __mtcr( CPU_ICR, ulICR );  /* Write back updated ICR. */                       \
        __isync();                                                                             \
        __enable();                                                                            \
    }


/* Set ICR.CCPN to configMAX_SYSCALL_INTERRUPT_PRIORITY */
extern unsigned long uxPortSetInterruptMaskFromISR( void );
#define portSET_INTERRUPT_MASK_FROM_ISR()                     uxPortSetInterruptMaskFromISR()

/* Pend a priority 1 interrupt, which will take care of the context switch. */
#define portYIELD_FROM_ISR( xHigherPriorityTaskWoken )        if( xHigherPriorityTaskWoken != pdFALSE ) { portYIELD(); }

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )

#include "IfxCpu_Intrinsics.h"
#define portMEMORY_BARRIER()         __asm( "" ::: "memory" )

/* Multi-core */
#define configNUM_CORES                      6
#define configMAIN_CORE                      0
#define configUSE_CORE_AFFINITY              1
#define configRUN_MULTIPLE_PRIORITIES        1
#define configUSE_TASK_PREEMPTION_DISABLE    0

#define portGET_CORE_ID()             IfxCpu_getCoreIndex()
extern void vPortYieldCore(BaseType_t xCoreID);
#define portYIELD_CORE(a)             vPortYieldCore(a)

extern void spinlock_take_task();
extern void spinlock_release_task();
extern void spinlock_take_isr();
extern void spinlock_release_isr();
#define portGET_TASK_LOCK()                         spinlock_take_task()
#define portRELEASE_TASK_LOCK()                     spinlock_release_task()
#define portGET_ISR_LOCK()                          spinlock_take_isr()
#define portRELEASE_ISR_LOCK()                      spinlock_release_isr()

extern BaseType_t xPortCheckIfInISR(void);
#define portCHECK_IF_IN_ISR()  xPortCheckIfInISR

#define __builtin_clz   __clz

#ifdef __cplusplus
    }
#endif

#endif /* PORTMACRO_H */
