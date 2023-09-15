/*
 * Copyright (c) 2023 Autra.Tech. All rights reserved.
 *
 * THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
 * ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
 * IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
 * PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

#define configUSE_PREEMPTION                            1
#define configUSE_TICKLESS_IDLE                         0
#define configCPU_CLOCK_HZ                              ( ( unsigned long ) 300000000UL )
#define configTICK_RATE_HZ                              ( ( TickType_t ) 1000UL )
#define configMAX_PRIORITIES                            10
#define configMINIMAL_STACK_SIZE                        128
#define configMAX_TASK_NAME_LEN                         32
#define configUSE_16_BIT_TICKS                          0
#define configIDLE_SHOULD_YIELD                         1
#define configUSE_TASK_NOTIFICATIONS                    1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES           1
#define configUSE_MUTEXES                               1
#define configUSE_RECURSIVE_MUTEXES                     1
#define configUSE_COUNTING_SEMAPHORES                   1
#define configQUEUE_REGISTRY_SIZE                       8
#define configUSE_QUEUE_SETS                            1
#define configUSE_TIME_SLICING                          1
#define configENABLE_BACKWARD_COMPATIBILITY             1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS         5
#define configUSE_APPLICATION_TASK_TAG                  1

#define configSUPPORT_STATIC_ALLOCATION                 1
#define configSUPPORT_DYNAMIC_ALLOCATION                1
#define configTOTAL_HEAP_SIZE                           ( (size_t)(48 * 1024) )
#define configAPPLICATION_ALLOCATED_HEAP                0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP       0


#define configUSE_IDLE_HOOK                             0
#define configUSE_TICK_HOOK                             0
#define configCHECK_FOR_STACK_OVERFLOW                  0
#define configUSE_MALLOC_FAILED_HOOK                    0
#define configUSE_DAEMON_TASK_STARTUP_HOOK              0


#define configGENERATE_RUN_TIME_STATS                   1
#define configUSE_TRACE_FACILITY                        1
#define configUSE_STATS_FORMATTING_FUNCTIONS            1


#define configUSE_CO_ROUTINES                           0
#define configMAX_CO_ROUTINE_PRIORITIES                 2


#define configUSE_TIMERS                                1
#define configTIMER_TASK_PRIORITY                       ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                        10
#define configTIMER_TASK_STACK_DEPTH                    1024


#define INCLUDE_vTaskPrioritySet                        1
#define INCLUDE_uxTaskPriorityGet                       1
#define INCLUDE_vTaskDelete                             0
#define INCLUDE_vTaskSuspend                            1
#define INCLUDE_xResumeFromISR                          1
#define INCLUDE_vTaskDelayUntil                         1
#define INCLUDE_vTaskDelay                              1
#define INCLUDE_xTaskGetSchedulerState                  1
#define INCLUDE_xTaskGetCurrentTaskHandle               1
#define INCLUDE_uxTaskGetStackHighWaterMark             1
#define INCLUDE_xTaskGetIdleTaskHandle                  1
#define INCLUDE_eTaskGetState                           1
#define INCLUDE_xEventGroupSetBitFromISR                1
#define INCLUDE_xTimerPendFunctionCall                  1
#define INCLUDE_xTaskAbortDelay                         1
#define INCLUDE_xTaskGetHandle                          1
#define INCLUDE_xTaskResumeFromISR                      1
#define INCLUDE_xQueueGetMutexHolder                    1

#define configKERNEL_INTERRUPT_PRIORITY                 1
#define configMAX_SYSCALL_INTERRUPT_PRIORITY            31
#define configMAX_API_CALL_INTERRUPT_PRIORITY           31

#define configASSERT( x ) if( ( x ) == 0 )              { portDISABLE_INTERRUPTS(); __debug(); vPortLoopForever();}

#endif /* FREERTOS_CONFIG_H */

