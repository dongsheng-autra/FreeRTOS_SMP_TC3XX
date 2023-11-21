/**********************************************************************************************************************
 * \file Cpu2_Main.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 * 
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of 
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 * 
 * Boost Software License - Version 1.0 - August 17th, 2003
 * 
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and 
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 * 
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all 
 * derivative works of the Software, unless such copies or derivative works are solely in the form of 
 * machine-executable object code generated by a source language processor.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN 
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 * IN THE SOFTWARE.
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"

#include "FreeRTOS.h"
#include "task.h"

#include "UART_VCOM.h"
#include "test.h"

static void task_cpu2_beat(void *arg)
{
    int* cpuid = arg;
    int  count = 0;
    int  result;
    uint8_t core_idx = IfxCpu_getCoreIndex();
    uint8_t crc_cal;

    while (1)
    {
        if (core_idx != *cpuid)
        {
            console_printf("CPU %d beat task run in Core %d.\n", *cpuid, core_idx);
        }

        xSemaphoreTake(g_mutex, portMAX_DELAY);

        result = test_semaphore(&g_semtest, &crc_cal);

        xSemaphoreGive(g_mutex);

        if (result != 0)
        {
            console_printf("CPU %d semaphore test fail.\n", *cpuid);
        }
        else
        {
            console_printf("CPU %d, run %d s.\n", *cpuid, count);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        count++;
    }
}

void core2_main(void)
{
    IfxCpu_enableInterrupts();
    
    /* !!WATCHDOG2 IS DISABLED HERE!!
     * Enable the watchdog and service it periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

    int cpuid = IfxCpu_getCoreIndex();

    xTaskCreateAffinitySet(task_cpu2_beat, "task_cpu2_beat", 512, (void * const)&cpuid, 5, 0x04, NULL);

    /* Start the tasks running. */
    vTaskStartScheduler(cpuid);

    while(1)
    {

    }
}
