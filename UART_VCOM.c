/**********************************************************************************************************************
 * \file UART_VCOM.c
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

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "IfxAsclin_Asc.h"
#include "IfxCpu_Irq.h"
#include "pkt_queue.h"

#include <stdint.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define SERIAL_BAUDRATE         115200                                      /* Baud rate in bit/s                   */

#define SERIAL_PIN_RX           IfxAsclin0_RXA_P14_1_IN                     /* RX pin of the board                  */
#define SERIAL_PIN_TX           IfxAsclin0_TX_P14_0_OUT                     /* TX pin of the board                  */

#define INTPRIO_ASCLIN0_TX      19                                          /* Priority of the ISR                  */

#define ASC_TX_BUFFER_SIZE      64                                          /* Definition of the buffer size        */

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxAsclin_Asc g_asc;                                                        /* Declaration of the ASC handle        */

/* The transfer buffers allocate memory for the data itself and for FIFO runtime variables.
 * 8 more bytes have to be added to ensure a proper circular buffer handling independent from
 * the address to which the buffers have been located.
 */
uint8 g_ascTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];             /* Declaration of the FIFOs parameters  */

struct pkt_queue tx_queue;
#define CONSOLE_TX_BUFFER_SIZE      1024

#define CONSOLE_TX_QUEUE_SIZE       32
#define CONSOLE_TX_QUEUE_MASK       (CONSOLE_TX_QUEUE_SIZE - 1)

static int g_status = 0;

struct console_buf {
    uint8_t buf[CONSOLE_TX_BUFFER_SIZE];
    uint32_t len;
};
struct console_buf pkt_tx_buf[CONSOLE_TX_QUEUE_SIZE];
int pkt_tx_index;
/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
IFX_INTERRUPT(asclin0_Tx_ISR, 5, INTPRIO_ASCLIN0_TX);                         /* Adding the Interrupt Service Routine */

void asclin0_Tx_ISR(void)
{
    IfxAsclin_Asc_isrTransmit(&g_asc);
}

void init_UART(void)
{
    /* Initialize an instance of IfxAsclin_Asc_Config with default values */
    IfxAsclin_Asc_Config ascConfig;
    IfxAsclin_Asc_initModuleConfig(&ascConfig, SERIAL_PIN_TX.module);

    /* Set the desired baud rate */
    ascConfig.baudrate.baudrate = SERIAL_BAUDRATE;

    /* ISR priorities and interrupt target */
    ascConfig.interrupt.txPriority = INTPRIO_ASCLIN0_TX;
    ascConfig.interrupt.typeOfService = IfxCpu_Irq_getTos(5);;

    /* FIFO configuration */
    ascConfig.txBuffer = &g_ascTxBuffer;
    ascConfig.txBufferSize = ASC_TX_BUFFER_SIZE;

    /* Port pins configuration */
    const IfxAsclin_Asc_Pins pins =
    {
        NULL_PTR,         IfxPort_InputMode_pullUp,     /* CTS pin not used     */
        &SERIAL_PIN_RX,   IfxPort_InputMode_pullUp,     /* RX pin not used      */
        NULL_PTR,         IfxPort_OutputMode_pushPull,  /* RTS pin not used     */
        &SERIAL_PIN_TX,   IfxPort_OutputMode_pushPull,  /* TX pin               */
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConfig.pins = &pins;

    IfxAsclin_Asc_initModule(&g_asc, &ascConfig);                       /* Initialize module with above parameters  */

    memset(pkt_tx_buf, 0, sizeof(pkt_tx_buf));
    pkt_queue_init(&tx_queue, CONSOLE_TX_QUEUE_SIZE);
    pkt_tx_index = 0;
    g_status = 0x55AA;
}

static IfxCpu_spinLock spinlock;

#include <stdio.h>
#include <stdarg.h>
void console_printf(const char *format, ...)
{
    Ifx_SizeT len;
    va_list ap;
    struct console_buf *buf;
    uint8_t *pkt_tx;

    if (g_status != 0x55AA) {
        return;
    }

    boolean flag = IfxCpu_setSpinLock(&spinlock, 0xFFFF);

    buf = (void *)&pkt_tx_buf[pkt_tx_index];
    memset(buf, 0, sizeof(struct console_buf));
    pkt_tx = buf->buf;

    va_start(ap, format);
    len = (Ifx_SizeT)vsnprintf(pkt_tx, CONSOLE_TX_BUFFER_SIZE, format, ap);
    va_end(ap);

    buf->len = len;
    pkt_queue_put(&tx_queue, (void *)buf);

    pkt_tx_index = ((pkt_tx_index + 1) & CONSOLE_TX_QUEUE_MASK);

    if (flag){
        IfxCpu_resetSpinLock(&spinlock);
    }
}

void console_process(void)
{
    struct console_buf *buf;

    if (pkt_queue_check(&tx_queue)) {
        buf = (struct console_buf *)pkt_queue_get(&tx_queue);
        if ((buf != NULL) && (buf->len > 0)) {
            IfxAsclin_Asc_write(&g_asc, (const void *)buf, &buf->len, TIME_INFINITE);
        }
    }
}