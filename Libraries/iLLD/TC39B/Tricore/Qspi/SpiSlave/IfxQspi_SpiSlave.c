/**
 * \file IfxQspi_SpiSlave.c
 * \brief QSPI SPISLAVE details
 *
 * \version iLLD_1_0_1_16_1_1
 * \copyright Copyright (c) 2023 Infineon Technologies AG. All rights reserved.
 *
 *
 *
 *                                 IMPORTANT NOTICE
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees. If and as long as no such terms
 * of use are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer, must
 * be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are
 * solely in the form of machine-executable object code generated by a source
 * language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "IfxQspi_SpiSlave.h"

/** \addtogroup IfxLld_Qspi_SpiSlave_Support
 * \{ */

/******************************************************************************/
/*-----------------------Private Function Prototypes--------------------------*/
/******************************************************************************/

/** \brief Reads data from the Rx FIFO
 * \param handle Module handle
 * \return None
 */
IFX_STATIC void IfxQspi_SpiSlave_read(IfxQspi_SpiSlave *handle);

/** \brief Writes data into the Tx FIFO
 * \param handle Module handle
 * \return None
 */
IFX_STATIC void IfxQspi_SpiSlave_write(IfxQspi_SpiSlave *handle);

/** \} */

/** \addtogroup IfxLld_Qspi_SpiSlave_DataStructures
 * \{ */

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

/** \brief dummy variable where recived data is placed
 */
IFX_STATIC uint32           IfxQspi_SpiSlave_dummyRxValue = 0;

/** \brief dummy value to be transmitted
 */
IFX_STATIC IFX_CONST uint32 IfxQspi_SpiSlave_dummyTxValue = ~0;

/** \} */

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

SpiIf_Status IfxQspi_SpiSlave_exchange(IfxQspi_SpiSlave *handle, const void *src, void *dest, Ifx_SizeT count)
{
    SpiIf_Status status = SpiIf_Status_busy;

    /* initiate transfer when resource is free */
    if (handle->onTransfer == FALSE)
    {
        status                  = SpiIf_Status_ok;

        handle->onTransfer      = TRUE;
        handle->txJob.data      = (void *)src;
        handle->txJob.remaining = count;
        handle->rxJob.data      = dest;
        handle->rxJob.remaining = count;
        IfxQspi_SpiSlave_write(handle);
    }

    return status;
}


SpiIf_Status IfxQspi_SpiSlave_getStatus(IfxQspi_SpiSlave *handle)
{
    SpiIf_Status status = SpiIf_Status_ok;

    if (handle->onTransfer != 0)
    {
        status = SpiIf_Status_busy;
    }

    return status;
}


void IfxQspi_SpiSlave_initModule(IfxQspi_SpiSlave *handle, const IfxQspi_SpiSlave_Config *config)
{
    Ifx_QSPI *qspiSFR = config->qspi;
    Ifx_DMA  *dmaSFR  = &MODULE_DMA;

    /* handle.base must be at offset 0 to be compatible with the standard interface SscIf */
    {
        uint16 password = IfxScuWdt_getCpuWatchdogPassword();
        IfxScuWdt_clearCpuEndinit(password);
        IfxQspi_setEnableModuleRequest(qspiSFR);
        IfxQspi_setSleepMode(qspiSFR, (config->allowSleepMode != FALSE) ? IfxQspi_SleepMode_enable : IfxQspi_SleepMode_disable);
        IfxScuWdt_setCpuEndinit(password);
    }

    {                                                        /* Configure GLOBAL, Note: at the moment default values for GLOBAL */
        Ifx_QSPI_GLOBALCON globalcon;
        globalcon.U        = 0;
        globalcon.B.TQ     = IfxQspi_calculateTimeQuantumLength(qspiSFR, config->base.maximumBaudrate);
        globalcon.B.EXPECT = IfxQspi_ExpectTimeout_2097152;  /* 2^(EXPECT+6) : timeout for expect phase in Tqspi */
        //globalcon.B.LB      = 0;                             /* 0 : disable loop-back w*/
        //globalcon.B.DEL0    = 0;                             /* 0 : disable delayed mode for SLSO 0 */
        //globalcon.B.STROBE  = 0;                             /* (STROBE+1) : strobe delay for SLSO 0 in Tq */
        //globalcon.B.SRF     = 0;                             /* 0 : disable stop-on-RXFIFO full feature */
        //globalcon.B.STIP    = 0;                             /* 0 : MRST = 0 when QSPI is deselected in slave mode */
        //globalcon.B.EN      = 0;                             /* 0 : PAUSE requested, 1 : RUN requested */
        globalcon.B.MS       = IfxQspi_Mode_master; /* select master mode during configuration - we will switch to slave mode at the end of this function */
        globalcon.B.AREN     = (config->pauseOnBaudrateSpikeErrors != FALSE) ? 1U : 0U;
        globalcon.B.RESETS   = 1;                   /* Reset SM and FIFOs */
        globalcon.B.CLKSEL   = 1;
        qspiSFR->GLOBALCON.U = globalcon.U;
    }

    {   /* Configure interrupt requests */
        Ifx_QSPI_GLOBALCON1 globalcon1;
        globalcon1.U           = 0;
        globalcon1.B.ERRORENS  = (config->base.erPriority > 0) ? IFXQSPI_ERRORENABLEMASK : 0;
        globalcon1.B.TXEN      = (config->base.txPriority > 0) || (config->dma.useDma);
        globalcon1.B.RXEN      = (config->base.rxPriority > 0) || (config->dma.useDma);
        globalcon1.B.TXFIFOINT = config->txFifoThreshold;
        globalcon1.B.RXFIFOINT = config->rxFifoThreshold;
        globalcon1.B.TXFM      = config->txFifoMode;
        globalcon1.B.RXFM      = config->rxFifoMode;

        qspiSFR->GLOBALCON1.U  = globalcon1.U;
    }

    handle->qspi               = qspiSFR;
    handle->base.driver        = handle;
    handle->base.sending       = 0U;
    handle->base.activeChannel = NULL_PTR;

    /* Protocol Configuration */
    {
        IfxQspi_SpiSlave_Protocol *protocol = (IfxQspi_SpiSlave_Protocol *)&config->protocol;

        SpiIf_ChConfig             chConfig;
        SpiIf_initChannelConfig(&chConfig, NULL_PTR);
        chConfig.mode.clockPolarity = protocol->clockPolarity;
        chConfig.mode.shiftClock    = protocol->shiftClock;
        chConfig.mode.dataHeading   = protocol->dataHeading;
        chConfig.mode.dataWidth     = protocol->dataWidth;
        chConfig.mode.parityMode    = protocol->parityMode;

        {
            Ifx_QSPI_BACON bacon;
            uint8          cs = 0;                                                                                                                      // not relevant for slave

            qspiSFR->ECON[cs].U = IfxQspi_calculateExtendedConfigurationValue(qspiSFR, cs, &chConfig);
            bacon.U             = IfxQspi_calculateBasicConfigurationValue(qspiSFR, IfxQspi_ChannelId_0, &chConfig.mode, config->base.maximumBaudrate); /*delay params not applicable to slave*/
            IfxQspi_writeBasicConfigurationBeginStream(qspiSFR, bacon.U);
        }
        handle->dataWidth = protocol->dataWidth;
    }

    handle->rxJob.data      = NULL_PTR;
    handle->rxJob.remaining = 0;
    handle->txJob.data      = NULL_PTR;
    handle->txJob.remaining = 0;
    handle->onTransfer      = FALSE;

    /* Configure I/O pins for slave mode */
    const IfxQspi_SpiSlave_Pins *pins = config->pins;

    if (pins != NULL_PTR)
    {
        const IfxQspi_Sclk_In *sclkIn = pins->sclk;

        if (sclkIn != NULL_PTR)
        {
            IfxQspi_initSclkInPinWithPadLevel(sclkIn, pins->sclkMode, pins->pinDriver);
        }

        const IfxQspi_Mtsr_In *mtsrIn = pins->mtsr;

        if (mtsrIn != NULL_PTR)
        {
            IfxQspi_initMtsrInPinWithPadLevel(mtsrIn, pins->mtsrMode, pins->pinDriver);
        }

        const IfxQspi_Mrst_Out *mrstOut = pins->mrst;

        if (mrstOut != NULL_PTR)
        {
            IfxQspi_initMrstOutPin(mrstOut, pins->mrstMode, pins->pinDriver);
        }

        const IfxQspi_Slsi_In *slsiIn = pins->slsi;

        if (slsiIn != NULL_PTR)
        {
            IfxQspi_initSlsiWithPadLevel(slsiIn, pins->slsiMode, pins->pinDriver);
        }
    }

    if (config->dma.useDma)
    {
        IfxDma_Dma               dma;
        IfxDma_Dma_createModuleHandle(&dma, dmaSFR);

        IfxDma_Dma_ChannelConfig dmaCfg;
        IfxDma_Dma_initChannelConfig(&dmaCfg, &dma);
        handle->dma.useDma = TRUE;
        {
            handle->dma.txDmaChannelId     = config->dma.txDmaChannelId;
            dmaCfg.channelId               = handle->dma.txDmaChannelId;
            dmaCfg.hardwareRequestEnabled  = FALSE; // will be triggered from qspi service request
            dmaCfg.channelInterruptEnabled = TRUE;  // trigger interrupt after transaction

            // source address and transfer count will be configured during runtime
            dmaCfg.sourceAddress               = 0;
            dmaCfg.sourceAddressCircularRange  = IfxDma_ChannelIncrementCircular_none;
            dmaCfg.sourceCircularBufferEnabled = FALSE;
            dmaCfg.transferCount               = 0;
            dmaCfg.moveSize                    = IfxDma_ChannelMoveSize_8bit;

            // destination address is fixed; use circular mode to stay at this address for each move
            dmaCfg.destinationAddress               = (uint32)&qspiSFR->DATAENTRY[0].U;
            dmaCfg.destinationAddressCircularRange  = IfxDma_ChannelIncrementCircular_none;
            dmaCfg.destinationCircularBufferEnabled = TRUE;

            dmaCfg.requestMode                      = IfxDma_ChannelRequestMode_oneTransferPerRequest;
            dmaCfg.operationMode                    = IfxDma_ChannelOperationMode_single;
            dmaCfg.blockMode                        = IfxDma_ChannelMove_1;

            IfxDma_Dma_initChannel(&handle->dma.txDmaChannel, &dmaCfg);
        }

        {
            handle->dma.rxDmaChannelId     = config->dma.rxDmaChannelId;
            dmaCfg.channelId               = handle->dma.rxDmaChannelId;
            dmaCfg.hardwareRequestEnabled  = FALSE; // will be triggered from qspi service request
            dmaCfg.channelInterruptEnabled = TRUE;  // trigger interrupt after transaction

            // source address is fixed; use circular mode to stay at this address for each move
            dmaCfg.sourceAddress               = (uint32)&qspiSFR->RXEXIT.U;
            dmaCfg.sourceAddressCircularRange  = IfxDma_ChannelIncrementCircular_none;
            dmaCfg.sourceCircularBufferEnabled = TRUE;

            // destination address and transfer count will be configured during runtime
            dmaCfg.destinationAddress               = 0;
            dmaCfg.destinationAddressCircularRange  = IfxDma_ChannelIncrementCircular_none;
            dmaCfg.destinationCircularBufferEnabled = FALSE;
            dmaCfg.transferCount                    = 0;

            dmaCfg.requestMode                      = IfxDma_ChannelRequestMode_oneTransferPerRequest;
            dmaCfg.operationMode                    = IfxDma_ChannelOperationMode_single;
            dmaCfg.moveSize                         = IfxDma_ChannelMoveSize_8bit;
            dmaCfg.blockMode                        = IfxDma_ChannelMove_1;

            IfxDma_Dma_initChannel(&handle->dma.rxDmaChannel, &dmaCfg);
        }
        /* Dma channel interrupt configuration */
        {
            volatile Ifx_SRC_SRCR *src = IfxDma_getSrcPointer(dmaSFR, (IfxDma_ChannelId)config->dma.txDmaChannelId);
            IfxSrc_init(src, config->base.isrProvider, config->base.txPriority);
            IfxSrc_enable(src);

            src = IfxDma_getSrcPointer(dmaSFR, (IfxDma_ChannelId)config->dma.rxDmaChannelId);
            IfxSrc_init(src, config->base.isrProvider, config->base.rxPriority);
            IfxSrc_enable(src);
        }
    }

    /* interrupt configuration */
    {
        IfxQspi_clearAllEventFlags(qspiSFR);

        if (handle->dma.useDma)
        {
            volatile Ifx_SRC_SRCR *src = IfxQspi_getTransmitSrc(qspiSFR);
            IfxSrc_init(src, IfxSrc_Tos_dma, (Ifx_Priority)config->dma.txDmaChannelId);
            IfxSrc_enable(src);

            src = IfxQspi_getReceiveSrc(qspiSFR);
            IfxSrc_init(src, IfxSrc_Tos_dma, (Ifx_Priority)config->dma.rxDmaChannelId);
            IfxSrc_enable(src);
        }
        else
        {
            if (config->base.txPriority != 0)
            {
                volatile Ifx_SRC_SRCR *src = IfxQspi_getTransmitSrc(qspiSFR);
                IfxSrc_init(src, config->base.isrProvider, config->base.txPriority);
                IfxSrc_enable(src);
            }

            if (config->base.rxPriority != 0)
            {
                volatile Ifx_SRC_SRCR *src = IfxQspi_getReceiveSrc(qspiSFR);
                IfxSrc_init(src, config->base.isrProvider, config->base.rxPriority);
                IfxSrc_enable(src);
            }

            if (config->base.erPriority != 0)
            {
                volatile Ifx_SRC_SRCR *src = IfxQspi_getErrorSrc(qspiSFR);
                IfxSrc_init(src, config->base.isrProvider, config->base.erPriority);
                IfxSrc_enable(src);
            }
        }
    }
    /* finally switch to slave mode */
    qspiSFR->GLOBALCON.B.MS = IfxQspi_Mode_slave;
    IfxQspi_run(qspiSFR);
}


void IfxQspi_SpiSlave_initModuleConfig(IfxQspi_SpiSlave_Config *config, Ifx_QSPI *qspi)
{
    const IfxQspi_SpiSlave_Protocol defaultProtocol = {
        .clockPolarity = SpiIf_ClockPolarity_idleLow,
        .shiftClock    = SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge,
        .dataHeading   = SpiIf_DataHeading_msbFirst,
        .dataWidth     = 8,
        .parityMode    = Ifx_ParityMode_even
    };

    SpiIf_initConfig(&config->base);

    config->qspi                       = qspi;
    config->allowSleepMode             = FALSE;
    config->pauseOnBaudrateSpikeErrors = FALSE,
    config->pauseRunTransition         = IfxQspi_PauseRunTransition_pause;
    config->txFifoThreshold            = IfxQspi_TxFifoInt_1;
    config->rxFifoThreshold            = IfxQspi_RxFifoInt_0;
    config->txFifoMode                 = IfxQspi_FifoMode_combinedMove;
    config->rxFifoMode                 = IfxQspi_FifoMode_combinedMove;
    config->pins                       = NULL_PTR;
    config->protocol                   = defaultProtocol;

    config->dma.rxDmaChannelId         = IfxDma_ChannelId_none;
    config->dma.txDmaChannelId         = IfxDma_ChannelId_none;
    config->dma.useDma                 = FALSE;
    config->base.maximumBaudrate       = 50000000;
}


void IfxQspi_SpiSlave_isrDmaReceive(IfxQspi_SpiSlave *qspiHandle)
{
    Ifx_DMA         *dmaSFR         = &MODULE_DMA;
    IfxDma_ChannelId rxDmaChannelId = qspiHandle->dma.rxDmaChannelId;

    if (IfxDma_getAndClearChannelInterrupt(dmaSFR, rxDmaChannelId))
    {
        qspiHandle->onTransfer = FALSE;
    }

    IfxDma_getAndClearChannelPatternDetectionInterrupt(dmaSFR, rxDmaChannelId);
}


void IfxQspi_SpiSlave_isrDmaTransmit(IfxQspi_SpiSlave *qspiHandle)
{
    Ifx_DMA         *dmaSFR         = &MODULE_DMA;
    IfxDma_ChannelId txDmaChannelId = qspiHandle->dma.txDmaChannelId;
    // TODO
    IfxDma_getAndClearChannelPatternDetectionInterrupt(dmaSFR, txDmaChannelId);
    IfxDma_getAndClearChannelInterrupt(dmaSFR, txDmaChannelId);
}


void IfxQspi_SpiSlave_isrError(IfxQspi_SpiSlave *handle)
{
    Ifx_QSPI *qspiSFR    = handle->qspi;
    uint16    errorFlags = IfxQspi_getErrorFlags(qspiSFR);
    IfxQspi_clearAllEventFlags(qspiSFR);
    Ifx_DMA  *dmaSFR     = &MODULE_DMA;

    /* store all the flags in the variable */

    if ((errorFlags & IfxQspi_Error_parity))
    {
        handle->errorFlags.parityError = 1;
    }

    if ((errorFlags & IfxQspi_Error_configuration))
    {
        handle->errorFlags.configurationError = 1;
    }

    if ((errorFlags & IfxQspi_Error_baudrate))
    {
        handle->errorFlags.baudrateError = 1;
    }

    if ((errorFlags & IfxQspi_Error_expectTimeout))
    {
        handle->errorFlags.expectTimeoutError = 1;
    }

    if ((errorFlags & IfxQspi_Error_txfifoOverflow))
    {
        handle->errorFlags.txFifoOverflowError = 1;
    }

    if ((errorFlags & IfxQspi_Error_txfifoUnderflow))
    {
        handle->errorFlags.txFifoUnderflowError = 1;
    }

    if ((errorFlags & IfxQspi_Error_rxfifoOverflow))
    {
        handle->errorFlags.rxFifoOverflowError = 1;
    }

    if ((errorFlags & IfxQspi_Error_rxfifoUnderflow))
    {
        handle->errorFlags.rxFifoUnderflowError = 1;
    }

    if ((errorFlags & IfxQspi_Error_slsiMisplacedInactivation))
    {
        handle->errorFlags.slsiMisplacedInactivation = 1;
    }

    if (errorFlags)
    {
        handle->onTransfer = FALSE;
    }

    if (handle->dma.useDma)
    {
        IfxDma_getAndClearChannelInterrupt(dmaSFR, handle->dma.rxDmaChannelId);
        IfxDma_getAndClearChannelInterrupt(dmaSFR, handle->dma.txDmaChannelId);
    }
}


void IfxQspi_SpiSlave_isrReceive(IfxQspi_SpiSlave *handle)
{
    IfxQspi_SpiSlave_read(handle);
}


void IfxQspi_SpiSlave_isrTransmit(IfxQspi_SpiSlave *handle)
{
    IfxQspi_SpiSlave_write(handle);
}


IFX_STATIC void IfxQspi_SpiSlave_read(IfxQspi_SpiSlave *handle)
{
    Ifx_QSPI  *qspiSFR = handle->qspi;
    SpiIf_Job *job     = &handle->rxJob;
    Ifx_SizeT  count   = (Ifx_SizeT)IfxQspi_getReceiveFifoLevel(qspiSFR);
    count = __min(job->remaining, count);

    if (job->data == NULL_PTR)
    {
        // no data should be buffered: do dummy reads
        int i;

        for (i = 0; i < count; ++i)
        {
            IfxQspi_readReceiveFifo(qspiSFR);
        }
    }
    else
    {
        if (handle->dataWidth <= 8)
        {
            IfxQspi_read8(qspiSFR, job->data, count);
            job->data = &(((uint8 *)job->data)[count]);
        }
        else if (handle->dataWidth <= 16)
        {
            IfxQspi_read16(qspiSFR, job->data, count);
            job->data = &(((uint16 *)job->data)[count]);
        }
        else
        {
            IfxQspi_read32(qspiSFR, job->data, count);
            job->data = &(((uint32 *)job->data)[count]);
        }
    }

    job->remaining = job->remaining - count;

    if (job->remaining == 0)
    {
        handle->onTransfer = FALSE;
    }
}


IFX_STATIC void IfxQspi_SpiSlave_write(IfxQspi_SpiSlave *handle)
{
    SpiIf_Job *job = &handle->txJob;

    if (handle->dma.useDma)
    {
        Ifx_DMA               *dmaSFR         = &MODULE_DMA;
        SpiIf_Job             *jobrx          = &handle->rxJob;

        Ifx_QSPI              *qspiSFR        = handle->qspi;
        volatile Ifx_SRC_SRCR *src            = IfxQspi_getTransmitSrc(qspiSFR);

        IfxDma_ChannelId       txDmaChannelId = handle->dma.txDmaChannelId;
        IfxDma_ChannelId       rxDmaChannelId = handle->dma.rxDmaChannelId;

        boolean                interruptState = IfxCpu_disableInterrupts();
        IfxDma_setChannelTransferCount(dmaSFR, txDmaChannelId, job->remaining);

        if (handle->dataWidth <= 8)
        {
            IfxDma_setChannelMoveSize(dmaSFR, txDmaChannelId, IfxDma_ChannelMoveSize_8bit);
        }
        else if (handle->dataWidth <= 16)
        {
            IfxDma_setChannelMoveSize(dmaSFR, txDmaChannelId, IfxDma_ChannelMoveSize_16bit);
        }
        else
        {
            IfxDma_setChannelMoveSize(dmaSFR, txDmaChannelId, IfxDma_ChannelMoveSize_32bit);
        }

        if (job->data == NULL_PTR)
        {
            IfxDma_setChannelSourceAddress(dmaSFR, txDmaChannelId, (void *)IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), &IfxQspi_SpiSlave_dummyTxValue));
            IfxDma_setChannelSourceIncrementStep(dmaSFR, txDmaChannelId, IfxDma_ChannelIncrementStep_1,
                IfxDma_ChannelIncrementDirection_positive, IfxDma_ChannelIncrementCircular_4);
        }
        else
        {
            IfxDma_setChannelSourceAddress(dmaSFR, txDmaChannelId, (void *)IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), job->data));
            IfxDma_setChannelSourceIncrementStep(dmaSFR, txDmaChannelId, IfxDma_ChannelIncrementStep_1,
                IfxDma_ChannelIncrementDirection_positive, IfxDma_ChannelIncrementCircular_none);
        }

        IfxDma_clearChannelInterrupt(dmaSFR, txDmaChannelId);

        /* Receive config */
        IfxDma_setChannelTransferCount(dmaSFR, rxDmaChannelId, job->remaining);

        if (handle->dataWidth <= 8)
        {
            IfxDma_setChannelMoveSize(dmaSFR, rxDmaChannelId, IfxDma_ChannelMoveSize_8bit);
        }
        else if (handle->dataWidth <= 16)
        {
            IfxDma_setChannelMoveSize(dmaSFR, rxDmaChannelId, IfxDma_ChannelMoveSize_16bit);
        }
        else
        {
            IfxDma_setChannelMoveSize(dmaSFR, rxDmaChannelId, IfxDma_ChannelMoveSize_32bit);
        }

        if (jobrx->data == NULL_PTR)
        {
            IfxDma_setChannelDestinationAddress(dmaSFR, rxDmaChannelId, (void *)IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), &IfxQspi_SpiSlave_dummyRxValue));
            IfxDma_setChannelDestinationIncrementStep(dmaSFR, rxDmaChannelId, IfxDma_ChannelIncrementStep_1,
                IfxDma_ChannelIncrementDirection_positive, IfxDma_ChannelIncrementCircular_4);
        }
        else
        {
            IfxDma_setChannelDestinationAddress(dmaSFR, rxDmaChannelId, (void *)IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), jobrx->data));
            IfxDma_setChannelDestinationIncrementStep(dmaSFR, rxDmaChannelId, IfxDma_ChannelIncrementStep_1,
                IfxDma_ChannelIncrementDirection_positive, IfxDma_ChannelIncrementCircular_none);
        }

        IfxDma_clearChannelInterrupt(dmaSFR, rxDmaChannelId);
        IfxQspi_clearAllEventFlags(qspiSFR);
        src = IfxQspi_getTransmitSrc(qspiSFR);
        IfxSrc_clearRequest(src);
        src = IfxQspi_getReceiveSrc(qspiSFR);
        IfxSrc_clearRequest(src);
        src = IfxQspi_getErrorSrc(qspiSFR);
        IfxSrc_clearRequest(src);

        IfxDma_clearChannelInterrupt(dmaSFR, rxDmaChannelId);
        IfxDma_clearChannelInterrupt(dmaSFR, txDmaChannelId);
        IfxDma_setChannelInterruptServiceRequest(dmaSFR, txDmaChannelId);
        IfxDma_setChannelInterruptServiceRequest(dmaSFR, rxDmaChannelId);
        IfxDma_enableChannelTransaction(dmaSFR, rxDmaChannelId);
        IfxDma_enableChannelTransaction(dmaSFR, txDmaChannelId);
        IfxDma_startChannelTransaction(dmaSFR, txDmaChannelId);

        IfxCpu_restoreInterrupts(interruptState);
    }
    else
    {
        IfxQspi_ChannelId cs = IfxQspi_ChannelId_0;

        if (job->remaining > 0)
        {
            Ifx_QSPI *qspiSFR        = handle->qspi;
            boolean   interruptState = IfxCpu_disableInterrupts();
            Ifx_SizeT count          = (Ifx_SizeT)(IFXQSPI_HWFIFO_DEPTH - 1 - IfxQspi_getTransmitFifoLevel(qspiSFR)); // -1, since BACON allocates one FIFO entry
            count = __min(job->remaining, count);

            if (count > 0)
            {
                job->remaining = job->remaining - count;

                if (job->data == NULL_PTR)
                {
                    // no data should be sent (only received): send all
                    int i;

                    for (i = 0; i < count; ++i)
                    {
                        IfxQspi_writeTransmitFifo(qspiSFR, ~0);
                    }
                }
                else
                {
                    if (handle->dataWidth <= 8)
                    {
                        IfxQspi_write8(qspiSFR, cs, job->data, count);
                        job->data = &(((uint8 *)job->data)[count]);
                    }
                    else if (handle->dataWidth <= 16)
                    {
                        IfxQspi_write16(qspiSFR, cs, job->data, count);
                        job->data = &(((uint16 *)job->data)[count]);
                    }
                    else
                    {
                        IfxQspi_write32(qspiSFR, cs, job->data, count);
                        job->data = &(((uint32 *)job->data)[count]);
                    }
                }
            }

            IfxCpu_restoreInterrupts(interruptState);
        }
    }
}
