/**
 * \file IfxGpt12_IncrEnc.c
 * \brief GPT12 INCRENC details
 *
 * \version iLLD_1_0_1_16_1_1
 * \copyright Copyright (c) 2022 Infineon Technologies AG. All rights reserved.
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

#include "IfxGpt12_IncrEnc.h"

/******************************************************************************/
/*-----------------------Private Function Prototypes--------------------------*/
/******************************************************************************/

/** \brief Update internal data when incremental mode is using T2.\n
 * This function shall be periodically called
 * \param driver driver handle
 * \return None
 */
IFX_STATIC void IfxGpt12_IncrEnc_updateFromT2(IfxGpt12_IncrEnc *driver);

/** \brief Update internal data when incremental mode is using T3.\n
 * This function shall be periodically called
 * \param driver driver handle
 * \return None
 */
IFX_STATIC void IfxGpt12_IncrEnc_updateFromT3(IfxGpt12_IncrEnc *driver);

/** \brief Updates the speed
 * \param driver driver handle
 * \param newPosition new position
 * \return None
 */
IFX_STATIC void IfxGpt12_IncrEnc_updateSpeedFromT2(IfxGpt12_IncrEnc *driver, sint32 newPosition);

/** \brief Updates the speed
 * \param driver driver handle
 * \param newPosition new position
 * \return None
 */
IFX_STATIC void IfxGpt12_IncrEnc_updateSpeedFromT3(IfxGpt12_IncrEnc *driver, sint32 newPosition);

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

float32 IfxGpt12_IncrEnc_getAbsolutePosition(IfxGpt12_IncrEnc *driver)
{
    return ((float32)driver->turn + (float32)driver->rawPosition / (float32)driver->resolution) * 2.0f * IFX_PI;
}


IfxStdIf_Pos_Dir IfxGpt12_IncrEnc_getDirection(IfxGpt12_IncrEnc *driver)
{
    return driver->direction;
}


IfxStdIf_Pos_Status IfxGpt12_IncrEnc_getFault(IfxGpt12_IncrEnc *driver)
{
    return driver->status;
}


sint32 IfxGpt12_IncrEnc_getOffset(IfxGpt12_IncrEnc *driver)
{
    return driver->offset;
}


uint16 IfxGpt12_IncrEnc_getPeriodPerRotation(IfxGpt12_IncrEnc *driver)
{
    IFX_UNUSED_PARAMETER(driver)
    return 1; /* Period per rotation is 1*/
}


float32 IfxGpt12_IncrEnc_getPosition(IfxGpt12_IncrEnc *driver)
{
    return (float32)driver->rawPosition * driver->positionConst;
}


sint32 IfxGpt12_IncrEnc_getRawPosition(IfxGpt12_IncrEnc *driver)
{
    return driver->rawPosition;
}


float32 IfxGpt12_IncrEnc_getRefreshPeriod(IfxGpt12_IncrEnc *driver)
{
    return driver->updatePeriod;
}


sint32 IfxGpt12_IncrEnc_getResolution(IfxGpt12_IncrEnc *driver)
{
    return driver->resolution;
}


IfxStdIf_Pos_SensorType IfxGpt12_IncrEnc_getSensorType(IfxGpt12_IncrEnc *driver)
{
    IFX_UNUSED_PARAMETER(driver)
    return IfxStdIf_Pos_SensorType_encoder;
}


float32 IfxGpt12_IncrEnc_getSpeed(IfxGpt12_IncrEnc *driver)
{
    return driver->speed;
}


sint32 IfxGpt12_IncrEnc_getTurn(IfxGpt12_IncrEnc *driver)
{
    return driver->turn;
}


boolean IfxGpt12_IncrEnc_init(IfxGpt12_IncrEnc *driver, const IfxGpt12_IncrEnc_Config *config)
{
    boolean    status = TRUE;
    Ifx_GPT12 *gpt12  = config->module;

    driver->module             = gpt12;

    driver->offset             = config->base.offset;
    driver->resolution         = config->base.resolution * config->base.resolutionFactor;
    driver->positionConst      = 1.0f / (float32)driver->resolution * 2.0f * IFX_PI;
    driver->speedModeThreshold = config->base.speedModeThreshold;
    IfxGpt12_IncrEnc_setRefreshPeriod(driver, config->base.updatePeriod);

    driver->status.status            = 0;
    driver->status.B.notSynchronised = 1;
    driver->minSpeed                 = config->base.minSpeed;
    driver->maxSpeed                 = config->base.maxSpeed;

    driver->rawPosition              = 0;
    driver->speed                    = 0;
    driver->direction                = IfxStdIf_Pos_Dir_unknown;
    driver->turn                     = 0;

    if (config->pinA->timer == 3)
    {
        /* T3 Configuration */
        IfxGpt12_T3_setMode(gpt12, IfxGpt12_Mode_incrementalInterfaceEdgeDetection);

        switch (config->base.resolutionFactor)
        {
        case IfxStdIf_Pos_ResolutionFactor_twoFold:
            IfxGpt12_T3_setIncrementalInterfaceInputMode(gpt12, IfxGpt12_IncrementalInterfaceInputMode_bothEdgesTxIN);
            break;
        case IfxStdIf_Pos_ResolutionFactor_fourFold:
            IfxGpt12_T3_setIncrementalInterfaceInputMode(gpt12, IfxGpt12_IncrementalInterfaceInputMode_bothEdgesTxINOrTxEUD);
            break;
        default:
            status = FALSE;
            break;
        }

        IfxGpt12_T3_setDirectionSource(gpt12, IfxGpt12_TimerDirectionSource_external);
        IfxGpt12_T3_setTimerDirection(gpt12, config->base.reversed ? IfxGpt12_TimerDirection_down : IfxGpt12_TimerDirection_up);
        IfxGpt12_T3_enableOutput(gpt12, FALSE);
        IfxGpt12_T3_run(gpt12, IfxGpt12_TimerRun_start);

        if (config->pinZ != NULL_PTR)
        {                       /* Only configure T4 if zero signal is available. Zero has to be on T4IN  */
            /* T4 Configuration */
            IfxGpt12_T4_setMode(gpt12, IfxGpt12_Mode_capture);
            IfxGpt12_T4_setCaptureInputMode(gpt12, IfxGpt12_CaptureInputMode_risingEdgeTxIN);
            IfxGpt12_T4_enableClearTimerT2(gpt12, FALSE);
            IfxGpt12_T4_enableClearTimerT3(gpt12, TRUE);
            IfxGpt12_T4_setInterruptEnable(gpt12, config->zeroIsrPriority != 0);
            IfxGpt12_T4_setRemoteControl(gpt12, IfxGpt12_TimerRemoteControl_off);
            IfxGpt12_T4_run(gpt12, IfxGpt12_TimerRun_stop);

            if (config->zeroIsrPriority)
            {
                /* setup interrupt */
                volatile Ifx_SRC_SRCR *src = IfxGpt12_T4_getSrc(gpt12);
                IfxSrc_init(src, config->zeroIsrProvider, config->zeroIsrPriority);
                IfxSrc_enable(src);
            }
        }

        /* T5 Configuration */
        IfxGpt12_T5_setMode(gpt12, IfxGpt12_Mode_timer);
        IfxGpt12_T5_setTimerPrescaler(gpt12, IfxGpt12_TimerInputPrescaler_4);
        IfxGpt12_T5_setCaptureTrigger(gpt12, IfxGpt12_CaptureTrigger_t3inOrT3EUD);
        IfxGpt12_T5_setCaptureTriggerMode(gpt12, IfxGpt12_CaptureTriggerMode_risingEdge);
        IfxGpt12_T5_enableClearTimer(gpt12, TRUE);
        IfxGpt12_T5_setCaptureTriggerEnable(gpt12, TRUE);
        IfxGpt12_T5_setRemoteControl(gpt12, IfxGpt12_TimerRemoteControl_off);
        IfxGpt12_T5_setDirectionSource(gpt12, IfxGpt12_TimerDirectionSource_internal);
        IfxGpt12_T5_setTimerDirection(gpt12, IfxGpt12_TimerDirection_up);
        IfxGpt12_T5_run(gpt12, IfxGpt12_TimerRun_start);

        driver->update = (IfxGpt12_IncrEnc_Update) & IfxGpt12_IncrEnc_updateFromT3;
    }
    else if (config->pinA->timer == 2)
    {
        /* T2 Configuration */
        IfxGpt12_T2_setMode(gpt12, IfxGpt12_Mode_incrementalInterfaceEdgeDetection);

        switch (config->base.resolutionFactor)
        {
        case IfxStdIf_Pos_ResolutionFactor_twoFold:
            IfxGpt12_T2_setIncrementalInterfaceInputMode(gpt12, IfxGpt12_IncrementalInterfaceInputMode_bothEdgesTxIN);
            break;
        case IfxStdIf_Pos_ResolutionFactor_fourFold:
            IfxGpt12_T2_setIncrementalInterfaceInputMode(gpt12, IfxGpt12_IncrementalInterfaceInputMode_bothEdgesTxINOrTxEUD);
            break;
        default:
            status = FALSE;
            break;
        }

        IfxGpt12_T2_setDirectionSource(gpt12, IfxGpt12_TimerDirectionSource_external);
        IfxGpt12_T2_setTimerDirection(gpt12, config->base.reversed ? IfxGpt12_TimerDirection_down : IfxGpt12_TimerDirection_up);
        IfxGpt12_T2_run(gpt12, IfxGpt12_TimerRun_start);

        if (config->pinZ != NULL_PTR)
        {                       /* Only configure T4 if zero signal is available. Zero has to be on T4EUD  */
            /* T4 Configuration */
            IfxGpt12_T4_setMode(gpt12, IfxGpt12_Mode_capture);
            IfxGpt12_T4_setCaptureInputMode(gpt12, IfxGpt12_CaptureInputMode_none);
            IfxGpt12_T4_enableClearTimerT2(gpt12, TRUE);
            IfxGpt12_T4_enableClearTimerT3(gpt12, FALSE);
            IfxGpt12_T4_setInterruptEnable(gpt12, FALSE);
            IfxGpt12_T4_setRemoteControl(gpt12, IfxGpt12_TimerRemoteControl_off);
            IfxGpt12_T4_run(gpt12, IfxGpt12_TimerRun_stop);
        }

        driver->update = (IfxGpt12_IncrEnc_Update) & IfxGpt12_IncrEnc_updateFromT2;
    }

    if (config->initPins == TRUE)
    {
        IfxGpt12_initTxInPinWithPadLevel(config->pinA, config->pinMode, config->pinDriver);
        IfxGpt12_initTxEudInPinWithPadLevel(config->pinB, config->pinMode, config->pinDriver);

        if (config->pinZ != NULL_PTR)
        {
            IfxGpt12_initTxInPinWithPadLevel(config->pinZ, config->pinMode, config->pinDriver);
        }
    }

    driver->speedConstTimeDiff =
        (2.0f * IFX_PI) / (config->base.resolution * 2) * IfxGpt12_T5_getFrequency(gpt12);

    return status;
}


void IfxGpt12_IncrEnc_initConfig(IfxGpt12_IncrEnc_Config *config, Ifx_GPT12 *gpt12)
{
    IfxStdIf_Pos_initConfig(&config->base);
    config->base.resolutionFactor          = IfxStdIf_Pos_ResolutionFactor_twoFold;
    config->base.minSpeed                  = 1.0f / 60.0f * (2 * IFX_PI);     // 1 rpm
    config->base.maxSpeed                  = 20000.0f / 60.0f * (2 * IFX_PI); // 20000 rpm
    config->base.speedFilerCutOffFrequency = config->base.maxSpeed / 2 * IFX_PI * 10;

    config->pinA                           = NULL_PTR;
    config->pinB                           = NULL_PTR;
    config->pinZ                           = NULL_PTR;
    config->pinMode                        = IfxPort_InputMode_noPullDevice;
    config->module                         = gpt12;
    config->zeroIsrPriority                = 0;
    config->zeroIsrProvider                = IfxSrc_Tos_cpu0;
    config->pinDriver                      = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    config->initPins                       = TRUE;
}


void IfxGpt12_IncrEnc_onZeroIrq(IfxGpt12_IncrEnc *driver)
{
    if (driver->status.B.notSynchronised)
    {
        driver->status.B.notSynchronised = 0;
    }

    if (driver->direction == IfxStdIf_Pos_Dir_forward)
    {
        driver->turn++;
    }
    else
    {
        driver->turn--;
    }
}


void IfxGpt12_IncrEnc_reset(IfxGpt12_IncrEnc *driver)
{
    driver->rawPosition              = 0;
    driver->turn                     = 0;
    driver->speed                    = 0;
    driver->status.status            = 0;
    driver->status.B.notSynchronised = 1;
}


void IfxGpt12_IncrEnc_resetFaults(IfxGpt12_IncrEnc *driver)
{
    IfxStdIf_Pos_Status status;
    status.status            = 0;
    status.B.notSynchronised = driver->status.B.notSynchronised;
    driver->status.status    = status.status;
}


void IfxGpt12_IncrEnc_setOffset(IfxGpt12_IncrEnc *driver, sint32 offset)
{
    driver->offset                   = offset;
    driver->status.B.notSynchronised = 0;
}


void IfxGpt12_IncrEnc_setRefreshPeriod(IfxGpt12_IncrEnc *driver, float32 updatePeriod)
{
    driver->updatePeriod           = updatePeriod;
    driver->speedConstPulseCount   = (2.0f * IFX_PI) / driver->resolution / updatePeriod;
    driver->speedModeThresholdTick = driver->speedModeThreshold * driver->resolution * updatePeriod / (2.0f * IFX_PI);
}


boolean IfxGpt12_IncrEnc_stdIfPosInit(IfxStdIf_Pos *stdif, IfxGpt12_IncrEnc *driver)
{
    /* Ensure the stdif is reset to zeros */
    memset(stdif, 0, sizeof(IfxStdIf_Pos));

    /* Set the driver */
    stdif->driver = driver;

    /* *INDENT-OFF* Note: this file was indented manually by the author. */
    /* Set the API link */
	stdif->onZeroIrq          =(IfxStdIf_Pos_OnZeroIrq               )&IfxGpt12_IncrEnc_onZeroIrq;
	stdif->getAbsolutePosition=(IfxStdIf_Pos_GetAbsolutePosition     )&IfxGpt12_IncrEnc_getAbsolutePosition;
	stdif->getDirection		  =(IfxStdIf_Pos_GetDirection            )&IfxGpt12_IncrEnc_getDirection;
	stdif->getFault           =(IfxStdIf_Pos_GetFault                )&IfxGpt12_IncrEnc_getFault;
	stdif->getOffset		  =(IfxStdIf_Pos_GetOffset			     )&IfxGpt12_IncrEnc_getOffset;
	stdif->getPeriodPerRotation  =(IfxStdIf_Pos_GetPeriodPerRotation )&IfxGpt12_IncrEnc_getPeriodPerRotation;
	stdif->getPosition		  =(IfxStdIf_Pos_GetPosition			 )&IfxGpt12_IncrEnc_getPosition;
	stdif->getRawPosition	  =(IfxStdIf_Pos_GetRawPosition         )&IfxGpt12_IncrEnc_getRawPosition;
	stdif->getRefreshPeriod   =(IfxStdIf_Pos_GetRefreshPeriod        )&IfxGpt12_IncrEnc_getRefreshPeriod;
	stdif->getResolution      =(IfxStdIf_Pos_GetResolution           )&IfxGpt12_IncrEnc_getResolution;
    stdif->getSensorType      =(IfxStdIf_Pos_GetSensorType           )&IfxGpt12_IncrEnc_getSensorType;
	stdif->reset			  =(IfxStdIf_Pos_Reset				     )&IfxGpt12_IncrEnc_reset;
	stdif->resetFaults		  =(IfxStdIf_Pos_ResetFaults			 )&IfxGpt12_IncrEnc_resetFaults;
	stdif->getSpeed           =(IfxStdIf_Pos_GetSpeed                )&IfxGpt12_IncrEnc_getSpeed;
	stdif->update			  =(IfxStdIf_Pos_Update				     )&IfxGpt12_IncrEnc_update;
	stdif->setOffset		  =(IfxStdIf_Pos_SetOffset			     )&IfxGpt12_IncrEnc_setOffset;
	stdif->setRefreshPeriod   =(IfxStdIf_Pos_SetRefreshPeriod        )&IfxGpt12_IncrEnc_setRefreshPeriod;
	stdif->getTurn            =(IfxStdIf_Pos_GetTurn                 )&IfxGpt12_IncrEnc_getTurn;
    /* *INDENT-ON* */

    return TRUE;
}


void IfxGpt12_IncrEnc_update(IfxGpt12_IncrEnc *driver)
{
    driver->update(driver);
}


IFX_STATIC void IfxGpt12_IncrEnc_updateFromT2(IfxGpt12_IncrEnc *driver)
{
    Ifx_GPT12 *gpt12 = driver->module;
    sint32     newPosition;
    driver->direction = gpt12->T2CON.B.T2RDIR ? IfxStdIf_Pos_Dir_backward : IfxStdIf_Pos_Dir_forward;

    newPosition       = gpt12->T2.U;

    newPosition       = (newPosition + driver->offset);

    if (newPosition >= driver->resolution)
    {
        newPosition %= driver->resolution;
    }
    else if (newPosition < 0)
    {
        newPosition = (newPosition + driver->resolution);
    }

    IfxGpt12_IncrEnc_updateSpeedFromT2(driver, newPosition);
    driver->rawPosition = newPosition;
}


IFX_STATIC void IfxGpt12_IncrEnc_updateFromT3(IfxGpt12_IncrEnc *driver)
{
    Ifx_GPT12 *gpt12 = driver->module;
    sint32     newPosition;
    driver->direction = gpt12->T3CON.B.T3RDIR ? IfxStdIf_Pos_Dir_backward : IfxStdIf_Pos_Dir_forward;

    newPosition       = gpt12->T3.U;

    newPosition       = (newPosition + driver->offset);

    if (newPosition >= driver->resolution)
    {
        newPosition %= driver->resolution;
    }
    else if (newPosition < 0)
    {
        newPosition = (newPosition + driver->resolution);
    }

    IfxGpt12_IncrEnc_updateSpeedFromT3(driver, newPosition);
    driver->rawPosition = newPosition;
}


IFX_STATIC void IfxGpt12_IncrEnc_updateSpeedFromT2(IfxGpt12_IncrEnc *driver, sint32 newPosition)
{
    float32 speed;
    sint32  diff;

    if (driver->direction == IfxStdIf_Pos_Dir_forward)
    {
        diff = newPosition - driver->rawPosition;
    }
    else
    {
        diff = driver->rawPosition - newPosition;
    }

    if (diff < 0)
    {
        diff += driver->resolution;
    }

    speed         = diff * driver->speedConstPulseCount;

    speed         = driver->direction == IfxStdIf_Pos_Dir_forward ? speed : -speed;
    driver->speed = speed;
}


IFX_STATIC void IfxGpt12_IncrEnc_updateSpeedFromT3(IfxGpt12_IncrEnc *driver, sint32 newPosition)
{
    float32 speed;
    sint32  diff;

    if (driver->direction == IfxStdIf_Pos_Dir_forward)
    {
        diff = newPosition - driver->rawPosition;
    }
    else
    {
        diff = driver->rawPosition - newPosition;
    }

    if (diff < 0)
    {
        diff += driver->resolution;
    }

    if (diff > driver->speedModeThresholdTick)
    {                           /* Use pulse count mode ( Fast speed ) */
        speed = diff * driver->speedConstPulseCount;
    }
    else
    {                           /* Use time diff mode (slow speed), only if T3 is used as core */
        Ifx_GPT12             *gpt12 = driver->module;
        volatile Ifx_SRC_SRCR *srcT5 = IfxGpt12_T5_getSrc(gpt12);

        if (srcT5->B.SRR != 1)
        {                       // NO overflow of T5
            volatile Ifx_SRC_SRCR *srcCap = IfxGpt12_getCaptureSrc(gpt12);

            // Check if a new value is captured
            if (srcCap->B.SRR != 0)
            {
                // Delete Capture Request Bit
                srcCap->B.CLRR = 1;
                speed          = driver->speedConstTimeDiff / gpt12->CAPREL.B.CAPREL;
            }
            else
            {
                speed = driver->speed;
            }
        }
        else
        {                       // T5 overflow detected
            // Delete Overflow Request bit
            srcT5->B.CLRR = 1;
            speed         = 0.0f;
        }
    }

    speed         = driver->direction == IfxStdIf_Pos_Dir_forward ? speed : -speed;
    driver->speed = speed;
}
