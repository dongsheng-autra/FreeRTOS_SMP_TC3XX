/**
 * \file IfxGtm_Tom_Dtm_PwmHl.c
 * \brief GTM DTM_PWMHL details
 *
 * \version iLLD_1_0_1_16_1_1
 * \copyright Copyright (c) 2021 Infineon Technologies AG. All rights reserved.
 *
 *
 *
 *                                 IMPORTANT NOTICE
 *
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees. If and as long as no such
 * terms of use are agreed, use of this file is subject to following:
 *
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or
 * organization obtaining a copy of the software and accompanying
 * documentation covered by this license (the "Software") to use, reproduce,
 * display, distribute, execute, and transmit the Software, and to prepare
 * derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
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

#include "IfxGtm_Tom_Dtm_PwmHl.h"
#include "_Utilities/Ifx_Assert.h"
#include "stddef.h"
#include "string.h"
#include "IfxGtm_Tom_PwmHl.h"

/******************************************************************************/
/*------------------------Inline Function Prototypes--------------------------*/
/******************************************************************************/

/** \brief Sets the ON time
 * \return None
 */
IFX_INLINE void IfxGtm_Tom_Dtm_PwmHl_setOnTime(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn);

/** \brief Inverts the active state
 * \param activeState Active state
 * \return State
 */
IFX_INLINE Ifx_ActiveState IfxGtm_Tom_Dtm_PwmHl_invertActiveState(Ifx_ActiveState activeState);

/******************************************************************************/
/*-----------------------Private Function Prototypes--------------------------*/
/******************************************************************************/

/** \brief Sets switched to OFF
 * \param driver driver GTM TOM PWM driver
 * \param tOn ON time
 * \param shift Shift value in ticks
 * \return None
 */
IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateAndShiftOff(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn, Ifx_TimerValue *shift);

/** \brief Updates the x output duty cycle in center aligned and center aligned inverted modes
 * \param driver GTM TOM PWM driver
 * \param tOn T on
 * \return None
 */
IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateCenterAligned(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn);

/** \brief Updates the x output duty cycle in edge aligned modes (left and right aligned)
 * \param driver GTM TOM PWM driver
 * \param tOn T On
 * \return None
 */
IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateEdgeAligned(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn);

/** \brief Set the outputs to inactive
 * \param driver GTM TOM PWM driver
 * \param tOn T On
 * \return None
 */
IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateOff(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn);

/** \brief Update Pulse
 * \param driver GTM TOM PWM driver
 * \param tOn ON times. Phase 0 top, phase 1 top, ... phase 0 bottom, phase 1 botteom, ...
 * \param offset Offset value in ticks. Phase 0 top, phase 1 top, ... phase 0 bottom, phase 1 botteom, ...
 * \return None
 */
IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updatePulse(IfxGtm_TomDtm_PwmHl *driver, float32 *tOn, float32 *offset);

/** \brief Set Pulse to OFF
 * \param driver GTM TOM PWM driver
 * \param tOn ON times. Phase 0 top, phase 1 top, ... phase 0 bottom, phase 1 botteom, ...
 * \param offset Offset value in ticks. Phase 0 top, phase 1 top, ... phase 0 bottom, phase 1 botteom, ...
 * \return None
 */
IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updatePulseOff(IfxGtm_TomDtm_PwmHl *driver, float32 *tOn, float32 *offset);

/** \brief Update Shift Center Aligned
 * \param driver GTM TOM PWM driver
 * \param tOn ON time
 * \param shift Shift value in ticks
 * \return None
 */
IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateShiftCenterAligned(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn, Ifx_TimerValue *shift);

/**
 */
IFX_STATIC Ifx_TimerValue IfxGtm_Tom_Dtm_PwmHl_getDeadtimeInTimerClockTicks(IfxGtm_TomDtm_PwmHl *driver);

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

IFX_STATIC IFX_CONST IfxGtm_Tom_Dtm_PwmHl_Mode IfxGtm_Tom_Dtm_PwmHl_modes[Ifx_Pwm_Mode_off + 1] = {
    {Ifx_Pwm_Mode_centerAligned,         FALSE, &IfxGtm_Tom_Dtm_PwmHl_updateCenterAligned, &IfxGtm_Tom_Dtm_PwmHl_updateShiftCenterAligned, &IfxGtm_Tom_Dtm_PwmHl_updatePulse   },
    {Ifx_Pwm_Mode_centerAlignedInverted, TRUE,  &IfxGtm_Tom_Dtm_PwmHl_updateCenterAligned, &IfxGtm_Tom_Dtm_PwmHl_updateShiftCenterAligned, &IfxGtm_Tom_Dtm_PwmHl_updatePulse   },
    {Ifx_Pwm_Mode_leftAligned,           FALSE, &IfxGtm_Tom_Dtm_PwmHl_updateEdgeAligned,   &IfxGtm_Tom_Dtm_PwmHl_updateAndShiftOff,        &IfxGtm_Tom_Dtm_PwmHl_updatePulse   },
    {Ifx_Pwm_Mode_rightAligned,          TRUE,  &IfxGtm_Tom_Dtm_PwmHl_updateEdgeAligned,   &IfxGtm_Tom_Dtm_PwmHl_updateAndShiftOff,        &IfxGtm_Tom_Dtm_PwmHl_updatePulse   },
    {Ifx_Pwm_Mode_off,                   FALSE, &IfxGtm_Tom_Dtm_PwmHl_updateOff,           &IfxGtm_Tom_Dtm_PwmHl_updateAndShiftOff,        &IfxGtm_Tom_Dtm_PwmHl_updatePulseOff},
};

/******************************************************************************/
/*---------------------Inline Function Implementations------------------------*/
/******************************************************************************/

IFX_INLINE void IfxGtm_Tom_Dtm_PwmHl_setOnTime(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn)
{
    driver->update(driver, tOn);
}


IFX_INLINE Ifx_ActiveState IfxGtm_Tom_Dtm_PwmHl_invertActiveState(Ifx_ActiveState activeState)
{
    return activeState == Ifx_ActiveState_low ? Ifx_ActiveState_high : Ifx_ActiveState_low;
}


/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

boolean IfxGtm_Tom_Dtm_PwmHl_init(IfxGtm_TomDtm_PwmHl *driver, const IfxGtm_Tom_Dtm_PwmHl_Config *config)
{
    boolean           result       = TRUE;
    uint16            channelMask;
    uint16            channelsMask = 0;
    uint32            channelIndex;

    IfxGtm_Tom_Timer *timer = config->timer;

    driver->base.mode             = Ifx_Pwm_Mode_init;
    driver->timer                 = timer;
    driver->base.setMode          = 0;
    driver->base.inverted         = FALSE;
    driver->base.ccxActiveState   = config->base.ccxActiveState;
    driver->base.coutxActiveState = config->base.coutxActiveState;
    driver->base.channelCount     = config->base.channelCount;

    driver->tom                   = &(timer->gtm->TOM[config->tom]);

    driver->tgc                   = (Ifx_GTM_TOM_TGC *)&driver->tom->TGC0_GLB_CTRL;

    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config->base.channelCount <= IFXGTM_TOM_PWMHL_MAX_NUM_CHANNELS);

    IfxGtm_Cmu_Clk clock = IfxGtm_Tom_Ch_getClockSource(timer->tom, timer->timerChannel);

    for (channelIndex = 0; channelIndex < config->base.channelCount; channelIndex++)
    {
        IfxGtm_Tom_Ch channel;

        /* Initialise the Deadtime generation */
        driver->dtm[channelIndex] = IfxGtm_Dtm_getDtmPointer(timer->gtm, IfxGtm_Dtm_getCdtmIndex(config->tom), IfxGtm_Dtm_getDtmIndex(config->tom, config->ccx[channelIndex]->channel));
        result                   &= driver->dtm[channelIndex] != NULL_PTR;

        IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, driver->dtm[channelIndex] != NULL_PTR);

        if (driver->dtm[channelIndex] != NULL_PTR)
        {
            /* CCX */
            channel                   = config->ccx[channelIndex]->channel;
            driver->ccx[channelIndex] = channel;
            channelMask               = 1 << channel;
            channelsMask             |= channelMask;

            /* Initialize the timer part */
            IfxGtm_Tom_Ch_setClockSource(driver->tom, channel, clock);
            IfxGtm_Tom_Ch_setResetSource(driver->tom, channel, IfxGtm_Tom_Ch_ResetEvent_onTrigger);
            IfxGtm_Tom_Ch_setSignalLevel(driver->tom, channel, config->base.ccxActiveState);
            IfxGtm_Tom_Ch_setTriggerOutput(driver->tom, channel, IfxGtm_Tom_Ch_OutputTrigger_forward);

            /* Initialize the DTM part */
            IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, IfxGtm_Dtm_getDtmChannel(config->ccx[channelIndex]->channel) == IfxGtm_Dtm_getDtmChannel(config->coutx[channelIndex]->channel));

            IfxGtm_Dtm_setClockSource(driver->dtm[channelIndex], config->deadTimeClock);
            driver->dtmChannel[channelIndex] = IfxGtm_Dtm_getDtmChannel(config->ccx[channelIndex]->channel);
            IfxGtm_Dtm_setOutput0DeadTimePath(driver->dtm[channelIndex], driver->dtmChannel[channelIndex], IfxGtm_Dtm_DeadTimePath_enable);
            IfxGtm_Dtm_setOutput1DeadTimePath(driver->dtm[channelIndex], driver->dtmChannel[channelIndex], IfxGtm_Dtm_DeadTimePath_enable);

            /* Initialize the port */
            if (config->initPins == TRUE)
            {
                IfxGtm_PinMap_setTomTout(config->ccx[channelIndex],
                    ((config->base.ccxOutputEnabled & (0x1 << channelIndex)) != 0) ? config->base.outputMode : IfxPort_OutputMode_none, config->base.outputDriver);
            }

            /* COUTX */
            channel                     = config->coutx[channelIndex]->channel;
            driver->coutx[channelIndex] = channel;

            /* Initialize the port */
            if (config->initPins == TRUE)
            {
                IfxGtm_PinMap_setTomTout(config->coutx[channelIndex],
                    ((config->base.coutxOutputEnabled & (0x1 << channelIndex)) != 0) ? config->base.outputMode : IfxPort_OutputMode_none, config->base.outputDriver);
            }
        }
    }

    /* Initialize the clock value, assumes all DTM are connected to the same clock */
    if (config->base.channelCount > 0)
    {
        driver->dtmClockFreq = IfxGtm_Dtm_getClockFrequency(timer->gtm, IfxGtm_Dtm_getCdtmIndex(config->tom), IfxGtm_Dtm_getDtmIndex(config->tom, config->ccx[0]->channel));
        result              &= IfxGtm_Tom_Dtm_PwmHl_setDeadtime(driver, config->base.deadtime);
    }
    else
    {
        driver->dtmClockFreq = 0;
    }

    IfxGtm_Tom_Dtm_PwmHl_setMinPulse(driver, config->base.minPulse); // must be called after IfxGtm_TomDtm_PwmHl_setDeadtime

    IfxGtm_Tom_Tgc_enableChannelsOutput(driver->tgc, channelsMask, 0, FALSE);
    IfxGtm_Tom_Tgc_enableChannels(driver->tgc, channelsMask, 0, FALSE);

    IfxGtm_Tom_Dtm_PwmHl_setMode(driver, Ifx_Pwm_Mode_off);

    Ifx_TimerValue tOn[IFXGTM_TOM_DTM_PWMHL_MAX_NUM_CHANNELS] = {0};
    IfxGtm_Tom_Dtm_PwmHl_updateOff(driver, tOn); /* tOn do not need defined values */

    /* Transfer the shadow registers */
    IfxGtm_Tom_Tgc_setChannelsForceUpdate(driver->tgc, channelsMask, 0, 0, 0);
    IfxGtm_Tom_Tgc_trigger(driver->tgc);
    IfxGtm_Tom_Tgc_setChannelsForceUpdate(driver->tgc, 0, channelsMask, 0, 0);

    /* Enable timer to update the channels */

    for (channelIndex = 0; channelIndex < driver->base.channelCount; channelIndex++)
    {
        IfxGtm_Tom_Timer_addToChannelMask(timer, driver->ccx[channelIndex]);
    }

    return result;
}


void IfxGtm_Tom_Dtm_PwmHl_initConfig(IfxGtm_Tom_Dtm_PwmHl_Config *config)
{
    IfxStdIf_PwmHl_initConfig(&config->base);
    config->timer         = NULL_PTR;
    config->tom           = IfxGtm_Tom_0;
    config->ccx           = NULL_PTR;
    config->coutx         = NULL_PTR;
    config->deadTimeClock = IfxGtm_Dtm_ClockSource_systemClock;
    config->initPins      = TRUE;
}


float32 IfxGtm_Tom_Dtm_PwmHl_getDeadtime(IfxGtm_TomDtm_PwmHl *driver)
{
    return IfxStdIf_Timer_tickToS(driver->dtmClockFreq, driver->base.deadtime);
}


Ifx_TickTime IfxGtm_Tom_Dtm_PwmHl_getDeadtimeTick(IfxGtm_TomDtm_PwmHl *driver)
{
    return driver->base.deadtime;
}


float32 IfxGtm_Tom_Dtm_PwmHl_getMinPulse(IfxGtm_TomDtm_PwmHl *driver)
{
    return IfxStdIf_Timer_tickToS(driver->timer->base.clockFreq, driver->base.minPulse - IfxGtm_Tom_Dtm_PwmHl_getDeadtimeInTimerClockTicks(driver));
}


Ifx_Pwm_Mode IfxGtm_Tom_Dtm_PwmHl_getMode(IfxGtm_TomDtm_PwmHl *driver)
{
    return driver->base.mode;
}


boolean IfxGtm_Tom_Dtm_PwmHl_setDeadtime(IfxGtm_TomDtm_PwmHl *driver, float32 deadtime)
{
    uint32         channelIndex;
    Ifx_TimerValue value = IfxStdIf_Timer_sToTick(driver->dtmClockFreq, deadtime);

    if (value > IFXGTM_TOM_DTM_PWMHL_MAX_DEADTIME_IN_TICKS)
    {
        return FALSE;
    }
    else
    {
        driver->base.deadtime = value;

        for (channelIndex = 0; channelIndex < driver->base.channelCount; channelIndex++)
        {
            IfxGtm_Dtm_setDeadtime(driver->dtm[channelIndex], driver->dtmChannel[channelIndex], value, value);
        }
    }

    return TRUE;
}


boolean IfxGtm_Tom_Dtm_PwmHl_setMinPulse(IfxGtm_TomDtm_PwmHl *driver, float32 minPulse)
{
    Ifx_TimerValue value = IfxStdIf_Timer_sToTick(driver->timer->base.clockFreq, minPulse);

    driver->base.minPulse = value + IfxGtm_Tom_Dtm_PwmHl_getDeadtimeInTimerClockTicks(driver);
    driver->base.maxPulse = driver->timer->base.period - driver->base.minPulse;

    return TRUE;
}


boolean IfxGtm_Tom_Dtm_PwmHl_setMode(IfxGtm_TomDtm_PwmHl *driver, Ifx_Pwm_Mode mode)
{
    boolean                    result = TRUE;
    IfxGtm_Tom_Dtm_PwmHl_Base *base   = &driver->base;

    if (base->mode != mode)
    {
        if ((mode > Ifx_Pwm_Mode_off) || (IfxGtm_Tom_Dtm_PwmHl_modes[mode].update == NULL_PTR))
        {
            mode   = Ifx_Pwm_Mode_off;
            result = FALSE;
        }

        IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, mode == IfxGtm_Tom_Dtm_PwmHl_modes[mode].mode);

        base->mode             = mode;
        driver->update         = IfxGtm_Tom_Dtm_PwmHl_modes[mode].update;
        driver->updateAndShift = IfxGtm_Tom_Dtm_PwmHl_modes[mode].updateAndShift;
        driver->updatePulse    = IfxGtm_Tom_Dtm_PwmHl_modes[mode].updatePulse;

        if (base->mode != Ifx_Pwm_Mode_off)
        {
            base->inverted = IfxGtm_Tom_Dtm_PwmHl_modes[mode].inverted;
        }
        else
        {   /* Keep previous inverted for off mode */
        }

        if (base->inverted)
        {
            // TODO update DTM to invert, user SR
        }
        else
        {
            // TODO update DTM to invert, user SR
        }
    }

    return result;
}


void IfxGtm_Tom_Dtm_PwmHl_setOnTimeAndShift(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn, Ifx_TimerValue *shift)
{
    driver->updateAndShift(driver, tOn, shift);
}


void IfxGtm_Tom_Dtm_PwmHl_setPulse(IfxGtm_TomDtm_PwmHl *driver, float32 *tOn, float32 *offset)
{
    driver->updatePulse(driver, tOn, offset);
}


void IfxGtm_Tom_Dtm_PwmHl_setupChannels(IfxGtm_TomDtm_PwmHl *driver, boolean *activeCh, boolean *stuckSt)
{
    /* Dummy Function for StdIf Compile*/
    IFX_UNUSED_PARAMETER(driver)
    IFX_UNUSED_PARAMETER(activeCh)
    IFX_UNUSED_PARAMETER(stuckSt)
}


boolean IfxGtm_Tom_Dtm_PwmHl_stdIfPwmHlInit(IfxStdIf_PwmHl *stdif, IfxGtm_TomDtm_PwmHl *driver)
{
    /* Ensure the stdif is reset to zeros */
    memset(stdif, 0, sizeof(IfxStdIf_PwmHl));

    /* *INDENT-OFF* Note: this file was indented manually by the author. */
    /* Set the API link */
    stdif->driver               = driver;
    stdif->setDeadtime          = (IfxStdIf_PwmHl_SetDeadtime      )&IfxGtm_Tom_Dtm_PwmHl_setDeadtime;
    stdif->getDeadtime          = (IfxStdIf_PwmHl_GetDeadtime      )&IfxGtm_Tom_Dtm_PwmHl_getDeadtime;
    stdif->getDeadtimeTick      = (IfxStdIf_PwmHl_GetDeadtimeTick  )&IfxGtm_Tom_Dtm_PwmHl_getDeadtimeTick ;
    stdif->setMinPulse          = (IfxStdIf_PwmHl_SetMinPulse      )&IfxGtm_Tom_Dtm_PwmHl_setMinPulse;
    stdif->getMinPulse          = (IfxStdIf_PwmHl_GetMinPulse      )&IfxGtm_Tom_Dtm_PwmHl_getMinPulse;
    stdif->getMode              = (IfxStdIf_PwmHl_GetMode          )&IfxGtm_Tom_Dtm_PwmHl_getMode;
    stdif->setMode              = (IfxStdIf_PwmHl_SetMode          )&IfxGtm_Tom_Dtm_PwmHl_setMode;
    stdif->setOnTime            = (IfxStdIf_PwmHl_SetOnTime        )&IfxGtm_Tom_Dtm_PwmHl_setOnTime;
    stdif->setOnTimeAndShift    = (IfxStdIf_PwmHl_SetOnTimeAndShift)&IfxGtm_Tom_Dtm_PwmHl_setOnTimeAndShift;
    stdif->setPulse             = (IfxStdIf_PwmHl_SetPulse         )&IfxGtm_Tom_Dtm_PwmHl_setPulse;
    stdif->setupChannels        = (IfxStdIf_PwmHl_SetupChannels    )&IfxGtm_Tom_Dtm_PwmHl_setupChannels;
    IfxGtm_Tom_Timer_stdIfTimerInit(&stdif->timer, driver->timer);
    /* *INDENT-ON* */

    return TRUE;
}


IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateAndShiftOff(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn, Ifx_TimerValue *shift)
{
    IfxGtm_Tom_Dtm_PwmHl_updateOff(driver, NULL_PTR);
    IFX_UNUSED_PARAMETER(tOn)
    IFX_UNUSED_PARAMETER(shift)
}


IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateCenterAligned(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn)
{
    /* FIXME make use of the Up Down counter mode */
    uint8          channelIndex;
    Ifx_TimerValue period;

    period = driver->timer->base.period;

    for (channelIndex = 0; channelIndex < driver->base.channelCount; channelIndex++)
    {
        Ifx_TimerValue x; /* x=period*dutyCycle, x=OnTime+deadTime */
        x = tOn[channelIndex];

        if (driver->base.inverted != FALSE)
        {
            x = period - x;
        }
        else
        {}

        if (x < driver->base.minPulse)
        {   /* For deadtime condition: avoid leading edge of top channel to occur after the trailing edge */
            x = 0;
        }
        else if (x > driver->base.maxPulse)
        {
            x = period;
        }
        else
        {}

        if (x == 0)
        {   /* 0% duty cycle */
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], period + 1, period + 1);
        }
        else
        {                           /* x% duty cycle and 100 % duty cycle  */
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], (period - x) / 2, (period + x) / 2);
        }
    }
}


IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateEdgeAligned(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn)
{
    uint8          channelIndex;
    Ifx_TimerValue period;

    period = driver->timer->base.period;

    for (channelIndex = 0; channelIndex < driver->base.channelCount; channelIndex++)
    {
        Ifx_TimerValue x; /* x=period*dutyCycle, x=OnTime+deadTime */
        Ifx_TimerValue cm0, cm1;
        x = tOn[channelIndex];

        if (driver->base.inverted != FALSE)
        {
            x = period - x;
        }
        else
        {}

        if (x < driver->base.minPulse)
        {   /* For deadtime condition: avoid leading edge of top channel to occur after the trailing edge */
            x = 0;
        }
        else if (x > driver->base.maxPulse)
        {
            x = period;
        }
        else
        {}

        /* Special handling due to GTM issue */
        if (x == period)   /* 100% duty cycle */
        {
            cm0 = 0;
            cm1 = period + 1;
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], cm0, cm1);
        }
        else if (x == 0)
        {
            cm0 = period + 1;
            cm1 = 0;
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], cm0, cm1);
        }
        else
        {
            /* x% duty cycle */

            if (driver->base.mode == Ifx_Pwm_Mode_leftAligned)
            {
                cm0 = 0;
                cm1 = x;
            }
            else
            {
                // right aligned
                cm0 = x;
                cm1 = 0;
            }

            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], cm0, cm1);
        }
    }
}


IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateOff(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn)
{
    IFX_UNUSED_PARAMETER(tOn)
    uint8 channelIndex;
    Ifx_TimerValue period;
    period = driver->timer->base.period;

    for (channelIndex = 0; channelIndex < driver->base.channelCount; channelIndex++)
    {
        /* 0% duty cycle */
        IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], period + 1, period + 1);
    }
}


IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updatePulse(IfxGtm_TomDtm_PwmHl *driver, float32 *tOn, float32 *offset)
{
    uint8          channelIndex;
    Ifx_TimerValue period;

    period = driver->timer->base.period;

    /* Top channels */
    for (channelIndex = 0; channelIndex < driver->base.channelCount; channelIndex++)
    {
        Ifx_TimerValue x; /* x=period*dutyCycle, x=OnTime+deadTime */
        Ifx_TimerValue o;
        Ifx_TimerValue cm0, cm1;

        x = IfxStdIf_Timer_sToTick(driver->timer->base.clockFreq, tOn[channelIndex]);
        o = IfxStdIf_Timer_sToTick(driver->timer->base.clockFreq, offset[channelIndex]);

        if (driver->base.inverted != FALSE)
        {
            x = period - x;
        }
        else
        {}

        if ((x < driver->base.minPulse) || (o > period))
        {
            x = 0;
        }
        else if ((x > driver->base.maxPulse) || (o + x > period))
        {
            x = period;
        }
        else
        {}

        /* Special handling due to GTM issue */
        if (x == period)
        {   /* 100% duty cycle */
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex],
                period + 1 /* No compare event */,
                2 /* 1st compare event (issue: expected to be 1)*/);
        }
        else if (x == 0)
        {
            cm0 = 1;
            cm1 = period + 2;
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], cm0, cm1);
        }
        else
        {                /* x% duty cycle */
            cm1 = 2 + o; // CM1, set to 2 due to a GTM issue. should be 1 according to spec
            cm0 = o + x; // CM0, set to x+2 due to a GTM issue. should be x+1 according to spec
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], cm0, cm1);
        }
    }

    /* FIXME enable selection between top and bottom channels. See PWM HL without DTM */
}


IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updatePulseOff(IfxGtm_TomDtm_PwmHl *driver, float32 *tOn, float32 *offset)
{
    IFX_UNUSED_PARAMETER(tOn)
    IFX_UNUSED_PARAMETER(offset)

    IfxGtm_Tom_Dtm_PwmHl_updateOff(driver, NULL_PTR);
}


IFX_STATIC void IfxGtm_Tom_Dtm_PwmHl_updateShiftCenterAligned(IfxGtm_TomDtm_PwmHl *driver, Ifx_TimerValue *tOn, Ifx_TimerValue *shift)
{
    uint8          channelIndex;
    Ifx_TimerValue period;

    period = driver->timer->base.period;

    for (channelIndex = 0; channelIndex < driver->base.channelCount; channelIndex++)
    {
        Ifx_TimerValue x; /* x=period*dutyCycle, x=OnTime+deadTime */
        Ifx_TimerValue s; /* Shift value */
        Ifx_TimerValue cm0, cm1;
        x = tOn[channelIndex];

        if (driver->base.inverted != FALSE)
        {
            x = period - x;
        }
        else
        {}

        if (x < driver->base.minPulse)
        {   /* For deadtime condition: avoid leading edge of top channel to occur after the trailing edge */
            x = 0;
        }
        else if (x > driver->base.maxPulse)
        {
            x = period;
        }
        else
        {}

        /* Special handling due to GTM issue */
        if (x == period)
        {   /* 100% duty cycle */
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex],
                period + 1 /* No compare event */,
                2 /* 1st compare event (issue: expected to be 1)*/);
        }
        else if (x == 0)
        {
            cm0 = 1;
            cm1 = period + 2;
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], cm0, cm1);
        }
        else
        {                           /* x% duty cycle */
            s = shift[channelIndex];

            if (s > 0)
            {
                s = __minX(s, (period - x) / 2 - 1);
            }
            else
            {
                s = __maxX(s, (x - period) / 2 + 1);
            }

            cm1 = s + (period - x) / 2; // CM1
            cm0 = s + (period + x) / 2; // CM0
            IfxGtm_Tom_Ch_setCompareShadow(driver->tom, driver->ccx[channelIndex], cm0, cm1);
        }
    }
}


IFX_STATIC Ifx_TimerValue IfxGtm_Tom_Dtm_PwmHl_getDeadtimeInTimerClockTicks(IfxGtm_TomDtm_PwmHl *driver)
{
    Ifx_TimerValue deadtimeInTimerClkTicks;

    if (driver->dtmClockFreq == driver->timer->base.clockFreq)
    {
        // same clock frequency
        deadtimeInTimerClkTicks = driver->base.deadtime;
    }
    else if (driver->base.deadtime == 0)
    {
        deadtimeInTimerClkTicks = 0;
    }
    else
    {
        // convert deadtime from DTM clk frequency to TIMER clk frequency
        float32 deadtimeInS = IfxStdIf_Timer_tickToS(driver->dtmClockFreq, driver->base.deadtime);
        deadtimeInTimerClkTicks = IfxStdIf_Timer_sToTick(driver->timer->base.clockFreq, deadtimeInS);
        //TODO: round-up?
    }

    return deadtimeInTimerClkTicks;
}
