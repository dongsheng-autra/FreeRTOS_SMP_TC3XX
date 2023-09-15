/**
 * \file IfxIom_Driver.c
 * \brief IOM DRIVER details
 *
 * \version iLLD_1_0_1_16_1_1
 * \copyright Copyright (c) 2018 Infineon Technologies AG. All rights reserved.
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
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "IfxIom_Driver.h"
#include "IfxIom_bf.h"
#include "_Utilities/Ifx_Assert.h"

/******************************************************************************/
/*----------------------------------Macros------------------------------------*/
/******************************************************************************/

#define IFXIOM_DRIVER_LAM_GET_REF_INPUT_SIGNAL(refInput) ((IfxIom_RefInputSignal)((refInput >> 8) & 0xFF))

#define IFXIOM_DRIVER_LAM_GET_MON_INPUT_SIGNAL(monInput) ((IfxIom_MonInputSignal)((monInput >> 8) & 0xFF))

#define IFXIOM_DRIVER_LAM_GET_REF_INPUT_INDEX(refInput)  ((refInput >> 0) & 0xFF)

#define IFXIOM_DRIVER_LAM_GET_MON_INPUT_INDEX(monInput)  ((monInput >> 0) & 0xFF)

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

void IfxIom_Driver_clearAllGlitch(IfxIom_Driver *driver)
{
    Ifx_IOM *module = driver->module;
    module->FPCESR.U = 0xFFFFFFFF;
}


void IfxIom_Driver_clearHistory(IfxIom_Driver *driver)
{
    Ifx_IOM *module = driver->module;
    module->ECMETH0.U = 0;
}


void IfxIom_Driver_clearLamMonGlitch(IfxIom_Driver_Lam *driver)
{
    Ifx_IOM *module = driver->iomDriver->module;

    if (driver->monInput == IfxIom_MonInputSignal_p)
    {
        module->FPCESR.U = 0x10001 << driver->monIndex;
    }
}


void IfxIom_Driver_clearLamRefGlitch(IfxIom_Driver_Lam *driver)
{
    Ifx_IOM *module = driver->iomDriver->module;

    if (driver->refInput == IfxIom_RefInputSignal_p)
    {
        module->FPCESR.U = 0x10001 << driver->refIndex;
    }
}


uint32 IfxIom_Driver_disableEvents(IfxIom_Driver *driver)
{
    uint32 value;
    value                     = driver->module->ECMSELR.U;
    driver->module->ECMSELR.U = 0;
    return value;
}


void IfxIom_Driver_disableLamEvent(IfxIom_Driver_Lam *driver)
{
    IfxIom_Driver *iomDriver = driver->iomDriver;
    Ifx_IOM       *module    = iomDriver->module;

    /* Configure the ECM */
    if (driver->systemEventTriggerThreshold == 1)
    {
        module->ECMSELR.U &= ~(1 << (driver->channel + IFX_IOM_ECMSELR_CES0_OFF));
    }
    else if (driver->systemEventTriggerThreshold >= 2)
    {
        module->ECMSELR.U &= ~(1 << (driver->accumulatedCounterIndex + IFX_IOM_ECMSELR_CTS0_OFF));
    }
    else
    {
        /* No event generated */
    }
}


void IfxIom_Driver_enableLamEvent(IfxIom_Driver_Lam *driver)
{
    IfxIom_Driver *iomDriver = driver->iomDriver;
    Ifx_IOM       *module    = iomDriver->module;

    /* Configure the ECM */
    if (driver->systemEventTriggerThreshold == 1)
    {
        module->ECMSELR.U |= (1 << (driver->channel + IFX_IOM_ECMSELR_CES0_OFF));
    }
    else if (driver->systemEventTriggerThreshold >= 2)
    {
        module->ECMSELR.U |= (1 << (driver->accumulatedCounterIndex + IFX_IOM_ECMSELR_CTS0_OFF));
    }
    else
    {
        /* No event generated */
    }
}


void IfxIom_Driver_getHistory(IfxIom_Driver *driver, uint16 *a, uint16 *b, uint16 *c, uint16 *d)
{
    Ifx_IOM *module = driver->module;
    uint32   value;

    value = module->ECMETH0.U;
    *a    = value & 0xFFFF;
    *b    = value >> 16;

    value = module->ECMETH1.U;
    *c    = value & 0xFFFF;
    *d    = value >> 16;
}


boolean IfxIom_Driver_init(IfxIom_Driver *driver, IfxIom_Driver_Config *config)
{
    driver->module                   = config->module;
    driver->accumulatedEventUsedMask = 0;
    driver->lamUsedMask              = 0;
    return TRUE;
}


void IfxIom_Driver_initConfig(IfxIom_Driver_Config *config, Ifx_IOM *module)
{
    config->module = module;
}


boolean IfxIom_Driver_initLam(IfxIom_Driver_Lam *driver, IfxIom_Driver_LamConfig *config)
{
    boolean        result    = TRUE;
    IfxIom_Driver *iomDriver = config->iomDriver;
    Ifx_IOM       *module    = iomDriver->module;
    float32        fiom;
    fiom = IfxIom_getFrequency(module);

    /* Check parameter ranges */
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, (config->systemEventTriggerThreshold & (~IFX_IOM_ECMCCFG_THRC0_MSK)) == 0);

    driver->accumulatedCounterIndex = -1;
    driver->channel                 = config->channel;
    driver->iomDriver               = iomDriver;

    if (iomDriver->lamUsedMask & (1 << driver->channel))
    {
        /* LAM already in use */
        result = FALSE;
    }
    else
    {
        iomDriver->lamUsedMask |= 1 << driver->channel;

        {
            /* Configure reference input signal */
            uint8                 refIndex;
            IfxIom_RefInputSignal refInput;

            refIndex                       = IFXIOM_DRIVER_LAM_GET_REF_INPUT_INDEX(config->ref.input);
            refInput                       = IFXIOM_DRIVER_LAM_GET_REF_INPUT_SIGNAL(config->ref.input);
            driver->refIndex               = refIndex;
            driver->refInput               = refInput;

            module->FPCCTR[refIndex].B.ISR = refInput;

            if (refInput == IfxIom_RefInputSignal_p)
            {
                if (config->ref.filter.mode == IfxIom_LamFilterMode_noFilter)
                {
                    module->FPCCTR[refIndex].B.MOD = IfxIom_LamFilterMode_immediateDebounceBothEdge;
                    module->FPCCTR[refIndex].B.CMP = 0;
                }
                else if ((config->ref.filter.mode == IfxIom_LamFilterMode_prescalerOnFallingEdge)
                         ||
                         (config->ref.filter.mode == IfxIom_LamFilterMode_prescalerOnRisingEdge))
                {
                    module->FPCCTR[refIndex].B.MOD = config->ref.filter.mode;
                    module->FPCCTR[refIndex].B.CMP = config->ref.filter.prescalerFactor - 1;
                }
                else
                {
                    module->FPCCTR[refIndex].B.MOD = config->ref.filter.mode;
                    module->FPCCTR[refIndex].B.CMP = fiom * config->ref.filter.risingEdgeFilterTime;
                    module->FPCCTR[refIndex].B.RTG = config->ref.filter.clearTimerOnGlitch ? 1 : 0;
                }
            }
        }

        {
            /* Configure monitor input signal */
            uint8                 monIndex;
            IfxIom_MonInputSignal monInput;

            monIndex                       = IFXIOM_DRIVER_LAM_GET_MON_INPUT_INDEX(config->mon.input);
            monInput                       = IFXIOM_DRIVER_LAM_GET_MON_INPUT_SIGNAL(config->mon.input);
            driver->monIndex               = monIndex;
            driver->monInput               = monInput;

            module->FPCCTR[monIndex].B.ISM = monInput;

            if (monInput == IfxIom_MonInputSignal_p)
            {
                if (config->mon.filter.mode == IfxIom_LamFilterMode_noFilter)
                {
                    module->FPCCTR[monIndex].B.MOD = IfxIom_LamFilterMode_immediateDebounceBothEdge;
                    module->FPCCTR[monIndex].B.CMP = 0;
                }
                else
                {
                    module->FPCCTR[monIndex].B.MOD = config->mon.filter.mode;
                    module->FPCCTR[monIndex].B.CMP = fiom * config->mon.filter.risingEdgeFilterTime;
                    module->FPCCTR[monIndex].B.RTG = config->mon.filter.clearTimerOnGlitch ? 1 : 0;
                }
            }
        }

        {
            /* Configure the LAM */
            module->LAMCFG[driver->channel].B.IVR = config->ref.inverted ? 1 : 0;
            module->LAMCFG[driver->channel].B.IVM = config->mon.inverted ? 1 : 0;
            module->LAMCFG[driver->channel].B.MOS = config->event.source == IfxIom_LamEventSource_mon ? 0 : 1;
            module->LAMCFG[driver->channel].B.RMS = config->eventWindow.run;
            module->LAMCFG[driver->channel].B.EWS = config->eventWindow.controlSource;
            module->LAMCFG[driver->channel].B.EDS =
                ((config->eventWindow.clearEvent) << 0)
                | ((config->event.trigger) << 2)
            ;
            module->LAMCFG[driver->channel].B.IVW = config->eventWindow.inverted ? 1 : 0;
            module->LAMCFG[driver->channel].B.MCS = IFXIOM_DRIVER_LAM_GET_MON_INPUT_INDEX(config->mon.input);
            module->LAMCFG[driver->channel].B.RCS = IFXIOM_DRIVER_LAM_GET_REF_INPUT_INDEX(config->ref.input);

            module->LAMEWS[driver->channel].B.THR = fiom * config->eventWindow.threshold;
        }

        {
            /* Configure the ECM */
            driver->systemEventTriggerThreshold = config->systemEventTriggerThreshold;

            if (driver->systemEventTriggerThreshold == 1)
            {
                module->ECMSELR.U |= 1 << (config->channel + IFX_IOM_ECMSELR_CES0_OFF);
            }
            else if (driver->systemEventTriggerThreshold >= 2)
            {
                /* Look for a free counter */
                sint8   index;
                uint8   accumulatedEventUsedMask = iomDriver->accumulatedEventUsedMask;
                boolean success                  = FALSE;

                for (index = 0; index < 4; index++)
                {
                    if ((accumulatedEventUsedMask & (1 << index)) == 0)
                    {
                        success                              = TRUE;
                        iomDriver->accumulatedEventUsedMask |= 1 << index;
                        driver->accumulatedCounterIndex      = index;
                        module->ECMSELR.U                   |= 1 << (index + IFX_IOM_ECMSELR_CTS0_OFF);

                        module->ECMCCFG.U                   |= (
                            (driver->channel << IFX_IOM_ECMCCFG_SELC0_OFF)
                            | (driver->systemEventTriggerThreshold << IFX_IOM_ECMCCFG_THRC0_OFF)
                            ) << (index * IFX_IOM_ECMCCFG_SELC1_OFF);
                        break;
                    }
                }

                result &= success;
            }
            else
            {
                /* No event generated */
            }
        }
    }

    return result;
}


void IfxIom_Driver_initLamConfig(IfxIom_Driver_LamConfig *config, IfxIom_Driver *driver)
{
    config->iomDriver                        = driver;
    config->channel                          = IfxIom_LamId_0;
    config->event.source                     = IfxIom_LamEventSource_mon;
    config->event.trigger                    = IfxIom_LamEventTrigger_none;
    config->eventWindow.clearEvent           = IfxIom_LamEventWindowClearEvent_anyEdge;
    config->eventWindow.controlSource        = IfxIom_LamEventWindowControlSource_ref;
    config->eventWindow.inverted             = FALSE;
    config->eventWindow.run                  = IfxIom_LamEventWindowRunControl_freeRunning;
    config->eventWindow.threshold            = 0.0;
    config->mon.filter.clearTimerOnGlitch    = FALSE;
    config->mon.filter.fallingEdgeFilterTime = 0.0;
    config->mon.filter.mode                  = IfxIom_LamFilterMode_noFilter;
    config->mon.filter.prescalerFactor       = 1;
    config->mon.filter.risingEdgeFilterTime  = 0.0;
    config->mon.input                        = IfxIom_MonInput_p33_0;
    config->mon.inverted                     = FALSE;
    config->ref.filter.clearTimerOnGlitch    = FALSE;
    config->ref.filter.fallingEdgeFilterTime = 0.0;
    config->ref.filter.mode                  = IfxIom_LamFilterMode_noFilter;
    config->ref.filter.prescalerFactor       = 1;
    config->ref.filter.risingEdgeFilterTime  = 0.0;
    config->ref.input                        = IfxIom_RefInput_p33_0;
    config->ref.inverted                     = FALSE;
    config->systemEventTriggerThreshold      = 1;
}


void IfxIom_Driver_isLamMonGlitch(IfxIom_Driver_Lam *driver, boolean *risingEdgeGlitch, boolean *fallingEdgeGlitch)
{
    Ifx_IOM *module = driver->iomDriver->module;

    if (driver->monInput == IfxIom_MonInputSignal_p)
    {
        uint32 index;
        index              = driver->monIndex;
        *risingEdgeGlitch  = (module->FPCESR.U >> (index + IFX_IOM_FPCESR_REG0_OFF)) != 0;
        *fallingEdgeGlitch = (module->FPCESR.U >> (index + IFX_IOM_FPCESR_FEG0_OFF)) != 0;
    }
    else
    {
        *risingEdgeGlitch  = FALSE;
        *fallingEdgeGlitch = FALSE;
    }
}


void IfxIom_Driver_isLamRefGlitch(IfxIom_Driver_Lam *driver, boolean *risingEdgeGlitch, boolean *fallingEdgeGlitch)
{
    Ifx_IOM *module = driver->iomDriver->module;

    if (driver->refInput == IfxIom_RefInputSignal_p)
    {
        uint32 index;
        index              = driver->refIndex;
        *risingEdgeGlitch  = (module->FPCESR.U >> (index + IFX_IOM_FPCESR_REG0_OFF)) != 0;
        *fallingEdgeGlitch = (module->FPCESR.U >> (index + IFX_IOM_FPCESR_FEG0_OFF)) != 0;
    }
    else
    {
        *risingEdgeGlitch  = FALSE;
        *fallingEdgeGlitch = FALSE;
    }
}


void IfxIom_Driver_restoreEvents(IfxIom_Driver *driver, uint32 mask)
{
    driver->module->ECMSELR.U = mask;
}