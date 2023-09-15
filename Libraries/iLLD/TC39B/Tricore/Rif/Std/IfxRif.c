/**
 * \file IfxRif.c
 * \brief RIF  basic functionality
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

#include "IfxRif.h"

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

void IfxRif_disableModule(Ifx_RIF *rif)
{
    uint16 password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);
    rif->CLC.B.DISR = 1U;
    IfxScuWdt_setCpuEndinit(password);
}


void IfxRif_enableFifos(Ifx_RIF *rif, uint8 count)
{
    IfxRif_disableAllFifos(rif);

    switch (count)
    {
    case 0:     /*All FIFOs disabled*/
        break;
    case 1:
        IfxRif_enableFifo(rif, IfxRif_FifoId_0);
        break;
    case 2:
        IfxRif_enableFifo(rif, IfxRif_FifoId_0);
        IfxRif_enableFifo(rif, IfxRif_FifoId_1);
        break;
    case 3:
    case 4:
        IfxRif_enableFifo(rif, IfxRif_FifoId_0);
        IfxRif_enableFifo(rif, IfxRif_FifoId_1);
        IfxRif_enableFifo(rif, IfxRif_FifoId_2);
        IfxRif_enableFifo(rif, IfxRif_FifoId_3);
        break;
    default:
        IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE); /* wrong selection  */
        break;
    }
}


void IfxRif_enableModule(Ifx_RIF *rif)
{
    uint16 password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);
    rif->CLC.B.DISR = 0U;
    IfxScuWdt_setCpuEndinit(password);

    /* Wait until module is enabled */
    while (IfxRif_isModuleEnabled(rif) == FALSE)
    {}
}


Ifx_RIF *IfxRif_getAddress(IfxRif_Index rif)
{
    Ifx_RIF *module;

    if (rif < IFXRIF_NUM_MODULES)
    {
        module = (Ifx_RIF *)IfxRif_cfg_indexMap[rif].module;
    }
    else
    {
        module = NULL_PTR;
    }

    return module;
}


IfxRif_Index IfxRif_getIndex(Ifx_RIF *rif)
{
    uint32       index;
    IfxRif_Index result;

    result = IfxRif_Index_none;

    for (index = 0; index < IFXRIF_NUM_MODULES; index++)
    {
        if (IfxRif_cfg_indexMap[index].module == rif)
        {
            result = (IfxRif_Index)IfxRif_cfg_indexMap[index].index;
            break;
        }
    }

    return result;
}


volatile Ifx_SRC_SRCR *IfxRif_getSrcPointerErr(Ifx_RIF *rif)
{
    return &MODULE_SRC.RIF.RIF[IfxRif_getIndex(rif)].ERR;
}


volatile Ifx_SRC_SRCR *IfxRif_getSrcPointerInt(Ifx_RIF *rif)
{
    return &MODULE_SRC.RIF.RIF[IfxRif_getIndex(rif)].INT;
}


void IfxRif_disableFifo(Ifx_RIF *rif, IfxRif_FifoId fifoId)
{
    uint32 setValue = ~(1U << fifoId);
    rif->DMI.U = (rif->DMI.U & setValue);
}


void IfxRif_disableAllFifos(Ifx_RIF *rif)
{
    rif->DMI.U = 0U;
}
