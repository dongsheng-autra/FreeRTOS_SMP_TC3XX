/**
 * \file IfxHspdm.h
 * \brief HSPDM  basic functionality
 * \ingroup IfxLld_Hspdm
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
 *
 * \defgroup IfxLld_Hspdm_Std_Enumerations Enumerations
 * \ingroup IfxLld_Hspdm_Std
 * \defgroup IfxLld_Hspdm_Std_Configuration Configuration Functions
 * \ingroup IfxLld_Hspdm_Std
 * \defgroup IfxLld_Hspdm_Std_Operative Operative Functions
 * \ingroup IfxLld_Hspdm_Std
 * \defgroup IfxLld_Hspdm_Std_Utility Utility Functions
 * \ingroup IfxLld_Hspdm_Std
 */

#ifndef IFXHSPDM_H
#define IFXHSPDM_H 1

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "_Impl/IfxHspdm_cfg.h"
#include "IfxHspdm_reg.h"
#include "_Utilities/Ifx_Assert.h"
#include "_PinMap/IfxHspdm_PinMap.h"
#include "Scu/Std/IfxScuWdt.h"
#include "Src/Std/IfxSrc.h"

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/** \addtogroup IfxLld_Hspdm_Std_Enumerations
 * \{ */
/** \brief Bit streaming blocks.
 */
typedef enum
{
    IfxHspdm_BSB_0 = 0,  /**< \brief Bit Streaming Block 0 */
    IfxHspdm_BSB_1 = 1   /**< \brief Bit Streaming Block 1 */
} IfxHspdm_BSB;

/** \brief specifies the buffer.
 */
typedef enum
{
    IfxHspdm_Buffer_a           = 0, /**< \brief Buffer A */
    IfxHspdm_Buffer_b           = 1, /**< \brief Buffer B */
    IfxHspdm_Buffer_bufferCount = 2  /**< \brief number of buffers */
} IfxHspdm_Buffer;

/** \brief buffer modes.\n
 * Definition in Ifx_Hspdm.CON.MM
 */
typedef enum
{
    IfxHspdm_BufferMode_singleBuffer = 0,  /**< \brief single buffer mode */
    IfxHspdm_BufferMode_dualBuffer   = 1   /**< \brief dual buffer mode */
} IfxHspdm_BufferMode;

/** \brief Dither level values.\n
 * Definition in Ifx_Hspdm.CON.DITH
 */
typedef enum
{
    IfxHspdm_DitherLevel_disabled   = 0,  /**< \brief disable dither (default) */
    IfxHspdm_DitherLevel_minimum    = 1,  /**< \brief Minimum Dither level */
    IfxHspdm_DitherLevel_low        = 2,  /**< \brief Low Dither Level */
    IfxHspdm_DitherLevel_lowMedium  = 3,  /**< \brief Low-Medium Dither */
    IfxHspdm_DitherLevel_medium     = 4,  /**< \brief Medium Dither Level */
    IfxHspdm_DitherLevel_mediumHigh = 5,  /**< \brief Medium High Dither */
    IfxHspdm_DitherLevel_high       = 6,  /**< \brief High Dither Level */
    IfxHspdm_DitherLevel_highest    = 7   /**< \brief Highest Dither level */
} IfxHspdm_DitherLevel;

/** \brief enumeration holding event flags of HSPDM.\n
 * Definition in IfxHspdm.FLAGSSET, IfxHspdm.FLAGSCLEAR, IfxHspdm.FLAGSEN
 */
typedef enum
{
    IfxHspdm_Flag_buffAStart  = 0, /**< \brief buffer A start flag */
    IfxHspdm_Flag_buffAEnd    = 1, /**< \brief Buffer A End flag */
    IfxHspdm_Flag_buffBStart  = 2, /**< \brief Buffer B Start flag */
    IfxHspdm_Flag_buffBEnd    = 3, /**< \brief Buffer B flag */
    IfxHspdm_Flag_mute0Start  = 4, /**< \brief Mute 0 start flag */
    IfxHspdm_Flag_mute0End    = 5, /**< \brief Mute 0 End flag */
    IfxHspdm_Flag_mute1Start  = 6, /**< \brief Mute 1 start flag */
    IfxHspdm_Flag_mute1End    = 7, /**< \brief Mute 1 end flag */
    IfxHspdm_Flag_ramOverflow = 8  /**< \brief Ram overflow error flag */
} IfxHspdm_Flag;

/** \brief active edge which triggers the HSPDM.\n
 * Definition in IfxHspdm.CON.HRAE.
 */
typedef enum
{
    IfxHspdm_HwRunActiveEdge_rising  = 0, /**< \brief Rising edge is active edge */
    IfxHspdm_HwRunActiveEdge_falling = 1  /**< \brief Falling edge is active edge */
} IfxHspdm_HwRunActiveEdge;

/** \brief mute signal polarity.\n
 * Definition in IfxHspdm.CON.MPOL
 */
typedef enum
{
    IfxHspdm_MutePolarity_activeHigh = 0,  /**< \brief active high */
    IfxHspdm_MutePolarity_activeLow  = 1   /**< \brief active low */
} IfxHspdm_MutePolarity;

/** \brief HSPDM running status.\n
 * Definition in IfxHspdm.CON.RUN
 */
typedef enum
{
    IfxHspdm_RunState_stopped = 0,  /**< \brief hspdm is stopped */
    IfxHspdm_RunState_running = 1   /**< \brief Hspdm is running */
} IfxHspdm_RunState;

/** \brief Sleep Modes.\n
 * Definition in IfxHspdm.CLC.B.EDIS
 */
typedef enum
{
    IfxHspdm_SleepMode_enabled  = 0, /**< \brief sleep mode enabled. */
    IfxHspdm_SleepMode_disabled = 1  /**< \brief sleep mode disabled. */
} IfxHspdm_SleepMode;

/** \brief Streaming modes available at HSPDM.\n
 * Definition in IfxHspdm.CON.SM
 */
typedef enum
{
    IfxHspdm_StreamingMode_deltaSigmaCICEnabled  = 0,  /**< \brief Delta Sigma with CIC and compactor enabled (default) */
    IfxHspdm_StreamingMode_deltaSigmaCICDisabled = 1,  /**< \brief Delta-Sigma with CIC and compactor disabled */
    IfxHspdm_StreamingMode_shiftRegister         = 2   /**< \brief Shift Register generated */
} IfxHspdm_StreamingMode;

/** \brief OCDS suspend Modes.\n
 * used to set IfxHspdm.OCS.B.SUS
 */
typedef enum
{
    IfxHspdm_SuspendMode_none = 0,  /**< \brief wil not suspend */
    IfxHspdm_SuspendMode_hard = 1,  /**< \brief hard suspend. Clock Switched Off. */
    IfxHspdm_SuspendMode_soft = 2   /**< \brief Soft Suspend. */
} IfxHspdm_SuspendMode;

/** \brief update frequency.\n
 * Definition in IfxHSPDM.CON.ITMDIV
 */
typedef enum
{
    IfxHspdm_UpdateFreq_1MHz   = 0,  /**< \brief divide by 1 */
    IfxHspdm_UpdateFreq_0p5MHz = 1,  /**< \brief divide by 0.5 */
    IfxHspdm_UpdateFreq_2MHz   = 2,  /**< \brief divide by 2 */
    IfxHspdm_UpdateFreq_10MHz  = 3   /**< \brief divide by 10 */
} IfxHspdm_UpdateFreq;

/** \} */

/** \brief Enumeration listing the mute ranges available for Mute Signal set and clear.
 * Definition as MUTE0 and MUTE1 registers in HSPDM.
 */
typedef enum
{
    IfxHspdm_MuteRange_0 = 0,  /**< \brief Mute Range 0 */
    IfxHspdm_MuteRange_1 = 1   /**< \brief Mute Range 1 */
} IfxHspdm_MuteRange;

/** \addtogroup IfxLld_Hspdm_Std_Configuration
 * \{ */

/******************************************************************************/
/*-------------------------Inline Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief disable the ADC Trigger.
 * \param hspdm hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_disableAdcTrigger(Ifx_HSPDM *hspdm);

/** \brief sets the ADC Trigger offset value (specified in counts of Fshift)
 * \param hspdm hspdm device
 * \param offsetCount offset counts
 * \return None
 */
IFX_INLINE void IfxHspdm_setAdcTriggerOffset(Ifx_HSPDM *hspdm, uint32 offsetCount);

/** \brief sets the trigger period in counts of Fshift.
 * If Period is set to <9 counts, Trigger will always be High.\n
 * This will assert an error.
 * \param hspdm hspdm device
 * \param periodCount period in Fshift counts
 * \return None
 */
IFX_INLINE void IfxHspdm_setAdcTriggerPeriod(Ifx_HSPDM *hspdm, uint32 periodCount);

/** \brief This will set the desired count as the ADC Trigger count value.\n
 * (TriggerCounts - 1) will be written into the TGCNT field.\n
 * \param hspdm hspdm device
 * \param triggerCounts trigger counts
 * \return None
 */
IFX_INLINE void IfxHspdm_setAdcTriggerCounts(Ifx_HSPDM *hspdm, uint32 triggerCounts);

/** \brief enable the Pad Assymetry Compensation.
 * \param hspdm hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_enablePac(Ifx_HSPDM *hspdm);

/** \brief sets the buffer mode
 * \param hspdm hspdm device.
 * \param bufferMode buffer modes.\n
 * Definition in Ifx_Hspdm.CON.MM
 * \return None
 */
IFX_INLINE void IfxHspdm_setBufferMode(Ifx_HSPDM *hspdm, IfxHspdm_BufferMode bufferMode);

/** \brief set the polarity of the mute signal as desired.
 * \param hspdm hspdm device
 * \param polarity mute signal polarity
 * \return None
 */
IFX_INLINE void IfxHspdm_setMutePolarity(Ifx_HSPDM *hspdm, IfxHspdm_MutePolarity polarity);

/** \brief enables the Adc Trigger.
 * \param hspdm hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_enableAdcTrigger(Ifx_HSPDM *hspdm);

/** \brief set the active edge of the trigger signal which will activate HSPDM.
 * \param hspdm hspdm device
 * \param activeEdge active edge
 * \return None
 */
IFX_INLINE void IfxHspdm_setHwRunActiveEdge(Ifx_HSPDM *hspdm, IfxHspdm_HwRunActiveEdge activeEdge);

/** \brief sets the Hardware Run Trigger Source to the selected source.
 * \param hspdm hspdm device
 * \param triggerSource hardware trigger source
 * \return None
 */
IFX_INLINE void IfxHspdm_setHwRunTriggerSource(Ifx_HSPDM *hspdm, IfxHspdm_HwTriggerSource triggerSource);

/** \brief Enable the Hardware Run Signal
 * \param hspdm hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_enableHwRun(Ifx_HSPDM *hspdm);

/** \brief disables the Hardware Run feature.
 * \param hspdm hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_disableHwRun(Ifx_HSPDM *hspdm);

/** \brief sets the OCDS suspend mode
 * \param hspdm pointer to the hspdm device
 * \param mode suspend mode.
 * \return None
 */
IFX_INLINE void IfxHspdm_setSuspendMode(Ifx_HSPDM *hspdm, IfxHspdm_SuspendMode mode);

/** \brief sets the sleep mode.
 * \param hspdm pointer to the hspdm device
 * \param mode suspend mode.
 * \return None
 */
IFX_INLINE void IfxHspdm_setSleepMode(Ifx_HSPDM *hspdm, IfxHspdm_SleepMode mode);

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief enable / disable the bit streaming blocks
 * \param hspdm hspdm device
 * \param bsb bsb number
 * \param enable enable status
 * \return None
 */
IFX_EXTERN void IfxHspdm_enableBSB(Ifx_HSPDM *hspdm, IfxHspdm_BSB bsb, boolean enable);

/** \brief initializes the BSB pin
 * \param bsPin BS pin
 * \param outputMode output mode
 * \param padDriver pad driver
 * \return None
 */
IFX_EXTERN void IfxHspdm_initBsPin(IfxHspdm_Bs_Out *bsPin, IfxPort_OutputMode outputMode, IfxPort_PadDriver padDriver);

/** \brief Initializes the Mute pin
 * \param mutePin mute pin
 * \param outputMode output mode
 * \param padDriver pad driver
 * \return None
 */
IFX_EXTERN void IfxHspdm_initMutePin(IfxHspdm_Mute_Out *mutePin, IfxPort_OutputMode outputMode, IfxPort_PadDriver padDriver);

/** \} */

/** \addtogroup IfxLld_Hspdm_Std_Operative
 * \{ */

/******************************************************************************/
/*-------------------------Inline Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Function used to set the streaming mode.
 * \param hspdm hspdm module
 * \param streamingMode Streaming modes available at HSPDM.\n
 * Definition in IfxHspdm.CON.SM
 * \return None
 */
IFX_INLINE void IfxHspdm_setStreamingMode(Ifx_HSPDM *hspdm, IfxHspdm_StreamingMode streamingMode);

/** \brief set the desired dither level in Bit stream output.
 * \param hspdm hspdm device
 * \param ditherLevel Dither level values.\n
 * Definition in Ifx_Hspdm.CON.DITH
 * \return None
 */
IFX_INLINE void IfxHspdm_setDitherLevel(Ifx_HSPDM *hspdm, IfxHspdm_DitherLevel ditherLevel);

/** \brief sets the divider value required in the ITMDIV field to realize the required update frequency.
 * \param hspdm hspdm device
 * \param updateFrequency update freq
 * \return None
 */
IFX_INLINE void IfxHspdm_setUpdateFreq(Ifx_HSPDM *hspdm, IfxHspdm_UpdateFreq updateFrequency);

/** \brief start the bit stream from HSPDM.
 * \param hspdm hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_startBitStream(Ifx_HSPDM *hspdm);

/** \brief stop the bit stream from HSPDM.
 * \param hspdm hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_stopBitStream(Ifx_HSPDM *hspdm);

/** \brief sets the specified flag.
 * \param hspdm hspdm device
 * \param flag event flag
 * \return None
 */
IFX_INLINE void IfxHspdm_setFlag(Ifx_HSPDM *hspdm, IfxHspdm_Flag flag);

/** \brief clears the specified flag.
 * \param hspdm hspdm device
 * \param flag event flag
 * \return None
 */
IFX_INLINE void IfxHspdm_clearFlag(Ifx_HSPDM *hspdm, IfxHspdm_Flag flag);

/** \brief enable the interrupt on event
 * \param hspdm hspdm device
 * \param flag event flag
 * \return None
 */
IFX_INLINE void IfxHspdm_enableFlag(Ifx_HSPDM *hspdm, IfxHspdm_Flag flag);

/** \brief disable the Pad Assymetry Compensation.
 * \param hspdm hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_disablePac(Ifx_HSPDM *hspdm);

/** \brief enable the HSPDM
 * \param hspdm pointer to the hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_enableModule(Ifx_HSPDM *hspdm);

/** \brief disables the hspdm module.
 * \param hspdm pointer to the hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_disableModule(Ifx_HSPDM *hspdm);

/** \brief resets the hspdm module.
 * \param hspdm pointer to the hspdm device
 * \return None
 */
IFX_INLINE void IfxHspdm_resetModule(Ifx_HSPDM *hspdm);

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief sets the start address of the chosen buffer.
 * \param hspdm hspdm device
 * \param buffer buffer number
 * \param address start address
 * \return None
 */
IFX_EXTERN void IfxHspdm_setStartAddress(Ifx_HSPDM *hspdm, IfxHspdm_Buffer buffer, uint32 address);

/** \brief sets the end address of the chosen buffer.
 * \param hspdm hspdm device
 * \param buffer buffer number
 * \param address end address
 * \return None
 */
IFX_EXTERN void IfxHspdm_setEndAddress(Ifx_HSPDM *hspdm, IfxHspdm_Buffer buffer, uint32 address);

/** \brief set the start and end address of the mute signals
 * \param hspdm hspdm device
 * \param muteRange Mute Range selection
 * \param startAddress Start Address
 * \param endAddress end address
 * \return None
 */
IFX_EXTERN void IfxHspdm_setMuteAddresses(Ifx_HSPDM *hspdm, IfxHspdm_MuteRange muteRange, uint32 startAddress, uint32 endAddress);

/** \} */

/** \addtogroup IfxLld_Hspdm_Std_Utility
 * \{ */

/******************************************************************************/
/*-------------------------Inline Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief function returns the current address being access in HSPDM SRAM
 * \param hspdm hspdm device
 * \return current address
 */
IFX_INLINE uint32 IfxHspdm_getCurrentSramAddr(Ifx_HSPDM *hspdm);

/** \brief returns the run status of the hspdm module.
 * \param hspdm hspdm device
 * \return HSPDM running status.\n
 * Definition in IfxHspdm.CON.RUN
 */
IFX_INLINE IfxHspdm_RunState IfxHspdm_getRunStatus(Ifx_HSPDM *hspdm);

/** \brief returns the physical level of the mute signal
 * \param hspdm hspdm device
 * \return mute level
 */
IFX_INLINE boolean IfxHspdm_getMuteLevel(Ifx_HSPDM *hspdm);

/** \brief returns the status of the flag specified.
 * \param hspdm hspdm device
 * \param flag enumeration holding event flags of HSPDM
 * \return flag status
 */
IFX_INLINE boolean IfxHspdm_getFlagStatus(Ifx_HSPDM *hspdm, IfxHspdm_Flag flag);

/** \brief Returns the SRC pointer for HSPDM buffer Interrupt
 * \return pointer to BFR interrupt node of HSPDM
 */
IFX_INLINE volatile Ifx_SRC_SRCR *IfxHspdm_getSrcPointerBFR(void);

/** \brief Returns the SRC pointer for HSPDM MUTE Interrupt.\n
 * This is mentioned as RAMP in the SRC regdef file.
 * \return pointer to MUTE interrupt node of HSPDM
 */
IFX_INLINE volatile Ifx_SRC_SRCR *IfxHspdm_getSrcPointerMUTE(void);

/** \brief Returns the SRC pointer for HSPDM error Interrupt.
 * \return pointer to ERR interrupt node of HSPDM
 */
IFX_INLINE volatile Ifx_SRC_SRCR *IfxHspdm_getSrcPointerERR(void);

/** \} */

/******************************************************************************/
/*---------------------Inline Function Implementations------------------------*/
/******************************************************************************/

IFX_INLINE void IfxHspdm_setStreamingMode(Ifx_HSPDM *hspdm, IfxHspdm_StreamingMode streamingMode)
{
    hspdm->CON.B.SM = (uint32)streamingMode;
}


IFX_INLINE void IfxHspdm_setDitherLevel(Ifx_HSPDM *hspdm, IfxHspdm_DitherLevel ditherLevel)
{
    hspdm->CON.B.DITH = (uint32)ditherLevel;
}


IFX_INLINE void IfxHspdm_setUpdateFreq(Ifx_HSPDM *hspdm, IfxHspdm_UpdateFreq updateFrequency)
{
    hspdm->CON.B.ITMDIV = (uint32)updateFrequency;
}


IFX_INLINE void IfxHspdm_disableAdcTrigger(Ifx_HSPDM *hspdm)
{
    hspdm->CON.B.ADCTGEN = (uint32)1;
}


IFX_INLINE void IfxHspdm_setAdcTriggerOffset(Ifx_HSPDM *hspdm, uint32 offsetCount)
{
    hspdm->ADCTG.B.OFFSET = offsetCount;
}


IFX_INLINE void IfxHspdm_setAdcTriggerPeriod(Ifx_HSPDM *hspdm, uint32 periodCount)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, periodCount >= 9);  /* for period < 9; we have to assert an error */
    hspdm->ADCTG.B.PERIOD = periodCount;
}


IFX_INLINE void IfxHspdm_setAdcTriggerCounts(Ifx_HSPDM *hspdm, uint32 triggerCounts)
{
    if (triggerCounts >= (uint32)1)
    {
        hspdm->ADCTGCNT.B.TGCNT = (triggerCounts - 1);
    }
    else
    {
        hspdm->ADCTGCNT.B.TGCNT = 0;
    }
}


IFX_INLINE void IfxHspdm_enablePac(Ifx_HSPDM *hspdm)
{
    hspdm->CON.B.PAC = (uint32)1;
}


IFX_INLINE uint32 IfxHspdm_getCurrentSramAddr(Ifx_HSPDM *hspdm)
{
    return (uint32)(hspdm->CURRAD.B.CURRAD);
}


IFX_INLINE void IfxHspdm_setBufferMode(Ifx_HSPDM *hspdm, IfxHspdm_BufferMode bufferMode)
{
    hspdm->CON.B.MM = (uint32)bufferMode;
}


IFX_INLINE void IfxHspdm_startBitStream(Ifx_HSPDM *hspdm)
{
    hspdm->CON.B.RUNS = (uint32)1;
}


IFX_INLINE void IfxHspdm_stopBitStream(Ifx_HSPDM *hspdm)
{
    hspdm->CON.B.RUNC = (uint32)1;
}


IFX_INLINE IfxHspdm_RunState IfxHspdm_getRunStatus(Ifx_HSPDM *hspdm)
{
    return (IfxHspdm_RunState)(hspdm->CON.B.RUN);
}


IFX_INLINE void IfxHspdm_setMutePolarity(Ifx_HSPDM *hspdm, IfxHspdm_MutePolarity polarity)
{
    hspdm->CON.B.MPOL = (uint32)polarity;
}


IFX_INLINE boolean IfxHspdm_getMuteLevel(Ifx_HSPDM *hspdm)
{
    return (boolean)(hspdm->FLAGS.B.MUTE);
}


IFX_INLINE boolean IfxHspdm_getFlagStatus(Ifx_HSPDM *hspdm, IfxHspdm_Flag flag)
{
    return (boolean)((hspdm->FLAGS.U >> (uint32)flag) & (uint32)1);
}


IFX_INLINE void IfxHspdm_setFlag(Ifx_HSPDM *hspdm, IfxHspdm_Flag flag)
{
    hspdm->FLAGSSET.U |= ((uint32)1 << (uint32)flag);
}


IFX_INLINE void IfxHspdm_clearFlag(Ifx_HSPDM *hspdm, IfxHspdm_Flag flag)
{
    hspdm->FLAGSCLEAR.U |= ((uint32)1 << (uint32)flag);
}


IFX_INLINE void IfxHspdm_enableFlag(Ifx_HSPDM *hspdm, IfxHspdm_Flag flag)
{
    hspdm->FLAGSEN.U |= ((uint32)1 << (uint32)flag);
}


IFX_INLINE void IfxHspdm_disablePac(Ifx_HSPDM *hspdm)
{
    hspdm->CON.B.PAC = (uint32)0;
}


IFX_INLINE void IfxHspdm_enableAdcTrigger(Ifx_HSPDM *hspdm)
{
    hspdm->CON.B.ADCTGEN = (uint32)0;
}


IFX_INLINE void IfxHspdm_setHwRunActiveEdge(Ifx_HSPDM *hspdm, IfxHspdm_HwRunActiveEdge activeEdge)
{
    hspdm->CON.B.HRAE = (uint32)activeEdge;
}


IFX_INLINE void IfxHspdm_setHwRunTriggerSource(Ifx_HSPDM *hspdm, IfxHspdm_HwTriggerSource triggerSource)
{
    hspdm->CON.B.HRSEL = (uint32)triggerSource;
}


IFX_INLINE void IfxHspdm_enableHwRun(Ifx_HSPDM *hspdm)
{
    hspdm->CON.B.HREN = (uint32)1;
}


IFX_INLINE void IfxHspdm_disableHwRun(Ifx_HSPDM *hspdm)
{
    hspdm->CON.B.HREN = (uint32)0;
}


IFX_INLINE void IfxHspdm_enableModule(Ifx_HSPDM *hspdm)
{
    uint16 psw = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(psw); /* clears the endinit protection*/
    hspdm->CLC.B.DISR = (uint32)0;  /* enables the module*/
    IfxScuWdt_setCpuEndinit(psw);   /* sets the endinit protection back on*/
}


IFX_INLINE void IfxHspdm_disableModule(Ifx_HSPDM *hspdm)
{
    uint16 psw = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(psw); /* clears the endinit protection*/
    hspdm->CLC.B.DISR = (uint32)1;  /* disables the module*/
    IfxScuWdt_setCpuEndinit(psw);
}


IFX_INLINE void IfxHspdm_resetModule(Ifx_HSPDM *hspdm)
{
    uint16 passwd = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(passwd);

    hspdm->KRST0.B.RST = 1;         /* Only if both Kernel reset bits are set a reset is executed */
    hspdm->KRST1.B.RST = 1;
    IfxScuWdt_setCpuEndinit(passwd);

    while (0 == hspdm->KRST0.B.RSTSTAT)     /* Wait until reset is executed */
    {}

    IfxScuWdt_clearCpuEndinit(passwd);
    hspdm->KRSTCLR.B.CLR = 1;           /* Clear Kernel reset status bit */

    IfxScuWdt_setCpuEndinit(passwd);
}


IFX_INLINE void IfxHspdm_setSuspendMode(Ifx_HSPDM *hspdm, IfxHspdm_SuspendMode mode)
{
    Ifx_HSPDM_OCS ocs;

    // remove protection and configure the suspend mode.
    ocs.B.SUS_P  = 1;
    ocs.B.SUS    = mode;
    hspdm->OCS.U = ocs.U;
}


IFX_INLINE void IfxHspdm_setSleepMode(Ifx_HSPDM *hspdm, IfxHspdm_SleepMode mode)
{
    uint16 passwd = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(passwd);
    hspdm->CLC.B.EDIS = mode;
    IfxScuWdt_setCpuEndinit(passwd);
}


IFX_INLINE volatile Ifx_SRC_SRCR *IfxHspdm_getSrcPointerBFR(void)
{
    return (Ifx_SRC_SRCR *)(&MODULE_SRC.HSPDM.HSPDM0.BFR);
}


IFX_INLINE volatile Ifx_SRC_SRCR *IfxHspdm_getSrcPointerMUTE(void)
{
    return (Ifx_SRC_SRCR *)(&MODULE_SRC.HSPDM.HSPDM0.RAMP);
}


IFX_INLINE volatile Ifx_SRC_SRCR *IfxHspdm_getSrcPointerERR(void)
{
    return (Ifx_SRC_SRCR *)(&MODULE_SRC.HSPDM.HSPDM0.ERR);
}


#endif /* IFXHSPDM_H */
