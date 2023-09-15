/**
 * \file IfxCpu_Irq.h
 * \brief This file contains the APIs for Interrupt related functions.
 *
 *
 * \version iLLD_1_0_1_16_1_1
 * \copyright Copyright (c) 2012 Infineon Technologies AG. All rights reserved.
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
 * \defgroup IfxLld_Cpu_Irq Interrupt Functions
 * \ingroup IfxLld_Cpu
 *
 * \defgroup IfxLld_Cpu_Irq_Usage How to define Interrupts?
 * \ingroup IfxLld_Cpu_Irq
 *
 */
#ifndef IFXCPU_IRQ_H_
#define IFXCPU_IRQ_H_

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include "Ifx_Cfg.h"
#include "Cpu/Std/Ifx_Types.h"
#include "Cpu/Std/IfxCpu.h"
#include "Src/Std/IfxSrc.h"
/*******************************************************************************
**                      Type definitions                                     **
*******************************************************************************/

/*******************************************************************************
**                Global Exported variables/constants                         **
*******************************************************************************/

/*******************************************************************************
**         Global Exported macros/inlines/function ptototypes                 **
*******************************************************************************/

#if defined(IFX_USE_SW_MANAGED_INT)
/** \addtogroup IfxLld_Cpu_Irq_Usage
 * \{  */
/** \brief API for Interrupt handler install for SW Managed interrupts.
 * This API installs the isr to SW interrupt vector.
 * This must be used only when IFX_USE_SW_MANAGED_INT is defined in Ifx_Cfg.h
 *
 * \param isrFuncPointer pointer to ISR function.
 * \param serviceReqPrioNumber ISR priority.
 */

IFX_EXTERN void IfxCpu_Irq_installInterruptHandler(void *isrFuncPointer, uint32 serviceReqPrioNumber);

IFX_INLINE void interruptHandlerInstall(uint32 srpn, uint32 addr)
{
    IfxCpu_Irq_installInterruptHandler((void *)addr, srpn);
}


/** \}  */
#endif /*defined(IFX_USE_SW_MANAGED_INT) */

/** \addtogroup IfxLld_Cpu_Irq_Usage
 * \{  */
/** \brief API to get type of service of the caller CPU.
 * \param coreId core id of the core
 * \return type of service for the corresponding CPU.
 */
IFX_EXTERN IfxSrc_Tos IfxCpu_Irq_getTos(IfxCpu_ResourceCpu coreId);

/** \}  */

/*Documentation */
/** \addtogroup IfxLld_Cpu_Irq_Usage
 * \{
 *
 * This page describes how to use interrupts with application framework.\n
 *
 * \section IfxLld_Cpu_Irq_Terminology Interrupts Terminology:
 * \subsection IfxLld_Cpu_Irq_HWManaged Hardware Managed Interrupt Mechanism.
 * Hardware managed interrupts have static interrupt vector which are defined for each priority separately.
 * These vectors have jump instruction to the interrupt handler.
 *
 * Advantages:\n
 * This mechanism has less interrupt latency time.
 *
 * \subsection IfxLld_Cpu_Irq_SWManaged Software Managed Interrupt Mechanism.
 * Software managed interrupts have single interrupt vector statically defined at vector position 255.
 * This address is assigned to BIV during startup.\n
 * For Tricore, this vector position is important, because whenever an interrupt occurs, with whichever priority,
 * the execution control jumps to this vector position. The code at this vector position will:\n
 * 1) fetch the priority of the targetted interrupt.\n
 * 2) fetch the interrupt handler defined for this priority (this is done by Interrupt handler installation. Refer
 * \ref IfxLld_Cpu_Irq_Step4\n
 * 3) Then call the handler as notmal function call.
 *
 * Advantages:\n
 * This kind of mechanism is useful when project wants to change the handler for an interrupt during runtime.
 *
 * Disadvantages:\n
 * This mechanism has more interrupt latency time.
 *
 * of the interrupt and in tand jumps to the function
 *
 * \section IfxLld_Cpu_Irq_Steps Steps to use Interrupt Mechanism.
 * Dependency: Ifx_Compilers, Ifx_Cpu, Ifx_Src, IfxCpu_Irq\n
 * Following are the steps to use interrupt mechanism.
 *
 * \section IfxLld_Cpu_Irq_Step1 Step1: Define Interrupt priorities.
 * Define priorities of all interrupts with names corresponding to their functionality. It is recommended to define
 * such priority definitions in single header file, because it is easy to detect if ISR priorities are conflicting.
 * In Tricore architecture, two Isrs can't have same priority at same point of time.
 * \note These defines shall be defined without brackets surrounding priority number. (eg. #define PRIO (10) is not allowed)
 *
 * In a user defined file eg. Ifx_IntPrioDef.h, placed in folder: 0_AppSw/Tricore/DemoApp:
 * \code
 * //file: Ifx_IntPrioDef.h.
 * #define IFX_INTPRIO_FUNCT1	1
 * #define IFX_INTPRIO_FUNCT2	2
 * #define IFX_INTPRIO_FUNCT3	5
 * #define IFX_INTPRIO_STM0	8
 * #define IFX_INTPRIO_ADC_FUNC1 10
 * //etc.
 * \endcode
 *
 * \note !! IMPORTANT !!\n As explained above, the definition with closing bracket around priority number as,
 *  #define IFX_INTPRIO_FUNCT1   (1) will cause compilation error. Because linker sections which are constructed
 *  using such information will also get these brackets included. Which look like ".intvec_tc0_(1)" instead of the
 *  expected ".intvec_tc0_1"\n
 *  Linker sections' definitions are predefined statically in .lsl file,
 * for all 255 interrupts, with the format ".intvec_tc<vector number>_<interrupt priority>".
 *
 * \section IfxLld_Cpu_Irq_Step2 Step2: Define Type of interrupt mechanism.
 * \subsection IfxLld_Cpu_Irq_HWManaged_Usage To use Hardware Managed Interrupt Mechanism.
 * Refer \ref IfxLld_Cpu_Irq_HWManaged
 * If project is designed for hardware managed interrupts, this feature is enabled at the file Ifx_Cfg.h, at path:
 * 0_Src/0_AppSw/Config/Common/, as shown below. IFX_USE_SW_MANAGED_INT definition must be undefined (i.e. the
 * statement "#define IFX_USE_SW_MANAGED_INT" shall be commented as below).
 *
 * \code
 * //file: Ifx_Cfg.h
 *
 * //#define IFX_USE_SW_MANAGED_INT
 *
 * \endcode
 *
 * \subsection IfxLld_Cpu_Irq_SWManaged_Usage To use Software Managed Interrupt Mechanism.
 * Refer \ref IfxLld_Cpu_Irq_SWManaged
 * If project is designed for software managed interrupts, this feature is enabled at the file Ifx_Cfg.h, at path:
 * 0_Src/0_AppSw/Config/Common/, as shown below.
 * IFX_USE_SW_MANAGED_INT definition must be defined.
 *
 * \code
 * //file: Ifx_Cfg.h
 *
 * #define IFX_USE_SW_MANAGED_INT
 *
 * \endcode
 *
 * Software managed interrupts must also install the "Interrupt Handlers" Refer \ref IfxLld_Cpu_Irq_Step4
 *
 * \section IfxLld_Cpu_Irq_Step3 Step3: How to define an Interrupt Service routine?
 * Interrupt service routines or interrupt handlers are defined in driver specific files or application specific
 * files.
 *
 * \code
 * //file usercode1.c
 * #include "Compilers.h"		// to get the compiler abstracted macros for interrupt definition
 * #include "Ifx_IntPrioDef.h"	// to get the priority numbers
 *
 * //define an ISR with name Isr_Stm0 with priority defined by IFX_INTPRIO_STM0
 * IFX_INTERRUPT (Isr_Stm0, 0, IFX_INTPRIO_STM0)
 * {
 *  //Isr code here
 * }
 * \endcode
 *
 * \code
 * //file usercode2.c
 * #include "Compilers.h"		// to get the compiler abstracted macros for interrupt definition
 * #include "Ifx_IntPrioDef.h"	// to get the priority numbers
 *
 * //define an ISR with name Isr_Adc_fun1 with priority defined by IFX_INTPRIO_ADC_FUNC1
 * IFX_INTERRUPT (Isr_Adc_fun1, 0, IFX_INTPRIO_ADC_FUNC1)
 * {
 *   //Isr code here
 * }
 * \endcode
 *
 * \section IfxLld_Cpu_Irq_Step4 Step4: How to install Interrupt Service routine/handler?
 * This step is not required for HW managed interrupts.\n
 * Interrupt service routines or interrupt handlers are installed in driver specific files or application specific
 * files
 *
 *  \code
 * //file usermain.c
 * #include "IfxCpu_Irq.h"
 * #include "Ifx_IntPrioDef.h"	// to get the priority numbers
 *
 * void userfunction_init(void)
 * {
 *   //code for user function init
 *   // :
 *   // :
 *   IfxCpu_Irq_installInterruptHandler (Isr_Stm0, IFX_INTPRIO_STM0);
 *   IfxCpu_Irq_installInterruptHandler (Isr_Adc_fun1, IFX_INTPRIO_ADC_FUNC1);
 *
 *   // :
 * }
 *
 * \endcode
 *
 * \section IfxLld_Cpu_Irq_Step5 Step5: Managing the Service Request Node.
 * For the interrupt to get activated, interrupt triggers are needed. These triggers are activated by peripheral modules
 * and corresponding service request node must be\n
 * 1) Configured with correct priority number\n
 * 2) The request node must be enabled\n
 * Refer to \ref IfxLld_Src_Usage
 */

/** \} */
#endif /* IFXCPU_IRQ_H_ */
