/* Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OSA__OSA_TASK_H
#define OSA__OSA_TASK_H
#define FSP__OSA__OSA_TASK_H                            1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/attributes.h>
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for rtosBool, rtosTaskHandle, rtosTick, rtos...

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/ct-assert.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")


typedef void                    *rtosTaskHandle;

void enter_critical_section(void);
void exit_critical_section(void);

#define rtosTaskEnterCritical() enter_critical_section()
#define rtosTaskExitCritical() exit_critical_section()

/**
 * @brief Request task yield from ISR
 *
 * This function is a wrapper for taskYIELD_FROM_ISR() in SafeRTOS.
 * It enables yield in an interrupt context to a task of
 * equal/higher priority than current task. Returns none.
 * Does nothing if input argument is rtosFALSE.
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Called from an interrupt context
 *
 * @func_req_id 8219153
 *
 * @param[in] xSwitchRequired Bool variable to indicate need for switch
 */
static inline void
rtosTaskYieldFromISR(rtosBool xSwitchRequired)
{
    UNUSED( xSwitchRequired );
}

/**
 * @brief Generate system tick
 *
 * This function is a wrapper for vTaskProcessSystemTickFromISR() in Little Kernel.
 * It is used to generate system tick and should be called from the
 * system tick handler.
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 * No input arguments needed and returns none.
 */
static inline void rtosSystemTickHandler(void)
{
}

#endif