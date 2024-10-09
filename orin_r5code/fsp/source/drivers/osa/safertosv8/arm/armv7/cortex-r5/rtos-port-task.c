/* Copyright (c) 2019-2021, NVIDIA CORPORATION. All rights reserved.
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

/* Compiler headers */
#include <stdint.h>                     // for int32_t

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <cpu/type-conversion.h>        // for fsp_i32_to_u32
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <osa/sections-osa.h>           // Immune from CT_ASSERT protection
#include <osa/rtos-task.h>              // for rtosTaskCreate, rtosTaskInitializeSche...
#include <osa/rtos-priv.h>              // for SETFIELD, SETFIELD_t

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
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_PRIV_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx")
/**
 * rtosTaskCreate()             - rtos wrapper for xTaskCreate()
 *
 * @pxTaskParameters:           pointer to the parameter structure that will
 *                              specify the attributes of the task being
 *                              created.
 * @pxCreatedTask:              pointer to where the task handle for the newly
 *                              created task will be stored.
 *
 * This function is a wrapper for xTaskCreate in SafeRTOS.  It provides
 * a translation of the structures and types used by NVIDIA code to those
 * used by SafeRTOS.
 *
 * Return Values:
 *      rtosPASS        if the task was created successfully
 *      other values    if the task could not be created successfully
 */
SECTION_OSA_CREATE_TEXT
rtosError
rtosTaskCreate(const rtosTaskParameters * const pxTaskParameters,
               rtosTaskHandle *pxCreatedTask)
{
    xTaskParameters     xParms;
    xTaskParameters     *pxParms        = &xParms;

    SETFIELD(pxParms, pxTaskParameters, pvTaskCode);
    SETFIELD(pxParms, pxTaskParameters, pcTaskName);
    SETFIELD(pxParms, pxTaskParameters, pxTCB);
    SETFIELD(pxParms, pxTaskParameters, pcStackBuffer);
    SETFIELD(pxParms, pxTaskParameters, uxStackDepthBytes);
    SETFIELD(pxParms, pxTaskParameters, pvParameters);
    SETFIELD(pxParms, pxTaskParameters, uxPriority);
    SETFIELD(pxParms, pxTaskParameters, pvObject);
    INLINE_RFD(MISRA, DEVIATE, Rule_10_5, "Approval: Bug 200532009, DR: SWE-FSP-020-SWSADR.docx");
    SETFIELD_t(pxParms, int32_t, pxTaskParameters, xUsingFPU);
    SETFIELD(pxParms, pxTaskParameters, uxPrivilegeLevel);
    SETFIELD(pxParms, pxTaskParameters, pxMPUParameters);

    /*
     * Call the SafeRTOS API to create the task
     */
    return xTaskCreate(pxParms, pxCreatedTask);
}


/**
 * rtosTaskInitializeScheduler()        - rtos wrapper for xTaskInitializeScheduler()
 *
 * @pxSchedInitParameters:      pointer to a structure that contains the
 *                              parameters that are used to initialize the
 *                              SafeRTOS scheduler.
 *
 * This function is a wrapper for xTaskInitializeScheduler in SafeRTOS.  It
 * provides a translation of the structures and types used by NVIDIA code to
 * those used by SafeRTOS.
 *
 * Return Values:
 *      none
 */
SECTION_OSA_INIT_TEXT
rtosError
rtosTaskInitializeScheduler(const rtosSchedParameters * const pxSchedInitParameters)
{
    xPORT_INIT_PARAMETERS       xParms;
    xPORT_INIT_PARAMETERS       *pxParms        = &xParms;

    SETFIELD(pxParms, pxSchedInitParameters, ulCPUClockHz);
    SETFIELD(pxParms, pxSchedInitParameters, ulTickRateHz);
    SETFIELD(pxParms, pxSchedInitParameters, uxAdditionalStackCheckMarginBytes);
    SETFIELD(pxParms, pxSchedInitParameters, pcIdleTaskStackBuffer);
    SETFIELD(pxParms, pxSchedInitParameters, uxIdleTaskStackSizeBytes);
    INLINE_RFD(MISRA, DEVIATE, Rule_10_5, "Approval: Bug 200532009, DR: SWE-FSP-020-SWSADR.docx");
    SETFIELD_t(pxParms, int32_t, pxSchedInitParameters, xIdleTaskUsingFPU);
    SETFIELD(pxParms, pxSchedInitParameters, pvIdleTaskTLSObject);
    SETFIELD(pxParms, pxSchedInitParameters, uxTimerTaskPriority);
    SETFIELD(pxParms, pxSchedInitParameters, uxTimerTaskStackSize);
    SETFIELD(pxParms, pxSchedInitParameters, pcTimerTaskStackBuffer);
    SETFIELD(pxParms, pxSchedInitParameters, uxTimerCommandQueueLength);
    SETFIELD(pxParms, pxSchedInitParameters, uxTimerCommandQueueBufferSize);
    SETFIELD(pxParms, pxSchedInitParameters, pcTimerCommandQueueBuffer);
    SETFIELD(pxParms, pxSchedInitParameters, pxIdleTaskMPUParameters);
    SETFIELD(pxParms, pxSchedInitParameters, pxTimerTaskMPUParameters);

    /* Always set idle task privilege level to pdTRUE */
    INLINE_RFD(MISRA, DEVIATE, Rule_10_5, "Approval: Bug 200532009, DR: SWE-FSP-020-SWSADR.docx");
    pxParms->uxIdleTaskPrivilegeLevel = fsp_i32_to_u32(pdTRUE);

    /*
     * Call the SafeRTOS API to initialize the scheduler.
     */
    return xTaskInitializeScheduler(pxParms);
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
