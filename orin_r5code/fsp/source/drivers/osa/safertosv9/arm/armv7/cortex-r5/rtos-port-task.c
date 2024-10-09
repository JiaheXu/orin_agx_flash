/* Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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

/* Module-specific FSP headers */
#include <osa/sections-osa.h>           // Immune from CT_ASSERT protection
#include <osa/rtos-task.h>              // for rtosTaskCreate, rtosTaskInitializeSche...
#include <osa/rtos-priv.h>              // for SETFIELD, SETFIELD_t
#include <ospl/rtos-port.h>             // for portTickType..,


/* The SVC stack base address - Based on the value from the linker script */
extern rtosUInt32Type Image$$SVC_STACK$$ZI$$Limit;
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
    CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__OSA__RTOS_PRIV_H, "Header file missing or invalid.")

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
    pxParms->uxIdleTaskPrivilegeLevel = pdTRUE;

#if defined(SAFERTOS_VARIANT_SVC) && (SAFERTOS_VARIANT_SVC == 1)
    pxParms->pcSVCStackBaseAddress = (void *)&Image$$SVC_STACK$$ZI$$Limit;
#endif

    /*
     * Call the SafeRTOS API to initialize the scheduler.
     */
    return xTaskInitializeScheduler(pxParms);
}


