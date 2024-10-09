/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CPU__ARMV7_MPU_SAFERTOS_PRIV_H
#define CPU__ARMV7_MPU_SAFERTOS_PRIV_H
#define FSP__CPU__ARMV7_MPU_SAFERTOS_PRIV_H      1

/* Compiler headers */
#include <stddef.h>                  // for NULL

/* Early FSP headers */
#include <misc/ct-assert.h>          // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>             // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <osa/rtos-task.h>           // for xTCB, portTaskHandleType
#include <cpu/armv7-mpu-safertos.h>  // for mpuParameters_t, FSP__CPU__ARMV7...

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
CT_ASSERT(FSP__CPU__ARMV7_MPU_SAFERTOS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file armv7-mpu-safertos-priv.h
 * @brief Internal functions for managing the MPU under SafeRTOS
 *
 * This file provides the interfaces for the SafeRTOS MPU functions for
 * ARMv7 CPUs (notably the Cortex R5) that are used internally by the
 * MPU driver code and should not be used by application code.
 */

/**
 * @brief Restore a task's MPU regions
 *
 * This function restores the specified task's regions in that it
 * programs the MPU as indicated by the array of region structures
 * supplied.
 *
 * For performance reasons, this function is implemented in assembler
 * and as a consequence any error checking should be performed prior
 * to calling this function as it is not expected to do any validation
 * checking.  That is, it will take the information contained in the
 * supplied regions array and just write the appropriate values into
 * the MPU.
 *
 * @pre MPU must be enabled via r5mpu_init().
 * @pre The task's region array must be properly initialized by
 *      calling r5mpu_task_region_init().
 *
 * @param[in]  task_regions  A pointer to an array of mpuParameter
 *                           structures that contains exactly
 *                           MPU_NTASK_REGIONS.
 * @return None
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
void armv7_task_mpu_restore(mpuParameters_t *task_regions);

/**
 * @brief Disable the per-task MPU regions
 *
 * This function disables the per-task MPU regions. The regions
 * to be disabled are passed in as an argument.
 *
 * @pre MPU must be enabled via r5mpu_init().
 *
 * @param[in]  task_regions  A point to an array of mpuParameter
 *                           structures that contains exactly
 *                           MPU_NTASK_REGIONS.
 *
 * @return None
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
void armv7_task_mpu_disable(mpuParameters_t *task_regions);

/**
 * @brief Application specific task switch handling
 *
 * This function is called by the context switch code in SafeRTOS.
 * It will perform the necessary low level operations necessary to
 * tear down the MPU mappings from the current task and set them up
 * for the next task.
 *
 * @pre MPU must be enabled via r5mpu_init().
 * @pre The task's region arrays must be properly initialized by
 *      calling r5mpu_task_region_init().
 *
 * @param[in] currentTask  A valid task handle to the current task
 *                         or NULL.
 * @param[in] nextTask     A valid task handle to the next task
 *                         or NULL.
 *
 * @return None
 */
void vApplicationTaskSwitch(const xTCB *pxTCBOfTaskSwitchedOut,
                            const xTCB *pxTCBOfTaskSwitchedIn);

/**
 * @brief Return a pointer to a task's TCB
 *
 * This function will, given a task handle, return a pointer to that
 * task's TCB structure.
 *
 * @pre Task represended by xCurrentTaskHandle must have been created.
 *
 * @param[in] xCurrentTaskHandle Task handle of the task whose TCB
 *                               pointer is to be returned.  The task
 *                               handle is assumed to be valid as this
 *                               function is called where the task handle
 *                               is supplied from SafeRTOS.
 *
 * @return pointer to an xTCB structure for the given task.
 */
static inline xTCB *
vPortTaskHandleToTCB(const portTaskHandleType xCurrentTaskHandle)
{
    xTCB        *pxTCB;

    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    pxTCB = (xTCB *)xCurrentTaskHandle;

    return pxTCB;
}

/**
 * @brief Return a pointer to a task's MPU regions
 *
 * This function will, given a task handle, return a pointer to that
 * task's MPU parameter array.
 *
 * @pre Task represented by xCurrentTaskHandle must have been created.
 *
 * @param[in] xCurrentTaksHandle Task handle of the task whose MPU
 *                               parameters are to be returned.  The
 *                               task handle is assumed to be valid
 *                               as this function is called where the
 *                               task handle is supplied from SafeRTOS.
 *
 * @return pointer to the task's array of mpuParameter_t structures.  NULL
 *         if the task has no per-task MPU regions.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
static inline mpuParameters_t * vPortTaskHandleToMPUPointer(const portTaskHandleType xCurrentTaskHandle)
{
    const xTCB          *pxTCB;

    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    pxTCB = (const xTCB *)xCurrentTaskHandle;

    return (pxTCB == NULL) ? NULL : pxTCB->pxMPUParameters;
}

/**
 * @brief Return the task's System Mode Setting
 *
 * This function will, given a task handle, return the value of the
 * the task's system mode setting.
 *
 * @pre None
 *
 * @param[in] xCurrentTaskHandle Task handle of the task for which its
 *                               system mode setting value is to be
 *                               returned.  The task handle is assumed
 *                               to be valid as this function is called
 *                               from where the task handle has been
 *                               supplied from SafeRTOS.
 *
 * @retval R5_PSR_MODE_SYSTEM    when the task is privileged
 * @retval R5_PSR_MODE_USER      when the task is unprivileged
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
static inline portUInt32Type uxPortTaskSystemModeSetting(const portTaskHandleType xCurrentTaskHandle)
{
    const xTCB          *pxTCB;

    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    pxTCB = (const xTCB *)xCurrentTaskHandle;

    return pxTCB->ulSystemModeSetting;
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
