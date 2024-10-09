/* Copyright (c) 2018-2021, NVIDIA CORPORATION. All rights reserved.
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

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for rtosBool, rtosTaskHandle, rtosTick, rtos...
#include <osa/osa-values.h>
#include <task.h>                       // IWYU pragma: export
                                        // IWYU pragma: no_include "task.h"
                                        // for vTaskStartScheduler, uxTaskPriorityGet

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
CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")

#define rtosIDLE_PRIORITY                   tskIDLE_PRIORITY
#define rtosTaskScheduler_Suspended         taskSCHEDULER_SUSPENDED
#define rtosTaskScheduler_Not_Started       taskSCHEDULER_NOT_STARTED
#define rtosTaskScheduler_Running           taskSCHEDULER_RUNNING

/**
 * @brief Initialize task scheduler
 *
 * This function is a wrapper for xTaskInitializeScheduler() in FreeRTOS to
 * initialize a task scheduler.
 * It configures scheduler private data and passes application specific data
 * to the FreeRTOS scheduler by translating structures and types used by NVIDIA
 * code to those used by FreeRTOS.
 *
 * @pre
 * - A task has been created with a valid handle referring to it.
 *
 * - Scheduler init parameters have been set.
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pxSchedInitParameters Pointer to rtosSchedParameters structure
 *                                  containing scheduler private data required for
 *                                  initialization.
 *
 * @retval rtosPASS                 On success
 */
static inline rtosError
rtosTaskInitializeScheduler(const rtosSchedParameters *const pxSchedInitParameters)
{
    (void) pxSchedInitParameters;
    vTaskStartScheduler();
    return rtosPASS;
}

/**
 * @brief Create a task
 *
 * This function is a wrapper for xTaskCreate() in FreeRTOS to create a new
 * task and place it in ready state. It provides a translation of structures
 * and types used by NVIDIA code to those used by FreeRTOS.
 * It uses static or non-static API depending on the buffer provided by the
 * application writer.
 * It updates the pointer to task handle using task init structure sent as
 * an argument to refer to the newly created task.
 *
 * @pre
 * - A valid buffer, r/w accessible to the RTOS kernel at all times has been
 * initialized for the task control block.
 *
 * - Task init parameters have been initialized
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pxTaskParameters Pointer to a rtosTaskParameters structure that
 *                             contains the parameters used to create a task
 * @param[out] pxCreatedTask   Pointer to task handle
 *
 * @retval rtosPASS success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED          Null pointer to handle/task
 *                                                 init structure
 * @retval E_RTOS_INVALID_PRIORITY                 Task priority above stated
 *                                                 maximum
 * @retval E_RTOS_INVALID_TASK_CODE_POINTER        Invalid pointer to task code
 * @retval E_RTOS_TASK_STACK_ALREADY_IN_USE        Memory pointed by stack
 *                                                 buffer already in use.
 * @retval E_RTOS_INVALID_TASK_CODE_POINTER        Pointer to task code
 *                                                 parameter is NULL
 * @retval E_RTOS_INVALID_BYTE_ALIGMENT            Stack buffer not aligned
 * @retval E_RTOS_INVALID_MPU_REGION_CONFIGURATION Improper size/alignment
 *                                                 of mpu region
 * @retval E_RTOS_SUPPLIED_BUFFER_TOO_SMALL        Supplied buffer not
 *                                                 sufficient to hold task
 * @retval E_RTOS_INVALID_BUFFER_SIZE              Buffer size not a power of 2
 */
static inline rtosError
rtosTaskCreate(const rtosTaskParameters * const pxTaskParameters,
               rtosTaskHandle *pxCreatedTask)
{

    if (pxTaskParameters->pxTCB == NULL)
       return xTaskCreate (pxTaskParameters->pvTaskCode,
                        pxTaskParameters->pcTaskName,
                        pxTaskParameters->uxStackDepthBytes,
                        pxTaskParameters->pvParameters,
                        pxTaskParameters->uxPriority,
                        (struct tskTaskControlBlock ** const) pxCreatedTask);
       else {
           *pxCreatedTask = (rtosTaskHandle *)xTaskCreateStatic (
                        pxTaskParameters->pvTaskCode,
                        pxTaskParameters->pcTaskName,
                        pxTaskParameters->uxStackDepthBytes,
                        pxTaskParameters->pvParameters,
                        pxTaskParameters->uxPriority,
                        pxTaskParameters->pcStackBuffer,
                        pxTaskParameters->pxTCB);

           return rtosPASS;
       }
}

/**
 * @brief Delete a task
 *
 * This function is a wrapper for xTaskDelete() in FreeRTOS to delete a task.
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * - Scheduler is in running state if the request is to delete the calling task
 *
 * @param[in] pxTaskToDelete Input task handle
 *
 * @retval rtosPASS          On success
 */
static inline rtosError
rtosTaskDelete(rtosTaskHandle pxTaskToDelete)
{
    vTaskDelete(pxTaskToDelete);
    return rtosPASS;
}

/**
 * @brief Delay a task for required time
 *
 * This function is a wrapper for xTaskDelay() in FreeRTOS.
 * Places active task in blocked state for fixed number of cycles.
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 *
 * @param[in] xTicksToDelay Time in ticks to delay the task. A value of 0
 *                          will cause the task to yield instead of entering the
 *                          blocked state
 *
 * @retval rtosPASS         On success
 */
static inline rtosError
rtosTaskDelay(rtosTick xTicksToDelay)
{
    vTaskDelay(xTicksToDelay);
    return rtosPASS;
}

/**
 * @brief Delay a task until specified time.
 *
 * This function is a wrapper for xTaskDelayUntil() in FreeRTOS.
 * It delays active task for a fixed period and wakes it
 * up again at tick(previous wake time + Delay time).
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pxPreviousWakeTime Pointer to variable of type rtosTick
 *                               having previous wake time. Must be initialized
 *                               with current time before first use. Updated
 *                               automatically by this call thereafter.
 * @param[in] xTimeIncrement     Time for which task must be delayed
 *                               (Cycle time period)
 *
 * @retval rtosPASS              On success
 */
static inline rtosError
rtosTaskDelayUntil(rtosTick *pxPreviousWakeTime,
                   rtosTick xTimeIncrement)
{
    vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement);
    return rtosPASS;
}

/**
 * @brief Get priority of task
 *
 * This function is a wrapper for xTaskPriorityGet() in FreeRTOS.
 * It updates value at pointer to variable of type
 * rtosPriority sent as an argument with latest priority of input task
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pxTask      Input task handle
 * @param[in] puxPriority Pointer to a variable holding task priority
 *
 * @retval The priority of xTask
 */
static inline rtosError
rtosTaskPriorityGet(rtosTaskHandle pxTask,
                    rtosPriority *puxPriority)
{
    return uxTaskPriorityGet(pxTask);
}

/**
 * @brief Set priority of a task
 *
 * This function is a wrapper for xTaskPrioritySet() in FreeRTOS to
 * change the priority of a task.
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pxTask        Input task handle
 * @param[in] uxNewPriority Input task priority. Maximum priority
 *                          is (configMAX_PRIORITIES - 1)
 *
 * @retval rtosPASS         On success
 */
static inline rtosError
rtosTaskPrioritySet(rtosTaskHandle pxTask,
                    rtosPriority uxNewPriority)
{
    vTaskPrioritySet(pxTask, uxNewPriority);
    return rtosPASS;
}

/**
 * @brief Suspend a task
 *
 * This function is a wrapper for xTaskSuspend() in FreeRTOS.
 * Places active task in suspended state
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pxTaskToSuspend Input task handle. Value of NULL
 *                            suspends the calling task
 *
 * @retval rtosPASS           On success
 */
static inline rtosError
rtosTaskSuspend(rtosTaskHandle pxTaskToSuspend)
{
    vTaskSuspend(pxTaskToSuspend);
    return rtosPASS;
}

/**
 * @brief Resume a suspended task
 *
 * This function is a wrapper for xTaskResume() in FreeRTOS.
 * Resumes a suspended task and puts it ready state.
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * - The task being referred was suspended.
 *
 * @param[in] pxTaskToResume Input task handle
 *
 * @retval rtosPASS          On success
 */
static inline rtosError
rtosTaskResume(rtosTaskHandle pxTaskToResume)
{
    vTaskResume(pxTaskToResume);
    return rtosPASS;
}

/**
 * @brief Get handle of current task
 *
 * This function is a wrapper for xTaskGetCurrentTaskHandle() in FreeRTOS.
 * It returns handle of current task.
 * No input arguments needed.
 *
 * @pre
 * - Scheduler has been initialized and started
 *
 * - Called from a non-interrupt context
 *
 * @retval xHandle Handle of the current task
 */
static inline rtosTaskHandle
rtosTaskGetCurrentTaskHandle(void)
{
    return xTaskGetCurrentTaskHandle();
}

/**
 * @brief Start a task scheduler
 *
 * This function is a wrapper for xTaskStartScheduler() in FreeRTOS.
 * It starts scheduler if a set of preconditions are met.
 * This can be ignored by passing input argument xUserKernelConfigurationChecks
 * as rtosFALSE in which case scheduler will start regardless of the state.
 *
 * @pre
 * - Scheduler has been initialized
 *
 * - A task has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context
 *
 * - Called with CPU in privileged mode
 *
 * @param[in] xUserKernelConfigurationChecks Bool variable to be set for
 *                                           checking the pre-conditions before
 *                                           starting the scheduler.
 *
 * @retval rtsoPASS                          On success
 */
static inline rtosError
rtosTaskStartScheduler(rtosBool xUserKernelConfigurationChecks)
{
    vTaskStartScheduler();
    return rtosPASS;
}

/**
 * @brief Suspend a scheduler
 *
 * This function is a wrapper for xTaskSuspendScheduler() in FreeRTOS
 * It suspends scheduler irrespective of current state.
 * No input arguments needed and returns none.
 *
 * @pre
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 */
static inline void
rtosTaskSuspendScheduler(void)
{
    vTaskSuspendAll();
}

/**
 * @brief Resume suspended scheduler
 *
 * This function is a wrapper for xTaskResumeScheduler() in FreeRTOS.
 * Transitions the suspended scheduler into active state.
 * No input arguments needed.
 *
 * @pre
 * - Scheduler has been started and currently in a suspended state
 *
 * - Called from a non-interrupt context
 *
 * @retval rtosPASS                       On success
 * @retval rtosFAIL                       On failure
 */
static inline rtosBool
rtosTaskResumeScheduler(void)
{
    return (rtosBool)xTaskResumeAll();
}

/**
 * @brief Get ticks count
 *
 * This function is a wrapper for xTaskGetTickCount() in FreeRTOS.
 * It returns ticks count since the scheduler was started.
 * No input arguments needed.
 *
 * @pre
 * - Called from a non-interrupt context.
 *
 * @retval xTicks Number of ticks passed since scheduler was started.
 */
static inline rtosTick
rtosTaskGetTickCount(void)
{
    return xTaskGetTickCount();
}

/**
 * @brief Get ticks count from ISR
 *
 * This function is a wrapper for xTaskGetTickCountFromISR() in FreeRTOS.
 * Returns ticks count in an interrupt context since the scheduler was started.
 * No input arguments needed.
 *
 * @pre
 * - Called from an interrupt context
 *
 * @retval xTicks Number of ticks passed since scheduler was started.
 */
static inline rtosTick
rtosTaskGetTickCountFromISR(void)
{
    return xTaskGetTickCountFromISR();
}

/**
 * @brief Request task yield.
 *
 * This function is a wrapper for taskYIELD() in FreeRTOS.
 * It enables yield to a task of equal/higher priority
 * than current task. No input arguments needed and returns none.
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 */
static inline void
rtosTaskYield(void)
{
    taskYIELD();
}

/**
 * @brief Request task yield from ISR
 *
 * This function is a wrapper for taskYIELD_FROM_ISR() in FreeRTOS.
 * It enables yield in an interrupt context to a task of
 * equal/higher priority than current task. Returns none.
 * Does nothing if input argument is rtosFALSE.
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Called from an interrupt context
 *
 * @param[in] xSwitchRequired Bool variable to indicate need for switch
 */
static inline void
rtosTaskYieldFromISR(rtosPortBaseType xSwitchRequired)
{
    portYIELD_FROM_ISR(xSwitchRequired);
}

/**
 * @brief Enter critical section
 *
 * This function is a wrapper for taskENTER_CRITICAL() in FreeRTOS.
 * Enables a task to enter critical section.
 * No input arguments needed and returns none.
 *
 * @pre
 * - Called from a non-interrupt context
 *
 */
static inline void
rtosTaskEnterCritical(void)
{
    taskENTER_CRITICAL();
}

/**
 * @brief Exit critical section
 *
 * This function is a wrapper for taskEXIT_CRITICAL() in FreeRTOS.
 * Enables a task to exit from critical section.
 * No input arguments needed and returns none.
 *
 * @pre
 * - RTOS state must already be in critical section
 *
 * - Called from a non-interrupt context
 *
 */
static inline void
rtosTaskExitCritical(void)
{
    taskEXIT_CRITICAL();
}


/**
 * @brief Disable interrupts
 *
 * This function is a wrapper for taskDISABLE_INTERRUPTS() in FreeRTOS.
 * Disable all maskable interrupts
 * No input arguments needed and returns none.
 *
 * @pre
 * - RTOS state must already be in critical section
 *
 * - Called from a non-interrupt context
 *
 */
static inline void
rtosTaskDisableInterrupts(void)
{
    taskDISABLE_INTERRUPTS();
}

/**
 * @brief Enable interrupts
 *
 * This function is a wrapper for taskENABLE_INTERRUPTS() in FreeRTOS.
 * Enable all maskable interrupts
 * No input arguments needed and returns none.
 *
 * @pre
 * - RTOS state must already be in critical section
 *
 * - Called from a non-interrupt context
 *
 */
static inline void
rtosTaskEnableInterrupts(void)
{
    taskENABLE_INTERRUPTS();
}

/**
 * @brief Retrieve scheduler status
 *
 * This function is a wrapper for xTaskIsSchedulerStarted() in FreeRTOS.
 * Checks and returns true if there has been a successful call to
 * rtosTaskStartScheduler()
 * No input arguments
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 * @retval rtosTRUE  Scheduler has been started
 * @retval rtosFALSE Otherwise
 */
static inline rtosBool
rtosTaskIsSchedulerStarted(void)
{
    return (rtosBool)xTaskGetSchedulerState();
}

/**
 * @brief Generate system tick
 *
 * This function is a wrapper for FreeRTOS_Tick_Handler() in FreeRTOS.
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
    FreeRTOS_Tick_Handler();
}

#endif
