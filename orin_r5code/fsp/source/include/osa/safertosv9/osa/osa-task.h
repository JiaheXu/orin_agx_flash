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
#include <misc/attributes.h>            // for UNUSED

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
    CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")

/**
 * @brief rtosTCBSize() computes size of task TCB needed for rtosTaskCreate.
 *
 * @macro-title Get size of task TCB
 *
 * @func_req_id 8321837
 *
 * @retval NumberOfBytes Size of TCB in bytes
 */
#define rtosTCBSize()           rtosTCB_OVERHEAD_BYTES

/**
 * @brief Initialize task scheduler
 *
 * This function is a wrapper for xTaskInitializeScheduler() in SafeRTOS to
 * initialize a task scheduler.
 * It configures scheduler private data and passes application specific data
 * to the SafeRTOS scheduler by translating structures and types used by NVIDIA
 * code to those used by SafeRTOS.
 *
 * @pre
 * - A task has been created with a valid handle referring to it.
 *
 * - Scheduler init parameters have been set.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219912
 *
 * @param[in] pxSchedInitParameters Pointer to rtosSchedParameters structure
 *                                  containing scheduler private data required for
 *                                  initialization.
 *
 * @retval rtosPASS                              on success
 * @retval E_RTOS_EXECUTING_IN_UNPRIVILEGED_MODE Processor in unprivileged mode
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED        Null init parameters
 */
rtosError
rtosTaskInitializeScheduler(const rtosSchedParameters *const pxSchedInitParameters);

/**
 * @brief Create a task
 *
 * This function is a wrapper for xTaskCreate() in SafeRTOS to create a new
 * task and place it in ready state. It provides a translation of structures
 * and types used by NVIDIA code to those used by SafeRTOS.
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
 * @func_req_id 8219111
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
rtosError
rtosTaskCreate(const rtosTaskParameters * const pxTaskParameters,
               rtosTaskHandle *pxCreatedTask);

/**
 * @brief Delete a task
 *
 * This function is a wrapper for xTaskDelete() in SafeRTOS to delete a task.
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
 * @func_req_id 8219114
 *
 * @param[in] pxTaskToDelete Input task handle
 *
 * @retval rtosPASS                   on success
 * @retval E_RTOS_INVALID_TASK_HANDLE Invalid task handle
 */
static inline rtosError
rtosTaskDelete(rtosTaskHandle pxTaskToDelete)
{
    return xTaskDelete(pxTaskToDelete);
}

/**
 * @brief Delay a task for required time
 *
 * This function is a wrapper for xTaskDelay() in SafeRTOS.
 * Places active task in blocked state for fixed number of cycles.
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219117
 *
 * @param[in] xTicksToDelay Time in ticks to delay the task. A value of 0
 *                          will cause the task to yield instead of entering the
 *                          blocked state
 *
 * @retval rtosPASS                      on success
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED Scheduler suspended
 */
static inline rtosError
rtosTaskDelay(rtosTick xTicksToDelay)
{
    return xTaskDelay(xTicksToDelay);
}

/**
 * @brief Delay a task until specified time.
 *
 * This function is a wrapper for xTaskDelayUntil() in SafeRTOS.
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
 * @func_req_id 8219120
 *
 * @param[in] pxPreviousWakeTime Pointer to variable of type rtosTick
 *                               having previous wake time. Must be initialized
 *                               with current time before first use. Updated
 *                               automatically by this call thereafter.
 * @param[in] xTimeIncrement     Time for which task must be delayed
 *                               (Cycle time period)
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED  Scheduler suspended
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null passed for previous wake time
 * @retval E_RTOS_DID_NOT_YIELD           Context Yield did not happen
 */
static inline rtosError
rtosTaskDelayUntil(rtosTick *pxPreviousWakeTime,
                   rtosTick xTimeIncrement)
{
    return xTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement);
}

/**
 * @brief Get priority of task
 *
 * This function is a wrapper for xTaskPriorityGet() in SafeRTOS.
 * It updates value at pointer to variable of type
 * rtosPriority sent as an argument with latest priority of input task
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219123
 *
 * @param[in] pxTask      Input task handle
 * @param[in] puxPriority Pointer to a variable holding task priority
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED NULL task handle/pointer
 * @retval E_RTOS_INVALID_TASK_HANDLE     Invalid task handle
 */
static inline rtosError
rtosTaskPriorityGet(rtosTaskHandle pxTask,
                    rtosPriority *puxPriority)
{
    return xTaskPriorityGet(pxTask, puxPriority);
}

/**
 * @brief Set priority of a task
 *
 * This function is a wrapper for xTaskPrioritySet() in SafeRTOS to
 * change the priority of a task.
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219126
 *
 * @param[in] pxTask        Input task handle
 * @param[in] uxNewPriority Input task priority. Maximum priority
 *                          is (configMAX_PRIORITIES - 1)
 *
 * @retval rtosPASS                   on success
 * @retval E_RTOS_INVALID_TASK_HANDLE Invalid task handle
 * @retval E_RTOS_INVALID_PRIORITY    Invalid task priority
 */
static inline rtosError
rtosTaskPrioritySet(rtosTaskHandle pxTask,
                    rtosPriority uxNewPriority)
{
    return xTaskPrioritySet(pxTask, uxNewPriority);
}

/**
 * @brief Suspend a task
 *
 * This function is a wrapper for xTaskSuspend() in SafeRTOS.
 * Places active task in suspended state
 *
 * @pre
 * - A task has been created with a valid handle referring to it
 *
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219129
 *
 * @param[in] pxTaskToSuspend Input task handle. Value of NULL
 *                            suspends the calling task
 *
 * @retval rtosPASS                      on success
 * @retval E_RTOS_SCHEDULER_SUSPENDED    Scheduler suspended
 * @retval E_RTOS_INVALID_TASK_HANDLE    Invalid handle
 * @retval E_RTOS_TASK_ALREADY_SUSPENDED Task already suspended
 */
static inline rtosError
rtosTaskSuspend(rtosTaskHandle pxTaskToSuspend)
{
    return xTaskSuspend(pxTaskToSuspend);
}

/**
 * @brief Resume a suspended task
 *
 * This function is a wrapper for xTaskResume() in SafeRTOS.
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
 * @func_req_id 8219132
 *
 * @param[in] pxTaskToResume Input task handle
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_TASK_WAS_NOT_SUSPENDED  Task was not suspended
 * @retval E_RTOS_INVALID_TASK_HANDLE     Invalid handle
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null task handle
 */
static inline rtosError
rtosTaskResume(rtosTaskHandle pxTaskToResume)
{
    return xTaskResume(pxTaskToResume);
}

/**
 * @brief Get handle of current task
 *
 * This function is a wrapper for xTaskGetCurrentTaskHandle() in SafeRTOS.
 * It returns handle of current task.
 * No input arguments needed.
 *
 * @pre
 * - Scheduler has been initialized and started
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219135
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
 * This function is a wrapper for xTaskStartScheduler() in SafeRTOS.
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
 * @func_req_id 8219138
 *
 * @param[in] xUserKernelConfigurationChecks Bool variable to be set for
 *                                           checking the pre-conditions before
 *                                           starting the scheduler.
 *
 * @retval rtsoPASS                                 on success
 * @retval E_RTOS_NO_TASKS_CREATED                  No tasks created
 * @retval E_RTOS_SCHEDULER_ALREADY_RUNNING         Scheduler is already running
 * @retval E_RTOS_BAD_OR_NO_TICK_RATE_CONFIGURATION Clock/tick rate parameter
 *                                                  invalid/null
 * @retval E_RTOS_EXECUTING_IN_UNPRIVILEGED_MODE    Processor in unprivileged
 *                                                  mode
 * @retval E_RTOS_INVALID_ALIGNMENT                 System stack address
 *                                                  misaligned
 * @retval E_RTOS_INVALID_BUFFER_SIZE               Size of the system stack is
 *                                                  insufficient
 * @retval E_RTOS_ERROR_IN_VECTOR_TABLE             SVC handler not found in the
 *                                                  vector table
 * @retval E_RTOS_NO_MPU_IN_DEVICE                  No MPU region reported by the
 *                                                  processor
 */
static inline rtosError
rtosTaskStartScheduler(rtosBool xUserKernelConfigurationChecks)
{
    UNUSED(xUserKernelConfigurationChecks);
    return (rtosError)xTaskStartScheduler();
}

/**
 * @brief Suspend a scheduler
 *
 * This function is a wrapper for xTaskSuspendScheduler() in SafeRTOS
 * It suspends scheduler irrespective of current state.
 * No input arguments needed and returns none.
 *
 * @pre
 * - Scheduler has been initialized and started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219141
 *
 */
static inline void
rtosTaskSuspendScheduler(void)
{
    vTaskSuspendScheduler();
}

/**
 * @brief Resume suspended scheduler
 *
 * This function is a wrapper for xTaskResumeScheduler() in SafeRTOS.
 * Transitions the suspended scheduler into active state.
 * No input arguments needed.
 *
 * @pre
 * - Scheduler has been started and currently in a suspended state
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219144
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_SCHEDULER_NOT_SUSPENDED Scheduler not suspended.
 */
static inline rtosBool
rtosTaskResumeScheduler(void)
{
    return (rtosBool)xTaskResumeScheduler();
}

/**
 * @brief Get ticks count
 *
 * This function is a wrapper for xTaskGetTickCount() in SafeRTOS.
 * It returns ticks count since the scheduler was started.
 * No input arguments needed.
 *
 * @pre
 * - Called from a non-interrupt context.
 *
 * @func_req_id 8219147
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
 * This function is a wrapper for xTaskGetTickCountFromISR() in SafeRTOS.
 * Returns ticks count in an interrupt context since the scheduler was started.
 * No input arguments needed.
 *
 * @pre
 * - Called from an interrupt context
 *
 * @func_req_id 8219147
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
 * This function is a wrapper for taskYIELD() in SafeRTOS.
 * It enables yield to a task of equal/higher priority
 * than current task. No input arguments needed and returns none.
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219153
 *
 */
static inline void
rtosTaskYield(void)
{
    safertosapiYIELD();
}

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
    UNUSED(xSwitchRequired);
    safertosapiYIELD_FROM_ISR();
}

/**
 * @brief Enter critical section
 *
 * This function is a wrapper for taskENTER_CRITICAL() in SafeRTOS.
 * Enables a task to enter critical section.
 * No input arguments needed and returns none.
 *
 * @pre
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219159
 *
 */
static inline void
rtosTaskEnterCritical(void)
{
    safertosapiENTER_CRITICAL();
}

/**
 * @brief Exit critical section
 *
 * This function is a wrapper for taskEXIT_CRITICAL() in SafeRTOS.
 * Enables a task to exit from critical section.
 * No input arguments needed and returns none.
 *
 * @pre
 * - RTOS state must already be in critical section
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219162
 *
 */
static inline void
rtosTaskExitCritical(void)
{
   safertosapiEXIT_CRITICAL();
}

/**
 * @brief Set required interrupts from ISR
 *
 * This function is a wrapper for taskSET_INTERRUPT_MASK_FROM_ISR() in SafeRTOS.
 * Sets the interrupt mask to mask all interrupts at the
 * configSYSTEM_INTERRUPT_PRIORITY or lower. This prevents any interrupts that
 * call a system API function from interrupting the current interrupt handler
 * It has significance only if interrupt nesting is supported on port.
 * Input arguments none.
 *
 * @pre
 * - Called only in interrupt context
 *
 * @func_req_id 8219165
 *
 * @retval uxOriginalInterruptState Original interrupt mask.
 */
static inline rtosInterruptState
rtosTaskSetInterruptMaskFromISR(void)
{
    return safertosapiSET_INTERRUPT_MASK_FROM_ISR();
}

/**
 * @brief Clear required interrupts from ISR
 *
 * This function is wrapper for taskCLEAR_INTERRUPT_MASK_FROM_ISR() in SafeRTOS.
 * It clears interrupt mask set using rtosTaskSetInterruptMaskFromISR
 * and retrieves the original mask based on input argument. Returns none.
 *
 * @pre
 * - Called only in interrupt context
 *
 * @func_req_id 8219168
 *
 * @param[in] uxOriginalInterruptState Original interrupt mask
 */
static inline void
rtosTaskClearInterruptMaskFromISR(rtosInterruptState uxOriginalInterruptState)
{
    safertosapiCLEAR_INTERRUPT_MASK_FROM_ISR(uxOriginalInterruptState);
}

/**
 * @brief Retrieve scheduler status
 *
 * This function is a wrapper for xTaskIsSchedulerStarted() in SafeRTOS.
 * Checks and returns true if there has been a successful call to
 * rtosTaskStartScheduler()
 * No input arguments
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219915
 *
 * @retval rtosTRUE  Scheduler has been started
 * @retval rtosFALSE Otherwise
 */
static inline rtosBool
rtosTaskIsSchedulerStarted(void)
{
    return (rtosBool)xTaskIsSchedulerStarted();
}

/**
 * @brief Retrieve scheduler status from ISR
 *
 * This function is a wrapper for xTaskIsSchedulerStartedFromISR() in SafeRTOS
 * Checks and returns true if there has been a successful call to
 * rtosTaskStartScheduler()
 * No input arguments
 *
 * @pre
 * - Called from an interrupt context
 *
 * @func_req_id 8219915
 *
 * @retval rtosTRUE  Scheduler has been started
 * @retval rtosFALSE Otherwise
 */
static inline rtosBool
rtosTaskIsSchedulerStartedFromISR(void)
{
    return (rtosBool)xTaskIsSchedulerStartedFromISRKrnl();
}

/**
 * @brief Wait for notification from a calling task with timeout
 *
 * This function is a wrapper for xTaskNotifyWait() in SafeRTOS
 * Each task has a notification value which is initialized to 0 when it is
 * created.
 * It blocks the calling task for notification from another task.
 *
 * @pre
 * - A task has been created with a valid handle referring to it.
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219177
 *
 * @param[in] uxBitsToClearOnEntry    Bit-mask to clear bits in the receiving
 *                                    task notification value on entering wait.
 *                                    Length of the bit-mask is 32 bits
 * @param[in] uxBitsToClearOnExit     Bit-mask to clear bits in the receiving
 *                                    task notification value before exiting wait
 *                                    Length of the bit-mask is 32 bits
 * @param[in] xTicksToWait            Timeout in ticks to wait for notification
 * @param[out] puxNotificationValue   Pointer to receiving task notification
 *                                    value. Value at this pointer is updated
 *                                    with calling task notification value as it
 *                                    was before clearing the bits using
 *                                    uxBitsToClearOnExit.
 *
 * @retval rtosPASS                         on success
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED    Scheduler is suspended.
 * @retval E_RTOS_NOTIFICATION_NOT_RECEIVED No notification has been received
 */
static inline rtosError
rtosTaskNotifyWait(rtosTaskNotifyBits uxBitsToClearOnEntry,
                   rtosTaskNotifyBits uxBitsToClearOnExit,
                   rtosTaskNotifyValue *puxNotificationValue,
                   rtosTick xTicksToWait)
{
    return xTaskNotifyWait(uxBitsToClearOnEntry,
                           uxBitsToClearOnExit,
                           puxNotificationValue,
                           xTicksToWait);
}

/**
 * @brief Send notification to waiting task.
 *
 * This function is wrapper for rtosTaskNotifySend() in SafeRTOS.
 * Each task has a notification value which is initialized to 0 when it is
 * created.
 * The function sends notification to a waiting task and updates the receiving
 * task's notification as per the set criteria and value sent as an argument.
 *
 * Each task can take one of the following notification states:
 *
 *  - rtosNotification_NOT_WAITING - Not waiting for any notification from other
 *                                   task
 *
 *  - rtosNotification_WAITING - Task is waiting for some notification
 *
 *  - rtosNotification_NOTIFIED - Task has sent a notification.
 *
 * Receiving task notification value can be updated using following criteria:
 *
 *  - rtosNotification_NO_ACTION Do not update task notification value
 *
 *  - rtosNotification_SET_BITS Set task notification value by ORing with uxValue
 *
 *  - rtosNotification_INCREMENT Increment the task notification value
 *
 *  - rtosNotification_SET_VALUE_WITH_OVERWRITE Copy uxValue to notification
 *                                              value even if the previous value
 *                                              has not been read by the task.
 *
 *  - rtosNotification_SET_VALUE_WITHOUT_OVERWRITE Set the receiving task's
 *                                                 notification value only if
 *                                                 there is no notification
 *                                                 pending.
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Another task is waiting for some notification.
 *
 * - Called from a non-interrupt context.
 *
 * @func_req_id 8219171
 *
 * @param[in] xTaskToNotify Handle of waiting task
 * @param[in] uxValue       Notification value to be updated for receiving task
 * @param[in] xAction       Receiving task notification update criteria
 *
 * @retval rtosPASS                            on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED      Null receiving task handle
 * @retval E_RTOS_INVALID_TASK_HANDLE          Invalid task handle
 * @retval E_RTOS_INVALID_PARAMETERS           Invalid notification value/action
 * @retval E_RTOS_NOTIFICATION_ALREADY_PENDING Notification pending
 */
static inline rtosError
rtosTaskNotifySend(rtosTaskHandle xTaskToNotify,
                   rtosTaskNotifyAction xAction,
                   rtosTaskNotifyValue uxValue)
{
    return xTaskNotifySend(xTaskToNotify,
                           xAction,
                           uxValue);
}

/**
 * @brief Send notification to waiting task from ISR.
 *
 * This function is a wrapper for rtosTaskNotifySendFromISR() in SafeRTOS.
 * Each task has a notification value which is initialized to 0 when it is
 * created.
 * It sends notification to a waiting task and updates
 * receiving task's notification as per the set criteria and value sent as an
 * argument.
 *
 * Each task can take one of the following notification states:
 *
 *  - rtosNotification_NOT_WAITING - Not waiting for any notification from other
 *                    task
 *
 *  - rtosNotification_WAITING - Task is waiting for some notification
 *
 *  - rtosNotification_NOTIFIED - Task has sent a notification.
 *
 * Receiving task notification value can be updated using following criteria:
 *
 *  - rtosNotification_NO_ACTION Do not update task notification value
 *
 *  - rtosNotification_SET_BITS Set task notification value as per uxValue
 *
 *  - rtosNotification_INCREMENT Increment the task notification value
 *
 *  - rtosNotification_SET_VALUE_WITH_OVERWRITE Update task notification value
 *                                              even if the previous
 *                                              notification is pending.
 *
 *  - rtosNotification_SET_VALUE_WITHOUT_OVERWRITE Set the task's notification
 *                                                 only if there is no pending
 *                                                 notification
 *
 * @pre
 * - Scheduler has been started.
 *
 * - Another task is waiting for some notification.
 *
 * - Called from an interrupt context.
 *
 * @func_req_id 8219171
 *
 * @param[in] xTaskToNotify              Handle of waiting task
 * @param[in] uxValue                    Notification value to be updated
 *                                       for receiving task
 * @param[in] xAction                    Notification update criteria
 * @param[in] pxHigherPriorityTaskWoken  An in/out parameter that is set if
 *                                       calling this function wakes an higher or
 *                                       equal priority task
 *
 * @retval rtosPASS                            on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED      Null receiving task handle
 * @retval E_RTOS_INVALID_TASK_HANDLE          Invalid task handle
 * @retval E_RTOS_INVALID_PARAMETERS           Invalid notification value/action
 * @retval E_RTOS_NOTIFICATION_ALREADY_PENDING Notification pending
 */
static inline rtosError
rtosTaskNotifySendFromISR(rtosTaskHandle xTaskToNotify,
                          rtosTaskNotifyAction xAction,
                          rtosTaskNotifyValue uxValue,
                          rtosBool *pxHigherPriorityTaskWoken)
{
    UNUSED(pxHigherPriorityTaskWoken);
    return xTaskNotifySendFromISR(xTaskToNotify,
                                  xAction,
                                  uxValue);
}

/**
 * @brief Generate system tick
 *
 * This function is a wrapper for vTaskProcessSystemTickFromISR() in SafeRTOS.
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
    vTaskProcessSystemTickFromISR();
}

#endif
