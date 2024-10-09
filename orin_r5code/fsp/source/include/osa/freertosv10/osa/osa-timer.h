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

#ifndef OSA__OSA_TIMER_H
#define OSA__OSA_TIMER_H
#define FSP__OSA__OSA_TIMER_H                           1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for rtosError, rtosTimerHandle, rtosTic
#include <osa/osa-values.h>             // for rtosPASS
#include <timers.h>                     // IWYU pragma: export
                                        // IWYU pragma: no_include "timers.h"
                                        // for pvTimerGetTimerID, xTimerIsTimerActive

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

/**
 * @brief Create a timer.
 *
 * This function is a wrapper for xTimerCreate() in FreeRTOS.
 * It updates the handle sent as
 * an argument to point to the newly created timer
 *
 * @pre
 * - A valid buffer, r/w accessible to the RTOS kernel at all times has been
 * pre-allocated to handle timer control block and instance parameters
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pxTimerParameters Pointer to timer init parameter
 *                              structure (rtosTimerInitParametersType)
 * @param[out] pxTimerHandle    Pointer to timer handle
 *
 * @retval rtosPASS                           on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED     Null pointer to timer init
 *                                            parameter/Timer handle
 * @retval E_RTOS_INVALID_PARAMETERS          Invalid timer init parameters
 * @retval E_RTOS_INVALID_TIMER_TASK_INSTANCE Invalid timer task instance
 * @retval E_RTOS_TIMER_ALREADY_IN_USE        Input Timer handle is valid
 *                                            and must be deleted before
 *                                            creating a new timer
 */
rtosError
rtosTimerCreate(const rtosTimerInitParametersType *const pxTimerParameters,
                rtosTimerHandle *pxTimerHandle);

/**
 * @brief Start a timer
 *
 * This function is a wrapper for xTimerStart() in FreeRTOS.
 * Calling task is held in blocked state for block time before successfully
 * sending the start command to the timer
 *
 * @pre
 * - A timer has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context.
 *
 * @param[in] xTimer     Timer handle
 * @param[in] xBlockTime Block time in ticks
 *
 * @retval rtosPASS      On success
 * @retval rtosFAIL      On failure
 */
static inline rtosError
rtosTimerStart(rtosTimerHandle xTimer,
               rtosTick xBlockTime)
{
    return xTimerStart(xTimer, xBlockTime);
}

/**
 * @brief Stop a timer
 *
 * This function is a wrapper for xTimerStop() in FreeRTOS.
 * Calling task is held in blocked state for block time before successfully
 * sending stop command to the timer
 *
 * @pre
 * - Referred timer has been started
 *
 * - Called from a non-interrupt context.
 *
 * @param[in] xTimer     Timer handle
 * @param[in] xBlockTime Block time in ticks
 *
 * @retval rtosPASS      On success
 * @retval rtosFAIL      On failure
 */
static inline rtosError
rtosTimerStop(rtosTimerHandle xTimer,
              rtosTick xBlockTime)
{
    return xTimerStop(xTimer, xBlockTime);
}

/**
 * @brief Change timer period
 *
 * This function is a wrapper for xTimerChangePeriod() in FreeRTOS.
 * Calling task is held in blocked state for block time before successfully
 * sending change period command to the timer
 *
 * @pre
 * - A timer has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context.
 *
 * @param[in] xTimer            Timer handle
 * @param[in] xBlockTime        Block time in ticks
 * @param[in] xNewPeriodInTicks New timer period in ticks. Should be
 *                              greater than 0.
 *
 * @retval rtosPASS             On success
 * @retval rtosFAIL             On failure
 */
static inline rtosError
rtosTimerChangePeriod(rtosTimerHandle xTimer,
                      rtosTick xNewPeriodInTicks,
                      rtosTick xBlockTime)
{
    return xTimerChangePeriod(xTimer, xNewPeriodInTicks, xBlockTime);
}

/**
 * @brief Delete a timer
 *
 * This function is a wrapper for xTimerDelete() in FreeRTOS.
 * Calling task is held in blocked state for block time before successfully
 * sending delete command to the timer
 *
 * @pre
 * - A timer has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context.
 *
 * @param[in] xTimer     Timer handle
 * @param[in] xBlockTime Block time in ticks
 *
 * @retval rtosPASS      On success
 * @retval rtosFAIL      On failure
 */
static inline rtosError
rtosTimerDelete(rtosTimerHandle xTimer,
                rtosTick xBlockTime)
{
    return xTimerDelete(xTimer, xBlockTime);
}

// ISR friendly commands of the timer commands

/**
 * @brief Start a timer from ISR
 *
 * This function is a wrapper for xTimerStartFromISR() in FreeRTOS.
 * It sets value at pxHigherPriorityTaskWoken if calling this function
 * unblocks the timer service whose priority is equal to/higher than
 * the current task
 *
 * @pre
 * - A timer has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from an interrupt context.
 *
 * @param[in] xTimer                     Timer handle
 * @param[out] pxHigherPriorityTaskWoken Pointer to rtosBool variable
 *                                       to indicate current context
 *
 * @retval rtosPASS                      On success
 * @retval rtosFAIL                      On failure
 */
static inline rtosError
rtosTimerStartFromISR(rtosTimerHandle xTimer,
                      rtosBool *pxHigherPriorityTaskWoken)
{
    return xTimerStartFromISR(xTimer, pxHigherPriorityTaskWoken);
}

/**
 * @brief Stop a timer from ISR
 *
 * This function is a wrapper for xTimerStopFromISR() in FreeRTOS.
 * It sets value at pxHigherPriorityTaskWoken if calling this function unblocks
 * the timer service whose priority is equal to/higher than the current task
 *
 * @pre
 * - Referred timer has been started
 *
 * - Scheduler has been started.
 *
 * - Called from an interrupt context.
 *
 * @param[in] xTimer                     Timer handle
 * @param[out] pxHigherPriorityTaskWoken Pointer to rtosBool variable
 *                                       to indicate current context
 *
 * @retval rtosPASS                      On success
 * @retval rtosFAIL                      On failure
 */
static inline rtosError
rtosTimerStopFromISR(rtosTimerHandle xTimer,
                     rtosBool *pxHigherPriorityTaskWoken)
{
    return xTimerStopFromISR(xTimer, pxHigherPriorityTaskWoken);
}

/**
 * @brief Change timer period from ISR
 *
 * This function is a wrapper for xTimerChangePeriodFromISR() in FreeRTOS.
 * It sets value at pxHigherPriorityTaskWoken if calling this function unblocks
 * the timer service whose priority is equal to/higher than the current task
 *
 * @pre
 * - A timer has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from an interrupt context.
 *
 * @param[in] xTimer                     Timer handle
 * @param[in] xNewPeriodInTicks          New timer period in ticks. Should be
 *                                       greater than 0.
 * @param[out] pxHigherPriorityTaskWoken Pointer to rtosBool variable
 *                                       to indicate current context
 *
 * @retval rtosPASS                      On success
 * @retval rtosFAIL                      On failure
 */
static inline rtosError
rtosTimerChangePeriodFromISR(rtosTimerHandle xTimer,
                             rtosTick xNewPeriodInTicks,
                             rtosBool *pxHigherPriorityTaskWoken)
{
    return xTimerChangePeriodFromISR(xTimer, xNewPeriodInTicks,
                                     pxHigherPriorityTaskWoken);
}

/**
 * @brief Check if timer is active
 *
 * This function is a wrapper for xTimerIsTimerActive() in FreeRTOS.
 *
 * @pre
 * - A timer has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context.
 *
 * @param[in] xTimer Timer handle
 *
 * @retval rtosTRUE                    if timer is active or exists in the
 *                                     overflow timer list
 * @retval rtosFALSE                   Timer not present in the active timer
 */
static inline rtosError
rtosTimerIsTimerActive(rtosTimerHandle xTimer)
{
    return xTimerIsTimerActive(xTimer);
}

/**
 * @brief Get timer ID
 *
 * This function is a wrapper for xTimerGetTimerID() in FreeRTOS.
 * It updates value at pxTimerID with ID of the requested timer
 * configured by the host application while creating it.
 *
 * @pre
 * - A timer has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context.
 *
 * @param[in]  xTimer    Timer handle
 * @param[out] pxTimerID Pointer to timer ID
 *
 * @retval rtosPASS      For valid timers
 */
static inline rtosError
rtosTimerGetTimerID(rtosTimerHandle xTimer, rtosTimerIDType *pxTimerID)
{
    pvTimerGetTimerID(xTimer);
    return rtosPASS;
}

#endif
