/* Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef OSA__OSA_SEMAPHORE_H
#define OSA__OSA_SEMAPHORE_H
#define FSP__OSA__OSA_SEMAPHORE_H                       1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */

/* Module-sepcific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for rtosSemaphoreHandle, rtosUCount, rtosBool
#include <queue.h>                      // IWYU pragma: keep
                                        // for xQueueHandle, xQueueReceive, xQueueSend,.
#include <semaphore.h>                  // IWYU pragma: export
                                        // IWYU pragma: no_include "semaphore.h"
                                        // for xSemaphoreGetCountDepth, xSemaphore...


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
 * @brief rtosSemaphoreSize() computes the size needed for rtosSemaphoreCreate.
 *
 * @macro-title Get size of semaphore object
 *
 * @func_req_id 8321837
 *
 * @retval NumberOfBytes Size of semaphore object in bytes
 */
#define rtosSemaphoreSize()     rtosSEMAPHORE_OVERHEAD_BYTES

/**
 * @brief Create a binary semaphore.
 *
 * This function is a wrapper for xSemaphoreCreateBinary() in SafeRTOS
 * to create a binary semaphore object.
 * It updates the handle sent as an argument to point to the newly
 * created semaphore
 *
 * @pre
 * - A valid buffer, r/w accessible to the RTOS kernel at all times has been
 * pre-allocated to handle a semaphore object.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8218877
 *
 * @param[in] pcSemaphoreMemory Pointer to the memory location at which
 *                              semaphore object will be created.
 *                              Clients shall use rtosSemaphoreSize() to get
 *                              the size of this pre-allocated memory.
 *                              Memory buffer shall be 4-byte aligned.
 * @param[out] xpSemaphore      Pointer to the semaphore handle
 *
 * @retval rtosPASS                         on success
 * @retval E_RTOS_INVALID_BUFFER_SIZE       Invalid input buffer size
 * @retval E_RTOS_SUPPLIED_BUFFER_TOO_SMALL Insufficient input buffer size
 */
static inline rtosError
rtosSemaphoreCreateBinary(void *pcSemaphoreMemory,
                          rtosSemaphoreHandle *xpSemaphore)
{
    return xSemaphoreCreateBinary((portInt8Type *)pcSemaphoreMemory,
                                  xpSemaphore);
}

/**
 * @brief Create a counting semaphore
 *
 * This function is a wrapper for xSemaphoreCreateCounting() in SafeRTOS
 * to create a counting semaphore object.
 * It updates the handle sent as an argument to point to the newly
 * created semaphore.
 *
 * @pre
 * - A valid buffer, r/w accessible to the RTOS kernel at all times has been
 * pre-allocated to handle a semaphore object.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8218892
 *
 * @param[in] uxMaxCount        Max limit of semaphore count.
 *                              Should be set greater than 0
 * @param[in] uxInitialCount    Min/Initial count of semaphore count.
 *                              Should be set less than uxMaxCount
 * @param[in] pcSemaphoreMemory Pointer to the memory location at which
 *                              semaphore object will be created.
 *                              Clients shall use rtosSemaphoreSize() to get
 *                              the size of this pre-allocated memory.
 *                              Memory buffer shall be 4-byte aligned.
 * @param[out] xpSemaphore      Pointer to the semaphore handle
 *
 * @retval rtosPASS                               on success
 * @retval E_RTOS_INVALID_BUFFER_SIZE             Invalid input buffer size
 * @retval E_RTOS_SUPPLIED_BUFFER_TOO_SMALL       Input buffer size tool small
 * @retval E_RTOS_INVALID_INITIAL_SEMAPHORE_COUNT Invalid semaphore initial
 * count
 */
static inline rtosError
rtosSemaphoreCreateCounting(void *pcSemaphoreMemory,
                            rtosUCount uxMaxCount,
                            rtosUCount uxInitialCount,
                            rtosSemaphoreHandle *xpSemaphore)
{
    return xSemaphoreCreateCounting(uxMaxCount,
                                    uxInitialCount,
                                    (portInt8Type *)pcSemaphoreMemory,
                                    xpSemaphore);
}

/**
 * @brief Release a semaphore
 *
 * This function is a wrapper for xSemaphoreGive() in SafeRTOS to
 * release an acquired semaphore.
 *
 * @pre
 * - A semaphore has been created with a valid handle referring to it
 *
 * - Referred Semaphore was previously acquired
 *
 * - Called from a non-interrupt context.
 *
 * - Scheduler has been started.
 *
 * @func_req_id 8218883
 *
 * @param[in] xSemaphore Semaphore handle
 *
 * @retval rtosPASS                        on success
 * @retval E_RTOS_INVALID_SEMAPHORE_HANDLE Invalid semaphore handle
 * @retval E_RTOS_SEMAPHORE_ALREADY_GIVEN  Semaphore already released
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED  Null semaphore handle
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED   Scheduler suspended
 */
static inline rtosError
rtosSemaphoreRelease(rtosSemaphoreHandle xSemaphore)
{
    return xSemaphoreGive(xSemaphore);
}

/**
 * @brief Release a semaphore from ISR
 *
 * This function is a wrapper for xSemaphoreGiveFromISR() in SafeRTOS to
 * release an acquired semaphore in interrupt context.
 * It sets value at pointer to rtos bool variable sent
 * as an argument if calling this API unblocks a task of priority equal
 * to/higher than currently running task.
 *
 * @pre
 * - A semaphore has been created with a valid handle referring to it.
 *
 * - Referred semaphore was previously acquired
 *
 * - Called from an interrupt context
 *
 * - Scheduler has been started.
 *
 * @func_req_id 8218883
 *
 * @param[in] xSemaphore                 Semaphore handle
 * @param[out] pbHigherPriorityTaskWoken Pointer to a boolean variable
 *
 * @retval rtosPASS                        on success
 * @retval E_RTOS_INVALID_SEMAPHORE_HANDLE Invalid handle
 * @retval E_RTOS_SEMAPHORE_ALREADY_GIVEN  Semaphore released already
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED  Null semaphore handle
 */
static inline rtosError
rtosSemaphoreReleaseFromISR(rtosSemaphoreHandle xSemaphore,
                            rtosBool *pbHigherPriorityTaskWoken)
{
    return xSemaphoreGiveFromISR(xSemaphore,
                                 pbHigherPriorityTaskWoken);
}

/**
 * @brief Acquire a semaphore
 *
 * This function is a wrapper for xSemaphoreTake() in SafeRTOS to
 * acquire a semaphore.
 *
 * @pre
 * - A semaphore has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context
 *
 * - Scheduler has been started.
 *
 * @func_req_id 8218880
 *
 * @param[in] xSemaphore   Semaphore handle
 * @param[in] xTicksToWait Timeout in ticks to wait until semaphore is acquired.
 *                         A value of zero prevents the calling task from
 *                         entering the Blocked state waiting for semaphore.
 *
 * @retval rtosPASS                        on success
 * @retval E_RTOS_SEMAPHORE_ALREADY_TAKEN  Semaphore already taken
 * @retval E_RTOS_INVALID_SEMAPHORE_HANDLE Invalid semaphore handle
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED  Null semaphore handle
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED   Scheduler is suspended
 */
static inline rtosError
rtosSemaphoreAcquire(rtosSemaphoreHandle xSemaphore,
                     rtosTick xTicksToWait)
{
    return xSemaphoreTake(xSemaphore, xTicksToWait);
}

/**
 * @brief Acquire semaphore from ISR
 *
 * This function is a wrapper for xSemaphoreTakeFromISR() in SafeRTOS to
 * acquire a semaphore in interrupt context.
 * It sets value at pointer to rtos bool variable sent as an argument
 * if calling this API unblocks a task of priority equal to/higher than
 * currently running task.
 *
 * @pre
 * - A semaphore has been created with a valid handle referring to it
 *
 * - Called from an interrupt context
 *
 * - Scheduler has been started.
 *
 * @func_req_id 8218880
 *
 * @param[in] xSemaphore                 Semaphore handle
 * @param[out] pbHigherPriorityTaskWoken Pointer to boolean variable
 *
 * @retval rtosPASS                        on success
 * @retval E_RTOS_SEMAPHORE_ALREADY_TAKEN  Semaphore already taken
 * @retval E_RTOS_INVALID_SEMAPHORE_HANDLE Invalid semaphore handle
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED  Null semaphore handle
 */
static inline rtosError
rtosSemaphoreAcquireFromISR(rtosSemaphoreHandle xSemaphore,
                            rtosBool *pbHigherPriorityTaskWoken)
{
    return xSemaphoreTakeFromISR(xSemaphore,
                                 pbHigherPriorityTaskWoken);
}

/**
 * @brief Get semaphore count depth
 *
 * This function is a wrapper for xSemaphoreGetCountDepth() in SafeRTOS.
 * It sets value at pointer to rtos count variable with current semaphore count.
 * This represents the number of times that rtosSemaphoreAcquire()/
 * rtosSemaphoreAcquireFromISR() can be successfully called before the next call
 * to rtosSemaphoreRelease()/rtosSemaphoreReleaseFromISR() is made.
 *
 * @pre
 * - A semaphore has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context.
 *
 * @func_req_id 9356207
 *
 * @param[in] xSemaphore     Semaphore handle
 * @param[out] puxCountDepth Pointer to rtos count variable
 *
 * @retval rtosPASS                        on success
 * @retval E_RTOS_INVALID_SEMAPHORE_HANDLE Invalid semaphore handle
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED  Null semaphore handle
 */
static inline rtosError
rtosSemaphoreGetCountDepth(rtosSemaphoreHandle xSemaphore,
                           rtosUCount *puxCountDepth)
{
    return xSemaphoreGetCountDepth(xSemaphore, puxCountDepth);
}

#endif
