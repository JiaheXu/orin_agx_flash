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
                                        // for rtosError, rtosSemaphoreHandle, rtosBool
#include <osa/osa-values.h>             // for rtosPASS
#include <semphr.h>                     // IWYU pragma: export
                                        // IWYU pragma: no_include "semphr.h"
                                        // for vSemaphoreDelete, xSemaphoreCreateBinary

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
 * @retval NumberOfBytes Size of semaphore object in bytes
 */
#define rtosSemaphoreSize()     4

/**
 * @brief Create a binary semaphore.
 *
 * This function is a wrapper for xSemaphoreCreateBinary() in FreeRTOS
 * to create a binary semaphore object.
 * It uses static or non-static API depending on the buffer provided by the
 * application writer.
 * It updates the handle sent as an argument to point to the newly
 * created semaphore
 *
 * @pre
 * - A valid buffer, r/w accessible to the RTOS kernel at all times has been
 * pre-allocated to handle a semaphore object.
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pcSemaphoreMemory Pointer to the memory location at which
 *                              semaphore object will be created.
 *                              Clients shall use rtosSemaphoreSize() to get
 *                              the size of this pre-allocated memory.
 *                              Memory buffer shall be 4-byte aligned.
 * @param[out] xpSemaphore      Pointer to the semaphore handle
 *
 * @retval rtosPASS             On success
 */
static inline rtosError
rtosSemaphoreCreateBinary(void *pcSemaphoreMemory,
                          rtosSemaphoreHandle *xpSemaphore)
{
    *xpSemaphore = (pcSemaphoreMemory == NULL) ?
                    xSemaphoreCreateBinary() :
                    xSemaphoreCreateBinaryStatic(pcSemaphoreMemory);
    return rtosPASS;
}

/**
 * @brief Create a counting semaphore
 *
 * This function is a wrapper for xSemaphoreCreateCounting() in FreeRTOS
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
 * @retval rtosPASS             On success
 */
static inline rtosError
rtosSemaphoreCreateCounting(void *pcSemaphoreMemory,
                            rtosUCount uxMaxCount,
                            rtosUCount uxInitialCount,
                            rtosSemaphoreHandle *xpSemaphore)
{
    *xpSemaphore = (pcSemaphoreMemory == NULL) ?
                   xSemaphoreCreateCounting(uxMaxCount,
                                            uxInitialCount) :
                   xSemaphoreCreateCountingStatic(uxMaxCount,
                                            uxInitialCount,
                                            pcSemaphoreMemory);
    return rtosPASS;
}

/**
 * @brief Release a semaphore
 *
 * This function is a wrapper for xSemaphoreGive() in FreeRTOS to
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
 * @param[in] xSemaphore Semaphore handle
 *
 * @retval rtosPASS      on success
 * @retval rtosFAIL      on failure
 */
static inline rtosError
rtosSemaphoreRelease(rtosSemaphoreHandle xSemaphore)
{
    return xSemaphoreGive(xSemaphore);
}

/**
 * @brief Release a semaphore from ISR
 *
 * This function is a wrapper for xSemaphoreGiveFromISR() in FreeRTOS to
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
 * @param[in] xSemaphore                 Semaphore handle
 * @param[out] pbHigherPriorityTaskWoken Pointer to a boolean variable
 *
 * @retval rtosPASS                      On success
 * @retval E_RTOS_QUEUE_FULL             Queue is full and item cannot be added
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
 * This function is a wrapper for xSemaphoreTake() in FreeRTOS to
 * acquire a semaphore.
 *
 * @pre
 * - A semaphore has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context
 *
 * - Scheduler has been started.
 *
 * @param[in] xSemaphore   Semaphore handle
 * @param[in] xTicksToWait Timeout in ticks to wait until semaphore is acquired.
 *                         A value of zero prevents the calling task from
 *                         entering the Blocked state waiting for semaphore.
 *
 * @retval rtosPASS        On success
 * @retval rtosFAIL        On failure
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
 * This function is a wrapper for xSemaphoreTakeFromISR() in FreeRTOS to
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
 * @param[in] xSemaphore                 Semaphore handle
 * @param[out] pbHigherPriorityTaskWoken Pointer to boolean variable
 *
 * @retval rtosPASS                      On success
 * @retval rtosFAIL                      On failure
 */
static inline rtosError
rtosSemaphoreAcquireFromISR(rtosSemaphoreHandle xSemaphore,
                            rtosBool *pbHigherPriorityTaskWoken)
{
    return xSemaphoreTakeFromISR(xSemaphore,
                                 pbHigherPriorityTaskWoken);
}

/**
 * @brief Delete Semaphore
 *
 * This function is a wrapper for vSemaphoreDelete() in FreeRTOS.
 * @pre
 * - A semaphore has been created with a valid handle referring to it.
 *
 * @param[in] xSemaphore     Semaphore handle
 *
 * @retval rtosPASS          On success
 */
static inline rtosError
rtosSemaphoreDelete(rtosSemaphoreHandle xSemaphore)
{
    vSemaphoreDelete(xSemaphore);
    return rtosPASS;
}

/**
 * @brief Get semaphore count depth
 *
 * @pre
 * - A semaphore has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context.
 *
 *
 * @param[in] xSemaphore     Semaphore handle
 * @param[out] puxCountDepth Pointer to rtos count variable
 *
 * @retval rtosPASS           on success
 */
static inline rtosError
rtosSemaphoreGetCountDepth(rtosSemaphoreHandle xSemaphore,
                           rtosUCount *puxCountDepth)
{
    *puxCountDepth = uxSemaphoreGetCount(xSemaphore);
    return rtosPASS;
}

#endif
