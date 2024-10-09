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

#ifndef OSA__OSA_MUTEX_H
#define OSA__OSA_MUTEX_H
#define FSP__OSA__OSA_MUTEX_H                           1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for rtosMutexHandle, FSP__OSPL__RTOS_PORT_H
#include <osa/osa-values.h>             // for rtosPASS
#include <semphr.h>                     // for xSemaphoreCreateMutex, xSemaphoreGiveRec...

/**
 * @brief Computes the size needed for rtosMutexCreate.
 *
 * @macro-title Get size of mutex object
 *
 * @retval NumberOfBytes Size of mutex object in bytes
 */
#define rtosMutexSize()         (1)


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
 * @brief Create a mutex lock
 *
 * This function is a wrapper for xMutexCreate() in FreeRTOS to create
 * a mutex object.
 * It uses static or non-static API depending on the buffer provided by the
 * application writer.
 * It updates pointer to the handle sent as an argument
 * with newly created mutex
 *
 * @pre
 * - A valid buffer, r/w accessible to the RTOS kernel at all times has been
 * pre-allocated to handle a mutex object
 *
 * - Called from a non-interrupt context
 *
 * @param[in] pcMutexBuffer Pointer to the memory location at which mutex
 *                          will be created. Size of the mutex buffer
 *                          is rtosMutexSize()
 *                          Mutex buffer shall be 4-byte aligned
 *
 * @retval    rtosPASS                         on success
 */
static inline rtosError
rtosMutexCreate(void *pcMutexBuffer,
                rtosMutexHandle *pxMutex)
{
    *pxMutex = (pcMutexBuffer == NULL) ?
               xSemaphoreCreateMutex() :
               xSemaphoreCreateMutexStatic(pcMutexBuffer);

     return rtosPASS;
}

/**
 * @brief Acquire a mutex
 *
 * This function is a wrapper for xMutexTake() in FreeRTOS to acquire a mutex
 *
 * @pre
 * - A mutex has been created with a valid handle referring to it.
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 * @param[in] xMutex     Mutex handle
 * @param[in] xBlocktime Timeout in ticks to wait until mutex is acquired.
 *                       A value of zero prevents the calling task from
 *                       entering the Blocked state.
 *
 * @retval    rtosPASS   on success
 * @retval    rtosFAIL   on failure
 */
static inline rtosError
rtosMutexAcquire(rtosMutexHandle xMutex,
                 rtosTick xBlockTime)
{
    return xSemaphoreTakeRecursive(xMutex,
                      xBlockTime);
}

/**
 * @brief Release acquired mutex
 *
 * This function is a wrapper for xMutexGive() in FreeRTOS to release
 * an acquired mutex.
 *
 * @pre
 * - A mutex has been created with a valid handle
 *
 * - Referred mutex was previously acquired.
 *
 * - Scheduler has been started
 *
 * - Called in a non-interrupt context
 *
 * @param[in] xMutex       Mutex handle
 *
 * @retval    rtosPASS     on success
 * @retval    rtosFAIL     on failure
 */
static inline rtosError
rtosMutexRelease(rtosMutexHandle xMutex)
{
    return xSemaphoreGiveRecursive(xMutex);
}

#endif
