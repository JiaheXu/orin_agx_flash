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

#ifndef OSA__OSA_QUEUE_H
#define OSA__OSA_QUEUE_H
#define FSP__OSA__OSA_QUEUE_H                           1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT
#include <misc/attributes.h>            // for UNUSED

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-values.h>           // for rtosFALSE

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for rtosQueueHandle, rtosTick, rtosUCoun
#include <SafeRTOS_API.h>               // for vTaskProcessSystemTickFromISR, xTaskIsSc...
#include <apiBaseTypes.h>               // for portInt8Type
#include <queueAPI.h>                   // IWYU pragma: export
                                        // IWYU pragma: no_include "queueAPI.h"
                                        // IWYU pragma: no_include "apiWrapper.h"
                                        // for xQueueReceiveFromISR, xQueueSendFro...

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
    CT_ASSERT(FSP__OSA__RTOS_VALUES_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")

/**
 * @brief rtosQueueSize() computes the size needed for rtosQueueCreate.
 *
 * @_n_                 max number of elements in queue
 * @_s_                 size of a queue element
 *
 * @macro-title Get number of bytes required for the queue.
 *
 * @func_req_id 8321837
 *
 * @retval NumberOfBytes Size of queue in bytes
 */
#define rtosQueueSize(_n_, _s_) (((_n_) * (_s_)) + rtosQUEUE_OVERHEAD_BYTES)

/**
 * @brief Create a task queue
 *
 * This function is a wrapper for xQueueCreate() in SafeRTOS.
 * It updates the pointer to the queue handle to point
 * to the newly created queue
 *
 * @pre
 * - A valid buffer, r/w accessible to the RTOS kernel at all times has been
 * pre-allocated to handle a queue object.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219033
 *
 * @param[in] pcQueueMemory  Pointer to memory at which queue object
 *                           will be created. Queue buffer should be 4-byte
 *                           aligned.
 * @param[in] uxBufferLength Buffer length to handle tasks. Should be equal to
 *                           rtosQueueSize(_n_, _s_)
 * @param[in] uxQueueLength  Maximum number of items the queue can hold
 * @param[out] pxQueue       Pointer to queue handle
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_QUEUE_ALREADY_IN_USE    Queue already in use
 * @retval E_RTOS_INVALID_BYTE_ALIGNMENT  Memory not aligned
 * @retval E_RTOS_INVALID_QUEUE_LENGTH    Invalid queue length
 * @retval E_RTOS_INVALID_BUFFER_SIZE     Invalid buffer size
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null pointer to queue memory/handle
 */
static inline rtosError
rtosQueueCreate(void *pcQueueMemory,
                rtosUCount uxBufferLength,
                rtosUCount uxQueueLength,
                rtosUCount uxItemSize,
                rtosQueueHandle *pxQueue)
{
    return xQueueCreate((portInt8Type *)pcQueueMemory,
                        uxBufferLength,
                        uxQueueLength,
                        uxItemSize,
                        pxQueue);
}

/**
 * @brief: Delete queue
 *
 * This function does not exist in SafeRTOS, will return rtosPASS.
 *
 * @param[in] pxQueue       Pointer to queue handle
 *
 * @retval rtosPASS         on success
 */
static inline rtosError
rtosQueueDelete(rtosQueueHandle *pxQueue)
{
    UNUSED(pxQueue);
    return rtosPASS;
}

/**
 * @brief Add an item at the end of a queue
 *
 * This function is a wrapper for xQueueSend() in SafeRTOS.
 * Sends a item in the queue in First In First Out (FIFO) order.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219036
 *
 * @param[in] xQueue        Queue handle
 * @param[in] pvItemToQueue Pointer to the item
 * @param[in] xTicksToWait  Timeout in ticks to wait until the
 *                          item gets added. A value of 0 prevents the task
 *                          from entering blocked state.
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null item passed
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED  Scheduler suspended
 * @retval E_RTOS_QUEUE_FULL              Queue is full and item cannot be added
 */
static inline rtosError
rtosQueueSend(rtosQueueHandle rtosQueue,
              const void * const pvItemToQueue,
              rtosTick xTicksToWait)
{
    return xQueueSend(rtosQueue,
                      pvItemToQueue,
                      xTicksToWait);
}

/**
 * @brief Add an item at the end of a queue
 *
 * This function is a wrapper for xQueueSend() in SafeRTOS.
 * Sends a item in the queue in First In First Out (FIFO) order.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 *
 *
 * @param[in] xQueue        Queue handle
 * @param[in] pvItemToQueue Pointer to the item
 * @param[in] xTicksToWait  Timeout in ticks to wait until the
 *                          item gets added. A value of 0 prevents the task
 *                          from entering blocked state.
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null item passed
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED  Scheduler suspended
 * @retval E_RTOS_QUEUE_FULL              Queue is full and item cannot be added
 */
static inline rtosError
rtosQueueSendToBack(rtosQueueHandle rtosQueue,
                    const void *const pvItemToQueue,
                    rtosTick xTicksToWait)
{
    return xQueueSend(rtosQueue,
                      pvItemToQueue,
                      xTicksToWait);
}

/**
 * @brief Add an item at the front of a queue
 *
 * This function is a wrapper for xQueueSendToFront() in SafeRTOS.
 * Sends an item in the queue in Last In First Out (LIFO) order.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219039
 *
 * @param[in] xQueue        Queue handle
 * @param[in] pvItemToQueue Pointer to item
 * @param[in] xTicksToWait  Timeout in ticks to wait until
 *                          item gets added. A value of 0 prevents the task from
 *                          entering blocked state
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null item passed
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED  Scheduler suspended
 * @retval E_RTOS_QUEUE_FULL              Queue is full and item cannot be added
 */
static inline rtosError
rtosQueueSendToFront(rtosQueueHandle rtosQueue,
                     const void * const pvItemToQueue,
                     rtosTick xTicksToWait)
{
    return xQueueSendToFront(rtosQueue,
                             pvItemToQueue,
                             xTicksToWait);
}

/**
 * @brief Pick an item from the queue
 *
 * This is a wrapper for xQueueReceive() in SafeRTOS
 * It always picks item from the beginning of a queue.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219045
 *
 * @param[in] xQueue       Queue handle
 * @param[in] pvBuffer     Pointer to the buffer to copy the item. Length of
 *                         buffer should be at least equal to the item size.
 * @param[in] xTicksToWait Timeout in ticks to wait until item is picked.
 *                         A value of 0 prevents the task from entering
 *                         blocked state
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null buffer passed
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED  Scheduler is suspended
 * @retval E_RTOS_QUEUE_EMPTY             Empty queue
 */
static inline rtosError
rtosQueueReceive(rtosQueueHandle rtosQueue,
                 void *const pvBuffer,
                 rtosTick xTicksToWait)
{
    return xQueueReceive(rtosQueue,
                         pvBuffer,
                         xTicksToWait);
}

/**
 * @brief Peek an item from queue
 *
 * This function is a wrapper for xQueuePeek() in SafeRTOS
 * It picks an item without removing it from the queue.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219048
 *
 * @param[in] xQueue       Queue handle
 * @param[in] pvBuffer     Pointer to a buffer to copy the item.
 *                         Length of the buffer should be at least equal to the
 *                         queue item size.
 * @param[in] xTicksToWait Timeout in ticks to wait until item is picked.
 *                         A value of 0 prevents the task from entering the
 *                         blocked state
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null buffer passed
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED  Scheduler is suspended
 * @retval E_RTOS_QUEUE_EMPTY             Empty queue
 */
static inline rtosError
rtosQueuePeek(rtosQueueHandle rtosQueue,
              void *const pvBuffer,
              rtosTick xTicksToWait)
{
    return xQueuePeek(rtosQueue,
                      pvBuffer,
                      xTicksToWait);
}

/**
 * @brief Get number of waiting messages in the queue.
 *
 * This function is a wrapper for xQueueMessagesWaiting() in SafeRTOS.
 * It updates value at pointer to a count variable sent as an argument
 * with latest number of pending items in the queue.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8219042
 *
 * @param[in] xQueue              Queue handle
 * @param[out] puxMessagesWaiting Pointer to a count variable
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null pointer to count variable passed
 */
static inline rtosError
rtosQueueMessageWaiting(rtosQueueHandle rtosQueue,
                        rtosUCount *puxMessagesWaiting)
{
    return xQueueMessagesWaiting(rtosQueue,
                                 puxMessagesWaiting);
}

/**
 * @brief Add an item at the end of a queue from ISR
 *
 * This function is a wrapper for xQueueSendFromISR() in SafeRTOS.
 * Adds an item to the queue from interrupt context in First In First Out (FIFO)
 * It sets value at pointer to a bool variable sent as an
 * argument if adding item to the queue unblocks a higher/equal priority task
 * than the current active task.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from an interrupt context
 *
 * @func_req_id 8219036
 *
 * @param[in] xQueue                     Queue handle
 * @param[in] pvItemToQueue              Pointer to item
 * @param[out] pbHigherPriorityTaskWoken Pointer to a bool variable
 *                                       indicating current context.
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null item/context variable passed
 * @retval E_RTOS_QUEUE_FULL              Queue full and item cannot be added
 */
static inline rtosError
rtosQueueSendFromISR(rtosQueueHandle rtosQueue,
                     const void *const pvItemToQueue,
                     rtosBool *pbHigherPriorityTaskWoken)
{
    UNUSED(pbHigherPriorityTaskWoken);
    return xQueueSendFromISR(rtosQueue,
                             pvItemToQueue);
}

/**
 * @brief Add an item at the end of a queue from ISR
 *
 * This function is a wrapper for xQueueSendFromISR().
 * Adds an item to the queue from interrupt context in First In First Out (FIFO)
 * It sets value at pointer to a bool variable sent as an
 * argument if adding item to the queue unblocks a higher/equal priority task
 * than the current active task.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from an interrupt context
 *
 * @param[in] xQueue                     Queue handle
 * @param[in] pvItemToQueue              Pointer to item
 * @param[out] pbHigherPriorityTaskWoken Pointer to a bool variable
 *                                       indicating current context.
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null item/context variable passed
 * @retval E_RTOS_QUEUE_FULL              Queue full and item cannot be added
 */
static inline rtosError
rtosQueueSendToBackFromISR(rtosQueueHandle rtosQueue,
                           const void *const pvItemToQueue,
                           rtosBool *pbHigherPriorityTaskWoken)
{
    UNUSED(pbHigherPriorityTaskWoken);
    return xQueueSendFromISR(rtosQueue, pvItemToQueue);
}

/**
 * @brief Add an item at the front of a queue from ISR
 *
 * This function is a wrapper for xQueueSendToFrontFromISR() in SafeRTOS.
 * Adds an item to the queue from interrupt context in Last In First Out (LIFO)
 * order.
 * It sets value at pointer to a bool variable sent as an argument if adding
 * item to the queue unblocks a higher/equal priority task than the current
 * active task.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from an interrupt context
 *
 * @func_req_id 8219039
 *
 * @param[in] xQueue                     Queue handle
 * @param[in] pvItemToQueue              Pointer to item
 * @param[out] pbHigherPriorityTaskWoken Pointer to a bool variable
 *                                       indicating current context.
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null item/context variable passed
 * @retval E_RTOS_QUEUE_FULL              Queue full and item cannot be added
 */
static inline rtosError
rtosQueueSendToFrontFromISR(rtosQueueHandle rtosQueue,
                            const void *const pvItemToQueue,
                            rtosBool *pbHigherPriorityTaskWoken)
{
    UNUSED(pbHigherPriorityTaskWoken);
    return xQueueSendToFrontFromISR(rtosQueue,
                                    pvItemToQueue);
}

/**
 * @brief Pick an item from the queue in interrupt context
 *
 * This function is a wrapper for xQueueReceiveFromISR() in SafeRTOS
 * Picks item from the beginning of a queue in an interrupt context
 * and sets value at pointer to a bool variable sent as an argument if calling
 * this function unblocks a higher priority task than the current active task.
 *
 * @pre
 * - A queue has been created with a valid handle referring to it.
 *
 * - Scheduler has been started.
 *
 * - Called from an interrupt context
 *
 * @func_req_id 8219045
 *
 * @param[in] xQueue                      Queue handle
 * @param[in] pvBuffer                    Pointer to a buffer to copy the item.
 *                                        Length of the buffer should be at
 *                                        least equal to the item size.
 * @param[out] pbHigherPriorityTaskWoken  Pointer to a bool variable
 *                                        indicating current context.
 *
 * @retval rtosPASS                       on success
 * @retval E_RTOS_INVALID_QUEUE_HANDLE    Invalid handle passed
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED Null buffer/context variable passed
 * @retval E_RTOS_QUEUE_EMPTY             Empty queue
 */
static inline rtosError
rtosQueueReceiveFromISR(rtosQueueHandle rtosQueue,
                        void *const pvBuffer,
                        rtosBool *pbHigherPriorityTaskWoken)
{
    UNUSED(pbHigherPriorityTaskWoken);
    return xQueueReceiveFromISR(rtosQueue,
                                pvBuffer);
}

#endif
