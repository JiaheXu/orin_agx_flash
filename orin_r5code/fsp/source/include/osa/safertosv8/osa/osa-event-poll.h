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

#ifndef OSA__OSA_EVENT_POLL_H
#define OSA__OSA_EVENT_POLL_H
#define FSP__OSA__OSA_EVENT_POLL_H                      1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for rtosEventPollHandle, rtosUCount, rtosEve...
#include <queue.h>                      // IWYU pragma: keep
                                        // for xQUEUE
#include <eventpoll.h>                  // IWYU pragma: export
                                        // IWYU pragma: no_include "eventpoll.h"
                                        // for eventPollObjectEventsType

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
 * @brief Computes the size needed for rtosEventPollCreate
 *
 * @macro-title Get size of event poll object
 *
 * @func_req_id 8321837
 *
 * @retval NumberOfBytes Size of event poll object in bytes.
 */
#define rtosEventPollSize(uxMaximumRegisteredObjectEvents)\
        rtosEVENT_POLL_OVERHEAD_BYTES(uxMaximumRegisteredObjectEvents)

/**
 * @brief Create an event poll object
 *
 * This function is a wrapper for xEventPollCreate() in SafeRTOS to create
 * an event poll object
 * It updates pointer to the event poll handle to point to the newly
 * created event group
 *
 * @pre
 * - A valid buffer, r/w accessible to the RTOS kernel at all times has been
 * pre-allocated to handle an event poll object.
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8642111
 *
 * @param[in] pcEventPollMemoryBuffer         Pointer to the memory at which
 *                                            event poll object will be created
 * @param[in] uxBufferLengthInBytes           Buffer length to handle event poll
 *                                            group. It should be equal to
 *                                            rtosEventPollSize()
 * @param[in] uxMaximumRegisteredObjectEvents Limit on the number of targets
 *                                            that can register to the event
 *                                            poll group. Should be greater than 0.
 * @param[in] xOwnerTaskHandle                Handle of task creating/owning the
 *                                            event group.
 * @param[out] pxEventPollHandle              Pointer to the event poll handle
 *
 * @retval rtosPASS                                on success
 * @retval E_RTOS_EVENT_POLL_OBJECT_ALREADY_IN_USE Input event poll already in
 *                                                 use.
 * @retval E_RTOS_INVALID_DATA_RANGE               Invalid input memory address
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED          Null parameter supplied for
 *                                                 Event poll handle/Task handle
 *                                                 /Memory buffer
 * @retval E_RTOS_INVALID_BYTE_ALIGNMENT           Invalid byte alignment for
 *                                                 input buffer
 * @retval E_RTOS_INVALID_PARAMETERS               Invalid input parameter to
 *                                                 set maximum number of
 *                                                 registered objects.
 * @retval E_RTOS_INVALID_BUFFER_SIZE              Invalid input buffer size
 * @retval E_RTOS_INVALID_TASK_HANDLE              Invalid input task handle
 * @retvak E_RTOS_EVENT_GROUP_DELETED              Event group object associated with
 *                                                 an object event deleted
 */
static inline rtosError
rtosEventPollCreate(void *pcEventPollMemoryBuffer,
                    rtosUCount uxBufferLengthInBytes,
                    rtosUCount uxMaximumRegisteredObjectEvents,
                    rtosTaskHandle xOwnerTaskHandle,
                    rtosEventPollHandle * pxEventPollHandle)
{
    return xEventPollCreate((portInt8Type *)pcEventPollMemoryBuffer,
                            uxBufferLengthInBytes, uxMaximumRegisteredObjectEvents,
                            xOwnerTaskHandle, pxEventPollHandle);
}

/**
 * @brief Register a target object event to the event poll object
 *
 * This function is a wrapper for xEventPollAddObjectEvents() in SafeRTOS
 * It registers target object along with the specified events to the
 * event poll group referenced using handle sent as an argument.
 *
 * @pre
 * - An event poll object has been created with a valid handle referring to it
 *
 * - An RTOS kernel object has been created with a valid handle referring to it
 *
 * - Scheduler has been started.
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8642222
 *
 * @param[in] xEventPollHandle     Handle referring to the event poll group
 * @param[in] pvTargetObjectHandle Handle referring to the target object.
 * @param[in] uxEvents             Events to register with poll group concerning
 *                                 target object
 *
 * @retval rtosPASS                                      on success
 * @retval E_RTOS_INVALID_EVENT_POLL_OPERATION           Input target object is
 *                                                       already registered with
 *                                                       event poll object.
 * @retval E_RTOS_EVENT_POLL_OBJECT_EVENTS_LIMIT_REACHED No more target objects
 *                                                       can be registered with
 *                                                       event poll object
 * @retval E_RTOS_SCHEDULER_IS_NOT_RUNNING               Scheduler is not running
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED                Null set for event poll
 *                                                       object handle/
 *                                                       target object handle
 * @retval E_RTOS_INVALID_EVENT_POLL_EVENTS              Attempt to set invalid
 *                                                       events for the target
 *                                                       object
 * @retval E_RTOS_INVALID_EVENT_POLL_HANDLE              Invalid event poll handle
 */
static inline rtosError
rtosEventPollAddObjectEvents(rtosEventPollHandle xEventPollHandle,
                             void *pvTargetObjectHandle,
                             rtosEventPollValue uxEvents)
{
    return xEventPollAddObjectEvents(xEventPollHandle, pvTargetObjectHandle, uxEvents);
}

/**
 * @brief Modify object-events of a target object within event poll group
 *
 * This function is a wrapper for xEventPollModifyObjectEvents() in SafeRTOS.
 * It modifies object-events concerning a target object registered with
 * event poll group referred using handle sent as an argument.
 *
 * @pre
 * - Target object associated with referred object event has been registered
 * with an event poll object
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8642282
 *
 * @param[in] xEventPollHandle     Handle referring to the event poll group
 * @param[in] pvTargetObjectHandle Handle referring to the target object.
 * @param[in] uxEvents             Event poll value to be set for the target
 *                                 object registered with poll group.
 *
 * @retval rtosPASS                            on success
 * @retval E_RTOS_INVALID_EVENT_POLL_OPERATION Input target object not
 *                                             registered with referred event
 *                                             poll object.
 * @retval E_RTOS_SCHEDULER_IS_NOT_RUNNING     Scheduler is not running
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED      Null set for event poll object
 *                                             handle/Target object handle
 * @retval E_RTOS_INVALID_EVENT_POLL_EVENTS    Attempt to set invalid events
 *                                             for the target object
 * @retval E_RTOS_INVALID_EVENT_POLL_HANDLE    Invalid event poll handle
 */
static inline rtosError
rtosEventPollModifyObjectEvents(rtosEventPollHandle xEventPollHandle,
                                const void *pvTargetObjectHandle,
                                rtosEventPollValue uxEvents)
{
    return xEventPollModifyObjectEvents(xEventPollHandle, pvTargetObjectHandle,
                                        uxEvents);
}

/**
 * @brief Unregister target object along with associated events from event poll
 * object
 *
 * This is a wrapper function for xEventPollRemoveObjectEvents() in SafeRTOS.
 * It unregisters target-object along with its events from the event poll object
 *
 * @pre
 * - Referred target object has been registered with the input event poll object.
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8642333
 *
 * @param[in] xEventPollHandle     Handle referring to the event poll group
 * @param[in] pvTargetObjectHandle Handle referring to the target object.
 *
 * @retval rtosTRUE                            on success
 * @retval E_RTOS_SCHEDULER_IS_NOT_RUNNING     Scheduler is not running. Request
 *                                             cannot be processed.
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED      Null set for event poll object/
 *                                             Target object handle
 * @retval E_RTOS_INVALID_EVENT_POLL_HANDLE    Invalid event poll handle
 * @retval E_RTOS_INVALID_EVENT_POLL_OPERATION Input target object not registered
 *                                             with referred event poll object.
 */
static inline rtosError
rtosEventPollRemoveObjectEvents(rtosEventPollHandle xEventPollHandle,
                                const void *pvTargetObjectHandle)
{
     return xEventPollRemoveObjectEvents(xEventPollHandle, pvTargetObjectHandle);
}


/**
 * @brief Wait for an event registered with event poll object
 *
 * This function is a wrapper for xEventPollWait() in SafeRTOS.
 * Object-events invoked by the event poll object referenced
 * using supplied handle are copied to the input object-events array.
 * If none of the registered object-events have occurred, this function blocks
 * the calling task waiting for an object-event until timeout.
 *
 * @pre
 * - Referred target object events have been registered with input event poll
 * object.
 *
 * - Scheduler has been started
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8642990
 *
 * @param[in] xEventPollHandle         Handle referring to event poll object
 * @param[in] uxObjectEventsArraySize  Size of input array to store object-event.
 *                                     Should be at least equal to
 *                                     uxMaximumRegisteredObjectEvents defined
 *                                     during event poll object creation
 * @param[in] xTicksToWait             Timeout in ticks to wait until an event occurs.
 * @param[out] puxNumberOfObjectEvents Pointer to a count variable to store
 *                                     number of object-events that occurred
 * @param[out] axObjectEvents[]        Array to store info of object-events that
 *                                     occurred. Length of this array should be at least
 *                                     equal to uxMaximumRegisteredObjectEvents defined
 *                                     during event poll object creation
 *
 * @retval rtosPASS                            on success.
 * @retval E_RTOS_SCHEDULER_IS_NOT_RUNNING     Scheduler is not running. Request
 *                                             cannot be processed.
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED      Null set for event poll object
 *                                             handle/target object handle/array
 *                                             to store object-events/pointer to
 *                                             store number of object-events
 * @retval E_RTOS_INVALID_EVENT_POLL_HANDLE    Invalid event poll handle
 * @retval E_RTOS_INVALID_EVENT_POLL_OPERATION No object-event registered with
 *                                             referred event poll object.
 * @retval E_RTOS_INVALID_PARAMETERS           Zero size set for axObjectEvents
 *                                             array
 * @retval E_RTOS_INVALID_DATA_RANGE           Invalid length of the axObjectEvents
 *                                             array.
 */
static inline rtosError
rtosEventPollWait(rtosEventPollHandle xEventPollHandle,
                  rtosEventPollObjectEventsType axObjectEvents[],
                  rtosUCount uxObjectEventsArraySize,
                  rtosUCount * puxNumberOfObjectEvents,
                  rtosTick xTicksToWait )
{
    return xEventPollWait(xEventPollHandle,
                          (eventPollObjectEventsType *)axObjectEvents,
                          uxObjectEventsArraySize, puxNumberOfObjectEvents,
                          xTicksToWait);
}

#endif
