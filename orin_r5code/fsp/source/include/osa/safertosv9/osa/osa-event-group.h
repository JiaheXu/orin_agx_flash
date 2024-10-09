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

#ifndef OSA__OSA_EVENT_GROUP_H
#define OSA__OSA_EVENT_GROUP_H
#define FSP__OSA__OSA_EVENT_GROUP_H                     1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/attributes.h>            // for UNUSED

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for rtosEventGroupHandle, rtosEventBits, rto...
#include <eventGroupsAPI.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "eventGroupsAPI.h"
                                        // for xEventGroupClearBitsFromISR, xEvent...

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
 * @brief Create an event group
 *
 * This function is a wrapper for xEventGroupCreate() in SafeRTOS to create an
 * event group object.
 * It updates pointer to the event group handle to point to the newly
 * created event group
 *
 * @pre
 * - A variable of type rtosEventGroup, r/w accessible to RTOS kernel at all
 * times has been declared to handle the event group object
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8218964
 *
 * @param[in]  pxEventGroup       Pointer to the pre-declared event group variable
 *                                of type rtosEventGroup
 * @param[out] pxEventGroupHandle Pointer to the event group handle
 *
 * @retval rtosPASS on success
 * @retval E_RTOS_EVENT_GROUP_ALREADY_IN_USE Input event group already in use.
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null Input parameter
 * @retval E_RTOS_EVENT_GROUP_ALREADY_IN_USE Event handle supplied is valid and
 *                                           must be deleted before creating a
 *                                           new group
 */
static inline rtosError
rtosEventGroupCreate(rtosEventGroup *pxEventGroup,
                     rtosEventGroupHandle *pxEventGroupHandle)
{
    return xEventGroupCreate(pxEventGroup,
                             pxEventGroupHandle);
}

/**
 * @brief Delete an event group
 *
 * This function is a wrapper for xEventGroupDelete() in SafeRTOS to delete an
 * event group object.
 *
 * @pre
 * - An event group object has been created with a valid handle referring to it
 *
 * - Called from a non-interrupt context
 *
 * - Scheduler has been started.
 *
 * @func_req_id 8218967
 *
 * @param[in] xHandle Event group handle
 *
 * @retval rtosPASS                          on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null input parameter
 * @retval E_RTOS_INVALID_EVENT_GROUP_HANDLE Invalid event group handle
 */
static inline rtosError
rtosEventGroupDelete(rtosEventGroupHandle xHandle)
{
    return xEventGroupDelete(xHandle);
}

/**
 * @brief Get event group bits.
 *
 * This function is a wrapper for xEventGroupGetBits() in SafeRTOS to
 * read the status of event group bits.
 * It updates the pointer to event group bits with latest bit-mask of
 * set fields in the event group.
 *
 * @pre
 * - An event group object has been created with a valid handle referring to it.
 *
 * - Called from a non-interrupt context.
 *
 * @func_req_id 8218970
 *
 * @param[in] xHandle        Event group handle
 * @param[in] pxEventBitsSet Pointer to event group bits
 *
 * @retval rtosPASS                          on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null input parameter
 * @retval E_RTOS_INVALID_EVENT_GROUP_HANDLE Invalid event group handle
 */
static inline rtosError
rtosEventGroupGetBits(rtosEventGroupHandle xHandle,
                      rtosEventBits *pxEventBitsSet)
{
    return xEventGroupGetBits(xHandle,
                              pxEventBitsSet);
}

/**
 * @brief Get event group bits from ISR.
 *
 * This function is a wrapper for xEventGroupGetBitsFromISR() in SafeRTOS to
 * read the status of event group bits from an ISR
 * It updates the pointer to event group bits with latest bit-mask of set fields
 * within the event group in an ISR context.
 *
 * @pre
 * - An event group object has been created with a valid handle referring to it
 *
 * - Called from an interrupt context.
 *
 * @func_req_id 8218970
 *
 * @param[in] xHandle        Event group handle
 * @param[in] pxEventBitsSet Pointer to event group bits
 *
 * @retval rtosPASS                          on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null input parameter
 * @retval E_RTOS_INVALID_EVENT_GROUP_HANDLE Invalid event group handle
 */
static inline rtosError
rtosEventGroupGetBitsFromISR(rtosEventGroupHandle xHandle,
                             rtosEventBits *pxEventBitsSet)
{
    return xEventGroupGetBitsFromISR(xHandle,
                                     pxEventBitsSet);
}

/**
 * Set event group bits
 *
 * This function is a wrapper for xEventGroupSetBits() in SafeRTOS to set one
 * or more bits to '1' in an event group
 * Sets required fields within the event group based on bit-mask sent as an
 * argument.
 *
 * @pre
 * - An event group object has been created with a valid handle referring to it
 *
 * - Called from a non-interrupt context.
 *
 * @func_req_id 8218973
 *
 * @param[in] xHandle    Event group handle
 * @param[in] xBitsToSet Bit mask to set required fields in the event group
 *                       Size of rtosEventBits type is platform dependent.
 *                       For ARM Cortex-R5, it is 32-bits. Since most
 *                       significant byte is used for control purposes, valid
 *                       bit-mask range is 0 - 23 bits.
 *
 * @retval rtosPASS                          on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null input parameter
 * @retval E_RTOS_INVALID_EVENT_GROUP_HANDLE Invalid event group handle
 * @retval E_RTOS_INVALID_PARAMETERS Invalid parameters
 */
static inline rtosError
rtosEventGroupSetBits(rtosEventGroupHandle xHandle,
                      const rtosEventBits xBitsToSet)
{
    return xEventGroupSetBits(xHandle,
                              xBitsToSet);
}

/**
 * Set event group bits from ISR
 *
 * This function is a wrapper for xEventGroupSetBitsFromISR() in SafeRTOS to
 * set one or more event group bits to '1' in an interrupt-context.
 * Sets required fields within the event group based on bit-mask sent as an
 * argument.
 * It sets the value at pbHigherPriorityTaskWoken if calling this function
 * unblocks a task of priority higher than currently running task indicating
 * that a context switch is needed before exiting the ISR
 *
 * @pre
 * - An event group object has been created with a valid handle referring to it
 *
 * - Called from an interrupt context.
 *
 * @func_req_id 8218973
 *
 * @param[in] xHandle                    Event group handle
 * @param[in] xBitsToSet                 Bit-mask to set required fields in the
 *                                       event group. Valid bit-mask range for
 *                                       ARM Cortex-R5 processor is 0 - 23 bits.
 * @param[out] pbHigherPriorityTaskWoken Pointer to rtosbool variable
 *
 * @retval rtosPASS                          on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null input parameter
 * @retval E_RTOS_INVALID_EVENT_GROUP_HANDLE Invalid event group handle
 * @retval E_RTOS_INVALID_PARAMETERS         Invalid parameters
 */
static inline rtosError
rtosEventGroupSetBitsFromISR(rtosEventGroupHandle xHandle,
                             const rtosEventBits xBitsToSet,
                             rtosBool *pbHigherPriorityTaskWoken)
{
    UNUSED(pbHigherPriorityTaskWoken);
    return xEventGroupSetBitsFromISR(xHandle,
                                     xBitsToSet);
}

/**
 * @brief Clear required event group bits
 *
 * This function is a wrapper for xEventGroupClearBits() in SafeRTOS to clear
 * one or more bits in an event group
 * Clears required fields within the event group based on bit-mask sent as an
 * argument.
 *
 * @pre
 * - An event group object has been created with a valid handle referring to it
 *
 * - Called from a non-interrupt context.
 *
 * @func_req_id 8218982
 *
 * @param[in] xHandle      Event group handle
 * @param[in] xBitsToClear Bit-mask to clear required fields.
 *                         Valid bit mask range for ARM Cortex-R5 processor is
 *                         0 - 23 bits.
 *
 * @retval rtosPASS                          on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null input parameter
 * @retval E_RTOS_INVALID_EVENT_GROUP_HANDLE Invalid event group handle
 * @retval E_RTOS_INVALID_PARAMETERS         Invalid parameters
 *
 */
static inline rtosError
rtosEventGroupClearBits(rtosEventGroupHandle xHandle,
                        const rtosEventBits xBitsToClear)
{
    return xEventGroupClearBits(xHandle,
                                xBitsToClear);
}

/**
 * @brief Clear required event group bits from ISR
 *
 * This function is a wrapper for xEventGroupClearBitsFromISR() in SafeRTOS to
 * clear one or more event group bits in an interrupt context.
 * Clears required fields within the event group based on bit-mask sent as an
 * argument.
 * It sets value at pbHigherPriorityTaskWoken if calling this function unblocks
 * a higher priority task than current task indicating a context switch is
 * needed before exiting from ISR
 *
 * @pre
 * - An event group object has been created with a valid handle referring to it
 *
 * - Called from an interrupt context.
 *
 * @func_req_id 8218982
 *
 * @param[in] xHandle                    Event group handle
 * @param[in] xBitsToClear               Bit-mask to clear required fields in
 *                                       the event group. Valid bit-mask range
 *                                       for ARM Cortex-R5 processor is 0 - 23
 *                                       bits
 * @param[out] pbHigherPriorityTaskWoken Pointer to rtosbool variable
 *
 * @retval rtosPASS                          on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null input parameter
 * @retval E_RTOS_INVALID_EVENT_GROUP_HANDLE Invalid event group handle
 * @retval E_RTOS_INVALID_PARAMETERS         Invalid parameters
 */
static inline rtosError
rtosEventGroupClearBitsFromISR(rtosEventGroupHandle xHandle,
                               const rtosEventBits xBitsToClear,
                               rtosBool *pbHigherPriorityTaskWoken)
{
    UNUSED(pbHigherPriorityTaskWoken);
    return xEventGroupClearBitsFromISR(xHandle,
                                       xBitsToClear);
}

/**
 * @brief Wait for one or more bits to set within an event group until timeout
 *
 * This function is a wrapper for xEventGroupWaitBits() in SafeRTOS to wait until
 * given pattern of event group bits is set
 * Calling task blocks on waiting for required bit fields to be set as long as wait
 * condition is met until timeout
 *
 * @pre
 * - An event group object has been created with a valid handle referring to it
 *
 * - Called from a non-interrupt context
 *
 * @func_req_id 8218988
 *
 * @param[in] xHandle         Event group handle
 * @param[in] xBitstoWaitFor  Bit-mask to check for required fields. Valid range
 *                            of bit-mask on ARM Cortex-R5 processor is
 *                            0 - 23 bits
 * @param[in] xClearOnExit    Set this bool variable to clear all the bit fields
 *                            before exiting wait
 * @param[in] xWaitforAllBits Set this bool variable if all the required bit
 *                            fields should be set to exit from wait
 * @param[in] pxEventBitsSet  Pointer to event group bits. Before exiting from
 *                            wait, value at this address is updated with the
 *                            status of event group bits
 * @param[in] xTicksToWait    Timeout in ticks for which the calling task should
 *                            be held in blocked state waiting for the pattern
 *                            condition to be set. A value of zero prevents the
 *                            task from entering the blocked state.
 *
 * @retval rtosPASS                          on success
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null input parameter
 * @retval E_RTOS_INVALID_EVENT_GROUP_HANDLE Invalid event group handle
 * @retval E_RTOS_INVALID_PARAMETERS         Invalid parameters
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null parameter for event group
 *                                           handle
 * @retval E_RTOS_SCHEDULER_IS_SUSPENDED     Scheduler is suspended
 * @retval E_RTOS_EVENT_GROUP_DELETED        Event group deleted
 * @retval E_RTOS_EVENT_GROUP_BITS_NOT_SET   Required event group bits not set
 */
static inline rtosError
rtosEventGroupWaitBits(rtosEventGroupHandle xHandle,
                       const rtosEventBits xBitsToWaitFor,
                       const rtosBool xClearOnExit,
                       const rtosBool xWaitForAllBits,
                       rtosEventBits *pxEventBitsSet,
                       rtosTick xTicksToWait)
{
    return xEventGroupWaitBits(xHandle,
                               xBitsToWaitFor,
                               (portBaseType)xClearOnExit,
                               (portBaseType)xWaitForAllBits,
                               pxEventBitsSet,
                               xTicksToWait);
}

#endif
