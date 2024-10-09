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

#ifndef OSA__OSA_VALUES_H
#define OSA__OSA_VALUES_H
#define FSP__OSA__OSA_VALUES_H                          1

/* Compiler headers */

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // for rtosMutexHandle, FSP__OSPL__RTOS_PORT_H
#include <eventgroups.h>                // for evgrpCLEAR_EVENTS_ON_EXIT_BIT, evgrpWAIT_FO...

START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
/**
 * @brief Mapping of SafeRTOS values to rtos value space
 *
 * @macro-title RTOS boolean values published by OSA layer
 *
 * @rtosTRUE  Defines true state of a boolean variable
 * @rtosFALSE Defines false state of a boolean variable
 * @rtosPASS  Defines return value indicating success
 * @rtosFAIL  Defines return value indicating failure
 */
#define rtosTRUE                                ((rtosBool)pdTRUE)
#define rtosFALSE                               ((rtosBool)pdFALSE)
#define rtosPASS                                pdPASS
#define rtosFAIL                                pdFAIL

/**
 * @brief Mapping of SafeRTOS kernel version values to rtos value space
 *
 * @macro-title Kernel version values published by OSA layer.
 *
 * @rtosKERNEL_MAJOR_VERSION Defines kernel major version number
 * @rtosKERNEL_MINOR_VERSION Defines kernel minor version number
 */
#define rtosKERNEL_MAJOR_VERSION                pdKERNEL_MAJOR_VERSION
#define rtosKERNEL_MINOR_VERSION                pdKERNEL_MINOR_VERSION

/**
 * @brief Mapping of SafeRTOS event group values to rtos value space
 *
 * @macro-title Event group values published by OSA layer
 *
 * @rtosEVENTGRP_CLEAR_EVENTS_ON_EXIT Defines bit-mask mapped to eventgroup
 *                                    control flag set to clear event group
 *                                    bits on exit
 * @rtosEVENTGRP_WAIT_FOR_ALL_BITS    Defines bit-mask mapped to eventgroup
 *                                    control flag, set to wait until all the
 *                                    requested event group bits are set.
 */
#define rtosEVENTGRP_CLEAR_EVENTS_ON_EXIT       evgrpCLEAR_EVENTS_ON_EXIT_BIT
#define rtosEVENTGRP_WAIT_FOR_ALL_BITS          evgrpWAIT_FOR_ALL_BITS

/**
 * @brief Mapping of SafeRTOS mutex values to rtos value space
 *
 * @macro-title Mutex state values published by OSA layer
 *
 * @rtosMUTEX_AVAILABLE Defines state of a mutex object indicating lock is
 *                      available
 * @rtosMUTEX_TAKEN     Defines state of a mutex object indicating lock is
 *                      acquired
 */
#define rtosMUTEX_TAKEN                         mutexTAKEN
#define rtosMUTEX_AVAILABLE                     mutexAVAILABLE

/**
 * @brief Mapping of SafeRTOS semaphore values to rtos value space
 *
 * @macro-title Semaphore value published by OSA layer
 *
 * @rtosSEMA_GIVE_NOBLOCK Defines macro to be used for setting semaphore block
 *                        time as 0.
 */
#define rtosSEMA_GIVE_NOBLOCK                   semSEM_GIVE_BLOCK_TIME

/**
 * @brief Mapping of SafeRTOS task values to rtos value space
 *
 * @macro-title Task notification values published by OSA layer
 *
 * @rtosNotification_NO_ACTION                   Sending task shall use this
 *                                               macro if no update is needed
 *                                               to the receiving task
 *                                               notification value
 * @rtosNotification_SET_BITS                    Sending task shall use this
 *                                               macro to update notification
 *                                               value of receiving task by
 *                                               ORing it with the sent value
 *                                               of the receiving task
 * @rtosNotification_INCREMENT                   Sending task shall use this
 *                                               macro to increment notification
 *                                               value of the receiving task
 * @rtosNotification_SET_VALUE_WITH_OVERWRITE    Sending task shall use this
 *                                               macro to set notification value
 *                                               of the receiving task with
 *                                               overwrite even if the previous
 *                                               notification is pending
 * @rtosNotification_SET_VALUE_WITHOUT_OVERWRITE Sending task shall use this
 *                                               macro to set notification value
 *                                               of the receiving task only if
 *                                               there is no previous
 *                                               notification pending
 * @rtosNotification_NOT_WAITING                 Defines state of a task
 *                                               indicating it's not waiting
 *                                               for any notification
 * @rtosNotification_WAITING                     Defines state of a task
 *                                               indicating it's waiting
 *                                               for a notification
 * @rtosNotification_NOTIFIED                    Defines state of a task
 *                                               indicating it has sent the
 *                                               notification
 * @rtosIDLE_PRIORITY                            Defines priority of an idle task
 */
#define rtosNotification_NO_ACTION                      taskNOTIFICATION_NO_ACTION
#define rtosNotification_SET_BITS                       taskNOTIFICATION_SET_BITS
#define rtosNotification_INCREMENT                      taskNOTIFICATION_INCREMENT
#define rtosNotification_SET_VALUE_WITH_OVERWRITE       taskNOTIFICATION_SET_VALUE_WITH_OVERWRITE
#define rtosNotification_SET_VALUE_WITHOUT_OVERWRITE    taskNOTIFICATION_SET_VALUE_WITHOUT_OVERWRITE
#define rtosNotification_NOT_WAITING                    taskNOTIFICATION_NOT_WAITING
#define rtosNotification_WAITING                        taskNOTIFICATION_WAITING
#define rtosNotification_NOTIFIED                       taskNOTIFICATION_NOTIFIED
#define rtosIDLE_PRIORITY                               taskIDLE_PRIORITY

/**
 * @brief Mapping of SafeRTOS event poll values to rtos value space
 *
 * @macro-title Event poll values published by OSA layer
 *
 * @rtoseventpollQUEUE_MESSAGE_WAITING      Defines event for a queue object
 *                                          indicating the queue contains at
 *                                          least one message waiting to be
 *                                          received.
 * @rtoseventpollQUEUE_SPACE_AVAILABLE      Defines event for a queue object
 *                                          indicating the queue has space for
 *                                          at least one message to be sent
 * @rtoseventpollSEMAPHORE_AVAILABLE        Defines event for a semaphore object
 *                                          indicating it is available
 * @rtoseventpollMUTEX_AVAILABLE            Defines event for a mutex object
 *                                          indicating it is available
 * @rtoseventpollTASK_NOTIFICATION_RECEIVED Defines event for a task indicating
 *                                          it has received a notification
 * @rtoseventpollEVENT_GROUP_BITS_SET       Defines event for an eventgroup
 *                                          object indicating requested
 *                                          eventgroup bits are set.
 */
#define rtosEventPollQUEUE_MESSAGE_WAITING         eventpollQUEUE_MESSAGE_WAITING
#define rtosEventPollQUEUE_SPACE_AVAILABLE         eventpollQUEUE_SPACE_AVAILABLE
#define rtosEventPollSEMAPHORE_AVAILABLE           eventpollSEMAPHORE_AVAILABLE
#define rtosEventPollMUTEX_AVAILABLE               eventpollMUTEX_AVAILABLE
#define rtosEventPollTASK_NOTIFICATION_RECEIVED    eventpollTASK_NOTIFICATION_RECEIVED
#define rtosEventPollEVENT_GROUP_BITS_SET          eventpollEVENT_GROUP_BITS_SET
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif
