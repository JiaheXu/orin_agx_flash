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

#ifndef OSA__OSA_ERRORS_H
#define OSA__OSA_ERRORS_H
#define FSP__OSA__OSA_ERRORS_H                          1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>  // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <base/module-id.h>  // for FSP__BASE__MODULE_ID_H, MODULE_ERROR_MODULE

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>  // for errSUPPLIED_BUFFER_TOO_SMALL, errBAD_OR_...

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
    CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")

/*
 * Ensure that the native SafeRTOS error code map can map into the global
 * error code space with no other manipulation.
 */
CT_ASSERT(MODULE_ERROR_MODULE(errSUPPLIED_BUFFER_TOO_SMALL) == MODULE_ID_RTOS, "SafeRTOS error codes collide with FSP error codes.")

/**
 * @brief Mapping of SafeRTOS return codes to generic rtos error codes.
 *
 * @macro-title Public error codes from OSA layer
 *
 * @E_RTOS_SUPPLIED_BUFFER_TOO_SMALL              Error value returned if
 *                                                supplied buffer is too small
 *                                                to create a kernel object
 * @E_RTOS_INVALID_PRIORITY                       Error value returned for
 *                                                invalid task priority
 * @E_RTOS_QUEUE_FULL                             Error value returned if queue
 *                                                is full and cannot accept a
 *                                                new request
 * @E_RTOS_NULL_PARAMETER_SUPPLIED                Error value returned if null is
 *                                                supplied as an input parameter
 * @E_RTOS_INVALID_QUEUE_LENGTH                   Error value returned if
 *                                                requested queue length
 *                                                is invalid
 * @E_RTOS_INVALID_TASK_CODE_POINTER              Error value returned for
 *                                                invalid input task code pointer
 * @E_RTOS_SCHEDULER_IS_SUSPENDED                 Error value returned if request
 *                                                could not be processed because
 *                                                the scheduler was suspended
 * @E_RTOS_INVALID_TASK_HANDLE                    Error value returned for
 *                                                invalid input task handle
 * @E_RTOS_DID_NOT_YIELD                          Error value returned if task
 *                                                did not yield for a yield
 *                                                request
 * @E_RTOS_TASK_ALREADY_SUSPENDED                 Error value returned if request
 *                                                to suspend a task could not be
 *                                                processed because the task
 *                                                was already suspended.
 * @E_RTOS_TASK_WAS_NOT_SUSPENDED                 Error value returned if request
 *                                                to resume a task
 *                                                could not be processed because
 *                                                the task was not suspended.
 * @E_RTOS_SCHEDULER_ALREADY_RUNNING              Error value returned if request
 *                                                to start a scheduler could not
 *                                                be processed because
 *                                                scheduler was already running
 * @E_RTOS_INVALID_QUEUE_HANDLE                   Error value returned for
 *                                                invalid input queue handle
 * @E_RTOS_INVALID_SEMAPHORE_HANDLE               Error value returned for
 *                                                invalid input semaphore handle
 * @E_RTOS_INVALID_MUTEX_HANDLE                   Error value returned for
 *                                                invalid input mutex handle
 * @E_RTOS_ERRONEOUS_UNBLOCK                      Error value returned for
 *                                                erroneous request to
 *                                                unblock a task
 * @E_RTOS_QUEUE_EMPTY                           Error value returned if request
 *                                                to pick a task from the queue
 *                                                could not be processed because
 *                                                queue was empty
 * @E_RTOS_SEMAPHORE_ALREADY_TAKEN                Error value returned if request
 *                                                to acquire a semaphore could
 *                                                not be processed because
 *                                                semaphore was already taken
 * @E_RTOS_MUTEX_ALREADY_TAKEN                    Error value returned if
 *                                                request to acquire a mutex
 *                                                could not be processed because
 *                                                mutex was already taken
 * @E_RTOS_INVALID_TICK_VALUE                     Error value returned for
 *                                                invalid input tick value
 * @E_RTOS_INVALID_TASK_SELECTED                  Error value returned for
 *                                                invalid task selection
 * @E_RTOS_TASK_STACK_OVERFLOW                    Error value returned when task
 *                                                stack overflows
 * @E_RTOS_SCHEDULER_WAS_NOT_SUSPENDED            Error value returned if request
 *                                                to resume scheduler could not
 *                                                be processed because
 *                                                scheduler was already running.
 * @E_RTOS_INVALID_BUFFER_SIZE                    Error value returned for
 *                                                invalid input buffer size
 * @E_RTOS_BAD_OR_NO_TICK_RATE_CONFIGURATION      Error value returned for
 *                                                invalid tick rate
 * @E_RTOS_ERROR_IN_VECTOR_TABLE                  Error value returned for error
 *                                                in vector table
 * @E_RTOS_INVALID_MPU_REGION_CONFIGURATION       Error value returned for
 *                                                invalid MPU region
 *                                                configuration
 * @E_RTOS_INVALID_MMU_REGION_CONFIGURATION       Error value returned for
 *                                                invalid MMU region
 *                                                configuration
 * @E_RTOS_TASK_STACK_ALREADY_IN_USE              Error value returned if task
 *                                                stack supplied to initialize a
 *                                                scheduler is already in use
 * @E_RTOS_NO_MPU_IN_DEVICE                       Error value returned if MPU
 *                                                does not exist in the device
 * @E_RTOS_EXECUTING_IN_UNPRIVILEGED_MODE         Error value returned if a task
 *                                                is expected to run in
 *                                                privileged mode but currently
 *                                                running in unprivileged mode
 * @E_RTOS_RTS_CALCULATION_ERROR                  Error value returned on
 *                                                detection of rts calculation
 *                                                error
 * @E_RTOS_INVALID_PERCENTAGE_HANDLE              Error value returned for
 *                                                invalid percentage handle
 * @E_RTOS_INVALID_INITIAL_SEMAPHORE_COUNT        Error value returned for
 *                                                invalid initial semaphore count
 * @E_RTOS_ROM_INTEGRITY_CHECK_FAILED             Error value returned for ROM
 *                                                integrity check failures
 * @E_RTOS_IN_PROGRESS                            Error value returned if request
 *                                                is already in progress
 * @E_RTOS_INVALID_PARAMETERS                     Error value returned if
 *                                                invalid value is set for
 *                                                input parameter
 * @E_RTOS_SPURIOUS_INTERRUPT                     Error value returned for
 *                                                spurious interrupts
 * @E_RTOS_SPURIOUS_FAST_INTERRUPT                Error value returned for
                                                  spurious fast interrupts
 * @E_RTOS_RAM_INTEGRITY_CHECK_FAILED             Error value returned for RAM
 *                                                integrity check failure
 * @E_RTOS_INVALID_TIMER_HANDLE                   Error value returned for
 *                                                invalid timer handle
 * @E_RTOS_INVALID_TIMER_TASK_INSTANCE            Error value returned for
 *                                                invalid input timer task
 *                                                instance
 * @E_RTOS_TIMER_ALREADY_IN_USE                   Error value returned if request
 *                                                to create a new timer could not
 *                                                be processed because the
 *                                                supplied timer handle was
 *                                                already in use.
 * @E_RTOS_NOTIFICATION_NOT_RECEIVED              Error value returned if
 *                                                notification is
 *                                                not received until timeout
 * @E_RTOS_NOTIFICATION_ALREADY_PENDING           Error value returned if a new
 *                                                request to wait for
 *                                                notification arises while the
 *                                                task is still waiting for
 *                                                notification
 * @E_RTOS_TASK_WAS_ALSO_ON_EVENT_LIST            Error value returned if the
 *                                                task is already part of the
 *                                                event list
 * @E_RTOS_QUEUE_ALREADY_IN_USE                   Error value returned if request
 *                                                to create a new queue could not
 *                                                be processed because input
 *                                                queue handle was already in use
 * @E_RTOS_EVENT_GROUP_ALREADY_IN_USE             Error value returned if request
 *                                                to create a new event group
 *                                                could not be processed because
 *                                                input event group handle was
 *                                                already in use
 * @E_RTOS_INVALID_EVENT_GROUP_HANDLE             Error value returned for
 *                                                invalid input event group
 *                                                handle
 * @E_RTOS_EVENT_GROUP_BITS_NOT_SET               Error value returned if event
 *                                                group bits could not be set
 * @E_RTOS_EVENT_GROUP_DELETED                    Error value returned if event
 *                                                group request could not be
 *                                                processed as referred object
 *                                                is deleted
 * @E_RTOS_MUTEX_NOT_OWNED_BY_CALLER             Error value returned if mutex
 *                                                request could not be processed
 *                                                as mutex is not owned by
 *                                                the caller
 * @E_RTOS_MUTEX_CORRUPTED                        Error value returned if mutex
 *                                                got corrupted
 * @E_RTOS_NEXT_UNBLOCK_TIME_EXPIRED              Error value returned if next
 *                                                unblock time for the referred
 *                                                task expired
 * @E_RTOS_WOKEN_UP_AFTER_NEXT_UNBLOCK_TIME       Error value returned if task
 *                                                got unblocked after next
 *                                                unblock time
 * @E_RTOS_TICKLESS_MODE_NOT_SUPPORTED            Error value returned if
 *                                                tickless mode is not supported
 * @E_RTOS_SCHEDULER_IS_NOT_RUNNING               Error value returned if the request
 *                                                could not be processed because
 *                                                scheduler is not running
 * @E_RTOS_EVENT_POLL_OBJECT_ALREADY_IN_USE       Error value returned if
 *                                                request to create a new event
 *                                                poll object could not be
 *                                                processed as event poll object
 *                                                handle was already in use
 * @E_RTOS_EVENT_POLL_OBJECT_EVENTS_LIMIT_REACHED Error value returned if no
 *                                                more target objects can be
 *                                                registered with the referred
 *                                                event poll object
 * @E_RTOS_INVALID_EVENT_POLL_OPERATION           Error value returned for
 *                                                invalid event poll operation
 * @E_RTOS_INVALID_EVENT_POLL_HANDLE              Error value returned for
 *                                                invalid event poll handle
 * @E_RTOS_INVALID_EVENT_POLL_EVENTS              Error value returned for
 *                                                invalid event poll events for
 *                                                the target object
 * @E_RTOS_NO_TASKS_CREATED                       Error value returned if
 *                                                request to start a scheduler
 *                                                could not be processed as no
 *                                                tasks are created
 * @E_RTOS_INVALID_ALIGNMENT                      Error returned for invalid byte
 *                                                alignment
 * @E_RTOS_EVENT_POLL_NO_EVENTS_OCCURRED          Error returned if no event has
 *                                                occurred while waiting for
 *                                                events registered with event
 *                                                poll object
 * @E_RTOS_INVALID_DATA_RANGE                     Error value returned for
 *                                                invalid data range
 * @E_RTOS_NO_TICK_SETUP_HOOK_DEFINED             Error value returned if no hook
 *                                                has been defined to setup ticks
 */
#define E_RTOS_SUPPLIED_BUFFER_TOO_SMALL              errSUPPLIED_BUFFER_TOO_SMALL
#define E_RTOS_INVALID_PRIORITY                       errINVALID_PRIORITY
#define E_RTOS_QUEUE_FULL                             errQUEUE_FULL
#define E_RTOS_INVALID_ALIGNMENT                      errINVALID_ALIGNMENT
#define E_RTOS_NULL_PARAMETER_SUPPLIED                errNULL_PARAMETER_SUPPLIED
#define E_RTOS_INVALID_QUEUE_LENGTH                   errINVALID_QUEUE_LENGTH
#define E_RTOS_INVALID_TASK_CODE_POINTER              errINVALID_TASK_CODE_POINTER
#define E_RTOS_SCHEDULER_IS_SUSPENDED                 errSCHEDULER_IS_SUSPENDED
#define E_RTOS_INVALID_TASK_HANDLE                    errINVALID_TASK_HANDLE
#define E_RTOS_DID_NOT_YIELD                          errDID_NOT_YIELD
#define E_RTOS_TASK_ALREADY_SUSPENDED                 errTASK_ALREADY_SUSPENDED
#define E_RTOS_TASK_WAS_NOT_SUSPENDED                 errTASK_WAS_NOT_SUSPENDED
#define E_RTOS_NO_TASKS_CREATED                       errNO_TASKS_CREATED
#define E_RTOS_SCHEDULER_ALREADY_RUNNING              errSCHEDULER_ALREADY_RUNNING
#define E_RTOS_INVALID_QUEUE_HANDLE                   errINVALID_QUEUE_HANDLE
#define E_RTOS_INVALID_SEMAPHORE_HANDLE               errINVALID_SEMAPHORE_HANDLE
#define E_RTOS_INVALID_MUTEX_HANDLE                   errINVALID_MUTEX_HANDLE
#define E_RTOS_ERRONEOUS_UNBLOCK                      errERRONEOUS_UNBLOCK
#define E_RTOS_QUEUE_EMPTY                            errQUEUE_EMPTY
#define E_RTOS_SEMAPHORE_ALREADY_TAKEN                errSEMAPHORE_ALREADY_TAKEN
#define E_RTOS_MUTEX_ALREADY_TAKEN                    errMUTEX_ALREADY_TAKEN
#define E_RTOS_INVALID_TICK_VALUE                     errINVALID_TICK_VALUE
#define E_RTOS_INVALID_TASK_SELECTED                  errINVALID_TASK_SELECTED
#define E_RTOS_TASK_STACK_OVERFLOW                    errTASK_STACK_OVERFLOW
#define E_RTOS_SCHEDULER_WAS_NOT_SUSPENDED            errSCHEDULER_WAS_NOT_SUSPENDED
#define E_RTOS_INVALID_BUFFER_SIZE                    errINVALID_BUFFER_SIZE
#define E_RTOS_BAD_OR_NO_TICK_RATE_CONFIGURATION      errBAD_OR_NO_TICK_RATE_CONFIGURATION
#define E_RTOS_ERROR_IN_VECTOR_TABLE                  errERROR_IN_VECTOR_TABLE
#define E_RTOS_INVALID_MPU_REGION_CONFIGURATION       errINVALID_MPU_REGION_CONFIGURATION
#define E_RTOS_INVALID_MMU_REGION_CONFIGURATION       errINVALID_MMU_REGION_CONFIGURATION
#define E_RTOS_TASK_STACK_ALREADY_IN_USE              errTASK_STACK_ALREADY_IN_USE
#define E_RTOS_NO_MPU_IN_DEVICE                       errNO_MPU_IN_DEVICE
#define E_RTOS_EXECUTING_IN_UNPRIVILEGED_MODE         errEXECUTING_IN_UNPRIVILEGED_MODE
#define E_RTOS_RTS_CALCULATION_ERROR                  errRTS_CALCULATION_ERROR
#define E_RTOS_INVALID_PERCENTAGE_HANDLE              errINVALID_PERCENTAGE_HANDLE
#define E_RTOS_INVALID_INITIAL_SEMAPHORE_COUNT        errINVALID_INITIAL_SEMAPHORE_COUNT
#define E_RTOS_ROM_INTEGRITY_CHECK_FAILED             errROM_INTEGRITY_CHECK_FAILED
#define E_RTOS_IN_PROGRESS                            errIN_PROGRESS
#define E_RTOS_INVALID_PARAMETERS                     errINVALID_PARAMETERS
#define E_RTOS_SPURIOUS_INTERRUPT                     errSPURIOUS_INTERRUPT
#define E_RTOS_SPURIOUS_FAST_INTERRUPT                errSPURIOUS_FAST_INTERRUPT
#define E_RTOS_RAM_INTEGRITY_CHECK_FAILED             errRAM_INTEGRITY_CHECK_FAILED
#define E_RTOS_INVALID_TIMER_HANDLE                   errINVALID_TIMER_HANDLE
#define E_RTOS_INVALID_TIMER_TASK_INSTANCE            errINVALID_TIMER_TASK_INSTANCE
#define E_RTOS_TIMER_ALREADY_IN_USE                   errTIMER_ALREADY_IN_USE
#define E_RTOS_NOTIFICATION_NOT_RECEIVED              errNOTIFICATION_NOT_RECEIVED
#define E_RTOS_NOTIFICATION_ALREADY_PENDING           errNOTIFICATION_ALREADY_PENDING
#define E_RTOS_TASK_WAS_ALSO_ON_EVENT_LIST            errTASK_WAS_ALSO_ON_EVENT_LIST
#define E_RTOS_QUEUE_ALREADY_IN_USE                   errQUEUE_ALREADY_IN_USE
#define E_RTOS_EVENT_GROUP_ALREADY_IN_USE             errEVENT_GROUP_ALREADY_IN_USE
#define E_RTOS_INVALID_EVENT_GROUP_HANDLE             errINVALID_EVENT_GROUP_HANDLE
#define E_RTOS_EVENT_GROUP_BITS_NOT_SET               errEVENT_GROUP_BITS_NOT_SET
#define E_RTOS_EVENT_GROUP_DELETED                    errEVENT_GROUP_DELETED
#define E_RTOS_MUTEX_NOT_OWNED_BY_CALLER              errMUTEX_NOT_OWNED_BY_CALLER
#define E_RTOS_MUTEX_CORRUPTED                        errMUTEX_CORRUPTED
#define E_RTOS_NEXT_UNBLOCK_TIME_EXPIRED              errNEXT_UNBLOCK_TIME_EXPIRED
#define E_RTOS_WOKEN_UP_AFTER_NEXT_UNBLOCK_TIME       errWOKEN_UP_AFTER_NEXT_UNBLOCK_TIME
#define E_RTOS_TICKLESS_MODE_NOT_SUPPORTED            errTICKLESS_MODE_NOT_SUPPORTED
#define E_RTOS_EVENT_POLL_OBJECT_ALREADY_IN_USE       errEVT_MPLX_OBJECT_ALREADY_IN_USE
#define E_RTOS_SCHEDULER_IS_NOT_RUNNING               errSCHEDULER_IS_NOT_RUNNING
#define E_RTOS_EVENT_POLL_OBJECT_EVENTS_LIMIT_REACHED errEVT_MPLX_OBJECT_EVENTS_LIMIT_REACHED
#define E_RTOS_INVALID_EVENT_POLL_OPERATION           errINVALID_EVT_MPLX_OPERATION
#define E_RTOS_INVALID_EVENT_POLL_HANDLE              errINVALID_EVT_MPLX_HANDLE
#define E_RTOS_INVALID_EVENT_POLL_EVENTS              errINVALID_EVT_MPLX_EVENTS
#define E_RTOS_EVENT_POLL_NO_EVENTS_OCCURRED          errEVT_MPLX_NO_EVENTS_OCCURRED
#define E_RTOS_INVALID_DATA_RANGE                     errINVALID_DATA_RANGE
#define E_RTOS_NO_TICK_SETUP_HOOK_DEFINED             errNO_TICK_SETUP_HOOK_DEFINED
#endif
