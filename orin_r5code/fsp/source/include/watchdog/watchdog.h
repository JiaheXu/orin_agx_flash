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

#ifndef WATCHDOG__WATCHDOG_H
#define WATCHDOG__WATCHDOG_H
#define FSP__WATCHDOG__WATCHDOG_H                       1

/* Compiler headers */
#include <stdint.h>                        // for uint64_t, uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>                // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-task.h>                 // for rtosTaskHandle, FSP__OSA__...
#include <error/common-errors.h>           // for error_t, FSP__ERROR__COMMO...
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <watchdog/watchdog-types.h>       // for WatchdogTaskID, FSP__WATCH...
#include <watchdog/watchdog-statistics.h>  // for FSP__WATCHDOG__WATCHDOG_ST...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_TYPES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_STATISTICS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file watchdog.h
 * @brief Exported structures and interfaces for watchdog operations.
 */

/**
 * @brief Initialize the watchdog infrastructure
 *
 * Initializes the watchdog infrastructure to allow for the other
 * watchdog APIs to be used.  If other APIs are called prior to this
 * being, they will return an error.
 *
 * @pre None
 *
 * @param[in] minTimeout      minimum time (in nano-seconds) for a task's
 *                            timeout value
 * @param[in] maxTimeout      maximum time (in nano-seconds) for a task's
 *                            timeout value
 * @param[in] defaultTimeout  default value (in nano-seconds) to apply to
 *                            a task's timeout
 *
 * @return Error code
 * @retval E_SUCCESS    Watchdog infrastructure was initialized successfully.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogInit(uint64_t minTimeout,
                     uint64_t maxTimeout,
                     uint64_t defaultTimeout);

/**
 * @brief Indicate that a task is to be monitored
 *
 * Indicate to the watchdog infrastructure that a task wants to be monitored
 *
 * @pre WatchdogInit() must have been previously called.
 *
 * @param[in]     taskID    RTOS task ID of the task to be monitored
 * @param[in]     maxTime   maximum time the task can spend processing an item
 * @param[in,out] pID       watchdog ID assigned to the task
 *
 * @return Error code
 *
 * @retval E_SUCCESS                    indicates that the task will now be
 *                                      monitored
 * @retval E_WDT_ERR_TOO_MANY_TASKS     indicates that there are too many tasks
 *                                      being monitored
 * @retval E_WDT_ERR_DUPLICATE_TASK     indicates that the task is already being
 *                                      monitored
 * @retval E_WDT_ERR_NULL_PTR           indicates that pID was NULL
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT  indicates that the watchdog framework
 *                                      has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogTaskMonitor(const rtosTaskHandle taskID, uint64_t maxTime,
                            WatchdogTaskID *pID);

/**
 * @brief Indicate that an item is waiting to be processed
 *
 * Indicates that an item has been added for the indicated task to operate on.
 *
 * This cannot be called from interrupt context.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @param[in] ID  ID of the task whose work list has been added to
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                    indicates that item was successfully
 *                                      logged to the task
 * @retval E_WDT_ERR_NO_TASK            indicates that the task ID was not valid
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT  indicates that the watchdog framework
 *                                      has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogAddItem(const WatchdogTaskID ID);

/**
 * @brief Indicate that an item is waiting to be processed.
 *
 * Indicates that an item has been added for the indicated task to operate on.
 *
 * This can only be called from an interrupt context.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @param[in] ID  ID of the task whose work list has been added to
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                    indicates that item was successfully
 *                                      logged to the task
 * @retval E_WDT_ERR_NO_TASK            indicates that the task ID was not valid
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT  indicates that the watchdog framework
 *                                      has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogAddItemFromISR(const WatchdogTaskID ID);

/**
 * @brief Indicate that a task has started working on an item
 *
 * Informs the watchdog framework that the task has started working on a
 * previously submitted item.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 * @pre WatchdogAddItem() or WatchdogAddItemFromISR() must have been called
 *      such that it appears to the watchdog framework that there is work pending
 *      for the task.
 *
 * @param[in] ID  ID of the task that is starting to work on an item
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                     indicates that the watchdog framework
 *                                       was successfully informed
 * @retval E_WDT_ERR_NO_TASK             indicates that the task ID was not valid
 * @retval E_WDT_ERR_NO_WORK             indicates that no items have been added
 *                                       to indicate that there is actually work
 *                                       for the task to perform
 * @retval E_WDT_ERR_SEQUNECE            indicates that there was no intervening
 *                                       WatchdogCompleteItem() after a previous
 *                                       call to WatchdogStartItem()
 * @retval E_WDT_ERR_TASK_OUT_OF_BOUNDS  indicates that the task exceeded its
 *                                       timeout during a previous call to
 *                                       WatchdogStartItem()
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT   indicates that the watchdog framework
 *                                       has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogTaskStartItem(const WatchdogTaskID ID);

/**
 * @brief Indicate that current task has started working on an item.
 *
 * Informs the watchdog framework that the current task has started working on a
 * previously submitted item.
 *
 * This function must be called from a non-interrupt context.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 * @pre WatchdogAddItem() or WatchdogAddItemFromISR() must have been called
 *      such that it appears to the watchdog framework that there is work pending
 *      for the task.
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                     indicates that the watchdog framework
 *                                       was successfully informed
 * @retval E_WDT_ERR_NO_TASK             indicates that the current task is not
 *                                       monitored
 * @retval E_WDT_ERR_NO_WORK             indicates that no items have been added
 *                                       to indicate that there is actually work
 *                                       to do
 * @retval E_WDT_ERR_SEQUNECE            indicates that there was no intervening
 *                                       WatchdogCompleteItem() after a previous
 *                                       call to WatchdogStartItem()
 * @retval E_WDT_ERR_TASK_OUT_OF_BOUNDS  indicates that the task exceeded its
 *                                       timeout during a previous call to
 *                                       WatchdogStartItem()
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT   indicates that the watchdog framework
 *                                       has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogStartItem(void);

/**
 * @brief Indicate a task has completed working on an item
 *
 * Informs the watchdog framework that the task has completed working on a
 * previously submitted item.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 * @pre WatchdogAddItem() or WatchdogAddItemFromISR() must have been called
 *      such that it appears to the watchdog framework that there is work queued
 *      for the task.
 *
 * @param[in] ID  ID of the task that is completing work on an item
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                     indicates that the watchdog framework
 *                                       has been successfully informed
 * @retval E_WDT_ERR_NO_TASK             indicates that this is being called
 *                                       from a task that has not been
 *                                       registered to be monitored or that the
 *                                       ID is not a valid ID
 * @retval E_WDT_ERR_NO_WORK             indicates that no items have been added
 *                                       to indicate that there is actually work
 *                                       to do
 * @retval E_WDT_ERR_SEQUNECE            indicates that there was no intervening
 *                                       WatchdogStartItem() after a previous
 *                                       call to WatchdogCompleteItem()
 * @retval E_WDT_ERR_TASK_OUT_OF_BOUNDS  indicates that the task exceeded its
 *                                       timeout during a previous call to
 *                                       WatchdogStartItem()
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT   indicates that the watchdog framework
 *                                       had not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogTaskCompleteItem(WatchdogTaskID ID);

/**
 * @brief Indicate that current task has completed working on an item
 *
 * Informs the watchdog framework that the current task has completed working
 * on a previously submitted item.
 *
 * This function must be called from a non-interrupt context.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 * @pre WatchdogTaskStartItem() or WatchdogStartItem() must have been previously
 *      called with no other intervening calls to WatchdogTaskCompleteItem() or
 *      WatchdogCompleteItem().
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                     indicates that the watchdog framework
 *                                       has been successfully informed
 * @retval E_WDT_ERR_NO_TASK             indicates that this is being called
 *                                       from a task that has not been
 *                                       registered to be monitored
 * @retval E_WDT_ERR_NO_WORK             indicates that no items have been added
 *                                       to indicate that there is actually work
 *                                       to do
 * @retval E_WDT_ERR_SEQUNECE            indicates that there was no intervening
 *                                       WatchdogStartItem() after a previous
 *                                       call to WatchdogCompleteItem()
 * @retval E_WDT_ERR_TASK_OUT_OF_BOUNDS  indicates that the task exceeded its
 *                                       timeout during a previous call to
 *                                       WatchdogStartItem()
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT   indicates that the watchdog framework
 *                                       had not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogCompleteItem(void);

/**
 * @brief Check the state of all tasks being monitored
 *
 * Checks the state of all the tasks being monitored by the watchdog framework
 * and return the overall state.
 *
 * This function must be called from a non-interrupt context.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                     indicates that all tasks are within
 *                                       their watchdog parameters
 * @retval E_WDT_ERR_TASK_OUT_OF_BOUNDS  indicates that one or more tasks are
 *                                       outside of the bounds specified
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT   indicates that the watchdog framework
 *                                       has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogCheckStatus(void);

/**
 * @brief Check the status of a particular task being monitored
 *
 * Checks the state of a particular task that is being monitored by the watchdog
 * framework and return its state.
 *
 * This function will not invoke the callback WatchdogCallbackTaskOutOfBounds().
 *
 * This function must be called from a non-interrupt context.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @param[in] ID  ID of the task whose state is to be returned
 *
 * @return Error Code
 *
 * @return E_SUCCESS                     indicates that the task is within its
 *                                       specified bounds
 *         E_WDT_ERR_NO_TASK             indicates that the task ID does not
 *                                       correspond to a monitored task
 *         E_WDT_ERR_TASK_OUT_OF_BOUNDS  indicates that the task's execution
 *                                       time exceeded it's specified timeout.
 *         E_WDT_ERR_WATCHDOG_NOT_INIT   indicates that the watchdog framework
 *                                       has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogCheckTaskStatus(const WatchdogTaskID ID);

/**
 * @brief Map an RTOS task ID to a watchdog task ID
 *
 * Maps an RTOS task ID to a watchdog task ID which is used by all other
 * watchdog APIs.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @param[in]     taskID  handle to an RTOS task
 * @param[in,out] pID     pointer to a variable that will be filled in which
 *                        represents the watchdog task ID for the indicated
 *                        RTOS task
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                    indicates that the variable that is
 *                                      pointed to by pID will be set to the
 *                                      watchdog task ID for the RTOS task
 * @retval E_WDT_ERR_NO_TASK            indicates that taskID does not map to a
 *                                      known watchdog task ID
 * @retval E_WDT_ERR_NULL_PTR           indicates that pID is NULL
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT  indicates that the watchdog framework
 *                                      has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogGetTaskID(const rtosTaskHandle taskID,
                          WatchdogTaskID *pID);

/**
 * @brief Return maximum task ID in use
 *
 * Returns the maximum monitored task ID that is in use.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @param[in,out]  pNumTaskIDs  pointer to a variable that will be set to the
 *                              maximum number of monitored task IDs
 *
 * @return Error Code
 *
 * @retval E_SUCCESS variable set with maximum number of monitored task IDs
 * @retval E_WDT_ERR_NULL_PTR pNumTaskIDs is NULL
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT indicates that the watchdog framework
 *                                     has not been initialized
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogGetNumTaskIDs(uint32_t *pNumTaskIDs);

/**
 * @brief Callback when a task is "out of bounds"
 *
 * Called by the watchdog framework when a task fails to make sufficient
 * progress.  This will only be called during the operation of the functions
 * WatchdogCheckStatus() and WatchdogCompleteItem().
 *
 * This function is provided by the application (usually the task that
 * is petting the watchdog hardware and calling WatchdogCheckStatus()).
 *
 * A "weak" version of this function is provided by the watchdog framework for
 * the case when the application does not wish to provide a callback.  In that
 * situation, the callback will cause a fatal abort.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @parm[in] ID  the ID of the task that failed to make progress.
 *
 * @return Error Code
 *
 * @retval E_SUCCESS       indicates that the framework should continue reporting
 *                         other tasks that are "out of bounds"
 * @retval E_WDT_ERR_DONE  indicates that the framework should not report any
 *                         additional tasks that are "out of bounds" during the
 *                         current invokation of WatchdogCheckStatus().
 *
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogCallbackTaskOutOfBounds(WatchdogTaskID ID);

/**
 * @brief Callback to perform other checks
 *
 * Called when the watchdog task has determined that all tasks are operating
 * properly.  It allows an application to perform it's own internal checks and
 * if those checks fail, prevent the watchdog task from "petting" the watchdog
 * hardware by returning E_WDT_ERR_NO_PET.
 *
 * This function is provided by the application.  This function will execute
 * within the context of the watchdog task.
 *
 * A "weak" version of this function is provided by the watchdog task for the
 * case when the application does not wish to provide a callback.  In that
 * situation, the callback will return E_SUCCESS.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @return Error Code
 *
 * @retval E_SUCCESS         indicates that the watchdog hardware should be "pet"
 * @retval E_WDT_ERR_NO_PET  indicates that the watchdog hardware should not
 *                           be "pet"
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogCallbackApplicationCheck(void);

/**
 * @brief Return Watchdog Statistics for a Task
 *
 * This function will return the statistics structure that represents the
 * statistics that have been collected by the watchdog framework about the
 * task.
 *
 * If the watchdog framework has not been configured to collect statistics
 * then this function will return an error.
 *
 * @pre WatchdogInit() must have been previously called.
 * @pre WatchdogTaskMonitor() must have been called to register the task to
 *      be monitored.
 *
 * @param[in] ID        the ID of the task whose statistics are to be returned
 * @param[in] stats     a pointer to a WatchdogTaskStatistics structure to be
 *                      filled in.
 *
 * @return Error Code
 *
 * @retval E_SUCCESS    indicates that the statistics were returned
 * @retval E_WDT_ERR_NO_TASK            indicates that taskID does not map to a
 *                                      known watchdog task ID
 * @retval E_WDT_ERR_NULL_PTR           indicates that pID is NULL
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT  indicates that the watchdog framework
 *                                      has not been initialized
 * @retval E_WDT_ERR_NO_STATS           statistics has not been configured
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
error_t WatchdogGetTaskStatistics(const WatchdogTaskID ID,
                                  WatchdogTaskStatistics *stats);

END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
#endif
