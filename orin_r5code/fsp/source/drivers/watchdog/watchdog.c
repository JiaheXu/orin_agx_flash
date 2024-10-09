_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_1_2 \"Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx\") \
         (deviate MISRA_C_2012_Directive_4_8 \"Approval: Bug 200534384, DR: SWE-FSP-011-SWSADR.docx\")")
/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */
#include <stdbool.h>                                // for bool, false, true
#include <stddef.h>                                 // for NULL
#include <stdint.h>                                 // for uint64_t, uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>                         // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-task.h>                          // for rtosTaskEnterCritical
#include <debug/abort-codes.h>                      // for ABORT_CODE, FSP__...
#include <debug/abort.h>                            // for tegra_abort, FSP_...
#include <error/common-errors.h>                    // for E_SUCCESS, error_t
#include <misc/attributes.h>                        // for WEAK
#include <misc/bitops.h>                            // for LOW32, HI32
#include <misc/macros.h>                            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <watchdog-config.h>                        // Immune from CT_ASSERT protection
#include <watchdog/sections-watchdog.h>             // Immune from CT_ASSERT protection
#include <watchdog/watchdog-errors.h>               // for E_WDT_ERR_TASK_OU...
#include <watchdog/watchdog-internal.h>             // for WatchdogTaskState
#include <watchdog/watchdog.h>                      // for FSP__WATCHDOG__WA...
#include <watchdog/watchdog-statistics.h>           // for WatchdogTaskStati...
#include <watchdog/watchdog-statistics-internal.h>  // for WatchdogStatsAddItem
#include <watchdog/watchdog-types.h>                // for WatchdogTaskID

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_CODES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_INTERNAL_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_STATISTICS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_STATISTICS_INTERNAL_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_TYPES_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file watchdog.c
 * @brief Implementation of Watchdog framework
 */

/**
 * @brief State of all monitored tasks
 *
 * This array is used by the watchdog framework to track the state of all
 * of the tasks that are being monitored.
 */
WatchdogTaskState        watchdogState[WATCHDOG_MAX_TASKS]
                                                        SECTION_WATCHDOG_DATA;

/**
 * @brief Number of monitored tasks
 *
 * Number of tasks that have been registered with the watchdog framework
 * to be monitored.
 */
static uint32_t                 num_tasks       SECTION_WATCHDOG_DATA = 0;

/**
 * @brief Watchdog framework initialized
 *
 * A boolean that indicates if the watchdog framework has been initialized
 * or not.
 */
static bool                     init            SECTION_WATCHDOG_DATA = false;

/**
 * @brief Minimum task timeout
 *
 * A value set during initialization of the watchdog framework that indicates
 * the minimum value that can be specified for a timeout when a task is
 * registered to be monitored.
 */
static uint64_t                 wdt_min_timeout SECTION_WATCHDOG_INIT_DATA = 0;

/**
 * @brief Maximum task timeout
 *
 * A value set during initialization of the watchdog framework that indicates
 * the maximum value that can be specified for a timeout when a task is
 * registered to be monitored.
 */
static uint64_t                 wdt_max_timeout SECTION_WATCHDOG_INIT_DATA = 0;

/**
 * @brief Default task timeout
 *
 * A value set during initialization of the watchdog framework that indicates
 * the default value that will be used for a timeout when a task is registered
 * to be monitored if the task does not specify a timeout value.
 */
static uint64_t                 wdt_default_timeout SECTION_WATCHDOG_INIT_DATA = 0;

START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx")
SECTION_WATCHDOG_TEXT error_t
WatchdogValidateID(const WatchdogTaskID ID)
{
    error_t                 rc              = E_SUCCESS;
    const WatchdogTaskState *statep;

    if (!init) {
        rc = E_WDT_ERR_WATCHDOG_NOT_INIT;
        goto out;
    }

    if (ID >= num_tasks) {
        rc = E_WDT_ERR_NO_TASK;
        goto out;
    }

    statep = &watchdogState[ID];

    if (statep->taskStatus == WATCHDOG_NOT_MONITORED) {
        rc = E_WDT_ERR_NO_TASK;
        goto out;
    }

  out:
    return rc;

}

SECTION_WATCHDOG_INIT_TEXT error_t
WatchdogInit(uint64_t minTimeout,
             uint64_t maxTimeout,
             uint64_t defaultTimeout)
{
    error_t     rc      = E_SUCCESS;

    /*
     * Perform various bounds checks on the timeout values:
     * 1) min <= max
     * 2) max < 2^32
     * 3) min <= default <= max
     */
    if (minTimeout > maxTimeout) {
        rc = E_WDT_ERR_INVALID_TIMEOUT;
        goto out;
    }

    if ((defaultTimeout < minTimeout)
        || (defaultTimeout > maxTimeout)) {
        rc = E_WDT_ERR_INVALID_TIMEOUT;
        goto out;
    }

    /*
     * Set the variables to be used when checking a task's max time
     */
    wdt_min_timeout = minTimeout;
    wdt_max_timeout = maxTimeout;
    wdt_default_timeout = defaultTimeout;

    init = true;

  out:
    return rc;
}

SECTION_WATCHDOG_INIT_TEXT error_t
WatchdogTaskMonitor(const rtosTaskHandle taskID,
                    uint64_t maxTime,
                    WatchdogTaskID *pID)
{
    error_t             rc              = E_SUCCESS;
    uint64_t            timeout         = maxTime;
    WatchdogTaskState   *statep;

    if (!init) {
        rc = E_WDT_ERR_WATCHDOG_NOT_INIT;
        goto out;
    }

    /*
     * Check that maxTime is within bounds
     */
    if (maxTime > wdt_max_timeout) {
        rc = E_WDT_ERR_INVALID_TIMEOUT;
        goto out;
    }

    /*
     * If maxTime is too small, use a default value
     */
    if (maxTime < wdt_min_timeout) {
        timeout = wdt_default_timeout;
    }

    /*
     * Make sure that we have space for adding another
     * task to be monitored.
     */
    if (num_tasks >= WATCHDOG_MAX_TASKS) {
        rc = E_WDT_ERR_TOO_MANY_TASKS;
        goto out;
    }

    statep = &watchdogState[num_tasks];

    /*
     * Make sure that the next slot isn't actually
     * used.
     */
    if (statep->taskStatus != WATCHDOG_NOT_MONITORED) {
        rc = E_WDT_ERR_DUPLICATE_TASK;
        goto out;
    }

    /*
     * Set up the watchdog monitor entry.
     */
    statep->taskStatus  = WATCHDOG_TASK_WAITING;
    statep->taskID      = taskID;
    statep->taskTimeout = timeout;

    WatchdogStatsInit(statep);

    *pID = num_tasks;
    num_tasks += 1UL;

  out:
    return rc;
}

SECTION_WATCHDOG_TEXT error_t
WatchdogAddItem(const WatchdogTaskID ID)
{
    error_t             rc              = E_SUCCESS;
    WatchdogTaskState   *statep;

    rc = WatchdogValidateID(ID);
    if (rc != E_SUCCESS) {
        goto out;
    }

    statep = &watchdogState[ID];

    rtosTaskEnterCritical();

    statep->itemsInQueue += 1U;

    WatchdogStatsAddItem(statep);

    rtosTaskExitCritical();

  out:
    return rc;
}

SECTION_WATCHDOG_TEXT error_t
WatchdogAddItemFromISR(const WatchdogTaskID ID)
{
    error_t             rc              = E_SUCCESS;
    WatchdogTaskState   *statep;

    rc = WatchdogValidateID(ID);
    if (rc != E_SUCCESS) {
        goto out;
    }

    statep = &watchdogState[ID];

    statep->itemsInQueue += 1U;

  out:
    return rc;
}

SECTION_WATCHDOG_TEXT error_t
WatchdogTaskStartItem(const WatchdogTaskID ID)
{
    error_t             rc              = E_SUCCESS;
    WatchdogTaskState   *statep;

    rc = WatchdogValidateID(ID);
    if (rc != E_SUCCESS) {
        goto out;
    }

    statep = &watchdogState[ID];

    rtosTaskEnterCritical();

    /*
     * Make sure that there's actually work to do
     */
    if (statep->itemsInQueue == 0UL) {
        rc = E_WDT_ERR_NO_WORK;
        goto out_critical;
    }

    /*
     * Make sure that the task has completed previous
     * work prior to starting next piece of work
     */
    if (statep->taskStatus == WATCHDOG_ITEM_STARTED) {
        rc = E_WDT_ERR_SEQUENCE;
        goto out_critical;
    }

    /*
     * Make sure that the task hasn't previously
     * timed out
     */
    if (statep->taskStatus == WATCHDOG_ITEM_TIMEOUT) {
        rc = E_WDT_ERR_TASK_OUT_OF_BOUNDS;
        goto out_critical;
    }

    /*
     * Capture the time and indicate that the task
     * has started work
     */
    statep->lastStartTime = WatchdogGetTime();
    statep->taskStatus = WATCHDOG_ITEM_STARTED;

    WatchdogStatsStartItem(statep);

  out_critical:
    rtosTaskExitCritical();

  out:
    return rc;
}

SECTION_WATCHDOG_TEXT error_t
WatchdogStartItem(void)
{
    error_t             rc              = E_SUCCESS;
    rtosTaskHandle      task;
    WatchdogTaskID      id;

    if (!init) {
        rc = E_WDT_ERR_WATCHDOG_NOT_INIT;
        goto out;
    }

    /*
     * Get the watchdog ID of the current task
     */
    task = rtosTaskGetCurrentTaskHandle();
    rc = WatchdogGetTaskID(task, &id);
    if (rc != E_SUCCESS) {
        goto out;
    }

    rc = WatchdogTaskStartItem(id);

  out:
    return rc;
}

SECTION_WATCHDOG_TEXT error_t
WatchdogTaskCompleteItem(WatchdogTaskID ID)
{
    error_t             rc              = E_SUCCESS;
    WatchdogTaskState   *statep;
    uint64_t            now;

    rc = WatchdogValidateID(ID);
    if (rc != E_SUCCESS) {
        goto out;
    }

    rtosTaskEnterCritical();

    statep = &watchdogState[ID];

    /*
     * Make sure that there's actually work to do
     */
    if (statep->itemsInQueue == 0UL) {
        rc = E_WDT_ERR_NO_WORK;
        goto out_critical;
    }

    /*
     * Make sure that the task is actually processing
     * work.
     */
    if (statep->taskStatus != WATCHDOG_ITEM_STARTED) {
        rc = E_WDT_ERR_SEQUENCE;
        goto out_critical;
    }

    statep->itemsInQueue -= 1U;

    /*
     * Make sure that there wasn't a timeout
     */
    now = WatchdogGetTime();
    if ((now - statep->lastStartTime) <= statep->taskTimeout) {
        statep->taskStatus = WATCHDOG_ITEM_COMPLETED;
    } else {
        statep->taskStatus = WATCHDOG_ITEM_TIMEOUT;

        (void)WatchdogCallbackTaskOutOfBounds(ID);

        rc = E_WDT_ERR_TASK_OUT_OF_BOUNDS;
    }

    WatchdogStatsCompleteItem(statep, now);

  out_critical:
    rtosTaskExitCritical();

  out:
    return rc;
}

SECTION_WATCHDOG_TEXT error_t
WatchdogCompleteItem(void)
{
    error_t             rc              = E_SUCCESS;
    rtosTaskHandle      task;
    WatchdogTaskID      id;

    if (!init) {
        rc = E_WDT_ERR_WATCHDOG_NOT_INIT;
        goto out;
    }

    /*
     * Get the watchdog ID of the current task
     */
    task = rtosTaskGetCurrentTaskHandle();
    rc = WatchdogGetTaskID(task, &id);
    if (rc != E_SUCCESS) {
        goto out;
    }

    rc = WatchdogTaskCompleteItem(id);

  out:
    return rc;
}

START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_4, "Approval: Bug 200532006, DR: SWE-FSP-029-SWSADR.docx")
SECTION_WATCHDOG_TEXT error_t
WatchdogCheckStatus(void)
{
    error_t                 rc              = E_SUCCESS;
    error_t                 err             = E_SUCCESS;
    const WatchdogTaskState *statep;
    WatchdogTaskID          id;

    if (!init) {
        rc = E_WDT_ERR_WATCHDOG_NOT_INIT;
        goto out;
    }

    for (id = 0; id < num_tasks; id += 1UL) {
        statep = &watchdogState[id];

        /*
         * Skip over non-active tasks
         */
        if ((statep->taskStatus == WATCHDOG_NOT_MONITORED)
            || (statep->taskStatus == WATCHDOG_TASK_WAITING)) {
            continue;
        }

        rc = WatchdogCheckTaskStatus(id);
        if (rc == E_WDT_ERR_TASK_OUT_OF_BOUNDS) {
            rc = WatchdogCallbackTaskOutOfBounds(id);
            if (rc == E_WDT_ERR_DONE) {
                rc = E_WDT_ERR_TASK_OUT_OF_BOUNDS;
                break;
            }

            err = E_WDT_ERR_TASK_OUT_OF_BOUNDS;
        }

        if (rc != E_SUCCESS) {
            break;
        }
    }

  out:
    return (err == E_SUCCESS) ? rc : err;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_15_4, "Approval: Bug 200532006, DR: SWE-FSP-029-SWSADR.docx")

SECTION_WATCHDOG_TEXT error_t
WatchdogCheckTaskStatus(const WatchdogTaskID ID)
{
    error_t                 rc              = E_SUCCESS;
    const WatchdogTaskState *statep;
    uint64_t                now;

    rc = WatchdogValidateID(ID);
    if (rc != E_SUCCESS) {
        goto out;
    }

    statep = &watchdogState[ID];

    rtosTaskEnterCritical();

    if (statep->taskStatus == WATCHDOG_ITEM_TIMEOUT) {
        rc = E_WDT_ERR_TASK_OUT_OF_BOUNDS;
        goto out_critical;
    }

    if (statep->taskStatus == WATCHDOG_ITEM_STARTED) {
        now = WatchdogGetTime();
        if ((now - statep->lastStartTime) > statep->taskTimeout) {
            rc = E_WDT_ERR_TASK_OUT_OF_BOUNDS;
        }
    }

  out_critical:
    rtosTaskExitCritical();

  out:
    return rc;
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WATCHDOG_TEXT error_t WatchdogGetTaskID(const rtosTaskHandle taskID, WatchdogTaskID *pID)
{
    error_t                 rc              = E_WDT_ERR_NO_TASK;
    const WatchdogTaskState *statep;
    WatchdogTaskID          id;

    if (!init) {
        rc = E_WDT_ERR_WATCHDOG_NOT_INIT;
        goto out;
    }

    if (pID == NULL) {
        rc = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    for (id = 0; id < num_tasks; id += 1UL) {
        statep = &watchdogState[id];

        if (statep->taskID == taskID) {
            *pID = id;
            rc = E_SUCCESS;
            break;
        }
    }

  out:
    return rc;
}

SECTION_WATCHDOG_TEXT error_t
WatchdogGetNumTaskIDs(uint32_t *pNumTaskIDs)
{
    error_t             rc              = E_SUCCESS;

    if (!init) {
        rc = E_WDT_ERR_WATCHDOG_NOT_INIT;
        goto out;
    }

    if (pNumTaskIDs == NULL) {
        rc = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    *pNumTaskIDs = num_tasks;

  out:
    return rc;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogCallbackTaskOutOfBounds(WatchdogTaskID ID)
{
    tegra_abort(ABORT_CODE(WDT, 1U), ID);

    /*
     * Not reached but needed to keep the compiler happy
     */
    return E_WDT_ERR_DONE;
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WATCHDOG_TEXT WEAK error_t WatchdogGetTaskStatistics(const WatchdogTaskID ID, WatchdogTaskStatistics *stats)
{
    UNUSED(ID);
    UNUSED(stats);
    return E_WDT_ERR_NO_STATS;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_8, "Approval: Bug 200534384, DR: SWE-FSP-011-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
