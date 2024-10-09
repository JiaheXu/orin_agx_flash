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
#include <stdint.h>                            // for uint64_t, UINT64_MAX
#include <string.h>                            // for NULL, memcpy

/* Early FSP headers */
#include <misc/ct-assert.h>                    // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-task.h>                     // for rtosTaskEnterCritical
#include <debug/assert.h>                      // for ASSERT
#include <error/common-errors.h>               // for E_SUCCESS, error_t
#include <misc/attributes.h>                   // for UNUSED
#include <misc/macros.h>                       // for START_RFD_BLOCK, END_RFD_BLOCK, ...

/* Module-specific FSP headers */
#include <watchdog/sections-watchdog.h>        // Immune from CT_ASSERT protection
#include <watchdog/watchdog-errors.h>          // for E_WDT_ERR_NULL_PTR
#include <watchdog/watchdog-statistics.h>      // for WatchdogStatistics
#include <watchdog/watchdog-internal.h>        // for WatchdogTaskState, Wat...
#include <watchdog/watchdog.h>                 // for WatchdogGetTaskStatistics
#include <watchdog/watchdog-stats-internal.h>  // for WatchdogStatsAddItem
#include <watchdog/watchdog-types.h>           // for WatchdogTaskID

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
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_STATISTICS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_INTERNAL_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_STATS_INTERNAL_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_TYPES_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file watchdog-statistics.c
 * @brief Implementation of Watchdog Statistics
 */

/*
 * There is no point in attempting to compile this file if
 * watchdog statistics are not enabled.  This checks for that
 * and causes an error to be reported.
 */
#if !defined(WATCHDOG_STATISTICS) || (WATCHDOG_STATISTICS != 1)
#error "WATCHDOG_STATISTICS != 1"
#endif

SECTION_WATCHDOG_INIT_TEXT void
WatchdogStatsInit(WatchdogTaskState *state)
{
    ASSERT(state != NULL);

    state->taskStats.maxItemsInQueue = 0;
    state->taskStats.itemStats.count = 0;
    state->taskStats.itemStats.accumulatedTime = 0;
    state->taskStats.itemStats.minTime = UINT64_MAX;
    state->taskStats.itemStats.maxTime = 0;
}

SECTION_WATCHDOG_TEXT void
WatchdogStatsAddItem(WatchdogTaskState *state)
{
    WatchdogTaskStatistics      *stats;

    ASSERT(state != NULL);

    stats = &state->taskStats;

    /*
     * Track the maximum number of items that have
     * appeared in the queue.
     */
    if (state->itemsInQueue > stats->maxItemsInQueue) {
        stats->maxItemsInQueue = state->itemsInQueue;
    }
}

SECTION_WATCHDOG_TEXT void
WatchdogStatsStartItem(const WatchdogTaskState *state)
{
    UNUSED(state);
}

SECTION_WATCHDOG_TEXT void
WatchdogStatsCompleteItem(WatchdogTaskState *state,
                          uint64_t completeTime)
{
    WatchdogStatistics          *stats;
    uint64_t                    deltaT;
    uint64_t                    maxTimeout;
    uint64_t                    histIndex;

    ASSERT(state != NULL);
    ASSERT(state->taskTimeout > 0ULL);

    stats = &state->taskStats.itemStats;

    deltaT = completeTime - state->lastStartTime;
    stats->accumulatedTime += deltaT;

    if (stats->minTime > deltaT) {
        stats->minTime = deltaT;
    }

    if (stats->maxTime < deltaT) {
        stats->maxTime = deltaT;
    }

    /*
     * Compute the index into the histogram that represents the
     * percentage (decimals...0-9, 10-19, 20-29, etc).
     */
    maxTimeout = state->taskTimeout;
    histIndex = (WATCHDOG_STATS_NUM_BUCKETS * deltaT) / maxTimeout;

    stats->histogram[histIndex] += 1U;

    stats->count += 1U;
}

START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx")
SECTION_WATCHDOG_TEXT error_t
WatchdogGetTaskStatistics(const WatchdogTaskID ID,
                          WatchdogTaskStatistics *stats)
{
    error_t                 rc              = E_SUCCESS;
    const WatchdogTaskState *statep         = NULL;

    rc = WatchdogValidateID(ID);
    if (rc != E_SUCCESS) {
        goto out;
    }

    if (stats == NULL) {
        rc = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    statep = &watchdogState[ID];

    rtosTaskEnterCritical();

    (void)memcpy(stats, &statep->taskStats, sizeof(WatchdogTaskStatistics));

    rtosTaskExitCritical();

  out:
    return rc;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
