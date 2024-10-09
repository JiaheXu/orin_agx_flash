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

#ifndef WATCHDOG__WATCHDOG_STATS_INTERNAL_H
#define WATCHDOG__WATCHDOG_STATS_INTERNAL_H
#define FSP__WATCHDOG__WATCHDOG_STATS_INTERNAL_H        1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                       // for START_RFD_BLOCK, END_RFD_BLOCK, ...

/* Module-specific FSP headers */
#include <watchdog/watchdog-internal.h>

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_INTERNAL_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @file watchdog-stats-internal.h
 * @brief Definitions for when Watchdog statistics are enabled
 *
 * This file provides the definitions for when watchdog statistics are
 * enabled so that statistics are gathered and can be retrieved.
 *
 * This file should not be included directly.  It should only be
 * included by <watchdog/watchdog-statistics-internal.h>.
 */

/**
 * @brief Initialize Watchdog Statistics for a monitored task
 *
 * This function will initialize the statistics structure for
 * a watchdog monitored task.
 *
 * @param[in] state     pointer to a task's watchdog state structure.
 *
 * @return None
 */
void
WatchdogStatsInit(WatchdogTaskState *state);

/**
 * @brief Capture statistics when an item is added to a task
 *
 * This function will update a task's statistics as a result of adding
 * an item to the task's work queue.
 *
 * @param[in] state     pointer to a task's watchdog state structure.
 *
 * @return None
 */
void
WatchdogStatsAddItem(WatchdogTaskState *state);

/**
 * @brief Capature statistics when a task starts work on an item
 *
 * This function will update a task's statistics as a result of starting
 * to process an item on the task's work queue.
 *
 * @param[in] state     pointer to a task's watchdog state structure.
 *
 * @return None
 */
void
WatchdogStatsStartItem(const WatchdogTaskState *state);

/**
 * @brief Capture statistics when a task completes work on an item
 *
 * This function will update a task's statistics as a result of completing
 * processing of an item on the task's work queue.
 *
 * @param[in] state             pointer to a task's watchdog state structure.
 * @param[in] completeTime      time when the item was completed.
 *
 * @return None
 */
void
WatchdogStatsCompleteItem(WatchdogTaskState *state,
                          uint64_t completeTime);


#endif
