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

#ifndef WATCHDOG__WATCHDOG_INTERNAL_H
#define WATCHDOG__WATCHDOG_INTERNAL_H
#define FSP__WATCHDOG__WATCHDOG_INTERNAL_H      1

/* Compiler headers */
#include <stdint.h>                        // for uint64_t, uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>                // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/osa-task.h>                  // for rtosTaskHandle
#include <error/common-errors.h>           // for error_t
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <tke/tke-tegra.h>                 // for tegra_tke_get_tsc64

/* Module-specific FSP headers */
#include <watchdog-config.h>               // Immune from CT_ASSERT protection
                                           // for WATCHDOG_MAX_TASKS
#include <watchdog/watchdog-statistics.h>  // for WatchdogTaskStatistics
#include <watchdog/watchdog-types.h>       // for WatchdogCount, WatchdogStatus

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
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__OSA_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_STATISTICS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_TYPES_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file watchdog-internal.h
 * @brief Internal definitions for watchdog framework
 */

/**
 * @brief Watchdog Monitored Task State
 *
 * States that a task could be in that is monitored by the watchdog
 * framework.
 */
#define WATCHDOG_NOT_MONITORED  0U      ///! Task is not monitored
#define WATCHDOG_ITEM_STARTED   1U      ///! Task started processing an item
#define WATCHDOG_ITEM_COMPLETED 2U      ///! Task completed processing an item
#define WATCHDOG_ITEM_TIMEOUT   3U      ///! Task did not complete processing
                                        ///!  within its timeout period
#define WATCHDOG_TASK_WAITING   4U      ///! Task is waiting for work
#define WATCHDOG_TASK_WARNING   5U

END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")


/**
 * @brief Watchdog Tracking Structure
 *
 * This structure is used to track the state and operation of the various
 * tasks that have been registered to be monitored by the Watchdog framework.
 */
typedef struct {
    uint64_t            lastStartTime;  ///! Last time WatchdogStartItem() was
                                        ///!  called for the task
    uint64_t            taskTimeout;    ///! Timeout for the task.  Represents
                                        ///!  the maximum amount of time that
                                        ///!  may transpire between a call to
                                        ///!  WatchdogStartItem() and a call to
                                        ///!  WatchdogCompleteItem()
    rtosTaskHandle      taskID;         ///! Task handle to the task being
                                        ///!  monitored
    WatchdogCount       itemsInQueue;   ///! Number of times that
                                        ///!  WatchdogAddItem() has been called
                                        ///!  vs number of times that
                                        ///!  WatchdogStartItem() has been
                                        ///!  called
    WatchdogStatus      taskStatus;     ///! Monitored state of the task.
#if defined(WATCHDOG_STATISTICS) && (WATCHDOG_STATISTICS==1)
    WatchdogTaskStatistics taskStats;   ///! Statistics for the task
#endif
} WatchdogTaskState;

extern WatchdogTaskState        watchdogState[WATCHDOG_MAX_TASKS];

/**
 * @brief Validate a watchdog task ID
 *
 * This function will validate a watchdog task ID to ensure that it
 * is valid.
 *
 * @param [in] ID       ID to validate
 *
 * @return Error Code
 *
 * @retval E_SUCCESS    ID is a valid task ID
 * @retval E_WDT_ERR_NO_TASK            indicates that the task ID was not valid
 * @retval E_WDT_ERR_WATCHDOG_NOT_INIT  indicates that the watchdog framework
 *                                      has not been initialized
 */
error_t
WatchdogValidateID(const WatchdogTaskID ID);

/**
 * @brief Return current time
 *
 * Returns the current time to the caller in nano-seconds.
 * Since this is a 64-bit value the rollover will happen in
 * about 584 years.
 *
 * @pre TSC counter must be running
 *
 * @return 64-bit value that represents the current time in nano-seconds
 */
static inline uint64_t
WatchdogGetTime(void)
{
    return tegra_tke_get_tsc64() << 5;
}

#endif
