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

#ifndef WATCHDOG__WATCHDOG_ERRORS_H
#define WATCHDOG__WATCHDOG_ERRORS_H
#define FSP__WATCHDOG__WATCHDOG_ERRORS_H                1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <base/module-id.h>     // IWYU pragma: export
                                // IWYU pragma: no_include "base/module-id.h"
#include <misc/macros.h>        // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */

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
CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file watchdog-errors.h
 * @brief Error codes that are specific to the watchdog framework
 */

/**
 * @brief No more processing of out of bounds tasks.
 *
 * Indicates that the framework should not report any additional tasks
 * that are "out of bounds" during the current invokation of
 * WatchdogCheckStatus().
 */
#define E_WDT_ERR_DONE                  MODULE_ERROR(WDT, 0UL)

/**
 * @brief Too many tasks being monitored.
 *
 * Indicates that more tasks are trying to be monitored than the
 * watchdog framework has been configured for.
 */
#define E_WDT_ERR_TOO_MANY_TASKS        MODULE_ERROR(WDT, 1UL)

/**
 * @brief Task is already being monitored
 *
 * Indicates that the task to be monitored has already been registered to
 * be monitored by the watchdog framework.
 */
#define E_WDT_ERR_DUPLICATE_TASK        MODULE_ERROR(WDT, 2UL)

/**
 * @brief NULL pointer has been passed
 *
 * Indicates that a pointer passed as an argument is NULL.
 */
#define E_WDT_ERR_NULL_PTR              MODULE_ERROR(WDT, 3UL)

/**
 * @brief Watchdog framework has not been initialized
 *
 * Indicates that one of the watchdog framework APIs has been called
 * prior to WatchdogInit() being called.
 */
#define E_WDT_ERR_WATCHDOG_NOT_INIT     MODULE_ERROR(WDT, 4UL)

/**
 * @brief ID is not a valid task ID
 *
 * Indicates that the task ID supplied to one of the watchdog framework APIs does
 * not represent a task that has been registered with the watchdog
 * framework.
 */
#define E_WDT_ERR_NO_TASK               MODULE_ERROR(WDT, 5UL)

/**
 * @brief Task's execution time is out of bounds
 *
 * Indicates that the task's execution time exceeded its specified timeout.
 */
#define E_WDT_ERR_TASK_OUT_OF_BOUNDS    MODULE_ERROR(WDT, 6UL)

/**
 * @brief Task has no work to perform
 *
 * Indicates that a task tried to start work when no work had been queued for
 * the task.
 */
#define E_WDT_ERR_NO_WORK               MODULE_ERROR(WDT, 7UL)

/**
 * @brief Sequence error in API calls
 *
 * Indicates that there was a sequence error in the order of calls to
 * WatchdogStartItem() and WatchdogCompleteItem().
 */
#define E_WDT_ERR_SEQUENCE              MODULE_ERROR(WDT, 8UL)

/**
 * @brief Watchdog hardware should not be pet.
 *
 * Indication from a callback function that the watchdog hardware should not
 * be "pet" at this time.
 */
#define E_WDT_ERR_NO_PET                MODULE_ERROR(WDT, 9UL)

/**
 * @brief Supplied timeout is invalid.
 */
#define E_WDT_ERR_INVALID_TIMEOUT       MODULE_ERROR(WDT, 10UL)

/**
 * @brief Watchdog callback configuration
 *
 * Watchdog interrupt was enabled but no callback function supplied.
 */
#define E_WDT_ERR_IRQ_CONFIG            MODULE_ERROR(WDT, 11UL)

/**
 * @brief Watchdog Timer Source Invalid
 *
 * Value specified for the watchdog timer source was not a valid
 * value (e.g. it had non-zero bits in fields that should be 0).
 */
#define E_WDT_ERR_TMRSRC_INVALID        MODULE_ERROR(WDT, 12UL)

/**
 * @brief Watchdog Timer Threshold Invalid
 *
 * Value specified for the watchdog timer threshold was not a valid
 * value (e.g. it had non-zero bits in fields that should be 0).
 */
#define E_WDT_ERR_THRESHOLD_INVALID     MODULE_ERROR(WDT, 13UL)

/**
 * @brief Watchdog Statistics Not Available
 *
 * A request was made to get the watchdog statistics for a task and
 * watchdog statistics has not been configured in the build.
 */
#define E_WDT_ERR_NO_STATS              MODULE_ERROR(WDT, 14U)

END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")

#endif
