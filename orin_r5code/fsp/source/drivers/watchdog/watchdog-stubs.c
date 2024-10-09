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
#include <stddef.h>                        // for NULL
#include <stdint.h>                        // for uint64_t, uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-task.h>                 // for rtosTaskHandle
#include <debug/assert.h>                  // for ASSERT, MODULE_ID_ABORT
#include <error/common-errors.h>           // for error_t, E_SUCCESS
#include <misc/attributes.h>               // for WEAK, UNUSED
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <watchdog/sections-watchdog.h>    /* Immune from CT_ASSERT protection */
#include <watchdog/watchdog.h>             // for WatchdogAddItem, WatchdogA...
#include <watchdog/watchdog-errors.h>      // for E_WDT_ERR_NO_STATS
#include <watchdog/watchdog-statistics.h>  // for WatchdogTaskStatistics
#include <watchdog/watchdog-types.h>       // for WatchdogTaskID

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
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_STATISTICS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_TYPES_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file watchdog-stubs.c
 * @brief Stubs for watchdog APIs if the watchdog framework is not used
 */

SECTION_WATCHDOG_INIT_TEXT WEAK error_t
WatchdogInit(uint64_t minTimeout,
             uint64_t maxTimeout,
             uint64_t defaultTimeout)
{
    UNUSED(minTimeout);
    UNUSED(maxTimeout);
    UNUSED(defaultTimeout);

    return E_SUCCESS;
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WATCHDOG_INIT_TEXT WEAK error_t WatchdogTaskMonitor(const rtosTaskHandle taskID, uint64_t maxTime, WatchdogTaskID *pID)
{
    UNUSED(taskID);
    UNUSED(maxTime);

    ASSERT(pID != NULL);

    *pID = 0;
    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogAddItem(const WatchdogTaskID ID)
{
    UNUSED(ID);

    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogAddItemFromISR(const WatchdogTaskID ID)
{
    UNUSED(ID);

    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogTaskStartItem(const WatchdogTaskID ID)
{
    UNUSED(ID);

    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogStartItem(void)
{
    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogTaskCompleteItem(WatchdogTaskID ID)
{
    UNUSED(ID);

    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogCompleteItem(void)
{
    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogCheckStatus(void)
{
    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogCheckTaskStatus(const WatchdogTaskID ID)
{
    UNUSED(ID);

    return E_SUCCESS;
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WATCHDOG_TEXT WEAK error_t WatchdogGetTaskID(const rtosTaskHandle taskID, WatchdogTaskID *pID)
{
    ASSERT(pID != NULL);

    UNUSED(taskID);

    *pID = 0;
    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogGetNumTaskIDs(uint32_t *pNumTaskIDs)
{
    ASSERT(pNumTaskIDs != NULL);

    *pNumTaskIDs = 0;

    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogCallbackTaskOutOfBounds(WatchdogTaskID ID)
{
    UNUSED(ID);

    return E_SUCCESS;
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogCallbackApplicationCheck(void)
{
    return E_SUCCESS;
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
              MISRA, DEVIATE, Directive_4_8, "Approval: Bug 200534384, DR: SWE-FSP-011-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
