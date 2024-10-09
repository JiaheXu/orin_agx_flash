/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION. All rights reserved.
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
#include <stddef.h>                 // for NULL
#include <stdint.h>                 // for uint32_t
#include <stdbool.h>                // for true

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-task.h>          // for rtosTaskGetCurrentTaskHandle, rto...
#include <osa/rtos-values.h>        // for rtosTRUE
#include <cpu/armv7-regs.h>         // for disable_interrupts, FSP__ARM__ARM...
#include <cpu/barriers.h>           // for barrier_compiler, barrier_memory_...
#include <cpu/cache.h>              // for dcache_clean_all, FSP__ARM__COMMO...
#include <error/common-errors.h>    // for error_t
#include <misc/attributes.h>        // for NO_RETURN, WEAK
#include <misc/macros.h>            // START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <debug/abort-sys-codes.h>  // for ABORT_RTOS, FSP__DEBUG__ABORT_SYS...
#include <debug/abort.h>            // for FSP__DEBUG__ABORT_H, abort_full
#include <debug/sections-abort.h>   // Immune from CT_ASSERT protection

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
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_VALUES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_REGS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__BARRIERS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__CACHE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_SYS_CODES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_H, "Header file missing or invalid.")

/**
 * abort_hook()         - default abort hook function
 *
 * @reason:             abort reason code
 * @xErrorCode:         error or argument to the abort
 * @xHandleOfTaskWithError:     task where the error originated from
 * @pcErrorString:      string to be reported with the error
 *
 * This function is a weakly bound function that serves as the default
 * implemenation of the abort hook in the case when an application doesn't
 * define its own version.
 *
 * Return Value:
 *      none
 */
INLINE_RFD(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx", MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
SECTION_ABORT_TEXT WEAK void
abort_hook(const uint32_t reason,
           error_t xErrorCode,
           rtosTaskHandle xHandleOfTaskWithError,
           const char *pcErrorString)
{
}

/**
 * abort_full()         - abort to allow all arguments to be provided
 *
 * @reason:             abort reason code
 * @xErrorCode:         error or argument to the abort
 * @xHandleOfTaskWithError:     task where the error originated from
 * @pcErrorString:      string to be reported with the error
 *
 * This is the full form of abort that unifies the various other interfaces
 * when the system fails.
 *
 * This function does not return.
 */
SECTION_ABORT_TEXT NO_RETURN void
abort_full(const uint32_t reason,
           error_t xErrorCode,
           rtosTaskHandle xHandleOfTaskWithError,
           const char *pcErrorString)
{
    /*
     * Don't allow anything else to run after this point.
     */
    (void)disable_interrupts();

    /*
     * Call the abort_hook.
     */
    abort_hook(reason, xErrorCode, xHandleOfTaskWithError, pcErrorString);

    /*
     * Wait for all operations to complete and to clean the
     * dCache so that someone can examine DRAM if they so wish.
     */
    barrier_memory_order();
    barrier_memory_complete();
    dcache_clean_all();

    /*
     * Done.
     */
    while (true) {
        /*
         * Make sure that the infinite loop has side effects
         * so that the optimizer doesn't try move or remove it.
         */
        barrier_compiler();
    }

    /*
     * Never reached.
     */
}

/**
 * tegra_abort()        - signal an abort and halt the R5
 *
 * @reason:             abort reason code
 * @arg:                a 32-bit argument
 *
 * This function will "abort" the exectution of the R5.  This is generally
 * called because some fatal condition has been detected and continued
 * execution is ill advised.
 *
 * It will disable the watchdog as well as suspending all tasks and stopping
 * the scheduler.  It will also disable all interrupts.
 *
 * Prior to entering into an infinte loop, it will call an "abort hook" to
 * allow the platform to run some additional code.
 */
SECTION_ABORT_TEXT void
tegra_abort(const uint32_t reason,
            const uint32_t arg)
{
    rtosTaskHandle      cur_task        = NULL;

    /*
     * When getting the current task handle, make sure that the
     * scheduler is running (mainly to make sure that it's been
     * initialized/statrted)
     */
    if (rtosTaskIsSchedulerStarted() == rtosTRUE) {
        cur_task = rtosTaskGetCurrentTaskHandle();
    }

    abort_full(reason, (error_t)arg, cur_task, NULL);

    /*
     * Never reached.
     */
}

/**
 * error_hook           - SafeRTOS will call this when it has a fatal error
 *
 * @xHandleOfTaskWithError:     handle of task that was running when error
 *                              occured
 * @pcErrorString:              pointer to string to be reported
 * @xErrorCode:                 Error code to be reported
 *
 * This function will be called when there SafeRTOS detects a fatal error.
 * The objective of this function is to report the error and then "safe"
 * the system.
 *
 * This function is not to be called directly execept by SafeRTOS.
 *
 * This function does not return.
 */
SECTION_ABORT_TEXT void
error_hook(rtosTaskHandle xHandleOfTaskWithError,
           const char *pcErrorString,
           error_t xErrorCode)
{
    abort_full(ABORT_RTOS, xErrorCode,
               xHandleOfTaskWithError, pcErrorString);

    /*
     * Never reached.
     */
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
