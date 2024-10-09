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
#include <stdint.h>                // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>        // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <cpu/barriers.h>          // for barrier_compiler, FSP__ARM__COMMON...
#include <misc/macros.h>           // START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <debug/sections-debug.h>  // Immune from CT_ASSERT protection
#include <debug/tegra-debug.h>     // for DEBUG_WAIT_BOOT_FL, FSP__DEBUG__TE...

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
CT_ASSERT(FSP__CPU__BARRIERS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__TEGRA_DEBUG_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/*
 * This variable must be marked "volatile" since the intent is that the
 * debugger will modify this value, so it is necessary for the generated
 * code to always read this variable when it is polling it.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx", MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
volatile uint32_t       debug_wait_flags        SECTION_DEBUG_DATA = 0;

/*
 * This variable is used to re-initialize the tegra_debug_wait_flags variable
 * each time that tegra_debugger_wait() is called.  This allows different
 * calls to tegra_debugger_wait() to still wait even if a debugger reset the
 * contents of tegra_debug_wait_flags on a previous call.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx", MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
uint32_t                debug_wait_init         SECTION_DEBUG_DATA
                                        = DEBUG_WAIT_BOOT_FL;

/**
 * debugger_wait_bp()     - internal debugger wait
 *
 * @wait_flag:                  Type of wait
 *
 * This function will wait until a specific bit (or bits) in the variable
 * tegra_debug_wait_flags has been cleared.
 *
 * This function is called when the wait is to be performed.  This allows
 * for a debugger to set a breakpoint on this function when a wait is
 * expected.
 *
 * Return Values:
 *      none
 */
void
debugger_wait_bp(const uint32_t wait_flag)
{
    /*
     * Wait until the flags are cleared
     */
    while ((debug_wait_flags & wait_flag) != 0) {
        barrier_compiler();
    }
}

/**
 * debugger_wait()              - waits for the debugger to clear a flag
 *
 * @wait_flag:                  flag that the debugger has to clear in order
 *                              for the function to exit.
 *
 * This function will wait on a specific bit (or bits) in the variable
 * tegra_debugger_wait_flags have been cleared.  This variable is set to
 * the initial values on entry so that this can be called multiple times
 * and will wait appropriately each time this function is called.
 *
 * Return Values:
 *      none
 */
void
debugger_wait(const uint32_t wait_flag)
{
    /*
     * Set the wait variable.  When the debugger breaks in, it will
     * clear the variable.
     */
    debug_wait_flags = debug_wait_init;

    if ((debug_wait_flags & wait_flag) != 0) {
        debugger_wait_bp(wait_flag);
    }
}
