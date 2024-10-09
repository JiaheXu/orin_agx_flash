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
#include <stdint.h>                 // for uint32_t
#include <stdbool.h>                // for bool

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/attributes.h>        // for FSP__MISC__ATTRIBUTES_H, NO_RETURN
#include <misc/macros.h>            // START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <debug/abort.h>            // for tegra_abort, FSP__DEBUG__ABORT_H	// IWYU pragma: keep
#include <debug/assert.h>           // for FSP__DEBUG__ASSERT_H, assert_hook
#include <debug/sections-assert.h>  // Immune from CT_ASSERT protection


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
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")

/**
 * assert_hook()        - default assert hook function
 *
 * @filename:           pointer to a string that contains the filename
 *                      where the assert occurred.
 * @linenum:            integer representing the line number in @filename
 *                      where the assert occurred
 *
 * This function is a weakly bound function that serves as the default
 * implementation of the assert hook if an application doesn't define
 * its own version.
 *
 * Return Value:
 *      none
 */
SECTION_ASSERT_ERR_TEXT WEAK void
assert_hook(const char * const filename,
            const uint32_t linenum)
{
}

/*
 * Split out for easier debugging.  It should not be  called other than
 * from assert().  Having it not be static means that it won't get
 * inlined.
 */
void asserted(const char * const filename,
              const uint32_t linenum)   NO_RETURN;

/**
 * asserted()           - log the fact that an assert fired
 *
 * @filename:           pointer to a string that represents the name of
 *                      the file where the assert failed.
 * @linenum:            integer that represents the line number within
 *                      @filename where the assert failed.
 *
 * This function is called by assert() when the assertion check
 * fails.
 *
 * It is split out as a separate function for ease of setting a breakpoint
 * in the debugger in order to more easily catch assertion failures.
 *
 * Return Values:
 *      This function never returns
 */
SECTION_ASSERT_ERR_TEXT void
asserted(const char * const filename,
         const uint32_t linenum)
{
    assert_hook(filename, linenum);
    tegra_abort(ABORT_ASSERT, 0U);
}

/**
 * assert()             - check to see if an assertion has failed
 *
 * @assertion:          A boolean that is the result of an assertion check.
 *                      If true, the assertion passed and this function will
 *                      just return.  If false, the assertion failed and this
 *                      function will log the assertion failure and abort.
 * @filename:           A pointer to a string that represents the name of
 *                      the file with the assertion check.
 * @linenum:            The binary value of the line number within @filename
 *                      of the assertion check.
 *
 * This function implements an assertion check that can be placed in various
 * places in the code to check for impossible or fatal conditions.  It is
 * intended that code desiring assertion checks should not call this function
 * directly but instead use a macro that wrappers the call and the generation
 * of the appropriate pointers (see ASSERT in debug/assert.h
 * how this is done).
 *
 * Return Values:
 *      none
 */
SECTION_ASSERT_TEXT void
fsp_assert(const bool assertion,
       const char * const filename,
       const uint32_t linenum)
{
    if (!assertion) {
        asserted(filename, linenum);
    }
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
