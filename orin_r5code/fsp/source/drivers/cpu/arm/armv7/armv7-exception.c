/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdint.h>                        // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>                // for CT_ASSERT
#include <misc/attributes.h>               // for UNUSED, WEAK

/* Hardware headers */

/* Late FSP headers */
#include <debug/abort-sys-codes.h>         // for ABORT_EXCEPTION, FSP__DEBU...
#include <debug/abort.h>                   // for tegra_abort, FSP__DEBUG__A...
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <cpu/armv7-exception.h>           // for armv7_exception_t, FSP__AR...
#include <cpu/armv7-regs.h>                // for rd_adfsr, rd_aifsr, rd_dfar
#include <cpu/sections-exception.h>        // Immune from CT_ASSERT protection

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
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_SYS_CODES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_EXCEPTION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_REGS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
/**
 * exception_hook()     - default exception hook
 *
 * @exception:          pointer to an exception structure
 *
 * This function is the default exception hook.  It is defined as a weak
 * function so that if the application doesn't provide an exception hook
 * this will be called instead.
 *
 * Return Values:
 *      none
 */
SECTION_EXCEPTION_TEXT WEAK void
exception_hook(const armv7_exception_t * const exception)
{
    UNUSED((exception->frame->sp));
    UNUSED((exception));
}

/**
 * armv7_exception()    - called when the CPU experiences an exception
 *
 * @type:               type of the exception
 * @frame:              pointer to the exception frame that contains
 *                      various state about the CPU (registers and such)
 *
 * This function is called when the CPU experiences an exception.  It will
 * take the existing state that is captured by the earlier handler and
 * adds additional information that may be useful in determining the cause
 * of the exception.  It will also call an application specific hook to
 * allow for application specific operations to report the exception.
 *
 * Return Values:
 *      This function does not return
 */
SECTION_EXCEPTION_TEXT void
armv7_exception(uint32_t type, struct armv7_exception_frame *frame)
{
    armv7_exception_t       exception;

    exception.type  = type;
    exception.frame = frame;
    exception.dfar  = rd_dfar();
    exception.dfsr  = rd_dfsr();
    exception.adfsr = rd_adfsr();
    exception.ifar  = rd_ifar();
    exception.ifsr  = rd_ifsr();
    exception.aifsr = rd_aifsr();

    exception_hook(&exception);

    tegra_abort(ABORT_EXCEPTION, type);

    /* Not Reached */
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
