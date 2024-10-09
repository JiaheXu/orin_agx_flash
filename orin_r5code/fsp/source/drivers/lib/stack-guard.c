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

/**
 * @file stack-guard.c
 * @brief Implementation of stack guard
 *
 * Provides implementations of the data and functions necessary for
 * the compiler to generate stack guard code when any version of the
 * -fstack-protector compiler switches is specified for any of the files.
 */

/* Compiler headers */
#include <stdint.h>                    // for uintptr_t

/* Early FSP headers */
#include <misc/ct-assert.h>            // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <debug/abort-sys-codes.h>     // for ABORT_STACK, FSP__DEBUG__ABORT...
#include <debug/abort.h>               // for tegra_abort, FSP__DEBUG__ABORT_H
#include <misc/attributes.h>           // for NO_RETURN
#include <misc/macros.h>               // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <lib/sections-stack-guard.h>  // Immune from CT_ASSERT protection
#include <lib/stack-guard.h>           // for FSP__LIB__STACK_GUARD_H, __sta...

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
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                CERTC, DEVIATE, DCL37_C, "Approval:  Bug 2731179, DR: SWE-FSP-036-SWSADR.docx")
CT_ASSERT(FSP__DEBUG__ABORT_SYS_CODES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LIB__STACK_GUARD_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief Initial Stack Check Value
 *
 * The value that will be placed on the stack of a function that upon
 * exit will be checked if any version of -fstack-guard is specified
 * on the command line of the compiler.
 *
 * This value is here so that the variable __stack_chk_guard is
 * always initialized to a non-zero value.
 */
#define STACK_CHK_GUARD_VALUE   ((uintptr_t)(0xDEADBEEFUL))

/**
 * @brief Stack Variable
 *
 * Contains the value that will be used to place a guard value on the
 * stack and again to check that value when a function is about to return.
 *
 * It will be initialized to a non-zero value but it is expected that
 * the application code will overwrite the contentes with a randomized
 * value during initialization.
 */
extern uintptr_t __stack_chk_guard;
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_9, "Approval: Bug 200532001, DR: SWE-FSP-016-SWSADR.docx")
INLINE_RFD(MISRA, DEVIATE, Rule_21_2, "Approval: JIRA TID-1063, DR: SWE-FSP-053-SWSADR.docx")
uintptr_t __stack_chk_guard     SECTION_STACKGUARD_DATA  = STACK_CHK_GUARD_VALUE;

/**
 * @brief Initialize Stack Guard Variable
 *
 * This function will initialize the stack guard variable with the
 * supplied value.  This allows the application a means to initialize
 * the stack check variable with a (hopefully) randomized value.
 *
 * @param[in] check_value  Value to be used to check if the stack has
 *                         been overwritten.
 *
 * @return None
 */
SECTION_STACKGUARD_INIT_TEXT void
stack_chk_init(uintptr_t check_value)
{
    __stack_chk_guard = check_value;
}


/**
 * @brief Stack Check Failure
 *
 * Called when the guard value on the stack has been overwritten.
 *
 * @return Does not return
 */
INLINE_RFD(MISRA, DEVIATE, Rule_21_2, "Approval: JIRA TID-1063, DR: SWE-FSP-053-SWSADR.docx")
NO_RETURN SECTION_STACKGUARD_TEXT void __stack_chk_fail(void)
{
    tegra_abort(ABORT_STACK, 0UL);
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_9, "Approval: Bug 200532001, DR: SWE-FSP-016-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              CERTC, DEVIATE, DCL37_C, "Approval:  Bug 2731179, DR: SWE-FSP-036-SWSADR.docx")
