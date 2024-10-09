/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION. All rights reserved.
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
#ifndef MISC__ATTRIBUTES_H
#define MISC__ATTRIBUTES_H
#define FSP__MISC__ATTRIBUTES_H                         1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>         // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#ifndef __ASSEMBLER__

#define NO_RETURN       __attribute__ ((noreturn))
#define NAKED           __attribute__ ((naked))

#ifdef LONG_CALLS
#define LONG_CALL       __attribute__ ((__long_call__))
#else
#define LONG_CALL
#endif

#define NO_INLINE       __attribute__ ((noinline))
#define SECTION(_s_)    __attribute__ ((section(_s_)))
#define ALIAS(_s_)      __attribute__ ((alias(STR(_s_))))
#define USED            __attribute__ ((used))
#define WEAK            __attribute__ ((weak))
#define ALIGN(_v_)      __attribute__ ((aligned(_v_)))

#ifndef USE_C11
#define C99_Alignof(_x_)    ((~sizeof(_x_))+1)
#define FSP_ALIGNOF         C99_Alignof
#else
INLINE_RFD(MISRA, DEVIATE, Rule_1_4, "Approval: JIRA TID-1311, DR: SWE-FSP-061-SWSADR.docx")
#define FSP_ALIGNOF         _Alignof
#endif

#define UNUSED(x)       ((void)x)

/*
 * Use UNUSED_NONCONST_PTR to declare deliberate design intent NOT to use const
 * even though the contents of the data pointed to is not modified.
 *
 * MISRA flags non-const pointers when the data is untouched even though
 * the data may be modified in future code updates or it is semantically changing.
 *
 * Please don't overuse. Prefer const data instead.
 * It usually leads to a cleaner design and more efficient workflow.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
inline static void UNUSED_NONCONST_PTR(void* whitelist_this)
{
        /*
         * This is likely to be flagged with MISRA Directive 8.13 violation.
         * It should be white-listed using static code scanning tools.
         */

        UNUSED(whitelist_this);
}

#endif
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
