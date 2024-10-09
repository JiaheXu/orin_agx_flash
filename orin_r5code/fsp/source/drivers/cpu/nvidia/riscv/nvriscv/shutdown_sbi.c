/**
 * @file shutdown_sbi.c
 *
 * @brief Describes behavior of shutdown API in S-Mode.
 *
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of NVIDIA CORPORATION nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS`` AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Compiler headers */

/* Early FSP headers */
#include <misc/gcc_attrs.h>
#include <misc/gcc_builtins.h>
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>       // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <cpu/shutdown.h>
#include <cpu/sbi.h>

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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__SBI_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__SHUTDOWN_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__GCC_ATTRS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__GCC_BUILTINS_H, "Header file missing or invalid.")

GCC_ATTR_NAKED
GCC_ATTR_NOINLINE
GCC_ATTR_NORETURN
GCC_ATTR_NO_SSP
void riscv_panic(void)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ ("li a7, 8 \n" // SBI_EXTENSION_SHUTDOWN
             "li a6, 0 \n" // SBI_NVFUNC_FIRST
             "ecall \n"
             "1: \n"
             "j 1b \n"
    );
    // If one of those assert is triggered, that means function needs to be updated
    _Static_assert(SBI_EXTENSION_SHUTDOWN == 8, "Shutdown extension id changed.");
    _Static_assert(SBI_NVFUNC_FIRST == 0, "Function id changed.");
    NVRV_BUILTIN_UNREACHABLE();
}

GCC_ATTR_NORETURN
GCC_ATTR_NO_SSP
void riscv_shutdown(void)
{
    (void) sbicall0((int32_t)SBI_EXTENSION_SHUTDOWN, (int32_t)SBI_NVFUNC_FIRST);
    NVRV_BUILTIN_UNREACHABLE();
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/** end of file **/
