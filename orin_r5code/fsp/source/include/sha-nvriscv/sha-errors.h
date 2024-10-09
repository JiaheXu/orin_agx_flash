/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION. All rights reserved.
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

#ifndef SHA_NVRISCV__SHA_ERRORS_H
#define SHA_NVRISCV__SHA_ERRORS_H
#define FSP__SHA_NVRISCV__SHA_ERRORS_H                          1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <base/module-id.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "base/module-id.h"
#include <misc/macros.h>                // for END_RFD_BLOCK, START_RFD_BLOCK

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
    CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @file sha-errors.h
 * @brief Error codes that are specific to the sha
 */

#define E_SHA_ENG_ERROR             MODULE_ERROR(SHA_NVRISCV, 0UL)
#define E_SHA_MUTEX_ACQUIRE_FAILED  MODULE_ERROR(SHA_NVRISCV, 1UL)
#define E_SHA_MUTEX_RELEASE_FAILED  MODULE_ERROR(SHA_NVRISCV, 2UL)
#define E_SHA_SW_RESET_TIMEOUT      MODULE_ERROR(SHA_NVRISCV, 3UL)
#define E_SHA_WAIT_IDLE_TIMEOUT     MODULE_ERROR(SHA_NVRISCV, 4UL)

#endif  /* SHA_NVRISCV__SHA_ERRORS_H */
