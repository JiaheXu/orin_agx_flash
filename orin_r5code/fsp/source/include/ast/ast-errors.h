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

#ifndef AST__AST_ERRORS_H
#define AST__AST_ERRORS_H
#define FSP__AST__AST_ERRORS_H                          1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <base/module-id.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "base/module-id.h"
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief Macro definitions for errors from AST layer
 *
 * @macro-title AST error codes
 *
 * @E_AST_INVALID_VMINDEX             Error value returned for invalid VM index
 * @E_AST_INVALID_AST_BASE            Error value returned for invalid or NULL
 *                                    AST base addresses
 * @E_AST_GLOBALLY_LOCKED             Error value returned if request could not
 *                                    be processed because AST is globally locked.
 * @E_AST_REGION_DISABLED             Error value returned if request could not
 *                                    be processed because AST region is disabled
 * @E_AST_REGION_STREAMID_DISABLED    Error value returned if request could not
 *                                    be processed because AST region streamID is
 *                                    disabled
 * E_AST_INVALID_INPUT_PARAMETERS     Error value returned for invalid or NULL
 *                                    input parameters
 * E_AST_INVALID_PHY_REGION_REQUEST   Error value returned when the request is
 *                                    not applicable to regions mapped with
 *                                    physical streamID
 */
#define E_AST_INVALID_VMINDEX            MODULE_ERROR(AST,1UL)
#define E_AST_INVALID_AST_BASE           MODULE_ERROR(AST,2UL)
#define E_AST_GLOBALLY_LOCKED            MODULE_ERROR(AST,3UL)
#define E_AST_REGION_DISABLED            MODULE_ERROR(AST,4UL)
#define E_AST_REGION_STREAMID_DISABLED   MODULE_ERROR(AST,5UL)
#define E_AST_INVALID_INPUT_PARAMETERS   MODULE_ERROR(AST,6UL)
#define E_AST_INVALID_PHY_REGION_REQUEST MODULE_ERROR(AST,7UL)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif
